// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Gradient-based joint-relaxation optimizer for JPEG lossless recompression
// (Lane B from plans/in-the-pass-aware-scheme-keen-barto.md).
//
// Iteration 1 status: forward pass for the AC cost component only. NZ cost,
// signalling overhead, soft row->prototype assignment, analytic gradients, and
// the Adam/annealing loop are deferred to later iterations (see
// plans/lane_b_progress.md).
//
// The state `GradientJointState` holds the continuous variables being
// optimized:
//   - `thresholds` — DC thresholds per axis (continuous reals, shared storage
//     with the existing integer `ThresholdSet` after rounding).
//   - `threshold_temperature` — sigmoid temperature for soft cell membership.
//   - `pass_logits` — per-block pre-softmax pass weights.
//   - `pass_temperature` — softmax temperature for pass assignment.
//
// The forward pass `ComputeSoftACCost` computes the AC entropy cost under the
// current soft state, reusing the existing `ctx_map` as a fixed hard clustering.
// The cost formula mirrors the hard version in `EvaluatePassAwareModel` but
// aggregates counts with soft weights.

#ifndef LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_GRAD_H_
#define LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_GRAD_H_

#include <array>
#include <cstdint>
#include <vector>

#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_passes.h"

namespace jxl {

// Continuous-variable state for the joint-relaxation optimizer.
//
// Threshold values live in the same integer DC domain as `ThresholdSet`. They
// are stored as `double` here so the sigmoid relaxation can place them
// off-integer during optimization; rounding to `int16_t` happens when
// collapsing back to a hard `PassSearchResult`.
struct GradientJointState {
  // Per-axis DC thresholds (continuous). Length of each axis matches the
  // factorization minus one (same convention as `ThresholdSet`).
  std::array<std::vector<double>, kNumCh> thresholds;

  // Sigmoid temperature used when computing soft cell membership from
  // thresholds. As `threshold_temperature -> 0+`, cell assignment becomes hard
  // step-function membership matching `AxisMaps::Bkt`.
  double threshold_temperature = 1.0;

  // Pass logits `[channel][block][pass]`. Softmax over the pass axis gives the
  // soft pass-assignment weight `pi_{b,p}`. Sizes: `pass_logits[c]` has length
  // `num_blocks[c] * num_passes` in row-major (block-major) order.
  std::array<std::vector<double>, kNumCh> pass_logits;

  // Softmax temperature for pass assignment. As `pass_temperature -> 0+`,
  // softmax collapses to one-hot argmax.
  double pass_temperature = 1.0;

  uint32_t num_passes = 1;
};

// Initializes `GradientJointState` from a hard `PassSearchResult`. Thresholds
// are cast to `double`. Pass logits are set to `+hard_logit` for the assigned
// pass and `-hard_logit` elsewhere; `hard_logit` should be large enough that
// softmax at `pass_temperature = 1` is numerically indistinguishable from
// one-hot. `threshold_temperature` and `pass_temperature` are set to the
// provided values (callers use a tiny value for hard-limit correctness tests).
GradientJointState InitGradientJointStateFromHard(
    const JPEGOptData& d, const PassSearchResult& hard, double hard_logit,
    double threshold_temperature, double pass_temperature);

struct SoftCostResult {
  // AC entropy cost under the soft aggregation. Units: bits (not fixed-point).
  // Multiply by `kFScale` and round to compare against `FixedPointCost`.
  double ac_cost_bits = 0.0;

  // NZ entropy cost (iteration 4). Zero when computed via
  // `ComputeSoftACCost` / `ComputeSoftACCostWithGrad`.
  double nz_cost_bits = 0.0;

  // Signalling overhead estimate: per-slot ANS-population-minus-Shannon plus a
  // flat per-pass overhead. Treated as constant for gradient purposes. Zero
  // when computed via `ComputeSoftACCost` / `ComputeSoftACCostWithGrad`.
  double signalling_overhead_bits = 0.0;

  // Sum of the three components. For AC-only entries, equals `ac_cost_bits`.
  double total_cost_bits = 0.0;

  // Number of (cluster, pass) slots visited; zero-total slots are skipped in
  // the cost sum to match `EvaluatePassAwareModel`.
  uint32_t num_cp_slots = 0;
};

// Analytic gradient of the soft AC cost with respect to the optimizable
// parameters. Shapes mirror `GradientJointState`: `thresholds[axis]` has the
// same length as `state.thresholds[axis]`, and `pass_logits[c]` has length
// `num_blocks[c] * num_passes`.
struct GradientJointGrad {
  std::array<std::vector<double>, kNumCh> thresholds;
  std::array<std::vector<double>, kNumCh> pass_logits;
};

// Zeros the gradient and sizes it to match `state`. Callers that accumulate
// across blocks should call this once before the first backward pass.
void ResetGradientJointGrad(const GradientJointState& state,
                            GradientJointGrad* grad);

// Computes soft AC cost for the given state.
//
// Arguments:
//   d          — opt data, provides per-block AC events and DC indices.
//   state      — continuous variables; `thresholds` and `pass_logits` are read.
//   ctx_map    — fixed hard clustering, layout `channel * num_cells + cell`.
//   num_clusters — number of active clusters (upper bound on `ctx_map` values).
//   num_passes — number of passes; must equal `state.num_passes`.
//
// The formula mirrors `EvaluatePassAwareModel`:
//   ac_cost = sum over (cluster, pass) cp of
//             [sum_zdc ftab(N_cp_zdc) - sum_hist_bin ftab(h_cp_hist_bin)]
// where the soft count at (cp, zdc) is
//   N_cp_zdc = sum over events in blocks of the block
//                (event count at that zdc)
//              * pi_{b, pass(cp)}
//              * [sum over cells mapped to cluster(cp): gamma_{b, cell}]
// and similarly for h_cp_hist_bin.
//
// `ftab` is evaluated on fractional counts via the continuous extension
// `n * log2(n)`; at integer `n` this matches the precomputed `ftab` table up to
// a half-ULP fixed-point rounding.
SoftCostResult ComputeSoftACCost(const JPEGOptData& d,
                                 const GradientJointState& state,
                                 const ContextMap& ctx_map,
                                 uint32_t num_clusters, uint32_t num_passes);

// Forward pass plus analytic backward pass. Computes the same cost as
// `ComputeSoftACCost` and additionally accumulates the gradient of the AC cost
// wrt `state.pass_logits` and `state.thresholds` into `*grad`. The caller must
// call `ResetGradientJointGrad` first (or otherwise zero and size `*grad`).
//
// Gradient accumulates rather than overwrites, so the same `grad` buffer can
// be reused across mini-batches if future iterations introduce them.
//
// The threshold-gradient formula uses the sigmoid relaxation's analytic
// derivative: for threshold `T[a][j]` with width `tau_t`,
//   dL/dT[a][j] += sigmoid'((T[a][j] - DC_a) / tau_t) / tau_t
//                    * (dL/dw[a][j] - dL/dw[a][j+1])
// summed over every block whose `DC_a` is the block's DC value on axis `a`.
// The softmax Jacobian for the pass logits uses the standard
//   dL/dlogit[q] = pi[q] * (dL/dpi[q] - sum_p pi[p] dL/dpi[p]) / tau_pi.
SoftCostResult ComputeSoftACCostWithGrad(const JPEGOptData& d,
                                         const GradientJointState& state,
                                         const ContextMap& ctx_map,
                                         uint32_t num_clusters,
                                         uint32_t num_passes,
                                         GradientJointGrad* grad);

// --- Iteration 4: NZ cost + signalling overhead + total cost ---------------

// Computes soft total cost: AC entropy + NZ entropy + signalling overhead +
// flat pass overhead. Mirrors `EvaluatePassAwareModel` fully (except for the
// pass-stream-based AC path, which this implementation replaces by walking
// `block_bins`). Returns all three components plus the sum.
//
// NZ forward:
//   For each block at (c, b):
//     predicted_nz is computed from the soft pass probabilities of the top
//     and left neighbors (pass_nz_* = pi_{neighbor, p} * nz_neighbor, summed
//     per the same position-dependent rule as `EvaluatePassAwareModel`), then
//     rounded to integer; pb is derived from the integer predicted_nz exactly
//     as in the hard code.
//     Per cell, per pass p, the block contributes soft weight `gamma * 1` to
//     `nz_hist_N[cp][pb]`, and splits between `bin_real = NZIndex(pb, nz_b)`
//     and `bin_zero = NZIndex(pb, 0)` with weights `gamma * pi_{b,p}` and
//     `gamma * (1 - pi_{b,p})` respectively.
//
// Signalling overhead:
//   Rounded to integer counts per slot and fed to `SignalOverheadFromHist` /
//   `SignalOverheadFromNZHist`-equivalent formulas. Added to the total plus a
//   flat per-pass `ComputePassOverhead(d) * num_passes` constant.
SoftCostResult ComputeSoftTotalCost(const JPEGOptData& d,
                                    const GradientJointState& state,
                                    const ContextMap& ctx_map,
                                    uint32_t num_clusters,
                                    uint32_t num_passes);

// Total cost with analytic gradient. Gradient flows through the AC term
// (iteration 2) and the NZ entropy term (block's own pi for the h split,
// block's cell weight for both h and N). The `predicted_nz` derivation and
// the pb bucket selection are treated as non-differentiable; likewise the
// signalling overhead term. The optimizer still benefits from reducing AC + NZ
// cost; signalling changes come along for the ride through rounding.
SoftCostResult ComputeSoftTotalCostWithGrad(const JPEGOptData& d,
                                            const GradientJointState& state,
                                            const ContextMap& ctx_map,
                                            uint32_t num_clusters,
                                            uint32_t num_passes,
                                            GradientJointGrad* grad);

// --- Iteration 3: Adam optimizer, annealing schedule, optimize loop ---------

// Per-parameter Adam running statistics. Shapes mirror `GradientJointState`.
struct AdamState {
  std::array<std::vector<double>, kNumCh> m_thresholds;
  std::array<std::vector<double>, kNumCh> v_thresholds;
  std::array<std::vector<double>, kNumCh> m_logits;
  std::array<std::vector<double>, kNumCh> v_logits;
  // 1-based step counter used for bias-corrected moment estimates.
  uint32_t step = 0;
};

// Standard Adam hyperparameters.
struct AdamConfig {
  double lr = 0.01;
  double beta1 = 0.9;
  double beta2 = 0.999;
  double eps = 1e-8;
};

void InitAdamState(const GradientJointState& state, AdamState* adam);

// Applies one Adam update to `state` from `grad`. Mutates `state` in place and
// advances `adam->step`. The caller must have already computed `grad` via
// `ComputeSoftACCostWithGrad` and not reset it between forward/backward and
// this call.
void AdamStep(const GradientJointGrad& grad, const AdamConfig& cfg,
              AdamState* adam, GradientJointState* state);

// Geometric annealing schedule driving `pass_temperature` and
// `threshold_temperature` from their init values to their final values over
// `hot_iters` (held constant) followed by `anneal_iters` (geometric decay).
struct AnnealSchedule {
  uint32_t hot_iters = 0;
  uint32_t anneal_iters = 0;
  double pass_init = 1.0;
  double pass_final = 0.05;
  double threshold_init = 50.0;
  double threshold_final = 0.5;
};

// Sets `state->pass_temperature` and `state->threshold_temperature` according
// to `schedule` for the given 0-based `step_index`.
//   step_index < hot_iters              => init values
//   step_index >= hot_iters + anneal_iters => final values (clamped)
//   otherwise                            => geometric interpolation
void ApplyAnnealing(const AnnealSchedule& schedule, uint32_t step_index,
                    GradientJointState* state);

// Projects thresholds to be strictly increasing. Required because gradient
// updates can swap adjacent thresholds, which would produce negative bucket
// weights in the forward pass. Uses `epsilon` as the minimum gap.
void ProjectThresholdsMonotonic(GradientJointState* state,
                                double epsilon = 1e-6);

struct OptimizeResult {
  // Cost (bits) at the very first forward pass, before any Adam step.
  double init_cost_bits = 0.0;
  // Cost (bits) at the final state after the full schedule.
  double final_cost_bits = 0.0;
  // Total number of forward+backward iterations executed.
  uint32_t iters_taken = 0;
};

// Top-level optimizer loop. Runs `hot_iters + anneal_iters` rounds of
// forward+backward + Adam step + annealing + monotonicity projection. Mutates
// `state` in place. Returns init and final costs for smoke-test assertions.
OptimizeResult RunGradientJointSolve(const JPEGOptData& d,
                                     const ContextMap& ctx_map,
                                     uint32_t num_clusters,
                                     uint32_t num_passes,
                                     const AdamConfig& adam_cfg,
                                     const AnnealSchedule& schedule,
                                     GradientJointState* state);

// Rounds a soft `GradientJointState` to a hard `PassSearchResult`. Pass
// assignment = argmax over logits per block. Thresholds are rounded to
// `int16_t` and projected to strictly increasing.
PassSearchResult RoundToHardAssignment(const JPEGOptData& d,
                                       const GradientJointState& state,
                                       const ContextMap& ctx_map,
                                       uint32_t num_clusters,
                                       uint32_t num_passes);

}  // namespace jxl

#endif  // LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_GRAD_H_
