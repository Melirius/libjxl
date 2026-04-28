// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Gradient-based joint-relaxation optimizer for JPEG lossless recompression
// (Lane B from plans/in-the-pass-aware-scheme-keen-barto.md). See
// plans/lane_b_progress.md for the iteration-by-iteration history.
//
// `GradientJointState` holds three continuous-variable bundles that the
// optimizer can move through gradient descent:
//   - `thresholds`      — per-axis DC thresholds, real-valued (rounded back to
//                         int16_t when collapsing to a hard `PassSearchResult`).
//   - `pass_logits`     — per-block pre-softmax pass weights.
//   - `cluster_logits`  — per-(channel, cell) pre-softmax cluster weights.
// Each bundle has its own sigmoid/softmax temperature; annealing all three
// to ~0 collapses the soft state back to a hard one-hot assignment.
//
// `ComputeSoftTotalCost` is the load-bearing forward pass: it walks every
// block of `JPEGOptData`, applies the three soft membership weights, and
// reduces to AC entropy + NZ entropy + signalling overhead. Counts match the
// existing `EvaluatePassAwareModel` at temperatures → 0; gradient flows
// analytically through entropy terms, signalling overhead is held constant.
//
// `RunGradientJointSolve` is the inner Adam + annealing loop for a single
// `(factorization, num_passes)` configuration.
// `SearchGradientJointContextModel` is the public entry point used by
// `enc_jpeg_frame.cc` when `effort.use_gradient_joint_search` is set: it
// sweeps `(factorization, num_passes)` tuples in parallel and returns the
// hard-rounded `PassSearchResult` with the lowest final cost.

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

  // Cluster logits `[channel][cell * num_clusters + cluster]` (iteration 5).
  // Softmax over the cluster axis per `(channel, cell)` gives soft cluster
  // membership `rho_{c,cell,k}`. Sizes: `cluster_logits[c]` has length
  // `num_cells * num_clusters` when `c < d.channels`, else empty.
  std::array<std::vector<double>, kNumCh> cluster_logits;

  // Softmax temperature for cluster assignment. As `cluster_temperature -> 0+`,
  // `rho` collapses to one-hot argmax, matching the old hard `ctx_map`.
  double cluster_temperature = 1.0;

  uint32_t num_passes = 1;

  // Number of active clusters in the context map. Iteration 5 stores this in
  // state so the cost functions no longer need a separate `num_clusters` arg.
  uint32_t num_clusters = 1;

  // Number of cells per channel. Iteration 5 stores this in state so the cost
  // functions no longer need a separate `ctx_map` arg; the cell count is
  // needed to decode `cluster_logits[c]` indexing. Equal to
  // `(thresholds[0].size() + 1) * (thresholds[1].size() + 1) *
  //  (thresholds[2].size() + 1)`.
  uint32_t num_cells = 1;
};

// Initializes `GradientJointState` from a hard `PassSearchResult`. Thresholds
// are cast to `double`. Pass logits are `+hard_logit` for the assigned pass,
// `-hard_logit` elsewhere. Cluster logits are `+hard_logit` for the cluster
// selected by `hard.ctx_map`, `-hard_logit` elsewhere. `hard_logit` should be
// large enough that softmax at `temperature = 1` is numerically
// indistinguishable from one-hot. All three temperatures are set to the
// provided values (callers use a tiny value for hard-limit correctness tests).
GradientJointState InitGradientJointStateFromHard(
    const JPEGOptData& d, const PassSearchResult& hard, double hard_logit,
    double threshold_temperature, double pass_temperature,
    double cluster_temperature);

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

// Analytic gradient of the soft cost with respect to the optimizable
// parameters. Shapes mirror `GradientJointState`.
struct GradientJointGrad {
  std::array<std::vector<double>, kNumCh> thresholds;
  std::array<std::vector<double>, kNumCh> pass_logits;
  std::array<std::vector<double>, kNumCh> cluster_logits;
};

// Zeros the gradient and sizes it to match `state`. Callers that accumulate
// across blocks should call this once before the first backward pass.
void ResetGradientJointGrad(const GradientJointState& state,
                            GradientJointGrad* grad);

// Computes soft AC cost for the given state. Cluster information comes from
// `state.cluster_logits` / `state.num_clusters`; iteration 5 removed the
// separate `ctx_map` / `num_clusters` parameters.
//
// The formula mirrors `EvaluatePassAwareModel`:
//   ac_cost = sum over (cluster, pass) cp of
//             [sum_zdc ftab(N_cp_zdc) - sum_hist_bin ftab(h_cp_hist_bin)]
// where the soft count at (cp, zdc) aggregates block-level contributions
// weighted by three soft memberships:
//   gamma_{b, cell} = cell membership via threshold sigmoids
//   pi_{b, p}       = pass softmax per block
//   rho_{c, cell, k} = cluster softmax per (channel, cell)
//
// `ftab` is evaluated on fractional counts via the continuous extension
// `n * log2(n)`; at integer `n` this matches the precomputed `ftab` table up to
// a half-ULP fixed-point rounding.
SoftCostResult ComputeSoftACCost(const JPEGOptData& d,
                                 const GradientJointState& state);

// Forward pass plus analytic backward pass. Gradient accumulates into `*grad`
// (call `ResetGradientJointGrad` first). Gradient flows through `thresholds`
// (AC only), `pass_logits` (AC + NZ), and `cluster_logits` (AC + NZ). The
// threshold-gradient formula uses the sigmoid's analytic derivative, the
// pass-logit gradient uses the softmax Jacobian scaled by 1/tau_pi, and the
// cluster-logit gradient uses the softmax Jacobian scaled by 1/tau_cluster.
SoftCostResult ComputeSoftACCostWithGrad(const JPEGOptData& d,
                                         const GradientJointState& state,
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
                                    const GradientJointState& state);

// Total cost with analytic gradient. Gradient flows through the AC term
// (iteration 2), the NZ entropy term (block's own pi for the h split, block's
// cell weight for both h and N, cluster logits from iteration 5), and
// `cluster_logits` on both AC and NZ. The `predicted_nz` derivation and the pb
// bucket selection are treated as non-differentiable; likewise the signalling
// overhead term. The optimizer still benefits from reducing AC + NZ cost;
// signalling changes come along for the ride through rounding.
SoftCostResult ComputeSoftTotalCostWithGrad(const JPEGOptData& d,
                                            const GradientJointState& state,
                                            GradientJointGrad* grad);

// --- Iteration 3: Adam optimizer, annealing schedule, optimize loop ---------

// Per-parameter Adam running statistics. Shapes mirror `GradientJointState`.
struct AdamState {
  std::array<std::vector<double>, kNumCh> m_thresholds;
  std::array<std::vector<double>, kNumCh> v_thresholds;
  std::array<std::vector<double>, kNumCh> m_logits;
  std::array<std::vector<double>, kNumCh> v_logits;
  std::array<std::vector<double>, kNumCh> m_cluster_logits;
  std::array<std::vector<double>, kNumCh> v_cluster_logits;
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
  double cluster_init = 1.0;
  double cluster_final = 0.05;
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
// `fa/fb/fc` are the DC interval counts per axis and `num_passes` is the pass
// count; all four are used only for debug logging to identify which worker
// slot is producing output.
OptimizeResult RunGradientJointSolve(const JPEGOptData& d,
                                     const AdamConfig& adam_cfg,
                                     const AnnealSchedule& schedule,
                                     GradientJointState* state,
                                     uint32_t fa = 0, uint32_t fb = 0,
                                     uint32_t fc = 0, uint32_t num_passes = 0);

// Rounds a soft `GradientJointState` to a hard `PassSearchResult`.
//   - Pass assignment: argmax over pass logits per block.
//   - `ctx_map`:       argmax over cluster logits per (channel, cell). Size is
//                      `d.channels * state.num_cells`.
//   - Thresholds:      rounded to `int16_t` and projected to strictly
//                      increasing.
//   - `num_passes` and `num_clusters` are copied from `state`.
PassSearchResult RoundToHardAssignment(const JPEGOptData& d,
                                       const GradientJointState& state);

// --- Iteration 6: Parallel factorization sweep --------------------------

// Cold initialization for a factorization `(a, b, c)`. Thresholds are derived
// from `InitThresh` per axis. Pass logits and cluster logits are all zero
// (uniform softmax), so Adam discovers structure from scratch. Temperatures
// and `num_clusters` / `num_passes` are set from the caller's arguments.
GradientJointState InitGradientJointStateFromFactorization(
    const JPEGOptData& d, const Factorization& f, uint32_t num_passes,
    uint32_t num_clusters, double threshold_temperature,
    double pass_temperature, double cluster_temperature);

// Greedy agglomerative cluster reduction on a hard `PassSearchResult`. Runs
// after `RoundToHardAssignment`: tries every pair of clusters `(i, j)` and
// merges the pair whose merge produces the largest decrease in
// `EvaluatePassAwareModel` total cost (entropy + NZ + signalling overhead +
// flat pass overhead). Repeats until no merge reduces cost. Updates
// `result.ctx_map`, `result.num_clusters`, and `result.{ac_cost, nz_cost,
// signalling_overhead, total_cost}` in place.
//
// This mirrors the biclustering path's `overhead_aware_tail` semantics for
// the gradient-search output: Adam minimizes entropy ignoring the
// piecewise-constant overhead, and this pass cleans up the resulting
// over-allocation of clusters when `entropy + overhead` favors fewer.
//
// Returns the number of merges accepted (0 if no improvement).
StatusOr<uint32_t> ReduceClustersAgglomerative(const JPEGOptData& d,
                                               PassSearchResult* result,
                                               ThreadPool* pool);

// Runs the gradient-based Lane B optimizer on every maximal factorization of
// `opt_data` in parallel, rounds each to a hard `PassSearchResult`, and
// returns the one with the lowest final soft total cost.
//
// Effort hyperparameters read from `effort`:
//   grad_hot_iters, grad_anneal_iters, grad_init_temperature, grad_lr.
// `num_passes = 1` for iteration 6 — multi-pass sweep is a follow-up.
// `num_clusters = kMaxClusters - (d.channels == 1)`, matching the hard path.
//
// Returns an error if `MaximalFactorizations(opt_data)` is empty.
StatusOr<PassSearchResult> SearchGradientJointContextModel(
    std::shared_ptr<const JPEGOptData> opt_data,
    const JPEGCtxEffortParams& effort, ThreadPool* pool);

}  // namespace jxl

#endif  // LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_GRAD_H_
