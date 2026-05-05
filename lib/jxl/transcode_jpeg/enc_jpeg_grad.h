// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Gradient-based joint-relaxation optimizer for JPEG lossless recompression
// (Lane B from plans/in-the-pass-aware-scheme-keen-barto.md). See
// plans/lane_b_progress.md for the iteration-by-iteration history.
//
// `GradientState` holds three continuous-variable bundles that the
// optimizer can move through gradient descent:
//   - `thresholds`      — per-axis DC thresholds in compact DC-index space,
//                         real-valued (mapped back to int16_t DC values when
//                         collapsing to a hard `PassSearchResult`).
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
// `RunGradientSolve` is the inner Adam + annealing loop for a single
// `(factorization, num_passes)` configuration.
// `SearchGradientContextModel` is the public entry point used by
// `enc_jpeg_frame.cc` when `effort.use_gradient_joint_search` is set: it
// sweeps `(factorization, num_passes)` tuples in parallel, rounds each state to
// a hard `PassSearchResult`, rescores it with the biclustering hard evaluator,
// and returns the result with the lowest final hard cost.

#ifndef LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_GRAD_H_
#define LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_GRAD_H_

#include <array>
#include <cstdint>
#include <vector>

#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_passes.h"

namespace jxl {

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
  double ctx_init = 1.0;
  double ctx_final = 0.05;
};

// Continuous-variable state for the joint-relaxation optimizer.
//
// Threshold values live in compact DC-index space: index `i` means
// `JPEGOptData::DC_vals[axis][i]`, the first DC value of the next bucket. They
// are stored as `double` here so the sigmoid relaxation can place them
// off-integer during optimization; conversion to actual `int16_t` DC thresholds
// happens when collapsing back to a hard `PassSearchResult`.
struct GradientState {
  // Per-axis DC-index thresholds (continuous). Length of each axis matches the
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

  // Soft context-map logits, flat: index = p*num_clusters*kZDC*H + k*kZDC*H +
  // zdc*H + h, where H = num_hists. Softmax over h gives the probability that
  // (cluster k, ZDC context zdc) routes to histogram h in pass p.
  // Size: num_passes * num_clusters * kZDC * num_hists. Empty when
  // num_hists<=1.
  std::vector<double> ctx_logits;

  // Softmax temperature for context-map assignment.
  double ctx_temperature = 1.0;

  // Number of histograms in the soft context map (≤ 255). Set to 1 to disable.
  uint32_t num_hists = 1;

  uint32_t num_passes = 1;

  // Number of active clusters in the context map.
  uint32_t num_clusters = 1;

  // Number of cells per channel needed to decode `cluster_logits[c]` indexing.
  // Equal to `(thresholds[0].size() + 1) * (thresholds[1].size() + 1) *
  //  (thresholds[2].size() + 1)`.
  uint32_t num_cells = 1;

  // Sets `state->pass_temperature` and `state->threshold_temperature` according
  // to `schedule` for the given 0-based `step_index`.
  //   step_index < hot_iters              => init values
  //   step_index >= hot_iters + anneal_iters => final values (clamped)
  //   otherwise                            => geometric interpolation
  void ApplyAnnealing(const AnnealSchedule& schedule, uint32_t step_index) {
    if (step_index < schedule.hot_iters || schedule.anneal_iters == 0) {
      pass_temperature = schedule.pass_init;
      threshold_temperature = schedule.threshold_init;
      cluster_temperature = schedule.cluster_init;
      ctx_temperature = schedule.ctx_init;
      return;
    }
    const uint32_t anneal_step = step_index - schedule.hot_iters;
    if (anneal_step >= schedule.anneal_iters) {
      pass_temperature = schedule.pass_final;
      threshold_temperature = schedule.threshold_final;
      cluster_temperature = schedule.cluster_final;
      ctx_temperature = schedule.ctx_final;
      return;
    }
    const double t = anneal_step / (schedule.anneal_iters - 1.0);
    // Geometric interpolation in log-space: final -> end, init -> start.
    const double log_pass = (1.0 - t) * std::log(schedule.pass_init) +
                            t * std::log(schedule.pass_final);
    const double log_thr = (1.0 - t) * std::log(schedule.threshold_init) +
                           t * std::log(schedule.threshold_final);
    const double log_clu = (1.0 - t) * std::log(schedule.cluster_init) +
                           t * std::log(schedule.cluster_final);
    const double log_ctx = (1.0 - t) * std::log(schedule.ctx_init) +
                           t * std::log(schedule.ctx_final);
    pass_temperature = std::exp(log_pass);
    threshold_temperature = std::exp(log_thr);
    cluster_temperature = std::exp(log_clu);
    ctx_temperature = std::exp(log_ctx);
  }
};

// Fills `state->ctx_logits` with a round-robin hard assignment: for each
// (pass, cluster k, zdc), the preferred histogram is `(k * kZDC + zdc) % H`
// and receives `+hard_logit`; all other H−1 histograms receive `−hard_logit`.
// Requires `state->num_hists`, `state->num_clusters`, and `state->num_passes`
// to be set before calling. Replaces any existing `ctx_logits` content.
void InitCtxLogitsRoundRobin(double hard_logit, GradientState* state);

struct SoftCostResult {
  // AC entropy cost under the soft aggregation. Units: bits (not fixed-point).
  // Multiply by `kFScale` and round to compare against `FixedPointCost`.
  double ac_cost_bits = 0.0;

  // NZ entropy cost (iteration 4).
  double nz_cost_bits = 0.0;

  // Signalling overhead estimate: ANS-population-minus-Shannon for the actual
  // AC histograms selected from the `H` budget and for NZ slots, plus a flat
  // per-pass overhead. Treated as constant for gradient purposes.
  double signalling_overhead_bits = 0.0;

  // Sum of the three components.
  double total_cost_bits = 0.0;

  // Number of non-empty actual AC histogram slots `(pass, h)` visited after
  // context-map routing.
  uint32_t num_cp_slots = 0;
};

// Analytic gradient of the soft cost with respect to the optimizable
// parameters. Shapes mirror `GradientState`.
struct GradientGrad {
  std::array<std::vector<double>, kNumCh> thresholds;
  std::array<std::vector<double>, kNumCh> pass_logits;
  std::array<std::vector<double>, kNumCh> cluster_logits;
  std::vector<double> ctx_logits;  // flat, same layout as GradientState

  // Zeros the gradient and sizes it to match `state`. Callers that accumulate
  // across blocks should call this once before the first backward pass.
  void Reset(const GradientState& state) {
    for (uint32_t a = 0; a < kNumCh; ++a) {
      thresholds[a].assign(state.thresholds[a].size(), 0.0);
      pass_logits[a].assign(state.pass_logits[a].size(), 0.0);
      cluster_logits[a].assign(state.cluster_logits[a].size(), 0.0);
    }
    ctx_logits.assign(state.ctx_logits.size(), 0.0);
  }
};

// --- Iteration 3: Adam optimizer, annealing schedule, optimize loop ---------

// Per-parameter Adam running statistics. Shapes mirror `GradientState`.
struct AdamState {
  std::array<std::vector<double>, kNumCh> m_thresholds;
  std::array<std::vector<double>, kNumCh> v_thresholds;
  std::array<std::vector<double>, kNumCh> m_logits;
  std::array<std::vector<double>, kNumCh> v_logits;
  std::array<std::vector<double>, kNumCh> m_cluster_logits;
  std::array<std::vector<double>, kNumCh> v_cluster_logits;
  std::vector<double> m_ctx_logits;  // flat, same layout as GradientState
  std::vector<double> v_ctx_logits;
  // 1-based step counter used for bias-corrected moment estimates.
  uint32_t step = 0;

  explicit AdamState(const GradientState& state) {
    for (uint32_t a = 0; a < kNumCh; ++a) {
      m_thresholds[a].assign(state.thresholds[a].size(), 0.0);
      v_thresholds[a].assign(state.thresholds[a].size(), 0.0);
      m_logits[a].assign(state.pass_logits[a].size(), 0.0);
      v_logits[a].assign(state.pass_logits[a].size(), 0.0);
      m_cluster_logits[a].assign(state.cluster_logits[a].size(), 0.0);
      v_cluster_logits[a].assign(state.cluster_logits[a].size(), 0.0);
    }
    m_ctx_logits.assign(state.ctx_logits.size(), 0.0);
    v_ctx_logits.assign(state.ctx_logits.size(), 0.0);
  };
};

// Adam hyperparameters.
// These more aggressive values allow larger gradient steps, which speeds up
// convergence.
struct AdamConfig {
  double lr = 0.01;
  double beta1 = 0.8;   // 0.9; //
  double beta2 = 0.95;  // 0.999; // 0.99
  double eps = 1e-8;
};

// Applies one Adam update to `state` from `grad`. Mutates `state` in place and
// advances `adam->step`. The caller must have already computed `grad` via
// `ComputeSoftTotalCostWithGrad` and not reset it between forward/backward and
// this call.
void AdamStep(const GradientGrad& grad, const AdamConfig& cfg, AdamState* adam,
              GradientState* state);

// Projects thresholds to be strictly increasing. Required because gradient
// updates can swap adjacent thresholds, which would produce negative bucket
// weights in the forward pass. Uses `epsilon` as the minimum gap.
void ProjectThresholdsMonotonic(GradientState* state, double epsilon = 1e-2);

struct OptimizeResult {
  // Cost (bits) at the very first forward pass, before any Adam step.
  double init_cost_bits = 0.0;
  // Cost (bits) at the final state after the full schedule.
  double final_cost_bits = 0.0;
  // Total number of forward+backward iterations executed.
  uint32_t iters_taken = 0;
};

struct GradientSearchCandidate {
  PassSearchResult result;
  double target_cost_bits = 0.0;
  uint32_t factorization[3] = {};
  uint32_t num_passes = 0;
  bool is_best = false;
};

// Top-level optimizer loop. Runs `hot_iters + anneal_iters` rounds of
// forward+backward + Adam step + annealing + monotonicity projection. Mutates
// `state` in place. Returns init and final costs for smoke-test assertions.
// `fa/fb/fc` are the DC interval counts per axis and `num_passes` is the pass
// count; all four are used only for debug logging to identify which worker
// slot is producing output.
OptimizeResult RunGradientSolve(const JPEGOptData& d,
                                const AdamConfig& adam_cfg,
                                const AnnealSchedule& schedule,
                                GradientState* state, uint32_t fa = 0,
                                uint32_t fb = 0, uint32_t fc = 0,
                                uint32_t num_passes = 0);

// Rounds a soft `GradientState` to a hard `PassSearchResult`.
//   - Pass assignment: argmax over pass logits per block.
//   - `ctx_map`:       argmax over cluster logits per (channel, cell). Size is
//                      `d.channels * state.num_cells`.
//   - Thresholds:      rounded in DC-index space, projected to strictly
//                      increasing, then mapped to actual `int16_t` DC values.
//   - `num_passes` and `num_clusters` are copied from `state`.
PassSearchResult RoundToHardAssignment(const JPEGOptData& d,
                                       const GradientState& state);

// --- Iteration 6: Parallel factorization sweep --------------------------

// Cold initialization for a factorization `(a, b, c)`. Thresholds are derived
// from `InitThresh` per axis. Pass logits and cluster logits are all zero
// (uniform softmax), so Adam discovers structure from scratch. Temperatures
// and `num_clusters` / `num_passes` are set from the caller's arguments.
GradientState InitGradientStateFromFactorization(
    const JPEGOptData& d, const Factorization& f, uint32_t num_passes,
    uint32_t num_clusters, double threshold_temperature,
    double pass_temperature, double cluster_temperature,
    uint32_t num_hists = 1);

// Removes thresholds that don't actually separate clusters: for each axis,
// scans thresholds and drops the ones whose adjacent buckets map to the
// same cluster across every (channel, perpendicular-cell) combination.
// Updates `result.thresholds` and `result.ctx_map` in place; cluster
// assignments per block are preserved exactly. Saves bitstream size by
// shrinking the factorization metadata and ctx_map. Returns the total number
// of thresholds removed across all axes.
//
// Typically run AFTER `ReduceClustersAgglomerative`: cluster merging makes
// many thresholds redundant by collapsing distinct clusters into shared ids.
uint32_t PruneRedundantThresholds(const JPEGOptData& d,
                                  PassSearchResult* result);

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
// `opt_data` in parallel, rounds each to a hard `PassSearchResult`, rescoring
// with the biclustering hard evaluator before ranking.
//
// Effort hyperparameters read from `effort`:
//   grad_hot_iters, grad_anneal_iters, grad_init_temperature, grad_lr.
// `num_passes = 1` for iteration 6 — multi-pass sweep is a follow-up.
// `num_clusters = kMaxClusters - (d.channels == 1)`, matching the hard path.
//
// Returns an error if `MaximalFactorizations(opt_data)` is empty.
StatusOr<PassSearchResult> SearchGradientContextModel(
    std::shared_ptr<const JPEGOptData> opt_data,
    const JPEGCtxEffortParams& effort, ThreadPool* pool,
    std::vector<GradientSearchCandidate>* debug_candidates = nullptr);

}  // namespace jxl

#endif  // LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_GRAD_H_
