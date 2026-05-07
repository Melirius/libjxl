// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Lane B iteration 4: soft forward and analytic backward for AC + NZ cost,
// plus signalling overhead. Builds on iterations 1-3.
//
// Iteration 1 established the AC forward pass.
// Iteration 2 added the analytic gradient wrt `pass_logits` and `thresholds`.
// Iteration 3 added the Adam optimizer, annealing, and rounding.
// Iteration 4 (this file):
//   - NZ entropy cost with soft neighbor pi for `predicted_nz`; pb is
//     integer-rounded (no gradient through bucket selection) and the per-block
//     h-contribution splits between bin_real = NZIndex(pb, nz_b) and
//     bin_zero = NZIndex(pb, 0) with weights pi[p] and 1 - pi[p].
//   - Signalling overhead = ANSPopulationCost - ShannonEntropy for routed AC
//     histograms and NZ slots. Fixed-P mode adds the flat pass overhead as a
//     constant; auto-P mode uses a smooth pass occupancy surrogate so gradients
//     can retire unused passes.

#include "lib/jxl/transcode_jpeg/enc_jpeg_grad.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <mutex>
#include <utility>
#include <vector>

#include "lib/jxl/transcode_jpeg/enc_jpeg_grad_internal.h"

#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "lib/jxl/transcode_jpeg/enc_jpeg_grad.cc"
// clang-format off
#include <hwy/foreach_target.h>
#include <hwy/highway.h>
#include <hwy/contrib/math/math-inl.h>
// clang-format on

#include "lib/jxl/ac_context.h"
#include "lib/jxl/base/status.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_bicluster.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_histogram.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_assign.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_cluster.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_stream.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_utils.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_passes.h"

HWY_BEFORE_NAMESPACE();
namespace jxl {
namespace HWY_NAMESPACE {

namespace hn = hwy::HWY_NAMESPACE;

void AdamApplyVec(const double* HWY_RESTRICT grad, double* HWY_RESTRICT param,
                  double* HWY_RESTRICT m, double* HWY_RESTRICT v, size_t n,
                  double beta1, double beta2, double lr, double inv_bias1,
                  double inv_bias2, double eps) {
  const hn::ScalableTag<double> d;
  const size_t N = hn::Lanes(d);
  const auto vb1 = hn::Set(d, beta1);
  const auto vomb1 = hn::Set(d, 1.0 - beta1);
  const auto vb2 = hn::Set(d, beta2);
  const auto vomb2 = hn::Set(d, 1.0 - beta2);
  const auto vlr = hn::Set(d, lr);
  const auto vib1 = hn::Set(d, inv_bias1);
  const auto vib2 = hn::Set(d, inv_bias2);
  const auto veps = hn::Set(d, eps);
  size_t i = 0;
  for (; i + N <= n; i += N) {
    const auto g = hn::LoadU(d, grad + i);
    const auto new_m = hn::MulAdd(vomb1, g, hn::Mul(vb1, hn::LoadU(d, m + i)));
    const auto new_v =
        hn::MulAdd(vomb2, hn::Mul(g, g), hn::Mul(vb2, hn::LoadU(d, v + i)));
    hn::StoreU(new_m, d, m + i);
    hn::StoreU(new_v, d, v + i);

    const auto m_hat = hn::Mul(new_m, vib1);
    const auto v_hat = hn::Mul(new_v, vib2);
    const auto denom = hn::Add(hn::Sqrt(v_hat), veps);
    const auto update = hn::Mul(vlr, hn::Div(m_hat, denom));
    hn::StoreU(hn::Sub(hn::LoadU(d, param + i), update), d, param + i);
  }
  for (; i < n; ++i) {
    m[i] = beta1 * m[i] + (1.0 - beta1) * grad[i];
    v[i] = beta2 * v[i] + (1.0 - beta2) * grad[i] * grad[i];
    const double m_hat = m[i] * inv_bias1;
    const double v_hat = v[i] * inv_bias2;
    param[i] -= lr * m_hat / (std::sqrt(v_hat) + eps);
  }
}

void AdamStepImpl(const GradientGrad& grad, const AdamConfig& cfg,
                  AdamState* adam, GradientState* state) {
  ++adam->step;
  const double beta1 = cfg.beta1;
  const double beta2 = cfg.beta2;
  const double inv_bias1 = 1.0 / (1.0 - std::pow(beta1, adam->step));
  const double inv_bias2 = 1.0 / (1.0 - std::pow(beta2, adam->step));

  auto apply = [&](const std::vector<double>& g, std::vector<double>* param,
                   std::vector<double>* m, std::vector<double>* v) {
    if (g.empty()) return;
    AdamApplyVec(g.data(), param->data(), m->data(), v->data(), g.size(), beta1,
                 beta2, cfg.lr, inv_bias1, inv_bias2, cfg.eps);
  };

  for (uint32_t a = 0; a < kNumCh; ++a) {
    apply(grad.thresholds[a], &state->thresholds[a], &adam->m_thresholds[a],
          &adam->v_thresholds[a]);
    apply(grad.pass_logits[a], &state->pass_logits[a], &adam->m_logits[a],
          &adam->v_logits[a]);
    apply(grad.cluster_logits[a], &state->cluster_logits[a],
          &adam->m_cluster_logits[a], &adam->v_cluster_logits[a]);
  }
  apply(grad.pass_gates, &state->pass_gates, &adam->m_pass_gates,
        &adam->v_pass_gates);
  apply(grad.ctx_logits, &state->ctx_logits, &adam->m_ctx_logits,
        &adam->v_ctx_logits);
}

}  // namespace HWY_NAMESPACE
}  // namespace jxl
HWY_AFTER_NAMESPACE();

#if HWY_ONCE

namespace jxl {

HWY_EXPORT(AdamStepImpl);

void InitCtxLogitsRoundRobin(double hard_logit, GradientState* state) {
  const size_t H = state->num_hists;
  const size_t K = state->num_clusters;
  const size_t P = state->num_passes;
  constexpr size_t kZDC = kZeroDensityContextCount;
  state->ctx_logits.assign(P * K * kZDC * H, -hard_logit);
  for (size_t p = 0; p < P; ++p) {
    for (size_t k = 0; k < K; ++k) {
      for (size_t zdc = 0; zdc < kZDC; ++zdc) {
        const size_t h = (k * kZDC + zdc) % H;
        state->ctx_logits[(p * K * kZDC + k * kZDC + zdc) * H + h] = hard_logit;
      }
    }
  }
}

SoftCostResult ComputeSoftTotalCost(const JPEGOptData& d,
                                    const GradientAux& aux,
                                    const GradientState& state,
                                    GradientScratch* scratch) {
  if (state.num_passes == 1) {
    return SoftForwardBackwardOnePass(d, aux, state, nullptr, scratch);
  }
  return SoftForwardBackwardManyPass(d, aux, state, nullptr, scratch);
}

SoftCostResult SoftForwardBackward(const JPEGOptData& d, const GradientAux& aux,
                                   const GradientState& state,
                                   GradientGrad* grad,
                                   GradientScratch* scratch) {
  if (state.num_passes == 1) {
    return SoftForwardBackwardOnePass(d, aux, state, grad, scratch);
  }
  return SoftForwardBackwardManyPass(d, aux, state, grad, scratch);
}

// --- Iteration 3: Adam optimizer, annealing schedule, optimize loop ---------

void AdamStep(const GradientGrad& grad, const AdamConfig& cfg, AdamState* adam,
              GradientState* state) {
  HWY_DYNAMIC_DISPATCH(AdamStepImpl)(grad, cfg, adam, state);
}

void ProjectThresholdsMonotonic(GradientState* state, double epsilon) {
  for (uint32_t a = 0; a < kNumCh; ++a) {
    auto& T = state->thresholds[a];
    for (size_t j = 1; j < T.size(); ++j) {
      const double lower_bound = T[j - 1] + epsilon;
      if (T[j] < lower_bound) T[j] = lower_bound;
    }
  }
}

static double DCThresholdValueToIndex(const JPEGOptData& d, uint32_t axis,
                                      int16_t threshold) {
  const auto& vals = d.DC_vals[axis];
  const auto it = std::lower_bound(vals.begin(), vals.end(), threshold);
  return static_cast<double>(it - vals.begin());
}

namespace {

constexpr uint32_t kGradientRaceWarmupIters = 10;
constexpr uint32_t kGradientRaceCheckPeriod = 5;
constexpr uint32_t kGradientRaceConsecutiveChecks = 2;
constexpr uint32_t kGradientRaceGlobalKeep = 4;
constexpr uint32_t kGradientRacePerPassKeepDuringHot = 2;
constexpr double kGradientRaceAbandonRatio = 1.01;

uint32_t CompactHardClusters(PassSearchResult* result) {
  std::array<bool, 256> used{};
  for (uint8_t id : result->ctx_map) {
    JXL_DASSERT(id < result->num_clusters);
    used[id] = true;
  }

  std::array<uint8_t, 256> remap{};
  uint32_t next = 0;
  for (uint32_t id = 0; id < used.size(); ++id) {
    if (!used[id]) continue;
    JXL_DASSERT(next <= std::numeric_limits<uint8_t>::max());
    remap[id] = static_cast<uint8_t>(next++);
  }

  for (uint8_t& id : result->ctx_map) {
    id = remap[id];
  }
  result->num_clusters = std::max<uint32_t>(next, 1);
  return result->num_clusters;
}

uint32_t CompactHardPasses(PassSearchResult* result) {
  std::array<bool, 256> used{};
  for (const auto& pass_assignment : result->pass_assignment) {
    for (uint8_t pass : pass_assignment) {
      JXL_DASSERT(pass < result->num_passes);
      used[pass] = true;
    }
  }

  std::array<uint8_t, 256> remap{};
  uint32_t next = 0;
  for (uint32_t pass = 0; pass < result->num_passes; ++pass) {
    if (!used[pass]) continue;
    JXL_DASSERT(next <= std::numeric_limits<uint8_t>::max());
    remap[pass] = static_cast<uint8_t>(next++);
  }
  if (next == 0) {
    result->num_passes = 1;
    return result->num_passes;
  }

  for (auto& pass_assignment : result->pass_assignment) {
    for (uint8_t& pass : pass_assignment) {
      pass = remap[pass];
    }
  }
  result->num_passes = next;
  return result->num_passes;
}

struct GradientRaceDecision {
  double best_cost_bits = std::numeric_limits<double>::infinity();
  uint32_t global_rank = std::numeric_limits<uint32_t>::max();
  uint32_t pass_rank = std::numeric_limits<uint32_t>::max();
  bool protected_by_global_keep = false;
  bool protected_by_pass_keep = false;
};

class GradientRaceState {
 public:
  explicit GradientRaceState(size_t num_candidates)
      : candidates_(num_candidates) {}

  GradientRaceDecision Update(uint32_t idx, uint32_t num_passes, uint32_t iter,
                              double cost_bits, uint32_t hot_iters) {
    std::lock_guard<std::mutex> lock(mutex_);
    JXL_DASSERT(idx < candidates_.size());
    Candidate& c = candidates_[idx];
    c.has_cost = true;
    c.latest_cost_bits = cost_bits;
    c.num_passes = num_passes;

    if (!c.abandoned && (cost_bits < best_cost_bits_ ||
                         (cost_bits == best_cost_bits_ && idx < best_idx_))) {
      best_cost_bits_ = cost_bits;
      best_idx_ = idx;
    }
    if (best_idx_ < candidates_.size() && candidates_[best_idx_].abandoned) {
      RecomputeBestLocked();
    }

    GradientRaceDecision decision;
    decision.best_cost_bits = best_cost_bits_;
    decision.global_rank = 1;
    decision.pass_rank = 1;
    for (size_t i = 0; i < candidates_.size(); ++i) {
      const Candidate& other = candidates_[i];
      if (!other.has_cost || other.abandoned) continue;
      const bool before = other.latest_cost_bits < cost_bits ||
                          (other.latest_cost_bits == cost_bits && i < idx);
      if (!before) continue;
      ++decision.global_rank;
      if (other.num_passes == num_passes) ++decision.pass_rank;
    }
    decision.protected_by_global_keep =
        decision.global_rank <= kGradientRaceGlobalKeep;
    decision.protected_by_pass_keep =
        iter <= hot_iters &&
        decision.pass_rank <= kGradientRacePerPassKeepDuringHot;
    return decision;
  }

  void MarkAbandoned(uint32_t idx) {
    std::lock_guard<std::mutex> lock(mutex_);
    JXL_DASSERT(idx < candidates_.size());
    Candidate& c = candidates_[idx];
    if (!c.abandoned) {
      c.abandoned = true;
      ++abandoned_count_;
    }
    if (idx == best_idx_) RecomputeBestLocked();
  }

  uint32_t abandoned_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return abandoned_count_;
  }

 private:
  struct Candidate {
    double latest_cost_bits = std::numeric_limits<double>::infinity();
    uint32_t num_passes = 0;
    bool has_cost = false;
    bool abandoned = false;
  };

  void RecomputeBestLocked() {
    best_cost_bits_ = std::numeric_limits<double>::infinity();
    best_idx_ = std::numeric_limits<uint32_t>::max();
    for (size_t i = 0; i < candidates_.size(); ++i) {
      const Candidate& c = candidates_[i];
      if (!c.has_cost || c.abandoned) continue;
      if (c.latest_cost_bits < best_cost_bits_ ||
          (c.latest_cost_bits == best_cost_bits_ &&
           static_cast<uint32_t>(i) < best_idx_)) {
        best_cost_bits_ = c.latest_cost_bits;
        best_idx_ = static_cast<uint32_t>(i);
      }
    }
  }

  mutable std::mutex mutex_;
  std::vector<Candidate> candidates_;
  double best_cost_bits_ = std::numeric_limits<double>::infinity();
  uint32_t best_idx_ = std::numeric_limits<uint32_t>::max();
  uint32_t abandoned_count_ = 0;
};

bool GradientDumpClusterLogitsEnabled() {
  static const bool enabled = [] {
    const char* env = std::getenv("JXL_DEBUG_GRADIENT_CLUSTER_LOGITS");
    return env != nullptr && env[0] != '\0' &&
           !(env[0] == '0' && env[1] == '\0');
  }();
  return enabled;
}

std::mutex& GradientDumpClusterLogitsMutex() {
  static std::mutex mutex;
  return mutex;
}

void DumpFinalClusterLogits(const JPEGOptData& d, const GradientState& state,
                            uint32_t fa, uint32_t fb, uint32_t fc,
                            uint32_t num_passes) {
  if (!GradientDumpClusterLogitsEnabled()) return;
  const uint32_t K = state.num_clusters;
  const uint32_t num_cells = state.num_cells;
  if (K == 0 || num_cells == 0) return;

  std::array<bool, 256> hard_used{};
  std::lock_guard<std::mutex> lock(GradientDumpClusterLogitsMutex());
  fprintf(stderr,
          "PLANNER: [gradient] [(%u,%u,%u) P=%u] Final cluster logits: "
          "channels=%u cells=%u clusters=%u T_cluster=%.6f\n",
          fa, fb, fc, num_passes, d.channels, num_cells, K,
          state.cluster_temperature);

  for (uint32_t c = 0; c < d.channels; ++c) {
    const std::vector<double>& logits = state.cluster_logits[c];
    if (logits.size() != static_cast<size_t>(num_cells) * K) {
      fprintf(stderr,
              "PLANNER: [gradient] [(%u,%u,%u) P=%u] cluster_logits c=%u "
              "size_mismatch size=%zu expected=%zu\n",
              fa, fb, fc, num_passes, c, logits.size(),
              static_cast<size_t>(num_cells) * K);
      continue;
    }
    for (uint32_t cell = 0; cell < num_cells; ++cell) {
      const double* base = &logits[static_cast<size_t>(cell) * K];
      uint32_t best = 0;
      uint32_t second = 0;
      double best_v = base[0];
      double second_v = -std::numeric_limits<double>::infinity();
      for (uint32_t k = 1; k < K; ++k) {
        const double v = base[k];
        if (v > best_v) {
          second = best;
          second_v = best_v;
          best = k;
          best_v = v;
        } else if (v > second_v) {
          second = k;
          second_v = v;
        }
      }
      if (K == 1) second_v = best_v;
      if (best < hard_used.size()) hard_used[best] = true;
      fprintf(stderr,
              "PLANNER: [gradient] [(%u,%u,%u) P=%u] cluster_logits "
              "c=%u cell=%u best=%u second=%u margin=%.9g logits=[",
              fa, fb, fc, num_passes, c, cell, best, second, best_v - second_v);
      for (uint32_t k = 0; k < K; ++k) {
        fprintf(stderr, "%s%.9g", k == 0 ? "" : ",", base[k]);
      }
      fprintf(stderr, "]\n");
    }
  }

  uint32_t used_count = 0;
  for (uint32_t k = 0; k < K && k < hard_used.size(); ++k) {
    if (hard_used[k]) ++used_count;
  }
  fprintf(stderr,
          "PLANNER: [gradient] [(%u,%u,%u) P=%u] Final cluster logits "
          "hard-argmax-used=%u/%u\n",
          fa, fb, fc, num_passes, used_count, K);
  fflush(stderr);
}

}  // namespace

OptimizeResult RunGradientSolve(const JPEGOptData& d, const GradientAux& aux,
                                const AdamConfig& adam_cfg,
                                const AnnealSchedule& schedule,
                                GradientState* state, uint32_t fa, uint32_t fb,
                                uint32_t fc, uint32_t num_passes,
                                GradientRaceState* race = nullptr,
                                uint32_t race_idx = 0) {
  auto start_solve = PlannerClock::now();

  OptimizeResult result;
  GradientScratch scratch(d, aux, *state);
  const uint32_t total_iters = schedule.hot_iters + schedule.anneal_iters;

  // Initial cost at caller-configured temperatures (schedule not applied yet).
  SoftCostResult r = ComputeSoftTotalCost(d, aux, *state, &scratch);
  result.init_cost_bits = r.total_cost_bits;
  result.final_cost_bits = r.total_cost_bits;  // for `total_iters == 0` case
  if (race != nullptr) {
    race->Update(race_idx, state->num_passes, /*iter=*/0, r.total_cost_bits,
                 schedule.hot_iters);
  }
  fprintf(stderr,
          "PLANNER: [gradient] [(%u,%u,%u) P=%u] Initial cost: %.2f bits "
          "(took %.2f ms)\n",
          fa, fb, fc, num_passes, r.total_cost_bits,
          NanosToMs(ElapsedNanos(start_solve, PlannerClock::now())));
  fflush(stderr);

  int64_t total_fwd_ns = 0;
  int64_t total_adam_ns = 0;
  double prev_cost = result.init_cost_bits;

  // Pre-loop forward+grad: applies anneal-step-0 and computes the gradient
  // that iter 0's AdamStep will consume. Subsequent iters reuse the *next*
  // iter's "after-step" forward as both their gradient computation and the
  // post-step cost report, so the per-iter print reflects the cost AFTER
  // this iter's update. The total number of forward+grad calls stays at
  // `total_iters`; we add only one forward-only call at the very end.
  if (total_iters > 0) {
    AdamState adam(*state);
    GradientGrad grad;
    uint32_t race_bad_checkpoints = 0;

    state->ApplyAnnealing(schedule, 0);
    grad.Reset(*state);
    auto start_fwd = PlannerClock::now();
    r = SoftForwardBackward(d, aux, *state, &grad, &scratch);
    auto end_fwd = PlannerClock::now();
    total_fwd_ns += ElapsedNanos(start_fwd, end_fwd);

    for (uint32_t t = 0; t < total_iters; ++t) {
      auto start_iter = PlannerClock::now();

      AdamStep(grad, adam_cfg, &adam, state);
      ProjectThresholdsMonotonic(state);
      auto end_adam = PlannerClock::now();
      total_adam_ns += ElapsedNanos(start_iter, end_adam);

      // Forward at the post-step state. For all but the last iter this
      // doubles as the next iter's gradient computation; for the last iter
      // it is forward-only.
      auto fwd_start = PlannerClock::now();
      if (t + 1 < total_iters) {
        state->ApplyAnnealing(schedule, t + 1);
        grad.Reset(*state);
        r = SoftForwardBackward(d, aux, *state, &grad, &scratch);
      } else {
        r = ComputeSoftTotalCost(d, aux, *state, &scratch);
      }
      auto fwd_end = PlannerClock::now();
      total_fwd_ns += ElapsedNanos(fwd_start, fwd_end);

      result.final_cost_bits = r.total_cost_bits;
      ++result.iters_taken;

      auto end_iter = PlannerClock::now();
      const bool is_hot = t < schedule.hot_iters;
      const double delta = r.total_cost_bits - prev_cost;
      const double pct = (prev_cost != 0.0) ? (delta / prev_cost) * 100.0 : 0.0;
      fprintf(stderr,
              "PLANNER: [gradient] [(%u,%u,%u) P=%u] Iter %u/%u (%s) "
              "cost=%.2f bits delta=%+.2f (%+.3f%%) "
              "fwd=%.2f ms adam=%.2f ms total=%.2f ms\n",
              fa, fb, fc, num_passes, t + 1, total_iters,
              is_hot ? "hot" : "anneal", r.total_cost_bits, delta, pct,
              NanosToMs(ElapsedNanos(fwd_start, fwd_end)),
              NanosToMs(ElapsedNanos(start_iter, end_adam)),
              NanosToMs(ElapsedNanos(start_iter, end_iter)));
      fflush(stderr);

      if (race != nullptr) {
        const uint32_t iter = t + 1;
        const GradientRaceDecision decision =
            race->Update(race_idx, state->num_passes, iter, r.total_cost_bits,
                         schedule.hot_iters);
        const bool is_checkpoint = iter >= kGradientRaceWarmupIters &&
                                   (iter % kGradientRaceCheckPeriod) == 0;
        const bool protected_candidate = decision.protected_by_global_keep ||
                                         decision.protected_by_pass_keep;
        const bool outside_margin =
            std::isfinite(decision.best_cost_bits) &&
            r.total_cost_bits >
                decision.best_cost_bits * kGradientRaceAbandonRatio;
        if (is_checkpoint && !protected_candidate && outside_margin) {
          ++race_bad_checkpoints;
        } else if (is_checkpoint) {
          race_bad_checkpoints = 0;
        }
        if (race_bad_checkpoints >= kGradientRaceConsecutiveChecks) {
          result.abandoned = true;
          race->MarkAbandoned(race_idx);
          fprintf(stderr,
                  "PLANNER: [gradient] [(%u,%u,%u) P=%u] Early abandon at "
                  "iter %u/%u: cost=%.2f bits, best_live=%.2f bits "
                  "(%.3f%% over), rank=%u pass_rank=%u\n",
                  fa, fb, fc, num_passes, iter, total_iters, r.total_cost_bits,
                  decision.best_cost_bits,
                  100.0 * (r.total_cost_bits / decision.best_cost_bits - 1.0),
                  decision.global_rank, decision.pass_rank);
          fflush(stderr);
          break;
        }
      }
      prev_cost = r.total_cost_bits;
    }
  }

  auto end_solve = PlannerClock::now();
  const double total_delta = result.final_cost_bits - result.init_cost_bits;
  const double total_pct = (result.init_cost_bits != 0.0)
                               ? (total_delta / result.init_cost_bits) * 100.0
                               : 0.0;
  fprintf(stderr,
          "PLANNER: [gradient] [(%u,%u,%u) P=%u] Solve %s: %u/%u iters, "
          "init=%.2f -> final=%.2f bits delta=%+.2f (%+.3f%%), "
          "fwd_total=%.2f ms adam_total=%.2f ms wall=%.2f ms\n",
          fa, fb, fc, num_passes, result.abandoned ? "abandoned" : "done",
          result.iters_taken, total_iters, result.init_cost_bits,
          result.final_cost_bits, total_delta, total_pct,
          NanosToMs(total_fwd_ns), NanosToMs(total_adam_ns),
          NanosToMs(ElapsedNanos(start_solve, end_solve)));
  fflush(stderr);
  if (!result.abandoned) {
    DumpFinalClusterLogits(d, *state, fa, fb, fc, num_passes);
  }
  return result;
}

OptimizeResult RunGradientSolve(const JPEGOptData& d,
                                const AdamConfig& adam_cfg,
                                const AnnealSchedule& schedule,
                                GradientState* state, uint32_t fa, uint32_t fb,
                                uint32_t fc, uint32_t num_passes) {
  const GradientAux aux(d);
  return RunGradientSolve(d, aux, adam_cfg, schedule, state, fa, fb, fc,
                          num_passes);
}

PassSearchResult RoundToHardAssignment(const JPEGOptData& d,
                                       const GradientState& state) {
  PassSearchResult r;
  const uint32_t num_passes = state.num_passes;
  const uint32_t num_clusters = state.num_clusters;
  const uint32_t num_cells = state.num_cells;
  const bool compact_passes = state.pass_gates.size() == num_passes;
  r.num_passes = num_passes;
  r.num_clusters = num_clusters;

  // ctx_map = argmax over cluster logits per (channel, cell).
  r.ctx_map.assign(d.channels * num_cells, 0);
  for (uint32_t c = 0; c < d.channels; ++c) {
    const auto& logits = state.cluster_logits[c];
    if (logits.size() != num_cells * num_clusters) {
      continue;
    }
    for (uint32_t cell = 0; cell < num_cells; ++cell) {
      const double* base = &logits[cell * num_clusters];
      uint32_t best = 0;
      double best_v = base[0];
      for (uint32_t k = 1; k < num_clusters; ++k) {
        if (base[k] > best_v) {
          best_v = base[k];
          best = k;
        }
      }
      r.ctx_map[c * num_cells + cell] = static_cast<uint8_t>(best);
    }
  }

  // Round thresholds from DC-index space back to actual DC values and enforce
  // strict monotonicity in the compact index domain.
  for (uint32_t a = 0; a < kNumCh; ++a) {
    const auto& soft = state.thresholds[a];
    const auto& vals = d.DC_vals[a];
    Thresholds& out = r.thresholds.T[a];
    out.clear();
    out.reserve(soft.size());
    if (soft.empty() || vals.empty()) continue;
    const int64_t max_idx = static_cast<int64_t>(vals.size() - 1);
    int64_t prev = 0;
    for (size_t j = 0; j < soft.size(); ++j) {
      const int64_t remaining = static_cast<int64_t>(soft.size() - 1 - j);
      const int64_t lower = (j == 0) ? 1 : prev + 1;
      const int64_t upper = std::max<int64_t>(lower, max_idx - remaining);
      const int64_t vi64 = std::llround(soft[j]);
      const int64_t idx = std::min<int64_t>(
          max_idx, std::max<int64_t>(lower, std::min<int64_t>(upper, vi64)));
      out.push_back(vals[idx]);
      prev = idx;
    }
  }

  // Argmax pass assignment per block. When auto-pass-count gates are present,
  // they participate in hardening and unused pass ids are compacted away after
  // all blocks have been assigned.
  for (uint32_t c = 0; c < kNumCh; ++c) {
    const uint32_t nb = (c < d.channels) ? d.num_blocks[c] : 0u;
    r.pass_assignment[c].assign(nb, 0);
    if (nb == 0 || num_passes == 0) continue;
    const auto& logits = state.pass_logits[c];
    for (uint32_t b = 0; b < nb; ++b) {
      const double* base = &logits[b * num_passes];
      uint32_t best = 0;
      double best_v = base[0] + (compact_passes ? state.pass_gates[0] : 0.0);
      for (uint32_t p = 1; p < num_passes; ++p) {
        const double v = base[p] + (compact_passes ? state.pass_gates[p] : 0.0);
        if (v > best_v) {
          best_v = v;
          best = p;
        }
      }
      r.pass_assignment[c][b] = static_cast<uint8_t>(best);
    }
  }
  if (compact_passes && num_passes > 1) {
    CompactHardPasses(&r);
  }
  CompactHardClusters(&r);
  return r;
}

// --- Iteration 6: Parallel factorization sweep --------------------------

GradientState InitGradientStateFromFactorization(
    const JPEGOptData& d, const Factorization& f, uint32_t num_passes,
    uint32_t num_clusters, double threshold_temperature,
    double pass_temperature, double cluster_temperature, uint32_t num_hists,
    bool optimize_pass_count) {
  GradientState state;
  state.num_passes = num_passes;
  state.num_clusters = num_clusters;
  state.threshold_temperature = threshold_temperature;
  state.pass_temperature = pass_temperature;
  state.cluster_temperature = cluster_temperature;

  // Per-axis thresholds via `InitThresh` (same starting point as the hard
  // search's factorization-to-candidate init), converted to DC-index space for
  // the soft optimizer.
  for (uint32_t axis = 0; axis < kNumCh; ++axis) {
    const Thresholds T = InitThresh(d, axis, f[axis]);
    state.thresholds[axis].resize(T.size());
    for (size_t i = 0; i < T.size(); ++i) {
      state.thresholds[axis][i] = DCThresholdValueToIndex(d, axis, T[i]);
    }
  }
  state.num_cells = static_cast<uint32_t>((state.thresholds[0].size() + 1) *
                                          (state.thresholds[1].size() + 1) *
                                          (state.thresholds[2].size() + 1));

  // Pass logits: per-block round-robin bias so block `b` starts biased toward
  // pass `b % num_passes`. The bias is chosen to be noticeable at temperature
  // ~1 but small enough that the optimizer can easily overcome it. Per-block
  // variation here mirrors the per-cell round-robin on cluster logits below;
  // both are needed so the soft state isn't degenerate along any axis.
  constexpr double kPassSymmetryBreak = 0.5;
  for (uint32_t c = 0; c < kNumCh; ++c) {
    const uint32_t nb = d.num_blocks[c];
    state.pass_logits[c].assign(nb * num_passes, 0.0);
    for (uint32_t b = 0; b < nb; ++b) {
      const uint32_t p = b % num_passes;
      state.pass_logits[c][b * num_passes + p] = kPassSymmetryBreak;
    }
    if (c < d.channels) {
      // Cluster logits: per-cell round-robin bias so each cell starts biased
      // toward a different cluster. Without per-cell variation `rho` is the
      // same across cells, which collapses `dL_dcell[cell] = sum_k rho[k]*D[k]`
      // to a constant, and that makes the threshold-axis gradient exactly
      // zero (the cell-difference cancels in the axis decomposition). The
      // round-robin ensures each cell sees a distinct rho profile, breaking
      // the threshold saddle without RNG plumbing.
      constexpr double kClusterSymmetryBreak = 0.5;
      state.cluster_logits[c].assign(state.num_cells * num_clusters, 0.0);
      for (uint32_t cell = 0; cell < state.num_cells; ++cell) {
        const uint32_t k = cell % num_clusters;
        state.cluster_logits[c][cell * num_clusters + k] =
            kClusterSymmetryBreak;
      }
    } else {
      state.cluster_logits[c].clear();
    }
  }
  if (optimize_pass_count && num_passes > 1) {
    // Global gates give the optimizer a low-dimensional way to retire a pass
    // for every block at once. The tiny descending bias breaks exact symmetry;
    // the per-block logits above still provide spatial variety when extra
    // passes pay for themselves.
    constexpr double kPassGateSymmetryBreak = 0.05;
    state.pass_gates.resize(num_passes);
    for (uint32_t p = 0; p < num_passes; ++p) {
      state.pass_gates[p] = -kPassGateSymmetryBreak * p;
    }
  }
  state.num_hists = num_hists;
  if (num_hists > 1) {
    constexpr double kCtxSymmetryBreak = 0.5;
    InitCtxLogitsRoundRobin(kCtxSymmetryBreak, &state);
  }
  return state;
}

namespace {

// Resolves `(min_passes, max_passes)` from `effort.optimize_passes_num`:
//   -1  -> force 1 pass (disabled)
//    0  -> sweep `[1, ComputeMaxNumPasses]`
//    K  -> force exactly K passes
std::pair<uint32_t, uint32_t> ResolvePassRange(
    const JPEGOptData& d, const JPEGCtxEffortParams& effort) {
  const uint32_t max_img = ComputeMaxNumPasses(d);
  const uint32_t min_passes =
      effort.optimize_passes_num <= 0
          ? 1
          : static_cast<uint32_t>(effort.optimize_passes_num);
  const uint32_t max_passes =
      effort.optimize_passes_num < 0
          ? 1
          : (effort.optimize_passes_num == 0
                 ? max_img
                 : std::min<uint32_t>(effort.optimize_passes_num, max_img));
  return {min_passes, std::max(min_passes, max_passes)};
}

Status RefreshHardCostWithBiclusterModel(const JPEGOptData& d,
                                         uint32_t proto_budget_per_pass,
                                         PassSearchResult* result) {
  CompactHardClusters(result);

  // The gradient search used in production currently runs with kToken420. For
  // other AC models, keep the older pass-aware evaluator as a conservative
  // fallback instead of failing an otherwise valid search path.
  if (d.AC_hist_model != JPEGTranscodeACModel::kToken420) {
    const ActiveRawBins active = BuildActiveRawBins(d);
    std::vector<uint32_t> pass_offsets;
    JXL_ASSIGN_OR_RETURN(
        std::vector<ACEntry> pass_stream,
        BuildPassStream(d, active, result->pass_assignment, result->num_passes,
                        &pass_offsets, /*pool=*/nullptr));
    JXL_ASSIGN_OR_RETURN(
        ModelEvaluation eval,
        EvaluatePassAwareModel(d, result->thresholds, result->ctx_map,
                               result->num_clusters, result->pass_assignment,
                               result->num_passes, pass_stream, pass_offsets));
    result->ac_cost = eval.corrected_entropy_cost >= 0
                          ? eval.corrected_entropy_cost
                          : eval.ac_cost;
    result->nz_cost = eval.nz_cost;
    result->signalling_overhead = eval.signalling_overhead;
    result->total_cost = eval.total_cost();
    return true;
  }

  const uint32_t proto_budget = std::max<uint32_t>(1, proto_budget_per_pass);
  const NZBlockCache nz_cache =
      BuildNZBlockCache(d, result->pass_assignment, result->num_passes);
  JXL_ASSIGN_OR_RETURN(
      RowSliceState row_state,
      BuildRowSliceState(d, result->thresholds, result->pass_assignment,
                         result->num_passes, nz_cache));
  std::vector<uint32_t> num_prototypes_per_pass(result->num_passes, 0);
  JXL_ASSIGN_OR_RETURN(
      ModelEvaluation eval,
      EvaluateBiclusterState(d, result->thresholds, result->ctx_map,
                             result->num_clusters, result->pass_assignment,
                             result->num_passes, proto_budget, row_state.rows,
                             &num_prototypes_per_pass));
  result->ac_cost = eval.corrected_entropy_cost >= 0
                        ? eval.corrected_entropy_cost
                        : eval.ac_cost;
  result->nz_cost = eval.nz_cost;
  result->signalling_overhead = eval.signalling_overhead;
  result->total_cost = eval.total_cost();
  return true;
}

StatusOr<ModelEvaluation> EvaluateHardPassResult(const JPEGOptData& d,
                                                 const ActiveRawBins& active,
                                                 PassSearchResult* result) {
  CompactHardPasses(result);
  std::vector<uint32_t> pass_offsets;
  JXL_ASSIGN_OR_RETURN(
      std::vector<ACEntry> pass_stream,
      BuildPassStream(d, active, result->pass_assignment, result->num_passes,
                      &pass_offsets, /*pool=*/nullptr));
  return EvaluatePassAwareModel(d, result->thresholds, result->ctx_map,
                                result->num_clusters, result->pass_assignment,
                                result->num_passes, pass_stream, pass_offsets);
}

StatusOr<uint32_t> ReducePassesAgglomerative(const JPEGOptData& d,
                                             PassSearchResult* result) {
  constexpr uint32_t kSourcePassCandidates = 3;
  CompactHardPasses(result);
  if (result->num_passes <= 1) return uint32_t{0};

  const ActiveRawBins active = BuildActiveRawBins(d);
  JXL_ASSIGN_OR_RETURN(ModelEvaluation current_eval,
                       EvaluateHardPassResult(d, active, result));
  FixedPointCost current_cost = current_eval.total_cost();
  uint32_t current_P = result->num_passes;
  PassAssignment working_assignment = result->pass_assignment;
  uint32_t merges = 0;

  while (current_P > 1) {
    std::vector<uint32_t> pass_blocks(current_P, 0);
    for (const auto& pass_assignment : working_assignment) {
      for (uint8_t pass : pass_assignment) ++pass_blocks[pass];
    }
    std::vector<uint32_t> source_order(current_P);
    for (uint32_t p = 0; p < current_P; ++p) source_order[p] = p;
    std::sort(source_order.begin(), source_order.end(),
              [&](uint32_t a, uint32_t b) {
                if (pass_blocks[a] != pass_blocks[b]) {
                  return pass_blocks[a] < pass_blocks[b];
                }
                return a > b;
              });

    bool found_merge = false;
    FixedPointCost best_cost = current_cost;
    ModelEvaluation best_eval = current_eval;
    PassAssignment best_assignment = working_assignment;
    uint32_t best_P = current_P;

    const uint32_t sources_to_try =
        std::min<uint32_t>(kSourcePassCandidates, current_P);
    for (uint32_t rank = 0; rank < sources_to_try; ++rank) {
      const uint32_t src = source_order[rank];
      for (uint32_t dst = 0; dst < current_P; ++dst) {
        if (src == dst) continue;

        PassSearchResult trial = *result;
        trial.pass_assignment = working_assignment;
        trial.num_passes = current_P;
        for (auto& pass_assignment : trial.pass_assignment) {
          for (uint8_t& pass : pass_assignment) {
            if (pass == src) pass = static_cast<uint8_t>(dst);
          }
        }
        CompactHardPasses(&trial);
        if (trial.num_passes >= current_P) continue;

        JXL_ASSIGN_OR_RETURN(ModelEvaluation eval,
                             EvaluateHardPassResult(d, active, &trial));
        const FixedPointCost trial_cost = eval.total_cost();
        if (trial_cost < best_cost) {
          found_merge = true;
          best_cost = trial_cost;
          best_eval = eval;
          best_assignment = std::move(trial.pass_assignment);
          best_P = trial.num_passes;
        }
      }
    }

    if (!found_merge) break;
    working_assignment = std::move(best_assignment);
    current_eval = best_eval;
    current_cost = best_cost;
    current_P = best_P;
    result->pass_assignment = working_assignment;
    result->num_passes = current_P;
    ++merges;
  }

  if (merges > 0) {
    result->pass_assignment = std::move(working_assignment);
    result->num_passes = current_P;
    result->ac_cost = current_eval.ac_cost;
    result->nz_cost = current_eval.nz_cost;
    result->signalling_overhead = current_eval.signalling_overhead;
    result->total_cost = current_eval.total_cost();
  }
  return merges;
}

}  // namespace

StatusOr<uint32_t> ReduceClustersAgglomerative(const JPEGOptData& d,
                                               PassSearchResult* result,
                                               ThreadPool* pool) {
  if (result->num_clusters <= 1) return uint32_t{0};
  const uint32_t num_cells =
      static_cast<uint32_t>((result->thresholds.TY().size() + 1) *
                            (result->thresholds.TCb().size() + 1) *
                            (result->thresholds.TCr().size() + 1));
  if (result->ctx_map.size() != d.channels * num_cells) {
    return JXL_FAILURE("ReduceClustersAgglomerative: ctx_map size mismatch");
  }
  CompactHardClusters(result);
  if (result->num_clusters <= 1) return uint32_t{0};

  // Build the pass-stream once. Pass assignment is fixed across merges, so
  // the stream stays valid for every cost evaluation we do below.
  const ActiveRawBins active = BuildActiveRawBins(d);
  std::vector<uint32_t> pass_offsets;
  JXL_ASSIGN_OR_RETURN(
      std::vector<ACEntry> pass_stream,
      BuildPassStream(d, active, result->pass_assignment, result->num_passes,
                      &pass_offsets, pool));

  // Initial cost (uses the same evaluator the encoder will).
  JXL_ASSIGN_OR_RETURN(
      ModelEvaluation current_eval,
      EvaluatePassAwareModel(d, result->thresholds, result->ctx_map,
                             result->num_clusters, result->pass_assignment,
                             result->num_passes, pass_stream, pass_offsets));
  FixedPointCost current_cost = current_eval.total_cost();
  uint32_t current_K = result->num_clusters;
  ContextMap working_ctx = result->ctx_map;

  uint32_t merges = 0;
  std::vector<uint8_t> remap(current_K, 0);
  ContextMap trial_ctx(working_ctx.size(), 0);

  while (current_K > 1) {
    int best_i = -1;
    int best_j = -1;
    FixedPointCost best_cost = current_cost;
    ModelEvaluation best_eval = current_eval;

    for (uint32_t i = 0; i + 1 < current_K; ++i) {
      for (uint32_t j = i + 1; j < current_K; ++j) {
        // Build a remap that collapses j into i and shifts higher ids down.
        for (uint32_t k = 0; k < current_K; ++k) {
          if (k == j) {
            remap[k] = static_cast<uint8_t>(i);
          } else if (k > j) {
            remap[k] = static_cast<uint8_t>(k - 1);
          } else {
            remap[k] = static_cast<uint8_t>(k);
          }
        }
        for (size_t e = 0; e < working_ctx.size(); ++e) {
          trial_ctx[e] = remap[working_ctx[e]];
        }
        const uint32_t trial_K = current_K - 1;
        JXL_ASSIGN_OR_RETURN(
            ModelEvaluation eval,
            EvaluatePassAwareModel(d, result->thresholds, trial_ctx, trial_K,
                                   result->pass_assignment, result->num_passes,
                                   pass_stream, pass_offsets));
        const FixedPointCost trial_cost = eval.total_cost();
        if (trial_cost < best_cost) {
          best_cost = trial_cost;
          best_eval = eval;
          best_i = static_cast<int>(i);
          best_j = static_cast<int>(j);
        }
      }
    }

    if (best_i < 0) break;  // no improving merge

    // Apply best merge permanently.
    for (uint32_t k = 0; k < current_K; ++k) {
      if (k == static_cast<uint32_t>(best_j)) {
        remap[k] = static_cast<uint8_t>(best_i);
      } else if (k > static_cast<uint32_t>(best_j)) {
        remap[k] = static_cast<uint8_t>(k - 1);
      } else {
        remap[k] = static_cast<uint8_t>(k);
      }
    }
    for (size_t e = 0; e < working_ctx.size(); ++e) {
      working_ctx[e] = remap[working_ctx[e]];
    }
    --current_K;
    remap.resize(current_K);
    current_cost = best_cost;
    current_eval = best_eval;
    ++merges;
  }

  if (merges > 0) {
    result->ctx_map = std::move(working_ctx);
    result->num_clusters = current_K;
    result->ac_cost = current_eval.ac_cost;
    result->nz_cost = current_eval.nz_cost;
    result->signalling_overhead = current_eval.signalling_overhead;
    result->total_cost = current_eval.total_cost();
  }
  return merges;
}

uint32_t PruneRedundantThresholds(const JPEGOptData& d,
                                  PassSearchResult* result) {
  const uint32_t channels = d.channels;
  uint32_t pruned_total = 0;

  // Helper: produce current per-axis bucket count `n[a] = T[a].size() + 1`.
  auto compute_n = [&]() {
    std::array<uint32_t, kNumCh> n{};
    for (uint32_t a = 0; a < kNumCh; ++a) {
      n[a] = static_cast<uint32_t>(result->thresholds.T[a].size()) + 1;
    }
    return n;
  };

  for (uint32_t axis = 0; axis < kNumCh; ++axis) {
    Thresholds& T = result->thresholds.T[axis];

    // Walk thresholds high-to-low so dropping one doesn't shift the indices we
    // still need to examine.
    for (int j = static_cast<int>(T.size()) - 1; j >= 0; --j) {
      const auto n = compute_n();
      const uint32_t num_cells = n[0] * n[1] * n[2];

      auto cell_at = [&](uint32_t k0, uint32_t k1, uint32_t k2) {
        return (k1 * n[2] + k2) * n[0] + k0;
      };

      // Threshold T[axis][j] separates bucket j and bucket j+1 on this axis.
      // It's redundant iff every (channel, perpendicular cell) sees the same
      // cluster id at bucket j and bucket j+1.
      bool redundant = true;
      for (uint32_t c = 0; c < channels && redundant; ++c) {
        if (axis == 0) {
          for (uint32_t k1 = 0; k1 < n[1] && redundant; ++k1) {
            for (uint32_t k2 = 0; k2 < n[2] && redundant; ++k2) {
              const uint8_t a_id =
                  result->ctx_map[c * num_cells +
                                  cell_at(static_cast<uint32_t>(j), k1, k2)];
              const uint8_t b_id =
                  result->ctx_map[c * num_cells + cell_at(j + 1, k1, k2)];
              if (a_id != b_id) redundant = false;
            }
          }
        } else if (axis == 1) {
          for (uint32_t k0 = 0; k0 < n[0] && redundant; ++k0) {
            for (uint32_t k2 = 0; k2 < n[2] && redundant; ++k2) {
              const uint8_t a_id =
                  result->ctx_map[c * num_cells +
                                  cell_at(k0, static_cast<uint32_t>(j), k2)];
              const uint8_t b_id =
                  result->ctx_map[c * num_cells + cell_at(k0, j + 1, k2)];
              if (a_id != b_id) redundant = false;
            }
          }
        } else {  // axis == 2
          for (uint32_t k0 = 0; k0 < n[0] && redundant; ++k0) {
            for (uint32_t k1 = 0; k1 < n[1] && redundant; ++k1) {
              const uint8_t a_id =
                  result->ctx_map[c * num_cells +
                                  cell_at(k0, k1, static_cast<uint32_t>(j))];
              const uint8_t b_id =
                  result->ctx_map[c * num_cells + cell_at(k0, k1, j + 1)];
              if (a_id != b_id) redundant = false;
            }
          }
        }
      }
      if (!redundant) continue;

      // Drop threshold j on `axis`. Bucket j and j+1 collapse into bucket j;
      // higher buckets shift down by one. The map from new bucket back to a
      // representative old bucket is `k_new <= j ? k_new : k_new + 1`.
      auto new_n = n;
      new_n[axis] -= 1;
      const uint32_t new_num_cells = new_n[0] * new_n[1] * new_n[2];
      ContextMap new_ctx(channels * new_num_cells);

      auto new_cell_at = [&](uint32_t k0, uint32_t k1, uint32_t k2) {
        return (k1 * new_n[2] + k2) * new_n[0] + k0;
      };
      auto remap = [j](uint32_t k_new) -> uint32_t {
        return k_new <= static_cast<uint32_t>(j) ? k_new : k_new + 1;
      };
      for (uint32_t c = 0; c < channels; ++c) {
        for (uint32_t k0 = 0; k0 < new_n[0]; ++k0) {
          for (uint32_t k1 = 0; k1 < new_n[1]; ++k1) {
            for (uint32_t k2 = 0; k2 < new_n[2]; ++k2) {
              const uint32_t k0_old = (axis == 0) ? remap(k0) : k0;
              const uint32_t k1_old = (axis == 1) ? remap(k1) : k1;
              const uint32_t k2_old = (axis == 2) ? remap(k2) : k2;
              new_ctx[c * new_num_cells + new_cell_at(k0, k1, k2)] =
                  result->ctx_map[c * num_cells +
                                  cell_at(k0_old, k1_old, k2_old)];
            }
          }
        }
      }
      result->ctx_map = std::move(new_ctx);
      T.erase(T.begin() + j);
      ++pruned_total;
    }
  }

  CompactHardClusters(result);
  return pruned_total;
}

StatusOr<PassSearchResult> SearchGradientContextModel(
    std::shared_ptr<const JPEGOptData> opt_data,
    const JPEGCtxEffortParams& effort, ThreadPool* pool,
    std::vector<GradientSearchCandidate>* debug_candidates) {
  auto start_total = PlannerClock::now();

  const JPEGOptData& d = *opt_data;
  const auto factorizations = MaximalFactorizations(d);
  if (factorizations.empty()) {
    return JXL_FAILURE("Gradient-joint search: no maximal factorizations");
  }

  const uint32_t num_clusters =
      kMaxClusters - static_cast<uint32_t>(d.channels == 1);
  const auto pass_range = ResolvePassRange(d, effort);
  const uint32_t min_passes = pass_range.first;
  const uint32_t max_passes = pass_range.second;
  const uint32_t pass_count_steps = max_passes - min_passes + 1;
  const uint32_t num_factorizations =
      static_cast<uint32_t>(factorizations.size());
  const bool optimize_pass_count =
      effort.optimize_passes_num == 0 && max_passes > 1;
  const uint32_t total_workers = optimize_pass_count
                                     ? num_factorizations
                                     : num_factorizations * pass_count_steps;

  fprintf(stderr,
          "PLANNER: [gradient] %u factorizations, pass range [%u, %u] "
          "(%u workers%s), %u clusters\n",
          num_factorizations, min_passes, max_passes, total_workers,
          optimize_pass_count ? ", auto-P via one Pmax solve" : "",
          num_clusters);
  fflush(stderr);

  AdamConfig adam_cfg;
  adam_cfg.lr = effort.grad_lr;

  AnnealSchedule sched;
  sched.hot_iters = effort.grad_hot_iters;
  sched.anneal_iters = effort.grad_anneal_iters;
  sched.pass_init = effort.grad_init_temperature;
  sched.pass_final = effort.grad_init_temperature * 0.05;
  sched.threshold_init = effort.grad_init_temperature * 50.0;
  sched.threshold_final = effort.grad_init_temperature * 0.5;
  sched.cluster_init = effort.grad_init_temperature;
  sched.cluster_final = effort.grad_init_temperature * 0.05;
  sched.ctx_init = effort.grad_init_temperature;
  sched.ctx_final = effort.grad_init_temperature * 0.05;

  const uint32_t num_hists = effort.grad_num_hists;
  const GradientAux aux(d);

  fprintf(stderr,
          "PLANNER: [gradient] Schedule: hot=%u anneal=%u total_iters=%u "
          "lr=%.4f T_pass=%.4f->%.4f T_thresh=%.4f->%.4f "
          "T_cluster=%.4f->%.4f T_ctx=%.4f->%.4f num_hists=%u\n",
          sched.hot_iters, sched.anneal_iters,
          sched.hot_iters + sched.anneal_iters, adam_cfg.lr, sched.pass_init,
          sched.pass_final, sched.threshold_init, sched.threshold_final,
          sched.cluster_init, sched.cluster_final, sched.ctx_init,
          sched.ctx_final, num_hists);
  fflush(stderr);

  // Flat work list. In fixed-P mode it still enumerates `(num_passes,
  // factorization_idx)` tuples. In auto-P mode each factorization runs once at
  // Pmax with global pass gates and smooth occupancy overhead; hard rounding
  // compacts unused pass ids away.
  struct Slot {
    PassSearchResult result;
    double final_cost_bits = std::numeric_limits<double>::max();
    uint32_t factorization_idx = 0;
    uint32_t num_passes = 0;
    bool abandoned = false;
    bool valid = false;
  };
  std::vector<Slot> slots(total_workers);
  GradientRaceState race(total_workers);

  auto start_sweep = PlannerClock::now();
  JXL_RETURN_IF_ERROR(RunOnPool(
      pool, 0, total_workers, ThreadPool::NoInit,
      [&](uint32_t idx, size_t /*thread_id*/) -> Status {
        const uint32_t num_passes = optimize_pass_count
                                        ? max_passes
                                        : min_passes + idx / num_factorizations;
        const uint32_t factorization_idx =
            optimize_pass_count ? idx : idx % num_factorizations;
        const Factorization& f = factorizations[factorization_idx];
        GradientState state = InitGradientStateFromFactorization(
            d, f, num_passes, num_clusters, sched.threshold_init,
            sched.pass_init, sched.cluster_init, num_hists,
            optimize_pass_count);
        const OptimizeResult opt =
            RunGradientSolve(d, aux, adam_cfg, sched, &state, f[0], f[1], f[2],
                             num_passes, &race, idx);
        slots[idx].factorization_idx = factorization_idx;
        slots[idx].num_passes = num_passes;
        if (opt.abandoned) {
          slots[idx].abandoned = true;
          return true;
        }
        slots[idx].result = RoundToHardAssignment(d, state);
        slots[idx].num_passes = slots[idx].result.num_passes;
        slots[idx].valid = true;
        if (optimize_pass_count) {
          fprintf(stderr,
                  "PLANNER: [gradient] [(%u,%u,%u) Pmax=%u] Rounded pass "
                  "count: %u\n",
                  f[0], f[1], f[2], num_passes, slots[idx].num_passes);
          fflush(stderr);
        }

        // Post-hoc agglomerative cluster reduction (Option C). The smooth
        // auto-P term only helps with pass occupancy; cluster signalling is
        // still piecewise enough that the soft optimizer can over-allocate
        // clusters. This pass greedily merges cluster pairs whose merge reduces
        // the encoder's actual cost and updates the slot's cost fields.
        if (effort.grad_overhead_aware_reduce) {
          auto merges_or = ReduceClustersAgglomerative(d, &slots[idx].result,
                                                       /*pool=*/nullptr);
          if (merges_or.ok()) {
            const uint32_t merges = std::move(merges_or).value_();
            if (merges > 0) {
              // Use the EvaluatePassAwareModel-derived total cost for the
              // pick-best comparison; soft cost is now stale.
              slots[idx].final_cost_bits =
                  static_cast<double>(slots[idx].result.total_cost) /
                  static_cast<double>(kFScale);
              fprintf(stderr,
                      "PLANNER: [gradient] [(%u,%u,%u) P=%u] "
                      "Agglomerative merge: %u clusters dropped, "
                      "final_cost=%.2f bits\n",
                      f[0], f[1], f[2], slots[idx].result.num_passes, merges,
                      slots[idx].final_cost_bits);
              fflush(stderr);
            }
          }
        }
        // Prune thresholds whose adjacent buckets all share the same
        // cluster (post-merge, often most of them). This shrinks the
        // bitstream factorization metadata and ctx_map size without
        // changing per-block cluster assignment other than dense
        // renumbering, so EvaluatePassAwareModel cost is unchanged but the
        // actual bitstream is smaller.
        const uint32_t pruned = PruneRedundantThresholds(d, &slots[idx].result);
        if (pruned > 0) {
          const auto& T0 = slots[idx].result.thresholds.TY();
          const auto& T1 = slots[idx].result.thresholds.TCb();
          const auto& T2 = slots[idx].result.thresholds.TCr();
          fprintf(stderr,
                  "PLANNER: [gradient] [(%u,%u,%u) P=%u] "
                  "Threshold pruning: %u redundant thresholds dropped, "
                  "factorization now (%zu,%zu,%zu)\n",
                  f[0], f[1], f[2], slots[idx].result.num_passes, pruned,
                  T0.size() + 1, T1.size() + 1, T2.size() + 1);
          fflush(stderr);
        }
        if (optimize_pass_count) {
          auto pass_merges_or = ReducePassesAgglomerative(d, &slots[idx].result);
          if (pass_merges_or.ok()) {
            const uint32_t pass_merges = std::move(pass_merges_or).value_();
            if (pass_merges > 0) {
              slots[idx].num_passes = slots[idx].result.num_passes;
              slots[idx].final_cost_bits =
                  static_cast<double>(slots[idx].result.total_cost) /
                  static_cast<double>(kFScale);
              fprintf(stderr,
                      "PLANNER: [gradient] [(%u,%u,%u) P=%u] "
                      "Agglomerative pass reduction: %u passes dropped, "
                      "final_cost=%.2f bits\n",
                      f[0], f[1], f[2], slots[idx].result.num_passes,
                      pass_merges, slots[idx].final_cost_bits);
              fflush(stderr);
            }
          }
        }
        JXL_RETURN_IF_ERROR(RefreshHardCostWithBiclusterModel(
            d, effort.bicluster_proto_budget_per_pass, &slots[idx].result));
        slots[idx].final_cost_bits = bit_cost(slots[idx].result.total_cost);
        slots[idx].num_passes = slots[idx].result.num_passes;
        if (optimize_pass_count) {
          fprintf(stderr,
                  "PLANNER: [gradient] [(%u,%u,%u) Pmax=%u] Final pass "
                  "count: %u\n",
                  f[0], f[1], f[2], num_passes, slots[idx].num_passes);
          fflush(stderr);
        }
        return true;
      },
      "JpegCtxGradSweep"));
  auto end_sweep = PlannerClock::now();
  const uint32_t abandoned_count = race.abandoned_count();

  // Pick the best. Deterministic tie-break: smaller flat index wins (which
  // corresponds to smaller num_passes first, then smaller factorization_idx).
  size_t best_idx = total_workers;
  double best_cost = std::numeric_limits<double>::max();
  for (size_t i = 0; i < slots.size(); ++i) {
    if (!slots[i].valid || slots[i].abandoned) continue;
    if (slots[i].final_cost_bits < best_cost) {
      best_cost = slots[i].final_cost_bits;
      best_idx = i;
    }
  }
  if (best_idx >= slots.size()) {
    return JXL_FAILURE("Gradient-joint search: no factorization succeeded");
  }

  if (debug_candidates != nullptr) {
    debug_candidates->clear();
    debug_candidates->reserve(slots.size());
    for (size_t i = 0; i < slots.size(); ++i) {
      if (!slots[i].valid || slots[i].abandoned) continue;
      const Factorization& sf = factorizations[slots[i].factorization_idx];
      GradientSearchCandidate candidate;
      candidate.result = slots[i].result;
      candidate.target_cost_bits = slots[i].final_cost_bits;
      candidate.factorization[0] = sf[0];
      candidate.factorization[1] = sf[1];
      candidate.factorization[2] = sf[2];
      candidate.num_passes = slots[i].num_passes;
      candidate.is_best = i == best_idx;
      debug_candidates->push_back(std::move(candidate));
    }
  }

  // Report per-slot results.
  for (size_t i = 0; i < slots.size(); ++i) {
    const Factorization& sf = factorizations[slots[i].factorization_idx];
    if (slots[i].abandoned) {
      fprintf(stderr,
              "PLANNER: [gradient] [(%u,%u,%u) P=%u] abandoned by race\n",
              sf[0], sf[1], sf[2], slots[i].num_passes);
      continue;
    }
    if (!slots[i].valid) continue;
    fprintf(stderr, "PLANNER: [gradient] [(%u,%u,%u) P=%u] cost=%.4f bits%s\n",
            sf[0], sf[1], sf[2], slots[i].num_passes, slots[i].final_cost_bits,
            i == best_idx ? " ** BEST **" : "");
  }
  fflush(stderr);

  const Slot& best = slots[best_idx];
  const Factorization& bf = factorizations[best.factorization_idx];
  fprintf(stderr,
          "PLANNER: [gradient] Sweep done: %u workers (%u abandoned) in "
          "%.2f ms, "
          "best=[(%u,%u,%u) P=%u] cost=%.4f bits "
          "(ac=%.2f nz=%.2f overhead=%.2f)\n",
          total_workers, abandoned_count,
          NanosToMs(ElapsedNanos(start_sweep, end_sweep)), bf[0], bf[1], bf[2],
          best.num_passes, best.final_cost_bits, bit_cost(best.result.ac_cost),
          bit_cost(best.result.nz_cost),
          bit_cost(best.result.signalling_overhead));
  auto end_total = PlannerClock::now();
  fprintf(stderr, "PLANNER: [gradient] Total search took %.2f ms\n",
          NanosToMs(ElapsedNanos(start_total, end_total)));
  fflush(stderr);

  return std::move(slots[best_idx].result);
}

}  // namespace jxl

#endif  // HWY_ONCE
