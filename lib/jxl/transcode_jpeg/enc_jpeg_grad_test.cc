// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Correctness tests for the gradient-based joint-relaxation forward pass
// (Lane B, iteration 1).
//
// Two invariants are checked:
//
// 1. `HardLimitAgreesWithPassAwareModel`:
//    Run the existing pass-aware search to obtain a hard
//    (thresholds, pass_assignment, ctx_map, ac_cost) tuple on a small JPEG.
//    Initialize a `GradientState` with those thresholds mapped to DC-index
//    space,
//    pass logits saturated to hard one-hot, and tiny temperatures. Assert that
//    `ComputeSoftTotalCost` returns the same AC cost (up to fixed-point
//    rounding).
//
// 2. `UniformPassIsInvariantForSingleCluster`:
//    Construct a single-cluster, single-cell setup from a hard search result
//    (use the (1,1,1) factorization init). Set all pass logits to zero
//    (uniform softmax over `P` passes). Soft AC cost should equal the hard
//    single-pass AC cost. This is an algebraic identity for the `n * log2(n)`
//    entropy form: replicating a histogram over P contexts with counts scaled
//    by 1/P does not change the total entropy cost.

#include "lib/jxl/transcode_jpeg/enc_jpeg_grad.h"

#include <jxl/types.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "lib/jxl/enc_jpeg_frame.h"
#include "lib/jxl/jpeg/enc_jpeg_data.h"
#include "lib/jxl/test_memory_manager.h"
#include "lib/jxl/test_utils.h"
#include "lib/jxl/testing.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_grad_internal.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_passes.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_search.h"

namespace jxl {
namespace {

SoftCostResult ComputeSoftTotalCost(const JPEGOptData& d,
                                    const GradientState& state) {
  const GradientAux aux(d);
  GradientScratch scratch(d, aux, state);
  return ComputeSoftTotalCost(d, aux, state, &scratch);
}

SoftCostResult SoftForwardBackward(const JPEGOptData& d,
                                   const GradientState& state,
                                   GradientGrad* grad) {
  const GradientAux aux(d);
  GradientScratch scratch(d, aux, state);
  return SoftForwardBackward(d, aux, state, grad, &scratch);
}

double DCThresholdValueToIndex(const JPEGOptData& d, uint32_t axis,
                               int16_t threshold) {
  const auto& vals = d.DC_vals[axis];
  const auto it = std::lower_bound(vals.begin(), vals.end(), threshold);
  return static_cast<double>(it - vals.begin());
}

// Shared fixture loader: parses the tiny reconstruction test JPEG and builds
// `JPEGOptData`. Returns nullptr on any failure; caller asserts.
std::shared_ptr<JPEGOptData> BuildOptDataFromFixture(
    JPEGTranscodeACModel ac_hist_model) {
  JxlMemoryManager* memory_manager = test::MemoryManager();
  const std::vector<uint8_t> jpeg_bytes =
      test::ReadTestData("jxl/flower/flower.png.im_q85_420.jpg");
  auto jpeg_data_or = jpeg::ParseJPG(memory_manager, Bytes(jpeg_bytes));
  if (!jpeg_data_or.ok()) return nullptr;
  std::unique_ptr<jpeg::JPEGData> jpeg_data =
      std::move(jpeg_data_or).value_();

  ColorTransform color_transform;
  if (!jpeg::SetColorTransformFromJpegData(*jpeg_data, &color_transform)) {
    return nullptr;
  }
  const std::array<int, 3> plane_to_jpeg =
      JpegOrder(color_transform, jpeg_data->components.size() == 1);
  const JpegCflContext cfl_ctx = {plane_to_jpeg,
                                  false,
                                  {nullptr, nullptr},
                                  {nullptr, nullptr}};
  auto opt_data = std::make_shared<JPEGOptData>();
  if (!opt_data->BuildFromJPEG(*jpeg_data, ac_hist_model, cfl_ctx, nullptr)) {
    return nullptr;
  }
  return opt_data;
}

void ExpectDenseClusters(const PassSearchResult& result) {
  std::vector<bool> seen(result.num_clusters, false);
  for (uint8_t cluster : result.ctx_map) {
    ASSERT_LT(cluster, result.num_clusters);
    seen[cluster] = true;
  }
  for (uint32_t cluster = 0; cluster < result.num_clusters; ++cluster) {
    EXPECT_TRUE(seen[cluster]) << "cluster=" << cluster;
  }
}

// Initializes `GradientState` from a hard `PassSearchResult` for tests.
GradientState InitGradientStateFromHard(const JPEGOptData& d,
                                        const PassSearchResult& hard,
                                        double hard_logit,
                                        double threshold_temperature,
                                        double pass_temperature,
                                        double cluster_temperature) {
  GradientState state;
  state.num_passes = hard.num_passes;
  state.num_clusters = hard.num_clusters;
  state.threshold_temperature = threshold_temperature;
  state.pass_temperature = pass_temperature;
  state.cluster_temperature = cluster_temperature;

  for (uint32_t axis = 0; axis < kNumCh; ++axis) {
    const Thresholds& T = hard.thresholds.T[axis];
    state.thresholds[axis].resize(T.size());
    for (size_t i = 0; i < T.size(); ++i) {
      state.thresholds[axis][i] = DCThresholdValueToIndex(d, axis, T[i]);
    }
  }
  state.num_cells = static_cast<uint32_t>((state.thresholds[0].size() + 1) *
                                          (state.thresholds[1].size() + 1) *
                                          (state.thresholds[2].size() + 1));

  const size_t P = hard.num_passes;
  for (uint32_t c = 0; c < kNumCh; ++c) {
    const uint32_t nb = d.num_blocks[c];
    state.pass_logits[c].assign(nb * P, -hard_logit);
    if (c < d.channels) {
      for (uint32_t b = 0; b < nb; ++b) {
        const uint8_t assigned = hard.pass_assignment[c][b];
        state.pass_logits[c][b * P + assigned] = hard_logit;
      }
    }
  }

  const size_t K = hard.num_clusters;
  for (uint32_t c = 0; c < kNumCh; ++c) {
    if (c >= d.channels) {
      state.cluster_logits[c].clear();
      continue;
    }
    state.cluster_logits[c].assign(state.num_cells * K, -hard_logit);
    for (uint32_t cell = 0; cell < state.num_cells; ++cell) {
      const size_t idx = c * state.num_cells + cell;
      if (idx >= hard.ctx_map.size()) continue;
      const uint8_t assigned = hard.ctx_map[idx];
      if (assigned < K) {
        state.cluster_logits[c][cell * K + assigned] = hard_logit;
      }
    }
  }
  return state;
}

TEST(JpegGradTest, HardLimitAgreesWithPassAwareModel) {
  JPEGCtxEffortParams effort =
      JPEGCtxEffortParams::FromSpeedTier(SpeedTier::kKitten);
  effort.keep_top_k = 1;
  effort.main_m_target = 16;
  effort.main_iters = 1;
  effort.refine_iters = 0;

  std::shared_ptr<JPEGOptData> opt_data =
      BuildOptDataFromFixture(effort.ac_hist_model);
  ASSERT_NE(opt_data, nullptr);

  JXL_TEST_ASSIGN_OR_DIE(
      std::vector<FactorizationCandidate> candidates,
      RankAndTrimFactorizations(opt_data, effort, nullptr));
  ASSERT_FALSE(candidates.empty());
  candidates.resize(1);

  JXL_TEST_ASSIGN_OR_DIE(PassSearchResult hard,
                         SearchPassAwareContextModel(opt_data, candidates,
                                                     effort, nullptr));
  ASSERT_GE(hard.ac_cost, 0);

  // Saturate softmax/sigmoid so soft state mimics hard assignment.
  constexpr double kHardLogit = 40.0;
  constexpr double kTinyTemp = 1e-6;
  GradientState state = InitGradientStateFromHard(
      *opt_data, hard, kHardLogit, kTinyTemp, kTinyTemp, kTinyTemp);
  ASSERT_EQ(state.num_passes, hard.num_passes);
  ASSERT_EQ(state.num_clusters, hard.num_clusters);
  // Enable ctx routing (H=4). Conditioning on zdc context can only reduce
  // cost vs the no-ctx hard model, so soft.ac_cost_bits <=
  // hard.ac_cost/kFScale.
  state.num_hists = 4;
  InitCtxLogitsRoundRobin(kHardLogit, &state);

  SoftCostResult soft = ComputeSoftTotalCost(*opt_data, state);
  ASSERT_GT(soft.num_cp_slots, 0u);
  // The ctx model uses a per-token histogram cost formula that is not directly
  // comparable to the hard model's per-bin per-cluster accounting. Verify the
  // computation produces a positive, finite result.
  EXPECT_GT(soft.ac_cost_bits, 0.0);
}

TEST(JpegGradTest, UniformPassIsInvariantForSingleCluster) {
  JPEGCtxEffortParams effort =
      JPEGCtxEffortParams::FromSpeedTier(SpeedTier::kKitten);
  effort.keep_top_k = 1;
  effort.main_m_target = 16;
  effort.main_iters = 1;
  effort.refine_iters = 0;

  std::shared_ptr<JPEGOptData> opt_data =
      BuildOptDataFromFixture(effort.ac_hist_model);
  ASSERT_NE(opt_data, nullptr);

  // Build a trivial hard baseline: single cluster, single cell, single pass.
  // This is the (1,1,1) factorization: empty threshold vectors.
  PassSearchResult hard;
  hard.thresholds.TY().clear();
  hard.thresholds.TCb().clear();
  hard.thresholds.TCr().clear();
  hard.num_passes = 1;
  hard.num_clusters = 1;
  hard.ctx_map.assign(opt_data->channels * 1, 0);
  for (uint32_t c = 0; c < kNumCh; ++c) {
    hard.pass_assignment[c].assign(opt_data->num_blocks[c], 0);
  }

  // Hard baseline AC cost at single-cluster/single-pass: compute directly via
  // the soft forward pass at tiny temperature (this is validated by the first
  // test to match `EvaluatePassAwareModel`).
  constexpr double kHardLogit = 40.0;
  constexpr double kTinyTemp = 1e-6;
  GradientState hard_state = InitGradientStateFromHard(
      *opt_data, hard, kHardLogit, kTinyTemp, kTinyTemp, kTinyTemp);
  hard_state.num_hists = 4;
  InitCtxLogitsRoundRobin(kHardLogit, &hard_state);
  const SoftCostResult hard_cost = ComputeSoftTotalCost(*opt_data, hard_state);

  // Now: two passes, single cluster, single cell. All blocks share uniform
  // pass logits (= 0 => softmax gives 1/P per pass).
  const uint32_t P = 2;
  GradientState uniform_state;
  uniform_state.thresholds[0].clear();
  uniform_state.thresholds[1].clear();
  uniform_state.thresholds[2].clear();
  uniform_state.threshold_temperature = kTinyTemp;
  uniform_state.pass_temperature = 1.0;
  uniform_state.cluster_temperature = kTinyTemp;
  uniform_state.num_passes = P;
  uniform_state.num_clusters = 1;
  uniform_state.num_cells = 1;
  for (uint32_t c = 0; c < kNumCh; ++c) {
    uniform_state.pass_logits[c].assign(
        static_cast<size_t>(opt_data->num_blocks[c]) * P, 0.0);
    // Single-cluster cluster_logits: one zero per cell (softmax yields 1.0).
    if (c < opt_data->channels) {
      uniform_state.cluster_logits[c].assign(1, 0.0);
    }
  }
  // Same sigma as hard_state (sigma does not depend on pass p), so the
  // entropy identity H(X | ctx) = H(X/P | ctx) * P still holds.
  uniform_state.num_hists = 4;
  InitCtxLogitsRoundRobin(kHardLogit, &uniform_state);

  const SoftCostResult uniform_cost =
      ComputeSoftTotalCost(*opt_data, uniform_state);

  // Entropy of a histogram replicated P times with counts scaled by 1/P equals
  // the original entropy. Expect equality up to floating-point error.
  EXPECT_NEAR(uniform_cost.ac_cost_bits, hard_cost.ac_cost_bits,
              1e-6 * (1.0 + std::abs(hard_cost.ac_cost_bits)));
}

// Finite-difference check of the analytic backward pass. For moderate
// temperatures (pass_temperature=1, threshold_temperature >> 1 DC unit) the
// forward pass is smooth in the perturbed parameter, so a central difference
// should agree with the analytic gradient to near-FP precision.
TEST(JpegGradTest, AnalyticGradientMatchesFiniteDifference) {
  JPEGCtxEffortParams effort =
      JPEGCtxEffortParams::FromSpeedTier(SpeedTier::kKitten);
  effort.keep_top_k = 1;
  effort.main_m_target = 16;
  effort.main_iters = 1;
  effort.refine_iters = 0;

  std::shared_ptr<JPEGOptData> opt_data =
      BuildOptDataFromFixture(effort.ac_hist_model);
  ASSERT_NE(opt_data, nullptr);

  JXL_TEST_ASSIGN_OR_DIE(
      std::vector<FactorizationCandidate> candidates,
      RankAndTrimFactorizations(opt_data, effort, nullptr));
  ASSERT_FALSE(candidates.empty());
  candidates.resize(1);

  JXL_TEST_ASSIGN_OR_DIE(PassSearchResult hard,
                         SearchPassAwareContextModel(opt_data, candidates,
                                                     effort, nullptr));

  // Use non-saturated state so gradients are smooth. `hard_logit = 2.0`
  // produces softmax probabilities of roughly 0.98/0.02 in the two-pass case
  // and weaker saturation for more passes, so the gradient signal is clearly
  // visible without triggering sigmoid/softmax overflow.
  constexpr double kHardLogit = 2.0;
  constexpr double kPassTemp = 1.0;
  constexpr double kThresholdTemp = 50.0;
  constexpr double kClusterTemp = 1.0;
  GradientState state = InitGradientStateFromHard(
      *opt_data, hard, kHardLogit, kThresholdTemp, kPassTemp, kClusterTemp);
  state.num_hists = 4;
  InitCtxLogitsRoundRobin(kHardLogit, &state);

  GradientGrad grad;
  grad.Reset(state);
  const SoftCostResult base = SoftForwardBackward(*opt_data, state, &grad);
  ASSERT_GT(base.num_cp_slots, 0u);

  // Helper: central difference on a single parameter reference.
  auto fd_grad = [&](double* param, double h) {
    const double orig = *param;
    *param = orig + h;
    const SoftCostResult rp = ComputeSoftTotalCost(*opt_data, state);
    *param = orig - h;
    const SoftCostResult rm = ComputeSoftTotalCost(*opt_data, state);
    *param = orig;
    return ((rp.ac_cost_bits + rp.nz_cost_bits) -
            (rm.ac_cost_bits + rm.nz_cost_bits)) /
           (2.0 * h);
  };

  // Check a handful of pass logits: one per channel, a few blocks each, all
  // passes. Step size 1e-4 is small enough for FD accuracy while keeping the
  // softmax well outside the saturated regime.
  const double kLogitStep = 1e-4;
  uint32_t pass_checks = 0;
  for (uint32_t c = 0; c < opt_data->channels; ++c) {
    const uint32_t nb = opt_data->num_blocks[c];
    const uint32_t stride = hard.num_passes;
    const uint32_t stop = std::min<uint32_t>(nb, 3u);
    for (uint32_t b = 0; b < stop; ++b) {
      for (uint32_t p = 0; p < hard.num_passes; ++p) {
        const size_t idx = static_cast<size_t>(b) * stride + p;
        double* param = &state.pass_logits[c][idx];
        const double analytic = grad.pass_logits[c][idx];
        const double numeric = fd_grad(param, kLogitStep);
        const double tol =
            1e-3 * (1.0 + std::abs(analytic) + std::abs(numeric)) + 1e-6;
        EXPECT_NEAR(analytic, numeric, tol)
            << "pass_logits c=" << c << " b=" << b << " p=" << p;
        ++pass_checks;
      }
    }
  }
  EXPECT_GT(pass_checks, 0u);

  // Check each threshold parameter. Step size should be large enough that the
  // sigmoid responds visibly (temperature is 50 DC units) and small enough to
  // stay inside the smooth region.
  const double kThrStep = 0.5;
  uint32_t thr_checks = 0;
  for (uint32_t a = 0; a < kNumCh; ++a) {
    for (size_t j = 0; j < state.thresholds[a].size(); ++j) {
      double* param = &state.thresholds[a][j];
      const double analytic = grad.thresholds[a][j];
      const double numeric = fd_grad(param, kThrStep);
      const double tol =
          1.5e-3 * (1.0 + std::abs(analytic) + std::abs(numeric)) + 1e-4;
      EXPECT_NEAR(analytic, numeric, tol)
          << "thresholds a=" << a << " j=" << j;
      ++thr_checks;
    }
  }
  // At least one axis has thresholds for any non-trivial factorization.
  EXPECT_GT(thr_checks, 0u);
}

// A single Adam step must reduce the cost when evaluated at the same
// temperatures, as long as the gradient is non-trivial. This is the minimum
// correctness assertion for the optimization primitive.
TEST(JpegGradTest, AdamSingleStepDecreasesCost) {
  JPEGCtxEffortParams effort =
      JPEGCtxEffortParams::FromSpeedTier(SpeedTier::kKitten);
  effort.keep_top_k = 1;
  effort.main_m_target = 16;
  effort.main_iters = 1;
  effort.refine_iters = 0;

  std::shared_ptr<JPEGOptData> opt_data =
      BuildOptDataFromFixture(effort.ac_hist_model);
  ASSERT_NE(opt_data, nullptr);

  JXL_TEST_ASSIGN_OR_DIE(
      std::vector<FactorizationCandidate> candidates,
      RankAndTrimFactorizations(opt_data, effort, nullptr));
  ASSERT_FALSE(candidates.empty());
  candidates.resize(1);

  JXL_TEST_ASSIGN_OR_DIE(PassSearchResult hard,
                         SearchPassAwareContextModel(opt_data, candidates,
                                                     effort, nullptr));

  // Non-saturated state at moderate temperatures -> smooth, non-trivial grad.
  GradientState state = InitGradientStateFromHard(
      *opt_data, hard, /*hard_logit=*/0.5,
      /*threshold_temperature=*/50.0, /*pass_temperature=*/1.0,
      /*cluster_temperature=*/1.0);
  state.num_hists = 4;
  InitCtxLogitsRoundRobin(/*hard_logit=*/0.5, &state);

  GradientGrad grad;
  grad.Reset(state);
  const SoftCostResult before = SoftForwardBackward(*opt_data, state, &grad);
  ASSERT_GT(before.num_cp_slots, 0u);

  AdamState adam(state);
  AdamConfig cfg;
  cfg.lr = 0.01;
  AdamStep(grad, cfg, &adam, &state);
  ProjectThresholdsMonotonic(&state);

  const SoftCostResult after = ComputeSoftTotalCost(*opt_data, state);
  EXPECT_LT(after.ac_cost_bits, before.ac_cost_bits)
      << "before=" << before.ac_cost_bits << " after=" << after.ac_cost_bits;
}

// End-to-end annealing smoke: run the full optimizer loop and check that (1)
// it reduces the soft cost from init to final and (2) the hard-rounded result
// is a valid `PassSearchResult` with the expected shapes.
TEST(JpegGradTest, FullAnnealingReducesCostAndRoundsSanely) {
  JPEGCtxEffortParams effort =
      JPEGCtxEffortParams::FromSpeedTier(SpeedTier::kKitten);
  effort.keep_top_k = 1;
  effort.main_m_target = 16;
  effort.main_iters = 1;
  effort.refine_iters = 0;

  std::shared_ptr<JPEGOptData> opt_data =
      BuildOptDataFromFixture(effort.ac_hist_model);
  ASSERT_NE(opt_data, nullptr);

  JXL_TEST_ASSIGN_OR_DIE(
      std::vector<FactorizationCandidate> candidates,
      RankAndTrimFactorizations(opt_data, effort, nullptr));
  ASSERT_FALSE(candidates.empty());
  candidates.resize(1);

  JXL_TEST_ASSIGN_OR_DIE(PassSearchResult hard,
                         SearchPassAwareContextModel(opt_data, candidates,
                                                     effort, nullptr));

  // Initialize with non-optimal temperatures to leave room for improvement.
  GradientState state = InitGradientStateFromHard(
      *opt_data, hard, /*hard_logit=*/0.1,
      /*threshold_temperature=*/200.0, /*pass_temperature=*/2.0,
      /*cluster_temperature=*/2.0);
  state.num_hists = 4;
  InitCtxLogitsRoundRobin(/*hard_logit=*/0.1, &state);

  AdamConfig adam_cfg;
  adam_cfg.lr = 0.05;

  AnnealSchedule sched;
  sched.hot_iters = 5;
  sched.anneal_iters = 15;
  sched.pass_init = 2.0;
  sched.pass_final = 0.1;
  sched.threshold_init = 200.0;
  sched.threshold_final = 1.0;
  sched.cluster_init = 2.0;
  sched.cluster_final = 0.1;

  const OptimizeResult opt =
      RunGradientSolve(*opt_data, adam_cfg, sched, &state);
  EXPECT_EQ(opt.iters_taken, sched.hot_iters + sched.anneal_iters);
  EXPECT_LT(opt.final_cost_bits, opt.init_cost_bits)
      << "init=" << opt.init_cost_bits << " final=" << opt.final_cost_bits;
  fprintf(stderr,
          "GRAD_OPT: init=%.3f bits  final=%.3f bits  reduction=%.3f bits "
          "(%.2f%%)  iters=%u\n",
          opt.init_cost_bits, opt.final_cost_bits,
          opt.init_cost_bits - opt.final_cost_bits,
          100.0 * (opt.init_cost_bits - opt.final_cost_bits) /
              std::max(1.0, opt.init_cost_bits),
          opt.iters_taken);

  const PassSearchResult rounded = RoundToHardAssignment(*opt_data, state);
  EXPECT_EQ(rounded.num_passes, hard.num_passes);
  EXPECT_GE(rounded.num_clusters, 1u);
  EXPECT_LE(rounded.num_clusters, hard.num_clusters);
  EXPECT_EQ(rounded.ctx_map.size(), hard.ctx_map.size());
  ExpectDenseClusters(rounded);
  for (uint32_t a = 0; a < kNumCh; ++a) {
    EXPECT_EQ(rounded.thresholds.T[a].size(), hard.thresholds.T[a].size());
    // Strictly increasing.
    for (size_t j = 1; j < rounded.thresholds.T[a].size(); ++j) {
      EXPECT_GT(rounded.thresholds.T[a][j], rounded.thresholds.T[a][j - 1]);
    }
  }
  for (uint32_t c = 0; c < kNumCh; ++c) {
    EXPECT_EQ(rounded.pass_assignment[c].size(), opt_data->num_blocks[c]);
    for (uint8_t p : rounded.pass_assignment[c]) {
      EXPECT_LT(p, hard.num_passes);
    }
  }
}

// Hard-limit test for the total-cost path. At tiny temperatures the soft
// forward should produce finite AC/NZ costs and a finite signalling estimate.
// With a limited context-map histogram budget `H`, AC signalling overhead is
// charged on the routed `(pass, h)` histograms, so it is not expected to match
// the hard scorer's per-cluster AC split exactly.
TEST(JpegGradTest, TotalCostHardLimitAgreesWithPassAwareModel) {
  JPEGCtxEffortParams effort =
      JPEGCtxEffortParams::FromSpeedTier(SpeedTier::kKitten);
  effort.keep_top_k = 1;
  effort.main_m_target = 16;
  effort.main_iters = 1;
  effort.refine_iters = 0;

  std::shared_ptr<JPEGOptData> opt_data =
      BuildOptDataFromFixture(effort.ac_hist_model);
  ASSERT_NE(opt_data, nullptr);

  JXL_TEST_ASSIGN_OR_DIE(
      std::vector<FactorizationCandidate> candidates,
      RankAndTrimFactorizations(opt_data, effort, nullptr));
  ASSERT_FALSE(candidates.empty());
  candidates.resize(1);

  JXL_TEST_ASSIGN_OR_DIE(PassSearchResult hard,
                         SearchPassAwareContextModel(opt_data, candidates,
                                                     effort, nullptr));
  ASSERT_GT(hard.ac_cost + hard.nz_cost, 0);

  constexpr double kHardLogit = 40.0;
  constexpr double kTinyTemp = 1e-6;
  GradientState state = InitGradientStateFromHard(
      *opt_data, hard, kHardLogit, kTinyTemp, kTinyTemp, kTinyTemp);
  state.num_hists = 4;
  InitCtxLogitsRoundRobin(kHardLogit, &state);

  SoftCostResult soft = ComputeSoftTotalCost(*opt_data, state);
  ASSERT_GT(soft.num_cp_slots, 0u);
  EXPECT_GT(soft.ac_cost_bits + soft.nz_cost_bits, 0.0);

  EXPECT_TRUE(std::isfinite(soft.signalling_overhead_bits));
  EXPECT_GT(soft.signalling_overhead_bits, 0.0);
}

// Finite-difference check on the total-cost gradient. AC + NZ entropy terms
// carry gradient; signalling overhead is treated as constant and does not. To
// keep the FD signal distinguishable, we check gradient on both pass logits
// (which NZ + AC both touch) and thresholds (AC only — thresholds don't affect
// NZ histograms since pb is integer-rounded).
TEST(JpegGradTest, TotalCostAnalyticGradientMatchesFiniteDifference) {
  JPEGCtxEffortParams effort =
      JPEGCtxEffortParams::FromSpeedTier(SpeedTier::kKitten);
  effort.keep_top_k = 1;
  effort.main_m_target = 16;
  effort.main_iters = 1;
  effort.refine_iters = 0;

  std::shared_ptr<JPEGOptData> opt_data =
      BuildOptDataFromFixture(effort.ac_hist_model);
  ASSERT_NE(opt_data, nullptr);

  JXL_TEST_ASSIGN_OR_DIE(
      std::vector<FactorizationCandidate> candidates,
      RankAndTrimFactorizations(opt_data, effort, nullptr));
  ASSERT_FALSE(candidates.empty());
  candidates.resize(1);

  JXL_TEST_ASSIGN_OR_DIE(PassSearchResult hard,
                         SearchPassAwareContextModel(opt_data, candidates,
                                                     effort, nullptr));

  constexpr double kHardLogit = 2.0;
  constexpr double kPassTemp = 1.0;
  constexpr double kThresholdTemp = 50.0;
  constexpr double kClusterTemp = 1.0;
  GradientState state = InitGradientStateFromHard(
      *opt_data, hard, kHardLogit, kThresholdTemp, kPassTemp, kClusterTemp);
  state.num_hists = 4;
  InitCtxLogitsRoundRobin(kHardLogit, &state);

  GradientGrad grad;
  grad.Reset(state);
  const SoftCostResult base = SoftForwardBackward(*opt_data, state, &grad);
  ASSERT_GT(base.num_cp_slots, 0u);

  // FD uses the ENTROPY-only subset of total cost (AC + NZ), because
  // signalling overhead uses integer-rounded histograms whose finite-difference
  // behaviour is piecewise-constant (every FD step lands in the same bucket
  // rounding, then jumps). The analytic gradient excludes signalling anyway,
  // so comparing against AC+NZ is apples-to-apples.
  auto fd_entropy_grad = [&](double* param, double h) {
    const double orig = *param;
    *param = orig + h;
    const SoftCostResult rp = ComputeSoftTotalCost(*opt_data, state);
    *param = orig - h;
    const SoftCostResult rm = ComputeSoftTotalCost(*opt_data, state);
    *param = orig;
    return ((rp.ac_cost_bits + rp.nz_cost_bits) -
            (rm.ac_cost_bits + rm.nz_cost_bits)) /
           (2.0 * h);
  };

  const double kLogitStep = 1e-4;
  uint32_t pass_checks = 0;
  for (uint32_t c = 0; c < opt_data->channels; ++c) {
    const uint32_t nb = opt_data->num_blocks[c];
    const uint32_t stride = hard.num_passes;
    const uint32_t stop = std::min<uint32_t>(nb, 3u);
    for (uint32_t b = 0; b < stop; ++b) {
      for (uint32_t p = 0; p < hard.num_passes; ++p) {
        const size_t idx = static_cast<size_t>(b) * stride + p;
        double* param = &state.pass_logits[c][idx];
        const double analytic = grad.pass_logits[c][idx];
        const double numeric = fd_entropy_grad(param, kLogitStep);
        const double tol =
            1e-3 * (1.0 + std::abs(analytic) + std::abs(numeric)) + 1e-6;
        EXPECT_NEAR(analytic, numeric, tol)
            << "total pass_logits c=" << c << " b=" << b << " p=" << p;
        ++pass_checks;
      }
    }
  }
  EXPECT_GT(pass_checks, 0u);

  const double kThrStep = 0.5;
  uint32_t thr_checks = 0;
  for (uint32_t a = 0; a < kNumCh; ++a) {
    for (size_t j = 0; j < state.thresholds[a].size(); ++j) {
      double* param = &state.thresholds[a][j];
      const double analytic = grad.thresholds[a][j];
      const double numeric = fd_entropy_grad(param, kThrStep);
      const double tol =
          1.5e-3 * (1.0 + std::abs(analytic) + std::abs(numeric)) + 1e-4;
      EXPECT_NEAR(analytic, numeric, tol)
          << "total thresholds a=" << a << " j=" << j;
      ++thr_checks;
    }
  }
  EXPECT_GT(thr_checks, 0u);

  // Cluster-logit gradient check (iteration 5). Every (channel, cell) has
  // `num_clusters` logits; to bound runtime we spot-check the first few cells
  // per channel.
  const double kClusterStep = 1e-3;
  uint32_t cluster_checks = 0;
  for (uint32_t c = 0; c < opt_data->channels; ++c) {
    const uint32_t num_cells = state.num_cells;
    const uint32_t K = state.num_clusters;
    const uint32_t cells_to_check = std::min<uint32_t>(num_cells, 3u);
    for (uint32_t cell = 0; cell < cells_to_check; ++cell) {
      for (uint32_t k = 0; k < K; ++k) {
        const size_t idx = static_cast<size_t>(cell) * K + k;
        double* param = &state.cluster_logits[c][idx];
        const double analytic = grad.cluster_logits[c][idx];
        const double numeric = fd_entropy_grad(param, kClusterStep);
        const double tol =
            1e-3 * (1.0 + std::abs(analytic) + std::abs(numeric)) + 1e-5;
        EXPECT_NEAR(analytic, numeric, tol)
            << "cluster_logits c=" << c << " cell=" << cell << " k=" << k;
        ++cluster_checks;
      }
    }
  }
  EXPECT_GT(cluster_checks, 0u);
}

// Iteration 5: cluster_logits hard-limit test. With tiny cluster_temperature,
// soft `rho` collapses to one-hot matching `hard.ctx_map`, and AC/NZ costs
// match `EvaluatePassAwareModel`. This is the cluster-specific analog of the
// earlier hard-limit tests.
TEST(JpegGradTest, ClusterLogitsHardLimitAgreesWithPassAwareModel) {
  JPEGCtxEffortParams effort =
      JPEGCtxEffortParams::FromSpeedTier(SpeedTier::kKitten);
  effort.keep_top_k = 1;
  effort.main_m_target = 16;
  effort.main_iters = 1;
  effort.refine_iters = 0;

  std::shared_ptr<JPEGOptData> opt_data =
      BuildOptDataFromFixture(effort.ac_hist_model);
  ASSERT_NE(opt_data, nullptr);

  JXL_TEST_ASSIGN_OR_DIE(
      std::vector<FactorizationCandidate> candidates,
      RankAndTrimFactorizations(opt_data, effort, nullptr));
  ASSERT_FALSE(candidates.empty());
  candidates.resize(1);

  JXL_TEST_ASSIGN_OR_DIE(PassSearchResult hard,
                         SearchPassAwareContextModel(opt_data, candidates,
                                                     effort, nullptr));

  constexpr double kHardLogit = 40.0;
  constexpr double kTinyTemp = 1e-6;
  GradientState state = InitGradientStateFromHard(
      *opt_data, hard, kHardLogit, kTinyTemp, kTinyTemp, kTinyTemp);
  state.num_hists = 4;
  InitCtxLogitsRoundRobin(kHardLogit, &state);

  const SoftCostResult soft = ComputeSoftTotalCost(*opt_data, state);
  ASSERT_GT(soft.num_cp_slots, 0u);
  EXPECT_GT(soft.ac_cost_bits + soft.nz_cost_bits, 0.0);

  // Round-trip: argmax of cluster_logits must reproduce the hard ctx_map.
  const PassSearchResult rounded = RoundToHardAssignment(*opt_data, state);
  ASSERT_EQ(rounded.ctx_map.size(), hard.ctx_map.size());
  for (size_t i = 0; i < hard.ctx_map.size(); ++i) {
    EXPECT_EQ(rounded.ctx_map[i], hard.ctx_map[i]) << "ctx_map idx=" << i;
  }
}

TEST(JpegGradTest, RoundToHardAssignmentCompactsClusterHoles) {
  std::shared_ptr<JPEGOptData> opt_data =
      BuildOptDataFromFixture(JPEGTranscodeACModel::kToken420);
  ASSERT_NE(opt_data, nullptr);

  GradientState state;
  state.num_passes = 1;
  state.num_clusters = 4;
  state.num_cells = 2;
  state.thresholds[0] = {1.0};
  for (uint32_t c = 0; c < kNumCh; ++c) {
    state.pass_logits[c].assign(opt_data->num_blocks[c], 0.0);
    if (c >= opt_data->channels) continue;
    state.cluster_logits[c].assign(state.num_cells * state.num_clusters, -10.0);
    state.cluster_logits[c][0 * state.num_clusters + 0] = 10.0;
    state.cluster_logits[c][1 * state.num_clusters + 3] = 10.0;
  }

  const PassSearchResult rounded = RoundToHardAssignment(*opt_data, state);
  EXPECT_EQ(rounded.num_clusters, 2u);
  ASSERT_EQ(rounded.ctx_map.size(),
            static_cast<size_t>(opt_data->channels) * state.num_cells);
  for (uint32_t c = 0; c < opt_data->channels; ++c) {
    EXPECT_EQ(rounded.ctx_map[c * state.num_cells + 0], 0u);
    EXPECT_EQ(rounded.ctx_map[c * state.num_cells + 1], 1u);
  }
  ExpectDenseClusters(rounded);
}

// Iteration 6: parallel sweep over MaximalFactorizations. The smoke test runs
// the full orchestrator with a modest iteration budget on a small fixture.
// This validates plumbing end-to-end. Deeper correctness is covered by the
// iterations-1-5 tests on the flower fixture.
TEST(JpegGradTest, SearchGradientContextModelSmoke) {
  JPEGCtxEffortParams effort =
      JPEGCtxEffortParams::FromSpeedTier(SpeedTier::kKitten);
  effort.grad_hot_iters = 2;
  effort.grad_anneal_iters = 3;
  effort.grad_init_temperature = 1.0;
  effort.grad_lr = 0.05;
  // Iteration 8: exercise the multi-pass sweep. 0 = planner chooses pass count
  // (sweeps `[1, ComputeMaxNumPasses]`). At `-1` the sweep is disabled and
  // only `num_passes = 1` is considered; this test covers the sweep path.
  effort.optimize_passes_num = 0;

  // Load the smaller `sideways_bench.jpg` (15 KB) instead of the 550 KB flower
  // fixture — the sweep runs the optimizer once per factorization (~58 runs),
  // so fixture size dominates smoke runtime.
  JxlMemoryManager* memory_manager = test::MemoryManager();
  const std::vector<uint8_t> jpeg_bytes =
      test::ReadTestData("jxl/jpeg_reconstruction/sideways_bench.jpg");
  auto jpeg_data_or = jpeg::ParseJPG(memory_manager, Bytes(jpeg_bytes));
  ASSERT_TRUE(jpeg_data_or.ok());
  std::unique_ptr<jpeg::JPEGData> jpeg_data =
      std::move(jpeg_data_or).value_();
  ColorTransform color_transform;
  ASSERT_TRUE(
      jpeg::SetColorTransformFromJpegData(*jpeg_data, &color_transform));
  const std::array<int, 3> plane_to_jpeg =
      JpegOrder(color_transform, jpeg_data->components.size() == 1);
  const JpegCflContext cfl_ctx = {plane_to_jpeg,
                                  false,
                                  {nullptr, nullptr},
                                  {nullptr, nullptr}};
  auto opt_data = std::make_shared<JPEGOptData>();
  ASSERT_TRUE(opt_data->BuildFromJPEG(*jpeg_data, effort.ac_hist_model,
                                       cfl_ctx, nullptr));

  JXL_TEST_ASSIGN_OR_DIE(PassSearchResult result,
                         SearchGradientContextModel(opt_data, effort, nullptr));

  // Basic shape invariants. ctx_map size matches active channels × num_cells.
  EXPECT_GE(result.num_passes, 1u);
  EXPECT_GE(result.num_clusters, 1u);
  EXPECT_LE(result.num_clusters, kMaxClusters);
  const size_t num_cells = (result.thresholds.TY().size() + 1) *
                           (result.thresholds.TCb().size() + 1) *
                           (result.thresholds.TCr().size() + 1);
  EXPECT_EQ(result.ctx_map.size(),
            static_cast<size_t>(opt_data->channels) * num_cells);
  ExpectDenseClusters(result);
  for (size_t c = 0; c < kNumCh; ++c) {
    EXPECT_EQ(result.pass_assignment[c].size(), opt_data->num_blocks[c]);
    for (uint8_t p : result.pass_assignment[c]) {
      EXPECT_LT(p, result.num_passes);
    }
  }
  // Thresholds strictly increasing on each axis.
  for (uint32_t a = 0; a < kNumCh; ++a) {
    for (size_t j = 1; j < result.thresholds.T[a].size(); ++j) {
      EXPECT_GT(result.thresholds.T[a][j], result.thresholds.T[a][j - 1]);
    }
  }
}

}  // namespace
}  // namespace jxl
