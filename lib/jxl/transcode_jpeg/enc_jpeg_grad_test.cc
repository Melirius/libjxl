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
//    Initialize a `GradientJointState` with those thresholds (copied verbatim),
//    pass logits saturated to hard one-hot, and tiny temperatures. Assert that
//    `ComputeSoftACCost` returns the same AC cost (up to fixed-point rounding).
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

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "lib/jxl/enc_jpeg_frame.h"
#include "lib/jxl/jpeg/enc_jpeg_data.h"
#include "lib/jxl/test_memory_manager.h"
#include "lib/jxl/test_utils.h"
#include "lib/jxl/testing.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_passes.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_search.h"

namespace jxl {
namespace {

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
  GradientJointState state = InitGradientJointStateFromHard(
      *opt_data, hard, kHardLogit, kTinyTemp, kTinyTemp);
  ASSERT_EQ(state.num_passes, hard.num_passes);

  SoftCostResult soft = ComputeSoftACCost(*opt_data, state, hard.ctx_map,
                                          hard.num_clusters, hard.num_passes);
  ASSERT_GT(soft.num_cp_slots, 0u);

  // Convert soft cost (bits) to fixed-point units for comparison.
  const FixedPointCost soft_fp = static_cast<FixedPointCost>(
      std::llround(soft.ac_cost_bits * static_cast<double>(kFScale)));

  // Two tolerance sources add:
  //   1. Fractional-count `ftab` via double arithmetic vs the precomputed
  //      integer-index `ftab`: a few ULPs per touched (cluster, pass) slot.
  //   2. Floating-point accumulation error across many events: roughly
  //      `hard.ac_cost * double_ulp * sqrt(num_events)`; we use a generous
  //      relative floor of 1e-9 which dominates on real inputs.
  const double rel_floor = 1e-9 * std::abs(static_cast<double>(hard.ac_cost));
  const double tol =
      std::max<double>(static_cast<double>(soft.num_cp_slots) * 16.0,
                       rel_floor) +
      1.0;

  EXPECT_NEAR(static_cast<double>(soft_fp), static_cast<double>(hard.ac_cost),
              tol);
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
  GradientJointState hard_state = InitGradientJointStateFromHard(
      *opt_data, hard, kHardLogit, kTinyTemp, kTinyTemp);
  const SoftCostResult hard_cost =
      ComputeSoftACCost(*opt_data, hard_state, hard.ctx_map, 1, 1);

  // Now: two passes, single cluster, single cell. All blocks share uniform
  // pass logits (= 0 => softmax gives 1/P per pass).
  const uint32_t P = 2;
  GradientJointState uniform_state;
  uniform_state.thresholds[0].clear();
  uniform_state.thresholds[1].clear();
  uniform_state.thresholds[2].clear();
  uniform_state.threshold_temperature = kTinyTemp;
  uniform_state.pass_temperature = 1.0;
  uniform_state.num_passes = P;
  for (uint32_t c = 0; c < kNumCh; ++c) {
    uniform_state.pass_logits[c].assign(
        static_cast<size_t>(opt_data->num_blocks[c]) * P, 0.0);
  }

  const SoftCostResult uniform_cost =
      ComputeSoftACCost(*opt_data, uniform_state, hard.ctx_map, 1, P);

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
  GradientJointState state = InitGradientJointStateFromHard(
      *opt_data, hard, kHardLogit, kThresholdTemp, kPassTemp);

  GradientJointGrad grad;
  ResetGradientJointGrad(state, &grad);
  const SoftCostResult base =
      ComputeSoftACCostWithGrad(*opt_data, state, hard.ctx_map,
                                hard.num_clusters, hard.num_passes, &grad);
  ASSERT_GT(base.num_cp_slots, 0u);

  // Helper: central difference on a single parameter reference.
  auto fd_grad = [&](double* param, double h) {
    const double orig = *param;
    *param = orig + h;
    const SoftCostResult rp =
        ComputeSoftACCost(*opt_data, state, hard.ctx_map, hard.num_clusters,
                          hard.num_passes);
    *param = orig - h;
    const SoftCostResult rm =
        ComputeSoftACCost(*opt_data, state, hard.ctx_map, hard.num_clusters,
                          hard.num_passes);
    *param = orig;
    return (rp.ac_cost_bits - rm.ac_cost_bits) / (2.0 * h);
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
            1e-4 * (1.0 + std::abs(analytic) + std::abs(numeric)) + 1e-6;
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
          1e-3 * (1.0 + std::abs(analytic) + std::abs(numeric)) + 1e-4;
      EXPECT_NEAR(analytic, numeric, tol)
          << "thresholds a=" << a << " j=" << j;
      ++thr_checks;
    }
  }
  // At least one axis has thresholds for any non-trivial factorization.
  EXPECT_GT(thr_checks, 0u);
}

}  // namespace
}  // namespace jxl
