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
      test::ReadTestData("jxl/jpeg_reconstruction/1x1_exif_xmp.jpg");
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

  // Convert soft cost (bits) to fixed-point units for comparison.
  const FixedPointCost soft_fp = static_cast<FixedPointCost>(
      std::llround(soft.ac_cost_bits * static_cast<double>(kFScale)));

  // Tolerance: fractional-count ftab evaluation rounds slightly differently
  // than precomputed integer-index `ftab`. Allow a small slack proportional to
  // the number of distinct histogram slots touched.
  const FixedPointCost tol =
      static_cast<FixedPointCost>(soft.num_cp_slots) * 16;

  EXPECT_NEAR(static_cast<double>(soft_fp), static_cast<double>(hard.ac_cost),
              static_cast<double>(tol) + 1.0);
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

}  // namespace
}  // namespace jxl
