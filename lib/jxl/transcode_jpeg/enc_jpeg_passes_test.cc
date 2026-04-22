// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include <jxl/types.h>

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include "lib/jxl/enc_jpeg_frame.h"
#include "lib/jxl/frame_header.h"
#include "lib/jxl/jpeg/enc_jpeg_data.h"
#include "lib/jxl/test_memory_manager.h"
#include "lib/jxl/test_utils.h"
#include "lib/jxl/testing.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_passes.h"

namespace jxl {
namespace {

TEST(JpegPassesTest, SearchPassAwareContextModelSmoke) {
  JxlMemoryManager* memory_manager = test::MemoryManager();
  const std::vector<uint8_t> jpeg_bytes =
      test::ReadTestData("jxl/jpeg_reconstruction/1x1_exif_xmp.jpg");
  JXL_TEST_ASSIGN_OR_DIE(std::unique_ptr<jpeg::JPEGData> jpeg_data,
                         jpeg::ParseJPG(memory_manager, Bytes(jpeg_bytes)));

  ColorTransform color_transform;
  ASSERT_TRUE(jpeg::SetColorTransformFromJpegData(*jpeg_data, &color_transform));
  const std::array<int, 3> plane_to_jpeg =
      JpegOrder(color_transform, jpeg_data->components.size() == 1);
  const JpegCflContext cfl_ctx = {plane_to_jpeg,
                                  false,
                                  {nullptr, nullptr},
                                  {nullptr, nullptr}};

  JPEGCtxEffortParams effort =
      JPEGCtxEffortParams::FromSpeedTier(SpeedTier::kKitten);
  effort.keep_top_k = 1;
  effort.main_m_target = 16;
  effort.main_iters = 1;
  effort.refine_iters = 0;
  std::shared_ptr<JPEGOptData> opt_data = std::make_shared<JPEGOptData>();
  ASSERT_TRUE(
      opt_data->BuildFromJPEG(*jpeg_data, effort.ac_hist_model, cfl_ctx, nullptr));

  JXL_TEST_ASSIGN_OR_DIE(std::vector<FactorizationCandidate> candidates,
                         RankAndTrimFactorizations(opt_data, effort, nullptr));
  ASSERT_FALSE(candidates.empty());
  candidates.resize(1);

  JXL_TEST_ASSIGN_OR_DIE(PassSearchResult result,
                         SearchPassAwareContextModel(opt_data, candidates,
                                                     effort, nullptr));

  EXPECT_GE(result.num_passes, 1u);
  EXPECT_GE(result.num_clusters, 1u);
  EXPECT_GE(result.ac_cost, 0);
  EXPECT_GE(result.nz_cost, 0);
  EXPECT_GE(result.signalling_overhead, 0);
  EXPECT_LT(result.total_cost, std::numeric_limits<int64_t>::max());

  const size_t num_cells = (result.thresholds.TY().size() + 1) *
                           (result.thresholds.TCb().size() + 1) *
                           (result.thresholds.TCr().size() + 1);
  EXPECT_EQ(result.ctx_map.size(),
            static_cast<size_t>(opt_data->channels) * num_cells);
  for (size_t c = 0; c < kNumCh; ++c) {
    EXPECT_EQ(result.pass_assignment[c].size(), opt_data->num_blocks[c]);
  }
}

TEST(JpegPassesTest, SearchBiclusteredContextModelSmoke) {
  JxlMemoryManager* memory_manager = test::MemoryManager();
  const std::vector<uint8_t> jpeg_bytes =
      test::ReadTestData("jxl/jpeg_reconstruction/1x1_exif_xmp.jpg");
  JXL_TEST_ASSIGN_OR_DIE(std::unique_ptr<jpeg::JPEGData> jpeg_data,
                         jpeg::ParseJPG(memory_manager, Bytes(jpeg_bytes)));

  ColorTransform color_transform;
  ASSERT_TRUE(jpeg::SetColorTransformFromJpegData(*jpeg_data, &color_transform));
  const std::array<int, 3> plane_to_jpeg =
      JpegOrder(color_transform, jpeg_data->components.size() == 1);
  const JpegCflContext cfl_ctx = {plane_to_jpeg,
                                  false,
                                  {nullptr, nullptr},
                                  {nullptr, nullptr}};

  JPEGCtxEffortParams effort =
      JPEGCtxEffortParams::FromSpeedTier(SpeedTier::kKitten);
  effort.keep_top_k = 1;
  effort.main_m_target = 16;
  effort.main_iters = 1;
  effort.refine_iters = 0;
  std::shared_ptr<JPEGOptData> opt_data = std::make_shared<JPEGOptData>();
  ASSERT_TRUE(
      opt_data->BuildFromJPEG(*jpeg_data, effort.ac_hist_model, cfl_ctx, nullptr));

  JXL_TEST_ASSIGN_OR_DIE(std::vector<FactorizationCandidate> candidates,
                         RankAndTrimFactorizations(opt_data, effort, nullptr));
  ASSERT_FALSE(candidates.empty());
  candidates.resize(1);

  JXL_TEST_ASSIGN_OR_DIE(BiclusterSearchResult result,
                         SearchBiclusteredContextModel(opt_data, candidates,
                                                       effort, nullptr));

  EXPECT_GE(result.num_passes, 1u);
  EXPECT_GE(result.num_row_clusters, 1u);
  EXPECT_LE(result.num_row_clusters, kMaxClusters);
  EXPECT_EQ(result.num_prototypes_per_pass.size(), result.num_passes);
  EXPECT_GE(result.total_num_prototypes, 1u);
  EXPECT_GE(result.ac_cost, 0);
  EXPECT_GE(result.nz_cost, 0);
  EXPECT_GE(result.signalling_overhead, 0);
  EXPECT_LT(result.total_cost, std::numeric_limits<int64_t>::max());
  EXPECT_EQ(result.ctx_map.size(),
            static_cast<size_t>(opt_data->channels) * result.num_cells);
  for (uint8_t id : result.ctx_map) {
    EXPECT_LT(id, result.num_row_clusters);
  }
  for (size_t c = 0; c < kNumCh; ++c) {
    EXPECT_EQ(result.pass_assignment[c].size(), opt_data->num_blocks[c]);
  }
}

TEST(JpegPassesTest, SearchBiclusteredContextModelThresholdFirstSmoke) {
  JxlMemoryManager* memory_manager = test::MemoryManager();
  const std::vector<uint8_t> jpeg_bytes =
      test::ReadTestData("jxl/jpeg_reconstruction/1x1_exif_xmp.jpg");
  JXL_TEST_ASSIGN_OR_DIE(std::unique_ptr<jpeg::JPEGData> jpeg_data,
                         jpeg::ParseJPG(memory_manager, Bytes(jpeg_bytes)));

  ColorTransform color_transform;
  ASSERT_TRUE(jpeg::SetColorTransformFromJpegData(*jpeg_data, &color_transform));
  const std::array<int, 3> plane_to_jpeg =
      JpegOrder(color_transform, jpeg_data->components.size() == 1);
  const JpegCflContext cfl_ctx = {plane_to_jpeg,
                                  false,
                                  {nullptr, nullptr},
                                  {nullptr, nullptr}};

  JPEGCtxEffortParams effort =
      JPEGCtxEffortParams::FromSpeedTier(SpeedTier::kKitten);
  effort.keep_top_k = 1;
  effort.main_m_target = 16;
  effort.main_iters = 1;
  effort.refine_iters = 0;
  effort.bicluster_threshold_first = true;
  std::shared_ptr<JPEGOptData> opt_data = std::make_shared<JPEGOptData>();
  ASSERT_TRUE(
      opt_data->BuildFromJPEG(*jpeg_data, effort.ac_hist_model, cfl_ctx, nullptr));

  JXL_TEST_ASSIGN_OR_DIE(std::vector<FactorizationCandidate> candidates,
                         RankAndTrimFactorizations(opt_data, effort, nullptr));
  ASSERT_FALSE(candidates.empty());
  candidates.resize(1);

  JXL_TEST_ASSIGN_OR_DIE(BiclusterSearchResult result,
                         SearchBiclusteredContextModel(opt_data, candidates,
                                                       effort, nullptr));

  EXPECT_GE(result.num_passes, 1u);
  EXPECT_GE(result.num_row_clusters, 1u);
  EXPECT_LE(result.num_row_clusters, kMaxClusters);
  EXPECT_EQ(result.num_prototypes_per_pass.size(), result.num_passes);
  EXPECT_GE(result.total_num_prototypes, 1u);
  EXPECT_GE(result.ac_cost, 0);
  EXPECT_GE(result.nz_cost, 0);
  EXPECT_GE(result.signalling_overhead, 0);
  EXPECT_LT(result.total_cost, std::numeric_limits<int64_t>::max());
  EXPECT_EQ(result.ctx_map.size(),
            static_cast<size_t>(opt_data->channels) * result.num_cells);
  for (uint8_t id : result.ctx_map) {
    EXPECT_LT(id, result.num_row_clusters);
  }
  for (size_t c = 0; c < kNumCh; ++c) {
    EXPECT_EQ(result.pass_assignment[c].size(), opt_data->num_blocks[c]);
  }
}

}  // namespace
}  // namespace jxl
