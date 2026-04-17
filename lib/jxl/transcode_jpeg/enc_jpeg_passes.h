// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#ifndef LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_PASSES_H_
#define LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_PASSES_H_

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "lib/jxl/transcode_jpeg/enc_jpeg_search.h"

namespace jxl {

using PassAssignment = std::array<std::vector<uint8_t>, kNumCh>;

struct PassSearchResult {
  ThresholdSet thresholds;
  ContextMap ctx_map;
  PassAssignment pass_assignment;
  uint32_t num_passes = 1;
  uint32_t num_clusters = 0;
  FixedPointCost ac_cost = 0;
  FixedPointCost nz_cost = 0;
  FixedPointCost signalling_overhead = 0;
  FixedPointCost total_cost = 0;
};

StatusOr<PassSearchResult> SearchPassAwareContextModel(
    std::shared_ptr<const JPEGOptData> opt_data,
    const std::vector<FactorizationCandidate>& candidates,
    const JPEGCtxEffortParams& effort, ThreadPool* pool);

StatusOr<PassSearchResult> SearchPassAwareContextModel(
    std::shared_ptr<const JPEGOptData> opt_data,
    const JPEGCtxEffortParams& effort, ThreadPool* pool);

}  // namespace jxl

#endif  // LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_PASSES_H_
