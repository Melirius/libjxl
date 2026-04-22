// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#ifndef LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_PASS_CLUSTER_H_
#define LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_PASS_CLUSTER_H_

#include <cstdint>
#include <vector>

#include "lib/jxl/base/status.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_passes.h"

namespace jxl {

class ThreadPool;

// Result of clustering pass-aware `(cell, pass)` contexts.
struct ClusterResult {
  ContextMap ctx_map;
  uint32_t num_clusters = 0;
};

// Cost decomposition shared by both experimental search paths.
struct ModelEvaluation {
  FixedPointCost ac_cost = 0;
  FixedPointCost nz_cost = 0;
  FixedPointCost signalling_overhead = 0;
  FixedPointCost corrected_entropy_cost = -1;

  FixedPointCost total_cost() const {
    return (corrected_entropy_cost >= 0 ? corrected_entropy_cost
                                        : ac_cost) + nz_cost +
           signalling_overhead;
  }
};

// Clusters pass-aware `(cell, pass)` contexts using agglomerative merging.
StatusOr<ClusterResult> ClusterContextsPassAware(
    const JPEGOptData& d, const ThresholdSet& thresholds,
    const std::vector<ACEntry>& pass_stream, const std::vector<uint32_t>& pass_offsets,
    uint32_t num_passes, uint32_t target_clusters);

// Evaluates the pass-aware model for given thresholds and clustering.
StatusOr<ModelEvaluation> EvaluatePassAwareModel(
    const JPEGOptData& d, const ThresholdSet& thresholds,
    const ContextMap& ctx_map, uint32_t num_clusters,
    const PassAssignment& pass_assignment, uint32_t num_passes,
    const std::vector<ACEntry>& pass_stream,
    const std::vector<uint32_t>& pass_offsets,
    FixedPointCost cutoff = std::numeric_limits<FixedPointCost>::max());

}  // namespace jxl

#endif  // LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_PASS_CLUSTER_H_
