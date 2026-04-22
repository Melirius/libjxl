// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#ifndef LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_BICLUSTER_H_
#define LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_BICLUSTER_H_

#include <cstdint>
#include <vector>

#include "lib/jxl/base/status.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_cluster.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_histogram.h"

namespace jxl {

class ThreadPool;

// Cost decomposition shared by both experimental search paths.
struct ModelEvaluation;

// Fixed-threshold histogram lattice for the biclustering prototype.
// Each original row is one `(channel, cell)` pair; within that row we store
// pass-local AC token histograms for every `zdc` slice and pass-local nz-count
// histograms for every predictor bucket.
struct RowSliceHistograms {
  uint32_t num_channels = 0;
  uint32_t num_cells = 0;
  uint32_t num_passes = 1;
  std::vector<DenseHistogram<kACTokenCount>> ac_hist;
  std::vector<uint32_t> ac_total;
  std::vector<DenseHistogram<kJPEGNonZeroRange>> nz_hist;
  std::vector<uint32_t> nz_total;
};

// Cached rough/refined row state for one threshold set. `block_rows` stores the
// current `(channel, cell)` row id for every block so refined thresholds can
// move only the blocks whose row changed, reusing the rough row-slice lattice.
struct RowSliceState {
  RowSliceHistograms rows;
  std::array<std::vector<uint16_t>, kNumCh> block_rows;
};

// Per-pass-configuration NZ predictor cache used by the biclustered row build.
// Entry `[c][b * num_passes + p]` stores the predictor bucket `pb` for block
// `(c, b)` when evaluated in pass `p`.
struct NZBlockCache {
  uint32_t num_passes = 1;
  std::array<std::vector<uint8_t>, kNumCh> pred_bucket;
};

// Builds the per-pass NZ predictor cache for bicluster operations.
NZBlockCache BuildNZBlockCache(const JPEGOptData& d,
                               const PassAssignment& pass_assignment,
                               uint32_t num_passes);

// Builds the initial row-slice histogram lattice for biclustering.
StatusOr<RowSliceState> BuildRowSliceState(
    const JPEGOptData& d, const ThresholdSet& thresholds,
    const PassAssignment& pass_assignment, uint32_t num_passes,
    const NZBlockCache& nz_cache);

// Refines row-slice state after threshold changes.
StatusOr<RowSliceState> RefineRowSliceState(
    const JPEGOptData& d, const ThresholdSet& thresholds,
    const PassAssignment& pass_assignment, uint32_t num_passes,
    const RowSliceState& base_state, const NZBlockCache& nz_cache);

// Clusters rows into prototypes for biclustering scoring.
StatusOr<ClusterResult> ClusterRowsBiclustered(
    const JPEGOptData& d, const RowSliceHistograms& rows,
    uint32_t row_budget);

// Evaluates the biclustering state for given thresholds and row clustering.
StatusOr<ModelEvaluation> EvaluateBiclusterState(
    const JPEGOptData& d, const ThresholdSet& thresholds, const ContextMap& ctx_map,
    uint32_t num_row_clusters, const PassAssignment& pass_assignment,
    uint32_t num_passes, uint32_t proto_budget_per_pass,
    const RowSliceHistograms& rows,
    std::vector<uint32_t>* num_prototypes_per_pass,
    FixedPointCost cutoff = std::numeric_limits<FixedPointCost>::max());

}  // namespace jxl

#endif  // LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_BICLUSTER_H_
