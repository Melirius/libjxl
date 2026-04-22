// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#ifndef LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_PASS_STREAM_H_
#define LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_PASS_STREAM_H_

#include <cstdint>
#include <vector>

#include "lib/jxl/base/status.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_axis_maps.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_assign.h"

namespace jxl {

class ThreadPool;
struct AxisMaps;

// Maps one block to its threshold cell id for the given threshold set. This is
// shared by the pass-aware evaluator and the biclustering lattice builder.
uint32_t BlockCell(const JPEGOptData& d, const AxisMaps& axis_maps,
                   const ThresholdSet& thresholds, uint32_t c, uint32_t b);

// Helper for agglomerative clustering union-find operations.
uint32_t FindRoot(std::vector<uint32_t>& parent, uint32_t x);

// Compute overhead cost for pass-aware encoding.
FixedPointCost ComputePassOverhead(const JPEGOptData& d);

// Builds the fixed-threshold `(channel, cell)` row assignment for all blocks
// given a threshold set. Used by both pass-aware and biclustering paths.
FixedRows BuildFixedRows(const JPEGOptData& d, const ThresholdSet& thresholds);

// Rebuilds the AC stream after pass assignment. The resulting stream keeps the
// same packed-entry layout as the canonical optimizer stream, but is split into
// independent contiguous pass ranges recorded in `pass_offsets`.
StatusOr<std::vector<ACEntry>> BuildPassStream(
    const JPEGOptData& d, const ActiveRawBins& active,
    const PassAssignment& pass_assignment, uint32_t num_passes,
    std::vector<uint32_t>* pass_offsets, ThreadPool* pool);

}  // namespace jxl

#endif  // LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_PASS_STREAM_H_
