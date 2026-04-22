// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#ifndef LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_PASS_UTILS_H_
#define LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_PASS_UTILS_H_

#include <chrono>
#include <cstdint>

#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"

namespace jxl {

// Shared constants for pass-aware optimization.
constexpr uint32_t kMaxIters = 100;
constexpr uint32_t kLargeImageThreshold = 1u << 15;
constexpr uint32_t kBatchChunkSize = 1u << 14;

// Timing helpers for planner debug instrumentation.
using PlannerClock = std::chrono::high_resolution_clock;

inline int64_t ElapsedNanos(const PlannerClock::time_point& start,
                            const PlannerClock::time_point& end) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(end - start)
      .count();
}

inline double NanosToMs(int64_t ns) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::nanoseconds(ns))
      .count();
}

// Result structure for threshold pruning.
struct PrunedCtxMapResult {
  ThresholdSet thresholds;
  ContextMap ctx_map;
};

// Removes thresholds that do not separate any cluster pair in the context map.
// Used when clustering produces fewer clusters than the threshold grid size.
PrunedCtxMapResult PruneDeadThresholdsFromCtxMap(const ThresholdSet& thresholds,
                                                 ContextMap ctx_map,
                                                 uint32_t channels);

}  // namespace jxl

#endif  // LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_PASS_UTILS_H_
