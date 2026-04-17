// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include "lib/jxl/enc_jpeg_frame.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <chrono>
#include <vector>

#include "lib/jxl/base/data_parallel.h"
#include "lib/jxl/base/status.h"
#include "lib/jxl/chroma_from_luma.h"
#include "lib/jxl/coeff_order_fwd.h"
#include "lib/jxl/frame_header.h"
#include "lib/jxl/jpeg/jpeg_data.h"
#include "lib/jxl/quantizer.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_cluster.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_passes.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_refine.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_search.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_threshold.h"

namespace jxl {

// This file implements context-map optimization for JPEG-source images being
// re-encoded into JPEG XL. The single public entry point is
// `OptimizeJPEGContextMap()`, which optimizes per-channel DC thresholds
// and AC context clustering to minimise the entropy-coded size
// of the JPEG AC DCT coefficients.
//
// The pipeline is:
//   1. `JPEGOptData::BuildFromJPEG` extracts per-block DC/AC statistics from
//      the source JPEG and builds a bin-indexed AC stream.
//   2. Enumerate "maximal factorisations" of the (Y, Cb, Cr) DC-threshold
//      space; each factorisation defines how many DC intervals each channel
//      is split into.
//   3. For every candidate factorisation (optionally pre-filtered by a cheap
//      ranking pass), `PartitioningCtx::OptimizeThresholds` refines the DC
//      thresholds via iterative greedy descent, `Clustering::Build` merges
//      similar contexts using entropy-cost-guided agglomerative clustering,
//      and `RefineClustered` performs a final local threshold search on the
//      clustered solution.
//   4. The candidate with the lowest total cost (entropy + histogram signalling
//      overhead) is selected and written into the `BlockCtxMap` consumed by
//      the rest of the encoder.
//
// All calculations (besides formation of `f(n) = n*log2(n)` lookup) are done in
// integer arithmetic to avoid floating point inaccuracies. Counters are
// generally `uint32_t`, that is enough for JPEG.
//
// General notation:
// `N` - number of 8x8 blocks of an image component
// `M` - number of distinct DC values along a component axis
// `M_eff` - number of distinct DC buckets along a component axis,
//           use of full `M` for large values is too slow by cache misses
// `ci` - cell index in 2D array of cells perpendicular to current axis,
//        `ci < 32` as axis search is meaningful with at least 2 cells per axis
// `K` - number of intervals (cells) along an axis
// `l` - left bound of interval (inclusive)
// `n` - right bound of interval (inclusive)
// `c` - component (axis) index, [0,3)
// `zdc` - `jxl::ZeroDensityContext` of an AC coefficient in a block
// `czdc` - `(c,zdc)` = `channel * kZeroDensityContextCount + zdc`, [0,3*458)
// `ai` - AC index = value of AC coefficient + `kDCTOff`

// Convert optimizer `ThresholdSet` + `ContextMap` into a `BlockCtxMap`.
// Shared by both the legacy single-pass and the pass-aware search paths.
Status ConvertToBlockCtxMap(const ThresholdSet& thr, const ContextMap& ctx,
                            const JPEGOptData& opt_data,
                            const JpegCflContext& cfl_ctx,
                            BlockCtxMap& ctx_map) {
  size_t n_Y = thr.TY().size() + 1;
  size_t n_Cb = thr.TCb().size() + 1;
  size_t n_Cr = thr.TCr().size() + 1;
  size_t num_dc_ctxs = n_Y * n_Cb * n_Cr;

  JXL_ENSURE(num_dc_ctxs <= kMaxCells);
  JXL_ENSURE(n_Y <= kMaxIntervals && n_Cb <= kMaxIntervals &&
             n_Cr <= kMaxIntervals);

  ctx_map.num_dc_ctxs = num_dc_ctxs;

  ctx_map.dc_thresholds[0].clear();
  ctx_map.dc_thresholds[1].clear();
  ctx_map.dc_thresholds[2].clear();

  uint32_t effective_channels = opt_data.channels;
  ctx_map.ctx_map.assign(3 * kNumOrders * num_dc_ctxs, 0);
  if (effective_channels == 1) {
    JXL_DASSERT(thr.TCb().empty());
    JXL_DASSERT(thr.TCr().empty());
    const size_t active_plane = opt_data.jpeg_to_plane[0];
    for (int16_t t : thr.TY()) {
      ctx_map.dc_thresholds[active_plane].push_back(t - 1);
    }
    const size_t slot = active_plane < 2 ? active_plane ^ 1 : 2;
    for (size_t cell = 0; cell < num_dc_ctxs; ++cell) {
      ctx_map.ctx_map[slot * kNumOrders * num_dc_ctxs + cell] =
          ctx[cell] + 1;
    }
  } else {
    for (size_t plane = 0; plane < 3; ++plane) {
      const uint32_t jpeg_c =
          static_cast<uint32_t>(cfl_ctx.plane_to_jpeg[plane]);
      for (int16_t t : thr.T[jpeg_c]) {
        ctx_map.dc_thresholds[plane].push_back(t - 1);
      }
      const size_t slot = plane < 2 ? plane ^ 1 : 2;
      for (size_t cell = 0; cell < num_dc_ctxs; ++cell) {
        ctx_map.ctx_map[slot * kNumOrders * num_dc_ctxs + cell] =
            ctx[jpeg_c * num_dc_ctxs + cell];
      }
    }
  }
  size_t num_ctxs =
      *std::max_element(ctx_map.ctx_map.begin(), ctx_map.ctx_map.end()) + 1;
  JXL_ENSURE(num_ctxs <= kMaxClusters);
  ctx_map.num_ctxs = num_ctxs;

  return true;
}

Status OptimizeJPEGContextMap(const jpeg::JPEGData& jpeg_data,
                              SpeedTier speed_tier,
                              const JpegCflContext& cfl_ctx,
                              BlockCtxMap& ctx_map, ThreadPool* pool) {
  const JPEGCtxEffortParams effort =
      JPEGCtxEffortParams::FromSpeedTier(speed_tier);
  auto opt_data = std::make_shared<JPEGOptData>();
  JXL_RETURN_IF_ERROR(
      opt_data->BuildFromJPEG(jpeg_data, effort.ac_hist_model, cfl_ctx, pool));

  JXL_ASSIGN_OR_RETURN(std::vector<FactorizationCandidate> candidates,
                       RankAndTrimFactorizations(opt_data, effort, pool));
  if (candidates.empty()) return true;

  JXL_DEBUG_V(2,
              "JPEG ctx effort at speed tier %i: %i candidates, rank_iters=%u "
              "main_iters=%u refine_iters=%u\n",
              static_cast<int>(speed_tier), static_cast<int>(candidates.size()),
              effort.rank_iters, effort.main_iters, effort.refine_iters);

  FixedPointCost best_cost = std::numeric_limits<FixedPointCost>::max();
  ThresholdSet best_thr;
  ContextMap best_ctx;
  std::mutex mu;

  std::vector<PartitioningCtx> ctx_pool;
  JXL_RETURN_IF_ERROR(RunOnPool(
      pool, 0, static_cast<uint32_t>(candidates.size()),
      [&](size_t num_threads) -> Status {
        ctx_pool.reserve(num_threads);
        for (size_t i = 0; i < num_threads; ++i) {
          ctx_pool.emplace_back(opt_data);
        }
        return true;
      },
      [&](uint32_t idx, size_t thread_id) -> Status {
        const FactorizationCandidate& candidate = candidates[idx];
        PartitioningCtx& ctx = ctx_pool[thread_id];

        FixedPointCost opt_cost = 0;
        ThresholdSet opt_thr = ctx.OptimizeThresholds(
            candidate.init, effort.main_m_target, effort.main_iters, &opt_cost);

        JXL_ASSIGN_OR_RETURN(
            Clustering cl_result,
            Clustering::Build(*opt_data, opt_thr,
                              kMaxClusters - (opt_data->channels == 1),
                              effort.overhead_aware_tail, nullptr));
        ContextMap& cluster_map = cl_result.ctx_map;

        auto refine_result =
            RefineClustered(*opt_data, opt_thr, cl_result, effort.refine_iters,
                            effort.refine_radius);
        ThresholdSet refined_thr = refine_result.thresholds;
        FixedPointCost entropy_cost = refine_result.cost;
        FixedPointCost nz_cost = refine_result.nz_cost;
        (void)nz_cost;
        FixedPointCost total_cost = entropy_cost;

        // Add signalling overhead for histogram headers
        JXL_ASSIGN_OR_RETURN(FixedPointCost overhead,
                             cl_result.ComputeSignallingOverhead(*opt_data));
        total_cost += overhead;

        std::lock_guard<std::mutex> lock(mu);
        JXL_DEBUG_V(2,
                    "(%u,%u,%u) cost: unclustered=%.2f clustered=%.2f "
                    "refined=%.2f nz=%.2f overhead=%.2f total=%.2f\n",
                    candidate.a, candidate.b, candidate.c, bit_cost(opt_cost),
                    bit_cost(cl_result.clustered_cost), bit_cost(entropy_cost),
                    bit_cost(nz_cost), bit_cost(overhead),
                    bit_cost(total_cost));
        if (total_cost < best_cost) {
          best_cost = total_cost;
          best_thr = refined_thr;
          best_ctx = cluster_map;
        }
        return true;
      },
      "JpegCtxOpt"));

  JXL_RETURN_IF_ERROR(ConvertToBlockCtxMap(best_thr, best_ctx, *opt_data,
                                           cfl_ctx, ctx_map));
  return true;
}

namespace {

// Same algorithm as the CfL search in `ComputeJPEGTranscodingData`
// (`enc_frame.cc`) but self-contained: computes CfL maps and scaled
// `qtables` from the JPEG data alone so the planner can run before
// the main transcoding function.
Status ComputeCflForPlanner(const jpeg::JPEGData& jpeg_data,
                            const std::array<int, 3>& jpeg_c_map,
                            size_t xsize_blocks, size_t ysize_blocks,
                            JxlMemoryManager* memory_manager,
                            ImageSB& ytox_map, ImageSB& ytob_map,
                            int32_t scaled_qtable_out[3][kDCTBlockSize]) {
  // Compute quant tables (not transposed, for CfL ratio computation).
  std::vector<int> qt(kDCTBlockSize * 3);
  for (size_t c = 0; c < 3; c++) {
    size_t jpeg_c = jpeg_c_map[c];
    const int32_t* quant =
        jpeg_data.quant[jpeg_data.components[jpeg_c].quant_idx].values.data();
    for (size_t y = 0; y < 8; y++) {
      for (size_t x = 0; x < 8; x++) {
        qt[kDCTBlockSize * c + 8 * x + y] = quant[8 * y + x];
      }
    }
  }
  // `scaled_qtable[c][pos] = qt[luma] / qt[c]` (fixed-point).
  for (size_t c = 0; c < 3; c++) {
    for (size_t y = 0; y < 8; y++) {
      for (size_t x = 0; x < 8; x++) {
        int coeffpos = y * 8 + x;
        scaled_qtable_out[c][8 * x + y] =
            (1 << kCFLFixedPointPrecision) * qt[kDCTBlockSize + coeffpos] /
            qt[kDCTBlockSize * c + coeffpos];
      }
    }
  }

  // Allocate CfL maps.
  size_t num_tiles_x = DivCeil(xsize_blocks, kColorTileDimInBlocks);
  size_t num_tiles_y = DivCeil(ysize_blocks, kColorTileDimInBlocks);
  JXL_ASSIGN_OR_RETURN(ytox_map, ImageSB::Create(memory_manager,
                                                  num_tiles_x, num_tiles_y));
  JXL_ASSIGN_OR_RETURN(ytob_map, ImageSB::Create(memory_manager,
                                                  num_tiles_x, num_tiles_y));

  auto jpeg_row = [&](size_t c, size_t y) -> const int16_t* {
    return jpeg_data.components[jpeg_c_map[c]].coeffs.data() +
           jpeg_data.components[jpeg_c_map[c]].width_in_blocks *
               kDCTBlockSize * y;
  };

  // Use default `ColorCorrelation` base values (JPEG-compatible defaults).
  const float kScale = kDefaultColorFactor;
  const int kOffset = 127;
  ColorCorrelation base_cc;
  const float kBaseX = base_cc.YtoXRatio(0);
  const float kBaseB = base_cc.YtoBRatio(0);

  for (size_t c : {0, 2}) {
    ImageSB& map = (c == 0 ? ytox_map : ytob_map);
    const float kBase = (c == 0) ? kBaseX : kBaseB;
    const float kZeroThresh =
        kScale * kZeroBiasDefault[c] * 0.9999f;

    for (size_t ty = 0; ty < num_tiles_y; ++ty) {
      int8_t* JXL_RESTRICT row_out = map.Row(ty);
      for (size_t tx = 0; tx < num_tiles_x; ++tx) {
        const size_t y0 = ty * kColorTileDimInBlocks;
        const size_t x0 = tx * kColorTileDimInBlocks;
        const size_t y1 = std::min(ysize_blocks,
                                   (ty + 1) * kColorTileDimInBlocks);
        const size_t x1 = std::min(xsize_blocks,
                                   (tx + 1) * kColorTileDimInBlocks);
        int32_t d_num_zeros[257] = {0};
        for (size_t y = y0; y < y1; ++y) {
          const int16_t* JXL_RESTRICT row_m = jpeg_row(1, y);
          const int16_t* JXL_RESTRICT row_s = jpeg_row(c, y);
          for (size_t x = x0; x < x1; ++x) {
            for (size_t coeffpos = 1; coeffpos < kDCTBlockSize; coeffpos++) {
              const float scaled_m =
                  row_m[x * kDCTBlockSize + coeffpos] *
                  scaled_qtable_out[c][coeffpos] *
                  (1.0f / (1 << kCFLFixedPointPrecision));
              const float scaled_s =
                  kScale * row_s[x * kDCTBlockSize + coeffpos] +
                  (kOffset - kBase * kScale) * scaled_m;
              if (std::abs(scaled_m) > 1e-8f) {
                float from, to;
                if (scaled_m > 0) {
                  from = (scaled_s - kZeroThresh) / scaled_m;
                  to = (scaled_s + kZeroThresh) / scaled_m;
                } else {
                  from = (scaled_s + kZeroThresh) / scaled_m;
                  to = (scaled_s - kZeroThresh) / scaled_m;
                }
                if (from < 0.0f) from = 0.0f;
                if (to > 255.0f) to = 255.0f;
                if (from <= to) {
                  d_num_zeros[static_cast<int>(std::ceil(from))]++;
                  d_num_zeros[static_cast<int>(std::floor(to + 1))]--;
                }
              }
            }
          }
        }
        // Find best CfL factor via prefix-sum maximum.
        int best = 0;
        int32_t best_sum = 0;
        int32_t val = 0;
        for (int i = 0; i < 256; ++i) {
          val += d_num_zeros[i];
          if (val > best_sum) {
            best_sum = val;
            best = i;
          }
        }
        int32_t offset_sum = 0;
        for (int i = 0; i <= kOffset; ++i) {
          offset_sum += d_num_zeros[i];
        }
        row_out[tx] = 0;
        if (best_sum > offset_sum + 1) {
          row_out[tx] = best - kOffset;
        }
      }
    }
  }
  return true;
}

}  // namespace

Status PlanJPEGPassAwareRecompression(JxlMemoryManager* memory_manager,
                                      const jpeg::JPEGData& jpeg_data,
                                      SpeedTier speed_tier,
                                      const JpegCflContext& cfl_ctx,
                                      JPEGPassEncodingPlan& plan,
                                      ThreadPool* pool) {
  fprintf(stderr, "PLANNER: Starting pass-aware recompression planning\n");
  auto start_total = std::chrono::high_resolution_clock::now();
  fflush(stderr);
  // Force the sophisticated search path: clamp `speed_tier` to at most
  // `kKitten` so that `FromSpeedTier` returns meaningful effort params.
  SpeedTier effective_tier = std::min(speed_tier, SpeedTier::kKitten);
  const JPEGCtxEffortParams effort =
      JPEGCtxEffortParams::FromSpeedTier(effective_tier);

  // Determine colour layout from the JPEG data.
  bool is_gray = (jpeg_data.components.size() == 1);
  // Determine color transform. JPEG recompression always uses YCbCr for
  // multi-component, kNone for grayscale.
  ColorTransform ct = is_gray ? ColorTransform::kNone : ColorTransform::kYCbCr;
  auto jpeg_c_map = JpegOrder(ct, is_gray);

  bool cfl_possible = !is_gray && (jpeg_data.components.size() == 3);
  bool cfl_enabled = cfl_ctx.enabled && cfl_possible;

  // Build CfL context for the optimizer.  When CfL is active we compute
  // the CfL maps ourselves so the optimizer can model residual coefficients.
  ImageSB ytox_map;
  ImageSB ytob_map;
  int32_t scaled_qtable[3][kDCTBlockSize] = {};
  JpegCflContext planner_cfl = cfl_ctx;
  if (cfl_enabled) {
    auto start_cfl = std::chrono::high_resolution_clock::now();
    size_t xsize_blocks = jpeg_data.components[jpeg_c_map[0]].width_in_blocks;
    size_t ysize_blocks = jpeg_data.components[jpeg_c_map[0]].height_in_blocks;
    JXL_RETURN_IF_ERROR(ComputeCflForPlanner(
        jpeg_data, jpeg_c_map, xsize_blocks, ysize_blocks, memory_manager,
        ytox_map, ytob_map, scaled_qtable));
    planner_cfl.cfl_map[0] = &ytox_map;
    planner_cfl.cfl_map[1] = &ytob_map;
    planner_cfl.scaled_qtable[0] = scaled_qtable[0];
    planner_cfl.scaled_qtable[1] = scaled_qtable[2];
    auto end_cfl = std::chrono::high_resolution_clock::now();
    fprintf(stderr, "PLANNER: ComputeCflForPlanner took %.2f ms\n",
            std::chrono::duration<double, std::milli>(end_cfl - start_cfl).count());
  }

  auto start_opt_data = std::chrono::high_resolution_clock::now();
  auto opt_data = std::make_shared<JPEGOptData>();
  JXL_RETURN_IF_ERROR(opt_data->BuildFromJPEG(jpeg_data, effort.ac_hist_model,
                                               planner_cfl, pool));
  auto end_opt_data = std::chrono::high_resolution_clock::now();
  fprintf(stderr, "PLANNER: BuildFromJPEG took %.2f ms\n",
          std::chrono::duration<double, std::milli>(end_opt_data - start_opt_data).count());

  auto start_search = std::chrono::high_resolution_clock::now();
  JXL_ASSIGN_OR_RETURN(PassSearchResult result,
                        SearchPassAwareContextModel(opt_data, effort, pool));
  auto end_search = std::chrono::high_resolution_clock::now();
  fprintf(stderr, "PLANNER: SearchPassAwareContextModel took %.2f ms (%u passes)\n",
          std::chrono::duration<double, std::milli>(end_search - start_search).count(),
          result.num_passes);

  JXL_DEBUG_V(2,
              "Pass-aware search: %u passes, %u clusters, "
              "ac=%.2f nz=%.2f overhead=%.2f total=%.2f\n",
              result.num_passes, result.num_clusters,
              bit_cost(result.ac_cost), bit_cost(result.nz_cost),
              bit_cost(result.signalling_overhead),
              bit_cost(result.total_cost));

  // Build Passes struct: all zero-shift spatial passes, no downsampling.
  plan.num_passes = result.num_passes;
  plan.passes.num_passes = result.num_passes;
  plan.passes.num_downsample = 0;
  for (uint32_t i = 0; i < result.num_passes; ++i) {
    plan.passes.shift[i] = 0;
  }

  // Convert thresholds + ctx_map into BlockCtxMap.
  JXL_RETURN_IF_ERROR(ConvertToBlockCtxMap(result.thresholds, result.ctx_map,
                                           *opt_data, planner_cfl,
                                           plan.block_ctx_map));

  // Copy pass assignment.
  for (size_t c = 0; c < 3; ++c) {
    plan.pass_assignment[c] = std::move(result.pass_assignment[c]);
  }

  auto end_total = std::chrono::high_resolution_clock::now();
  fprintf(stderr, "PLANNER: Total integration planning took %.2f ms\n",
          std::chrono::duration<double, std::milli>(end_total - start_total).count());
  return true;
}

}  // namespace jxl
