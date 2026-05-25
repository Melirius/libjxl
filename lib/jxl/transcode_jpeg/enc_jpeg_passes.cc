// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Experimental pass-aware context-model search for JPEG lossless
// recompression.
//
// The classic optimizer in `enc_jpeg_frame.cc` searches one global context map
// over the canonical AC stream. This file implements two related experimental
// planner lanes that instead reason about multiple progressive AC passes:
//
// `SearchPassAwareContextModel`
//   Assigns blocks to passes, rebuilds a pass-local AC stream, clusters the
//   resulting `(cell, pass)` contexts, and evaluates thresholds on that
//   pass-aware model.
//
// `SearchBiclusteredContextModel`
//   Reuses the same pass assignment and threshold search, but also materializes
//   a fixed `(row, pass, slice)` histogram lattice used to score an initial
//   biclustering-style objective. In the current prototype this still reuses
//   the pass-aware row clustering path; it is the scaffolding for the more
//   ambitious hierarchical biclustering experiment described in
//   `plans/Passes_histo_clustering.md`.
//
// Internal helpers are grouped into four layers:
//
// `ActiveRawBins`, `AssignPassesGreedy`, `BuildPassStream`
//   Build the pass-local view of the AC stream and the block->pass assignment.
//
// `ClusterContextsPassAware`, `EvaluatePassAwareModel`
//   Cluster and score the pass-aware `(cell, pass)` contexts.
//
// `RowSliceHistograms`, `BuildRowSliceHistograms`, `EvaluateBiclusterState`
//   Materialize and score the richer `(row, pass, zdc/pb)` biclustering state.
//
// `SearchPassAwareContextModel`, `SearchBiclusteredContextModel`
//   Drive the candidate search over thresholds and pick the best-scoring model.

#include "lib/jxl/transcode_jpeg/enc_jpeg_passes.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "lib/jxl/base/data_parallel.h"
#include "lib/jxl/enc_ans_params.h"
#include "lib/jxl/frame_dimensions.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_axis_maps.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_cluster.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_histogram.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_assign.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_stream.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_threshold.h"

namespace jxl {

namespace {

using PlannerClock = std::chrono::high_resolution_clock;

// Tiny timing helpers used only for the planner debug prints. We keep them
// file-local so the instrumentation stays lightweight and does not leak into
// the public search API.
int64_t ElapsedNanos(const PlannerClock::time_point& start,
                     const PlannerClock::time_point& end) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(end - start)
      .count();
}

double NanosToMs(int64_t ns) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::nanoseconds(ns))
      .count();
}

using SparseHistogram = std::vector<std::unordered_map<uint32_t, uint32_t>>;

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

void BlockDCIndices(const JPEGOptData& d, uint32_t c, uint32_t b, uint32_t* dc0,
                    uint32_t* dc1, uint32_t* dc2);
uint32_t BlockCell(const JPEGOptData& d, const AxisMaps& axis_maps,
                   const ThresholdSet& thresholds, uint32_t c, uint32_t b);
// Small union-find helper for the agglomerative merge loop in
// `ClusterContextsPassAware`.
uint32_t FindRoot(std::vector<uint32_t>& parent, uint32_t x) {
  uint32_t root = x;
  while (parent[root] != root) {
    root = parent[root];
  }
  while (parent[x] != x) {
    const uint32_t next = parent[x];
    parent[x] = root;
    x = next;
  }
  return root;
}

struct PrunedCtxMapResult {
  ThresholdSet thresholds;
  ContextMap ctx_map;
};

PrunedCtxMapResult PruneDeadThresholdsFromCtxMap(const ThresholdSet& thresholds,
                                                 ContextMap ctx_map,
                                                 uint32_t channels) {
  PrunedCtxMapResult out;
  out.thresholds = thresholds;
  out.ctx_map = std::move(ctx_map);
  ThresholdSet& T = out.thresholds;

  if (channels == 1 && T.TCb().empty() && T.TCr().empty()) {
    const uint32_t old_n = static_cast<uint32_t>(T.TY().size() + 1);
    Thresholds new_thr;
    new_thr.reserve(T.TY().size());
    std::vector<uint32_t> old_from_new = {0};
    for (uint32_t t = 0; t < T.TY().size(); ++t) {
      if (out.ctx_map[t] != out.ctx_map[t + 1]) {
        old_from_new.push_back(t + 1);
        new_thr.push_back(T.TY()[t]);
      }
    }
    T.TY().swap(new_thr);
    const uint32_t new_n = static_cast<uint32_t>(T.TY().size() + 1);
    ContextMap new_ctx_map(channels * new_n, 0);
    for (uint32_t c = 0; c < channels; ++c) {
      const uint32_t old_base = c * old_n;
      const uint32_t new_base = c * new_n;
      for (uint32_t x = 0; x < new_n; ++x) {
        new_ctx_map[new_base + x] = out.ctx_map[old_base + old_from_new[x]];
      }
    }
    out.ctx_map.swap(new_ctx_map);
    return out;
  }

  const uint32_t sizes[3] = {static_cast<uint32_t>(T.TY().size() + 1),
                             static_cast<uint32_t>(T.TCb().size() + 1),
                             static_cast<uint32_t>(T.TCr().size() + 1)};
  const uint32_t num_cells_init = sizes[0] * sizes[1] * sizes[2];
  const uint32_t axis_stride[3] = {1, sizes[0] * sizes[2], sizes[0]};
  std::array<std::vector<uint32_t>, kNumCh> old_from_new = {{{0}, {0}, {0}}};

  for (uint32_t axis = 0; axis < kNumCh; ++axis) {
    Thresholds& thr = T.T[axis];
    const uint32_t ax1 = (axis + 1) % 3;
    const uint32_t ax2 = (axis + 2) % 3;
    Thresholds new_thr;
    new_thr.reserve(thr.size());
    std::vector<uint32_t>& ofn = old_from_new[axis];
    auto add_active = [&](uint32_t t) {
      uint32_t b[3] = {};
      b[axis] = t;
      for (uint32_t c = 0; c < channels; ++c) {
        const uint32_t c_base = c * num_cells_init;
        for (uint32_t k1 = 0; k1 < sizes[ax1]; ++k1) {
          b[ax1] = k1;
          for (uint32_t k2 = 0; k2 < sizes[ax2]; ++k2) {
            b[ax2] = k2;
            const uint32_t gl = (b[1] * sizes[2] + b[2]) * sizes[0] + b[0];
            if (out.ctx_map[c_base + gl] !=
                out.ctx_map[c_base + gl + axis_stride[axis]]) {
              ofn.push_back(t + 1);
              new_thr.push_back(thr[t]);
              return;
            }
          }
        }
      }
    };

    for (uint32_t t = 0; t < thr.size(); ++t) {
      add_active(t);
    }
    thr.swap(new_thr);
  }

  const uint32_t new_sizes[3] = {static_cast<uint32_t>(T.TY().size() + 1),
                                 static_cast<uint32_t>(T.TCb().size() + 1),
                                 static_cast<uint32_t>(T.TCr().size() + 1)};
  const uint32_t new_num_cells = new_sizes[0] * new_sizes[1] * new_sizes[2];
  ContextMap new_ctx_map(channels * new_num_cells, 0);
  for (uint32_t c = 0; c < channels; ++c) {
    const uint32_t old_base = c * num_cells_init;
    const uint32_t new_base = c * new_num_cells;
    for (uint32_t Cb = 0; Cb < new_sizes[1]; ++Cb) {
      for (uint32_t Cr = 0; Cr < new_sizes[2]; ++Cr) {
        for (uint32_t Y = 0; Y < new_sizes[0]; ++Y) {
          const uint32_t g_old =
              (old_from_new[1][Cb] * sizes[2] + old_from_new[2][Cr]) *
                  sizes[0] +
              old_from_new[0][Y];
          const uint32_t g_new = (Cb * new_sizes[2] + Cr) * new_sizes[0] + Y;
          new_ctx_map[new_base + g_new] = out.ctx_map[old_base + g_old];
        }
      }
    }
  }
  out.ctx_map.swap(new_ctx_map);
  return out;
}

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

// --- Biclustering-state helpers ------------------------------------------------

NZBlockCache BuildNZBlockCache(const JPEGOptData& d,
                              const PassAssignment& pass_assignment,
                              uint32_t num_passes) {
  NZBlockCache cache;
  cache.num_passes = num_passes;
  for (uint32_t c = 0; c < d.channels; ++c) {
    cache.pred_bucket[c].resize(static_cast<size_t>(d.num_blocks[c]) *
                                num_passes);
    for (uint32_t y = 0; y < d.block_grid_h[c]; ++y) {
      for (uint32_t x = 0; x < d.block_grid_w[c]; ++x) {
        const uint32_t b = y * d.block_grid_w[c] + x;
        const uint32_t b_top = (y - 1) * d.block_grid_w[c] + x;
        const uint32_t b_left = y * d.block_grid_w[c] + (x - 1);
        const uint32_t nz_top = (y > 0) ? d.block_nonzeros[c][b_top] : 0u;
        const uint32_t nz_left = (x > 0) ? d.block_nonzeros[c][b_left] : 0u;
        const uint8_t nz_top_pass = (y > 0) ? pass_assignment[c][b_top] : 255;
        const uint8_t nz_left_pass =
            (x > 0) ? pass_assignment[c][b_left] : 255;
        for (uint32_t p = 0; p < num_passes; ++p) {
          const uint32_t pass_nz_top = (nz_top_pass == p) ? nz_top : 0u;
          const uint32_t pass_nz_left = (nz_left_pass == p) ? nz_left : 0u;
          uint32_t predicted_nz;
          if (x == 0 && y == 0) {
            predicted_nz = 32u;
          } else if (x == 0) {
            predicted_nz = pass_nz_top;
          } else if (y == 0) {
            predicted_nz = pass_nz_left;
          } else {
            predicted_nz = (pass_nz_top + pass_nz_left + 1u) / 2u;
          }
          cache.pred_bucket[c][static_cast<size_t>(b) * num_passes + p] =
              static_cast<uint8_t>((predicted_nz < 8) ? predicted_nz
                                                      : (4 + predicted_nz / 2));
        }
      }
    }
  }
  return cache;
}

void MoveBlockRowContributions(const JPEGOptData& d,
                               const NZBlockCache& nz_cache,
                               const PassAssignment& pass_assignment,
                               uint32_t num_passes, uint32_t c, uint32_t b,
                               uint32_t old_row, uint32_t new_row,
                               RowSliceHistograms* rows) {
  if (old_row == new_row) return;
  const uint32_t pass = pass_assignment[c][b];
  for (uint32_t pi = d.block_offsets[c][b]; pi < d.block_offsets[c][b + 1];
       ++pi) {
    const CompactACEvent ac_event = d.FromBin(d.block_bins[c][pi]);
    const SignallingHistSymbol hist_symbol = d.SignallingHistSymbolFromSymbol(
        d.ACHistogram().dense_to_zdcvalue[ac_event.hist_bin]);
    const size_t old_idx =
        (static_cast<size_t>(old_row) * num_passes + pass) *
            kZeroDensityContextCount +
        ac_event.zdc;
    const size_t new_idx =
        (static_cast<size_t>(new_row) * num_passes + pass) *
            kZeroDensityContextCount +
        ac_event.zdc;
    rows->ac_hist[old_idx].Subtract(hist_symbol.token);
    --rows->ac_total[old_idx];
    rows->ac_hist[new_idx].Add(hist_symbol.token);
    ++rows->ac_total[new_idx];
  }

  for (uint32_t p = 0; p < num_passes; ++p) {
    const uint32_t pb =
        nz_cache.pred_bucket[c][static_cast<size_t>(b) * num_passes + p];
    const uint32_t nz = (pass == p) ? d.block_nonzeros[c][b] : 0u;
    const size_t old_idx =
        (static_cast<size_t>(old_row) * num_passes + p) *
            kJPEGNonZeroBuckets +
        pb;
    const size_t new_idx =
        (static_cast<size_t>(new_row) * num_passes + p) *
            kJPEGNonZeroBuckets +
        pb;
    rows->nz_hist[old_idx].Subtract(nz);
    --rows->nz_total[old_idx];
    rows->nz_hist[new_idx].Add(nz);
    ++rows->nz_total[new_idx];
  }
}

// Builds the fixed `(row, pass, slice)` histogram lattice for one threshold
// set and one block-to-pass assignment. AC slices are stored already regrouped
// by signalling token within a fixed `zdc`, so the per-slice alphabet stays
// small (`kACTokenCount`) and memory use stays bounded.
StatusOr<RowSliceState> BuildRowSliceState(
    const JPEGOptData& d, const ThresholdSet& thresholds,
    const PassAssignment& pass_assignment, uint32_t num_passes,
    const NZBlockCache& nz_cache) {
  if (d.AC_hist_model != JPEGTranscodeACModel::kToken420) {
    return JXL_FAILURE(
        "Biclustered search currently supports only kToken420 AC histograms");
  }
  AxisMaps axis_maps(d);
  axis_maps.Update(thresholds);
  const uint32_t n0 = static_cast<uint32_t>(thresholds.TY().size() + 1);
  const uint32_t num_cells =
      n0 * static_cast<uint32_t>(thresholds.TCb().size() + 1) *
      static_cast<uint32_t>(thresholds.TCr().size() + 1);
  const uint32_t total_rows = d.channels * num_cells;

  RowSliceState state;
  RowSliceHistograms& rows = state.rows;
  rows.num_channels = d.channels;
  rows.num_cells = num_cells;
  rows.num_passes = num_passes;
  rows.ac_hist.resize(static_cast<size_t>(total_rows) * num_passes *
                      kZeroDensityContextCount);
  rows.ac_total.assign(static_cast<size_t>(total_rows) * num_passes *
                           kZeroDensityContextCount,
                       0);
  rows.nz_hist.resize(static_cast<size_t>(total_rows) * num_passes *
                      kJPEGNonZeroBuckets);
  rows.nz_total.assign(static_cast<size_t>(total_rows) * num_passes *
                           kJPEGNonZeroBuckets,
                       0);
  for (uint32_t c = 0; c < d.channels; ++c) {
    state.block_rows[c].resize(d.num_blocks[c]);
  }

  for (uint32_t c = 0; c < d.channels; ++c) {
    for (uint32_t b = 0; b < d.num_blocks[c]; ++b) {
      const uint32_t cell = BlockCell(d, axis_maps, thresholds, c, b);
      const uint32_t row = c * num_cells + cell;
      state.block_rows[c][b] = static_cast<uint16_t>(row);
      const uint32_t pass = pass_assignment[c][b];

      for (uint32_t pi = d.block_offsets[c][b]; pi < d.block_offsets[c][b + 1];
           ++pi) {
        const CompactACEvent ac_event = d.FromBin(d.block_bins[c][pi]);
        const size_t idx =
            (static_cast<size_t>(row) * num_passes + pass) *
                kZeroDensityContextCount +
            ac_event.zdc;
        const SignallingHistSymbol hist_symbol =
            d.SignallingHistSymbolFromSymbol(
                d.ACHistogram().dense_to_zdcvalue[ac_event.hist_bin]);
        rows.ac_hist[idx].Add(hist_symbol.token);
        ++rows.ac_total[idx];
      }
    }
  }

  for (uint32_t c = 0; c < d.channels; ++c) {
    for (uint32_t y = 0; y < d.block_grid_h[c]; ++y) {
      for (uint32_t x = 0; x < d.block_grid_w[c]; ++x) {
        uint32_t b = y * d.block_grid_w[c] + x;
        uint32_t cell = BlockCell(d, axis_maps, thresholds, c, b);
        uint32_t row = c * num_cells + cell;
        uint32_t pass = pass_assignment[c][b];

        for (size_t p = 0; p < num_passes; ++p) {
          uint8_t pb = nz_cache.pred_bucket[c][b * num_passes + p];
          size_t idx = (row * num_passes + p) * kJPEGNonZeroBuckets + pb;
          rows.nz_hist[idx].Add(pass == p ? d.block_nonzeros[c][b] : 0u);
          ++rows.nz_total[idx];
        }
      }
    }
  }

  return state;
}

// Reuses an already-built rough row lattice for refined thresholds by moving
// only the blocks whose `(channel, cell)` row id changed.
StatusOr<RowSliceState> RefineRowSliceState(
    const JPEGOptData& d, const ThresholdSet& thresholds,
    const PassAssignment& pass_assignment, uint32_t num_passes,
    const RowSliceState& base_state, const NZBlockCache& nz_cache) {
  if (d.AC_hist_model != JPEGTranscodeACModel::kToken420) {
    return JXL_FAILURE(
        "Biclustered search currently supports only kToken420 AC histograms");
  }
  AxisMaps axis_maps(d);
  axis_maps.Update(thresholds);
  const uint32_t n0 = static_cast<uint32_t>(thresholds.TY().size() + 1);
  const uint32_t num_cells =
      n0 * static_cast<uint32_t>(thresholds.TCb().size() + 1) *
      static_cast<uint32_t>(thresholds.TCr().size() + 1);
  if (num_cells != base_state.rows.num_cells) {
    return BuildRowSliceState(d, thresholds, pass_assignment, num_passes,
                              nz_cache);
  }

  RowSliceState refined_state = base_state;
  for (uint32_t c = 0; c < d.channels; ++c) {
    for (uint32_t b = 0; b < d.num_blocks[c]; ++b) {
      const uint32_t cell = BlockCell(d, axis_maps, thresholds, c, b);
      const uint32_t new_row = c * num_cells + cell;
      const uint32_t old_row = refined_state.block_rows[c][b];
      if (old_row == new_row) continue;
      MoveBlockRowContributions(d, nz_cache, pass_assignment, num_passes, c, b,
                                old_row, new_row, &refined_state.rows);
      refined_state.block_rows[c][b] = static_cast<uint16_t>(new_row);
    }
  }
  return refined_state;
}

template <typename Func>
void ForEachSortedIntersection(const std::vector<uint16_t>& lhs,
                               const std::vector<uint16_t>& rhs, Func&& fn) {
  size_t i = 0;
  size_t j = 0;
  while (i < lhs.size() && j < rhs.size()) {
    if (lhs[i] < rhs[j]) {
      ++i;
    } else if (rhs[j] < lhs[i]) {
      ++j;
    } else {
      fn(lhs[i]);
      ++i;
      ++j;
    }
  }
}

void MergeSortedSliceLists(std::vector<uint16_t>* dst,
                           const std::vector<uint16_t>& src) {
  if (src.empty()) return;
  std::vector<uint16_t> merged;
  merged.reserve(dst->size() + src.size());
  std::set_union(dst->begin(), dst->end(), src.begin(), src.end(),
                 std::back_inserter(merged));
  dst->swap(merged);
}

// Agglomerative row clustering for the biclustering prototype. Rows are the
// original `(channel, cell)` contexts; their per-pass AC/nz slice histograms
// are merged with the same local entropy deltas used by the classic optimizer,
// but now summed over all passes and fixed slice slots.
StatusOr<ClusterResult> ClusterRowsBiclustered(
    const JPEGOptData& d, const RowSliceHistograms& rows,
    uint32_t row_budget) {
  const uint32_t total_rows = rows.num_channels * rows.num_cells;
  if (total_rows == 0) {
    ClusterResult out;
    out.num_clusters = 1;
    return out;
  }

  const uint32_t ac_slices_per_row =
      rows.num_passes * kZeroDensityContextCount;
  const uint32_t nz_slices_per_row =
      rows.num_passes * kJPEGNonZeroBuckets;
  std::vector<std::vector<uint16_t>> active_ac_slices(total_rows);
  std::vector<std::vector<uint16_t>> active_nz_slices(total_rows);
  std::vector<uint8_t> row_was_active(total_rows, 0);
  std::vector<uint32_t> active;
  active.reserve(total_rows);
  for (uint32_t row = 0; row < total_rows; ++row) {
    auto& ac_list = active_ac_slices[row];
    auto& nz_list = active_nz_slices[row];
    const size_t ac_base = static_cast<size_t>(row) * ac_slices_per_row;
    const size_t nz_base = static_cast<size_t>(row) * nz_slices_per_row;
    for (uint32_t slice = 0; slice < ac_slices_per_row; ++slice) {
      if (rows.ac_total[ac_base + slice] != 0) {
        ac_list.push_back(static_cast<uint16_t>(slice));
      }
    }
    for (uint32_t slice = 0; slice < nz_slices_per_row; ++slice) {
      if (rows.nz_total[nz_base + slice] != 0) {
        nz_list.push_back(static_cast<uint16_t>(slice));
      }
    }
    if (!ac_list.empty() || !nz_list.empty()) {
      row_was_active[row] = 1;
      active.push_back(row);
    }
  }

  ClusterResult out;
  out.ctx_map.assign(total_rows, 0);
  if (active.empty()) {
    out.num_clusters = 1;
    return out;
  }
  if (active.size() <= row_budget) {
    out.num_clusters = static_cast<uint32_t>(std::max<size_t>(1, active.size()));
    for (uint32_t i = 0; i < active.size(); ++i) {
      out.ctx_map[active[i]] = static_cast<uint8_t>(i);
    }
    return out;
  }

  std::vector<uint32_t> parent(total_rows);
  for (uint32_t i = 0; i < total_rows; ++i) parent[i] = i;

  std::vector<DenseHistogram<kACTokenCount>> ac_cluster_hist = rows.ac_hist;
  std::vector<uint32_t> ac_cluster_total = rows.ac_total;
  std::vector<DenseHistogram<kJPEGNonZeroRange>> nz_cluster_hist = rows.nz_hist;
  std::vector<uint32_t> nz_cluster_total = rows.nz_total;

  std::vector<FixedPointCost> deltas(static_cast<size_t>(total_rows) * total_rows,
                                     0);
  auto delta_ref = [&](uint32_t a, uint32_t b) -> FixedPointCost& {
    if (a > b) std::swap(a, b);
    return deltas[static_cast<size_t>(a) * total_rows + b];
  };

  auto merge_delta = [&](uint32_t a, uint32_t b) {
    FixedPointCost delta = 0;
    ForEachSortedIntersection(active_ac_slices[a], active_ac_slices[b],
                              [&](uint16_t slice) {
        const size_t ia = static_cast<size_t>(a) * ac_slices_per_row + slice;
        const size_t ib = static_cast<size_t>(b) * ac_slices_per_row + slice;
        const uint32_t total_a = ac_cluster_total[ia];
        const uint32_t total_b = ac_cluster_total[ib];
        if (total_a != 0 && total_b != 0) {
          delta += d.ftab[total_a + total_b] - d.ftab[total_a] -
                   d.ftab[total_b];
          for (uint32_t t = 0; t < kACTokenCount; ++t) {
            const uint32_t ca = ac_cluster_hist[ia][t];
            const uint32_t cb = ac_cluster_hist[ib][t];
            if (ca == 0 || cb == 0) continue;
            delta -= d.ftab[ca + cb] - d.ftab[ca] - d.ftab[cb];
          }
        }
      });
    ForEachSortedIntersection(active_nz_slices[a], active_nz_slices[b],
                              [&](uint16_t slice) {
        const size_t ia = static_cast<size_t>(a) * nz_slices_per_row + slice;
        const size_t ib = static_cast<size_t>(b) * nz_slices_per_row + slice;
        const uint32_t total_a = nz_cluster_total[ia];
        const uint32_t total_b = nz_cluster_total[ib];
        if (total_a != 0 && total_b != 0) {
          delta += d.NZFTab(total_a + total_b) - d.NZFTab(total_a) -
                   d.NZFTab(total_b);
          for (uint32_t nz = 0; nz < kJPEGNonZeroRange; ++nz) {
            const uint32_t ca = nz_cluster_hist[ia][nz];
            const uint32_t cb = nz_cluster_hist[ib][nz];
            if (ca == 0 || cb == 0) continue;
            delta -= d.NZFTab(ca + cb) - d.NZFTab(ca) - d.NZFTab(cb);
          }
        }
      });
    return delta;
  };

  for (size_t i = 0; i + 1 < active.size(); ++i) {
    for (size_t j = i + 1; j < active.size(); ++j) {
      delta_ref(active[i], active[j]) = merge_delta(active[i], active[j]);
    }
  }

  while (active.size() > row_budget && active.size() > 1) {
    size_t best_i = 0;
    size_t best_j = 1;
    FixedPointCost best_delta = delta_ref(active[0], active[1]);
    for (size_t i = 0; i + 1 < active.size(); ++i) {
      for (size_t j = i + 1; j < active.size(); ++j) {
        const FixedPointCost delta = delta_ref(active[i], active[j]);
        if (delta < best_delta) {
          best_delta = delta;
          best_i = i;
          best_j = j;
        }
      }
    }

    const uint32_t keep = active[best_i];
    const uint32_t drop = active[best_j];
    for (uint16_t slice : active_ac_slices[drop]) {
      const size_t ik = static_cast<size_t>(keep) * ac_slices_per_row + slice;
      const size_t id = static_cast<size_t>(drop) * ac_slices_per_row + slice;
      ac_cluster_hist[ik].AddHistogram(ac_cluster_hist[id]);
      ac_cluster_total[ik] += ac_cluster_total[id];
    }
    for (uint16_t slice : active_nz_slices[drop]) {
      const size_t ik = static_cast<size_t>(keep) * nz_slices_per_row + slice;
      const size_t id = static_cast<size_t>(drop) * nz_slices_per_row + slice;
      nz_cluster_hist[ik].AddHistogram(nz_cluster_hist[id]);
      nz_cluster_total[ik] += nz_cluster_total[id];
    }
    MergeSortedSliceLists(&active_ac_slices[keep], active_ac_slices[drop]);
    MergeSortedSliceLists(&active_nz_slices[keep], active_nz_slices[drop]);
    active_ac_slices[drop].clear();
    active_nz_slices[drop].clear();
    parent[drop] = keep;
    active.erase(active.begin() + best_j);
    for (uint32_t other : active) {
      if (other == keep) continue;
      delta_ref(keep, other) = merge_delta(keep, other);
    }
  }

  out.num_clusters = static_cast<uint32_t>(active.size());
  std::unordered_map<uint32_t, uint32_t> cluster_id;
  cluster_id.reserve(active.size());
  for (uint32_t i = 0; i < active.size(); ++i) {
    cluster_id[active[i]] = i;
  }
  for (uint32_t row = 0; row < total_rows; ++row) {
    if (!row_was_active[row]) {
      out.ctx_map[row] = 0;
      continue;
    }
    const uint32_t root = FindRoot(parent, row);
    out.ctx_map[row] = static_cast<uint8_t>(cluster_id[root]);
  }
  return out;
}

// Appends the non-empty clustered AC and nz slice histograms for one pass to a
// flat generic `Histogram` pool so the encoder's `ClusterHistograms` helper can
// be used directly on the biclustering prototype state.
void BuildBiclusterPassHistograms(
    uint32_t pass, uint32_t num_passes, uint32_t num_row_clusters,
    const std::vector<DenseHistogram<kACTokenCount>>& ac_cluster_hist,
    const std::vector<uint32_t>& ac_cluster_total,
    const std::vector<DenseHistogram<kJPEGNonZeroRange>>& nz_cluster_hist,
    const std::vector<uint32_t>& nz_cluster_total,
    std::vector<Histogram>* histograms, std::vector<int32_t>* ac_hist_index,
    std::vector<int32_t>* nz_hist_index) {
  histograms->clear();
  histograms->reserve(static_cast<size_t>(num_row_clusters) *
                      (kZeroDensityContextCount + kJPEGNonZeroBuckets));
  if (ac_hist_index != nullptr) {
    ac_hist_index->assign(static_cast<size_t>(num_row_clusters) *
                              kZeroDensityContextCount,
                          -1);
  }
  if (nz_hist_index != nullptr) {
    nz_hist_index->assign(static_cast<size_t>(num_row_clusters) *
                              kJPEGNonZeroBuckets,
                          -1);
  }

  for (uint32_t cluster = 0; cluster < num_row_clusters; ++cluster) {
    for (uint32_t zdc = 0; zdc < kZeroDensityContextCount; ++zdc) {
      const size_t idx = (static_cast<size_t>(cluster) * num_passes + pass) *
                             kZeroDensityContextCount +
                         zdc;
      if (ac_cluster_total[idx] == 0) continue;

      uint32_t max_token = 0;
      for (uint32_t t = 0; t < kACTokenCount; ++t) {
        if (ac_cluster_hist[idx][t] != 0) max_token = t;
      }
      Histogram h(max_token + 1);
      h.total_count = ac_cluster_total[idx];
      for (uint32_t t = 0; t <= max_token; ++t) {
        h.counts[t] = static_cast<ANSHistBin>(ac_cluster_hist[idx][t]);
      }
      if (ac_hist_index != nullptr) {
        (*ac_hist_index)[static_cast<size_t>(cluster) * kZeroDensityContextCount +
                         zdc] = histograms->size();
      }
      histograms->push_back(std::move(h));
    }

    for (uint32_t pb = 0; pb < kJPEGNonZeroBuckets; ++pb) {
      const size_t idx = (static_cast<size_t>(cluster) * num_passes + pass) *
                             kJPEGNonZeroBuckets +
                         pb;
      if (nz_cluster_total[idx] == 0) continue;

      uint32_t max_nz = 0;
      for (uint32_t nz = 0; nz < kJPEGNonZeroRange; ++nz) {
        if (nz_cluster_hist[idx][nz] != 0) max_nz = nz;
      }
      Histogram h(max_nz + 1);
      h.total_count = nz_cluster_total[idx];
      for (uint32_t nz = 0; nz <= max_nz; ++nz) {
        h.counts[nz] = static_cast<ANSHistBin>(nz_cluster_hist[idx][nz]);
      }
      if (nz_hist_index != nullptr) {
        (*nz_hist_index)[static_cast<size_t>(cluster) * kJPEGNonZeroBuckets +
                         pb] = histograms->size();
      }
      histograms->push_back(std::move(h));
    }
  }
}

// Planner-local mirror of the encoder TOC size buckets in `enc_frame.cc`.
// TODO: move to common header with `enc_frame.cc`
constexpr size_t kPlannerGroupSizeOffset[4] = {0, 1024, 17408, 4211712};
constexpr size_t kPlannerTOCBits[4] = {12, 16, 24, 32};

// Approximate section-local overhead beyond entropy-coded symbols:
// - ANS final state / initialization footprint (~32 bits)
// - byte alignment / small stream framing (~0-7 bits)
constexpr FixedPointCost kEstimatedNonEmptyGroupBits = 40 * kFScale;

struct BiclusterPassCostModel {
  std::vector<int16_t> ac_proto_by_slice;
  std::vector<int16_t> nz_proto_by_slice;
  std::vector<std::vector<FixedPointCost>> proto_symbol_cost;
};

size_t TOCBucketForEstimatedSize(size_t group_size_bytes) {
  size_t bucket = 0;
  while (bucket < 3 &&
         group_size_bytes >= kPlannerGroupSizeOffset[bucket + 1]) {
    ++bucket;
  }
  return bucket;
}

std::vector<FixedPointCost> BuildHistogramCostTable(const Histogram& h) {
  std::vector<FixedPointCost> costs(h.counts.size(), 0);
  if (h.total_count == 0) return costs;
  for (size_t symbol = 0; symbol < h.counts.size(); ++symbol) {
    const ANSHistBin count = h.counts[symbol];
    if (count <= 0) continue;
    costs[symbol] = static_cast<FixedPointCost>(std::llround(
        std::log2(static_cast<double>(h.total_count) / count) * kFScale));
  }
  return costs;
}

FixedPointCost ComputePassGlobalOverhead() {
  // Additional per-pass global overhead beyond the per-group streams:
  // - coefficient reordering (roughly 3 * log2(63!) bits)
  // - context-map signalling for AC/nz histograms
  // - pass header and nearby bookkeeping
  return 64000 * kFScale;
}

FixedPointCost ComputePassOverhead(const JPEGOptData& d) {
  // Legacy flat estimate kept for the older pass-aware scorer.
  uint32_t groups_x = (d.w_max + 31) / 32;
  uint32_t groups_y = (d.h_max + 31) / 32;
  uint32_t groups = groups_x * groups_y;
  return (groups * 64 + 64000) * kFScale;
}

FixedPointCost EstimateBiclusterGroupOverhead(
    const JPEGOptData& d, const ThresholdSet& thresholds,
    const ContextMap& ctx_map, const RowSliceHistograms& rows,
    const PassAssignment& pass_assignment, uint32_t num_passes,
    const std::vector<BiclusterPassCostModel>& pass_models,
    FixedPointCost cutoff = std::numeric_limits<FixedPointCost>::max()) {
  AxisMaps axis_maps(d);
  axis_maps.Update(thresholds);
  const uint32_t groups_x =
      (d.w_max + static_cast<uint32_t>(kGroupDimInBlocks) - 1) /
      static_cast<uint32_t>(kGroupDimInBlocks);
  const uint32_t groups_y =
      (d.h_max + static_cast<uint32_t>(kGroupDimInBlocks) - 1) /
      static_cast<uint32_t>(kGroupDimInBlocks);
  const uint32_t num_groups = groups_x * groups_y;
  std::vector<FixedPointCost> group_bits(static_cast<size_t>(num_passes) *
                                             num_groups,
                                         0);
  std::vector<uint8_t> group_nonempty(static_cast<size_t>(num_passes) *
                                          num_groups,
                                      0);

  auto group_slot = [&](uint32_t pass, uint32_t group) {
    return static_cast<size_t>(pass) * num_groups + group;
  };
  auto mark_symbol = [&](uint32_t pass, uint32_t group, FixedPointCost bits) {
    const size_t slot = group_slot(pass, group);
    group_bits[slot] += bits;
    group_nonempty[slot] = 1;
  };

  for (uint32_t c = 0; c < d.channels; ++c) {
    for (uint32_t y = 0; y < d.block_grid_h[c]; ++y) {
      for (uint32_t x = 0; x < d.block_grid_w[c]; ++x) {
        const uint32_t b = y * d.block_grid_w[c] + x;
        const uint32_t row = c * rows.num_cells + BlockCell(d, axis_maps, thresholds, c, b);
        const uint32_t cluster = ctx_map[row];
        const uint32_t pass = pass_assignment[c][b];
        const uint32_t group_x = (x << d.hshift[c]) /
                                 static_cast<uint32_t>(kGroupDimInBlocks);
        const uint32_t group_y = (y << d.vshift[c]) /
                                 static_cast<uint32_t>(kGroupDimInBlocks);
        const uint32_t group = group_y * groups_x + group_x;

        const BiclusterPassCostModel& ac_model = pass_models[pass];
        for (uint32_t pi = d.block_offsets[c][b]; pi < d.block_offsets[c][b + 1];
             ++pi) {
          const CompactACEvent ac_event = d.FromBin(d.block_bins[c][pi]);
          const SignallingHistSymbol hist_symbol = d.SignallingHistSymbolFromSymbol(
              d.ACHistogram().dense_to_zdcvalue[ac_event.hist_bin]);
          const int16_t proto = ac_model.ac_proto_by_slice
              [static_cast<size_t>(cluster) * kZeroDensityContextCount + ac_event.zdc];
          JXL_DASSERT(proto >= 0);
          JXL_DASSERT(static_cast<size_t>(proto) < ac_model.proto_symbol_cost.size());
          JXL_DASSERT(static_cast<size_t>(hist_symbol.token) <
                      ac_model.proto_symbol_cost[proto].size());
          mark_symbol(pass, group,
                      ac_model.proto_symbol_cost[proto][hist_symbol.token]);
        }

        const uint32_t b_top = (y - 1) * d.block_grid_w[c] + x;
        const uint32_t b_left = y * d.block_grid_w[c] + (x - 1);
        const uint32_t nz_top = (y > 0) ? d.block_nonzeros[c][b_top] : 0u;
        const uint32_t nz_left = (x > 0) ? d.block_nonzeros[c][b_left] : 0u;
        const uint8_t nz_top_pass = (y > 0) ? pass_assignment[c][b_top] : 255;
        const uint8_t nz_left_pass = (x > 0) ? pass_assignment[c][b_left] : 255;
        for (uint32_t p = 0; p < num_passes; ++p) {
          const uint32_t pass_nz_top = (nz_top_pass == p) ? nz_top : 0u;
          const uint32_t pass_nz_left = (nz_left_pass == p) ? nz_left : 0u;
          uint32_t predicted_nz;
          if (x == 0 && y == 0) {
            predicted_nz = 32u;
          } else if (x == 0) {
            predicted_nz = pass_nz_top;
          } else if (y == 0) {
            predicted_nz = pass_nz_left;
          } else {
            predicted_nz = (pass_nz_top + pass_nz_left + 1u) / 2u;
          }
          const uint32_t pb =
              (predicted_nz < 8) ? predicted_nz : (4 + predicted_nz / 2);
          const uint32_t nz = (pass == p) ? d.block_nonzeros[c][b] : 0u;
          const BiclusterPassCostModel& nz_model = pass_models[p];
          const int16_t proto = nz_model.nz_proto_by_slice
              [static_cast<size_t>(cluster) * kJPEGNonZeroBuckets + pb];
          JXL_DASSERT(proto >= 0);
          JXL_DASSERT(static_cast<size_t>(proto) < nz_model.proto_symbol_cost.size());
          JXL_DASSERT(static_cast<size_t>(nz) <
                      nz_model.proto_symbol_cost[proto].size());
          mark_symbol(p, group, nz_model.proto_symbol_cost[proto][nz]);
        }
      }
    }
  }

  FixedPointCost overhead = 0;
  const FixedPointCost kByteScale = 8 * kFScale;
  for (size_t slot = 0; slot < group_bits.size(); ++slot) {
    FixedPointCost section_bits = group_bits[slot];
    if (group_nonempty[slot] != 0) {
      section_bits += kEstimatedNonEmptyGroupBits;
      overhead += kEstimatedNonEmptyGroupBits;
      if (overhead >= cutoff) return overhead;
    }
    const size_t section_bytes =
        section_bits <= 0
            ? 0
            : static_cast<size_t>((section_bits + kByteScale - 1) / kByteScale);
    overhead +=
        static_cast<FixedPointCost>(
            kPlannerTOCBits[TOCBucketForEstimatedSize(section_bytes)]) *
        kFScale;
    if (overhead >= cutoff) return overhead;
  }
  return overhead;
}

// Scores a biclustering prototype state by first collapsing original rows into
// the provided `ctx_map` clusters, then summing entropy and histogram-header
// cost over the resulting pass-local AC and nz slice histograms.
//
// The current prototype does not yet perform the full alternating row/prototype
// agglomeration from the design doc; it uses this evaluator on top of the
// pass-aware row clustering result to estimate the richer objective.
StatusOr<ModelEvaluation> EvaluateBiclusterState(
    const JPEGOptData& d, const ThresholdSet& thresholds, const ContextMap& ctx_map,
    uint32_t num_row_clusters, const PassAssignment& pass_assignment,
    uint32_t num_passes, uint32_t proto_budget_per_pass,
    const RowSliceHistograms& rows,
    std::vector<uint32_t>* num_prototypes_per_pass,
    FixedPointCost cutoff = std::numeric_limits<FixedPointCost>::max()) {
  ModelEvaluation eval;
  const uint32_t total_rows = rows.num_channels * rows.num_cells;
  const size_t ac_slots =
      static_cast<size_t>(num_row_clusters) * num_passes * kZeroDensityContextCount;
  const size_t nz_slots =
      static_cast<size_t>(num_row_clusters) * num_passes * kJPEGNonZeroBuckets;
  std::vector<DenseHistogram<kACTokenCount>> ac_cluster_hist(ac_slots);
  std::vector<uint32_t> ac_cluster_total(ac_slots, 0);
  std::vector<DenseHistogram<kJPEGNonZeroRange>> nz_cluster_hist(nz_slots);
  std::vector<uint32_t> nz_cluster_total(nz_slots, 0);

  for (uint32_t row = 0; row < total_rows; ++row) {
    const uint32_t cluster = ctx_map[row];
    for (uint32_t pass = 0; pass < num_passes; ++pass) {
      for (uint32_t zdc = 0; zdc < kZeroDensityContextCount; ++zdc) {
        const size_t src =
            (static_cast<size_t>(row) * num_passes + pass) * kZeroDensityContextCount +
            zdc;
        const size_t dst = (static_cast<size_t>(cluster) * num_passes + pass) *
                               kZeroDensityContextCount +
                           zdc;
        if (!rows.ac_hist[src].empty()) {
          ac_cluster_hist[dst].AddHistogram(rows.ac_hist[src]);
          ac_cluster_total[dst] += rows.ac_total[src];
        }
      }
      for (uint32_t pb = 0; pb < kJPEGNonZeroBuckets; ++pb) {
        const size_t src =
            (static_cast<size_t>(row) * num_passes + pass) * kJPEGNonZeroBuckets +
            pb;
        const size_t dst = (static_cast<size_t>(cluster) * num_passes + pass) *
                               kJPEGNonZeroBuckets +
                           pb;
        if (!rows.nz_hist[src].empty()) {
          nz_cluster_hist[dst].AddHistogram(rows.nz_hist[src]);
          nz_cluster_total[dst] += rows.nz_total[src];
        }
      }
    }
  }

  num_prototypes_per_pass->assign(num_passes, 0);
  eval.corrected_entropy_cost = 0;
  for (uint32_t cluster = 0; cluster < num_row_clusters; ++cluster) {
    for (uint32_t pass = 0; pass < num_passes; ++pass) {
      for (uint32_t zdc = 0; zdc < kZeroDensityContextCount; ++zdc) {
        const size_t idx = (static_cast<size_t>(cluster) * num_passes + pass) *
                               kZeroDensityContextCount +
                           zdc;
        if (ac_cluster_total[idx] == 0) continue;
        eval.ac_cost += d.ftab[ac_cluster_total[idx]];
        for (uint32_t t = 0; t < kACTokenCount; ++t) {
          if (ac_cluster_hist[idx][t] == 0) continue;
          eval.ac_cost -= d.ftab[ac_cluster_hist[idx][t]];
        }
      }

      for (uint32_t pb = 0; pb < kJPEGNonZeroBuckets; ++pb) {
        const size_t idx = (static_cast<size_t>(cluster) * num_passes + pass) *
                               kJPEGNonZeroBuckets +
                           pb;
        if (nz_cluster_total[idx] == 0) continue;
        eval.nz_cost += d.NZFTab(nz_cluster_total[idx]);
        for (uint32_t nz = 0; nz < kJPEGNonZeroRange; ++nz) {
          eval.nz_cost -= d.NZFTab(nz_cluster_hist[idx][nz]);
        }
      }
    }
  }

  HistogramParams params;
  params.clustering = HistogramParams::ClusteringType::kBest;
  std::vector<BiclusterPassCostModel> pass_models(num_passes);
  std::vector<Histogram> pass_histograms;
  std::vector<Histogram> clustered;
  std::vector<uint32_t> histogram_symbols;
  std::vector<int32_t> ac_hist_index;
  std::vector<int32_t> nz_hist_index;
  const FixedPointCost fixed_pass_overhead =
      ComputePassGlobalOverhead() * num_passes;
  for (uint32_t pass = 0; pass < num_passes; ++pass) {
    BiclusterPassCostModel& pass_model = pass_models[pass];
    pass_model.ac_proto_by_slice.assign(
        static_cast<size_t>(num_row_clusters) * kZeroDensityContextCount, -1);
    pass_model.nz_proto_by_slice.assign(
        static_cast<size_t>(num_row_clusters) * kJPEGNonZeroBuckets, -1);
    pass_model.proto_symbol_cost.clear();
    BuildBiclusterPassHistograms(pass, num_passes, num_row_clusters,
                                 ac_cluster_hist, ac_cluster_total,
                                 nz_cluster_hist, nz_cluster_total,
                                 &pass_histograms, &ac_hist_index,
                                 &nz_hist_index);
    clustered.clear();
    histogram_symbols.clear();
    if (pass_histograms.empty()) continue;
    JXL_RETURN_IF_ERROR(ClusterHistograms(params, pass_histograms,
                                          proto_budget_per_pass, &clustered,
                                          &histogram_symbols));
    (*num_prototypes_per_pass)[pass] = clustered.size();
    for (size_t i = 0; i < ac_hist_index.size(); ++i) {
      if (ac_hist_index[i] >= 0) {
        pass_model.ac_proto_by_slice[i] =
            static_cast<int16_t>(histogram_symbols[ac_hist_index[i]]);
      }
    }
    for (size_t i = 0; i < nz_hist_index.size(); ++i) {
      if (nz_hist_index[i] >= 0) {
        pass_model.nz_proto_by_slice[i] =
            static_cast<int16_t>(histogram_symbols[nz_hist_index[i]]);
      }
    }
    pass_model.proto_symbol_cost.resize(clustered.size());
    for (const auto& h : clustered) {
      eval.corrected_entropy_cost +=
          static_cast<FixedPointCost>(h.ShannonEntropy() * kFScale);
      JXL_ASSIGN_OR_RETURN(FixedPointCost header_cost, HistogramHeaderCost(h));
      eval.signalling_overhead += header_cost;
    }
    for (size_t i = 0; i < clustered.size(); ++i) {
      pass_model.proto_symbol_cost[i] = BuildHistogramCostTable(clustered[i]);
    }
    if (eval.corrected_entropy_cost + eval.signalling_overhead +
            fixed_pass_overhead >=
        cutoff) {
      eval.signalling_overhead += fixed_pass_overhead;
      return eval;
    }
  }

  // Per-pass global signalling plus per-(pass, group) TOC / stream overhead
  // estimated from the final clustered histograms in one sweep over blocks.
  eval.signalling_overhead += fixed_pass_overhead;
  if (eval.total_cost() >= cutoff) return eval;
  eval.signalling_overhead += EstimateBiclusterGroupOverhead(
      d, thresholds, ctx_map, rows, pass_assignment, num_passes, pass_models,
      cutoff - eval.total_cost());
  return eval;
}

// --- Pass-aware stream construction --------------------------------------------

struct EmitBin {
  ACBin raw_bin;
  uint32_t hist_key;
  uint32_t compact_id;
};

// Returns the top-left-anchored DC bucket indices for the current block in all
// components. This mirrors the coordinate convention used by the main
// clustering path and lets pass-aware code rebuild the same cell ids.
void BlockDCIndices(const JPEGOptData& d, uint32_t c, uint32_t b, uint32_t* dc0,
                    uint32_t* dc1, uint32_t* dc2) {
  const uint32_t y = b / d.block_grid_w[c];
  const uint32_t x = b % d.block_grid_w[c];
  const uint32_t b0 = MapTopLeftBlockIndex(d, c, y, x, 0);
  *dc0 = d.block_DC_idx[0][b0];
  if (d.channels == 1) {
    *dc1 = 0;
    *dc2 = 0;
    return;
  }
  const uint32_t b1 = MapTopLeftBlockIndex(d, c, y, x, 1);
  const uint32_t b2 = MapTopLeftBlockIndex(d, c, y, x, 2);
  *dc1 = d.block_DC_idx[1][b1];
  *dc2 = d.block_DC_idx[2][b2];
}

// Maps one block to its threshold cell id for the given threshold set. This is
// shared by the pass-aware evaluator and the biclustering lattice builder.
uint32_t BlockCell(const JPEGOptData& d, const AxisMaps& axis_maps,
                   const ThresholdSet& thresholds, uint32_t c, uint32_t b) {
  if (d.channels == 1) {
    return axis_maps.ax0_to_k[d.block_DC_idx[0][b]];
  }
  const uint32_t y = b / d.block_grid_w[c];
  const uint32_t x = b % d.block_grid_w[c];
  const uint32_t b0 = MapTopLeftBlockIndex(d, c, y, x, 0);
  const uint32_t b1 = MapTopLeftBlockIndex(d, c, y, x, 1);
  const uint32_t b2 = MapTopLeftBlockIndex(d, c, y, x, 2);
  const uint32_t n0 = static_cast<uint32_t>(thresholds.TY().size() + 1);
  return (axis_maps.ax1_row[d.block_DC_idx[1][b1]] +
          axis_maps.ax2_col[d.block_DC_idx[2][b2]]) *
             n0 +
         axis_maps.ax0_to_k[d.block_DC_idx[0][b0]];
}

// Metadata for one compact active bin while rebuilding the pass-local AC
// stream. `hist_key` matches the encoder's chosen AC histogram model so bins
// can be emitted in histogram-major order.
std::vector<EmitBin> BuildEmitBins(const JPEGOptData& d,
                                   const ActiveRawBins& active) {
  std::vector<EmitBin> emit_bins;
  emit_bins.reserve(active.active_bins.size());
  for (uint32_t compact_id = 0; compact_id < active.active_bins.size();
       ++compact_id) {
    const ACBin raw_bin = active.active_bins[compact_id];
    emit_bins.push_back({raw_bin, d.ACHistogramKey(raw_bin), compact_id});
  }
  if (d.AC_hist_model == JPEGTranscodeACModel::kToken420) {
    std::sort(emit_bins.begin(), emit_bins.end(),
              [](const EmitBin& a, const EmitBin& b) {
                if (a.hist_key != b.hist_key) return a.hist_key < b.hist_key;
                return a.raw_bin < b.raw_bin;
              });
  }
  return emit_bins;
}

// Rebuilds the AC stream after pass assignment. The resulting stream keeps the
// same packed-entry layout as the canonical optimizer stream, but is split into
// independent contiguous pass ranges recorded in `pass_offsets`.
StatusOr<std::vector<ACEntry>> BuildPassStream(
    const JPEGOptData& d, const ActiveRawBins& active,
    const PassAssignment& pass_assignment, uint32_t num_passes,
    std::vector<uint32_t>* pass_offsets, ThreadPool* pool) {
  const uint32_t M = static_cast<uint32_t>(active.active_bins.size());
  // Each AC entry is bucketed by (pass, compact_bin, dc0_high_bit). The high
  // bit of dc0 splits entries into two halves per (pass, bin), which enables
  // the `emit_half` lambda below to emit a dc0-base jump marker when crossing
  // from the low half to the high half.
  const uint32_t num_buckets = num_passes * M * 2;
  constexpr uint32_t kBlocksPerTask = 2048;
  std::array<uint32_t, kNumCh + 1> block_prefix = {};
  for (uint32_t c = 0; c < d.channels; ++c) {
    block_prefix[c + 1] = block_prefix[c] + d.num_blocks[c];
  }
  for (uint32_t c = d.channels; c < kNumCh; ++c) {
    block_prefix[c + 1] = block_prefix[c];
  }
  const uint32_t total_blocks = block_prefix[d.channels];
  const uint32_t num_tasks =
      (total_blocks + kBlocksPerTask - 1) / kBlocksPerTask;

  // Counting sort: first pass — compute bucket sizes.
  std::vector<uint32_t> bucket_start(static_cast<size_t>(num_buckets) + 1, 0);
  std::vector<std::vector<uint32_t>> thread_bucket_pos;
  JXL_RETURN_IF_ERROR(RunOnPool(
      pool, 0, num_tasks,
      [&](size_t num_threads) -> Status {
        thread_bucket_pos.assign(
            num_threads, std::vector<uint32_t>(num_buckets, 0));
        return true;
      },
      [&](uint32_t task, size_t thread) -> Status {
        std::vector<uint32_t>& local_counts = thread_bucket_pos[thread];
        const uint32_t begin = task * kBlocksPerTask;
        const uint32_t end = std::min(total_blocks, begin + kBlocksPerTask);
        uint32_t c = 0;
        while (c + 1 < block_prefix.size() && begin >= block_prefix[c + 1]) {
          ++c;
        }
        uint32_t global = begin;
        while (global < end && c < d.channels) {
          const uint32_t local_begin = global - block_prefix[c];
          const uint32_t local_end =
              std::min(end, block_prefix[c + 1]) - block_prefix[c];
          for (uint32_t b = local_begin; b < local_end; ++b) {
          uint32_t dc0 = 0;
          uint32_t dc1 = 0;
          uint32_t dc2 = 0;
          BlockDCIndices(d, c, b, &dc0, &dc1, &dc2);
          const uint32_t pass = pass_assignment[c][b];
          for (uint32_t pi = d.block_offsets[c][b];
               pi < d.block_offsets[c][b + 1]; ++pi) {
            const ACBin raw_bin = d.block_bins[c][pi];
            const uint32_t compact_id = active.raw_to_compact[raw_bin];
            ++local_counts[(static_cast<size_t>(pass) * M + compact_id) * 2 +
                           (dc0 >> 10)];
          }
          }
          global = block_prefix[c] + local_end;
          ++c;
        }
        return true;
      },
      "BuildPassStreamCount"));
  for (uint32_t bucket = 0; bucket < num_buckets; ++bucket) {
    uint32_t total = 0;
    for (size_t thread = 0; thread < thread_bucket_pos.size(); ++thread) {
      total += thread_bucket_pos[thread][bucket];
    }
    bucket_start[bucket + 1] = total;
  }
  // Prefix sum to turn counts into exclusive start positions.
  for (size_t i = 0; i < num_buckets; ++i) {
    bucket_start[i + 1] += bucket_start[i];
  }
  for (uint32_t bucket = 0; bucket < num_buckets; ++bucket) {
    uint32_t pos = bucket_start[bucket];
    for (size_t thread = 0; thread < thread_bucket_pos.size(); ++thread) {
      const uint32_t count = thread_bucket_pos[thread][bucket];
      thread_bucket_pos[thread][bucket] = pos;
      pos += count;
    }
  }

  // Counting sort: second pass — scatter entries into buckets.
  // Each entry packs (dc0_low_10_bits, dc1_11_bits, dc2_11_bits) into 32 bits.
  std::vector<uint32_t> flat(bucket_start.back());
  std::vector<std::atomic<uint32_t>> write_pos(num_buckets);
  for (uint32_t bucket = 0; bucket < num_buckets; ++bucket) {
    write_pos[bucket].store(bucket_start[bucket], std::memory_order_relaxed);
  }
  JXL_RETURN_IF_ERROR(RunOnPool(
      pool, 0, num_tasks, ThreadPool::NoInit,
      [&](uint32_t task, size_t /*thread*/) -> Status {
        const uint32_t begin = task * kBlocksPerTask;
        const uint32_t end = std::min(total_blocks, begin + kBlocksPerTask);
        uint32_t c = 0;
        while (c + 1 < block_prefix.size() && begin >= block_prefix[c + 1]) {
          ++c;
        }
        uint32_t global = begin;
        while (global < end && c < d.channels) {
          const uint32_t local_begin = global - block_prefix[c];
          const uint32_t local_end =
              std::min(end, block_prefix[c + 1]) - block_prefix[c];
          for (uint32_t b = local_begin; b < local_end; ++b) {
          uint32_t dc0 = 0;
          uint32_t dc1 = 0;
          uint32_t dc2 = 0;
          BlockDCIndices(d, c, b, &dc0, &dc1, &dc2);
          const uint32_t pass = pass_assignment[c][b];
          for (uint32_t pi = d.block_offsets[c][b];
               pi < d.block_offsets[c][b + 1]; ++pi) {
            const ACBin raw_bin = d.block_bins[c][pi];
            const uint32_t compact_id = active.raw_to_compact[raw_bin];
            const uint32_t bucket =
                (static_cast<size_t>(pass) * M + compact_id) * 2 + (dc0 >> 10);
            const uint32_t pos =
                write_pos[bucket].fetch_add(1, std::memory_order_relaxed);
            flat[pos] =
                ((dc0 & 0x3FFu) << 22) | (dc1 << 11) | dc2;
          }
          }
          global = block_prefix[c] + local_end;
          ++c;
        }
        return true;
      },
      "BuildPassStreamScatter"));

  // Sort within each bucket by the packed DC indices. This groups entries
  // with the same dc0/dc1/dc2 together, enabling run-length encoding in the
  // `emit_half` lambda below.
  JXL_RETURN_IF_ERROR(RunOnPool(
      pool, 0, num_buckets, ThreadPool::NoInit,
      [&](uint32_t bucket, size_t /*thread*/) -> Status {
        const uint32_t begin = bucket_start[bucket];
        const uint32_t end = bucket_start[bucket + 1];
        if (begin < end) {
          std::sort(flat.begin() + begin, flat.begin() + end);
        }
        return true;
      },
      "BuildPassStreamSort"));

  // Emit the final AC stream by iterating over passes and bins in
  // histogram-major order. Within each (pass, bin) group, entries are emitted
  // from the sorted buckets with run-length encoding and dc0 jump markers.
  const std::vector<EmitBin> emit_bins = BuildEmitBins(d, active);
  std::vector<ACEntry> stream;
  stream.reserve(flat.size() + flat.size() / 16);
  if (pass_offsets != nullptr) pass_offsets->assign(num_passes + 1, 0);

  for (uint32_t pass = 0; pass < num_passes; ++pass) {
    if (pass_offsets != nullptr) (*pass_offsets)[pass] = stream.size();
    uint32_t prev_hist_key = UINT32_MAX;
    uint32_t prev_ctx_key = UINT32_MAX;
    uint32_t cur_dc0 = 0;

    for (const EmitBin& emit_bin : emit_bins) {
      const uint32_t bucket_lo = (pass * M + emit_bin.compact_id) * 2;
      const uint32_t s_lo = bucket_start[bucket_lo];
      const uint32_t e_lo = bucket_start[bucket_lo + 1];
      const uint32_t s_hi = bucket_start[bucket_lo + 1];
      const uint32_t e_hi = bucket_start[bucket_lo + 2];
      if (s_lo == e_lo && s_hi == e_hi) continue;

      const uint32_t ctx_key = JPEGOptData::ACBinCZDC(emit_bin.raw_bin);
      const bool bin_change =
          prev_hist_key != UINT32_MAX && emit_bin.hist_key != prev_hist_key;
      const bool ctx_change = prev_ctx_key != UINT32_MAX && ctx_key != prev_ctx_key;

      auto emit_half = [&](uint32_t begin, uint32_t end, uint32_t dc0_base,
                           bool* first_in_bin) {
        uint32_t i = begin;
        while (i < end) {
          uint32_t j = i + 1;
          while (j < end && flat[j] == flat[i]) ++j;
          uint32_t run = j - i;
          const uint32_t dc0 = (flat[i] >> 22) | dc0_base;
          const uint32_t dc1 = (flat[i] >> 11) & 0x7FFu;
          const uint32_t dc2 = flat[i] & 0x7FFu;
          if (*first_in_bin || dc0 - cur_dc0 > 15u) {
            stream.push_back(
                (1u << 31) |
                (static_cast<uint32_t>(ctx_change && *first_in_bin) << 30) |
                (static_cast<uint32_t>(bin_change && *first_in_bin) << 29) |
                (emit_bin.raw_bin << 7) | (dc0 >> 4));
            cur_dc0 = (dc0 >> 4) << 4;
          }

          uint32_t delta_dc0 = dc0 - cur_dc0;
          uint32_t header = (delta_dc0 << 27) | (dc1 << 16) | (dc2 << 5);
          const uint32_t cont_header = (dc1 << 16) | (dc2 << 5);
          while (run > 32) {
            stream.push_back(header | 31u);
            run -= 32;
            header = cont_header;
          }
          stream.push_back(header | (run - 1));
          cur_dc0 = dc0;
          *first_in_bin = false;
          i = j;
        }
      };

      bool first_in_bin = true;
      emit_half(s_lo, e_lo, 0u, &first_in_bin);
      emit_half(s_hi, e_hi, 0x400u, &first_in_bin);
      prev_hist_key = emit_bin.hist_key;
      prev_ctx_key = ctx_key;
    }
  }
  if (pass_offsets != nullptr) (*pass_offsets)[num_passes] = stream.size();
  return stream;
}

// --- Pass-aware clustering and scoring -----------------------------------------

// Iterates over keys present in both sparse histograms, calling `fn(key,
// count_lhs, count_rhs)`. Used by the agglomerative clustering to compute
// the entropy delta of merging two contexts.
template <typename Func>
void ForEachIntersection(const std::unordered_map<uint32_t, uint32_t>& lhs,
                         const std::unordered_map<uint32_t, uint32_t>& rhs,
                         Func&& fn) {
  const auto* smaller = &lhs;
  const auto* larger = &rhs;
  if (smaller->size() > larger->size()) std::swap(smaller, larger);
  for (const auto& entry : *smaller) {
    auto it = larger->find(entry.first);
    if (it != larger->end()) {
      fn(entry.first, entry.second, it->second);
    }
  }
}

// Entropy proxy for one sparse nz histogram. Kept file-local because only the
// pass-aware search uses this sparse-map representation.
FixedPointCost HistogramCost(const JPEGOptData& d,
                      const std::unordered_map<uint32_t, uint32_t>& hist) {
  FixedPointCost cost = 0;
  for (const auto& entry : hist) {
    cost += d.NZFTab(entry.second);
  }
  return cost;
}

// Agglomerative clustering of the pass-aware `(cell, pass)` contexts.
// Thresholds define the rows/cells; the prebuilt pass stream defines the
// pass-local AC events. The result is a flat `ctx_map` over original
// `(channel, cell)` rows.
StatusOr<ClusterResult> ClusterContextsPassAware(
    const JPEGOptData& d, const ThresholdSet& thresholds,
    const std::vector<ACEntry>& pass_stream, const std::vector<uint32_t>& pass_offsets,
    uint32_t num_passes, uint32_t target_clusters) {
  AxisMaps axis_maps(d);
  axis_maps.Update(thresholds);
  const uint32_t n0 = static_cast<uint32_t>(thresholds.TY().size() + 1);
  const uint32_t num_cells =
      n0 * static_cast<uint32_t>(thresholds.TCb().size() + 1) *
      static_cast<uint32_t>(thresholds.TCr().size() + 1);
  const uint32_t total_ctxs = d.channels * num_cells;

  // Build per-(context, pass) sparse histograms by sweeping the pass-local
  // AC stream. `hist_h` tracks per-symbol counts, `hist_N` tracks per-zdc
  // counts; both are indexed by `(channel * num_cells + cell) * num_passes +
  // pass`.
  SparseHistogram hist_h(static_cast<size_t>(total_ctxs) * num_passes);
  SparseHistogram hist_N(static_cast<size_t>(total_ctxs) * num_passes);

  for (uint32_t pass = 0; pass < num_passes; ++pass) {
    SweepACStreamRange(
        pass_stream.begin() + pass_offsets[pass],
        pass_stream.begin() + pass_offsets[pass + 1], []() {}, []() {},
        [&](uint32_t dc0_idx, uint32_t dc1_idx, uint32_t dc2_idx, uint32_t run,
            uint32_t bin_state) {
          const uint32_t c = JPEGOptData::ACBinChannel(bin_state);
          const uint32_t cell = (axis_maps.ax1_row[dc1_idx] + axis_maps.ax2_col[dc2_idx]) *
                                    n0 +
                                axis_maps.ax0_to_k[dc0_idx];
          const uint32_t idx = (c * num_cells + cell) * num_passes + pass;
          const CompactACEvent ac_event = d.FromBin(bin_state);
          hist_h[idx][ac_event.hist_bin] += run;
          hist_N[idx][ac_event.zdc] += run;
        });
  }

  // Early exit: if there are fewer contexts than the target, identity map.
  if (total_ctxs <= target_clusters) {
    ClusterResult out;
    out.num_clusters = total_ctxs;
    out.ctx_map.resize(total_ctxs);
    for (uint32_t i = 0; i < total_ctxs; ++i) out.ctx_map[i] = static_cast<uint8_t>(i);
    return out;
  }

  auto is_active_ctx = [&](uint32_t ctx) {
    for (uint32_t pass = 0; pass < num_passes; ++pass) {
      if (!hist_N[ctx * num_passes + pass].empty()) return true;
    }
    return false;
  };

  // Compute the standalone entropy cost of each context and collect the
  // active (non-empty) ones. Inactive contexts are mapped to cluster 0.
  std::vector<FixedPointCost> cost(total_ctxs, 0);
  std::vector<uint32_t> active;
  active.reserve(total_ctxs);
  for (uint32_t ctx = 0; ctx < total_ctxs; ++ctx) {
    if (is_active_ctx(ctx)) active.push_back(ctx);
    FixedPointCost ctx_cost = 0;
    for (uint32_t pass = 0; pass < num_passes; ++pass) {
      const size_t idx = static_cast<size_t>(ctx) * num_passes + pass;
      for (const auto& entry : hist_N[idx]) ctx_cost += d.ftab[entry.second];
      for (const auto& entry : hist_h[idx]) ctx_cost -= d.ftab[entry.second];
    }
    cost[ctx] = ctx_cost;
  }

  // If the number of active contexts is already within budget, assign each
  // to its own cluster.
  if (active.size() <= target_clusters) {
    ClusterResult out;
    out.num_clusters = static_cast<uint32_t>(std::max<size_t>(1, active.size()));
    out.ctx_map.assign(total_ctxs, 0);
    for (uint32_t i = 0; i < active.size(); ++i) {
      out.ctx_map[active[i]] = static_cast<uint8_t>(i);
    }
    return out;
  }

  // Agglomerative clustering: union-find `parent` tracks merges, and the
  // upper-triangular `deltas` matrix stores the pairwise merge cost delta.
  std::vector<uint32_t> parent(total_ctxs);
  for (uint32_t i = 0; i < total_ctxs; ++i) parent[i] = i;

  // O(active²) pairwise merge deltas. Only the upper triangle is stored.
  std::vector<FixedPointCost> deltas(static_cast<size_t>(total_ctxs) * total_ctxs, 0);
  auto delta_ref = [&](uint32_t a, uint32_t b) -> FixedPointCost& {
    if (a > b) std::swap(a, b);
    return deltas[static_cast<size_t>(a) * total_ctxs + b];
  };

  auto merge_delta = [&](uint32_t a, uint32_t b) {
    FixedPointCost delta = 0;
    for (uint32_t pass = 0; pass < num_passes; ++pass) {
      const size_t ia = static_cast<size_t>(a) * num_passes + pass;
      const size_t ib = static_cast<size_t>(b) * num_passes + pass;
      ForEachIntersection(hist_N[ia], hist_N[ib],
                          [&](uint32_t, uint32_t ca, uint32_t cb) {
                            delta += d.ftab[ca + cb] - d.ftab[ca] - d.ftab[cb];
                          });
      ForEachIntersection(hist_h[ia], hist_h[ib],
                          [&](uint32_t, uint32_t ca, uint32_t cb) {
                            delta -= d.ftab[ca + cb] - d.ftab[ca] - d.ftab[cb];
                          });
    }
    return delta;
  };

  for (size_t i = 0; i + 1 < active.size(); ++i) {
    for (size_t j = i + 1; j < active.size(); ++j) {
      delta_ref(active[i], active[j]) = merge_delta(active[i], active[j]);
    }
  }

  while (active.size() > target_clusters && active.size() > 1) {
    size_t best_i = 0;
    size_t best_j = 1;
    FixedPointCost best_delta = delta_ref(active[0], active[1]);
    for (size_t i = 0; i + 1 < active.size(); ++i) {
      for (size_t j = i + 1; j < active.size(); ++j) {
        const FixedPointCost delta = delta_ref(active[i], active[j]);
        if (delta < best_delta) {
          best_delta = delta;
          best_i = i;
          best_j = j;
        }
      }
    }

    const uint32_t keep = active[best_i];
    const uint32_t drop = active[best_j];
    cost[keep] += cost[drop] + best_delta;
    for (uint32_t pass = 0; pass < num_passes; ++pass) {
      const size_t ik = static_cast<size_t>(keep) * num_passes + pass;
      const size_t id = static_cast<size_t>(drop) * num_passes + pass;
      for (const auto& entry : hist_N[id]) hist_N[ik][entry.first] += entry.second;
      for (const auto& entry : hist_h[id]) hist_h[ik][entry.first] += entry.second;
    }
    parent[drop] = keep;
    active.erase(active.begin() + best_j);
    for (uint32_t other : active) {
      if (other == keep) continue;
      delta_ref(keep, other) = merge_delta(keep, other);
    }
  }

  ClusterResult out;
  out.num_clusters = static_cast<uint32_t>(active.size());
  out.ctx_map.assign(total_ctxs, 0);
  std::unordered_map<uint32_t, uint32_t> cluster_id;
  cluster_id.reserve(active.size());
  for (uint32_t i = 0; i < active.size(); ++i) {
    cluster_id[active[i]] = i;
  }
  for (uint32_t ctx = 0; ctx < total_ctxs; ++ctx) {
    const uint32_t root = FindRoot(parent, ctx);
    auto it = cluster_id.find(root);
    if (it != cluster_id.end()) {
      out.ctx_map[ctx] = static_cast<uint8_t>(it->second);
    } else {
      out.ctx_map[ctx] = 0;
    }
  }
  return out;
}

// Estimates histogram-header cost for one sparse AC histogram by regrouping its
// symbols into signalling-token histograms split by `zdc`.
StatusOr<FixedPointCost> SignalOverheadFromHist(
    const JPEGOptData& d,
    const std::unordered_map<uint32_t, uint32_t>& hist_h) {
  std::array<std::array<uint32_t, kACTokenCount>, kZeroDensityContextCount>
      signalling_hist = {};
  const auto& dense_to_symbol = d.ACHistogram().dense_to_zdcvalue;
  for (const auto& entry : hist_h) {
    const SignallingHistSymbol sym =
        d.SignallingHistSymbolFromSymbol(dense_to_symbol[entry.first]);
    signalling_hist[sym.zdc][sym.token] += entry.second;
  }

  FixedPointCost overhead = 0;
  for (uint32_t zdc = 0; zdc < kZeroDensityContextCount; ++zdc) {
    size_t total = 0;
    uint32_t max_token = 0;
    for (uint32_t token = 0; token < kACTokenCount; ++token) {
      if (signalling_hist[zdc][token] == 0) continue;
      total += signalling_hist[zdc][token];
      max_token = token;
    }
    if (total == 0) continue;

    Histogram h(max_token + 1);
    for (uint32_t token = 0; token <= max_token; ++token) {
      h.counts[token] = static_cast<ANSHistBin>(signalling_hist[zdc][token]);
    }
    h.total_count = total;
    JXL_ASSIGN_OR_RETURN(float ans_cost, h.ANSPopulationCost());
    const float shannon = h.ShannonEntropy();
    const float header_cost = ans_cost - shannon;
    if (header_cost > 0) {
      overhead += static_cast<FixedPointCost>(header_cost * kFScale);
    }
  }
  return overhead;
}

// Estimates histogram-header cost for one sparse nz histogram by splitting it
// into one histogram per predictor bucket.
StatusOr<FixedPointCost> SignalOverheadFromNZHist(
    const std::unordered_map<uint32_t, uint32_t>& nz_hist) {
  FixedPointCost overhead = 0;
  for (uint32_t pb = 0; pb < kJPEGNonZeroBuckets; ++pb) {
    uint32_t max_nz = 0;
    size_t total = 0;
    for (const auto& entry : nz_hist) {
      const uint32_t cur_pb = entry.first / kJPEGNonZeroRange;
      if (cur_pb != pb) continue;
      const uint32_t nz = entry.first % kJPEGNonZeroRange;
      max_nz = std::max(max_nz, nz);
      total += entry.second;
    }
    if (total == 0) continue;

    Histogram h(max_nz + 1);
    for (const auto& entry : nz_hist) {
      const uint32_t cur_pb = entry.first / kJPEGNonZeroRange;
      if (cur_pb != pb) continue;
      const uint32_t nz = entry.first % kJPEGNonZeroRange;
      h.counts[nz] = static_cast<ANSHistBin>(entry.second);
    }
    h.total_count = total;
    JXL_ASSIGN_OR_RETURN(float ans_cost, h.ANSPopulationCost());
    const float shannon = h.ShannonEntropy();
    const float header_cost = ans_cost - shannon;
    if (header_cost > 0) {
      overhead += static_cast<FixedPointCost>(header_cost * kFScale);
    }
  }
  return overhead;
}

// Fully evaluates one pass-aware model: rebuilds clustered AC and nz
// histograms, computes entropy terms, and adds the estimated histogram-header
// signalling cost.
StatusOr<ModelEvaluation> EvaluatePassAwareModel(
    const JPEGOptData& d, const ThresholdSet& thresholds,
    const ContextMap& ctx_map, uint32_t num_clusters,
    const PassAssignment& pass_assignment, uint32_t num_passes,
    const std::vector<ACEntry>& pass_stream, const std::vector<uint32_t>& pass_offsets) {
  AxisMaps axis_maps(d);
  axis_maps.Update(thresholds);
  uint32_t n0 = static_cast<uint32_t>(thresholds.TY().size() + 1);
  uint32_t num_cells =
      n0 * static_cast<uint32_t>(thresholds.TCb().size() + 1) *
      static_cast<uint32_t>(thresholds.TCr().size() + 1);
  uint32_t cp_count = num_clusters * num_passes;

  SparseHistogram ac_hist_h(cp_count);
  SparseHistogram ac_hist_N(cp_count);
  SparseHistogram nz_hist_h(cp_count);
  SparseHistogram nz_hist_N(cp_count);

  for (uint32_t pass = 0; pass < num_passes; ++pass) {
    SweepACStreamRange(
        pass_stream.begin() + pass_offsets[pass],
        pass_stream.begin() + pass_offsets[pass + 1], []() {}, []() {},
        [&](uint32_t dc0_idx, uint32_t dc1_idx, uint32_t dc2_idx, uint32_t run,
            uint32_t bin_state) {
          const uint32_t c = JPEGOptData::ACBinChannel(bin_state);
          const uint32_t cell = (axis_maps.ax1_row[dc1_idx] + axis_maps.ax2_col[dc2_idx]) *
                                    n0 +
                                axis_maps.ax0_to_k[dc0_idx];
          const uint32_t cluster = ctx_map[c * num_cells + cell];
          const uint32_t cp = cluster * num_passes + pass;
          const CompactACEvent ac_event = d.FromBin(bin_state);
          ac_hist_h[cp][ac_event.hist_bin] += run;
          ac_hist_N[cp][ac_event.zdc] += run;
        });
  }

  // Build per-(cluster, pass) NZ predictor histograms. For each block, we
  // evaluate the NZ predictor for every pass (not just the block's assigned
  // pass) because the NZ context model is defined over all passes. The
  // actual NZ count is used only for the assigned pass; for other passes,
  // nz=0 is recorded (the block does not contribute NZ events in those passes).
  for (uint32_t c = 0; c < d.channels; ++c) {
    for (uint32_t y = 0; y < d.block_grid_h[c]; ++y) {
      for (uint32_t x = 0; x < d.block_grid_w[c]; ++x) {
        const uint32_t b = y * d.block_grid_w[c] + x;
        const uint32_t pass = pass_assignment[c][b];
        uint32_t cell = BlockCell(d, axis_maps, thresholds, c, b);
        uint32_t cluster = ctx_map[c * num_cells + cell];

        uint32_t b_top = (y - 1) * d.block_grid_w[c] + x;
        uint32_t b_left = y * d.block_grid_w[c] + (x - 1);
        uint32_t nz_top = (y > 0) ? d.block_nonzeros[c][b_top] : 0u;
        uint32_t nz_left = (x > 0) ? d.block_nonzeros[c][b_left] : 0u;
        uint8_t nz_top_pass = (y > 0) ? pass_assignment[c][b_top] : 255;
        uint8_t nz_left_pass = (x > 0) ? pass_assignment[c][b_left] : 255;

        for (uint32_t p = 0; p < num_passes; ++p) {
          uint32_t cp = cluster * num_passes + p;

          // Same NZ predictor logic as `PredictNZBucket`: only same-pass
          // neighbors contribute their actual NZ count.
          uint32_t predicted_nz;
          uint32_t pass_nz_top = (nz_top_pass == p) ? nz_top : 0u;
          uint32_t pass_nz_left = (nz_left_pass == p) ? nz_left : 0u;
          if (x == 0 && y == 0) {
            predicted_nz = 32u;
          } else if (x == 0) {
            predicted_nz = pass_nz_top;
          } else if (y == 0) {
            predicted_nz = pass_nz_left;
          } else {
            predicted_nz = (pass_nz_top + pass_nz_left + 1u) / 2u;
          }

          uint32_t pb =
              (predicted_nz < 8) ? predicted_nz : (4 + predicted_nz / 2);
          // Nonzero count is only non-zero for the block's assigned pass.
          uint32_t nz = pass == p ? d.block_nonzeros[c][b] : 0u;
          ++nz_hist_h[cp][NZHistogramIndex(pb, nz)];
          ++nz_hist_N[cp][pb];
        }
      }
    }
  }

  // Sum the entropy cost and estimated histogram-header signalling overhead
  // across all (cluster, pass) slots.
  ModelEvaluation eval;
  for (uint32_t cp = 0; cp < cp_count; ++cp) {
    // AC entropy: Σ ftab[N] - Σ ftab[h] (same formula as `TotalCost`).
    for (const auto& entry : ac_hist_N[cp]) eval.ac_cost += d.ftab[entry.second];
    for (const auto& entry : ac_hist_h[cp]) eval.ac_cost -= d.ftab[entry.second];
    // NZ entropy: Σ NZFTab[N] - Σ NZFTab[h].
    for (const auto& entry : nz_hist_N[cp]) eval.nz_cost += d.NZFTab(entry.second);
    for (const auto& entry : nz_hist_h[cp]) eval.nz_cost -= d.NZFTab(entry.second);
    JXL_ASSIGN_OR_RETURN(FixedPointCost ac_overhead,
                         SignalOverheadFromHist(d, ac_hist_h[cp]));
    JXL_ASSIGN_OR_RETURN(FixedPointCost nz_overhead,
                         SignalOverheadFromNZHist(nz_hist_h[cp]));
    eval.signalling_overhead += ac_overhead + nz_overhead;
  }

  // An additional passes overhead.
  eval.signalling_overhead += ComputePassOverhead(d) * num_passes;

  return eval;
}

// Optional threshold refinement stage reused by both experimental searches.
ThresholdSet RefinePassAwareThresholds(
    PartitioningCtx& ctx, const ThresholdSet& thresholds,
    const std::vector<ACEntry>& pass_stream,
    const JPEGCtxEffortParams& effort) {
  if (effort.refine_iters == 0) return thresholds;
  FixedPointCost ignored_cost = 0;
  return ctx.OptimizeThresholds(thresholds, pass_stream, effort.main_m_target,
                                effort.refine_iters, &ignored_cost);
}

// Upper bound for the number of progressive passes worth considering from
// the image size. 11 is a hard limit by the standard, and number of
// histogram clusters is limited by max `num_hf_presets` which is written by
// `u(ceil(log2(num_groups))) + 1`.
uint32_t ComputeMaxNumPasses(const JPEGOptData& d) {
  const double groups_x = static_cast<double>((d.w_max + 31) / 32);
  const double groups_y = static_cast<double>((d.h_max + 31) / 32);
  const double groups = std::max(1.0, groups_x * groups_y);
  return static_cast<uint32_t>(
      std::min(11.0, std::ceil(std::log2(groups)) + 1.0));
}

}  // namespace

// Runs the pass-aware planner on a fixed candidate list. For each candidate we
// optimize thresholds against the pass-local AC stream, cluster the resulting
// `(cell, pass)` contexts, evaluate the model, optionally refine thresholds,
// and keep the best-scoring result.
StatusOr<PassSearchResult> SearchPassAwareContextModel(
    std::shared_ptr<const JPEGOptData> opt_data,
    const std::vector<FactorizationCandidate>& candidates,
    const JPEGCtxEffortParams& effort, ThreadPool* pool) {
  if (candidates.empty()) {
    return JXL_FAILURE("Pass-aware search requires at least one candidate");
  }

  const JPEGOptData& d = *opt_data;
  const ActiveRawBins active = BuildActiveRawBins(d);
  const uint32_t min_passes =
      effort.optimize_passes_num <= 0
          ? 1
          : static_cast<uint32_t>(effort.optimize_passes_num);
  const uint32_t max_img_passes = ComputeMaxNumPasses(d);
  const uint32_t max_passes =
      effort.optimize_passes_num < 0
          ? 1
          : (effort.optimize_passes_num == 0
                 ? max_img_passes
                 : std::min<uint32_t>(effort.optimize_passes_num,
                                      max_img_passes));
  const uint32_t target_clusters =
      kMaxClusters - static_cast<uint32_t>(d.channels == 1);

  std::unique_ptr<AssignPassesRangeResult> assign_range_result;
  if (max_passes > min_passes) {
    assign_range_result = jxl::make_unique<AssignPassesRangeResult>(
        AssignPassesGreedyAllK(d, active, min_passes, max_passes, pool));
    fprintf(stderr,
            "PLANNER: AssignPassesGreedyAllK took %.2f ms total "
            "(%i batch %.2f ms, %i sequential %.2f ms)\n",
            NanosToMs(assign_range_result->shared_timings.total_ns),
            assign_range_result->shared_timings.batch_iters,
            NanosToMs(assign_range_result->shared_timings.batch_ns),
            assign_range_result->shared_timings.seq_iters,
            NanosToMs(assign_range_result->shared_timings.sequential_ns));
    fflush(stderr);
  }

  // Accumulates the best result across all pass configurations. Updated by
  // deterministic reduction after each pass loop.
  PassSearchResult best_result;
  best_result.total_cost = std::numeric_limits<FixedPointCost>::max();
  // Tiebreaker for deterministic results: among equal-cost candidates the
  // one with the lowest candidate index wins, independent of thread count.
  uint32_t overall_best_idx = std::numeric_limits<uint32_t>::max();

  for (uint32_t num_passes = min_passes; num_passes <= max_passes;
       ++num_passes) {
    auto start_pass_config = std::chrono::high_resolution_clock::now();
    fprintf(stderr, "PLANNER: Testing configuration with %u passes\n", num_passes);
    fflush(stderr);
    AssignPassesResult assign_result_storage;
    const AssignPassesResult* assign_result = nullptr;
    if (assign_range_result != nullptr) {
      assign_result =
          &assign_range_result->results[num_passes - min_passes];
    } else {
      assign_result_storage = AssignPassesGreedy(d, active, num_passes, pool);
      assign_result = &assign_result_storage;
      fprintf(stderr,
              "PLANNER: AssignPassesGreedy took %.2f ms total "
              "(batch %.2f ms, sequential %.2f ms)\n",
              NanosToMs(assign_result->timings.total_ns),
              NanosToMs(assign_result->timings.batch_ns),
              NanosToMs(assign_result->timings.sequential_ns));
      fflush(stderr);
    }
    const PassAssignment& pass_assignment = assign_result->pass_assignment;

    auto start_build_stream = PlannerClock::now();
    std::vector<uint32_t> pass_offsets;
    JXL_ASSIGN_OR_RETURN(std::vector<ACEntry> pass_stream,
                         BuildPassStream(d, active, pass_assignment, num_passes,
                                         &pass_offsets, pool));
    auto end_build_stream = PlannerClock::now();
    fprintf(stderr, "PLANNER: BuildPassStream took %.2f ms\n",
            NanosToMs(ElapsedNanos(start_build_stream, end_build_stream)));
    fflush(stderr);

    // Per-stage timing accumulators (atomics for lock-free accumulation from
    // multiple threads; relaxed ordering is sufficient — visibility is ensured
    // by the `RunOnPool` join barrier).
    std::atomic<int64_t> rough_opt_ns(0);
    std::atomic<int64_t> cluster_ns(0);
    std::atomic<int64_t> rough_eval_ns(0);
    std::atomic<int64_t> refine_ns(0);
    std::atomic<int64_t> refined_eval_ns(0);
    std::atomic<uint32_t> processed_candidates(0);

    // Per-thread best result. Each thread writes only its own slot
    // `thread_best[thread_id]`, so no synchronization is needed. After
    // `RunOnPool` returns, a single-threaded reduction picks the overall best.
    struct PassThreadBest {
      PassSearchResult result;
      uint32_t best_idx = std::numeric_limits<uint32_t>::max();
    };
    std::vector<PassThreadBest> thread_best;
    // One `PartitioningCtx` per thread.
    std::vector<PartitioningCtx> ctx_pool;
    auto start_candidate_loop = PlannerClock::now();
    JXL_RETURN_IF_ERROR(RunOnPool(
        pool, 0, static_cast<uint32_t>(candidates.size()),
        [&](size_t num_threads) -> Status {
          ctx_pool.reserve(num_threads);
          for (size_t i = 0; i < num_threads; ++i) {
            ctx_pool.emplace_back(opt_data);
          }
          thread_best.resize(num_threads);
          return true;
        },
        [&](uint32_t idx, size_t thread_id) -> Status {
          PartitioningCtx& ctx = ctx_pool[thread_id];
          const FactorizationCandidate& candidate = candidates[idx];

          // Stage 1: rough threshold optimization from the candidate's init
          // point. Produces an initial `ThresholdSet`.
          auto start_rough_opt = PlannerClock::now();
          FixedPointCost rough_unclustered_cost = 0;
          ThresholdSet rough_thresholds =
              ctx.OptimizeThresholds(candidate.init, pass_stream,
                                     effort.main_m_target, effort.main_iters,
                                     &rough_unclustered_cost);
          auto end_rough_opt = PlannerClock::now();
          rough_opt_ns.fetch_add(
              ElapsedNanos(start_rough_opt, end_rough_opt),
              std::memory_order_relaxed);

          // Stage 2: cluster the (cell, pass) contexts implied by the rough
          // thresholds into at most `target_clusters` groups.
          auto start_cluster = PlannerClock::now();
          JXL_ASSIGN_OR_RETURN(
              ClusterResult cluster_result,
              ClusterContextsPassAware(d, rough_thresholds, pass_stream,
                                       pass_offsets, num_passes,
                                       target_clusters));
          auto end_cluster = PlannerClock::now();
          cluster_ns.fetch_add(ElapsedNanos(start_cluster, end_cluster),
                               std::memory_order_relaxed);

          auto start_rough_eval = PlannerClock::now();
          JXL_ASSIGN_OR_RETURN(
              ModelEvaluation rough_eval,
              EvaluatePassAwareModel(d, rough_thresholds, cluster_result.ctx_map,
                                     cluster_result.num_clusters, pass_assignment,
                                     num_passes, pass_stream, pass_offsets));
          auto end_rough_eval = PlannerClock::now();
          rough_eval_ns.fetch_add(
              ElapsedNanos(start_rough_eval, end_rough_eval),
              std::memory_order_relaxed);

          // Stage 3: refine thresholds around the rough optimum.
          auto start_refine = PlannerClock::now();
          ThresholdSet refined_thresholds =
              RefinePassAwareThresholds(ctx, rough_thresholds, pass_stream,
                                        effort);
          auto end_refine = PlannerClock::now();
          refine_ns.fetch_add(ElapsedNanos(start_refine, end_refine),
                              std::memory_order_relaxed);

          auto start_refined_eval = PlannerClock::now();
          JXL_ASSIGN_OR_RETURN(
              ModelEvaluation refined_eval,
              EvaluatePassAwareModel(d, refined_thresholds, cluster_result.ctx_map,
                                     cluster_result.num_clusters, pass_assignment,
                                     num_passes, pass_stream, pass_offsets));
          auto end_refined_eval = PlannerClock::now();
          refined_eval_ns.fetch_add(
              ElapsedNanos(start_refined_eval, end_refined_eval),
              std::memory_order_relaxed);
          // relaxed is safe: only used for diagnostic printing after join.
          processed_candidates.fetch_add(1, std::memory_order_relaxed);

          // Pick whichever of rough/refined scored better.
          const bool refined_is_better =
              refined_eval.total_cost() < rough_eval.total_cost();
          const ThresholdSet& best_thresholds =
              refined_is_better ? refined_thresholds : rough_thresholds;
          const ModelEvaluation& best_eval =
              refined_is_better ? refined_eval : rough_eval;
          // Per-thread best; no lock needed. Tiebreaker: lowest candidate
          // index for deterministic results independent of thread count.
          auto& local = thread_best[thread_id];
          if (best_eval.total_cost() < local.result.total_cost ||
              (best_eval.total_cost() == local.result.total_cost &&
               idx < local.best_idx)) {
            local.result.thresholds = best_thresholds;
            local.result.ctx_map = cluster_result.ctx_map;
            local.result.pass_assignment = pass_assignment;
            local.result.num_passes = num_passes;
            local.result.num_clusters = cluster_result.num_clusters;
            local.result.ac_cost = best_eval.ac_cost;
            local.result.nz_cost = best_eval.nz_cost;
            local.result.signalling_overhead = best_eval.signalling_overhead;
            local.result.total_cost = best_eval.total_cost();
            local.best_idx = idx;
          }
          return true;
        },
        "JpegCtxPasses"));
    // Deterministic reduction: merge per-thread bests into the overall
    // best_result. Tiebreaker: lowest candidate index. This also computes
    // best_pass_cost (the best cost for this particular pass configuration)
    // for diagnostic printing below.
    FixedPointCost best_pass_cost = std::numeric_limits<FixedPointCost>::max();
    for (auto& tb : thread_best) {
      best_pass_cost = std::min(tb.result.total_cost, best_pass_cost);
      if (tb.result.total_cost < best_result.total_cost ||
          (tb.result.total_cost == best_result.total_cost &&
           tb.best_idx < overall_best_idx)) {
        best_result = std::move(tb.result);
        overall_best_idx = tb.best_idx;
      }
    }

    auto end_candidate_loop = PlannerClock::now();
    const uint32_t num_processed = processed_candidates.load(
        std::memory_order_relaxed);
    fprintf(stderr,
            "PLANNER: Candidate loop took %.2f ms wall time (%u candidates)\n",
            NanosToMs(ElapsedNanos(start_candidate_loop, end_candidate_loop)),
            num_processed);
    if (num_processed != 0) {
      // relaxed is safe here: RunOnPool join establishes happens-before, so
      // all fetch_add writes from worker threads are visible.
      const double rough_opt_ms =
          NanosToMs(rough_opt_ns.load(std::memory_order_relaxed));
      const double cluster_ms =
          NanosToMs(cluster_ns.load(std::memory_order_relaxed));
      const double rough_eval_ms =
          NanosToMs(rough_eval_ns.load(std::memory_order_relaxed));
      const double refine_ms =
          NanosToMs(refine_ns.load(std::memory_order_relaxed));
      const double refined_eval_ms =
          NanosToMs(refined_eval_ns.load(std::memory_order_relaxed));
      fprintf(stderr,
              "PLANNER: Candidate stages (sum/avg ms): \nrough_opt=%.2f/%.2f "
              "\ncluster=%.2f/%.2f \nrough_eval=%.2f/%.2f "
              "\nrefine=%.2f/%.2f \nrefined_eval=%.2f/%.2f\n",
              rough_opt_ms, rough_opt_ms / num_processed, cluster_ms,
              cluster_ms / num_processed, rough_eval_ms,
              rough_eval_ms / num_processed, refine_ms,
              refine_ms / num_processed, refined_eval_ms,
              refined_eval_ms / num_processed);
    }
    fflush(stderr);
    if (best_pass_cost != std::numeric_limits<FixedPointCost>::max()) {
      fprintf(stderr,
              "PLANNER: Best cost for %u passes = %.2f bits\n",
              num_passes, bit_cost(best_pass_cost));
      fflush(stderr);
    }
    auto end_pass_config = std::chrono::high_resolution_clock::now();
    fprintf(stderr, "PLANNER: Pass configuration %u took %.2f ms\n", num_passes,
            std::chrono::duration<double, std::milli>(end_pass_config - start_pass_config).count());
    fflush(stderr);
  }

  if (best_result.total_cost == std::numeric_limits<FixedPointCost>::max()) {
    return JXL_FAILURE("Pass-aware search did not produce a result");
  }
  return best_result;
}

// Convenience overload that ranks/trims factorization candidates first.
StatusOr<PassSearchResult> SearchPassAwareContextModel(
    std::shared_ptr<const JPEGOptData> opt_data,
    const JPEGCtxEffortParams& effort, ThreadPool* pool) {
  auto start_rank = std::chrono::high_resolution_clock::now();
  JXL_ASSIGN_OR_RETURN(std::vector<FactorizationCandidate> candidates,
                       RankAndTrimFactorizations(opt_data, effort, pool));
  auto end_rank = std::chrono::high_resolution_clock::now();
  fprintf(stderr, "PLANNER: RankAndTrimFactorizations took %.2f ms (found %zu candidates)\n",
          std::chrono::duration<double, std::milli>(end_rank - start_rank).count(),
          candidates.size());
  fflush(stderr);
  return SearchPassAwareContextModel(opt_data, candidates, effort, pool);
}

// Biclustering-prototype search on a fixed candidate list. This currently
// shares the pass-aware threshold optimization and row clustering steps, then
// re-scores each candidate on the richer `(row, pass, slice)` lattice.
StatusOr<BiclusterSearchResult> SearchBiclusteredContextModel(
    std::shared_ptr<const JPEGOptData> opt_data,
    const std::vector<FactorizationCandidate>& candidates,
    const JPEGCtxEffortParams& effort, ThreadPool* pool) {
  if (candidates.empty()) {
    return JXL_FAILURE("Biclustered search requires at least one candidate");
  }

  const JPEGOptData& d = *opt_data;
  const ActiveRawBins active = BuildActiveRawBins(d);
  const uint32_t min_num_passes =
      effort.optimize_passes_num <= 0
          ? 1
          : static_cast<uint32_t>(effort.optimize_passes_num);
  const uint32_t max_num_passes =
      effort.optimize_passes_num <= 0
          ? ComputeMaxNumPasses(d)
          : static_cast<uint32_t>(effort.optimize_passes_num);
  const uint32_t target_clusters =
      kMaxClusters - static_cast<uint32_t>(d.channels == 1);

  std::unique_ptr<AssignPassesRangeResult> assign_range_result;
  if (max_num_passes > min_num_passes) {
    assign_range_result = jxl::make_unique<AssignPassesRangeResult>(
        AssignPassesGreedyAllK(d, active, min_num_passes, max_num_passes,
                               pool));
    fprintf(stderr,
            "PLANNER: [bicluster] AssignPassesGreedyAllK took %.2f ms total "
            "(batch %.2f ms, sequential %.2f ms)\n",
            NanosToMs(assign_range_result->shared_timings.total_ns),
            NanosToMs(assign_range_result->shared_timings.batch_ns),
            NanosToMs(assign_range_result->shared_timings.sequential_ns));
    fflush(stderr);
  }
  // Accumulates the best result across all pass configurations. Updated by
  // deterministic reduction after each pass loop.
  BiclusterSearchResult best_result;
  best_result.total_cost = std::numeric_limits<FixedPointCost>::max();
  // Tiebreaker: lowest candidate index for deterministic results.
  uint32_t overall_best_idx = std::numeric_limits<uint32_t>::max();

  for (uint32_t num_passes = min_num_passes; num_passes <= max_num_passes;
       ++num_passes) {
    auto start_pass_config = std::chrono::high_resolution_clock::now();
    fprintf(stderr, "PLANNER: [bicluster] Testing configuration with %u passes\n",
            num_passes);
    fflush(stderr);

    AssignPassesResult assign_result_storage;
    const AssignPassesResult* assign_result = nullptr;
    if (assign_range_result != nullptr) {
      assign_result =
          &assign_range_result->results[num_passes - min_num_passes];
    } else {
      assign_result_storage = AssignPassesGreedy(d, active, num_passes, pool);
      assign_result = &assign_result_storage;
      fprintf(stderr,
              "PLANNER: [bicluster] AssignPassesGreedy took %.2f ms total "
              "(batch %.2f ms, sequential %.2f ms)\n",
              NanosToMs(assign_result->timings.total_ns),
              NanosToMs(assign_result->timings.batch_ns),
              NanosToMs(assign_result->timings.sequential_ns));
      fflush(stderr);
    }
    const PassAssignment& pass_assignment = assign_result->pass_assignment;
    const NZBlockCache nz_cache =
        BuildNZBlockCache(d, pass_assignment, num_passes);

    auto start_build_stream = PlannerClock::now();
    std::vector<uint32_t> pass_offsets;
    JXL_ASSIGN_OR_RETURN(std::vector<ACEntry> pass_stream,
                         BuildPassStream(d, active, pass_assignment,
                                         num_passes, &pass_offsets, pool));
    auto end_build_stream = PlannerClock::now();
    fprintf(stderr, "PLANNER: [bicluster] BuildPassStream took %.2f ms\n",
            NanosToMs(ElapsedNanos(start_build_stream, end_build_stream)));
    fflush(stderr);

    // Per-stage timing accumulators (relaxed atomics; join ensures visibility).
    std::atomic<int64_t> rough_opt_ns(0);
    std::atomic<int64_t> rough_row_cluster_ns(0);
    std::atomic<int64_t> rough_build_rows_ns(0);
    std::atomic<int64_t> rough_eval_ns(0);
    std::atomic<int64_t> refine_ns(0);
    std::atomic<int64_t> refined_row_cluster_ns(0);
    std::atomic<int64_t> refined_build_rows_ns(0);
    std::atomic<int64_t> refined_eval_ns(0);
    std::atomic<uint32_t> processed_candidates(0);

    // Per-thread best result.
    struct BiclusterThreadBest {
      BiclusterSearchResult result;
      uint32_t best_idx = std::numeric_limits<uint32_t>::max();
    };
    std::vector<BiclusterThreadBest> thread_best;
    std::vector<PartitioningCtx> ctx_pool;
    auto start_candidate_loop = PlannerClock::now();
    JXL_RETURN_IF_ERROR(RunOnPool(
        pool, 0, static_cast<uint32_t>(candidates.size()),
        [&](size_t num_threads) -> Status {
          ctx_pool.reserve(num_threads);
          for (size_t i = 0; i < num_threads; ++i) {
            ctx_pool.emplace_back(opt_data);
          }
          thread_best.resize(num_threads);
          return true;
        },
        [&](uint32_t idx, size_t thread_id) -> Status {
          PartitioningCtx& ctx = ctx_pool[thread_id];
          const FactorizationCandidate& candidate = candidates[idx];
          auto& local = thread_best[thread_id];

          // Stage 1: rough threshold optimization.
          auto start_rough_opt = PlannerClock::now();
          FixedPointCost rough_unclustered_cost = 0;
          ThresholdSet rough_thresholds =
              ctx.OptimizeThresholds(candidate.init, pass_stream,
                                     effort.main_m_target, effort.main_iters,
                                     &rough_unclustered_cost);
          auto end_rough_opt = PlannerClock::now();
          rough_opt_ns.fetch_add(ElapsedNanos(start_rough_opt, end_rough_opt),
                                 std::memory_order_relaxed);

          // Shared helper: given a row-slice state, cluster rows biclustered
          // and evaluate the full bicluster model. The refined path can reuse
          // the rough state and update only the blocks whose row changed.
          std::vector<uint32_t> rough_num_prototypes;
          ClusterResult rough_cluster_result;
          auto evaluate_state =
              [&](const ThresholdSet& thresholds, const RowSliceState& state,
                  std::atomic<int64_t>* row_cluster_ns,
                  std::atomic<int64_t>* eval_ns,
                  ClusterResult* cluster_result,
                  std::vector<uint32_t>* num_prototypes,
                  FixedPointCost cutoff)
              -> StatusOr<ModelEvaluation> {
            auto start_row_cluster = PlannerClock::now();
            JXL_ASSIGN_OR_RETURN(
                *cluster_result,
                ClusterRowsBiclustered(d, state.rows,
                                       std::min(target_clusters,
                                                effort.bicluster_row_budget)));
            auto end_row_cluster = PlannerClock::now();
            row_cluster_ns->fetch_add(
                ElapsedNanos(start_row_cluster, end_row_cluster),
                std::memory_order_relaxed);

            auto start_eval = PlannerClock::now();
            JXL_ASSIGN_OR_RETURN(ModelEvaluation eval, EvaluateBiclusterState(
                d, thresholds, cluster_result->ctx_map,
                cluster_result->num_clusters, pass_assignment,
                num_passes, effort.bicluster_proto_budget_per_pass, state.rows,
                num_prototypes, cutoff));
            auto end_eval = PlannerClock::now();
            eval_ns->fetch_add(ElapsedNanos(start_eval, end_eval),
                               std::memory_order_relaxed);
            return eval;
          };
          auto start_build_rows = PlannerClock::now();
          JXL_ASSIGN_OR_RETURN(
              RowSliceState rough_state,
              BuildRowSliceState(d, rough_thresholds, pass_assignment,
                                 num_passes, nz_cache));
          auto end_build_rows = PlannerClock::now();
          rough_build_rows_ns.fetch_add(
              ElapsedNanos(start_build_rows, end_build_rows),
              std::memory_order_relaxed);
          JXL_ASSIGN_OR_RETURN(ModelEvaluation rough_eval,
                               evaluate_state(rough_thresholds,
                                              rough_state,
                                              &rough_row_cluster_ns,
                                              &rough_eval_ns,
                                              &rough_cluster_result,
                                              &rough_num_prototypes,
                                              local.result.total_cost));
          PrunedCtxMapResult pruned_rough =
              PruneDeadThresholdsFromCtxMap(rough_thresholds,
                                           rough_cluster_result.ctx_map,
                                           d.channels);
          const bool rough_pruned =
              (pruned_rough.thresholds.T != rough_thresholds.T);
          ThresholdSet rough_output_thresholds = rough_thresholds;
          ClusterResult rough_output_cluster_result = rough_cluster_result;
          if (rough_pruned) {
            rough_output_thresholds = std::move(pruned_rough.thresholds);
            rough_output_cluster_result.ctx_map = std::move(pruned_rough.ctx_map);
          }

          auto start_refine = PlannerClock::now();
          ThresholdSet refined_thresholds =
              RefinePassAwareThresholds(ctx,
                                        rough_pruned ? rough_output_thresholds
                                                     : rough_thresholds,
                                        pass_stream, effort);
          auto end_refine = PlannerClock::now();
          refine_ns.fetch_add(ElapsedNanos(start_refine, end_refine),
                              std::memory_order_relaxed);
          auto start_refined_build_rows = PlannerClock::now();
          RowSliceState refined_seed_state;
          const RowSliceState* refine_base_state = &rough_state;
          if (rough_pruned) {
            JXL_ASSIGN_OR_RETURN(
                refined_seed_state,
                RefineRowSliceState(d, rough_output_thresholds, pass_assignment,
                                    num_passes, rough_state, nz_cache));
            refine_base_state = &refined_seed_state;
          }
          JXL_ASSIGN_OR_RETURN(
              RowSliceState refined_state,
              RefineRowSliceState(d, refined_thresholds, pass_assignment,
                                  num_passes, *refine_base_state, nz_cache));
          auto end_refined_build_rows = PlannerClock::now();
          refined_build_rows_ns.fetch_add(
              ElapsedNanos(start_refined_build_rows, end_refined_build_rows),
              std::memory_order_relaxed);
          std::vector<uint32_t> refined_num_prototypes;
          ClusterResult refined_cluster_result;
          JXL_ASSIGN_OR_RETURN(ModelEvaluation refined_eval,
                               evaluate_state(refined_thresholds,
                                              refined_state,
                                              &refined_row_cluster_ns,
                                              &refined_eval_ns,
                                              &refined_cluster_result,
                                              &refined_num_prototypes,
                                              std::min(local.result.total_cost,
                                                       rough_eval.total_cost())));
          processed_candidates.fetch_add(1, std::memory_order_relaxed);

          // Pick whichever of rough/refined scored better.
          const bool refined_is_better =
              refined_eval.total_cost() < rough_eval.total_cost();
          const ThresholdSet& best_thresholds =
              refined_is_better ? refined_thresholds : rough_output_thresholds;
          const ModelEvaluation& best_eval =
              refined_is_better ? refined_eval : rough_eval;
          const ClusterResult& best_cluster_result =
              refined_is_better ? refined_cluster_result
                                : rough_output_cluster_result;
          const std::vector<uint32_t>& best_num_prototypes =
              refined_is_better ? refined_num_prototypes : rough_num_prototypes;
          // Per-thread best; no lock needed. Tiebreaker: lowest candidate
          // index for deterministic results independent of thread count.
          if (best_eval.total_cost() < local.result.total_cost ||
              (best_eval.total_cost() == local.result.total_cost &&
               idx < local.best_idx)) {
            local.result.thresholds = best_thresholds;
            local.result.ctx_map = best_cluster_result.ctx_map;
            local.result.pass_assignment = pass_assignment;
            local.result.num_passes = num_passes;
            local.result.num_cells = static_cast<uint32_t>(
                best_cluster_result.ctx_map.size() / d.channels);
            local.result.num_row_clusters = best_cluster_result.num_clusters;
            local.result.num_prototypes_per_pass = best_num_prototypes;
            local.result.total_num_prototypes = 0;
            for (uint32_t n : best_num_prototypes) {
              local.result.total_num_prototypes += n;
            }
            // Prefer the ANS-clustering-corrected cost when available
            // (non-negative); fall back to the raw AC entropy cost.
            local.result.ac_cost = best_eval.corrected_entropy_cost >= 0
                                       ? best_eval.corrected_entropy_cost
                                       : best_eval.ac_cost;
            local.result.nz_cost = best_eval.nz_cost;
            local.result.signalling_overhead = best_eval.signalling_overhead;
            local.result.total_cost = best_eval.total_cost();
            local.best_idx = idx;
          }
          return true;
        },
        "JpegCtxBicluster"));
    // Deterministic reduction: pick best across all threads.
    // Tiebreaker: lowest candidate index.
    FixedPointCost best_pass_cost = std::numeric_limits<FixedPointCost>::max();
    for (auto& tb : thread_best) {
      best_pass_cost = std::min(tb.result.total_cost, best_pass_cost);
      if (tb.result.total_cost < best_result.total_cost ||
          (tb.result.total_cost == best_result.total_cost &&
           tb.best_idx < overall_best_idx)) {
        best_result = std::move(tb.result);
        overall_best_idx = tb.best_idx;
      }
    }

    auto end_candidate_loop = PlannerClock::now();
    const uint32_t num_processed =
        processed_candidates.load(std::memory_order_relaxed);
    fprintf(stderr,
            "PLANNER: [bicluster] Candidate loop took %.2f ms wall time (%u candidates)\n",
            NanosToMs(ElapsedNanos(start_candidate_loop, end_candidate_loop)),
            num_processed);
    if (num_processed != 0) {
      // relaxed is safe here: `RunOnPool` join establishes happens-before, so
      // all `fetch_add` writes from worker threads are visible.
      const double rough_opt_ms =
          NanosToMs(rough_opt_ns.load(std::memory_order_relaxed));
      const double rough_row_cluster_ms =
          NanosToMs(rough_row_cluster_ns.load(std::memory_order_relaxed));
      const double rough_build_rows_ms =
          NanosToMs(rough_build_rows_ns.load(std::memory_order_relaxed));
      const double rough_eval_ms =
          NanosToMs(rough_eval_ns.load(std::memory_order_relaxed));
      const double refine_ms =
          NanosToMs(refine_ns.load(std::memory_order_relaxed));
      const double refined_row_cluster_ms =
          NanosToMs(refined_row_cluster_ns.load(std::memory_order_relaxed));
      const double refined_build_rows_ms =
          NanosToMs(refined_build_rows_ns.load(std::memory_order_relaxed));
      const double refined_eval_ms =
          NanosToMs(refined_eval_ns.load(std::memory_order_relaxed));
      fprintf(stderr,
              "PLANNER: [bicluster] Candidate stages (sum/avg ms): "
              "rough_opt=%.2f/%.2f rough_row_cluster=%.2f/%.2f "
              "rough_build_rows=%.2f/%.2f rough_eval=%.2f/%.2f "
              "refine=%.2f/%.2f refined_row_cluster=%.2f/%.2f "
              "refined_build_rows=%.2f/%.2f "
              "refined_eval=%.2f/%.2f\n",
              rough_opt_ms, rough_opt_ms / num_processed, rough_row_cluster_ms,
              rough_row_cluster_ms / num_processed, rough_build_rows_ms,
              rough_build_rows_ms / num_processed, rough_eval_ms,
              rough_eval_ms / num_processed, refine_ms,
              refine_ms / num_processed, refined_row_cluster_ms,
              refined_row_cluster_ms / num_processed, refined_build_rows_ms,
              refined_build_rows_ms / num_processed, refined_eval_ms,
              refined_eval_ms / num_processed);
    }
    fflush(stderr);
    if (best_pass_cost != std::numeric_limits<FixedPointCost>::max()) {
      fprintf(stderr,
              "PLANNER: [bicluster] Best cost for %u passes = %.2f bits\n",
              num_passes, bit_cost(best_pass_cost));
      fflush(stderr);
    }
    auto end_pass_config = std::chrono::high_resolution_clock::now();
    fprintf(stderr, "PLANNER: [bicluster] Pass configuration %u took %.2f ms\n",
            num_passes,
            std::chrono::duration<double, std::milli>(end_pass_config -
                                                      start_pass_config)
                .count());
    fflush(stderr);
  }

  if (best_result.total_cost == std::numeric_limits<FixedPointCost>::max()) {
    return JXL_FAILURE("Biclustered search did not produce a result");
  }
  return best_result;
}

// Convenience overload that ranks/trims factorization candidates first.
StatusOr<BiclusterSearchResult> SearchBiclusteredContextModel(
    std::shared_ptr<const JPEGOptData> opt_data,
    const JPEGCtxEffortParams& effort, ThreadPool* pool) {
  JXL_ASSIGN_OR_RETURN(std::vector<FactorizationCandidate> candidates,
                       RankAndTrimFactorizations(opt_data, effort, pool));
  return SearchBiclusteredContextModel(opt_data, candidates, effort, pool);
}

}  // namespace jxl
