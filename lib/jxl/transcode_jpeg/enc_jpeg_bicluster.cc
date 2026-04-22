// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.


#include "lib/jxl/transcode_jpeg/enc_jpeg_bicluster.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "lib/jxl/enc_ans_params.h"
#include "lib/jxl/enc_cluster.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_cluster.h"
#include "lib/jxl/frame_dimensions.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_axis_maps.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_histogram.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_cluster.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_stream.h"

namespace jxl {

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
    FixedPointCost cutoff) {
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


}  // namespace jxl
