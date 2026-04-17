// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include "lib/jxl/transcode_jpeg/enc_jpeg_passes.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <chrono>
#include <vector>

#include "lib/jxl/enc_ans_params.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_axis_maps.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_cluster.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_histogram.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_stream.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_threshold.h"

namespace jxl {

namespace {

using SparseHistogram = std::vector<std::unordered_map<uint32_t, uint32_t>>;

struct ActiveRawBins {
  std::vector<ACBin> active_bins;
  std::vector<uint32_t> raw_to_compact;
  std::vector<uint16_t> compact_to_czdc;
};

struct ClusterResult {
  ContextMap ctx_map;
  uint32_t num_clusters = 0;
};

struct ModelEvaluation {
  FixedPointCost ac_cost = 0;
  FixedPointCost nz_cost = 0;
  FixedPointCost signalling_overhead = 0;

  FixedPointCost total_cost() const {
    return ac_cost + nz_cost + signalling_overhead;
  }
};

void BlockDCIndices(const JPEGOptData& d, uint32_t c, uint32_t b, uint32_t* dc0,
                    uint32_t* dc1, uint32_t* dc2);
uint32_t BlockCell(const JPEGOptData& d, const AxisMaps& axis_maps,
                   const ThresholdSet& thresholds, uint32_t c, uint32_t b);

struct RowSliceHistograms {
  uint32_t num_channels = 0;
  uint32_t num_cells = 0;
  uint32_t num_passes = 1;
  std::vector<CompactHistogram> ac_hist;
  std::vector<uint32_t> ac_total;
  std::vector<DenseHistogram<kJPEGNonZeroRange>> nz_hist;
  std::vector<uint32_t> nz_total;
};

CompactHistogram& EnsureACEntry(RowSliceHistograms* rows, uint32_t row,
                                uint32_t pass, uint32_t zdc,
                                uint32_t ac_alphabet_size) {
  const size_t idx =
      (static_cast<size_t>(row) * rows->num_passes + pass) *
          kZeroDensityContextCount +
      zdc;
  CompactHistogram& hist = rows->ac_hist[idx];
  if (hist.counts.empty()) {
    hist = CompactHistogram(ac_alphabet_size);
  }
  return hist;
}

StatusOr<RowSliceHistograms> BuildRowSliceHistograms(
    const JPEGOptData& d, const ThresholdSet& thresholds,
    const PassAssignment& pass_assignment, uint32_t num_passes) {
  AxisMaps axis_maps(d);
  axis_maps.Update(thresholds);
  const uint32_t n0 = static_cast<uint32_t>(thresholds.TY().size() + 1);
  const uint32_t num_cells =
      n0 * static_cast<uint32_t>(thresholds.TCb().size() + 1) *
      static_cast<uint32_t>(thresholds.TCr().size() + 1);
  const uint32_t total_rows = d.channels * num_cells;

  RowSliceHistograms rows;
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
    for (uint32_t b = 0; b < d.num_blocks[c]; ++b) {
      uint32_t dc0 = 0;
      uint32_t dc1 = 0;
      uint32_t dc2 = 0;
      BlockDCIndices(d, c, b, &dc0, &dc1, &dc2);
      const uint32_t cell =
          (axis_maps.ax1_row[dc1] + axis_maps.ax2_col[dc2]) * n0 +
          axis_maps.ax0_to_k[dc0];
      const uint32_t row = c * num_cells + cell;
      const uint32_t pass = pass_assignment[c][b];

      for (uint32_t pi = d.block_offsets[c][b]; pi < d.block_offsets[c][b + 1];
           ++pi) {
        const CompactACEvent ac_event = d.FromBin(d.block_bins[c][pi]);
        CompactHistogram& hist =
            EnsureACEntry(&rows, row, pass, ac_event.zdc, d.ACHistogramSize());
        hist.Add(ac_event.hist_bin);
        const size_t idx =
            (static_cast<size_t>(row) * num_passes + pass) *
                kZeroDensityContextCount +
            ac_event.zdc;
        ++rows.ac_total[idx];
      }
    }
  }

  for (uint32_t c = 0; c < d.channels; ++c) {
    for (uint32_t y = 0; y < d.block_grid_h[c]; ++y) {
      for (uint32_t x = 0; x < d.block_grid_w[c]; ++x) {
        const uint32_t b = y * d.block_grid_w[c] + x;
        const uint32_t cell = BlockCell(d, axis_maps, thresholds, c, b);
        const uint32_t row = c * num_cells + cell;
        const uint32_t pass = pass_assignment[c][b];

        uint32_t b_top = (y - 1) * d.block_grid_w[c] + x;
        uint32_t b_left = y * d.block_grid_w[c] + (x - 1);
        uint32_t nz_top = (y > 0) ? d.block_nonzeros[c][b_top] : 0u;
        uint32_t nz_left = (x > 0) ? d.block_nonzeros[c][b_left] : 0u;
        uint8_t nz_top_pass = (y > 0) ? pass_assignment[c][b_top] : 255;
        uint8_t nz_left_pass = (x > 0) ? pass_assignment[c][b_left] : 255;

        uint32_t predicted_nz;
        uint32_t pass_nz_top = (nz_top_pass == pass) ? nz_top : 0u;
        uint32_t pass_nz_left = (nz_left_pass == pass) ? nz_left : 0u;
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
        const size_t idx = (static_cast<size_t>(row) * num_passes + pass) *
                               kJPEGNonZeroBuckets +
                           pb;
        rows.nz_hist[idx].Add(d.block_nonzeros[c][b]);
        ++rows.nz_total[idx];
      }
    }
  }

  return rows;
}

StatusOr<ModelEvaluation> EvaluateBiclusterState(
    const JPEGOptData& d, const ThresholdSet& thresholds, const ContextMap& ctx_map,
    uint32_t num_row_clusters, const PassAssignment& pass_assignment,
    uint32_t num_passes, const RowSliceHistograms& rows,
    std::vector<uint32_t>* num_prototypes_per_pass) {
  ModelEvaluation eval;
  const uint32_t total_rows = rows.num_channels * rows.num_cells;
  const size_t ac_slots =
      static_cast<size_t>(num_row_clusters) * num_passes * kZeroDensityContextCount;
  const size_t nz_slots =
      static_cast<size_t>(num_row_clusters) * num_passes * kJPEGNonZeroBuckets;
  std::vector<CompactHistogram> ac_cluster_hist(ac_slots,
                                                CompactHistogram(d.ACHistogramSize()));
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
        if (!rows.ac_hist[src].counts.empty()) {
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
  for (uint32_t cluster = 0; cluster < num_row_clusters; ++cluster) {
    for (uint32_t pass = 0; pass < num_passes; ++pass) {
      for (uint32_t zdc = 0; zdc < kZeroDensityContextCount; ++zdc) {
        const size_t idx = (static_cast<size_t>(cluster) * num_passes + pass) *
                               kZeroDensityContextCount +
                           zdc;
        if (ac_cluster_total[idx] == 0) continue;
        ++(*num_prototypes_per_pass)[pass];
        eval.ac_cost += d.ftab[ac_cluster_total[idx]];
        ac_cluster_hist[idx].ForEachNonZero(
            [&](uint32_t, uint32_t freq) { eval.ac_cost -= d.ftab[freq]; });
        std::array<uint32_t, kACTokenCount> token_counts = {};
        const auto& dense_to_zdcvalue = d.ACHistogram().dense_to_zdcvalue;
        ac_cluster_hist[idx].ForEachNonZero([&](uint32_t id, uint32_t freq) {
          const SignallingHistSymbol hist_symbol =
              d.SignallingHistSymbolFromSymbol(dense_to_zdcvalue[id]);
          token_counts[hist_symbol.token] += freq;
        });
        uint32_t max_token = 0;
        size_t total = 0;
        for (uint32_t t = 0; t < kACTokenCount; ++t) {
          if (token_counts[t] == 0) continue;
          max_token = t;
          total += token_counts[t];
        }
        if (total != 0) {
          Histogram h(max_token + 1);
          for (uint32_t t = 0; t <= max_token; ++t) {
            h.counts[t] = static_cast<ANSHistBin>(token_counts[t]);
          }
          h.total_count = total;
          JXL_ASSIGN_OR_RETURN(FixedPointCost header_cost, HistogramHeaderCost(h));
          eval.signalling_overhead += header_cost;
        }
      }

      for (uint32_t pb = 0; pb < kJPEGNonZeroBuckets; ++pb) {
        const size_t idx = (static_cast<size_t>(cluster) * num_passes + pass) *
                               kJPEGNonZeroBuckets +
                           pb;
        if (nz_cluster_total[idx] == 0) continue;
        ++(*num_prototypes_per_pass)[pass];
        eval.nz_cost += d.NZFTab(nz_cluster_total[idx]);
        for (uint32_t nz = 0; nz < kJPEGNonZeroRange; ++nz) {
          eval.nz_cost -= d.NZFTab(nz_cluster_hist[idx][nz]);
        }
        uint32_t max_nz = 0;
        for (uint32_t nz = 0; nz < kJPEGNonZeroRange; ++nz) {
          if (nz_cluster_hist[idx][nz] != 0) max_nz = nz;
        }
        Histogram h(max_nz + 1);
        h.total_count = nz_cluster_total[idx];
        for (uint32_t nz = 0; nz <= max_nz; ++nz) {
          h.counts[nz] = static_cast<ANSHistBin>(nz_cluster_hist[idx][nz]);
        }
        JXL_ASSIGN_OR_RETURN(FixedPointCost header_cost, HistogramHeaderCost(h));
        eval.signalling_overhead += header_cost;
      }
    }
  }
  (void)thresholds;
  (void)pass_assignment;
  return eval;
}

struct EmitBin {
  ACBin raw_bin;
  uint32_t hist_key;
  uint32_t compact_id;
};

ActiveRawBins BuildActiveRawBins(const JPEGOptData& d) {
  ActiveRawBins out;
  const size_t raw_bin_count = static_cast<size_t>(d.channels) * kMaxACSymbolCount;
  out.raw_to_compact.assign(raw_bin_count, kInvalidCompactH);
  out.active_bins.reserve(d.ACHistogram().dense_to_zdcvalue.size());
  for (uint32_t c = 0; c < d.channels; ++c) {
    for (uint32_t b = 0; b < d.num_blocks[c]; ++b) {
      for (uint32_t pi = d.block_offsets[c][b];
           pi < d.block_offsets[c][b + 1]; ++pi) {
        const ACBin bin = d.block_bins[c][pi];
        if (out.raw_to_compact[bin] != kInvalidCompactH) continue;
        out.raw_to_compact[bin] = static_cast<uint32_t>(out.active_bins.size());
        out.active_bins.push_back(bin);
        out.compact_to_czdc.push_back(static_cast<uint16_t>(JPEGOptData::ACBinCZDC(bin)));
      }
    }
  }
  return out;
}

template <typename Func>
void ForEachBlockBin(const JPEGOptData& d, uint32_t c, uint32_t b, Func&& fn) {
  for (uint32_t pi = d.block_offsets[c][b]; pi < d.block_offsets[c][b + 1];
       ++pi) {
    fn(d.block_bins[c][pi]);
  }
}

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

PassAssignment AssignPassesGreedy(const JPEGOptData& d, const ActiveRawBins& active,
                                  uint32_t num_passes) {
  PassAssignment pass_assignment;
  for (uint32_t c = 0; c < kNumCh; ++c) {
    pass_assignment[c].assign(d.num_blocks[c], 0);
  }
  if (num_passes <= 1 || active.active_bins.empty()) return pass_assignment;

  struct BlockRef {
    uint16_t c;
    uint32_t b;
  };
  std::vector<BlockRef> blocks;
  for (uint32_t c = 0; c < d.channels; ++c) {
    for (uint32_t b = 0; b < d.num_blocks[c]; ++b) {
      if (d.block_offsets[c][b] == d.block_offsets[c][b + 1]) continue;
      blocks.push_back({static_cast<uint16_t>(c), b});
    }
  }
  if (blocks.empty()) return pass_assignment;

  const uint32_t M = static_cast<uint32_t>(active.active_bins.size());
  const uint32_t czdc_size = d.channels * kZeroDensityContextCount;
  std::vector<uint32_t> hist_h(static_cast<size_t>(M) * num_passes, 0);
  std::vector<uint32_t> hist_N(static_cast<size_t>(czdc_size) * num_passes, 0);

  for (size_t i = 0; i < blocks.size(); ++i) {
    const uint32_t pass =
        std::min(static_cast<uint32_t>(i * num_passes / blocks.size()),
                 num_passes - 1);
    const BlockRef& ref = blocks[i];
    pass_assignment[ref.c][ref.b] = static_cast<uint8_t>(pass);
    ForEachBlockBin(d, ref.c, ref.b, [&](ACBin bin) {
      const uint32_t compact_id = active.raw_to_compact[bin];
      const uint32_t czdc = active.compact_to_czdc[compact_id];
      ++hist_h[static_cast<size_t>(compact_id) * num_passes + pass];
      ++hist_N[static_cast<size_t>(czdc) * num_passes + pass];
    });
  }

  std::vector<FixedPointCost> delta(num_passes);
  std::vector<uint16_t> touched_czdc;
  touched_czdc.reserve(128);
  std::vector<uint32_t> czdc_counts(czdc_size, 0);

  auto find_best_pass = [&](const BlockRef& ref, uint32_t cur_pass) {
    std::fill(delta.begin(), delta.end(), 0);
    touched_czdc.clear();

    ForEachBlockBin(d, ref.c, ref.b, [&](ACBin bin) {
      const uint32_t compact_id = active.raw_to_compact[bin];
      const uint32_t czdc = active.compact_to_czdc[compact_id];
      if (czdc_counts[czdc]++ == 0) touched_czdc.push_back(static_cast<uint16_t>(czdc));

      const uint32_t* h_row = &hist_h[static_cast<size_t>(compact_id) * num_passes];
      const FixedPointCost rm_cost = d.ftab[h_row[cur_pass] - 1] - d.ftab[h_row[cur_pass]];
      for (uint32_t p = 0; p < num_passes; ++p) {
        delta[p] -= (d.ftab[h_row[p] + 1] - d.ftab[h_row[p]]) + rm_cost;
      }
    });

    for (uint16_t czdc : touched_czdc) {
      const uint32_t n = czdc_counts[czdc];
      czdc_counts[czdc] = 0;
      const uint32_t* n_row = &hist_N[static_cast<size_t>(czdc) * num_passes];
      const FixedPointCost rm_cost = d.ftab[n_row[cur_pass] - n] - d.ftab[n_row[cur_pass]];
      for (uint32_t p = 0; p < num_passes; ++p) {
        delta[p] += rm_cost + d.ftab[n_row[p] + n] - d.ftab[n_row[p]];
      }
    }

    delta[cur_pass] = 1;
    FixedPointCost best_delta = 0;
    uint32_t best_pass = cur_pass;
    for (uint32_t p = 0; p < num_passes; ++p) {
      if (delta[p] < best_delta) {
        best_delta = delta[p];
        best_pass = p;
      }
    }
    return best_pass;
  };

  constexpr uint32_t kMaxIters = 50;
  for (uint32_t iter = 0; iter < kMaxIters; ++iter) {
    uint32_t moves = 0;
    for (const BlockRef& ref : blocks) {
      const uint32_t cur_pass = pass_assignment[ref.c][ref.b];
      const uint32_t new_pass = find_best_pass(ref, cur_pass);
      if (new_pass == cur_pass) continue;
      ForEachBlockBin(d, ref.c, ref.b, [&](ACBin bin) {
        const uint32_t compact_id = active.raw_to_compact[bin];
        const uint32_t czdc = active.compact_to_czdc[compact_id];
        --hist_h[static_cast<size_t>(compact_id) * num_passes + cur_pass];
        ++hist_h[static_cast<size_t>(compact_id) * num_passes + new_pass];
        --hist_N[static_cast<size_t>(czdc) * num_passes + cur_pass];
        ++hist_N[static_cast<size_t>(czdc) * num_passes + new_pass];
      });
      pass_assignment[ref.c][ref.b] = static_cast<uint8_t>(new_pass);
      ++moves;
    }
    if (moves == 0) break;
  }

  return pass_assignment;
}

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

StatusOr<std::vector<ACEntry>> BuildPassStream(
    const JPEGOptData& d, const ActiveRawBins& active,
    const PassAssignment& pass_assignment, uint32_t num_passes,
    std::vector<uint32_t>* pass_offsets) {
  const uint32_t M = static_cast<uint32_t>(active.active_bins.size());
  const uint32_t num_buckets = num_passes * M * 2;
  std::vector<uint32_t> bucket_start(static_cast<size_t>(num_buckets) + 1, 0);
  for (uint32_t c = 0; c < d.channels; ++c) {
    for (uint32_t b = 0; b < d.num_blocks[c]; ++b) {
      uint32_t dc0 = 0;
      uint32_t dc1 = 0;
      uint32_t dc2 = 0;
      BlockDCIndices(d, c, b, &dc0, &dc1, &dc2);
      const uint32_t pass = pass_assignment[c][b];
      for (uint32_t pi = d.block_offsets[c][b]; pi < d.block_offsets[c][b + 1];
           ++pi) {
        const ACBin raw_bin = d.block_bins[c][pi];
        const uint32_t compact_id = active.raw_to_compact[raw_bin];
        ++bucket_start[(static_cast<size_t>(pass) * M + compact_id) * 2 +
                       (dc0 >> 10) + 1];
      }
    }
  }
  for (size_t i = 0; i < num_buckets; ++i) {
    bucket_start[i + 1] += bucket_start[i];
  }

  std::vector<uint32_t> flat(bucket_start.back());
  std::vector<uint32_t> write_pos = bucket_start;
  for (uint32_t c = 0; c < d.channels; ++c) {
    for (uint32_t b = 0; b < d.num_blocks[c]; ++b) {
      uint32_t dc0 = 0;
      uint32_t dc1 = 0;
      uint32_t dc2 = 0;
      BlockDCIndices(d, c, b, &dc0, &dc1, &dc2);
      const uint32_t pass = pass_assignment[c][b];
      for (uint32_t pi = d.block_offsets[c][b]; pi < d.block_offsets[c][b + 1];
           ++pi) {
        const ACBin raw_bin = d.block_bins[c][pi];
        const uint32_t compact_id = active.raw_to_compact[raw_bin];
        const uint32_t bucket =
            (static_cast<size_t>(pass) * M + compact_id) * 2 + (dc0 >> 10);
        flat[write_pos[bucket]++] = ((dc0 & 0x3FFu) << 22) | (dc1 << 11) | dc2;
      }
    }
  }

  for (uint32_t bucket = 0; bucket < num_buckets; ++bucket) {
    const uint32_t begin = bucket_start[bucket];
    const uint32_t end = bucket_start[bucket + 1];
    if (begin < end) {
      std::sort(flat.begin() + begin, flat.begin() + end);
    }
  }

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

FixedPointCost HistogramCost(const JPEGOptData& d,
                      const std::unordered_map<uint32_t, uint32_t>& hist) {
  FixedPointCost cost = 0;
  for (const auto& entry : hist) {
    cost += d.NZFTab(entry.second);
  }
  return cost;
}

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

  if (active.size() <= target_clusters) {
    ClusterResult out;
    out.num_clusters = static_cast<uint32_t>(std::max<size_t>(1, active.size()));
    out.ctx_map.assign(total_ctxs, 0);
    for (uint32_t i = 0; i < active.size(); ++i) {
      out.ctx_map[active[i]] = static_cast<uint8_t>(i);
    }
    return out;
  }

  std::vector<uint32_t> parent(total_ctxs);
  for (uint32_t i = 0; i < total_ctxs; ++i) parent[i] = i;

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

  // Fill NZ histograms for each pass
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
          uint32_t nz = pass == p ? d.block_nonzeros[c][b] : 0u;
          ++nz_hist_h[cp][NZHistogramIndex(pb, nz)];
          ++nz_hist_N[cp][pb];
        }
      }
    }
  }

  ModelEvaluation eval;
  for (uint32_t cp = 0; cp < cp_count; ++cp) {
    for (const auto& entry : ac_hist_N[cp]) eval.ac_cost += d.ftab[entry.second];
    for (const auto& entry : ac_hist_h[cp]) eval.ac_cost -= d.ftab[entry.second];
    for (const auto& entry : nz_hist_N[cp]) eval.nz_cost += d.NZFTab(entry.second);
    for (const auto& entry : nz_hist_h[cp]) eval.nz_cost -= d.NZFTab(entry.second);
    JXL_ASSIGN_OR_RETURN(FixedPointCost ac_overhead,
                         SignalOverheadFromHist(d, ac_hist_h[cp]));
    JXL_ASSIGN_OR_RETURN(FixedPointCost nz_overhead,
                         SignalOverheadFromNZHist(nz_hist_h[cp]));
    eval.signalling_overhead += ac_overhead + nz_overhead;
  }
  return eval;
}

ThresholdSet RefinePassAwareThresholds(
    PartitioningCtx& ctx, const ThresholdSet& thresholds,
    const std::vector<ACEntry>& pass_stream,
    const JPEGCtxEffortParams& effort) {
  if (effort.refine_iters == 0) return thresholds;
  FixedPointCost ignored_cost = 0;
  return ctx.OptimizeThresholds(thresholds, pass_stream, effort.main_m_target,
                                effort.refine_iters, &ignored_cost);
}

uint32_t ComputeMaxNumPasses(const JPEGOptData& d) {
  const double groups_x = static_cast<double>((d.w_max + 31) / 32);
  const double groups_y = static_cast<double>((d.h_max + 31) / 32);
  const double groups = std::max(1.0, groups_x * groups_y);
  return static_cast<uint32_t>(
      std::min(11.0, std::ceil(std::log2(groups)) + 1.0));
}

}  // namespace

StatusOr<PassSearchResult> SearchPassAwareContextModel(
    std::shared_ptr<const JPEGOptData> opt_data,
    const std::vector<FactorizationCandidate>& candidates,
    const JPEGCtxEffortParams& effort, ThreadPool* pool) {
  if (candidates.empty()) {
    return JXL_FAILURE("Pass-aware search requires at least one candidate");
  }

  const JPEGOptData& d = *opt_data;
  const ActiveRawBins active = BuildActiveRawBins(d);
  //const uint32_t max_num_passes = ComputeMaxNumPasses(d);
  const uint32_t target_clusters =
      kMaxClusters - static_cast<uint32_t>(d.channels == 1);

  PassSearchResult best_result;
  best_result.total_cost = std::numeric_limits<FixedPointCost>::max();
  std::mutex mu;

  //for (uint32_t num_passes = 1; num_passes <= max_num_passes; ++num_passes) 
  {
    uint32_t num_passes = 3;
    auto start_pass_config = std::chrono::high_resolution_clock::now();
    fprintf(stderr, "PLANNER: Testing configuration with %u passes\n", num_passes);
    fflush(stderr);
    PassAssignment pass_assignment =
        AssignPassesGreedy(d, active, num_passes);
    std::vector<uint32_t> pass_offsets;
    JXL_ASSIGN_OR_RETURN(std::vector<ACEntry> pass_stream,
                         BuildPassStream(d, active, pass_assignment, num_passes,
                                         &pass_offsets));

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
          PartitioningCtx& ctx = ctx_pool[thread_id];
          const FactorizationCandidate& candidate = candidates[idx];

          FixedPointCost rough_unclustered_cost = 0;
          ThresholdSet rough_thresholds =
              ctx.OptimizeThresholds(candidate.init, pass_stream,
                                     effort.main_m_target, effort.main_iters,
                                     &rough_unclustered_cost);

          JXL_ASSIGN_OR_RETURN(
              ClusterResult cluster_result,
              ClusterContextsPassAware(d, rough_thresholds, pass_stream,
                                       pass_offsets, num_passes,
                                       target_clusters));

          JXL_ASSIGN_OR_RETURN(
              ModelEvaluation rough_eval,
              EvaluatePassAwareModel(d, rough_thresholds, cluster_result.ctx_map,
                                     cluster_result.num_clusters, pass_assignment,
                                     num_passes, pass_stream, pass_offsets));

          ThresholdSet refined_thresholds =
              RefinePassAwareThresholds(ctx, rough_thresholds, pass_stream,
                                        effort);
          JXL_ASSIGN_OR_RETURN(
              ModelEvaluation refined_eval,
              EvaluatePassAwareModel(d, refined_thresholds, cluster_result.ctx_map,
                                     cluster_result.num_clusters, pass_assignment,
                                     num_passes, pass_stream, pass_offsets));

          const bool refined_is_better =
              refined_eval.total_cost() < rough_eval.total_cost();
          const ThresholdSet& best_thresholds =
              refined_is_better ? refined_thresholds : rough_thresholds;
          const ModelEvaluation& best_eval =
              refined_is_better ? refined_eval : rough_eval;

          std::lock_guard<std::mutex> lock(mu);
          if (best_eval.total_cost() < best_result.total_cost) {
            best_result.thresholds = best_thresholds;
            best_result.ctx_map = cluster_result.ctx_map;
            best_result.pass_assignment = pass_assignment;
            best_result.num_passes = num_passes;
            best_result.num_clusters = cluster_result.num_clusters;
            best_result.ac_cost = best_eval.ac_cost;
            best_result.nz_cost = best_eval.nz_cost;
            best_result.signalling_overhead = best_eval.signalling_overhead;
            best_result.total_cost = best_eval.total_cost();
          }
          return true;
        },
        "JpegCtxPasses"));
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

StatusOr<BiclusterSearchResult> SearchBiclusteredContextModel(
    std::shared_ptr<const JPEGOptData> opt_data,
    const std::vector<FactorizationCandidate>& candidates,
    const JPEGCtxEffortParams& effort, ThreadPool* pool) {
  if (candidates.empty()) {
    return JXL_FAILURE("Biclustered search requires at least one candidate");
  }

  const JPEGOptData& d = *opt_data;
  const ActiveRawBins active = BuildActiveRawBins(d);
  const uint32_t target_clusters =
      kMaxClusters - static_cast<uint32_t>(d.channels == 1);
  const uint32_t target_num_passes = 4;
  BiclusterSearchResult best_result;
  best_result.total_cost = std::numeric_limits<FixedPointCost>::max();
  std::mutex mu;

  PassAssignment pass_assignment =
      AssignPassesGreedy(d, active, target_num_passes);
  std::vector<uint32_t> pass_offsets;
  JXL_ASSIGN_OR_RETURN(std::vector<ACEntry> pass_stream,
                       BuildPassStream(d, active, pass_assignment,
                                       target_num_passes, &pass_offsets));

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
        PartitioningCtx& ctx = ctx_pool[thread_id];
        const FactorizationCandidate& candidate = candidates[idx];

        FixedPointCost rough_unclustered_cost = 0;
        ThresholdSet rough_thresholds =
            ctx.OptimizeThresholds(candidate.init, pass_stream,
                                   effort.main_m_target, effort.main_iters,
                                   &rough_unclustered_cost);

        JXL_ASSIGN_OR_RETURN(
            ClusterResult cluster_result,
            ClusterContextsPassAware(d, rough_thresholds, pass_stream,
                                     pass_offsets, target_num_passes,
                                     std::min(target_clusters,
                                              effort.bicluster_row_budget)));

        JXL_ASSIGN_OR_RETURN(
            RowSliceHistograms rough_rows,
            BuildRowSliceHistograms(d, rough_thresholds, pass_assignment,
                                    target_num_passes));
        std::vector<uint32_t> rough_num_prototypes;
        JXL_ASSIGN_OR_RETURN(ModelEvaluation rough_eval,
                             EvaluateBiclusterState(
                                 d, rough_thresholds, cluster_result.ctx_map,
                                 cluster_result.num_clusters, pass_assignment,
                                 target_num_passes, rough_rows,
                                 &rough_num_prototypes));

        ThresholdSet refined_thresholds =
            RefinePassAwareThresholds(ctx, rough_thresholds, pass_stream, effort);
        JXL_ASSIGN_OR_RETURN(
            RowSliceHistograms refined_rows,
            BuildRowSliceHistograms(d, refined_thresholds, pass_assignment,
                                    target_num_passes));
        std::vector<uint32_t> refined_num_prototypes;
        JXL_ASSIGN_OR_RETURN(ModelEvaluation refined_eval,
                             EvaluateBiclusterState(
                                 d, refined_thresholds, cluster_result.ctx_map,
                                 cluster_result.num_clusters, pass_assignment,
                                 target_num_passes, refined_rows,
                                 &refined_num_prototypes));

        const bool refined_is_better =
            refined_eval.total_cost() < rough_eval.total_cost();
        const ThresholdSet& best_thresholds =
            refined_is_better ? refined_thresholds : rough_thresholds;
        const ModelEvaluation& best_eval =
            refined_is_better ? refined_eval : rough_eval;
        const std::vector<uint32_t>& best_num_prototypes =
            refined_is_better ? refined_num_prototypes : rough_num_prototypes;

        std::lock_guard<std::mutex> lock(mu);
        if (best_eval.total_cost() < best_result.total_cost) {
          best_result.thresholds = best_thresholds;
          best_result.ctx_map = cluster_result.ctx_map;
          best_result.pass_assignment = pass_assignment;
          best_result.num_passes = target_num_passes;
          best_result.num_cells =
              static_cast<uint32_t>(cluster_result.ctx_map.size() / d.channels);
          best_result.num_row_clusters = cluster_result.num_clusters;
          best_result.num_prototypes_per_pass = best_num_prototypes;
          best_result.total_num_prototypes = 0;
          for (uint32_t n : best_num_prototypes) {
            best_result.total_num_prototypes += n;
          }
          best_result.ac_cost = best_eval.ac_cost;
          best_result.nz_cost = best_eval.nz_cost;
          best_result.signalling_overhead = best_eval.signalling_overhead;
          best_result.total_cost = best_eval.total_cost();
        }
        return true;
      },
      "JpegCtxBicluster"));

  if (best_result.total_cost == std::numeric_limits<FixedPointCost>::max()) {
    return JXL_FAILURE("Biclustered search did not produce a result");
  }
  return best_result;
}

StatusOr<BiclusterSearchResult> SearchBiclusteredContextModel(
    std::shared_ptr<const JPEGOptData> opt_data,
    const JPEGCtxEffortParams& effort, ThreadPool* pool) {
  JXL_ASSIGN_OR_RETURN(std::vector<FactorizationCandidate> candidates,
                       RankAndTrimFactorizations(opt_data, effort, pool));
  return SearchBiclusteredContextModel(opt_data, candidates, effort, pool);
}

}  // namespace jxl
