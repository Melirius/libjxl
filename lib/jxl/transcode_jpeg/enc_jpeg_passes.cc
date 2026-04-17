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

// Dense remapping of the AC bins that are actually present in the image.
// `raw_to_compact` maps raw `ACBin` ids to a compact active-bin index, and
// `compact_to_czdc` stores the corresponding `(channel, zdc)` selector used by
// pass assignment.
struct ActiveRawBins {
  std::vector<ACBin> active_bins;
  std::vector<uint32_t> raw_to_compact;
  std::vector<uint16_t> compact_to_czdc;
};

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
                                        : ac_cost + nz_cost) +
           signalling_overhead;
  }
};

void BlockDCIndices(const JPEGOptData& d, uint32_t c, uint32_t b, uint32_t* dc0,
                    uint32_t* dc1, uint32_t* dc2);
uint32_t BlockCell(const JPEGOptData& d, const AxisMaps& axis_maps,
                   const ThresholdSet& thresholds, uint32_t c, uint32_t b);

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

// --- Biclustering-state helpers ------------------------------------------------

// Builds the fixed `(row, pass, slice)` histogram lattice for one threshold
// set and one block-to-pass assignment. AC slices are stored already regrouped
// by signalling token within a fixed `zdc`, so the per-slice alphabet stays
// small (`kACTokenCount`) and memory use stays bounded.
StatusOr<RowSliceHistograms> BuildRowSliceHistograms(
    const JPEGOptData& d, const ThresholdSet& thresholds,
    const PassAssignment& pass_assignment, uint32_t num_passes) {
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

// Appends the non-empty clustered AC and nz slice histograms for one pass to a
// flat generic `Histogram` pool so the encoder's `ClusterHistograms` helper can
// be used directly on the biclustering prototype state.
void BuildBiclusterPassHistograms(
    uint32_t pass, uint32_t num_passes, uint32_t num_row_clusters,
    const std::vector<DenseHistogram<kACTokenCount>>& ac_cluster_hist,
    const std::vector<uint32_t>& ac_cluster_total,
    const std::vector<DenseHistogram<kJPEGNonZeroRange>>& nz_cluster_hist,
    const std::vector<uint32_t>& nz_cluster_total,
    std::vector<Histogram>* histograms) {
  histograms->clear();
  histograms->reserve(static_cast<size_t>(num_row_clusters) *
                      (kZeroDensityContextCount + kJPEGNonZeroBuckets));

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
      histograms->push_back(std::move(h));
    }
  }
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
    std::vector<uint32_t>* num_prototypes_per_pass) {
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
  std::vector<Histogram> pass_histograms;
  std::vector<Histogram> clustered;
  std::vector<uint32_t> histogram_symbols;
  for (uint32_t pass = 0; pass < num_passes; ++pass) {
    BuildBiclusterPassHistograms(pass, num_passes, num_row_clusters,
                                 ac_cluster_hist, ac_cluster_total,
                                 nz_cluster_hist, nz_cluster_total,
                                 &pass_histograms);
    clustered.clear();
    histogram_symbols.clear();
    if (pass_histograms.empty()) continue;
    JXL_RETURN_IF_ERROR(ClusterHistograms(params, pass_histograms,
                                          proto_budget_per_pass, &clustered,
                                          &histogram_symbols));
    (*num_prototypes_per_pass)[pass] = clustered.size();
    for (const auto& h : clustered) {
      eval.corrected_entropy_cost +=
          static_cast<FixedPointCost>(h.ShannonEntropy() * kFScale);
      JXL_ASSIGN_OR_RETURN(FixedPointCost header_cost, HistogramHeaderCost(h));
      eval.signalling_overhead += header_cost;
    }
  }
  (void)thresholds;
  (void)pass_assignment;
  return eval;
}

// --- Pass-aware stream construction --------------------------------------------

struct EmitBin {
  ACBin raw_bin;
  uint32_t hist_key;
  uint32_t compact_id;
};

// Scans the JPEG AC stream and compacts the raw `ACBin` space down to only the
// bins that actually occur in the image. The resulting active-bin tables drive
// both greedy pass assignment and pass-stream construction.
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

// Greedy local-search assignment of image blocks to progressive AC passes.
// The objective is the same `ftab`-based entropy proxy used elsewhere in the
// planner, but applied to pass-local AC and `zdc` counts.
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

// --- Pass-aware clustering and scoring -----------------------------------------

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

// Heuristic upper bound for the number of progressive passes worth considering
// from the image size. The current experimental path still hardcodes a single
// pass count, but this helper documents the intended scaling rule.
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
    auto start_assign_passes = PlannerClock::now();
    PassAssignment pass_assignment =
        AssignPassesGreedy(d, active, num_passes);
    auto end_assign_passes = PlannerClock::now();
    fprintf(stderr, "PLANNER: AssignPassesGreedy took %.2f ms\n",
            NanosToMs(ElapsedNanos(start_assign_passes, end_assign_passes)));
    fflush(stderr);

    auto start_build_stream = PlannerClock::now();
    std::vector<uint32_t> pass_offsets;
    JXL_ASSIGN_OR_RETURN(std::vector<ACEntry> pass_stream,
                         BuildPassStream(d, active, pass_assignment, num_passes,
                                         &pass_offsets));
    auto end_build_stream = PlannerClock::now();
    fprintf(stderr, "PLANNER: BuildPassStream took %.2f ms\n",
            NanosToMs(ElapsedNanos(start_build_stream, end_build_stream)));
    fflush(stderr);

    std::atomic<int64_t> rough_opt_ns(0);
    std::atomic<int64_t> cluster_ns(0);
    std::atomic<int64_t> rough_eval_ns(0);
    std::atomic<int64_t> refine_ns(0);
    std::atomic<int64_t> refined_eval_ns(0);
    std::atomic<uint32_t> processed_candidates(0);

    std::vector<PartitioningCtx> ctx_pool;
    auto start_candidate_loop = PlannerClock::now();
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
          processed_candidates.fetch_add(1, std::memory_order_relaxed);

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
    auto end_candidate_loop = PlannerClock::now();
    const uint32_t num_processed = processed_candidates.load(
        std::memory_order_relaxed);
    fprintf(stderr,
            "PLANNER: Candidate loop took %.2f ms wall time (%u candidates)\n",
            NanosToMs(ElapsedNanos(start_candidate_loop, end_candidate_loop)),
            num_processed);
    if (num_processed != 0) {
      fprintf(stderr,
              "PLANNER: Candidate stages (sum/avg ms): rough_opt=%.2f/%.2f "
              "cluster=%.2f/%.2f rough_eval=%.2f/%.2f refine=%.2f/%.2f "
              "refined_eval=%.2f/%.2f\n",
              NanosToMs(rough_opt_ns.load(std::memory_order_relaxed)),
              NanosToMs(rough_opt_ns.load(std::memory_order_relaxed)) /
                  num_processed,
              NanosToMs(cluster_ns.load(std::memory_order_relaxed)),
              NanosToMs(cluster_ns.load(std::memory_order_relaxed)) /
                  num_processed,
              NanosToMs(rough_eval_ns.load(std::memory_order_relaxed)),
              NanosToMs(rough_eval_ns.load(std::memory_order_relaxed)) /
                  num_processed,
              NanosToMs(refine_ns.load(std::memory_order_relaxed)),
              NanosToMs(refine_ns.load(std::memory_order_relaxed)) /
                  num_processed,
              NanosToMs(refined_eval_ns.load(std::memory_order_relaxed)),
              NanosToMs(refined_eval_ns.load(std::memory_order_relaxed)) /
                  num_processed);
    }
    fflush(stderr);
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
  const uint32_t target_clusters =
      kMaxClusters - static_cast<uint32_t>(d.channels == 1);
  const uint32_t target_num_passes = 4;
  BiclusterSearchResult best_result;
  best_result.total_cost = std::numeric_limits<FixedPointCost>::max();
  std::mutex mu;

  auto start_assign_passes = PlannerClock::now();
  PassAssignment pass_assignment =
      AssignPassesGreedy(d, active, target_num_passes);
  auto end_assign_passes = PlannerClock::now();
  fprintf(stderr, "PLANNER: [bicluster] AssignPassesGreedy took %.2f ms\n",
          NanosToMs(ElapsedNanos(start_assign_passes, end_assign_passes)));
  fflush(stderr);

  auto start_build_stream = PlannerClock::now();
  std::vector<uint32_t> pass_offsets;
  JXL_ASSIGN_OR_RETURN(std::vector<ACEntry> pass_stream,
                       BuildPassStream(d, active, pass_assignment,
                                       target_num_passes, &pass_offsets));
  auto end_build_stream = PlannerClock::now();
  fprintf(stderr, "PLANNER: [bicluster] BuildPassStream took %.2f ms\n",
          NanosToMs(ElapsedNanos(start_build_stream, end_build_stream)));
  fflush(stderr);

  std::atomic<int64_t> rough_opt_ns(0);
  std::atomic<int64_t> cluster_ns(0);
  std::atomic<int64_t> rough_build_rows_ns(0);
  std::atomic<int64_t> rough_eval_ns(0);
  std::atomic<int64_t> refine_ns(0);
  std::atomic<int64_t> refined_build_rows_ns(0);
  std::atomic<int64_t> refined_eval_ns(0);
  std::atomic<uint32_t> processed_candidates(0);

  std::vector<PartitioningCtx> ctx_pool;
  auto start_candidate_loop = PlannerClock::now();
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

        auto start_rough_opt = PlannerClock::now();
        FixedPointCost rough_unclustered_cost = 0;
        ThresholdSet rough_thresholds =
            ctx.OptimizeThresholds(candidate.init, pass_stream,
                                   effort.main_m_target, effort.main_iters,
                                   &rough_unclustered_cost);
        auto end_rough_opt = PlannerClock::now();
        rough_opt_ns.fetch_add(ElapsedNanos(start_rough_opt, end_rough_opt),
                               std::memory_order_relaxed);

        auto start_cluster = PlannerClock::now();
        JXL_ASSIGN_OR_RETURN(
            ClusterResult cluster_result,
            ClusterContextsPassAware(d, rough_thresholds, pass_stream,
                                     pass_offsets, target_num_passes,
                                     std::min(target_clusters,
                                              effort.bicluster_row_budget)));
        auto end_cluster = PlannerClock::now();
        cluster_ns.fetch_add(ElapsedNanos(start_cluster, end_cluster),
                             std::memory_order_relaxed);

        std::vector<uint32_t> rough_num_prototypes;
        auto evaluate_thresholds =
            [&](const ThresholdSet& thresholds, std::atomic<int64_t>* build_ns,
                std::atomic<int64_t>* eval_ns,
                std::vector<uint32_t>* num_prototypes)
            -> StatusOr<ModelEvaluation> {
          auto start_build_rows = PlannerClock::now();
          JXL_ASSIGN_OR_RETURN(
              RowSliceHistograms rows,
              BuildRowSliceHistograms(d, thresholds, pass_assignment,
                                      target_num_passes));
          auto end_build_rows = PlannerClock::now();
          build_ns->fetch_add(ElapsedNanos(start_build_rows, end_build_rows),
                              std::memory_order_relaxed);

          auto start_eval = PlannerClock::now();
          JXL_ASSIGN_OR_RETURN(ModelEvaluation eval, EvaluateBiclusterState(
              d, thresholds, cluster_result.ctx_map,
              cluster_result.num_clusters, pass_assignment,
              target_num_passes, effort.bicluster_proto_budget_per_pass, rows,
              num_prototypes));
          auto end_eval = PlannerClock::now();
          eval_ns->fetch_add(ElapsedNanos(start_eval, end_eval),
                             std::memory_order_relaxed);
          return eval;
        };
        JXL_ASSIGN_OR_RETURN(ModelEvaluation rough_eval,
                             evaluate_thresholds(rough_thresholds,
                                                 &rough_build_rows_ns,
                                                 &rough_eval_ns,
                                                 &rough_num_prototypes));

        auto start_refine = PlannerClock::now();
        ThresholdSet refined_thresholds =
            RefinePassAwareThresholds(ctx, rough_thresholds, pass_stream, effort);
        auto end_refine = PlannerClock::now();
        refine_ns.fetch_add(ElapsedNanos(start_refine, end_refine),
                            std::memory_order_relaxed);
        std::vector<uint32_t> refined_num_prototypes;
        JXL_ASSIGN_OR_RETURN(ModelEvaluation refined_eval,
                             evaluate_thresholds(refined_thresholds,
                                                 &refined_build_rows_ns,
                                                 &refined_eval_ns,
                                                 &refined_num_prototypes));
        processed_candidates.fetch_add(1, std::memory_order_relaxed);

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
          best_result.ac_cost =
              best_eval.corrected_entropy_cost >= 0
                  ? best_eval.corrected_entropy_cost
                  : best_eval.ac_cost;
          best_result.nz_cost =
              best_eval.corrected_entropy_cost >= 0 ? 0 : best_eval.nz_cost;
          best_result.signalling_overhead = best_eval.signalling_overhead;
          best_result.total_cost = best_eval.total_cost();
        }
        return true;
      },
      "JpegCtxBicluster"));
  auto end_candidate_loop = PlannerClock::now();
  const uint32_t num_processed =
      processed_candidates.load(std::memory_order_relaxed);
  fprintf(stderr,
          "PLANNER: [bicluster] Candidate loop took %.2f ms wall time (%u candidates)\n",
          NanosToMs(ElapsedNanos(start_candidate_loop, end_candidate_loop)),
          num_processed);
  if (num_processed != 0) {
    fprintf(stderr,
            "PLANNER: [bicluster] Candidate stages (sum/avg ms): "
            "rough_opt=%.2f/%.2f cluster=%.2f/%.2f "
            "rough_build_rows=%.2f/%.2f rough_eval=%.2f/%.2f "
            "refine=%.2f/%.2f refined_build_rows=%.2f/%.2f "
            "refined_eval=%.2f/%.2f\n",
            NanosToMs(rough_opt_ns.load(std::memory_order_relaxed)),
            NanosToMs(rough_opt_ns.load(std::memory_order_relaxed)) /
                num_processed,
            NanosToMs(cluster_ns.load(std::memory_order_relaxed)),
            NanosToMs(cluster_ns.load(std::memory_order_relaxed)) /
                num_processed,
            NanosToMs(rough_build_rows_ns.load(std::memory_order_relaxed)),
            NanosToMs(rough_build_rows_ns.load(std::memory_order_relaxed)) /
                num_processed,
            NanosToMs(rough_eval_ns.load(std::memory_order_relaxed)),
            NanosToMs(rough_eval_ns.load(std::memory_order_relaxed)) /
                num_processed,
            NanosToMs(refine_ns.load(std::memory_order_relaxed)),
            NanosToMs(refine_ns.load(std::memory_order_relaxed)) /
                num_processed,
            NanosToMs(refined_build_rows_ns.load(std::memory_order_relaxed)),
            NanosToMs(refined_build_rows_ns.load(std::memory_order_relaxed)) /
                num_processed,
            NanosToMs(refined_eval_ns.load(std::memory_order_relaxed)),
            NanosToMs(refined_eval_ns.load(std::memory_order_relaxed)) /
                num_processed);
  }
  fflush(stderr);

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
