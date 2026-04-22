// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.


#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_cluster.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

#include "lib/jxl/enc_ans_params.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_axis_maps.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_histogram.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_stream.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_stream.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_threshold.h"

namespace jxl {

using SparseHistogram = std::vector<std::unordered_map<uint32_t, uint32_t>>;

// Forward declaration of helper used by EvaluatePassAwareModel
FixedPointCost ComputePassOverhead(const JPEGOptData& d);

// --- Pass-aware clustering and scoring -----------------------------------------

// Small union-find helper for the agglomerative merge loop.
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
    const std::vector<ACEntry>& pass_stream,
    const std::vector<uint32_t>& pass_offsets,
    FixedPointCost cutoff) {
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

FixedPointCost ComputePassOverhead(const JPEGOptData& d) {
  // Legacy flat estimate kept for the older pass-aware scorer.
  uint32_t groups_x = (d.w_max + 31) / 32;
  uint32_t groups_y = (d.h_max + 31) / 32;
  uint32_t groups = groups_x * groups_y;
  return (groups * 64 + 64000) * kFScale;
}

}  // namespace jxl
