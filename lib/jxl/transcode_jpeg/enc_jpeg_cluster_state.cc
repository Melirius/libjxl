// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Cluster-state helpers for JPEG lossless recompression.
//
// Once `Clustering::Build` has produced a clustering, the optimizer needs a
// few read / update operations on that state: estimate histogram signalling
// overhead, recompute the nonzero-count entropy term, expose threshold-local
// cluster boundaries for refinement, and prune thresholds that no longer
// separate distinct clusters. This file implements those state-side helpers;
// see `enc_jpeg_cluster.h` for the public `Clustering` interface and the
// overall clustering pipeline.
//
// `ComputeSignallingOverhead`
//   Estimates ANS histogram header cost for the current clustered state.
//
// `ComputeNZCost`
//   Computes the nonzero-count entropy portion of the clustered cost.
//
// `BuildLocalClusterBoundaries`
//   Builds threshold-major `{lo, hi}` boundary views for refinement.
//
// `PruneDeadThresholds`
//   Removes structurally inert thresholds and rebuilds `ctx_map`.

#include <cmath>

#include "lib/jxl/enc_ans_params.h"
#include "lib/jxl/enc_cluster.h"
#include "lib/jxl/enc_context_map.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_cluster.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"

namespace jxl {

namespace {

void AppendTokenHistograms(const JPEGOptData& d, const CompactHistogram& cluster,
                           std::vector<Histogram>* histograms) {
  if (cluster.empty()) return;

  const CompactACHistogramData& ac_hist = d.ACHistogram();
  std::vector<std::array<uint32_t, kACTokenCount>> zdc_counts(
      kZeroDensityContextCount);
  cluster.ForEachNonZero([&](uint32_t id, uint32_t freq) {
    JXL_DASSERT(ac_hist.dense_to_zdc[id] < kZeroDensityContextCount);
    JXL_DASSERT(ac_hist.dense_to_token[id] < kACTokenCount);
    zdc_counts[ac_hist.dense_to_zdc[id]][ac_hist.dense_to_token[id]] += freq;
  });

  for (uint32_t zdc = 0; zdc < kZeroDensityContextCount; ++zdc) {
    const auto& counts = zdc_counts[zdc];
    uint32_t max_token = 0;
    size_t total = 0;
    for (uint32_t t = 0; t < kACTokenCount; ++t) {
      if (counts[t] != 0) {
        max_token = t;
        total += counts[t];
      }
    }
    if (total == 0) continue;

    size_t alphabet_size = max_token + 1;
    histograms->emplace_back(alphabet_size);
    Histogram& h = histograms->back();
    for (uint32_t t = 0; t < alphabet_size; ++t) {
      h.counts[t] = static_cast<ANSHistBin>(counts[t]);
    }
    h.total_count = total;
  }
}

void AppendNZHistograms(const DenseNZHistogram& nz_cluster,
                        std::vector<Histogram>* histograms) {
  if (nz_cluster.empty()) return;

  for (uint32_t pb = 0; pb < kJPEGNonZeroBuckets; ++pb) {
    uint32_t max_nz = 0;
    size_t total = 0;
    const uint32_t base = pb * kJPEGNonZeroRange;
    for (uint32_t nz_count = 0; nz_count < kJPEGNonZeroRange; ++nz_count) {
      uint32_t freq = nz_cluster[base + nz_count];
      if (freq == 0) continue;
      max_nz = nz_count;
      total += freq;
    }
    if (total == 0) continue;

    size_t alphabet_size = max_nz + 1;
    histograms->emplace_back(alphabet_size);
    Histogram& h = histograms->back();
    for (uint32_t nz_count = 0; nz_count < alphabet_size; ++nz_count) {
      h.counts[nz_count] =
          static_cast<ANSHistBin>(nz_cluster[base + nz_count]);
    }
    h.total_count = total;
  }
}

void BuildFinalHistograms(const JPEGOptData& d, const Clustering& clustering,
                          std::vector<Histogram>* histograms) {
  histograms->clear();
  histograms->reserve(clustering.hist_h.size() *
                      (kZeroDensityContextCount + kJPEGNonZeroBuckets));
  for (size_t cluster_id = 0; cluster_id < clustering.hist_h.size();
       ++cluster_id) {
    AppendTokenHistograms(d, clustering.hist_h[cluster_id], histograms);
    AppendNZHistograms(clustering.hist_nz_h[cluster_id], histograms);
  }
}

Status ClusterFinalHistograms(const JPEGOptData& d, const Clustering& clustering,
                              std::vector<Histogram>* histograms,
                              std::vector<Histogram>* clustered,
                              std::vector<uint32_t>* histogram_symbols) {
  BuildFinalHistograms(d, clustering, histograms);
  clustered->clear();
  histogram_symbols->clear();
  if (histograms->empty()) return true;

  HistogramParams params;
  params.clustering = HistogramParams::ClusteringType::kBest;
  return ClusterHistograms(params, *histograms, kClustersLimit, clustered,
                           histogram_symbols);
}

}  // namespace

StatusOr<FixedPointCost> HistogramHeaderCost(const Histogram& h) {
  if (h.total_count == 0) return 0;
  JXL_ASSIGN_OR_RETURN(float ans_cost, h.ANSPopulationCost());
  float shannon = h.ShannonEntropy();
  float header_cost = ans_cost - shannon;
  return header_cost > 0 ? static_cast<FixedPointCost>(header_cost * kFScale)
                         : 0;
}

// Estimates ANS histogram header cost for one AC cluster after regrouping its
// symbols by signalling context (`zdc`) and token.
StatusOr<FixedPointCost>
Clustering::SignallingTokenHist::ClusterSignallingOverhead(
    const JPEGOptData& d, const CompactHistogram& cluster,
    FixedPointCost cutoff) {
  FixedPointCost overhead = 0;
  hist = {};
  const CompactACHistogramData& ac_hist = d.ACHistogram();
  cluster.ForEachNonZero([&](uint32_t id, uint32_t freq) {
    JXL_DASSERT(ac_hist.dense_to_zdc[id] < kZeroDensityContextCount);
    JXL_DASSERT(ac_hist.dense_to_token[id] < kACTokenCount);
    hist[ac_hist.dense_to_zdc[id]][ac_hist.dense_to_token[id]] += freq;
  });

  for (uint32_t zdc = 0; zdc < kZeroDensityContextCount; ++zdc) {
    const auto& th = hist[zdc];
    uint32_t max_token = 0;
    size_t total = 0;
    for (uint32_t t = 0; t < kACTokenCount; ++t) {
      if (th[t] != 0) {
        max_token = t;
        total += th[t];
      }
    }
    if (total == 0) continue;

    size_t alphabet_size = max_token + 1;
    Histogram h(alphabet_size);
    for (uint32_t t = 0; t < alphabet_size; ++t) {
      h.counts[t] = static_cast<ANSHistBin>(th[t]);
    }
    h.total_count = total;

    JXL_ASSIGN_OR_RETURN(float ans_cost, h.ANSPopulationCost());
    float shannon = h.ShannonEntropy();
    float header_cost = ans_cost - shannon;
    if (header_cost > 0) {
      overhead += static_cast<FixedPointCost>(header_cost * kFScale);
      if (overhead >= cutoff) return overhead;
    }
  }

  return overhead;
}

// Computes signalling overhead for one cluster by combining the AC-token
// signalling cost with the per-predicted-bucket nonzero-count histograms.
StatusOr<FixedPointCost> Clustering::ComputeClusterSignallingOverhead(
    const JPEGOptData& d, uint32_t cluster_id, FixedPointCost cutoff) const {
  FixedPointCost overhead = 0;
  JXL_DASSERT(cluster_id < hist_h.size());
  JXL_DASSERT(cluster_id < hist_nz_h.size());

  if (!signalling_token_hist_) {
    signalling_token_hist_ = jxl::make_unique<SignallingTokenHist>();
  }
  SignallingTokenHist& signalling_token_hist = *signalling_token_hist_;

  // Process `hist_h`: split by `zdc` and compute overhead per histogram.
  const auto& cluster = hist_h[cluster_id];
  if (!cluster.empty()) {
    JXL_ASSIGN_OR_RETURN(
        overhead,
        signalling_token_hist.ClusterSignallingOverhead(d, cluster, cutoff));
    if (overhead >= cutoff) return overhead;
  }

  // Process `hist_nz_h`: split by predicted bucket `pb`.
  const auto& nz_cluster = hist_nz_h[cluster_id];
  if (!nz_cluster.empty()) {
    for (uint32_t pb = 0; pb < kJPEGNonZeroBuckets; ++pb) {
      uint32_t max_nz = 0;
      size_t total = 0;
      const uint32_t base = pb * kJPEGNonZeroRange;
      for (uint32_t nz_count = 0; nz_count < kJPEGNonZeroRange; ++nz_count) {
        uint32_t freq = nz_cluster[base + nz_count];
        if (freq == 0) continue;
        max_nz = nz_count;
        total += freq;
      }
      if (total == 0) continue;

      size_t alphabet_size = max_nz + 1;
      Histogram h(alphabet_size);
      for (uint32_t nz_count = 0; nz_count <= max_nz; ++nz_count) {
        h.counts[nz_count] =
            static_cast<ANSHistBin>(nz_cluster[base + nz_count]);
      }
      h.total_count = total;

      JXL_ASSIGN_OR_RETURN(float ans_cost, h.ANSPopulationCost());
      float shannon = h.ShannonEntropy();
      float header_cost = ans_cost - shannon;
      if (header_cost > 0) {
        overhead += static_cast<FixedPointCost>(header_cost * kFScale);
        if (overhead >= cutoff) return overhead;
      }
    }
  }

  return overhead;
}

StatusOr<FixedPointCost> Clustering::ComputeSignallingOverhead(
    const JPEGOptData& d, FixedPointCost cutoff) const {
  FixedPointCost overhead = 0;
  std::vector<Histogram> histograms;
  std::vector<Histogram> clustered;
  std::vector<uint32_t> histogram_symbols;
  JXL_RETURN_IF_ERROR(
      ClusterFinalHistograms(d, *this, &histograms, &clustered,
                             &histogram_symbols));
  if (clustered.empty()) return overhead;

  for (const auto& h : clustered) {
    JXL_ASSIGN_OR_RETURN(FixedPointCost header_cost, HistogramHeaderCost(h));
    overhead += header_cost;
    if (overhead >= cutoff) return overhead;
  }

  // The encoder writes one context-map entry per original histogram context in
  // the combined AC + nz pool. We keep the existing lower-bound estimate.
  if (clustered.size() > 1) {
    double ctx_map_bits =
        static_cast<double>(histograms.size()) *
        std::log2(static_cast<double>(clustered.size()));
    overhead += static_cast<FixedPointCost>(ctx_map_bits * kFScale);
  }

  return overhead;
}

FixedPointCost Clustering::ComputeNZCost(const JPEGOptData& d) const {
  FixedPointCost nz_cost = 0;
  for (const auto& cl_N : hist_nz_N) {
    for (uint32_t freq : cl_N) {
      if (freq != 0) nz_cost += d.NZFTab(freq);
    }
  }
  for (const auto& cl_h : hist_nz_h) {
    for (uint32_t freq : cl_h) {
      if (freq != 0) nz_cost -= d.NZFTab(freq);
    }
  }
  return nz_cost;
}

StatusOr<FixedPointCost> Clustering::ComputeClusteredEntropyCost(
    const JPEGOptData& d) const {
  std::vector<Histogram> histograms;
  std::vector<Histogram> clustered;
  std::vector<uint32_t> histogram_symbols;
  JXL_RETURN_IF_ERROR(
      ClusterFinalHistograms(d, *this, &histograms, &clustered,
                             &histogram_symbols));

  FixedPointCost entropy = 0;
  for (const auto& h : clustered) {
    entropy += static_cast<FixedPointCost>(h.ShannonEntropy() * kFScale);
  }
  return entropy;
}

std::array<std::vector<ClusterBoundary>, kNumCh>
Clustering::BuildLocalClusterBoundaries(const ThresholdSet& thresholds,
                                        uint32_t channels) const {
  std::array<std::vector<ClusterBoundary>, kNumCh> local_cluster_boundary;
  const uint32_t size_Y = static_cast<uint32_t>(thresholds.TY().size() + 1);
  const uint32_t size_Cb = static_cast<uint32_t>(thresholds.TCb().size() + 1);
  const uint32_t size_Cr = static_cast<uint32_t>(thresholds.TCr().size() + 1);
  const uint32_t num_cells = size_Y * size_Cb * size_Cr;

  for (uint32_t axis = 0; axis < channels; ++axis) {
    const uint32_t ax1 = (axis + 1) % 3;
    const uint32_t ax2 = (axis + 2) % 3;
    const uint32_t na = static_cast<uint32_t>(thresholds.T[axis].size() + 1);
    const uint32_t n1 = static_cast<uint32_t>(thresholds.T[ax1].size() + 1);
    const uint32_t n2 = static_cast<uint32_t>(thresholds.T[ax2].size() + 1);
    const uint32_t num_rows = n1 * n2;

    local_cluster_boundary[axis].assign(
        static_cast<size_t>(channels) * (na - 1) * num_rows, {});

    for (uint32_t c = 0; c < channels; ++c) {
      for (uint32_t thr_ind = 0; thr_ind + 1 < na; ++thr_ind) {
        ClusterBoundary* dst = local_cluster_boundary[axis].data() +
                               (c * (na - 1) + thr_ind) * num_rows;
        for (uint32_t k1 = 0; k1 < n1; ++k1) {
          for (uint32_t k2 = 0; k2 < n2; ++k2) {
            uint32_t bkt[3] = {};
            bkt[axis] = thr_ind;
            bkt[ax1] = k1;
            bkt[ax2] = k2;
            const uint32_t lo_global_cell =
                (bkt[1] * size_Cr + bkt[2]) * size_Y + bkt[0];
            bkt[axis] = thr_ind + 1;
            const uint32_t hi_global_cell =
                (bkt[1] * size_Cr + bkt[2]) * size_Y + bkt[0];
            const uint32_t ci = k1 * n2 + k2;
            dst[ci] = {ctx_map[c * num_cells + lo_global_cell],
                       ctx_map[c * num_cells + hi_global_cell]};
          }
        }
      }
    }
  }
  return local_cluster_boundary;
}

ThresholdSet Clustering::PruneDeadThresholds(const ThresholdSet& thresholds) {
  ThresholdSet T = thresholds;
  // Grayscale: the grid is Y axis only, `ctx_map` is flat per channel.
  if (T.TCb().empty() && T.TCr().empty()) {
    Thresholds new_thr;
    new_thr.reserve(T.TY().size());
    std::vector<uint32_t> old_from_new = {0};
    for (uint32_t t = 0; t < T.TY().size(); ++t) {
      if (ctx_map[t] != ctx_map[t + 1]) {
        old_from_new.push_back(t + 1);
        new_thr.push_back(T.TY()[t]);
      }
    }
    T.TY().swap(new_thr);
    const uint32_t new_n = static_cast<uint32_t>(T.TY().size() + 1);
    ContextMap new_ctx_map(kNumCh * new_n, 0);
    for (uint32_t x = 0; x < new_n; ++x) {
      new_ctx_map[x] = ctx_map[old_from_new[x]];
    }
    ctx_map.swap(new_ctx_map);
    return T;
  }

  const uint32_t sizes[3] = {static_cast<uint32_t>(T.TY().size() + 1),
                             static_cast<uint32_t>(T.TCb().size() + 1),
                             static_cast<uint32_t>(T.TCr().size() + 1)};
  const uint32_t num_cells_init = sizes[0] * sizes[1] * sizes[2];
  // Stride when stepping from cell `t` to `t+1` along each axis.
  // Cell layout: `index = (b[1]*sizes[2] + b[2])*sizes[0] + b[0]`
  // where b[0]=Y, b[1]=Cb, b[2]=Cr (Y is innermost/stride-1, matching
  // the `dc_idx` formula in `compressed_dc.cc` and the `AxisMaps::Update`
  // bucket maps).
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
      for (uint32_t c = 0; c < kNumCh; ++c) {
        const uint32_t c_base = c * num_cells_init;
        for (uint32_t k1 = 0; k1 < sizes[ax1]; ++k1) {
          b[ax1] = k1;
          for (uint32_t k2 = 0; k2 < sizes[ax2]; ++k2) {
            b[ax2] = k2;
            const uint32_t gl = (b[1] * sizes[2] + b[2]) * sizes[0] + b[0];
            if (ctx_map[c_base + gl] !=
                ctx_map[c_base + gl + axis_stride[axis]]) {
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
  ContextMap new_ctx_map(kNumCh * new_num_cells, 0);
  for (uint32_t c = 0; c < kNumCh; ++c) {
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
          new_ctx_map[new_base + g_new] = ctx_map[old_base + g_old];
        }
      }
    }
  }
  ctx_map.swap(new_ctx_map);
  return T;
}

}  // namespace jxl
