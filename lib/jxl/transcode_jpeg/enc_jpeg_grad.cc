// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Lane B iteration 1: soft forward pass for AC cost.
//
// This file implements only the forward pass for the AC entropy component. NZ
// cost, signalling overhead, and analytic gradients are deferred to later
// iterations (see plans/lane_b_progress.md).

#include "lib/jxl/transcode_jpeg/enc_jpeg_grad.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "lib/jxl/ac_context.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_passes.h"

namespace jxl {

namespace {

// Sigmoid with numeric-stable evaluation and graceful behavior at extreme
// inputs. Returns exactly 0 or 1 when saturated to avoid denormal arithmetic
// in the hard-temperature limit used by correctness tests.
inline double SafeSigmoid(double x) {
  if (x > 500.0) return 1.0;
  if (x < -500.0) return 0.0;
  return 1.0 / (1.0 + std::exp(-x));
}

// Continuous extension of `ftab(n) = n * log2(n)` to fractional `n >= 0`. At
// `n = 0` the limit is 0; the natural extension is used to stay differentiable
// and to match the precomputed integer `ftab` at integer arguments up to
// fixed-point rounding.
inline double SoftFTab(double n) {
  if (n <= 0.0) return 0.0;
  return n * std::log2(n);
}

// Computes softmax of `P` logits with the given temperature, writing `P`
// probabilities into `out`.
void Softmax(const double* logits, uint32_t P, double temperature,
             double* out) {
  double inv_t = 1.0 / temperature;
  double max_val = logits[0];
  for (uint32_t p = 1; p < P; ++p) {
    if (logits[p] > max_val) max_val = logits[p];
  }
  double sum = 0.0;
  for (uint32_t p = 0; p < P; ++p) {
    out[p] = std::exp((logits[p] - max_val) * inv_t);
    sum += out[p];
  }
  const double inv_sum = 1.0 / sum;
  for (uint32_t p = 0; p < P; ++p) out[p] *= inv_sum;
}

// Computes per-bucket weights for one axis given the block's DC value and the
// threshold set for that axis. Writes `thresholds.size() + 1` weights into
// `out`. At `temperature -> 0+`, this collapses to a one-hot indicator
// matching `AxisMaps::Bkt`.
void AxisBucketWeights(const std::vector<double>& thresholds, int dc,
                       double temperature, std::vector<double>* out) {
  const uint32_t K = static_cast<uint32_t>(thresholds.size()) + 1;
  out->assign(K, 0.0);
  if (K == 1) {
    (*out)[0] = 1.0;
    return;
  }
  const double inv_t = 1.0 / temperature;
  double prev = 0.0;
  for (uint32_t k = 0; k + 1 < K; ++k) {
    const double cur =
        SafeSigmoid((thresholds[k] - static_cast<double>(dc)) * inv_t);
    (*out)[k] = cur - prev;
    prev = cur;
  }
  (*out)[K - 1] = 1.0 - prev;
}

// Returns the DC value of the block owning position (y, x) in `src_channel`
// when projected onto `dst_channel`'s grid.
int DCValueForAxis(const JPEGOptData& d, uint32_t src_channel, uint32_t y,
                   uint32_t x, uint32_t dst_channel) {
  const uint32_t b = MapTopLeftBlockIndex(d, src_channel, y, x, dst_channel);
  return d.DC_vals[dst_channel][d.block_DC_idx[dst_channel][b]];
}

// Per-block event histogram summary: lists of (symbol, count) pairs keyed by
// the compact histogram bin and the zdc slot. We accumulate into small dense
// arrays and then emit only non-zero entries, which is cheap for small blocks
// and keeps the hot accumulation loop branch-free.
struct BlockEventSummary {
  // (hist_bin, count) for each distinct histogram bin touched by this block.
  std::vector<std::pair<uint32_t, uint32_t>> h_entries;
  // (zdc, count) for each distinct zdc bucket touched by this block.
  std::vector<std::pair<uint32_t, uint32_t>> N_entries;
};

BlockEventSummary SummarizeBlockEvents(const JPEGOptData& d, uint32_t c,
                                       uint32_t b,
                                       std::vector<uint32_t>* h_scratch,
                                       std::vector<uint32_t>* N_scratch,
                                       std::vector<uint32_t>* h_touched,
                                       std::vector<uint32_t>* N_touched) {
  const uint32_t start = d.block_offsets[c][b];
  const uint32_t end = d.block_offsets[c][b + 1];
  for (uint32_t e = start; e < end; ++e) {
    const CompactACEvent evt = d.FromBin(d.block_bins[c][e]);
    if (evt.hist_bin == kInvalidCompactH) continue;
    if ((*h_scratch)[evt.hist_bin] == 0) h_touched->push_back(evt.hist_bin);
    ++(*h_scratch)[evt.hist_bin];
    if ((*N_scratch)[evt.zdc] == 0) N_touched->push_back(evt.zdc);
    ++(*N_scratch)[evt.zdc];
  }
  BlockEventSummary s;
  s.h_entries.reserve(h_touched->size());
  for (uint32_t bin : *h_touched) {
    s.h_entries.emplace_back(bin, (*h_scratch)[bin]);
    (*h_scratch)[bin] = 0;
  }
  h_touched->clear();
  s.N_entries.reserve(N_touched->size());
  for (uint32_t zdc : *N_touched) {
    s.N_entries.emplace_back(zdc, (*N_scratch)[zdc]);
    (*N_scratch)[zdc] = 0;
  }
  N_touched->clear();
  return s;
}

}  // namespace

GradientJointState InitGradientJointStateFromHard(
    const JPEGOptData& d, const PassSearchResult& hard, double hard_logit,
    double threshold_temperature, double pass_temperature) {
  GradientJointState state;
  state.num_passes = hard.num_passes;
  state.threshold_temperature = threshold_temperature;
  state.pass_temperature = pass_temperature;

  for (uint32_t axis = 0; axis < kNumCh; ++axis) {
    const Thresholds& T = hard.thresholds.T[axis];
    state.thresholds[axis].assign(T.begin(), T.end());
  }

  const uint32_t P = hard.num_passes;
  for (uint32_t c = 0; c < kNumCh; ++c) {
    const uint32_t nb = d.num_blocks[c];
    state.pass_logits[c].assign(static_cast<size_t>(nb) * P, -hard_logit);
    if (c < d.channels) {
      for (uint32_t b = 0; b < nb; ++b) {
        const uint8_t assigned = hard.pass_assignment[c][b];
        state.pass_logits[c][static_cast<size_t>(b) * P + assigned] =
            hard_logit;
      }
    }
  }
  return state;
}

SoftCostResult ComputeSoftACCost(const JPEGOptData& d,
                                 const GradientJointState& state,
                                 const ContextMap& ctx_map,
                                 uint32_t num_clusters, uint32_t num_passes) {
  SoftCostResult result;
  if (num_clusters == 0 || num_passes == 0) return result;

  const uint32_t n0 =
      static_cast<uint32_t>(state.thresholds[0].size()) + 1;
  const uint32_t n1 =
      static_cast<uint32_t>(state.thresholds[1].size()) + 1;
  const uint32_t n2 =
      static_cast<uint32_t>(state.thresholds[2].size()) + 1;
  const uint32_t num_cells = n0 * n1 * n2;
  if (ctx_map.size() != static_cast<size_t>(d.channels) * num_cells) {
    return result;  // Caller precondition failure; return zero cost.
  }

  const uint32_t cp_count = num_clusters * num_passes;
  const uint32_t ac_alpha = d.ACHistogramSize();
  constexpr uint32_t kZDC = kZeroDensityContextCount;

  // Soft histograms: per (cluster, pass) slot, per histogram bin / zdc.
  std::vector<std::vector<double>> ac_h(cp_count,
                                        std::vector<double>(ac_alpha, 0.0));
  std::vector<std::vector<double>> ac_N(cp_count,
                                        std::vector<double>(kZDC, 0.0));

  // Per-block scratch for summarizing events.
  std::vector<uint32_t> h_scratch(ac_alpha, 0u);
  std::vector<uint32_t> N_scratch(kZDC, 0u);
  std::vector<uint32_t> h_touched;
  std::vector<uint32_t> N_touched;

  // Per-axis bucket weights (reused across blocks).
  std::vector<double> w0, w1, w2;
  std::vector<double> cell_weight(num_cells, 0.0);
  std::vector<double> pi(num_passes, 0.0);

  for (uint32_t c = 0; c < d.channels; ++c) {
    const uint32_t nb = d.num_blocks[c];
    const uint32_t grid_w = d.block_grid_w[c];
    for (uint32_t b = 0; b < nb; ++b) {
      // Soft pass weights.
      Softmax(&state.pass_logits[c][static_cast<size_t>(b) * num_passes],
              num_passes, state.pass_temperature, pi.data());

      // Soft cell weights. For grayscale, only axis 0 is meaningful.
      int dc0, dc1 = 0, dc2 = 0;
      if (d.channels == 1) {
        dc0 = d.DC_vals[0][d.block_DC_idx[0][b]];
      } else {
        const uint32_t y = b / grid_w;
        const uint32_t x = b % grid_w;
        dc0 = DCValueForAxis(d, c, y, x, 0);
        dc1 = DCValueForAxis(d, c, y, x, 1);
        dc2 = DCValueForAxis(d, c, y, x, 2);
      }
      AxisBucketWeights(state.thresholds[0], dc0, state.threshold_temperature,
                        &w0);
      AxisBucketWeights(state.thresholds[1], dc1, state.threshold_temperature,
                        &w1);
      AxisBucketWeights(state.thresholds[2], dc2, state.threshold_temperature,
                        &w2);

      // Cell layout matches `BlockCell`: cell = (k1*n2 + k2) * n0 + k0.
      for (uint32_t k1 = 0; k1 < n1; ++k1) {
        const double v1 = w1[k1];
        for (uint32_t k2 = 0; k2 < n2; ++k2) {
          const double v12 = v1 * w2[k2];
          for (uint32_t k0 = 0; k0 < n0; ++k0) {
            cell_weight[(k1 * n2 + k2) * n0 + k0] = v12 * w0[k0];
          }
        }
      }

      // Summarize block events once.
      const BlockEventSummary summary =
          SummarizeBlockEvents(d, c, b, &h_scratch, &N_scratch, &h_touched,
                               &N_touched);

      // Accumulate into (cluster, pass) histograms.
      for (uint32_t cell = 0; cell < num_cells; ++cell) {
        const double w_cell = cell_weight[cell];
        if (w_cell == 0.0) continue;
        const uint32_t cluster = ctx_map[c * num_cells + cell];
        for (uint32_t p = 0; p < num_passes; ++p) {
          const double w = w_cell * pi[p];
          if (w == 0.0) continue;
          const uint32_t cp = cluster * num_passes + p;
          std::vector<double>& h_row = ac_h[cp];
          std::vector<double>& N_row = ac_N[cp];
          for (const auto& entry : summary.h_entries) {
            h_row[entry.first] += w * static_cast<double>(entry.second);
          }
          for (const auto& entry : summary.N_entries) {
            N_row[entry.first] += w * static_cast<double>(entry.second);
          }
        }
      }
    }
  }

  // Sum entropy cost across all (cluster, pass) slots.
  double ac_cost = 0.0;
  uint32_t touched_slots = 0;
  for (uint32_t cp = 0; cp < cp_count; ++cp) {
    bool any = false;
    for (uint32_t z = 0; z < kZDC; ++z) {
      const double n = ac_N[cp][z];
      if (n > 0.0) {
        ac_cost += SoftFTab(n);
        any = true;
      }
    }
    for (uint32_t i = 0; i < ac_alpha; ++i) {
      const double n = ac_h[cp][i];
      if (n > 0.0) {
        ac_cost -= SoftFTab(n);
        any = true;
      }
    }
    if (any) ++touched_slots;
  }

  result.ac_cost_bits = ac_cost;
  result.num_cp_slots = touched_slots;
  return result;
}

}  // namespace jxl
