// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Lane B iteration 4: soft forward and analytic backward for AC + NZ cost,
// plus signalling overhead. Builds on iterations 1-3.
//
// Iteration 1 established the AC forward pass.
// Iteration 2 added the analytic gradient wrt `pass_logits` and `thresholds`.
// Iteration 3 added the Adam optimizer, annealing, and rounding.
// Iteration 4 (this file):
//   - NZ entropy cost with soft neighbor pi for `predicted_nz`; pb is
//     integer-rounded (no gradient through bucket selection) and the per-block
//     h-contribution splits between bin_real = NZIndex(pb, nz_b) and
//     bin_zero = NZIndex(pb, 0) with weights pi[p] and 1 - pi[p].
//   - Signalling overhead = per-slot ANSPopulationCost - ShannonEntropy for
//     AC and NZ histograms, plus a flat `ComputePassOverhead(d) * num_passes`.
//     Treated as constant for gradient purposes.

#include "lib/jxl/transcode_jpeg/enc_jpeg_grad.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

#include "lib/jxl/ac_context.h"
#include "lib/jxl/enc_ans_params.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_histogram.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_passes.h"

namespace jxl {

namespace {

// log2(e), the constant offset in d(n*log2(n))/dn = log2(n) + log2(e).
constexpr double kLog2E = 1.4426950408889634;

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

// Derivative `d(n * log2(n))/dn = log2(n) + log2(e)`. At `n <= 0` returns 0:
// the soft forward pass only produces zero `n` when all contributing soft
// weights are zero, in which case the chain rule collapses to zero anyway.
inline double SoftFTabPrime(double n) {
  constexpr double kEps = 1e-18;
  if (n <= kEps) return 0.0;
  return std::log2(n) + kLog2E;
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

// Per-axis state saved during the backward pass so the sigmoid derivative can
// be evaluated without recomputing the sigmoid.
struct AxisBucketBackward {
  // Saved per-threshold sigmoid values `sigma_{a,j} = sigmoid((T[a][j]-DC)/τ)`.
  // `sigma[j]` for `j in [0, |T[a]|)`.
  std::vector<double> sigma;
};

// Computes per-bucket weights for one axis and optionally records the sigmoid
// values needed for the backward pass.
//
// Uses a half-integer offset `(T - DC - 0.5)` inside the sigmoid. At the hard
// temperature limit this matches the strict `T[j] > DC` convention of
// `AxisMaps::Bkt`: a block whose DC equals a threshold lands in the bucket
// *after* the threshold, since `(T - T - 0.5)/tau -> -inf` drives the sigmoid
// to 0 rather than sitting at the ambiguous 0.5 midpoint.
void AxisBucketWeights(const std::vector<double>& thresholds, int dc,
                       double inv_temperature, std::vector<double>* out,
                       AxisBucketBackward* back = nullptr) {
  const uint32_t K = static_cast<uint32_t>(thresholds.size()) + 1;
  out->assign(K, 0.0);
  if (back != nullptr) back->sigma.assign(thresholds.size(), 0.0);
  if (K == 1) {
    (*out)[0] = 1.0;
    return;
  }
  double prev = 0.0;
  for (uint32_t k = 0; k + 1 < K; ++k) {
    const double cur = SafeSigmoid(
        (thresholds[k] - static_cast<double>(dc) - 0.5) * inv_temperature);
    (*out)[k] = cur - prev;
    if (back != nullptr) back->sigma[k] = cur;
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

// Per-block event histogram summary.
struct BlockEventSummary {
  std::vector<std::pair<uint32_t, uint32_t>> h_entries;  // (hist_bin, count)
  std::vector<std::pair<uint32_t, uint32_t>> N_entries;  // (zdc, count)
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

// Resolves the block's DC values on all three DC axes, honoring subsampling
// and the grayscale fast path.
void BlockDCValues(const JPEGOptData& d, uint32_t c, uint32_t b, int out[3]) {
  if (d.channels == 1) {
    out[0] = d.DC_vals[0][d.block_DC_idx[0][b]];
    out[1] = 0;
    out[2] = 0;
    return;
  }
  const uint32_t y = b / d.block_grid_w[c];
  const uint32_t x = b % d.block_grid_w[c];
  out[0] = DCValueForAxis(d, c, y, x, 0);
  out[1] = DCValueForAxis(d, c, y, x, 1);
  out[2] = DCValueForAxis(d, c, y, x, 2);
}

enum class CostMode { kACOnly, kTotal };

// Derives the integer predictor bucket `pb` from a fractional `predicted_nz`.
// Mirrors the hard formula in `EvaluatePassAwareModel` exactly once the
// fractional value is floored.
inline uint32_t PredictorBucketFromPredictedNZ(double predicted_nz) {
  int nz_int = static_cast<int>(std::floor(predicted_nz));
  if (nz_int < 0) nz_int = 0;
  uint32_t pb = (nz_int < 8) ? static_cast<uint32_t>(nz_int)
                             : (4u + static_cast<uint32_t>(nz_int) / 2u);
  if (pb >= kJPEGNonZeroBuckets) pb = kJPEGNonZeroBuckets - 1;
  return pb;
}

// Flat per-pass overhead constant; duplicated from `ComputePassOverhead` in
// enc_jpeg_pass_cluster.cc to avoid a circular include. Units: bits (not
// fixed-point). Matches the legacy scorer within a `kFScale` conversion.
inline double FlatPassOverheadBits(const JPEGOptData& d) {
  const uint32_t groups_x = (d.w_max + 31) / 32;
  const uint32_t groups_y = (d.h_max + 31) / 32;
  const uint32_t groups = groups_x * groups_y;
  return static_cast<double>(groups * 64u + 64000u);
}

// Computes the AC-histogram signalling overhead for one (cluster, pass) slot.
// Buckets symbols by zdc into sub-histograms over tokens, then returns
// `ANSPopulationCost - ShannonEntropy` per sub-histogram.
//
// Counts are rounded from soft `double` to integer via `std::llround`. The
// overhead term is not differentiated — this integer projection is enough.
double ACSignallingOverheadBitsForSlot(const JPEGOptData& d,
                                       const std::vector<double>& ac_h) {
  std::array<std::array<uint32_t, kACTokenCount>, kZeroDensityContextCount>
      signalling_hist = {};
  const auto& dense_to_symbol = d.ACHistogram().dense_to_zdcvalue;
  const size_t dense_size = dense_to_symbol.size();
  double overhead_bits = 0.0;
  for (size_t idx = 0; idx < ac_h.size() && idx < dense_size; ++idx) {
    const double v = ac_h[idx];
    if (v <= 0.0) continue;
    const uint32_t count =
        static_cast<uint32_t>(std::llround(std::max<double>(v, 0.0)));
    if (count == 0) continue;
    const SignallingHistSymbol sym =
        d.SignallingHistSymbolFromSymbol(dense_to_symbol[idx]);
    signalling_hist[sym.zdc][sym.token] += count;
  }
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
    auto ans_or = h.ANSPopulationCost();
    if (!ans_or.ok()) continue;
    const float ans_cost = std::move(ans_or).value_();
    const float shannon = h.ShannonEntropy();
    const float header_cost = ans_cost - shannon;
    if (header_cost > 0.0f) overhead_bits += static_cast<double>(header_cost);
  }
  return overhead_bits;
}

// Signalling overhead for one NZ-histogram slot: per predictor bucket, build a
// sub-histogram over `nz_count` and return `ANSPopulationCost - ShannonEntropy`.
double NZSignallingOverheadBitsForSlot(const std::vector<double>& nz_h) {
  double overhead_bits = 0.0;
  for (uint32_t pb = 0; pb < kJPEGNonZeroBuckets; ++pb) {
    uint32_t max_nz = 0;
    size_t total = 0;
    // First pass: find max symbol and total.
    for (uint32_t nz = 0; nz < kJPEGNonZeroRange; ++nz) {
      const double v = nz_h[NZHistogramIndex(pb, nz)];
      if (v <= 0.0) continue;
      const uint32_t count =
          static_cast<uint32_t>(std::llround(std::max<double>(v, 0.0)));
      if (count == 0) continue;
      if (nz > max_nz) max_nz = nz;
      total += count;
    }
    if (total == 0) continue;
    Histogram h(max_nz + 1);
    for (uint32_t nz = 0; nz <= max_nz; ++nz) {
      const double v = nz_h[NZHistogramIndex(pb, nz)];
      if (v <= 0.0) continue;
      h.counts[nz] = static_cast<ANSHistBin>(
          std::llround(std::max<double>(v, 0.0)));
    }
    h.total_count = total;
    auto ans_or = h.ANSPopulationCost();
    if (!ans_or.ok()) continue;
    const float ans_cost = std::move(ans_or).value_();
    const float shannon = h.ShannonEntropy();
    const float header_cost = ans_cost - shannon;
    if (header_cost > 0.0f) overhead_bits += static_cast<double>(header_cost);
  }
  return overhead_bits;
}

// Forward + optional backward. Shared body for all public entry points.
// `mode` selects AC-only (iteration 2) or AC+NZ+overhead (iteration 4).
// Iteration 5 moved `ctx_map` and `num_clusters` into `state` and added soft
// cluster membership via `state.cluster_logits`.
SoftCostResult ComputeSoftACCostImpl(const JPEGOptData& d,
                                     const GradientJointState& state,
                                     CostMode mode,
                                     GradientJointGrad* grad) {
  SoftCostResult result;
  const uint32_t num_passes = state.num_passes;
  const uint32_t num_clusters = state.num_clusters;
  if (num_clusters == 0 || num_passes == 0) return result;

  const std::array<uint32_t, 3> n_axis = {
      static_cast<uint32_t>(state.thresholds[0].size()) + 1,
      static_cast<uint32_t>(state.thresholds[1].size()) + 1,
      static_cast<uint32_t>(state.thresholds[2].size()) + 1};
  const uint32_t num_cells = n_axis[0] * n_axis[1] * n_axis[2];
  if (state.num_cells != num_cells) return result;  // state inconsistent.
  for (uint32_t c = 0; c < d.channels; ++c) {
    if (state.cluster_logits[c].size() !=
        static_cast<size_t>(num_cells) * num_clusters) {
      return result;  // cluster_logits not sized to match state.
    }
  }

  const uint32_t cp_count = num_clusters * num_passes;
  const uint32_t ac_alpha = d.ACHistogramSize();
  constexpr uint32_t kZDC = kZeroDensityContextCount;
  constexpr uint32_t kNZBins = kNZHistogramsSize;       // 36 * 64 = 2304
  constexpr uint32_t kNZBuckets = kJPEGNonZeroBuckets;  // 36
  const double inv_pass_t = 1.0 / state.pass_temperature;
  const double inv_thr_t = 1.0 / state.threshold_temperature;
  const double inv_cluster_t = 1.0 / state.cluster_temperature;
  const bool include_nz = (mode == CostMode::kTotal);

  // Forward accumulators.
  std::vector<std::vector<double>> ac_h(cp_count,
                                        std::vector<double>(ac_alpha, 0.0));
  std::vector<std::vector<double>> ac_N(cp_count,
                                        std::vector<double>(kZDC, 0.0));
  std::vector<std::vector<double>> nz_h;
  std::vector<std::vector<double>> nz_N;
  if (include_nz) {
    nz_h.assign(cp_count, std::vector<double>(kNZBins, 0.0));
    nz_N.assign(cp_count, std::vector<double>(kNZBuckets, 0.0));
  }

  // Precompute block-pi vectors when NZ is needed (neighbor lookups).
  std::array<std::vector<std::vector<double>>, kNumCh> pi_cache;
  if (include_nz) {
    for (uint32_t c = 0; c < d.channels; ++c) {
      const uint32_t nb = d.num_blocks[c];
      pi_cache[c].resize(nb);
      for (uint32_t b = 0; b < nb; ++b) {
        pi_cache[c][b].resize(num_passes);
        Softmax(&state.pass_logits[c][static_cast<size_t>(b) * num_passes],
                num_passes, state.pass_temperature,
                pi_cache[c][b].data());
      }
    }
  }

  // Precompute per-(channel, cell) cluster-softmax `rho`. Iteration 5 softens
  // the old `cluster = ctx_map[c*num_cells + cell]` lookup into a distribution
  // over clusters: `rho_{c,cell,k} = softmax(cluster_logits[c][cell*K..],
  // cluster_temperature)[k]`.
  std::array<std::vector<double>, kNumCh> rho_cache;
  for (uint32_t c = 0; c < d.channels; ++c) {
    rho_cache[c].assign(
        static_cast<size_t>(num_cells) * num_clusters, 0.0);
    for (uint32_t cell = 0; cell < num_cells; ++cell) {
      Softmax(&state.cluster_logits[c][static_cast<size_t>(cell) * num_clusters],
              num_clusters, state.cluster_temperature,
              &rho_cache[c][static_cast<size_t>(cell) * num_clusters]);
    }
  }

  // Per-block scratch.
  std::vector<uint32_t> h_scratch(ac_alpha, 0u);
  std::vector<uint32_t> N_scratch(kZDC, 0u);
  std::vector<uint32_t> h_touched;
  std::vector<uint32_t> N_touched;
  std::vector<double> w0, w1, w2;
  std::vector<double> cell_weight(num_cells, 0.0);
  std::vector<double> pi_local(num_passes, 0.0);

  // --- Forward pass ---------------------------------------------------------
  for (uint32_t c = 0; c < d.channels; ++c) {
    const uint32_t nb = d.num_blocks[c];
    const uint32_t grid_w = d.block_grid_w[c];
    for (uint32_t b = 0; b < nb; ++b) {
      // Soft pass weights (use cache when NZ needs neighbors too).
      const double* pi = nullptr;
      if (include_nz) {
        pi = pi_cache[c][b].data();
      } else {
        Softmax(&state.pass_logits[c][static_cast<size_t>(b) * num_passes],
                num_passes, state.pass_temperature, pi_local.data());
        pi = pi_local.data();
      }

      int dc[3];
      BlockDCValues(d, c, b, dc);
      AxisBucketWeights(state.thresholds[0], dc[0], inv_thr_t, &w0);
      AxisBucketWeights(state.thresholds[1], dc[1], inv_thr_t, &w1);
      AxisBucketWeights(state.thresholds[2], dc[2], inv_thr_t, &w2);

      for (uint32_t k1 = 0; k1 < n_axis[1]; ++k1) {
        const double v1 = w1[k1];
        for (uint32_t k2 = 0; k2 < n_axis[2]; ++k2) {
          const double v12 = v1 * w2[k2];
          for (uint32_t k0 = 0; k0 < n_axis[0]; ++k0) {
            cell_weight[(k1 * n_axis[2] + k2) * n_axis[0] + k0] =
                v12 * w0[k0];
          }
        }
      }

      const BlockEventSummary summary =
          SummarizeBlockEvents(d, c, b, &h_scratch, &N_scratch, &h_touched,
                               &N_touched);

      // AC accumulation (iteration 5: soft cluster via `rho`).
      for (uint32_t cell = 0; cell < num_cells; ++cell) {
        const double w_cell = cell_weight[cell];
        if (w_cell == 0.0) continue;
        const double* rho_cell =
            &rho_cache[c][static_cast<size_t>(cell) * num_clusters];
        for (uint32_t p = 0; p < num_passes; ++p) {
          const double w_cp_factor = w_cell * pi[p];
          if (w_cp_factor == 0.0) continue;
          for (uint32_t k = 0; k < num_clusters; ++k) {
            const double w = w_cp_factor * rho_cell[k];
            if (w == 0.0) continue;
            const uint32_t cp = k * num_passes + p;
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

      // NZ accumulation (forward).
      if (include_nz) {
        const uint32_t y = b / grid_w;
        const uint32_t x = b % grid_w;
        const uint32_t nz_b = d.block_nonzeros[c][b];
        const double* pi_top = nullptr;
        const double* pi_left = nullptr;
        uint32_t nz_top = 0;
        uint32_t nz_left = 0;
        if (y > 0) {
          const uint32_t b_top = (y - 1) * grid_w + x;
          pi_top = pi_cache[c][b_top].data();
          nz_top = d.block_nonzeros[c][b_top];
        }
        if (x > 0) {
          const uint32_t b_left = y * grid_w + (x - 1);
          pi_left = pi_cache[c][b_left].data();
          nz_left = d.block_nonzeros[c][b_left];
        }
        for (uint32_t cell = 0; cell < num_cells; ++cell) {
          const double w_cell = cell_weight[cell];
          if (w_cell == 0.0) continue;
          const double* rho_cell =
              &rho_cache[c][static_cast<size_t>(cell) * num_clusters];
          for (uint32_t p = 0; p < num_passes; ++p) {
            double pass_nz_top = (y > 0) ? pi_top[p] *
                                                static_cast<double>(nz_top)
                                         : 0.0;
            double pass_nz_left = (x > 0) ? pi_left[p] *
                                                 static_cast<double>(nz_left)
                                          : 0.0;
            double predicted_nz;
            if (x == 0 && y == 0) {
              predicted_nz = 32.0;
            } else if (x == 0) {
              predicted_nz = pass_nz_top;
            } else if (y == 0) {
              predicted_nz = pass_nz_left;
            } else {
              predicted_nz = (pass_nz_top + pass_nz_left + 1.0) / 2.0;
            }
            const uint32_t pb = PredictorBucketFromPredictedNZ(predicted_nz);
            const uint32_t bin_real = NZHistogramIndex(pb, nz_b);
            const uint32_t bin_zero = NZHistogramIndex(pb, 0);
            const double pi_p = pi[p];
            for (uint32_t k = 0; k < num_clusters; ++k) {
              const double weight = w_cell * rho_cell[k];
              if (weight == 0.0) continue;
              const uint32_t cp = k * num_passes + p;
              nz_N[cp][pb] += weight;
              nz_h[cp][bin_real] += weight * pi_p;
              nz_h[cp][bin_zero] += weight * (1.0 - pi_p);
            }
          }
        }
      }
    }
  }

  // --- AC cost reduction ----------------------------------------------------
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

  // --- NZ cost + signalling overhead ---------------------------------------
  double nz_cost = 0.0;
  double overhead_bits = 0.0;
  if (include_nz) {
    for (uint32_t cp = 0; cp < cp_count; ++cp) {
      for (uint32_t pb = 0; pb < kNZBuckets; ++pb) {
        const double n = nz_N[cp][pb];
        if (n > 0.0) nz_cost += SoftFTab(n);
      }
      for (uint32_t i = 0; i < kNZBins; ++i) {
        const double n = nz_h[cp][i];
        if (n > 0.0) nz_cost -= SoftFTab(n);
      }
      overhead_bits += ACSignallingOverheadBitsForSlot(d, ac_h[cp]);
      overhead_bits += NZSignallingOverheadBitsForSlot(nz_h[cp]);
    }
    overhead_bits += FlatPassOverheadBits(d) * static_cast<double>(num_passes);
    result.nz_cost_bits = nz_cost;
    result.signalling_overhead_bits = overhead_bits;
  }
  result.total_cost_bits = ac_cost + nz_cost + overhead_bits;

  if (grad == nullptr) return result;

  // --- Backward pass --------------------------------------------------------
  // Precompute upstream gradients wrt accumulators.
  //   dL/dN = +ftab'(N)
  //   dL/dh = -ftab'(h)
  std::vector<std::vector<double>> dL_dN(cp_count,
                                         std::vector<double>(kZDC, 0.0));
  std::vector<std::vector<double>> dL_dh(cp_count,
                                         std::vector<double>(ac_alpha, 0.0));
  for (uint32_t cp = 0; cp < cp_count; ++cp) {
    for (uint32_t z = 0; z < kZDC; ++z) {
      dL_dN[cp][z] = SoftFTabPrime(ac_N[cp][z]);
    }
    for (uint32_t i = 0; i < ac_alpha; ++i) {
      dL_dh[cp][i] = -SoftFTabPrime(ac_h[cp][i]);
    }
  }

  // Per-block backward storage.
  AxisBucketBackward back0, back1, back2;
  std::vector<double> dL_dcell(num_cells, 0.0);
  std::vector<double> dL_dpi(num_passes, 0.0);
  std::vector<std::vector<double>> dL_dw_ax(3);

  // Channel-wide accumulator for `dL/drho[c][cell * K + k]`. Accumulates
  // contributions from every block in channel `c`; converted to
  // `dL/dcluster_logits[c]` via the softmax Jacobian after the block loop.
  std::array<std::vector<double>, kNumCh> dL_drho;
  for (uint32_t c = 0; c < d.channels; ++c) {
    dL_drho[c].assign(static_cast<size_t>(num_cells) * num_clusters, 0.0);
  }

  // NZ upstream gradients (only needed when include_nz).
  std::vector<std::vector<double>> dL_dnz_N;
  std::vector<std::vector<double>> dL_dnz_h;
  if (include_nz) {
    dL_dnz_N.assign(cp_count, std::vector<double>(kNZBuckets, 0.0));
    dL_dnz_h.assign(cp_count, std::vector<double>(kNZBins, 0.0));
    for (uint32_t cp = 0; cp < cp_count; ++cp) {
      for (uint32_t pb = 0; pb < kNZBuckets; ++pb) {
        dL_dnz_N[cp][pb] = SoftFTabPrime(nz_N[cp][pb]);
      }
      for (uint32_t i = 0; i < kNZBins; ++i) {
        dL_dnz_h[cp][i] = -SoftFTabPrime(nz_h[cp][i]);
      }
    }
  }

  for (uint32_t c = 0; c < d.channels; ++c) {
    const uint32_t nb = d.num_blocks[c];
    const uint32_t grid_w = d.block_grid_w[c];
    for (uint32_t b = 0; b < nb; ++b) {
      const double* pi = nullptr;
      if (include_nz) {
        pi = pi_cache[c][b].data();
      } else {
        Softmax(&state.pass_logits[c][static_cast<size_t>(b) * num_passes],
                num_passes, state.pass_temperature, pi_local.data());
        pi = pi_local.data();
      }

      int dc[3];
      BlockDCValues(d, c, b, dc);
      AxisBucketWeights(state.thresholds[0], dc[0], inv_thr_t, &w0, &back0);
      AxisBucketWeights(state.thresholds[1], dc[1], inv_thr_t, &w1, &back1);
      AxisBucketWeights(state.thresholds[2], dc[2], inv_thr_t, &w2, &back2);

      for (uint32_t k1 = 0; k1 < n_axis[1]; ++k1) {
        const double v1 = w1[k1];
        for (uint32_t k2 = 0; k2 < n_axis[2]; ++k2) {
          const double v12 = v1 * w2[k2];
          for (uint32_t k0 = 0; k0 < n_axis[0]; ++k0) {
            cell_weight[(k1 * n_axis[2] + k2) * n_axis[0] + k0] =
                v12 * w0[k0];
          }
        }
      }

      const BlockEventSummary summary =
          SummarizeBlockEvents(d, c, b, &h_scratch, &N_scratch, &h_touched,
                               &N_touched);

      // Zero per-block accumulators.
      std::fill(dL_dcell.begin(), dL_dcell.end(), 0.0);
      std::fill(dL_dpi.begin(), dL_dpi.end(), 0.0);

      // AC contribution (iteration 5 soft cluster): sum over (cell, pass, k)
      // tuples. For this block,
      //   h contribution at (cp, bin) = gamma[cell] * pi[p] * rho[k] * count
      // so partials at (cell, p, k) split as
      //   dL/dgamma  += pi * rho * delta_ac
      //   dL/dpi     += gamma * rho * delta_ac
      //   dL/drho[k] += gamma * pi * delta_ac
      // where delta_ac = sum_events count * dL/dh[cp][bin] +
      //                  sum_events count * dL/dN[cp][zdc].
      for (uint32_t cell = 0; cell < num_cells; ++cell) {
        const double w_cell = cell_weight[cell];
        const double* rho_cell =
            &rho_cache[c][static_cast<size_t>(cell) * num_clusters];
        double* dL_drho_cell =
            &dL_drho[c][static_cast<size_t>(cell) * num_clusters];
        for (uint32_t p = 0; p < num_passes; ++p) {
          for (uint32_t k = 0; k < num_clusters; ++k) {
            const uint32_t cp = k * num_passes + p;
            double delta = 0.0;
            const std::vector<double>& h_row = dL_dh[cp];
            const std::vector<double>& N_row = dL_dN[cp];
            for (const auto& entry : summary.h_entries) {
              delta +=
                  static_cast<double>(entry.second) * h_row[entry.first];
            }
            for (const auto& entry : summary.N_entries) {
              delta +=
                  static_cast<double>(entry.second) * N_row[entry.first];
            }
            const double rho_k = rho_cell[k];
            dL_dpi[p] += w_cell * rho_k * delta;
            dL_dcell[cell] += pi[p] * rho_k * delta;
            dL_drho_cell[k] += w_cell * pi[p] * delta;
          }
        }
      }

      // NZ contribution: pb is held constant (no gradient flows through
      // predicted_nz); dL/dpi[p] picks up the h_real/h_zero split, dL/dcell
      // picks up both h weighted by pi / (1-pi) and the N term.
      if (include_nz) {
        const uint32_t y = b / grid_w;
        const uint32_t x = b % grid_w;
        const uint32_t nz_b = d.block_nonzeros[c][b];
        const double* pi_top = nullptr;
        const double* pi_left = nullptr;
        uint32_t nz_top = 0;
        uint32_t nz_left = 0;
        if (y > 0) {
          const uint32_t b_top = (y - 1) * grid_w + x;
          pi_top = pi_cache[c][b_top].data();
          nz_top = d.block_nonzeros[c][b_top];
        }
        if (x > 0) {
          const uint32_t b_left = y * grid_w + (x - 1);
          pi_left = pi_cache[c][b_left].data();
          nz_left = d.block_nonzeros[c][b_left];
        }
        for (uint32_t cell = 0; cell < num_cells; ++cell) {
          const double w_cell = cell_weight[cell];
          if (w_cell == 0.0) continue;
          const double* rho_cell =
              &rho_cache[c][static_cast<size_t>(cell) * num_clusters];
          double* dL_drho_cell =
              &dL_drho[c][static_cast<size_t>(cell) * num_clusters];
          for (uint32_t p = 0; p < num_passes; ++p) {
            double pass_nz_top = (y > 0) ? pi_top[p] *
                                                static_cast<double>(nz_top)
                                         : 0.0;
            double pass_nz_left = (x > 0) ? pi_left[p] *
                                                 static_cast<double>(nz_left)
                                          : 0.0;
            double predicted_nz;
            if (x == 0 && y == 0) {
              predicted_nz = 32.0;
            } else if (x == 0) {
              predicted_nz = pass_nz_top;
            } else if (y == 0) {
              predicted_nz = pass_nz_left;
            } else {
              predicted_nz = (pass_nz_top + pass_nz_left + 1.0) / 2.0;
            }
            const uint32_t pb = PredictorBucketFromPredictedNZ(predicted_nz);
            const uint32_t bin_real = NZHistogramIndex(pb, nz_b);
            const uint32_t bin_zero = NZHistogramIndex(pb, 0);
            const double pi_p = pi[p];
            for (uint32_t k = 0; k < num_clusters; ++k) {
              const uint32_t cp = k * num_passes + p;
              const double h_real_g = dL_dnz_h[cp][bin_real];
              const double h_zero_g = dL_dnz_h[cp][bin_zero];
              const double N_g = dL_dnz_N[cp][pb];
              // Common factor T = pi*h_real + (1-pi)*h_zero + N.
              const double T =
                  pi_p * h_real_g + (1.0 - pi_p) * h_zero_g + N_g;
              const double rho_k = rho_cell[k];
              dL_dpi[p] += w_cell * rho_k * (h_real_g - h_zero_g);
              dL_dcell[cell] += rho_k * T;
              dL_drho_cell[k] += w_cell * T;
            }
          }
        }
      }

      // Softmax Jacobian: dL/dlogit[q] = pi[q] * (dL/dpi[q] - s) / tau_pi,
      // where s = sum_p pi[p] * dL/dpi[p].
      double s = 0.0;
      for (uint32_t p = 0; p < num_passes; ++p) s += pi[p] * dL_dpi[p];
      double* grad_logits =
          &grad->pass_logits[c][static_cast<size_t>(b) * num_passes];
      for (uint32_t q = 0; q < num_passes; ++q) {
        grad_logits[q] += inv_pass_t * pi[q] * (dL_dpi[q] - s);
      }

      // Decompose dL/dcell into dL/dw_axis. Axis 0's weight at k0 pairs with
      // w1[k1]*w2[k2] in the product, so that's the factor we multiply by.
      dL_dw_ax[0].assign(n_axis[0], 0.0);
      dL_dw_ax[1].assign(n_axis[1], 0.0);
      dL_dw_ax[2].assign(n_axis[2], 0.0);
      for (uint32_t k1 = 0; k1 < n_axis[1]; ++k1) {
        for (uint32_t k2 = 0; k2 < n_axis[2]; ++k2) {
          for (uint32_t k0 = 0; k0 < n_axis[0]; ++k0) {
            const double g =
                dL_dcell[(k1 * n_axis[2] + k2) * n_axis[0] + k0];
            dL_dw_ax[0][k0] += g * w1[k1] * w2[k2];
            dL_dw_ax[1][k1] += g * w0[k0] * w2[k2];
            dL_dw_ax[2][k2] += g * w0[k0] * w1[k1];
          }
        }
      }

      // Threshold gradient per axis. Threshold T[a][j] affects buckets j and
      // j+1 in opposite directions.
      const AxisBucketBackward* axis_back[3] = {&back0, &back1, &back2};
      for (uint32_t a = 0; a < 3; ++a) {
        const uint32_t Tlen = static_cast<uint32_t>(state.thresholds[a].size());
        if (Tlen == 0) continue;
        for (uint32_t j = 0; j < Tlen; ++j) {
          const double sigma = axis_back[a]->sigma[j];
          const double sigma_prime = sigma * (1.0 - sigma) * inv_thr_t;
          grad->thresholds[a][j] +=
              sigma_prime *
              (dL_dw_ax[a][j] - dL_dw_ax[a][j + 1]);
        }
      }
    }
  }

  // Softmax Jacobian for cluster logits: per (c, cell), convert
  // `dL/drho[c][cell*K + k]` into `dL/dcluster_logits[c][cell*K + k]` via
  //   dL/dlogit[m] = rho[m] * (dL/drho[m] - sum_k rho[k] * dL/drho[k])
  //                  / cluster_temperature.
  for (uint32_t c = 0; c < d.channels; ++c) {
    for (uint32_t cell = 0; cell < num_cells; ++cell) {
      const size_t base = static_cast<size_t>(cell) * num_clusters;
      const double* rho_cell = &rho_cache[c][base];
      const double* dL_drho_cell = &dL_drho[c][base];
      double s = 0.0;
      for (uint32_t k = 0; k < num_clusters; ++k) {
        s += rho_cell[k] * dL_drho_cell[k];
      }
      double* grad_cluster = &grad->cluster_logits[c][base];
      for (uint32_t m = 0; m < num_clusters; ++m) {
        grad_cluster[m] +=
            inv_cluster_t * rho_cell[m] * (dL_drho_cell[m] - s);
      }
    }
  }

  return result;
}

}  // namespace

GradientJointState InitGradientJointStateFromHard(
    const JPEGOptData& d, const PassSearchResult& hard, double hard_logit,
    double threshold_temperature, double pass_temperature,
    double cluster_temperature) {
  GradientJointState state;
  state.num_passes = hard.num_passes;
  state.num_clusters = hard.num_clusters;
  state.threshold_temperature = threshold_temperature;
  state.pass_temperature = pass_temperature;
  state.cluster_temperature = cluster_temperature;

  for (uint32_t axis = 0; axis < kNumCh; ++axis) {
    const Thresholds& T = hard.thresholds.T[axis];
    state.thresholds[axis].assign(T.begin(), T.end());
  }
  state.num_cells = static_cast<uint32_t>(
      (state.thresholds[0].size() + 1) * (state.thresholds[1].size() + 1) *
      (state.thresholds[2].size() + 1));

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

  // Cluster logits: one softmax vector per (channel, cell). Sized only for
  // active channels. Assigned cluster from `hard.ctx_map` is set to
  // +hard_logit, others to -hard_logit.
  const uint32_t K = hard.num_clusters;
  for (uint32_t c = 0; c < kNumCh; ++c) {
    if (c >= d.channels) {
      state.cluster_logits[c].clear();
      continue;
    }
    state.cluster_logits[c].assign(
        static_cast<size_t>(state.num_cells) * K, -hard_logit);
    for (uint32_t cell = 0; cell < state.num_cells; ++cell) {
      const size_t idx = c * static_cast<size_t>(state.num_cells) + cell;
      if (idx >= hard.ctx_map.size()) continue;
      const uint8_t assigned = hard.ctx_map[idx];
      if (assigned < K) {
        state.cluster_logits[c][static_cast<size_t>(cell) * K + assigned] =
            hard_logit;
      }
    }
  }
  return state;
}

void ResetGradientJointGrad(const GradientJointState& state,
                            GradientJointGrad* grad) {
  for (uint32_t a = 0; a < kNumCh; ++a) {
    grad->thresholds[a].assign(state.thresholds[a].size(), 0.0);
    grad->pass_logits[a].assign(state.pass_logits[a].size(), 0.0);
    grad->cluster_logits[a].assign(state.cluster_logits[a].size(), 0.0);
  }
}

SoftCostResult ComputeSoftACCost(const JPEGOptData& d,
                                 const GradientJointState& state) {
  return ComputeSoftACCostImpl(d, state, CostMode::kACOnly, nullptr);
}

SoftCostResult ComputeSoftACCostWithGrad(const JPEGOptData& d,
                                         const GradientJointState& state,
                                         GradientJointGrad* grad) {
  return ComputeSoftACCostImpl(d, state, CostMode::kACOnly, grad);
}

SoftCostResult ComputeSoftTotalCost(const JPEGOptData& d,
                                    const GradientJointState& state) {
  return ComputeSoftACCostImpl(d, state, CostMode::kTotal, nullptr);
}

SoftCostResult ComputeSoftTotalCostWithGrad(const JPEGOptData& d,
                                            const GradientJointState& state,
                                            GradientJointGrad* grad) {
  return ComputeSoftACCostImpl(d, state, CostMode::kTotal, grad);
}

// --- Iteration 3: Adam optimizer, annealing schedule, optimize loop ---------

void InitAdamState(const GradientJointState& state, AdamState* adam) {
  for (uint32_t a = 0; a < kNumCh; ++a) {
    adam->m_thresholds[a].assign(state.thresholds[a].size(), 0.0);
    adam->v_thresholds[a].assign(state.thresholds[a].size(), 0.0);
    adam->m_logits[a].assign(state.pass_logits[a].size(), 0.0);
    adam->v_logits[a].assign(state.pass_logits[a].size(), 0.0);
    adam->m_cluster_logits[a].assign(state.cluster_logits[a].size(), 0.0);
    adam->v_cluster_logits[a].assign(state.cluster_logits[a].size(), 0.0);
  }
  adam->step = 0;
}

namespace {

inline void AdamApply(double grad, const AdamConfig& cfg, uint32_t step,
                      double* param, double* m, double* v) {
  const double b1 = cfg.beta1;
  const double b2 = cfg.beta2;
  *m = b1 * (*m) + (1.0 - b1) * grad;
  *v = b2 * (*v) + (1.0 - b2) * grad * grad;
  const double m_hat = *m / (1.0 - std::pow(b1, static_cast<double>(step)));
  const double v_hat = *v / (1.0 - std::pow(b2, static_cast<double>(step)));
  *param -= cfg.lr * m_hat / (std::sqrt(v_hat) + cfg.eps);
}

}  // namespace

void AdamStep(const GradientJointGrad& grad, const AdamConfig& cfg,
              AdamState* adam, GradientJointState* state) {
  ++adam->step;
  for (uint32_t a = 0; a < kNumCh; ++a) {
    for (size_t j = 0; j < state->thresholds[a].size(); ++j) {
      AdamApply(grad.thresholds[a][j], cfg, adam->step,
                &state->thresholds[a][j], &adam->m_thresholds[a][j],
                &adam->v_thresholds[a][j]);
    }
    for (size_t i = 0; i < state->pass_logits[a].size(); ++i) {
      AdamApply(grad.pass_logits[a][i], cfg, adam->step,
                &state->pass_logits[a][i], &adam->m_logits[a][i],
                &adam->v_logits[a][i]);
    }
    for (size_t i = 0; i < state->cluster_logits[a].size(); ++i) {
      AdamApply(grad.cluster_logits[a][i], cfg, adam->step,
                &state->cluster_logits[a][i], &adam->m_cluster_logits[a][i],
                &adam->v_cluster_logits[a][i]);
    }
  }
}

void ApplyAnnealing(const AnnealSchedule& schedule, uint32_t step_index,
                    GradientJointState* state) {
  if (step_index < schedule.hot_iters || schedule.anneal_iters == 0) {
    state->pass_temperature = schedule.pass_init;
    state->threshold_temperature = schedule.threshold_init;
    state->cluster_temperature = schedule.cluster_init;
    return;
  }
  const uint32_t anneal_step = step_index - schedule.hot_iters;
  if (anneal_step >= schedule.anneal_iters) {
    state->pass_temperature = schedule.pass_final;
    state->threshold_temperature = schedule.threshold_final;
    state->cluster_temperature = schedule.cluster_final;
    return;
  }
  const double t = static_cast<double>(anneal_step) /
                   static_cast<double>(schedule.anneal_iters - 1);
  // Geometric interpolation in log-space: final -> end, init -> start.
  const double log_pass =
      (1.0 - t) * std::log(schedule.pass_init) +
      t * std::log(schedule.pass_final);
  const double log_thr =
      (1.0 - t) * std::log(schedule.threshold_init) +
      t * std::log(schedule.threshold_final);
  const double log_clu =
      (1.0 - t) * std::log(schedule.cluster_init) +
      t * std::log(schedule.cluster_final);
  state->pass_temperature = std::exp(log_pass);
  state->threshold_temperature = std::exp(log_thr);
  state->cluster_temperature = std::exp(log_clu);
}

void ProjectThresholdsMonotonic(GradientJointState* state, double epsilon) {
  for (uint32_t a = 0; a < kNumCh; ++a) {
    auto& T = state->thresholds[a];
    for (size_t j = 1; j < T.size(); ++j) {
      const double lower_bound = T[j - 1] + epsilon;
      if (T[j] < lower_bound) T[j] = lower_bound;
    }
  }
}

OptimizeResult RunGradientJointSolve(const JPEGOptData& d,
                                     const AdamConfig& adam_cfg,
                                     const AnnealSchedule& schedule,
                                     GradientJointState* state) {
  OptimizeResult result;
  const uint32_t total_iters = schedule.hot_iters + schedule.anneal_iters;
  if (total_iters == 0) {
    const SoftCostResult r = ComputeSoftTotalCost(d, *state);
    result.init_cost_bits = r.total_cost_bits;
    result.final_cost_bits = r.total_cost_bits;
    return result;
  }

  AdamState adam;
  InitAdamState(*state, &adam);

  GradientJointGrad grad;
  ResetGradientJointGrad(*state, &grad);

  // Initial cost at caller-configured temperatures (schedule not applied yet).
  {
    const SoftCostResult r = ComputeSoftTotalCost(d, *state);
    result.init_cost_bits = r.total_cost_bits;
  }

  for (uint32_t t = 0; t < total_iters; ++t) {
    ApplyAnnealing(schedule, t, state);
    ResetGradientJointGrad(*state, &grad);
    const SoftCostResult r = ComputeSoftTotalCostWithGrad(d, *state, &grad);
    AdamStep(grad, adam_cfg, &adam, state);
    ProjectThresholdsMonotonic(state);
    result.final_cost_bits = r.total_cost_bits;
    ++result.iters_taken;
  }
  return result;
}

PassSearchResult RoundToHardAssignment(const JPEGOptData& d,
                                       const GradientJointState& state) {
  PassSearchResult r;
  const uint32_t num_passes = state.num_passes;
  const uint32_t num_clusters = state.num_clusters;
  const uint32_t num_cells = state.num_cells;
  r.num_passes = num_passes;
  r.num_clusters = num_clusters;

  // ctx_map = argmax over cluster logits per (channel, cell).
  r.ctx_map.assign(static_cast<size_t>(d.channels) * num_cells, 0);
  for (uint32_t c = 0; c < d.channels; ++c) {
    const auto& logits = state.cluster_logits[c];
    if (logits.size() !=
        static_cast<size_t>(num_cells) * num_clusters) {
      continue;
    }
    for (uint32_t cell = 0; cell < num_cells; ++cell) {
      const double* base =
          &logits[static_cast<size_t>(cell) * num_clusters];
      uint32_t best = 0;
      double best_v = base[0];
      for (uint32_t k = 1; k < num_clusters; ++k) {
        if (base[k] > best_v) {
          best_v = base[k];
          best = k;
        }
      }
      r.ctx_map[c * num_cells + cell] = static_cast<uint8_t>(best);
    }
  }

  // Round thresholds to int16_t and enforce strict monotonicity.
  constexpr int16_t kMinDC = -1024;
  constexpr int16_t kMaxDC = 1023;
  for (uint32_t a = 0; a < kNumCh; ++a) {
    const auto& soft = state.thresholds[a];
    Thresholds& out = r.thresholds.T[a];
    out.clear();
    out.reserve(soft.size());
    int16_t prev = kMinDC - 1;
    for (double v : soft) {
      long vi = std::lround(v);
      if (vi < kMinDC) vi = kMinDC;
      if (vi > kMaxDC) vi = kMaxDC;
      int16_t val = static_cast<int16_t>(vi);
      if (val <= prev) val = static_cast<int16_t>(prev + 1);
      if (val > kMaxDC) val = kMaxDC;
      out.push_back(val);
      prev = val;
    }
  }

  // Argmax pass assignment per block.
  for (uint32_t c = 0; c < kNumCh; ++c) {
    const uint32_t nb = (c < d.channels) ? d.num_blocks[c] : 0u;
    r.pass_assignment[c].assign(nb, 0);
    if (nb == 0 || num_passes == 0) continue;
    const auto& logits = state.pass_logits[c];
    for (uint32_t b = 0; b < nb; ++b) {
      const double* base = &logits[static_cast<size_t>(b) * num_passes];
      uint32_t best = 0;
      double best_v = base[0];
      for (uint32_t p = 1; p < num_passes; ++p) {
        if (base[p] > best_v) {
          best_v = base[p];
          best = p;
        }
      }
      r.pass_assignment[c][b] = static_cast<uint8_t>(best);
    }
  }
  return r;
}

}  // namespace jxl
