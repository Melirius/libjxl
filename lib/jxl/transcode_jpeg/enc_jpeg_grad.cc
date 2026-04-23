// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Lane B iteration 2: soft forward and analytic backward pass for AC cost.
//
// Iteration 1 established the forward pass (see plans/lane_b_progress.md).
// Iteration 2 adds the analytic gradient wrt `pass_logits` and `thresholds`.
// NZ cost, signalling overhead, and soft row->prototype assignment remain
// deferred.

#include "lib/jxl/transcode_jpeg/enc_jpeg_grad.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "lib/jxl/ac_context.h"
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

// Forward + optional backward. Shared body for both public entry points.
SoftCostResult ComputeSoftACCostImpl(const JPEGOptData& d,
                                     const GradientJointState& state,
                                     const ContextMap& ctx_map,
                                     uint32_t num_clusters, uint32_t num_passes,
                                     GradientJointGrad* grad) {
  SoftCostResult result;
  if (num_clusters == 0 || num_passes == 0) return result;

  const std::array<uint32_t, 3> n_axis = {
      static_cast<uint32_t>(state.thresholds[0].size()) + 1,
      static_cast<uint32_t>(state.thresholds[1].size()) + 1,
      static_cast<uint32_t>(state.thresholds[2].size()) + 1};
  const uint32_t num_cells = n_axis[0] * n_axis[1] * n_axis[2];
  if (ctx_map.size() != static_cast<size_t>(d.channels) * num_cells) {
    return result;  // Caller precondition failure; return zero cost.
  }

  const uint32_t cp_count = num_clusters * num_passes;
  const uint32_t ac_alpha = d.ACHistogramSize();
  constexpr uint32_t kZDC = kZeroDensityContextCount;
  const double inv_pass_t = 1.0 / state.pass_temperature;
  const double inv_thr_t = 1.0 / state.threshold_temperature;

  // Forward accumulators.
  std::vector<std::vector<double>> ac_h(cp_count,
                                        std::vector<double>(ac_alpha, 0.0));
  std::vector<std::vector<double>> ac_N(cp_count,
                                        std::vector<double>(kZDC, 0.0));

  // Per-block scratch.
  std::vector<uint32_t> h_scratch(ac_alpha, 0u);
  std::vector<uint32_t> N_scratch(kZDC, 0u);
  std::vector<uint32_t> h_touched;
  std::vector<uint32_t> N_touched;
  std::vector<double> w0, w1, w2;
  std::vector<double> cell_weight(num_cells, 0.0);
  std::vector<double> pi(num_passes, 0.0);

  // --- Forward pass ---------------------------------------------------------
  for (uint32_t c = 0; c < d.channels; ++c) {
    const uint32_t nb = d.num_blocks[c];
    for (uint32_t b = 0; b < nb; ++b) {
      Softmax(&state.pass_logits[c][static_cast<size_t>(b) * num_passes],
              num_passes, state.pass_temperature, pi.data());

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

  // --- Cost reduction -------------------------------------------------------
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

  for (uint32_t c = 0; c < d.channels; ++c) {
    const uint32_t nb = d.num_blocks[c];
    for (uint32_t b = 0; b < nb; ++b) {
      Softmax(&state.pass_logits[c][static_cast<size_t>(b) * num_passes],
              num_passes, state.pass_temperature, pi.data());

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

      // Accumulate dL/dpi[p] and dL/dcell[cell] from (cell, pass) events.
      for (uint32_t cell = 0; cell < num_cells; ++cell) {
        const double w_cell = cell_weight[cell];
        const uint32_t cluster = ctx_map[c * num_cells + cell];
        for (uint32_t p = 0; p < num_passes; ++p) {
          const uint32_t cp = cluster * num_passes + p;
          double delta = 0.0;
          const std::vector<double>& h_row = dL_dh[cp];
          const std::vector<double>& N_row = dL_dN[cp];
          for (const auto& entry : summary.h_entries) {
            delta += static_cast<double>(entry.second) * h_row[entry.first];
          }
          for (const auto& entry : summary.N_entries) {
            delta += static_cast<double>(entry.second) * N_row[entry.first];
          }
          dL_dpi[p] += w_cell * delta;
          dL_dcell[cell] += pi[p] * delta;
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

  return result;
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

void ResetGradientJointGrad(const GradientJointState& state,
                            GradientJointGrad* grad) {
  for (uint32_t a = 0; a < kNumCh; ++a) {
    grad->thresholds[a].assign(state.thresholds[a].size(), 0.0);
    grad->pass_logits[a].assign(state.pass_logits[a].size(), 0.0);
  }
}

SoftCostResult ComputeSoftACCost(const JPEGOptData& d,
                                 const GradientJointState& state,
                                 const ContextMap& ctx_map,
                                 uint32_t num_clusters, uint32_t num_passes) {
  return ComputeSoftACCostImpl(d, state, ctx_map, num_clusters, num_passes,
                               nullptr);
}

SoftCostResult ComputeSoftACCostWithGrad(const JPEGOptData& d,
                                         const GradientJointState& state,
                                         const ContextMap& ctx_map,
                                         uint32_t num_clusters,
                                         uint32_t num_passes,
                                         GradientJointGrad* grad) {
  return ComputeSoftACCostImpl(d, state, ctx_map, num_clusters, num_passes,
                               grad);
}

}  // namespace jxl
