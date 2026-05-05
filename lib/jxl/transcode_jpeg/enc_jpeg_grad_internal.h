// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#ifndef LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_GRAD_INTERNAL_H_
#define LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_GRAD_INTERNAL_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "lib/jxl/enc_ans_params.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_grad.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_histogram.h"

namespace jxl {

struct GradientBlockAux {
  std::array<uint16_t, kNumCh> dc_idx;
  uint16_t onepass_nz_pb = 0;
  uint16_t onepass_nz_bin_real = 0;
};

struct GradientAux {
  uint32_t ac_alpha = 0;
  std::array<std::vector<GradientBlockAux>, kNumCh> blocks;

  explicit GradientAux(const JPEGOptData& d) {
    static_assert(kJPEGNonZeroBuckets <= 0x10000u,
                  "one-pass NZ predictor bucket must fit in uint16_t");
    static_assert(kNZHistogramsSize <= 0x10000u,
                  "one-pass NZ histogram index must fit in uint16_t");

    ac_alpha = d.ACHistogramSize();

    for (uint32_t c = 0; c < d.channels; ++c) {
      const uint32_t nb = d.num_blocks[c];
      const uint32_t grid_w = d.block_grid_w[c];
      blocks[c].resize(nb);
      uint32_t x = 0;
      for (uint32_t b = 0; b < nb; ++b) {
        GradientBlockAux& block = blocks[c][b];
        block.dc_idx = AuxBlockDCIndices(d, c, b);

        const bool has_top = b >= grid_w;
        const bool has_left = x != 0;
        const uint32_t nz_top = has_top ? d.block_nonzeros[c][b - grid_w] : 0;
        const uint32_t nz_left = has_left ? d.block_nonzeros[c][b - 1] : 0;
        uint32_t predicted_nz_int;
        if (!has_top && !has_left) {
          predicted_nz_int = 32;
        } else if (!has_left) {
          predicted_nz_int = nz_top;
        } else if (!has_top) {
          predicted_nz_int = nz_left;
        } else {
          predicted_nz_int = (nz_top + nz_left + 1u) / 2u;
        }
        const uint32_t pb = AuxPredictorBucketFromNZInt(predicted_nz_int);
        block.onepass_nz_pb = static_cast<uint16_t>(pb);
        block.onepass_nz_bin_real =
            static_cast<uint16_t>(NZHistogramIndex(pb, d.block_nonzeros[c][b]));
        if (++x == grid_w) x = 0;
      }
    }
  }

 private:
  static uint32_t AuxPredictorBucketFromNZInt(uint32_t nz_int) {
    uint32_t pb = (nz_int < 8) ? nz_int : (4u + nz_int / 2u);
    return std::min<uint32_t>(pb, kJPEGNonZeroBuckets - 1);
  }

  static uint16_t AuxDCIndexForAxis(const JPEGOptData& d, uint32_t src_channel,
                                    uint32_t y, uint32_t x,
                                    uint32_t dst_channel) {
    const uint32_t b = MapTopLeftBlockIndex(d, src_channel, y, x, dst_channel);
    return d.block_DC_idx[dst_channel][b];
  }

  static std::array<uint16_t, kNumCh> AuxBlockDCIndices(const JPEGOptData& d,
                                                       uint32_t c,
                                                       uint32_t b) {
    if (d.channels == 1) {
      return {d.block_DC_idx[0][b], 0, 0};
    }
    const uint32_t y = b / d.block_grid_w[c];
    const uint32_t x = b % d.block_grid_w[c];
    return {AuxDCIndexForAxis(d, c, y, x, 0),
            AuxDCIndexForAxis(d, c, y, x, 1),
            AuxDCIndexForAxis(d, c, y, x, 2)};
  }
};

struct GradientScratch {
  std::array<std::vector<double>, kNumCh> pi_cache;
  std::array<std::vector<double>, kNumCh> rho_cache;
  std::array<std::vector<double>, kNumCh> axis_sigma;
  std::array<std::vector<double>, kNumCh> dL_dw_ax;
  std::array<std::vector<double>, kNumCh> dL_drho;

  std::vector<double> ac_h;
  std::vector<double> ac_N;
  std::vector<double> nz_h;
  std::vector<double> nz_N;
  std::vector<double> sigma;
  std::vector<double> ctx_h;
  std::vector<double> ctx_N;

  std::vector<double> dL_dh;
  std::vector<double> dL_dN;
  std::vector<double> dL_dh_trans;
  std::vector<double> dL_dN_trans;
  std::vector<double> dL_dctx_h;
  std::vector<double> dL_dctx_N;
  std::vector<double> dL_dsigma;
  std::vector<double> dL_dnz_h;
  std::vector<double> dL_dnz_N;

  std::vector<double> w0;
  std::vector<double> w1;
  std::vector<double> w2;
  std::vector<double> cell_weight;
  std::vector<double> B;
  std::vector<double> w_vec;
  std::vector<double> dL_dcell;
  std::vector<double> dL_dpi;
  std::vector<double> delta_ac_kp;
  std::vector<double> delta_ac_k;
  std::vector<double> nz_T_kp;
  std::vector<double> nz_diff_h_kp;
  std::vector<double> block_D;
  Histogram overhead_hist;

  GradientScratch(const JPEGOptData& d, const GradientAux& aux,
                  const GradientState& state) {
    constexpr uint32_t kZDC = kZeroDensityContextCount;
    constexpr uint32_t kNZBins = kNZHistogramsSize;
    constexpr uint32_t kNZBuckets = kJPEGNonZeroBuckets;

    const size_t num_passes = state.num_passes;
    const size_t num_clusters = state.num_clusters;
    const uint32_t H = state.num_hists;
    const std::array<uint32_t, kNumCh> n_axis = {
        static_cast<uint32_t>(state.thresholds[0].size()) + 1,
        static_cast<uint32_t>(state.thresholds[1].size()) + 1,
        static_cast<uint32_t>(state.thresholds[2].size()) + 1};
    const size_t num_cells = n_axis[0] * n_axis[1] * n_axis[2];
    const size_t cp_count = num_clusters * num_passes;
    const size_t ac_alpha = d.ACHistogramSize();

    for (uint32_t c = 0; c < kNumCh; ++c) {
      const size_t nb = (c < d.channels) ? d.num_blocks[c] : 0;
      pi_cache[c].resize(num_passes == 1 ? 0 : nb * num_passes);
      const size_t rho_size = (c < d.channels) ? num_cells * num_clusters : 0;
      rho_cache[c].resize(rho_size);
      dL_drho[c].resize(rho_size);
      axis_sigma[c].resize(state.thresholds[c].size());
      dL_dw_ax[c].resize(n_axis[c]);
    }

    ac_h.resize(ac_alpha * cp_count);
    ac_N.resize(kZDC * cp_count);
    nz_h.resize(kNZBins * cp_count);
    nz_N.resize(kNZBuckets * cp_count);
    sigma.resize(num_passes * num_clusters * kZDC * H);
    ctx_h.resize(num_passes * kACTokenCount * H);
    ctx_N.resize(num_passes * H);

    dL_dh.resize(cp_count * ac_alpha);
    dL_dN.resize(cp_count * kZDC);
    dL_dh_trans.resize(ac_alpha * cp_count);
    dL_dN_trans.resize(kZDC * cp_count);
    dL_dctx_h.resize(num_passes * kACTokenCount * H);
    dL_dctx_N.resize(num_passes * H);
    dL_dsigma.resize(state.ctx_logits.size());
    dL_dnz_h.resize(kNZBins * cp_count);
    dL_dnz_N.resize(kNZBuckets * cp_count);

    w0.resize(n_axis[0]);
    w1.resize(n_axis[1]);
    w2.resize(n_axis[2]);
    cell_weight.resize(num_cells);
    B.resize(num_clusters);
    w_vec.resize(cp_count);
    dL_dcell.resize(num_cells);
    dL_dpi.resize(num_passes);
    delta_ac_kp.resize(cp_count);
    delta_ac_k.resize(cp_count);
    nz_T_kp.resize(cp_count);
    nz_diff_h_kp.resize(cp_count);
    block_D.resize(num_clusters);
    overhead_hist.EnsureCapacity(kJPEGNonZeroRange);
  }
};

inline double ACSignallingOverheadBits(const double* ctx_h,
                                                   size_t p, size_t h,
                                                   size_t H,
                                                   uint32_t* touched_slots,
                                                   Histogram* hist) {
  std::fill(hist->counts.begin(), hist->counts.end(), ANSHistBin{0});
  hist->total_count = 0;
  size_t total = 0;
  const size_t p_off = p * kACTokenCount * H;
  for (uint32_t token = 0; token < kACTokenCount; ++token) {
    const double v = ctx_h[p_off + token * H + h];
    if (v <= 0.0) continue;
    const uint32_t count = static_cast<uint32_t>(std::llround(v));
    if (count == 0) continue;
    hist->counts[token] = static_cast<ANSHistBin>(count);
    total += count;
  }
  if (total == 0) return 0.0;
  ++*touched_slots;
  hist->total_count = total;
  auto ans_or = hist->ANSPopulationCost();
  if (!ans_or.ok()) return 0.0;
  const float ans_cost = std::move(ans_or).value_();
  const float shannon = hist->ShannonEntropy();
  const float header_cost = ans_cost - shannon;
  return header_cost > 0.0f ? static_cast<double>(header_cost) : 0.0;
}

inline double NZSignallingOverheadBits(const double* nz_h,
                                              size_t slot_idx, size_t cp_count,
                                              Histogram* hist) {
  double overhead_bits = 0.0;
  for (uint32_t pb = 0; pb < kJPEGNonZeroBuckets; ++pb) {
    std::fill(hist->counts.begin(), hist->counts.end(),
              static_cast<ANSHistBin>(0));
    hist->total_count = 0;
    size_t total = 0;
    for (uint32_t nz = 0; nz < kJPEGNonZeroRange; ++nz) {
      const double v =
          nz_h[static_cast<size_t>(NZHistogramIndex(pb, nz)) * cp_count +
               slot_idx];
      if (v <= 0.0) continue;
      const uint32_t count = static_cast<uint32_t>(std::llround(v));
      if (count == 0) continue;
      hist->counts[nz] = static_cast<ANSHistBin>(count);
      total += count;
    }
    if (total == 0) continue;
    hist->total_count = total;
    auto ans_or = hist->ANSPopulationCost();
    if (!ans_or.ok()) continue;
    const float ans_cost = std::move(ans_or).value_();
    const float shannon = hist->ShannonEntropy();
    const float header_cost = ans_cost - shannon;
    if (header_cost > 0.0f) overhead_bits += static_cast<double>(header_cost);
  }
  return overhead_bits;
}

// Sigmoid with numeric-stable evaluation and graceful behavior at extreme
// inputs. Returns exactly 0 or 1 when saturated to avoid denormal arithmetic
// in the hard-temperature limit used by correctness tests.
inline double SafeSigmoid(double x) {
  if (x > 500.0) return 1.0;
  if (x < -500.0) return 0.0;
  return 1.0 / (1.0 + std::exp(-x));
}

SoftCostResult ComputeSoftTotalCost(const JPEGOptData& d,
                                    const GradientAux& aux,
                                    const GradientState& state,
                                    GradientScratch* scratch);

SoftCostResult SoftForwardBackward(const JPEGOptData& d, const GradientAux& aux,
                                   const GradientState& state,
                                   GradientGrad* grad,
                                   GradientScratch* scratch);

SoftCostResult SoftForwardBackwardOnePass(const JPEGOptData& d,
                                          const GradientAux& aux,
                                          const GradientState& state,
                                          GradientGrad* grad,
                                          GradientScratch* scratch);

}  // namespace jxl

#endif  // LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_GRAD_INTERNAL_H_
