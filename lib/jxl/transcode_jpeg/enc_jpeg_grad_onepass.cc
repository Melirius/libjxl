// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "lib/jxl/transcode_jpeg/enc_jpeg_grad_internal.h"

#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "lib/jxl/transcode_jpeg/enc_jpeg_grad_onepass.cc"
// clang-format off
#include <hwy/foreach_target.h>
#include <hwy/highway.h>
#include <hwy/contrib/math/math-inl.h>
// clang-format on

#include "lib/jxl/ac_context.h"
#include "lib/jxl/base/status.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"

HWY_BEFORE_NAMESPACE();
namespace jxl {
namespace HWY_NAMESPACE {

namespace hn = hwy::HWY_NAMESPACE;

// Target-specific helpers defined in `enc_jpeg_grad.cc`.
double SoftFTabReduceVecFast(const double* HWY_RESTRICT data, size_t n);
void SoftFTabPrimeVecFast(const double* HWY_RESTRICT src,
                          double* HWY_RESTRICT dst, size_t n);
void SoftFTabPrimeNegVecFast(const double* HWY_RESTRICT src,
                             double* HWY_RESTRICT dst, size_t n);
void VecAddVec(double* HWY_RESTRICT dst, const double* HWY_RESTRICT src,
               size_t n);
void VecAdd2Vec(double* HWY_RESTRICT dst, const double* HWY_RESTRICT a,
                const double* HWY_RESTRICT b, size_t n);
void ClusterWeightsVec(const double* HWY_RESTRICT cell_weight,
                       const double* HWY_RESTRICT rho, double* HWY_RESTRICT B,
                       size_t num_cells, size_t num_clusters);
void CellGradientVec(const double* HWY_RESTRICT cell_weight,
                     const double* HWY_RESTRICT rho,
                     const double* HWY_RESTRICT D, double* HWY_RESTRICT dcell,
                     double* HWY_RESTRICT drho, size_t num_cells,
                     size_t num_clusters);
void SoftmaxJacobianRowsVec(const double* HWY_RESTRICT prob,
                            const double* HWY_RESTRICT dprob,
                            double* HWY_RESTRICT dst, double scale,
                            size_t num_rows, size_t row_size);
void ContextForwardVec(const double* HWY_RESTRICT ac_h,
                       const double* HWY_RESTRICT sigma,
                       const uint16_t* HWY_RESTRICT dense_to_zdc,
                       const uint16_t* HWY_RESTRICT dense_to_token,
                       double* HWY_RESTRICT ctx_h, double* HWY_RESTRICT ctx_N,
                       size_t num_clusters, size_t num_passes, size_t ac_alpha,
                       size_t zdc_count, size_t token_count, size_t num_hists);
void ContextBackwardVec(const double* HWY_RESTRICT ac_h,
                        const double* HWY_RESTRICT ac_N,
                        const double* HWY_RESTRICT sigma,
                        const double* HWY_RESTRICT dctx_h,
                        const double* HWY_RESTRICT dctx_N,
                        const uint16_t* HWY_RESTRICT dense_to_zdc,
                        const uint16_t* HWY_RESTRICT dense_to_token,
                        double* HWY_RESTRICT dL_dh, double* HWY_RESTRICT dL_dN,
                        double* HWY_RESTRICT dL_dsigma, size_t num_clusters,
                        size_t num_passes, size_t ac_alpha, size_t zdc_count,
                        size_t token_count, size_t num_hists);

namespace {

void Softmax(const double* HWY_RESTRICT logits, uint32_t P, double temperature,
             double* HWY_RESTRICT out) {
  const double inv_t = 1.0 / temperature;
  if (P < 16) {
    GradientSoftmaxScalar(logits, P, inv_t, out);
    return;
  }

  const hn::ScalableTag<double> d;
  const size_t N = hn::Lanes(d);
  const auto vinv_t = hn::Set(d, inv_t);
  auto vmax = hn::Set(d, logits[0]);

  uint32_t p = 0;
  for (; p + N <= P; p += N) {
    vmax = hn::Max(vmax, hn::LoadU(d, logits + p));
  }
  double max_val = hn::ReduceMax(d, vmax);
  for (; p < P; ++p) {
    if (logits[p] > max_val) max_val = logits[p];
  }

  const bool use_unshifted =
      std::abs(max_val * inv_t) < kSoftmaxUnshiftedMaxArg;
  const auto vmax_val = hn::Set(d, max_val);
  auto vsum = hn::Zero(d);
  double sum;
  if (use_unshifted) {
    for (p = 0; p + N <= P; p += N) {
      const auto prob = hn::Exp(d, hn::Mul(hn::LoadU(d, logits + p), vinv_t));
      hn::StoreU(prob, d, out + p);
      vsum = hn::Add(vsum, prob);
    }
    sum = hn::ReduceSum(d, vsum);
    for (; p < P; ++p) {
      out[p] = std::exp(logits[p] * inv_t);
      sum += out[p];
    }
  } else {
    for (p = 0; p + N <= P; p += N) {
      const auto prob = hn::Exp(
          d, hn::Mul(hn::Sub(hn::LoadU(d, logits + p), vmax_val), vinv_t));
      hn::StoreU(prob, d, out + p);
      vsum = hn::Add(vsum, prob);
    }
    sum = hn::ReduceSum(d, vsum);
    for (; p < P; ++p) {
      out[p] = std::exp((logits[p] - max_val) * inv_t);
      sum += out[p];
    }
  }

  const double inv_sum = 1.0 / sum;
  const auto vinv_sum = hn::Set(d, inv_sum);
  for (p = 0; p + N <= P; p += N) {
    hn::StoreU(hn::Mul(hn::LoadU(d, out + p), vinv_sum), d, out + p);
  }
  for (; p < P; ++p) out[p] *= inv_sum;
}

inline double FlatPassOverheadBits(const JPEGOptData& d) {
  const uint32_t groups_x = (d.w_max + 31) / 32;
  const uint32_t groups_y = (d.h_max + 31) / 32;
  const uint32_t groups = groups_x * groups_y;
  return static_cast<double>(groups * 64u + 64000u);
}

}  // namespace

SoftCostResult SoftForwardBackwardOnePassImpl(const JPEGOptData& d,
                                              const GradientAux& aux,
                                              const GradientState& state,
                                              GradientGrad* grad,
                                              GradientScratch* scratch) {
  SoftCostResult result;
  JXL_DASSERT(scratch != nullptr);
  GradientScratch& work = *scratch;
  const size_t num_clusters = state.num_clusters;
  JXL_DASSERT(state.num_passes == 1);
  JXL_DASSERT(num_clusters > 0);

  const std::array<uint32_t, kNumCh> n_axis = {
      static_cast<uint32_t>(state.thresholds[0].size()) + 1,
      static_cast<uint32_t>(state.thresholds[1].size()) + 1,
      static_cast<uint32_t>(state.thresholds[2].size()) + 1};
  const size_t num_cells = n_axis[0] * n_axis[1] * n_axis[2];
  JXL_DASSERT(state.num_cells == num_cells);
  for (uint32_t c = 0; c < d.channels; ++c) {
    JXL_DASSERT(state.cluster_logits[c].size() == num_cells * num_clusters);
  }

  const size_t cp_count = num_clusters;
  const CompactACHistogramData& ac_hist = d.ACHistogram();
  const uint32_t ac_alpha = d.ACHistogramSize();
  constexpr uint32_t kZDC = kZeroDensityContextCount;
  const double inv_thr_t = 1.0 / state.threshold_temperature;
  const double inv_cluster_t = 1.0 / state.cluster_temperature;
  const double inv_ctx_t = 1.0 / state.ctx_temperature;
  const uint32_t H = state.num_hists;
  JXL_DASSERT(state.ctx_logits.size() == num_clusters * kZDC * H);
  JXL_DASSERT(aux.ac_alpha == ac_alpha);
  JXL_DASSERT(ac_hist.dense_to_zdc.size() == ac_alpha);
  JXL_DASSERT(ac_hist.dense_to_token.size() == ac_alpha);
  for (uint32_t c = 0; c < d.channels; ++c) {
    JXL_DASSERT(aux.blocks[c].size() == d.num_blocks[c]);
  }

  std::vector<double>& ac_h = work.ac_h;
  std::vector<double>& ac_N = work.ac_N;
  std::fill(ac_h.begin(), ac_h.end(), 0.0);
  std::fill(ac_N.begin(), ac_N.end(), 0.0);
  // One-pass NZ histograms use layout that allows per-block updates and
  // backward replay add the full cluster vector sequentially:
  //   `nz_h[bin * K + k], nz_N[pb * K + k]`.
  std::vector<double>& nz_h = work.nz_h;
  std::vector<double>& nz_N = work.nz_N;
  std::fill(nz_h.begin(), nz_h.end(), 0.0);
  std::fill(nz_N.begin(), nz_N.end(), 0.0);

  auto& rho_cache = work.rho_cache;
  for (uint32_t c = 0; c < d.channels; ++c) {
    for (size_t cell = 0; cell < num_cells; ++cell) {
      Softmax(&state.cluster_logits[c][cell * num_clusters], num_clusters,
              state.cluster_temperature, &rho_cache[c][cell * num_clusters]);
    }
  }

  std::vector<double>& sigma = work.sigma;
  for (uint32_t k = 0; k < num_clusters; ++k) {
    for (uint32_t zdc = 0; zdc < kZDC; ++zdc) {
      const size_t off = (k * kZDC + zdc) * H;
      Softmax(&state.ctx_logits[off], H, state.ctx_temperature, &sigma[off]);
    }
  }

  std::vector<double>& w0 = work.w0;
  std::vector<double>& w1 = work.w1;
  std::vector<double>& w2 = work.w2;
  std::vector<double>& cell_weight = work.cell_weight;
  std::vector<double>& B = work.B;

  for (uint32_t c = 0; c < d.channels; ++c) {
    const uint32_t nb = d.num_blocks[c];
    for (uint32_t b = 0; b < nb; ++b) {
      const auto& block_aux = aux.blocks[c][b];
      AxisBucketWeights(state.thresholds[0], block_aux.dc_idx[0], inv_thr_t,
                        w0.data());
      AxisBucketWeights(state.thresholds[1], block_aux.dc_idx[1], inv_thr_t,
                        w1.data());
      AxisBucketWeights(state.thresholds[2], block_aux.dc_idx[2], inv_thr_t,
                        w2.data());

      for (uint32_t k1 = 0; k1 < n_axis[1]; ++k1) {
        const double v1 = w1[k1];
        for (uint32_t k2 = 0; k2 < n_axis[2]; ++k2) {
          const double v12 = v1 * w2[k2];
          for (uint32_t k0 = 0; k0 < n_axis[0]; ++k0) {
            cell_weight[(k1 * n_axis[2] + k2) * n_axis[0] + k0] = v12 * w0[k0];
          }
        }
      }

      ClusterWeightsVec(cell_weight.data(), rho_cache[c].data(), B.data(),
                        num_cells, num_clusters);

      const uint32_t ev_start = d.block_offsets[c][b];
      const uint32_t ev_end = d.block_offsets[c][b + 1];
      for (uint32_t e = ev_start; e < ev_end; ++e) {
        const CompactACEvent evt = d.FromBin(d.block_bins[c][e]);
        if (evt.hist_bin == kInvalidCompactH) continue;
        VecAddVec(&ac_h[evt.hist_bin * cp_count], B.data(), cp_count);
        VecAddVec(&ac_N[evt.zdc * cp_count], B.data(), cp_count);
      }

      const uint32_t pb = block_aux.onepass_nz_pb;
      const uint32_t bin_real = block_aux.onepass_nz_bin_real;
      VecAddVec(&nz_N[pb * cp_count], B.data(), cp_count);
      VecAddVec(&nz_h[bin_real * cp_count], B.data(), cp_count);
    }
  }

  std::vector<double>& ctx_h = work.ctx_h;
  std::vector<double>& ctx_N = work.ctx_N;
  std::fill(ctx_h.begin(), ctx_h.end(), 0.0);
  std::fill(ctx_N.begin(), ctx_N.end(), 0.0);
  ContextForwardVec(ac_h.data(), sigma.data(), ac_hist.dense_to_zdc.data(),
                    ac_hist.dense_to_token.data(), ctx_h.data(), ctx_N.data(),
                    num_clusters, 1, ac_alpha, kZDC, kACTokenCount, H);

  const double N_sum = SoftFTabReduceVecFast(ctx_N.data(), H);
  const double h_sum = SoftFTabReduceVecFast(ctx_h.data(), kACTokenCount * H);
  result.ac_cost_bits = N_sum - h_sum;

  double nz_cost = SoftFTabReduceVecFast(nz_N.data(), nz_N.size()) -
                   SoftFTabReduceVecFast(nz_h.data(), nz_h.size());
  double overhead_bits = 0.0;
  uint32_t touched_slots = 0;
  for (uint32_t h = 0; h < H; ++h) {
    overhead_bits += ACSignallingOverheadBits(
        ctx_h.data(), 0, h, H, &touched_slots, &work.overhead_hist);
  }
  result.num_cp_slots = touched_slots;
  for (size_t slot = 0; slot < cp_count; ++slot) {
    overhead_bits += NZSignallingOverheadBits(
        nz_h.data(), slot, cp_count, &work.overhead_hist);
  }
  overhead_bits += FlatPassOverheadBits(d);
  result.nz_cost_bits = nz_cost;
  result.signalling_overhead_bits = overhead_bits;
  result.total_cost_bits = result.ac_cost_bits + nz_cost + overhead_bits;

  if (grad == nullptr) return result;

  std::vector<double>& dL_dN = work.dL_dN;
  std::vector<double>& dL_dh = work.dL_dh;
  std::vector<double>& dL_dctx_N = work.dL_dctx_N;
  std::vector<double>& dL_dctx_h = work.dL_dctx_h;
  SoftFTabPrimeVecFast(ctx_N.data(), dL_dctx_N.data(), H);
  SoftFTabPrimeNegVecFast(ctx_h.data(), dL_dctx_h.data(), kACTokenCount * H);

  JXL_DASSERT(grad->ctx_logits.size() == state.ctx_logits.size());
  std::vector<double>& dL_dsigma = work.dL_dsigma;
  std::fill(dL_dsigma.begin(), dL_dsigma.end(), 0.0);
  ContextBackwardVec(ac_h.data(), ac_N.data(), sigma.data(), dL_dctx_h.data(),
                     dL_dctx_N.data(), ac_hist.dense_to_zdc.data(),
                     ac_hist.dense_to_token.data(), dL_dh.data(), dL_dN.data(),
                     dL_dsigma.data(), num_clusters, 1, ac_alpha, kZDC,
                     kACTokenCount, H);

  std::vector<double>& dL_dh_trans = work.dL_dh_trans;
  for (uint32_t di = 0; di < ac_alpha; ++di) {
    double* HWY_RESTRICT dst = &dL_dh_trans[di * cp_count];
    for (uint32_t k = 0; k < cp_count; ++k) {
      dst[k] = dL_dh[k * ac_alpha + di];
    }
  }
  std::vector<double>& dL_dN_trans = work.dL_dN_trans;
  for (uint32_t zdc = 0; zdc < kZDC; ++zdc) {
    double* HWY_RESTRICT dst = &dL_dN_trans[zdc * cp_count];
    for (uint32_t k = 0; k < cp_count; ++k) {
      dst[k] = dL_dN[k * kZDC + zdc];
    }
  }

  SoftmaxJacobianRowsVec(sigma.data(), dL_dsigma.data(),
                         grad->ctx_logits.data(), inv_ctx_t,
                         num_clusters * kZDC, H);

  auto& axis_sigma = work.axis_sigma;
  std::vector<double>& dL_dcell = work.dL_dcell;
  auto& dL_dw_ax = work.dL_dw_ax;

  auto& dL_drho = work.dL_drho;
  for (uint32_t c = 0; c < d.channels; ++c) {
    std::fill(dL_drho[c].begin(), dL_drho[c].end(), 0.0);
  }

  std::vector<double>& dL_dnz_N = work.dL_dnz_N;
  std::vector<double>& dL_dnz_h = work.dL_dnz_h;
  SoftFTabPrimeVecFast(nz_N.data(), dL_dnz_N.data(), nz_N.size());
  SoftFTabPrimeNegVecFast(nz_h.data(), dL_dnz_h.data(), nz_h.size());

  std::vector<double>& delta_ac_k = work.delta_ac_k;

  for (uint32_t c = 0; c < d.channels; ++c) {
    const uint32_t nb = d.num_blocks[c];
    for (uint32_t b = 0; b < nb; ++b) {
      const auto& block_aux = aux.blocks[c][b];
      AxisBucketWeights(state.thresholds[0], block_aux.dc_idx[0], inv_thr_t,
                        w0.data(), axis_sigma[0].data());
      AxisBucketWeights(state.thresholds[1], block_aux.dc_idx[1], inv_thr_t,
                        w1.data(), axis_sigma[1].data());
      AxisBucketWeights(state.thresholds[2], block_aux.dc_idx[2], inv_thr_t,
                        w2.data(), axis_sigma[2].data());

      for (uint32_t k1 = 0; k1 < n_axis[1]; ++k1) {
        const double v1 = w1[k1];
        for (uint32_t k2 = 0; k2 < n_axis[2]; ++k2) {
          const double v12 = v1 * w2[k2];
          for (uint32_t k0 = 0; k0 < n_axis[0]; ++k0) {
            cell_weight[(k1 * n_axis[2] + k2) * n_axis[0] + k0] = v12 * w0[k0];
          }
        }
      }

      std::fill(dL_dcell.begin(), dL_dcell.end(), 0.0);

      const uint32_t ev_start = d.block_offsets[c][b];
      const uint32_t ev_end = d.block_offsets[c][b + 1];
      ClusterWeightsVec(cell_weight.data(), rho_cache[c].data(), B.data(),
                        num_cells, num_clusters);

      std::fill(delta_ac_k.begin(), delta_ac_k.end(), 0.0);
      for (uint32_t e = ev_start; e < ev_end; ++e) {
        const CompactACEvent evt = d.FromBin(d.block_bins[c][e]);
        if (evt.hist_bin == kInvalidCompactH) continue;
        VecAdd2Vec(delta_ac_k.data(), &dL_dh_trans[evt.hist_bin * cp_count],
                   &dL_dN_trans[evt.zdc * cp_count], cp_count);
      }

      const uint32_t pb = block_aux.onepass_nz_pb;
      const uint32_t bin_real = block_aux.onepass_nz_bin_real;

      VecAdd2Vec(delta_ac_k.data(), &dL_dnz_h[bin_real * cp_count],
                 &dL_dnz_N[pb * cp_count], cp_count);

      CellGradientVec(cell_weight.data(), rho_cache[c].data(),
                      delta_ac_k.data(), dL_dcell.data(), dL_drho[c].data(),
                      num_cells, num_clusters);

      for (auto& a : dL_dw_ax) {
        std::fill(a.begin(), a.end(), 0.0);
      }
      for (uint32_t k1 = 0; k1 < n_axis[1]; ++k1) {
        for (uint32_t k2 = 0; k2 < n_axis[2]; ++k2) {
          for (uint32_t k0 = 0; k0 < n_axis[0]; ++k0) {
            const double g = dL_dcell[(k1 * n_axis[2] + k2) * n_axis[0] + k0];
            dL_dw_ax[0][k0] += g * w1[k1] * w2[k2];
            dL_dw_ax[1][k1] += g * w0[k0] * w2[k2];
            dL_dw_ax[2][k2] += g * w0[k0] * w1[k1];
          }
        }
      }

      for (uint32_t a = 0; a < kNumCh; ++a) {
        const size_t Tlen = state.thresholds[a].size();
        if (Tlen == 0) continue;
        for (size_t j = 0; j < Tlen; ++j) {
          const double sig_val = axis_sigma[a][j];
          const double sigma_prime = sig_val * (1.0 - sig_val) * inv_thr_t;
          grad->thresholds[a][j] +=
              sigma_prime * (dL_dw_ax[a][j] - dL_dw_ax[a][j + 1]);
        }
      }
    }
  }

  for (uint32_t c = 0; c < d.channels; ++c) {
    SoftmaxJacobianRowsVec(rho_cache[c].data(), dL_drho[c].data(),
                           grad->cluster_logits[c].data(), inv_cluster_t,
                           num_cells, num_clusters);
  }

  return result;
}

}  // namespace HWY_NAMESPACE
}  // namespace jxl
HWY_AFTER_NAMESPACE();

#if HWY_ONCE

namespace jxl {

HWY_EXPORT(SoftForwardBackwardOnePassImpl);

SoftCostResult SoftForwardBackwardOnePass(const JPEGOptData& d,
                                          const GradientAux& aux,
                                          const GradientState& state,
                                          GradientGrad* grad,
                                          GradientScratch* scratch) {
  return HWY_DYNAMIC_DISPATCH(SoftForwardBackwardOnePassImpl)(d, aux, state,
                                                              grad, scratch);
}

}  // namespace jxl

#endif  // HWY_ONCE
