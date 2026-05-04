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
#include <cstdio>
#include <cstdint>
#include <utility>
#include <vector>

#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "lib/jxl/transcode_jpeg/enc_jpeg_grad.cc"
// clang-format off
#include <hwy/foreach_target.h>
#include <hwy/highway.h>
#include <hwy/contrib/math/math-inl.h>
// clang-format on

#include "lib/jxl/ac_context.h"
#include "lib/jxl/enc_ans_params.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_bicluster.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_histogram.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_assign.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_cluster.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_stream.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_utils.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_passes.h"

HWY_BEFORE_NAMESPACE();
namespace jxl {
namespace HWY_NAMESPACE {

namespace hn = hwy::HWY_NAMESPACE;

// log2(e), the constant offset in d(n*log2(n))/dn = log2(n) + log2(e).
constexpr double kLog2E = 1.4426950408889634;

// Experimental double-lane analogue of FastLog2f: same exponent/mantissa
// reduction and 2/2 approximation, but adapted to IEEE-754 double bits.
// Undefined for negative / NaN inputs. For zero it returns a finite value, so
// `0.0 * FastLog2d(0.0)` contributes zero.
template <class D, class V>
HWY_INLINE V FastLog2d(const D d, V x) {
  const hn::Rebind<int64_t, D> di;
  const auto x_bits = hn::BitCast(di, x);

  // 0x3FE5555555555555 is the double encoding of a value just below 2/3.
  // Subtracting it before the exponent shift keeps the reconstructed mantissa
  // close to 1.0, matching the FastLog2f range-reduction trick.
  const auto exp_bits =
      hn::Sub(x_bits, hn::Set(di, int64_t{0x3FE5555555555555LL}));
  const auto exp_shifted = hn::ShiftRight<52>(exp_bits);
  const auto mantissa =
      hn::BitCast(d, hn::Sub(x_bits, hn::ShiftLeft<52>(exp_shifted)));
  const auto xm1 = hn::Sub(mantissa, hn::Set(d, 1.0));

  const auto p =
      hn::MulAdd(hn::MulAdd(hn::Set(d, 7.4245873327820566E-01), xm1,
                            hn::Set(d, 1.4287160470083755E+00)),
                 xm1, hn::Set(d, -1.8503833400518310E-06));
  const auto q =
      hn::MulAdd(hn::MulAdd(hn::Set(d, 1.7409343003366853E-01), xm1,
                            hn::Set(d, 1.0096718572241148E+00)),
                 xm1, hn::Set(d, 9.9032814277590719E-01));
  return hn::Add(hn::Div(p, q), hn::ConvertTo(d, exp_shifted));
}

// Derivative of x * FastLog2d(x), ignoring the piecewise-constant exponent
// term's jump discontinuities. This keeps the analytic gradient consistent
// with SoftFTabReduceVecFast's approximate forward objective.
template <class D, class V>
HWY_INLINE V FastLog2dFTabPrime(const D d, V x) {
  const hn::Rebind<int64_t, D> di;
  const auto x_bits = hn::BitCast(di, x);
  const auto exp_bits =
      hn::Sub(x_bits, hn::Set(di, int64_t{0x3FE5555555555555LL}));
  const auto exp_shifted = hn::ShiftRight<52>(exp_bits);
  const auto mantissa =
      hn::BitCast(d, hn::Sub(x_bits, hn::ShiftLeft<52>(exp_shifted)));
  const auto xm1 = hn::Sub(mantissa, hn::Set(d, 1.0));

  const auto p =
      hn::MulAdd(hn::MulAdd(hn::Set(d, 7.4245873327820566E-01), xm1,
                            hn::Set(d, 1.4287160470083755E+00)),
                 xm1, hn::Set(d, -1.8503833400518310E-06));
  const auto q =
      hn::MulAdd(hn::MulAdd(hn::Set(d, 1.7409343003366853E-01), xm1,
                            hn::Set(d, 1.0096718572241148E+00)),
                 xm1, hn::Set(d, 9.9032814277590719E-01));
  const auto fast_log2 = hn::Add(hn::Div(p, q), hn::ConvertTo(d, exp_shifted));

  const auto p_prime =
      hn::MulAdd(hn::Set(d, 1.4849174665564113E+00), xm1,
                 hn::Set(d, 1.4287160470083755E+00));
  const auto q_prime =
      hn::MulAdd(hn::Set(d, 3.4818686006733706E-01), xm1,
                 hn::Set(d, 1.0096718572241148E+00));
  const auto ratio_prime =
      hn::Div(hn::Sub(hn::Mul(p_prime, q), hn::Mul(p, q_prime)),
              hn::Mul(q, q));
  return hn::Add(fast_log2, hn::Mul(mantissa, ratio_prime));
}

// Returns sum_{i: data[i]>0} data[i]*log2(data[i]).
// `ShannonEntropy()`, but for `double` inputs.
double SoftFTabReduceVec(const double* HWY_RESTRICT data, size_t n) {
  const hn::ScalableTag<double> d;
  const size_t N = hn::Lanes(d);
  const auto vzero = hn::Zero(d);
  const auto vone = hn::Set(d, 1.0);
  auto vsum = hn::Zero(d);
  size_t i = 0;
  for (; i + N <= n; i += N) {
    const auto v = hn::LoadU(d, data + i);
    const auto mask = hn::Gt(v, vzero);
    const auto safe_v = hn::IfThenElse(mask, v, vone);
    vsum = hn::MulAdd(v, hn::Log2(d, safe_v), vsum);
  }
  double sum = hn::ReduceSum(d, vsum);
  for (; i < n; ++i) {
    if (data[i] > 0.0) sum += data[i] * std::log2(data[i]);
  }
  return sum;
}

// Experimental unchecked variant for nonnegative histogram/count data.
// Unlike `SoftFTabReduceVec`, this relies on `FastLog2d(0.0)` being finite
// and does not mask negative lanes.
double SoftFTabReduceVecFast(const double* HWY_RESTRICT data, size_t n) {
  const hn::ScalableTag<double> d;
  const size_t N = hn::Lanes(d);
  auto vsum = hn::Zero(d);
  size_t i = 0;
  for (; i + N <= n; i += N) {
    const auto v = hn::LoadU(d, data + i);
    vsum = hn::MulAdd(v, FastLog2d(d, v), vsum);
  }
  double sum = hn::ReduceSum(d, vsum);
  const HWY_CAPPED(double, 1) d1;
  for (; i < n; ++i) {
    const auto v = hn::Set(d1, data[i]);
    sum += data[i] * hn::GetLane(FastLog2d(d1, v));
  }
  return sum;
}

// Writes scale*(log2(src[i])+log2e) into dst[i] for src[i]>1e-18, else 0.
void SoftFTabPrimeVec(const double* HWY_RESTRICT src, double* HWY_RESTRICT dst,
                      double scale, size_t n) {
  const hn::ScalableTag<double> d;
  const size_t N = hn::Lanes(d);
  const auto veps = hn::Set(d, 1e-18);
  const auto vscale = hn::Set(d, scale * kLog2E);
  size_t i = 0;
  for (; i + N <= n; i += N) {
    const auto v = hn::LoadU(d, src + i);
    const auto mask = hn::Gt(v, veps);
    const auto safe_v = hn::IfThenElse(mask, v, veps);
    hn::StoreU(hn::IfThenElseZero(
                   mask, hn::MulAdd(vscale, hn::Log(d, safe_v), vscale)),
               d, dst + i);
  }
  for (; i < n; ++i) {
    dst[i] = (src[i] > 1e-18) ? scale * kLog2E * (std::log(src[i]) + 1.0) : 0.0;
  }
}

// Experimental unchecked variant for nonnegative histogram/count data.
// Keeps the zeroing mask for the derivative, but differentiates the same
// FastLog2d approximation used by SoftFTabReduceVecFast.
void SoftFTabPrimeVecFast(const double* HWY_RESTRICT src,
                          double* HWY_RESTRICT dst, double scale, size_t n) {
  const hn::ScalableTag<double> d;
  const size_t N = hn::Lanes(d);
  const auto veps = hn::Set(d, 1e-18);
  const auto vscale = hn::Set(d, scale);
  size_t i = 0;
  for (; i + N <= n; i += N) {
    const auto v = hn::LoadU(d, src + i);
    const auto mask = hn::Gt(v, veps);
    const auto y = hn::Mul(vscale, FastLog2dFTabPrime(d, v));
    hn::StoreU(hn::IfThenElseZero(mask, y), d, dst + i);
  }
  const HWY_CAPPED(double, 1) d1;
  for (; i < n; ++i) {
    if (src[i] > 1e-18) {
      const auto v = hn::Set(d1, src[i]);
      dst[i] = scale * hn::GetLane(FastLog2dFTabPrime(d1, v));
    } else {
      dst[i] = 0.0;
    }
  }
}

// dst[k] += src[k] for k in [0, n).
void VecAddVec(double* HWY_RESTRICT dst, const double* HWY_RESTRICT src,
               size_t n) {
  const hn::ScalableTag<double> d;
  const size_t N = hn::Lanes(d);
  size_t k = 0;
  for (; k + N <= n; k += N) {
    hn::StoreU(hn::Add(hn::LoadU(d, dst + k), hn::LoadU(d, src + k)), d,
               dst + k);
  }
  for (; k < n; ++k) dst[k] += src[k];
}

// dst[k] += w * src[k] for k in [0, n).
void AccumScaledVec(double* HWY_RESTRICT dst, double w,
                    const double* HWY_RESTRICT src, size_t n) {
  const hn::ScalableTag<double> d;
  const size_t N = hn::Lanes(d);
  const auto vw = hn::Set(d, w);
  size_t k = 0;
  for (; k + N <= n; k += N) {
    hn::StoreU(hn::MulAdd(vw, hn::LoadU(d, src + k), hn::LoadU(d, dst + k)), d,
               dst + k);
  }
  for (; k < n; ++k) dst[k] += w * src[k];
}

// Returns sum_k a[k]*b[k] for k in [0, n).
double DotProductVec(const double* HWY_RESTRICT a, const double* HWY_RESTRICT b,
                     size_t n) {
  const hn::ScalableTag<double> d;
  const size_t N = hn::Lanes(d);
  auto vsum = hn::Zero(d);
  size_t i = 0;
  for (; i + N <= n; i += N) {
    vsum = hn::MulAdd(hn::LoadU(d, a + i), hn::LoadU(d, b + i), vsum);
  }
  double sum = hn::ReduceSum(d, vsum);
  for (; i < n; ++i) sum += a[i] * b[i];
  return sum;
}

// dst[m] += scale * rho[m] * (drho[m] - s) for m in [0, n).
void ClusterJacobianVec(double* HWY_RESTRICT dst, double scale,
                        const double* HWY_RESTRICT rho,
                        const double* HWY_RESTRICT drho, double s, size_t n) {
  const hn::ScalableTag<double> d;
  const size_t N = hn::Lanes(d);
  const auto vscale = hn::Set(d, scale);
  const auto vs = hn::Set(d, s);
  size_t m = 0;
  for (; m + N <= n; m += N) {
    hn::StoreU(
        hn::MulAdd(hn::Mul(vscale, hn::LoadU(d, rho + m)),
                   hn::Sub(hn::LoadU(d, drho + m), vs), hn::LoadU(d, dst + m)),
        d, dst + m);
  }
  for (; m < n; ++m) dst[m] += scale * rho[m] * (drho[m] - s);
}

// B[k] = sum_cell cell_weight[cell] * rho[cell, k].
void ClusterWeightsVec(const double* HWY_RESTRICT cell_weight,
                       const double* HWY_RESTRICT rho,
                       double* HWY_RESTRICT B, size_t num_cells,
                       size_t num_clusters) {
  const hn::ScalableTag<double> d;
  const size_t N = hn::Lanes(d);
  size_t k = 0;
  for (; k + N <= num_clusters; k += N) {
    hn::StoreU(hn::Zero(d), d, B + k);
  }
  for (; k < num_clusters; ++k) B[k] = 0.0;

  for (size_t cell = 0; cell < num_cells; ++cell) {
    const double w = cell_weight[cell];
    if (w == 0.0) continue;
    const double* HWY_RESTRICT rho_cell = rho + cell * num_clusters;
    const auto vw = hn::Set(d, w);
    k = 0;
    for (; k + N <= num_clusters; k += N) {
      hn::StoreU(hn::MulAdd(vw, hn::LoadU(d, rho_cell + k),
                            hn::LoadU(d, B + k)),
                 d, B + k);
    }
    for (; k < num_clusters; ++k) B[k] += w * rho_cell[k];
  }
}

// For each cell:
//   dcell[cell] += dot(rho[cell, :], D)
//   drho[cell, k] += cell_weight[cell] * D[k]
void CellGradientVec(const double* HWY_RESTRICT cell_weight,
                     const double* HWY_RESTRICT rho,
                     const double* HWY_RESTRICT D,
                     double* HWY_RESTRICT dcell,
                     double* HWY_RESTRICT drho, size_t num_cells,
                     size_t num_clusters) {
  const hn::ScalableTag<double> d;
  const size_t N = hn::Lanes(d);
  for (size_t cell = 0; cell < num_cells; ++cell) {
    const double* HWY_RESTRICT rho_cell = rho + cell * num_clusters;
    double* HWY_RESTRICT drho_cell = drho + cell * num_clusters;
    auto vsum = hn::Zero(d);
    size_t k = 0;
    const double w = cell_weight[cell];
    if (w == 0.0) {
      for (; k + N <= num_clusters; k += N) {
        vsum = hn::MulAdd(hn::LoadU(d, rho_cell + k), hn::LoadU(d, D + k),
                          vsum);
      }
      double sum = hn::ReduceSum(d, vsum);
      for (; k < num_clusters; ++k) sum += rho_cell[k] * D[k];
      dcell[cell] += sum;
      continue;
    }

    const auto vw = hn::Set(d, w);
    for (; k + N <= num_clusters; k += N) {
      const auto vd = hn::LoadU(d, D + k);
      vsum = hn::MulAdd(hn::LoadU(d, rho_cell + k), vd, vsum);
      hn::StoreU(hn::MulAdd(vw, vd, hn::LoadU(d, drho_cell + k)), d,
                 drho_cell + k);
    }
    double sum = hn::ReduceSum(d, vsum);
    for (; k < num_clusters; ++k) {
      sum += rho_cell[k] * D[k];
      drho_cell[k] += w * D[k];
    }
    dcell[cell] += sum;
  }
}

// Converts row-wise softmax probability gradients into logit gradients:
//   dst[row, m] += scale * prob[row, m] *
//                  (dprob[row, m] - dot(prob[row, :], dprob[row, :])).
void SoftmaxJacobianRowsVec(const double* HWY_RESTRICT prob,
                            const double* HWY_RESTRICT dprob,
                            double* HWY_RESTRICT dst, double scale,
                            size_t num_rows, size_t row_size) {
  const hn::ScalableTag<double> d;
  const size_t N = hn::Lanes(d);
  const auto vscale = hn::Set(d, scale);
  for (size_t row = 0; row < num_rows; ++row) {
    const double* HWY_RESTRICT p_row = prob + row * row_size;
    const double* HWY_RESTRICT dp_row = dprob + row * row_size;
    double* HWY_RESTRICT dst_row = dst + row * row_size;

    auto vsum = hn::Zero(d);
    size_t k = 0;
    for (; k + N <= row_size; k += N) {
      vsum = hn::MulAdd(hn::LoadU(d, p_row + k), hn::LoadU(d, dp_row + k),
                        vsum);
    }
    double s = hn::ReduceSum(d, vsum);
    for (; k < row_size; ++k) s += p_row[k] * dp_row[k];

    const auto vs = hn::Set(d, s);
    k = 0;
    for (; k + N <= row_size; k += N) {
      hn::StoreU(hn::MulAdd(hn::Mul(vscale, hn::LoadU(d, p_row + k)),
                            hn::Sub(hn::LoadU(d, dp_row + k), vs),
                            hn::LoadU(d, dst_row + k)),
                 d, dst_row + k);
    }
    for (; k < row_size; ++k) {
      dst_row[k] += scale * p_row[k] * (dp_row[k] - s);
    }
  }
}

// `ac_h` layout: `[di * cp_count + cp]` where `cp_count = num_clusters * num_passes`.
// Looping `di` outer keeps each `v_row` sequential in memory (L1-resident).
void ContextForwardVec(const double* HWY_RESTRICT ac_h,
                       const double* HWY_RESTRICT sigma,
                       const uint32_t* HWY_RESTRICT dense_to_zdc,
                       const uint32_t* HWY_RESTRICT dense_to_token,
                       double* HWY_RESTRICT ctx_h,
                       double* HWY_RESTRICT ctx_N, size_t num_clusters,
                       size_t num_passes, size_t ac_alpha, size_t zdc_count,
                       size_t token_count, size_t num_hists) {
  const hn::ScalableTag<double> d;
  const size_t N = hn::Lanes(d);
  const size_t cp_count = num_clusters * num_passes;
  const size_t sigma_pk_stride = num_clusters * zdc_count * num_hists;
  const size_t sigma_k_stride = zdc_count * num_hists;
  const size_t ctx_h_p_stride = token_count * num_hists;

  for (size_t di = 0; di < ac_alpha; ++di) {
    const double* HWY_RESTRICT v_row = ac_h + di * cp_count;
    const size_t zdc = dense_to_zdc[di];
    const size_t token = dense_to_token[di];
    for (size_t p = 0; p < num_passes; ++p) {
      double* HWY_RESTRICT dst = ctx_h + p * ctx_h_p_stride + token * num_hists;
      const double* HWY_RESTRICT sigma_p = sigma + p * sigma_pk_stride;
      for (size_t k = 0; k < num_clusters; ++k) {
        const double v = v_row[k * num_passes + p];
        if (v == 0.0) continue;
        const double* HWY_RESTRICT sig =
            sigma_p + k * sigma_k_stride + zdc * num_hists;
        const auto vv = hn::Set(d, v);
        size_t h = 0;
        for (; h + N <= num_hists; h += N) {
          hn::StoreU(hn::MulAdd(vv, hn::LoadU(d, sig + h),
                                hn::LoadU(d, dst + h)),
                     d, dst + h);
        }
        for (; h < num_hists; ++h) dst[h] += v * sig[h];
      }
    }
  }

  // `ctx_N[p, h] = sum_token ctx_h[p, token, h]`.
  for (size_t p = 0; p < num_passes; ++p) {
    double* HWY_RESTRICT ctx_N_p = ctx_N + p * num_hists;
    const double* HWY_RESTRICT ctx_h_p = ctx_h + p * ctx_h_p_stride;
    size_t h = 0;
    for (; h + N <= num_hists; h += N) {
      auto vsum = hn::Zero(d);
      for (size_t token = 0; token < token_count; ++token) {
        vsum = hn::Add(vsum, hn::LoadU(d, ctx_h_p + token * num_hists + h));
      }
      hn::StoreU(vsum, d, ctx_N_p + h);
    }
    for (; h < num_hists; ++h) {
      double sum = 0.0;
      for (size_t token = 0; token < token_count; ++token) {
        sum += ctx_h_p[token * num_hists + h];
      }
      ctx_N_p[h] = sum;
    }
  }
}

// `ac_h` layout: `[di * cp_count + cp]`.
// `ac_N` layout: `[zdc * cp_count + cp]`.
// `dL_dh` output layout: `[cp * ac_alpha + di]` (original, consumed by block backward).
void ContextBackwardVec(const double* HWY_RESTRICT ac_h,
                        const double* HWY_RESTRICT ac_N,
                        const double* HWY_RESTRICT sigma,
                        const double* HWY_RESTRICT dctx_h,
                        const double* HWY_RESTRICT dctx_N,
                        const uint32_t* HWY_RESTRICT dense_to_zdc,
                        const uint32_t* HWY_RESTRICT dense_to_token,
                        double* HWY_RESTRICT dL_dh,
                        double* HWY_RESTRICT dL_dN,
                        double* HWY_RESTRICT dL_dsigma, size_t num_clusters,
                        size_t num_passes, size_t ac_alpha, size_t zdc_count,
                        size_t token_count, size_t num_hists) {
  const hn::ScalableTag<double> d;
  const size_t N = hn::Lanes(d);
  const size_t cp_count = num_clusters * num_passes;
  const size_t sigma_pk_stride = num_clusters * zdc_count * num_hists;
  const size_t sigma_k_stride = zdc_count * num_hists;
  const size_t ctx_h_p_stride = token_count * num_hists;

  // `dL_dh[cp][di]` and `dL_dN[cp][zdc]`: keep `(k,p)` outer since outputs are
  // `cp`-major and sigma access is the same as before.
  for (size_t k = 0; k < num_clusters; ++k) {
    for (size_t p = 0; p < num_passes; ++p) {
      const size_t cp = k * num_passes + p;
      const double* HWY_RESTRICT sigma_pk =
          sigma + p * sigma_pk_stride + k * sigma_k_stride;
      const double* HWY_RESTRICT dctx_h_p = dctx_h + p * ctx_h_p_stride;
      const double* HWY_RESTRICT dctx_N_p = dctx_N + p * num_hists;
      double* HWY_RESTRICT dh_cp = dL_dh + cp * ac_alpha;
      double* HWY_RESTRICT dN_cp = dL_dN + cp * zdc_count;

      for (size_t di = 0; di < ac_alpha; ++di) {
        const size_t zdc = dense_to_zdc[di];
        const size_t token = dense_to_token[di];
        const double* HWY_RESTRICT sig = sigma_pk + zdc * num_hists;
        const double* HWY_RESTRICT src = dctx_h_p + token * num_hists;
        auto vsum = hn::Zero(d);
        size_t h = 0;
        for (; h + N <= num_hists; h += N) {
          vsum = hn::MulAdd(hn::LoadU(d, sig + h), hn::LoadU(d, src + h),
                            vsum);
        }
        double sum = hn::ReduceSum(d, vsum);
        for (; h < num_hists; ++h) sum += sig[h] * src[h];
        dh_cp[di] = sum;
      }

      for (size_t zdc = 0; zdc < zdc_count; ++zdc) {
        const double* HWY_RESTRICT sig = sigma_pk + zdc * num_hists;
        auto vsum = hn::Zero(d);
        size_t h = 0;
        for (; h + N <= num_hists; h += N) {
          vsum = hn::MulAdd(hn::LoadU(d, sig + h),
                            hn::LoadU(d, dctx_N_p + h), vsum);
        }
        double sum = hn::ReduceSum(d, vsum);
        for (; h < num_hists; ++h) sum += sig[h] * dctx_N_p[h];
        dN_cp[zdc] = sum;
      }
    }
  }

  if (dL_dsigma == nullptr) return;

  // `dL_dsigma` from `ac_h`: `di`-outer loop keeps `v_row` sequential.
  for (size_t di = 0; di < ac_alpha; ++di) {
    const double* HWY_RESTRICT v_row = ac_h + di * cp_count;
    const size_t zdc = dense_to_zdc[di];
    const size_t token = dense_to_token[di];
    for (size_t p = 0; p < num_passes; ++p) {
      const double* HWY_RESTRICT src =
          dctx_h + p * ctx_h_p_stride + token * num_hists;
      for (size_t k = 0; k < num_clusters; ++k) {
        const double v = v_row[k * num_passes + p];
        if (v == 0.0) continue;
        double* HWY_RESTRICT dst =
            dL_dsigma + p * sigma_pk_stride + k * sigma_k_stride +
            zdc * num_hists;
        const auto vv = hn::Set(d, v);
        size_t h = 0;
        for (; h + N <= num_hists; h += N) {
          hn::StoreU(hn::MulAdd(vv, hn::LoadU(d, src + h),
                                hn::LoadU(d, dst + h)),
                     d, dst + h);
        }
        for (; h < num_hists; ++h) dst[h] += v * src[h];
      }
    }
  }

  // `dL_dsigma` from `ac_N`: `zdc`-outer loop keeps `vN_row` sequential.
  for (size_t zdc = 0; zdc < zdc_count; ++zdc) {
    const double* HWY_RESTRICT vN_row = ac_N + zdc * cp_count;
    for (size_t p = 0; p < num_passes; ++p) {
      const double* HWY_RESTRICT src = dctx_N + p * num_hists;
      for (size_t k = 0; k < num_clusters; ++k) {
        const double v = vN_row[k * num_passes + p];
        if (v == 0.0) continue;
        double* HWY_RESTRICT dst =
            dL_dsigma + p * sigma_pk_stride + k * sigma_k_stride +
            zdc * num_hists;
        const auto vv = hn::Set(d, v);
        size_t h = 0;
        for (; h + N <= num_hists; h += N) {
          hn::StoreU(hn::MulAdd(vv, hn::LoadU(d, src + h),
                                hn::LoadU(d, dst + h)),
                     d, dst + h);
        }
        for (; h < num_hists; ++h) dst[h] += v * src[h];
      }
    }
  }
}

void AdamApplyVec(const double* HWY_RESTRICT grad,
                  double* HWY_RESTRICT param,
                  double* HWY_RESTRICT m,
                  double* HWY_RESTRICT v, size_t n, double beta1,
                  double beta2, double lr, double inv_bias1,
                  double inv_bias2, double eps) {
  const hn::ScalableTag<double> d;
  const size_t N = hn::Lanes(d);
  const auto vb1 = hn::Set(d, beta1);
  const auto vomb1 = hn::Set(d, 1.0 - beta1);
  const auto vb2 = hn::Set(d, beta2);
  const auto vomb2 = hn::Set(d, 1.0 - beta2);
  const auto vlr = hn::Set(d, lr);
  const auto vib1 = hn::Set(d, inv_bias1);
  const auto vib2 = hn::Set(d, inv_bias2);
  const auto veps = hn::Set(d, eps);
  size_t i = 0;
  for (; i + N <= n; i += N) {
    const auto g = hn::LoadU(d, grad + i);
    const auto new_m = hn::MulAdd(vomb1, g, hn::Mul(vb1, hn::LoadU(d, m + i)));
    const auto new_v =
        hn::MulAdd(vomb2, hn::Mul(g, g), hn::Mul(vb2, hn::LoadU(d, v + i)));
    hn::StoreU(new_m, d, m + i);
    hn::StoreU(new_v, d, v + i);

    const auto m_hat = hn::Mul(new_m, vib1);
    const auto v_hat = hn::Mul(new_v, vib2);
    const auto denom = hn::Add(hn::Sqrt(v_hat), veps);
    const auto update = hn::Mul(vlr, hn::Div(m_hat, denom));
    hn::StoreU(hn::Sub(hn::LoadU(d, param + i), update), d, param + i);
  }
  for (; i < n; ++i) {
    m[i] = beta1 * m[i] + (1.0 - beta1) * grad[i];
    v[i] = beta2 * v[i] + (1.0 - beta2) * grad[i] * grad[i];
    const double m_hat = m[i] * inv_bias1;
    const double v_hat = v[i] * inv_bias2;
    param[i] -= lr * m_hat / (std::sqrt(v_hat) + eps);
  }
}

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
  // Saved per-threshold sigmoid values `sigma_{a,j} = sigmoid((T[a][j]-DC)/tau)`.
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
                       double inv_temperature, double* out,
                       double* sigma = nullptr) {
  const size_t K = thresholds.size() + 1;
  if (K == 1) {
    out[0] = 1.0;
    return;
  }
  double prev = 0.0;
  for (size_t k = 0; k + 1 < K; ++k) {
    const double cur = SafeSigmoid((thresholds[k] - dc - 0.5) * inv_temperature);
    out[k] = cur - prev;
    if (sigma != nullptr) sigma[k] = cur;
    prev = cur;
  }
  out[K - 1] = 1.0 - prev;
}

// Returns the DC value of the block owning position (y, x) in `src_channel`
// when projected onto `dst_channel`'s grid.
int DCValueForAxis(const JPEGOptData& d, uint32_t src_channel, uint32_t y,
                   uint32_t x, uint32_t dst_channel) {
  const uint32_t b = MapTopLeftBlockIndex(d, src_channel, y, x, dst_channel);
  return d.DC_vals[dst_channel][d.block_DC_idx[dst_channel][b]];
}

// Resolves the block's DC values on all three DC axes, honoring subsampling
// and the grayscale fast path.
void BlockDCValues(const JPEGOptData& d, uint32_t c, uint32_t b, int out[kNumCh]) {
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

// Derives the integer predictor bucket `pb` from a fractional `predicted_nz`.
// Mirrors the hard formula in `EvaluatePassAwareModel` exactly once the
// fractional value is floored.
inline uint32_t PredictorBucketFromPredictedNZ(double predicted_nz) {
  const int nz_int =
      std::max(0, static_cast<int>(std::floor(predicted_nz)));
  uint32_t pb = (nz_int < 8) ? static_cast<uint32_t>(nz_int)
                             : (4u + static_cast<uint32_t>(nz_int) / 2u);
  return std::min<uint32_t>(pb, kJPEGNonZeroBuckets - 1);
}

// Flat per-pass overhead constant; duplicated from `ComputePassOverhead` in
// `enc_jpeg_pass_cluster.cc` to avoid a circular include. Units: bits (not
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
// overhead term is not differentiated; this integer projection is enough.
double ACSignallingOverheadBitsForSlot(const JPEGOptData& d,
                                       const double* ac_h_transposed,
                                       size_t cp, size_t cp_count,
                                       size_t ac_h_size) {
  std::array<std::array<uint32_t, kACTokenCount>, kZeroDensityContextCount>
      signalling_hist = {};
  const auto& dense_to_symbol = d.ACHistogram().dense_to_zdcvalue;
  const size_t dense_size = dense_to_symbol.size();
  double overhead_bits = 0.0;
  for (size_t idx = 0; idx < ac_h_size && idx < dense_size; ++idx) {
    const double v = ac_h_transposed[idx * cp_count + cp];
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
double NZSignallingOverheadBitsForSlot(const double* nz_h) {
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
      max_nz = std::max(max_nz, nz);
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

}  // namespace

// Forward + optional backward. Shared body for all public entry points.
// Computes AC + NZ + signalling overhead.
// Iteration 5 moved `ctx_map` and `num_clusters` into `state` and added soft
// cluster membership via `state.cluster_logits`.
SoftCostResult ComputeSoftCostImpl(const JPEGOptData& d,
                                   const GradientJointState& state,
                                   GradientJointGrad* grad) {
  SoftCostResult result;
  const size_t num_passes = state.num_passes;
  const size_t num_clusters = state.num_clusters;
  if (num_clusters == 0 || num_passes == 0) return result;

  const std::array<uint32_t, kNumCh> n_axis = {
      static_cast<uint32_t>(state.thresholds[0].size()) + 1,
      static_cast<uint32_t>(state.thresholds[1].size()) + 1,
      static_cast<uint32_t>(state.thresholds[2].size()) + 1};
  const size_t num_cells = n_axis[0] * n_axis[1] * n_axis[2];
  if (state.num_cells != num_cells) return result;  // state inconsistent.
  for (uint32_t c = 0; c < d.channels; ++c) {
    if (state.cluster_logits[c].size() != num_cells * num_clusters) {
      return result;  // cluster_logits not sized to match state.
    }
  }

  const size_t cp_count = num_clusters * num_passes;
  const uint32_t ac_alpha = d.ACHistogramSize();
  constexpr uint32_t kZDC = kZeroDensityContextCount;
  constexpr uint32_t kNZBins = kNZHistogramsSize;       // 36 * 64 = 2304
  constexpr uint32_t kNZBuckets = kJPEGNonZeroBuckets;  // 36
  const double inv_pass_t = 1.0 / state.pass_temperature;
  const double inv_thr_t = 1.0 / state.threshold_temperature;
  const double inv_cluster_t = 1.0 / state.cluster_temperature;
  const double inv_ctx_t = 1.0 / state.ctx_temperature;
  const uint32_t H = state.num_hists;
  if (state.ctx_logits.size() != num_passes * num_clusters * kZDC * H) {
    return result;  // ctx_logits not initialized; caller must set num_hists.
  }

  // AC forward accumulators in di-major / zdc-major transposed layouts so each
  // AC event triggers one VecAddVec(cp_count) rather than K*P scatter writes.
  std::vector<double> ac_h(ac_alpha * cp_count, 0.0);
  std::vector<double> ac_N(kZDC * cp_count, 0.0);
  std::vector<double> nz_h(cp_count * kNZBins, 0.0);
  std::vector<double> nz_N(cp_count * kNZBuckets, 0.0);

  // Lookup tables: dense bin -> zdc and token for ctx routing.
  std::vector<uint32_t> dense_to_zdc_lut(ac_alpha);
  std::vector<uint32_t> dense_to_token_lut(ac_alpha);
  {
    const CompactACHistogramData& hist = d.ACHistogram();
    for (uint32_t di = 0; di < ac_alpha; ++di) {
      const SignallingHistSymbol sym =
          d.SignallingHistSymbolFromSymbol(hist.dense_to_zdcvalue[di]);
      dense_to_zdc_lut[di] = sym.zdc;
      dense_to_token_lut[di] = sym.token;
    }
  }

  // Precompute block-pi vectors for pass weights and NZ neighbor lookups.
  // Flat: pi_cache[c][b * num_passes + p].
  std::array<std::vector<double>, kNumCh> pi_cache;
  for (uint32_t c = 0; c < d.channels; ++c) {
    const size_t nb = d.num_blocks[c];
    pi_cache[c].resize(nb * num_passes);
    for (size_t b = 0; b < nb; ++b) {
      Softmax(&state.pass_logits[c][b * num_passes], num_passes,
              state.pass_temperature, pi_cache[c].data() + b * num_passes);
    }
  }

  // Precompute per-(channel, cell) cluster-softmax `rho`. Iteration 5 softens
  // the old `cluster = ctx_map[c*num_cells + cell]` lookup into a distribution
  // over clusters: `rho_{c,cell,k} = softmax(cluster_logits[c][cell*K..],
  // cluster_temperature)[k]`.
  std::array<std::vector<double>, kNumCh> rho_cache;
  for (uint32_t c = 0; c < d.channels; ++c) {
    rho_cache[c].assign(num_cells * num_clusters, 0.0);
    for (size_t cell = 0; cell < num_cells; ++cell) {
      Softmax(&state.cluster_logits[c][cell * num_clusters], num_clusters,
              state.cluster_temperature, &rho_cache[c][cell * num_clusters]);
    }
  }

  // Soft context-map sigma: softmax(ctx_logits[p,k,zdc,:], ctx_temperature).
  // Flat layout: (p * K * kZDC + k * kZDC + zdc) * H + h.
  std::vector<double> sigma(num_passes * num_clusters * kZDC * H);
  for (uint32_t p = 0; p < num_passes; ++p) {
    for (uint32_t k = 0; k < num_clusters; ++k) {
      for (uint32_t zdc = 0; zdc < kZDC; ++zdc) {
        const size_t off = (p * num_clusters * kZDC + k * kZDC + zdc) * H;
        Softmax(&state.ctx_logits[off], H, state.ctx_temperature, &sigma[off]);
      }
    }
  }

  // Per-block scratch.
  std::vector<double> w0(n_axis[0]);
  std::vector<double> w1(n_axis[1]);
  std::vector<double> w2(n_axis[2]);
  std::vector<double> cell_weight(num_cells, 0.0);
  // Per-block per-cluster aggregate weight, B[k] = sum_cell w_cell*rho[c,cell,k].
  // Reused by both AC and NZ forward and by the backward pass; sized to fit
  // any cluster axis count, allocated once per call.
  std::vector<double> B(num_clusters, 0.0);
  // w_vec[k*P+p] = B[k]*pi[p]; drives VecAddVec per AC event.
  std::vector<double> w_vec(cp_count);

  // --- Forward pass ---------------------------------------------------------
  for (uint32_t c = 0; c < d.channels; ++c) {
    const uint32_t nb = d.num_blocks[c];
    const uint32_t grid_w = d.block_grid_w[c];
    for (uint32_t b = 0; b < nb; ++b) {
      const double* pi = &pi_cache[c][b * num_passes];

      int dc[kNumCh];
      BlockDCValues(d, c, b, dc);
      AxisBucketWeights(state.thresholds[0], dc[0], inv_thr_t, w0.data());
      AxisBucketWeights(state.thresholds[1], dc[1], inv_thr_t, w1.data());
      AxisBucketWeights(state.thresholds[2], dc[2], inv_thr_t, w2.data());

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

      const uint32_t ev_start = d.block_offsets[c][b];
      const uint32_t ev_end = d.block_offsets[c][b + 1];

      // Per-block per-cluster aggregate `B[k] = sum_cell w_cell*rho[c,cell,k]`.
      // This collapses the cell axis out of the inner accumulation, dropping a
      // factor of `num_cells` from the AC and NZ inner loops.
      ClusterWeightsVec(cell_weight.data(), rho_cache[c].data(), B.data(),
                        num_cells, num_clusters);

      // AC accumulation.
      // Compute w_vec[k*P+p] = B[k]*pi[p] once, then per event add
      // w_vec into the transposed accumulators with a single VecAddVec.
      // This replaces K*P random scatter writes with one sequential SIMD
      // add per event, and keeps ac_h footprint L2-resident.
      for (uint32_t k = 0; k < num_clusters; ++k) {
        const double Bk = B[k];
        for (uint32_t p = 0; p < num_passes; ++p) {
          w_vec[k * num_passes + p] = Bk * pi[p];
        }
      }
      for (uint32_t e = ev_start; e < ev_end; ++e) {
        const CompactACEvent evt = d.FromBin(d.block_bins[c][e]);
        if (evt.hist_bin == kInvalidCompactH) continue;
        VecAddVec(&ac_h[evt.hist_bin * cp_count], w_vec.data(), cp_count);
        VecAddVec(&ac_N[evt.zdc * cp_count], w_vec.data(), cp_count);
      }

      // NZ accumulation: pb depends only on (block, pass), so precompute it
      // outside the cluster loop. Same B[k] aggregation collapses the cell
      // axis here too.
      const uint32_t y = b / grid_w;
      const uint32_t x = b % grid_w;
      const uint32_t nz_b = d.block_nonzeros[c][b];
      const double* pi_top = nullptr;
      const double* pi_left = nullptr;
      double nz_top = 0;
      double nz_left = 0;
      if (y > 0) {
        const uint32_t b_top = (y - 1) * grid_w + x;
        pi_top = &pi_cache[c][b_top * num_passes];
        nz_top = d.block_nonzeros[c][b_top];
      }
      if (x > 0) {
        const uint32_t b_left = y * grid_w + (x - 1);
        pi_left = &pi_cache[c][b_left * num_passes];
        nz_left = d.block_nonzeros[c][b_left];
      }
      for (uint32_t p = 0; p < num_passes; ++p) {
        double pass_nz_top = (y > 0) ? pi_top[p] * nz_top : 0.0;
        double pass_nz_left = (x > 0) ? pi_left[p] * nz_left : 0.0;
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
        const double one_minus_pi_p = 1.0 - pi_p;
        for (uint32_t k = 0; k < num_clusters; ++k) {
          const double Bk = B[k];
          if (Bk == 0.0) continue;
          const size_t cp = k * num_passes + p;
          nz_N[cp * kNZBuckets + pb] += Bk;
          nz_h[cp * kNZBins + bin_real] += Bk * pi_p;
          nz_h[cp * kNZBins + bin_zero] += Bk * one_minus_pi_p;
        }
      }
    }
  }

  // Context histograms: ctx_h[p, token, h] and ctx_N[p, h].
  // Layouts: ctx_h index = p * kACTokenCount * H + token * H + h,
  //          ctx_N index = p * H + h.
  std::vector<double> ctx_h(num_passes * kACTokenCount * H, 0.0);
  std::vector<double> ctx_N(num_passes * H, 0.0);
  ContextForwardVec(ac_h.data(), sigma.data(), dense_to_zdc_lut.data(),
                    dense_to_token_lut.data(), ctx_h.data(), ctx_N.data(),
                    num_clusters, num_passes, ac_alpha, kZDC, kACTokenCount,
                    H);

  // --- AC cost reduction ----------------------------------------------------
  double ac_cost = 0.0;
  uint32_t touched_slots = 0;
  for (uint32_t p = 0; p < num_passes; ++p) {
    const double N_sum = SoftFTabReduceVecFast(&ctx_N[p * H], H);
    const double h_sum = SoftFTabReduceVecFast(
        &ctx_h[p * kACTokenCount * H], kACTokenCount * H);
    ac_cost += N_sum - h_sum;
    if (N_sum != 0.0 || h_sum != 0.0) ++touched_slots;
  }
  result.ac_cost_bits = ac_cost;
  result.num_cp_slots = touched_slots;

  // --- NZ cost + signalling overhead ---------------------------------------
  double nz_cost = 0.0;
  double overhead_bits = 0.0;
  for (uint32_t cp = 0; cp < cp_count; ++cp) {
    nz_cost +=
        SoftFTabReduceVecFast(&nz_N[cp * kNZBuckets], kNZBuckets);
    nz_cost -= SoftFTabReduceVecFast(&nz_h[cp * kNZBins], kNZBins);
    overhead_bits += ACSignallingOverheadBitsForSlot(
        d, ac_h.data(), cp, cp_count, ac_alpha);
    overhead_bits += NZSignallingOverheadBitsForSlot(&nz_h[cp * kNZBins]);
  }
  overhead_bits += FlatPassOverheadBits(d) * num_passes;
  result.nz_cost_bits = nz_cost;
  result.signalling_overhead_bits = overhead_bits;
  result.total_cost_bits = ac_cost + nz_cost + overhead_bits;

  if (grad == nullptr) return result;

  // --- Backward pass --------------------------------------------------------
  // Precompute upstream gradients wrt ac_h / ac_N accumulators.
  std::vector<double> dL_dN(cp_count * kZDC, 0.0);
  std::vector<double> dL_dh(cp_count * ac_alpha, 0.0);
  // Upstream from context histograms: dL/dctx_N = +ftab', dL/dctx_h = -ftab'.
  std::vector<double> dL_dctx_N(num_passes * H, 0.0);
  std::vector<double> dL_dctx_h(num_passes * kACTokenCount * H, 0.0);
  SoftFTabPrimeVecFast(ctx_N.data(), dL_dctx_N.data(), 1.0, num_passes * H);
  SoftFTabPrimeVecFast(ctx_h.data(), dL_dctx_h.data(), -1.0,
                       num_passes * kACTokenCount * H);

  // Propagate through sigma to get dL/dh, dL/dN, and optionally dL/dsigma.
  std::vector<double> dL_dsigma;
  double* dL_dsigma_data = nullptr;
  if (!grad->ctx_logits.empty()) {
    dL_dsigma.assign(num_passes * num_clusters * kZDC * H, 0.0);
    dL_dsigma_data = dL_dsigma.data();
  }
  ContextBackwardVec(ac_h.data(), ac_N.data(), sigma.data(),
                     dL_dctx_h.data(), dL_dctx_N.data(),
                     dense_to_zdc_lut.data(), dense_to_token_lut.data(),
                     dL_dh.data(), dL_dN.data(), dL_dsigma_data, num_clusters,
                     num_passes, ac_alpha, kZDC, kACTokenCount, H);

  // dL/dsigma softmax Jacobian -> grad->ctx_logits.
  if (dL_dsigma_data != nullptr) {
    // Softmax Jacobian per (p, k, zdc) row.
    SoftmaxJacobianRowsVec(sigma.data(), dL_dsigma.data(),
                           grad->ctx_logits.data(), inv_ctx_t,
                           num_passes * num_clusters * kZDC, H);
  }

  // Per-block backward storage.
  AxisBucketBackward back0;
  AxisBucketBackward back1;
  AxisBucketBackward back2;
  back0.sigma.resize(state.thresholds[0].size());
  back1.sigma.resize(state.thresholds[1].size());
  back2.sigma.resize(state.thresholds[2].size());
  std::vector<double> dL_dcell(num_cells, 0.0);
  std::vector<double> dL_dpi(num_passes, 0.0);
  std::vector<double> dL_dw_ax[kNumCh];
  for (uint32_t a = 0; a < kNumCh; ++a) {
    dL_dw_ax[a].resize(n_axis[a]);
  }

  // Channel-wide accumulator for `dL/drho[c][cell * K + k]`. Accumulates
  // contributions from every block in channel `c`; converted to
  // `dL/dcluster_logits[c]` via the softmax Jacobian after the block loop.
  std::array<std::vector<double>, kNumCh> dL_drho;
  for (uint32_t c = 0; c < d.channels; ++c) {
    dL_drho[c].assign(num_cells * num_clusters, 0.0);
  }

  // NZ upstream gradients.
  std::vector<double> dL_dnz_N(cp_count * kNZBuckets, 0.0);
  std::vector<double> dL_dnz_h(cp_count * kNZBins, 0.0);
  for (uint32_t cp = 0; cp < cp_count; ++cp) {
    SoftFTabPrimeVecFast(&nz_N[cp * kNZBuckets],
                         &dL_dnz_N[cp * kNZBuckets], 1.0, kNZBuckets);
    SoftFTabPrimeVecFast(&nz_h[cp * kNZBins], &dL_dnz_h[cp * kNZBins], -1.0,
                         kNZBins);
  }

  // Per-block-per-(cluster, pass) scratch tables for the factored backward.
  // `delta_ac_kp[cp]` is the events-aggregated AC gradient at slot cp;
  // `nz_T_kp[cp]` and `nz_diff_h_kp[cp]` carry the NZ-derived per-(k,p)
  // factors. `block_D[k]` is the per-block per-cluster reduction over passes.
  std::vector<double> delta_ac_kp(cp_count, 0.0);
  std::vector<double> nz_T_kp(cp_count, 0.0);
  std::vector<double> nz_diff_h_kp(cp_count, 0.0);
  std::vector<double> block_D(num_clusters, 0.0);

  for (uint32_t c = 0; c < d.channels; ++c) {
    const uint32_t nb = d.num_blocks[c];
    const uint32_t grid_w = d.block_grid_w[c];
    for (uint32_t b = 0; b < nb; ++b) {
      const double* pi = &pi_cache[c][b * num_passes];

      int dc[kNumCh];
      BlockDCValues(d, c, b, dc);
      AxisBucketWeights(state.thresholds[0], dc[0], inv_thr_t, w0.data(),
                        back0.sigma.data());
      AxisBucketWeights(state.thresholds[1], dc[1], inv_thr_t, w1.data(),
                        back1.sigma.data());
      AxisBucketWeights(state.thresholds[2], dc[2], inv_thr_t, w2.data(),
                        back2.sigma.data());

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

      // Zero per-block accumulators.
      std::fill(dL_dcell.begin(), dL_dcell.end(), 0.0);
      std::fill(dL_dpi.begin(), dL_dpi.end(), 0.0);

      const uint32_t ev_start = d.block_offsets[c][b];
      const uint32_t ev_end = d.block_offsets[c][b + 1];

      // Per-block per-cluster aggregate `B[k] = sum_cell w_cell*rho[c,cell,k]`.
      // Matches the forward pass; reused by both AC and NZ backward to factor
      // the cell axis out of the inner loops.
      ClusterWeightsVec(cell_weight.data(), rho_cache[c].data(), B.data(),
                        num_cells, num_clusters);

      // AC contribution: per-(k, p) delta.
      //   dL/dpi[p]   = sum_k B[k] * delta_ac[k*P+p]
      //   dL/dcell    = sum_k rho_cell[k] * D[k], D[k] = sum_p pi[p]*delta
      //   dL/drho[c,cell,k] += w_cell * D[k]
      for (uint32_t k = 0; k < num_clusters; ++k) {
        for (uint32_t p = 0; p < num_passes; ++p) {
          const uint32_t cp = k * num_passes + p;
          double delta = 0.0;
          const double* h_row = dL_dh.data() + cp * ac_alpha;
          const double* N_row = dL_dN.data() + cp * kZDC;
          for (uint32_t e = ev_start; e < ev_end; ++e) {
            const CompactACEvent evt = d.FromBin(d.block_bins[c][e]);
            if (evt.hist_bin == kInvalidCompactH) continue;
            delta += h_row[evt.hist_bin] + N_row[evt.zdc];
          }
          delta_ac_kp[cp] = delta;
        }
      }

      // NZ contribution: pb is per-(block, pass), so precompute outside the
      // cluster loop. For each (k, p) we record:
      //   nz_diff_h[cp] = h_real_g - h_zero_g  (drives dL/dpi)
      //   nz_T_kp[cp]   = pi_p*h_real_g + (1-pi_p)*h_zero_g + N_g
      //                   (combines into D[k] for cell/rho gradients)
      const uint32_t y = b / grid_w;
      const uint32_t x = b % grid_w;
      const uint32_t nz_b = d.block_nonzeros[c][b];
      const double* pi_top = nullptr;
      const double* pi_left = nullptr;
      uint32_t nz_top = 0;
      uint32_t nz_left = 0;
      if (y > 0) {
        const uint32_t b_top = (y - 1) * grid_w + x;
        pi_top = pi_cache[c].data() + b_top * num_passes;
        nz_top = d.block_nonzeros[c][b_top];
      }
      if (x > 0) {
        const uint32_t b_left = y * grid_w + (x - 1);
        pi_left = pi_cache[c].data() + b_left * num_passes;
        nz_left = d.block_nonzeros[c][b_left];
      }
      for (uint32_t p = 0; p < num_passes; ++p) {
        double pass_nz_top = (y > 0) ? pi_top[p] * nz_top : 0.0;
        double pass_nz_left = (x > 0) ? pi_left[p] * nz_left : 0.0;
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
        const double one_minus_pi_p = 1.0 - pi_p;
        for (uint32_t k = 0; k < num_clusters; ++k) {
          const uint32_t cp = k * num_passes + p;
          const double h_real_g = dL_dnz_h[cp * kNZBins + bin_real];
          const double h_zero_g = dL_dnz_h[cp * kNZBins + bin_zero];
          const double N_g = dL_dnz_N[cp * kNZBuckets + pb];
          nz_diff_h_kp[cp] = h_real_g - h_zero_g;
          nz_T_kp[cp] = pi_p * h_real_g + one_minus_pi_p * h_zero_g + N_g;
        }
      }

      // Build D[k] = sum_p (pi[p] * delta_ac[cp] + nz_T_kp[cp]) and pi-axis
      // gradient sums.
      std::vector<double>& D = block_D;
      for (uint32_t k = 0; k < num_clusters; ++k) {
        double dk = 0.0;
        for (uint32_t p = 0; p < num_passes; ++p) {
          const uint32_t cp = k * num_passes + p;
          dk += pi[p] * delta_ac_kp[cp];
          dk += nz_T_kp[cp];
        }
        D[k] = dk;
      }

      // dL/dpi[p] from AC and NZ contributions, factored via B[k].
      for (uint32_t k = 0; k < num_clusters; ++k) {
        const double Bk = B[k];
        if (Bk == 0.0) continue;
        for (uint32_t p = 0; p < num_passes; ++p) {
          const uint32_t cp = k * num_passes + p;
          dL_dpi[p] += Bk * delta_ac_kp[cp];
          dL_dpi[p] += Bk * nz_diff_h_kp[cp];
        }
      }

      // dL/dcell[cell] = sum_k rho_cell[k] * D[k]; dL/drho[c,cell,k] +=
      // w_cell * D[k]. Single per-cell pass over clusters covers both.
      CellGradientVec(cell_weight.data(), rho_cache[c].data(), D.data(),
                      dL_dcell.data(), dL_drho[c].data(), num_cells,
                      num_clusters);

      // Softmax Jacobian: dL/dlogit[q] = pi[q] * (dL/dpi[q] - s) / tau_pi,
      // where s = sum_p pi[p] * dL/dpi[p].
      double s = 0.0;
      for (uint32_t p = 0; p < num_passes; ++p) s += pi[p] * dL_dpi[p];
      double* grad_logits = &grad->pass_logits[c][b * num_passes];
      for (uint32_t q = 0; q < num_passes; ++q) {
        grad_logits[q] += inv_pass_t * pi[q] * (dL_dpi[q] - s);
      }

      // Decompose dL/dcell into dL/dw_axis. Axis 0's weight at k0 pairs with
      // w1[k1]*w2[k2] in the product, so that's the factor we multiply by.
      for (uint32_t a = 0; a < kNumCh; ++a) {
        std::fill(dL_dw_ax[a].begin(), dL_dw_ax[a].end(), 0.0);
      }
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
      const AxisBucketBackward* axis_back[kNumCh] = {&back0, &back1, &back2};
      for (uint32_t a = 0; a < kNumCh; ++a) {
        const size_t Tlen = state.thresholds[a].size();
        if (Tlen == 0) continue;
        for (size_t j = 0; j < Tlen; ++j) {
          const double sig_val = axis_back[a]->sigma[j];
          const double sigma_prime = sig_val * (1.0 - sig_val) * inv_thr_t;
          grad->thresholds[a][j] += sigma_prime * (dL_dw_ax[a][j] - dL_dw_ax[a][j + 1]);
        }
      }
    }
  }

  // Softmax Jacobian for cluster logits: per (c, cell), convert
  // `dL/drho[c][cell*K + k]` into `dL/dcluster_logits[c][cell*K + k]` via
  //   dL/dlogit[m] = rho[m] * (dL/drho[m] - sum_k rho[k] * dL/drho[k])
  //                  / cluster_temperature.
  for (uint32_t c = 0; c < d.channels; ++c) {
    SoftmaxJacobianRowsVec(rho_cache[c].data(), dL_drho[c].data(),
                           grad->cluster_logits[c].data(), inv_cluster_t,
                           num_cells, num_clusters);
  }

  return result;
}

void AdamStepImpl(const GradientJointGrad& grad, const AdamConfig& cfg,
                  AdamState* adam, GradientJointState* state) {
  ++adam->step;
  const double beta1 = cfg.beta1;
  const double beta2 = cfg.beta2;
  const double inv_bias1 = 1.0 / (1.0 - std::pow(beta1, adam->step));
  const double inv_bias2 = 1.0 / (1.0 - std::pow(beta2, adam->step));

  auto apply = [&](const std::vector<double>& g, std::vector<double>* param,
                   std::vector<double>* m, std::vector<double>* v) {
    if (g.empty()) return;
    AdamApplyVec(g.data(), param->data(), m->data(), v->data(), g.size(),
                 beta1, beta2, cfg.lr, inv_bias1, inv_bias2, cfg.eps);
  };

  for (uint32_t a = 0; a < kNumCh; ++a) {
    apply(grad.thresholds[a], &state->thresholds[a], &adam->m_thresholds[a],
          &adam->v_thresholds[a]);
    apply(grad.pass_logits[a], &state->pass_logits[a], &adam->m_logits[a],
          &adam->v_logits[a]);
    apply(grad.cluster_logits[a], &state->cluster_logits[a],
          &adam->m_cluster_logits[a], &adam->v_cluster_logits[a]);
  }
  apply(grad.ctx_logits, &state->ctx_logits, &adam->m_ctx_logits,
        &adam->v_ctx_logits);
}

}  // namespace HWY_NAMESPACE
}  // namespace jxl
HWY_AFTER_NAMESPACE();

#if HWY_ONCE

namespace jxl {

HWY_EXPORT(ComputeSoftCostImpl);
HWY_EXPORT(AdamStepImpl);

void InitCtxLogitsRoundRobin(double hard_logit, GradientJointState* state) {
  const size_t H = state->num_hists;
  const size_t K = state->num_clusters;
  const size_t P = state->num_passes;
  constexpr size_t kZDC = kZeroDensityContextCount;
  state->ctx_logits.assign(P * K * kZDC * H, -hard_logit);
  for (size_t p = 0; p < P; ++p) {
    for (size_t k = 0; k < K; ++k) {
      for (size_t zdc = 0; zdc < kZDC; ++zdc) {
        const size_t h = (k * kZDC + zdc) % H;
        state->ctx_logits[(p * K * kZDC + k * kZDC + zdc) * H + h] = hard_logit;
      }
    }
  }
}

void ResetGradientJointGrad(const GradientJointState& state,
                            GradientJointGrad* grad) {
  for (uint32_t a = 0; a < kNumCh; ++a) {
    grad->thresholds[a].assign(state.thresholds[a].size(), 0.0);
    grad->pass_logits[a].assign(state.pass_logits[a].size(), 0.0);
    grad->cluster_logits[a].assign(state.cluster_logits[a].size(), 0.0);
  }
  grad->ctx_logits.assign(state.ctx_logits.size(), 0.0);
}

SoftCostResult ComputeSoftTotalCost(const JPEGOptData& d,
                                    const GradientJointState& state) {
  return HWY_DYNAMIC_DISPATCH(ComputeSoftCostImpl)(d, state, nullptr);
}

SoftCostResult ComputeSoftTotalCostWithGrad(const JPEGOptData& d,
                                            const GradientJointState& state,
                                            GradientJointGrad* grad) {
  return HWY_DYNAMIC_DISPATCH(ComputeSoftCostImpl)(d, state, grad);
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
  adam->m_ctx_logits.assign(state.ctx_logits.size(), 0.0);
  adam->v_ctx_logits.assign(state.ctx_logits.size(), 0.0);
  adam->step = 0;
}

void AdamStep(const GradientJointGrad& grad, const AdamConfig& cfg,
              AdamState* adam, GradientJointState* state) {
  HWY_DYNAMIC_DISPATCH(AdamStepImpl)(grad, cfg, adam, state);
}

void ApplyAnnealing(const AnnealSchedule& schedule, uint32_t step_index,
                    GradientJointState* state) {
  if (step_index < schedule.hot_iters || schedule.anneal_iters == 0) {
    state->pass_temperature = schedule.pass_init;
    state->threshold_temperature = schedule.threshold_init;
    state->cluster_temperature = schedule.cluster_init;
    state->ctx_temperature = schedule.ctx_init;
    return;
  }
  const uint32_t anneal_step = step_index - schedule.hot_iters;
  if (anneal_step >= schedule.anneal_iters) {
    state->pass_temperature = schedule.pass_final;
    state->threshold_temperature = schedule.threshold_final;
    state->cluster_temperature = schedule.cluster_final;
    state->ctx_temperature = schedule.ctx_final;
    return;
  }
  const double t = anneal_step / (schedule.anneal_iters - 1.0);
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
  const double log_ctx = (1.0 - t) * std::log(schedule.ctx_init) +
                         t * std::log(schedule.ctx_final);
  state->pass_temperature = std::exp(log_pass);
  state->threshold_temperature = std::exp(log_thr);
  state->cluster_temperature = std::exp(log_clu);
  state->ctx_temperature = std::exp(log_ctx);
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
                                     GradientJointState* state,
                                     uint32_t fa, uint32_t fb,
                                     uint32_t fc, uint32_t num_passes) {
  OptimizeResult result;
  const uint32_t total_iters = schedule.hot_iters + schedule.anneal_iters;
  if (total_iters == 0) {
    const SoftCostResult r = ComputeSoftTotalCost(d, *state);
    result.init_cost_bits = r.total_cost_bits;
    result.final_cost_bits = r.total_cost_bits;
    return result;
  }

  auto start_solve = PlannerClock::now();

  AdamState adam;
  InitAdamState(*state, &adam);

  GradientJointGrad grad;
  ResetGradientJointGrad(*state, &grad);

  // Initial cost at caller-configured temperatures (schedule not applied yet).
  {
    auto start_init = PlannerClock::now();
    const SoftCostResult r = ComputeSoftTotalCost(d, *state);
    result.init_cost_bits = r.total_cost_bits;
    auto end_init = PlannerClock::now();
    fprintf(stderr,
            "PLANNER: [gradient] [(%u,%u,%u) P=%u] Initial cost: %.2f bits "
            "(took %.2f ms)\n",
            fa, fb, fc, num_passes,
            r.total_cost_bits,
            NanosToMs(ElapsedNanos(start_init, end_init)));
    fflush(stderr);
  }

  int64_t total_fwd_ns = 0;
  int64_t total_adam_ns = 0;
  double prev_cost = result.init_cost_bits;

  // Pre-loop forward+grad: applies anneal-step-0 and computes the gradient
  // that iter 0's AdamStep will consume. Subsequent iters reuse the *next*
  // iter's "after-step" forward as both their gradient computation and the
  // post-step cost report, so the per-iter print reflects the cost AFTER
  // this iter's update. The total number of forward+grad calls stays at
  // `total_iters`; we add only one forward-only call at the very end.
  if (total_iters > 0) {
    ApplyAnnealing(schedule, 0, state);
    ResetGradientJointGrad(*state, &grad);
    auto start_fwd = PlannerClock::now();
    SoftCostResult r = ComputeSoftTotalCostWithGrad(d, *state, &grad);
    auto end_fwd = PlannerClock::now();
    total_fwd_ns += ElapsedNanos(start_fwd, end_fwd);

    for (uint32_t t = 0; t < total_iters; ++t) {
      auto start_iter = PlannerClock::now();

      auto start_adam = PlannerClock::now();
      AdamStep(grad, adam_cfg, &adam, state);
      ProjectThresholdsMonotonic(state);
      auto end_adam = PlannerClock::now();
      total_adam_ns += ElapsedNanos(start_adam, end_adam);

      // Forward at the post-step state. For all but the last iter this
      // doubles as the next iter's gradient computation; for the last iter
      // it is forward-only.
      auto fwd_start = PlannerClock::now();
      if (t + 1 < total_iters) {
        ApplyAnnealing(schedule, t + 1, state);
        ResetGradientJointGrad(*state, &grad);
        r = ComputeSoftTotalCostWithGrad(d, *state, &grad);
      } else {
        r = ComputeSoftTotalCost(d, *state);
      }
      auto fwd_end = PlannerClock::now();
      total_fwd_ns += ElapsedNanos(fwd_start, fwd_end);

      result.final_cost_bits = r.total_cost_bits;
      ++result.iters_taken;

      auto end_iter = PlannerClock::now();
      const bool is_hot = t < schedule.hot_iters;
      const double delta = r.total_cost_bits - prev_cost;
      const double pct = (prev_cost != 0.0) ? (delta / prev_cost) * 100.0 : 0.0;
      fprintf(stderr,
              "PLANNER: [gradient] [(%u,%u,%u) P=%u] Iter %u/%u (%s) "
              "cost=%.2f bits delta=%+.2f (%+.3f%%) "
              "fwd=%.2f ms adam=%.2f ms total=%.2f ms\n",
              fa, fb, fc, num_passes,
              t + 1, total_iters, is_hot ? "hot" : "anneal",
              r.total_cost_bits, delta, pct,
              NanosToMs(ElapsedNanos(fwd_start, fwd_end)),
              NanosToMs(ElapsedNanos(start_adam, end_adam)),
              NanosToMs(ElapsedNanos(start_iter, end_iter)));
      fflush(stderr);
      prev_cost = r.total_cost_bits;
    }
  }

  auto end_solve = PlannerClock::now();
  const double total_delta = result.final_cost_bits - result.init_cost_bits;
  const double total_pct = (result.init_cost_bits != 0.0)
                               ? (total_delta / result.init_cost_bits) * 100.0
                               : 0.0;
  fprintf(stderr,
          "PLANNER: [gradient] [(%u,%u,%u) P=%u] Solve done: %u iters, "
          "init=%.2f -> final=%.2f bits delta=%+.2f (%+.3f%%), "
          "fwd_total=%.2f ms adam_total=%.2f ms wall=%.2f ms\n",
          fa, fb, fc, num_passes,
          total_iters, result.init_cost_bits, result.final_cost_bits,
          total_delta, total_pct,
          NanosToMs(total_fwd_ns), NanosToMs(total_adam_ns),
          NanosToMs(ElapsedNanos(start_solve, end_solve)));
  fflush(stderr);
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
  r.ctx_map.assign(d.channels * num_cells, 0);
  for (uint32_t c = 0; c < d.channels; ++c) {
    const auto& logits = state.cluster_logits[c];
    if (logits.size() != num_cells * num_clusters) {
      continue;
    }
    for (uint32_t cell = 0; cell < num_cells; ++cell) {
      const double* base = &logits[cell * num_clusters];
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
      const int64_t vi64 = std::llround(v);
      const int16_t clamped = static_cast<int16_t>(
          std::min<int64_t>(kMaxDC, std::max<int64_t>(kMinDC, vi64)));
      int16_t val = std::min<int16_t>(
          kMaxDC,
          std::max<int16_t>(clamped, static_cast<int16_t>(prev + 1)));
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
      const double* base = &logits[b * num_passes];
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

// --- Iteration 6: Parallel factorization sweep --------------------------

GradientJointState InitGradientJointStateFromFactorization(
    const JPEGOptData& d, const Factorization& f, uint32_t num_passes,
    uint32_t num_clusters, double threshold_temperature,
    double pass_temperature, double cluster_temperature, uint32_t num_hists) {
  GradientJointState state;
  state.num_passes = num_passes;
  state.num_clusters = num_clusters;
  state.threshold_temperature = threshold_temperature;
  state.pass_temperature = pass_temperature;
  state.cluster_temperature = cluster_temperature;

  // Per-axis thresholds via `InitThresh` (same starting point as the hard
  // search's factorization-to-candidate init).
  for (uint32_t axis = 0; axis < kNumCh; ++axis) {
    const Thresholds T = InitThresh(d, axis, f[axis]);
    state.thresholds[axis].assign(T.begin(), T.end());
  }
  state.num_cells = static_cast<uint32_t>(
      (state.thresholds[0].size() + 1) * (state.thresholds[1].size() + 1) *
      (state.thresholds[2].size() + 1));

  // Pass logits: per-block round-robin bias so block `b` starts biased toward
  // pass `b % num_passes`. The bias is chosen to be noticeable at temperature
  // ~1 but small enough that the optimizer can easily overcome it. Per-block
  // variation here mirrors the per-cell round-robin on cluster logits below;
  // both are needed so the soft state isn't degenerate along any axis.
  constexpr double kPassSymmetryBreak = 0.5;
  for (uint32_t c = 0; c < kNumCh; ++c) {
    const uint32_t nb = d.num_blocks[c];
    state.pass_logits[c].assign(nb * num_passes, 0.0);
    for (uint32_t b = 0; b < nb; ++b) {
      const uint32_t p = b % num_passes;
      state.pass_logits[c][b * num_passes + p] = kPassSymmetryBreak;
    }
    if (c < d.channels) {
      // Cluster logits: per-cell round-robin bias so each cell starts biased
      // toward a different cluster. Without per-cell variation `rho` is the
      // same across cells, which collapses `dL_dcell[cell] = sum_k rho[k]*D[k]`
      // to a constant, and that makes the threshold-axis gradient exactly
      // zero (the cell-difference cancels in the axis decomposition). The
      // round-robin ensures each cell sees a distinct rho profile, breaking
      // the threshold saddle without RNG plumbing.
      constexpr double kClusterSymmetryBreak = 0.5;
      state.cluster_logits[c].assign(state.num_cells * num_clusters, 0.0);
      for (uint32_t cell = 0; cell < state.num_cells; ++cell) {
        const uint32_t k = cell % num_clusters;
        state.cluster_logits[c][cell * num_clusters + k] = kClusterSymmetryBreak;
      }
    } else {
      state.cluster_logits[c].clear();
    }
  }
  state.num_hists = num_hists;
  if (num_hists > 1) {
    constexpr double kCtxSymmetryBreak = 0.5;
    InitCtxLogitsRoundRobin(kCtxSymmetryBreak, &state);
  }
  return state;
}

namespace {

// Resolves `(min_passes, max_passes)` from `effort.optimize_passes_num`:
//   -1  -> force 1 pass (disabled)
//    0  -> sweep `[1, ComputeMaxNumPasses]`
//    K  -> force exactly K passes
std::pair<uint32_t, uint32_t> ResolvePassRange(
    const JPEGOptData& d, const JPEGCtxEffortParams& effort) {
  const uint32_t max_img = ComputeMaxNumPasses(d);
  const uint32_t min_passes =
      effort.optimize_passes_num <= 0
          ? 1
          : static_cast<uint32_t>(effort.optimize_passes_num);
  const uint32_t max_passes =
      effort.optimize_passes_num < 0
          ? 1
          : (effort.optimize_passes_num == 0
                 ? max_img
                 : std::min<uint32_t>(effort.optimize_passes_num, max_img));
  return {min_passes, std::max(min_passes, max_passes)};
}

Status RefreshHardCostWithBiclusterModel(const JPEGOptData& d,
                                         uint32_t proto_budget_per_pass,
                                         PassSearchResult* result) {
  // The gradient search used in production currently runs with kToken420. For
  // other AC models, keep the older pass-aware evaluator as a conservative
  // fallback instead of failing an otherwise valid search path.
  if (d.AC_hist_model != JPEGTranscodeACModel::kToken420) {
    const ActiveRawBins active = BuildActiveRawBins(d);
    std::vector<uint32_t> pass_offsets;
    JXL_ASSIGN_OR_RETURN(
        std::vector<ACEntry> pass_stream,
        BuildPassStream(d, active, result->pass_assignment,
                        result->num_passes, &pass_offsets, /*pool=*/nullptr));
    JXL_ASSIGN_OR_RETURN(
        ModelEvaluation eval,
        EvaluatePassAwareModel(d, result->thresholds, result->ctx_map,
                               result->num_clusters, result->pass_assignment,
                               result->num_passes, pass_stream, pass_offsets));
    result->ac_cost = eval.corrected_entropy_cost >= 0
                          ? eval.corrected_entropy_cost
                          : eval.ac_cost;
    result->nz_cost = eval.nz_cost;
    result->signalling_overhead = eval.signalling_overhead;
    result->total_cost = eval.total_cost();
    return true;
  }

  const uint32_t proto_budget = std::max<uint32_t>(1, proto_budget_per_pass);
  const NZBlockCache nz_cache =
      BuildNZBlockCache(d, result->pass_assignment, result->num_passes);
  JXL_ASSIGN_OR_RETURN(
      RowSliceState row_state,
      BuildRowSliceState(d, result->thresholds, result->pass_assignment,
                         result->num_passes, nz_cache));
  std::vector<uint32_t> num_prototypes_per_pass(result->num_passes, 0);
  JXL_ASSIGN_OR_RETURN(
      ModelEvaluation eval,
      EvaluateBiclusterState(d, result->thresholds, result->ctx_map,
                             result->num_clusters, result->pass_assignment,
                             result->num_passes, proto_budget,
                             row_state.rows, &num_prototypes_per_pass));
  result->ac_cost = eval.corrected_entropy_cost >= 0
                        ? eval.corrected_entropy_cost
                        : eval.ac_cost;
  result->nz_cost = eval.nz_cost;
  result->signalling_overhead = eval.signalling_overhead;
  result->total_cost = eval.total_cost();
  return true;
}

}  // namespace

StatusOr<uint32_t> ReduceClustersAgglomerative(const JPEGOptData& d,
                                               PassSearchResult* result,
                                               ThreadPool* pool) {
  if (result->num_clusters <= 1) return uint32_t{0};
  const uint32_t num_cells =
      static_cast<uint32_t>((result->thresholds.TY().size() + 1) *
                            (result->thresholds.TCb().size() + 1) *
                            (result->thresholds.TCr().size() + 1));
  if (result->ctx_map.size() != d.channels * num_cells) {
    return JXL_FAILURE("ReduceClustersAgglomerative: ctx_map size mismatch");
  }

  // Build the pass-stream once. Pass assignment is fixed across merges, so
  // the stream stays valid for every cost evaluation we do below.
  const ActiveRawBins active = BuildActiveRawBins(d);
  std::vector<uint32_t> pass_offsets;
  JXL_ASSIGN_OR_RETURN(
      std::vector<ACEntry> pass_stream,
      BuildPassStream(d, active, result->pass_assignment, result->num_passes,
                      &pass_offsets, pool));

  // Initial cost (uses the same evaluator the encoder will).
  JXL_ASSIGN_OR_RETURN(
      ModelEvaluation current_eval,
      EvaluatePassAwareModel(d, result->thresholds, result->ctx_map,
                             result->num_clusters, result->pass_assignment,
                             result->num_passes, pass_stream, pass_offsets));
  FixedPointCost current_cost = current_eval.total_cost();
  uint32_t current_K = result->num_clusters;
  ContextMap working_ctx = result->ctx_map;

  uint32_t merges = 0;
  std::vector<uint8_t> remap(current_K, 0);
  ContextMap trial_ctx(working_ctx.size(), 0);

  while (current_K > 1) {
    int best_i = -1;
    int best_j = -1;
    FixedPointCost best_cost = current_cost;
    ModelEvaluation best_eval = current_eval;

    for (uint32_t i = 0; i + 1 < current_K; ++i) {
      for (uint32_t j = i + 1; j < current_K; ++j) {
        // Build a remap that collapses j into i and shifts higher ids down.
        for (uint32_t k = 0; k < current_K; ++k) {
          if (k == j) {
            remap[k] = static_cast<uint8_t>(i);
          } else if (k > j) {
            remap[k] = static_cast<uint8_t>(k - 1);
          } else {
            remap[k] = static_cast<uint8_t>(k);
          }
        }
        for (size_t e = 0; e < working_ctx.size(); ++e) {
          trial_ctx[e] = remap[working_ctx[e]];
        }
        const uint32_t trial_K = current_K - 1;
        JXL_ASSIGN_OR_RETURN(
            ModelEvaluation eval,
            EvaluatePassAwareModel(d, result->thresholds, trial_ctx, trial_K,
                                   result->pass_assignment, result->num_passes,
                                   pass_stream, pass_offsets));
        const FixedPointCost trial_cost = eval.total_cost();
        if (trial_cost < best_cost) {
          best_cost = trial_cost;
          best_eval = eval;
          best_i = static_cast<int>(i);
          best_j = static_cast<int>(j);
        }
      }
    }

    if (best_i < 0) break;  // no improving merge

    // Apply best merge permanently.
    for (uint32_t k = 0; k < current_K; ++k) {
      if (k == static_cast<uint32_t>(best_j)) {
        remap[k] = static_cast<uint8_t>(best_i);
      } else if (k > static_cast<uint32_t>(best_j)) {
        remap[k] = static_cast<uint8_t>(k - 1);
      } else {
        remap[k] = static_cast<uint8_t>(k);
      }
    }
    for (size_t e = 0; e < working_ctx.size(); ++e) {
      working_ctx[e] = remap[working_ctx[e]];
    }
    --current_K;
    remap.resize(current_K);
    current_cost = best_cost;
    current_eval = best_eval;
    ++merges;
  }

  if (merges > 0) {
    result->ctx_map = std::move(working_ctx);
    result->num_clusters = current_K;
    result->ac_cost = current_eval.ac_cost;
    result->nz_cost = current_eval.nz_cost;
    result->signalling_overhead = current_eval.signalling_overhead;
    result->total_cost = current_eval.total_cost();
  }
  return merges;
}

uint32_t PruneRedundantThresholds(const JPEGOptData& d,
                                  PassSearchResult* result) {
  const uint32_t channels = d.channels;
  uint32_t pruned_total = 0;

  // Helper: produce current per-axis bucket count `n[a] = T[a].size() + 1`.
  auto compute_n = [&]() {
    std::array<uint32_t, kNumCh> n{};
    for (uint32_t a = 0; a < kNumCh; ++a) {
      n[a] = static_cast<uint32_t>(result->thresholds.T[a].size()) + 1;
    }
    return n;
  };

  for (uint32_t axis = 0; axis < kNumCh; ++axis) {
    Thresholds& T = result->thresholds.T[axis];

    // Walk thresholds high-to-low so dropping one doesn't shift the indices we
    // still need to examine.
    for (int j = static_cast<int>(T.size()) - 1; j >= 0; --j) {
      const auto n = compute_n();
      const uint32_t num_cells = n[0] * n[1] * n[2];

      auto cell_at = [&](uint32_t k0, uint32_t k1, uint32_t k2) {
        return (k1 * n[2] + k2) * n[0] + k0;
      };

      // Threshold T[axis][j] separates bucket j and bucket j+1 on this axis.
      // It's redundant iff every (channel, perpendicular cell) sees the same
      // cluster id at bucket j and bucket j+1.
      bool redundant = true;
      for (uint32_t c = 0; c < channels && redundant; ++c) {
        if (axis == 0) {
          for (uint32_t k1 = 0; k1 < n[1] && redundant; ++k1) {
            for (uint32_t k2 = 0; k2 < n[2] && redundant; ++k2) {
              const uint8_t a_id =
                  result->ctx_map[c * num_cells +
                                  cell_at(static_cast<uint32_t>(j), k1, k2)];
              const uint8_t b_id = result->ctx_map[c * num_cells +
                                                   cell_at(j + 1, k1, k2)];
              if (a_id != b_id) redundant = false;
            }
          }
        } else if (axis == 1) {
          for (uint32_t k0 = 0; k0 < n[0] && redundant; ++k0) {
            for (uint32_t k2 = 0; k2 < n[2] && redundant; ++k2) {
              const uint8_t a_id =
                  result->ctx_map[c * num_cells +
                                  cell_at(k0, static_cast<uint32_t>(j), k2)];
              const uint8_t b_id =
                  result->ctx_map[c * num_cells + cell_at(k0, j + 1, k2)];
              if (a_id != b_id) redundant = false;
            }
          }
        } else {  // axis == 2
          for (uint32_t k0 = 0; k0 < n[0] && redundant; ++k0) {
            for (uint32_t k1 = 0; k1 < n[1] && redundant; ++k1) {
              const uint8_t a_id =
                  result->ctx_map[c * num_cells +
                                  cell_at(k0, k1, static_cast<uint32_t>(j))];
              const uint8_t b_id =
                  result->ctx_map[c * num_cells + cell_at(k0, k1, j + 1)];
              if (a_id != b_id) redundant = false;
            }
          }
        }
      }
      if (!redundant) continue;

      // Drop threshold j on `axis`. Bucket j and j+1 collapse into bucket j;
      // higher buckets shift down by one. The map from new bucket back to a
      // representative old bucket is `k_new <= j ? k_new : k_new + 1`.
      auto new_n = n;
      new_n[axis] -= 1;
      const uint32_t new_num_cells = new_n[0] * new_n[1] * new_n[2];
      ContextMap new_ctx(channels * new_num_cells);

      auto new_cell_at = [&](uint32_t k0, uint32_t k1, uint32_t k2) {
        return (k1 * new_n[2] + k2) * new_n[0] + k0;
      };
      auto remap = [j](uint32_t k_new) -> uint32_t {
        return k_new <= static_cast<uint32_t>(j) ? k_new : k_new + 1;
      };
      for (uint32_t c = 0; c < channels; ++c) {
        for (uint32_t k0 = 0; k0 < new_n[0]; ++k0) {
          for (uint32_t k1 = 0; k1 < new_n[1]; ++k1) {
            for (uint32_t k2 = 0; k2 < new_n[2]; ++k2) {
              const uint32_t k0_old = (axis == 0) ? remap(k0) : k0;
              const uint32_t k1_old = (axis == 1) ? remap(k1) : k1;
              const uint32_t k2_old = (axis == 2) ? remap(k2) : k2;
              new_ctx[c * new_num_cells + new_cell_at(k0, k1, k2)] =
                  result->ctx_map[c * num_cells +
                                  cell_at(k0_old, k1_old, k2_old)];
            }
          }
        }
      }
      result->ctx_map = std::move(new_ctx);
      T.erase(T.begin() + j);
      ++pruned_total;
    }
  }

  return pruned_total;
}

StatusOr<PassSearchResult> SearchGradientJointContextModel(
    std::shared_ptr<const JPEGOptData> opt_data,
    const JPEGCtxEffortParams& effort, ThreadPool* pool,
    std::vector<GradientSearchCandidate>* debug_candidates) {
  auto start_total = PlannerClock::now();

  const JPEGOptData& d = *opt_data;
  const auto factorizations = MaximalFactorizations(d);
  if (factorizations.empty()) {
    return JXL_FAILURE("Gradient-joint search: no maximal factorizations");
  }

  const uint32_t num_clusters =
      kMaxClusters - static_cast<uint32_t>(d.channels == 1);
  const auto pass_range = ResolvePassRange(d, effort);
  const uint32_t min_passes = pass_range.first;
  const uint32_t max_passes = pass_range.second;
  const uint32_t pass_count_steps = max_passes - min_passes + 1;
  const uint32_t num_factorizations =
      static_cast<uint32_t>(factorizations.size());

  fprintf(stderr,
          "PLANNER: [gradient] %u factorizations, pass range [%u, %u] "
          "(%u workers), %u clusters\n",
          num_factorizations, min_passes, max_passes,
          num_factorizations * pass_count_steps, num_clusters);
  fflush(stderr);

  AdamConfig adam_cfg;
  adam_cfg.lr = effort.grad_lr;

  AnnealSchedule sched;
  sched.hot_iters = effort.grad_hot_iters;
  sched.anneal_iters = effort.grad_anneal_iters;
  sched.pass_init = effort.grad_init_temperature;
  sched.pass_final = effort.grad_init_temperature * 0.05;
  sched.threshold_init = effort.grad_init_temperature * 50.0;
  sched.threshold_final = effort.grad_init_temperature * 0.5;
  sched.cluster_init = effort.grad_init_temperature;
  sched.cluster_final = effort.grad_init_temperature * 0.05;
  sched.ctx_init = effort.grad_init_temperature;
  sched.ctx_final = effort.grad_init_temperature * 0.05;

  const uint32_t num_hists = effort.grad_num_hists;

  fprintf(stderr,
          "PLANNER: [gradient] Schedule: hot=%u anneal=%u total_iters=%u "
          "lr=%.4f T_pass=%.4f->%.4f T_thresh=%.4f->%.4f "
          "T_cluster=%.4f->%.4f T_ctx=%.4f->%.4f num_hists=%u\n",
          sched.hot_iters, sched.anneal_iters,
          sched.hot_iters + sched.anneal_iters, adam_cfg.lr, sched.pass_init,
          sched.pass_final, sched.threshold_init, sched.threshold_final,
          sched.cluster_init, sched.cluster_final, sched.ctx_init,
          sched.ctx_final, num_hists);
  fflush(stderr);

  // Flat work list of `(factorization_idx, num_passes)` tuples. Each worker
  // writes its own slot; no shared mutation across threads.
  struct Slot {
    PassSearchResult result;
    double final_cost_bits = std::numeric_limits<double>::max();
    uint32_t factorization_idx = 0;
    uint32_t num_passes = 0;
    bool valid = false;
  };
  const uint32_t total_workers = num_factorizations * pass_count_steps;
  std::vector<Slot> slots(total_workers);

  auto start_sweep = PlannerClock::now();
  JXL_RETURN_IF_ERROR(RunOnPool(
      pool, 0, total_workers, ThreadPool::NoInit,
      [&](uint32_t idx, size_t /*thread_id*/) -> Status {
        const uint32_t factorization_idx = idx / pass_count_steps;
        const uint32_t num_passes =
            min_passes + (idx % pass_count_steps);
        const Factorization& f = factorizations[factorization_idx];
        GradientJointState state = InitGradientJointStateFromFactorization(
            d, f, num_passes, num_clusters, sched.threshold_init,
            sched.pass_init, sched.cluster_init, num_hists);
        RunGradientJointSolve(d, adam_cfg, sched, &state,
                              f[0], f[1], f[2], num_passes);
        slots[idx].result = RoundToHardAssignment(d, state);
        slots[idx].factorization_idx = factorization_idx;
        slots[idx].num_passes = num_passes;
        slots[idx].valid = true;

        // Post-hoc agglomerative cluster reduction (Option C). Adam's
        // gradient ignores signalling overhead, so it tends to keep all 16
        // clusters even when overhead would be saved by merging. This pass
        // greedily merges cluster pairs whose merge reduces the encoder's
        // actual cost (entropy + overhead) and updates the slot's cost
        // fields and tie-break key.
        if (effort.grad_overhead_aware_reduce) {
          auto merges_or = ReduceClustersAgglomerative(
              d, &slots[idx].result, /*pool=*/nullptr);
          if (merges_or.ok()) {
            const uint32_t merges = std::move(merges_or).value_();
            if (merges > 0) {
              // Use the EvaluatePassAwareModel-derived total cost for the
              // pick-best comparison; soft cost is now stale.
              slots[idx].final_cost_bits =
                  static_cast<double>(slots[idx].result.total_cost) /
                  static_cast<double>(kFScale);
              fprintf(stderr,
                      "PLANNER: [gradient] [(%u,%u,%u) P=%u] "
                      "Agglomerative merge: %u clusters dropped, "
                      "final_cost=%.2f bits\n",
                      f[0], f[1], f[2], num_passes, merges,
                      slots[idx].final_cost_bits);
              fflush(stderr);
            }
          }
          // Prune thresholds whose adjacent buckets all share the same
          // cluster (post-merge, often most of them). This shrinks the
          // bitstream factorization metadata and ctx_map size without
          // changing per-block cluster assignment, so EvaluatePassAwareModel
          // cost is unchanged but the actual bitstream is smaller.
          const uint32_t pruned =
              PruneRedundantThresholds(d, &slots[idx].result);
          if (pruned > 0) {
            const auto& T0 = slots[idx].result.thresholds.TY();
            const auto& T1 = slots[idx].result.thresholds.TCb();
            const auto& T2 = slots[idx].result.thresholds.TCr();
            fprintf(stderr,
                    "PLANNER: [gradient] [(%u,%u,%u) P=%u] "
                    "Threshold pruning: %u redundant thresholds dropped, "
                    "factorization now (%zu,%zu,%zu)\n",
                    f[0], f[1], f[2], num_passes, pruned, T0.size() + 1,
                    T1.size() + 1, T2.size() + 1);
            fflush(stderr);
          }
        }
        JXL_RETURN_IF_ERROR(RefreshHardCostWithBiclusterModel(
            d, effort.bicluster_proto_budget_per_pass, &slots[idx].result));
        slots[idx].final_cost_bits = bit_cost(slots[idx].result.total_cost);
        return true;
      },
      "JpegCtxGradSweep"));
  auto end_sweep = PlannerClock::now();

  // Pick the best. Deterministic tie-break: smaller flat index wins (which
  // corresponds to smaller factorization_idx first, then smaller num_passes).
  size_t best_idx = total_workers;
  double best_cost = std::numeric_limits<double>::max();
  for (size_t i = 0; i < slots.size(); ++i) {
    if (!slots[i].valid) continue;
    if (slots[i].final_cost_bits < best_cost) {
      best_cost = slots[i].final_cost_bits;
      best_idx = i;
    }
  }
  if (best_idx >= slots.size()) {
    return JXL_FAILURE("Gradient-joint search: no factorization succeeded");
  }

  if (debug_candidates != nullptr) {
    debug_candidates->clear();
    debug_candidates->reserve(slots.size());
    for (size_t i = 0; i < slots.size(); ++i) {
      if (!slots[i].valid) continue;
      const Factorization& sf = factorizations[slots[i].factorization_idx];
      GradientSearchCandidate candidate;
      candidate.result = slots[i].result;
      candidate.target_cost_bits = slots[i].final_cost_bits;
      candidate.factorization[0] = sf[0];
      candidate.factorization[1] = sf[1];
      candidate.factorization[2] = sf[2];
      candidate.num_passes = slots[i].num_passes;
      candidate.is_best = i == best_idx;
      debug_candidates->push_back(std::move(candidate));
    }
  }

  // Report per-slot results.
  for (size_t i = 0; i < slots.size(); ++i) {
    if (!slots[i].valid) continue;
    const Factorization& sf = factorizations[slots[i].factorization_idx];
    fprintf(stderr,
            "PLANNER: [gradient] [(%u,%u,%u) P=%u] cost=%.4f bits%s\n",
            sf[0], sf[1], sf[2], slots[i].num_passes,
            slots[i].final_cost_bits,
            i == best_idx ? " ** BEST **" : "");
  }
  fflush(stderr);

  const Slot& best = slots[best_idx];
  const Factorization& bf = factorizations[best.factorization_idx];
  fprintf(stderr,
          "PLANNER: [gradient] Sweep done: %u workers in %.2f ms, "
          "best=[(%u,%u,%u) P=%u] cost=%.4f bits "
          "(ac=%.2f nz=%.2f overhead=%.2f)\n",
          total_workers,
          NanosToMs(ElapsedNanos(start_sweep, end_sweep)),
          bf[0], bf[1], bf[2], best.num_passes,
          best.final_cost_bits,
          bit_cost(best.result.ac_cost),
          bit_cost(best.result.nz_cost),
          bit_cost(best.result.signalling_overhead));
  auto end_total = PlannerClock::now();
  fprintf(stderr, "PLANNER: [gradient] Total search took %.2f ms\n",
          NanosToMs(ElapsedNanos(start_total, end_total)));
  fflush(stderr);

  return std::move(slots[best_idx].result);
}

}  // namespace jxl

#endif  // HWY_ONCE
