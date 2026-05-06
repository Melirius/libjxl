// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Shared Highway helpers for the JPEG gradient optimizer.
//
// Intentionally no include guard: this file is included inside
// `namespace jxl::HWY_NAMESPACE` from target-specific .cc files.

// Experimental double-lane analogue of `FastLog2f`: same exponent/mantissa
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

  const auto p = hn::MulAdd(hn::MulAdd(hn::Set(d, 7.4245873327820566E-01), xm1,
                                       hn::Set(d, 1.4287160470083755E+00)),
                            xm1, hn::Set(d, -1.8503833400518310E-06));
  const auto q = hn::MulAdd(hn::MulAdd(hn::Set(d, 1.7409343003366853E-01), xm1,
                                       hn::Set(d, 1.0096718572241148E+00)),
                            xm1, hn::Set(d, 9.9032814277590719E-01));
  return hn::Add(hn::Div(p, q), hn::ConvertTo(d, exp_shifted));
}

// Derivative of `x * FastLog2d(x)`, ignoring the piecewise-constant exponent
// term's jump discontinuities. This keeps the analytic gradient consistent
// with `SoftFTabReduceVecFast`'s approximate forward objective.
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

  const auto p = hn::MulAdd(hn::MulAdd(hn::Set(d, 7.4245873327820566E-01), xm1,
                                       hn::Set(d, 1.4287160470083755E+00)),
                            xm1, hn::Set(d, -1.8503833400518310E-06));
  const auto q = hn::MulAdd(hn::MulAdd(hn::Set(d, 1.7409343003366853E-01), xm1,
                                       hn::Set(d, 1.0096718572241148E+00)),
                            xm1, hn::Set(d, 9.9032814277590719E-01));
  const auto fast_log2 = hn::Add(hn::Div(p, q), hn::ConvertTo(d, exp_shifted));

  const auto p_prime = hn::MulAdd(hn::Set(d, 1.4849174665564113E+00), xm1,
                                  hn::Set(d, 1.4287160470083755E+00));
  const auto q_prime = hn::MulAdd(hn::Set(d, 3.4818686006733706E-01), xm1,
                                  hn::Set(d, 1.0096718572241148E+00));
  const auto ratio_prime =
      hn::Div(hn::Sub(hn::Mul(p_prime, q), hn::Mul(p, q_prime)), hn::Mul(q, q));
  return hn::Add(fast_log2, hn::Mul(mantissa, ratio_prime));
}

// Returns `sum_{i: data[i]>0} data[i]*log2(data[i])`.
// `ShannonEntropy()`, but for `double` inputs.
HWY_INLINE double SoftFTabReduceVec(const double* HWY_RESTRICT data, size_t n) {
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
HWY_INLINE double SoftFTabReduceVecFast(const double* HWY_RESTRICT data,
                                        size_t n) {
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

// Experimental unchecked variant for nonnegative histogram/count data.
// Keeps the zeroing mask for the derivative, but differentiates the same
// FastLog2d approximation used by SoftFTabReduceVecFast.
HWY_INLINE void SoftFTabPrimeVecFast(const double* HWY_RESTRICT src,
                                     double* HWY_RESTRICT dst, size_t n) {
  const hn::ScalableTag<double> d;
  const size_t N = hn::Lanes(d);
  const auto veps = hn::Set(d, 1e-18);
  size_t i = 0;
  for (; i + N <= n; i += N) {
    const auto v = hn::LoadU(d, src + i);
    const auto mask = hn::Gt(v, veps);
    const auto y = FastLog2dFTabPrime(d, v);
    hn::StoreU(hn::IfThenElseZero(mask, y), d, dst + i);
  }
  const HWY_CAPPED(double, 1) d1;
  for (; i < n; ++i) {
    if (src[i] > 1e-18) {
      const auto v = hn::Set(d1, src[i]);
      dst[i] = hn::GetLane(FastLog2dFTabPrime(d1, v));
    } else {
      dst[i] = 0.0;
    }
  }
}

HWY_INLINE void SoftFTabPrimeNegVecFast(const double* HWY_RESTRICT src,
                                        double* HWY_RESTRICT dst, size_t n) {
  const hn::ScalableTag<double> d;
  const size_t N = hn::Lanes(d);
  const auto veps = hn::Set(d, 1e-18);
  size_t i = 0;
  for (; i + N <= n; i += N) {
    const auto v = hn::LoadU(d, src + i);
    const auto mask = hn::Gt(v, veps);
    const auto y = hn::Neg(FastLog2dFTabPrime(d, v));
    hn::StoreU(hn::IfThenElseZero(mask, y), d, dst + i);
  }
  const HWY_CAPPED(double, 1) d1;
  for (; i < n; ++i) {
    if (src[i] > 1e-18) {
      const auto v = hn::Set(d1, src[i]);
      dst[i] = -hn::GetLane(FastLog2dFTabPrime(d1, v));
    } else {
      dst[i] = 0.0;
    }
  }
}

// `dst[k] += src[k]` for `k` in `[0, n)`.
HWY_INLINE void VecAddVec(double* HWY_RESTRICT dst,
                          const double* HWY_RESTRICT src, size_t n) {
  const hn::ScalableTag<double> d;
  const size_t N = hn::Lanes(d);
  size_t k = 0;
  for (; k + N <= n; k += N) {
    hn::StoreU(hn::Add(hn::LoadU(d, dst + k), hn::LoadU(d, src + k)), d,
               dst + k);
  }
  for (; k < n; ++k) dst[k] += src[k];
}

// `dst[k] += w * src[k]` for `k` in `[0, n)`.
HWY_INLINE void AccumScaledVec(double* HWY_RESTRICT dst, double w,
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

// `B[k] = sum_cell cell_weight[cell] * rho[cell, k]`.
HWY_INLINE void ClusterWeightsVec(const double* HWY_RESTRICT cell_weight,
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
      hn::StoreU(
          hn::MulAdd(vw, hn::LoadU(d, rho_cell + k), hn::LoadU(d, B + k)), d,
          B + k);
    }
    for (; k < num_clusters; ++k) B[k] += w * rho_cell[k];
  }
}

// For each cell:
//   `dcell[cell] += dot(rho[cell, :], D)`
//   `drho[cell, k] += cell_weight[cell] * D[k]`
HWY_INLINE void CellGradientVec(const double* HWY_RESTRICT cell_weight,
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
        vsum =
            hn::MulAdd(hn::LoadU(d, rho_cell + k), hn::LoadU(d, D + k), vsum);
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
//   `dst[row, m] += scale * prob[row, m] *`
//                  `(dprob[row, m] - dot(prob[row, :], dprob[row, :]))`.
HWY_INLINE void SoftmaxJacobianRowsVec(const double* HWY_RESTRICT prob,
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
      vsum =
          hn::MulAdd(hn::LoadU(d, p_row + k), hn::LoadU(d, dp_row + k), vsum);
    }
    double s = hn::ReduceSum(d, vsum);
    for (; k < row_size; ++k) s += p_row[k] * dp_row[k];

    const auto vs = hn::Set(d, s);
    for (k = 0; k + N <= row_size; k += N) {
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

// `ac_h` layout: `[di * cp_count + cp]` where `cp_count = num_clusters *
// num_passes`. Looping `di` outer keeps each `v_row` sequential in memory
// (L1-resident).
HWY_INLINE void ContextForwardVec(
    const double* HWY_RESTRICT ac_h, const double* HWY_RESTRICT sigma,
    const uint16_t* HWY_RESTRICT dense_to_zdc,
    const uint16_t* HWY_RESTRICT dense_to_token, double* HWY_RESTRICT ctx_h,
    double* HWY_RESTRICT ctx_N, size_t num_clusters, size_t num_passes,
    size_t ac_alpha, size_t zdc_count, size_t token_count, size_t num_hists) {
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
          hn::StoreU(
              hn::MulAdd(vv, hn::LoadU(d, sig + h), hn::LoadU(d, dst + h)), d,
              dst + h);
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
// `dL_dh` output layout: `[cp * ac_alpha + di]` (original, consumed by block
// backward).
HWY_INLINE void ContextBackwardVec(
    const double* HWY_RESTRICT ac_h, const double* HWY_RESTRICT ac_N,
    const double* HWY_RESTRICT sigma, const double* HWY_RESTRICT dctx_h,
    const double* HWY_RESTRICT dctx_N,
    const uint16_t* HWY_RESTRICT dense_to_zdc,
    const uint16_t* HWY_RESTRICT dense_to_token, double* HWY_RESTRICT dL_dh,
    double* HWY_RESTRICT dL_dN, double* HWY_RESTRICT dL_dsigma,
    size_t num_clusters, size_t num_passes, size_t ac_alpha, size_t zdc_count,
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
          vsum = hn::MulAdd(hn::LoadU(d, sig + h), hn::LoadU(d, src + h), vsum);
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
          vsum = hn::MulAdd(hn::LoadU(d, sig + h), hn::LoadU(d, dctx_N_p + h),
                            vsum);
        }
        double sum = hn::ReduceSum(d, vsum);
        for (; h < num_hists; ++h) sum += sig[h] * dctx_N_p[h];
        dN_cp[zdc] = sum;
      }
    }
  }

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
        double* HWY_RESTRICT dst = dL_dsigma + p * sigma_pk_stride +
                                   k * sigma_k_stride + zdc * num_hists;
        const auto vv = hn::Set(d, v);
        size_t h = 0;
        for (; h + N <= num_hists; h += N) {
          hn::StoreU(
              hn::MulAdd(vv, hn::LoadU(d, src + h), hn::LoadU(d, dst + h)), d,
              dst + h);
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
        double* HWY_RESTRICT dst = dL_dsigma + p * sigma_pk_stride +
                                   k * sigma_k_stride + zdc * num_hists;
        const auto vv = hn::Set(d, v);
        size_t h = 0;
        for (; h + N <= num_hists; h += N) {
          hn::StoreU(
              hn::MulAdd(vv, hn::LoadU(d, src + h), hn::LoadU(d, dst + h)), d,
              dst + h);
        }
        for (; h < num_hists; ++h) dst[h] += v * src[h];
      }
    }
  }
}

// Computes softmax of `P` logits with the given temperature, writing `P`
// probabilities into `out`.
HWY_INLINE void Softmax(const double* HWY_RESTRICT logits, uint32_t P,
                        double temperature, double* HWY_RESTRICT out) {
  const double inv_t = 1.0 / temperature;
  // Use the cheaper unshifted exp only while the largest exponent argument is
  // comfortably finite. Low annealed temperatures can otherwise overflow or
  // underflow the whole row, so we fall back to the row-max shift.
  // Keep tiny rows on std::exp; the hot context-map rows have H=128.
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
