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
#define HWY_TARGET_INCLUDE "lib/jxl/transcode_jpeg/enc_jpeg_grad_manypass.cc"
// clang-format off
#include <hwy/foreach_target.h>
#include <hwy/highway.h>
#include <hwy/contrib/math/math-inl.h>
// clang-format on

#include "lib/jxl/ac_context.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"

HWY_BEFORE_NAMESPACE();
namespace jxl {
namespace HWY_NAMESPACE {

namespace hn = hwy::HWY_NAMESPACE;

#include "lib/jxl/transcode_jpeg/enc_jpeg_grad_simd-inl.h"

namespace {

// Computes block-major pass softmax rows `[b * P + p]`. Pass counts are small,
// so vectorize across blocks: each SIMD lane owns one block.
void PassSoftmaxCacheVec(const double* HWY_RESTRICT logits,
                         const double* HWY_RESTRICT gates, size_t num_blocks,
                         uint32_t P, double temperature,
                         double* HWY_RESTRICT out) {
  constexpr size_t kMaxPasses = 16;
  if (num_blocks == 0 || P == 0) return;

  const double inv_t = 1.0 / temperature;
  if (P == 1) {
    std::fill(out, out + num_blocks, 1.0);
    return;
  }
  if (P > kMaxPasses) {
    std::vector<double> row(P);
    for (size_t b = 0; b < num_blocks; ++b) {
      const double* base = &logits[b * P];
      if (gates == nullptr) {
        Softmax(base, P, temperature, &out[b * P]);
      } else {
        for (uint32_t p = 0; p < P; ++p) row[p] = base[p] + gates[p];
        GradientSoftmaxScalar(row.data(), P, inv_t, &out[b * P]);
      }
    }
    return;
  }

  const hn::ScalableTag<double> d;
  const hn::RebindToSigned<decltype(d)> di;
  using V = decltype(hn::Zero(d));
  const size_t N = hn::Lanes(d);
  const auto vinv_t = hn::Set(d, inv_t);
  const auto vone = hn::Set(d, 1.0);
  const auto pass_stride = hn::Set(di, static_cast<int64_t>(P));
  const auto block_offsets = hn::Mul(hn::Iota(di, 0), pass_stride);
  std::array<V, kMaxPasses> probs;

  size_t b = 0;
  for (; b + N <= num_blocks; b += N) {
    const double* HWY_RESTRICT logits_base = logits + b * P;
    double* HWY_RESTRICT out_base = out + b * P;
    const auto p0 = hn::Add(hn::GatherIndex(d, logits_base, block_offsets),
                            hn::Set(d, gates == nullptr ? 0.0 : gates[0]));
    auto vmax = p0;
    probs[0] = hn::Exp(d, hn::Mul(p0, vinv_t));
    auto vsum = probs[0];
    for (uint32_t p = 1; p < P; ++p) {
      const auto pass_offsets =
          hn::Add(block_offsets, hn::Set(di, static_cast<int64_t>(p)));
      const auto logits_p =
          hn::Add(hn::GatherIndex(d, logits_base, pass_offsets),
                  hn::Set(d, gates == nullptr ? 0.0 : gates[p]));
      vmax = hn::Max(vmax, logits_p);
      probs[p] = hn::Exp(d, hn::Mul(logits_p, vinv_t));
      vsum = hn::Add(vsum, probs[p]);
    }

    // Fast path keeps the unshifted probabilities already computed above.
    // If any lane's largest exponent would be unsafe, overwrite them
    // with the shifted form for the whole SIMD chunk.
    const bool use_unshifted =
        hn::AllTrue(d, hn::Lt(hn::Abs(hn::Mul(vmax, vinv_t)),
                              hn::Set(d, kSoftmaxUnshiftedMaxArg)));
    if (!use_unshifted) {
      vsum = hn::Zero(d);
      for (uint32_t p = 0; p < P; ++p) {
        const auto pass_offsets =
            hn::Add(block_offsets, hn::Set(di, static_cast<int64_t>(p)));
        const auto logits_p =
            hn::Add(hn::GatherIndex(d, logits_base, pass_offsets),
                    hn::Set(d, gates == nullptr ? 0.0 : gates[p]));
        probs[p] = hn::Exp(d, hn::Mul(hn::Sub(logits_p, vmax), vinv_t));
        vsum = hn::Add(vsum, probs[p]);
      }
    }

    const auto vinv_sum = hn::Div(vone, vsum);
    for (uint32_t p = 0; p < P; ++p) {
      const auto pass_offsets =
          hn::Add(block_offsets, hn::Set(di, static_cast<int64_t>(p)));
      hn::ScatterIndex(hn::Mul(probs[p], vinv_sum), d, out_base, pass_offsets);
    }
  }

  for (; b < num_blocks; ++b) {
    const double* base = &logits[b * P];
    if (gates == nullptr) {
      GradientSoftmaxScalar(base, P, inv_t, &out[b * P]);
    } else {
      std::array<double, kMaxPasses> row;
      for (uint32_t p = 0; p < P; ++p) row[p] = base[p] + gates[p];
      GradientSoftmaxScalar(row.data(), P, inv_t, &out[b * P]);
    }
  }
}

// Derives the integer predictor bucket `pb` from a fractional `predicted_nz`.
// Mirrors the hard formula in `EvaluatePassAwareModel` exactly once the
// fractional value is floored.
inline uint32_t PredictorBucketFromPredictedNZ(double predicted_nz) {
  const double nz_int = std::floor(predicted_nz);
  uint32_t pb = (nz_int < 8) ? nz_int : (4 + nz_int / 2);
  return std::min(pb, kJPEGNonZeroBuckets - 1);
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

inline double SmoothAlive(double mass, double tau) {
  if (mass <= 0.0) return 0.0;
  return -std::expm1(-mass / tau);
}

inline double SmoothAlivePrime(double mass, double tau) {
  if (mass <= 0.0) return 1.0 / tau;
  return std::exp(-mass / tau) / tau;
}

inline size_t TotalChannelBlocks(const JPEGOptData& d) {
  size_t total = 0;
  for (uint32_t c = 0; c < d.channels; ++c) total += d.num_blocks[c];
  return total;
}

inline double SmoothPassOccupancyOverheadBits(
    const std::vector<double>& pass_mass,
    const std::vector<double>& pass_group_mass, uint32_t num_passes,
    uint32_t pass_group_count, double tau_pass, double tau_group,
    std::vector<double>* pass_mass_grad,
    std::vector<double>* pass_group_mass_grad) {
  constexpr double kPassHeaderBits = 64000.0;
  constexpr double kPassGroupBits = 64.0;
  double bits = 0.0;
  if (pass_mass_grad != nullptr) {
    pass_mass_grad->resize(num_passes);
    pass_group_mass_grad->resize(num_passes * pass_group_count);
  }
  for (uint32_t p = 0; p < num_passes; ++p) {
    const double mass = pass_mass[p];
    bits += kPassHeaderBits * SmoothAlive(mass, tau_pass);
    if (pass_mass_grad != nullptr) {
      (*pass_mass_grad)[p] = kPassHeaderBits * SmoothAlivePrime(mass, tau_pass);
    }
    for (uint32_t g = 0; g < pass_group_count; ++g) {
      const size_t idx = p * pass_group_count + g;
      const double group_mass = pass_group_mass[idx];
      bits += kPassGroupBits * SmoothAlive(group_mass, tau_group);
      if (pass_group_mass_grad != nullptr) {
        (*pass_group_mass_grad)[idx] =
            kPassGroupBits * SmoothAlivePrime(group_mass, tau_group);
      }
    }
  }
  return bits;
}
}  // namespace

// Forward + optional backward. Shared body for all public entry points.
// Computes AC + NZ + signalling overhead.
// Iteration 5 moved `ctx_map` and `num_clusters` into `state` and added soft
// cluster membership via `state.cluster_logits`.
SoftCostResult SoftForwardBackwardManyPassImpl(const JPEGOptData& d,
                                               const GradientAux& aux,
                                               const GradientState& state,
                                               GradientGrad* grad,
                                               GradientScratch* scratch) {
  SoftCostResult result;
  JXL_DASSERT(scratch != nullptr);
  GradientScratch& work = *scratch;
  const size_t num_passes = state.num_passes;
  const size_t num_clusters = state.num_clusters;
  JXL_DASSERT(num_clusters > 0);
  JXL_DASSERT(num_passes > 0);

  const std::array<uint32_t, kNumCh> n_axis = {
      static_cast<uint32_t>(state.thresholds[0].size()) + 1,
      static_cast<uint32_t>(state.thresholds[1].size()) + 1,
      static_cast<uint32_t>(state.thresholds[2].size()) + 1};
  const size_t num_cells = n_axis[0] * n_axis[1] * n_axis[2];
  JXL_DASSERT(state.num_cells == num_cells);
  for (uint32_t c = 0; c < d.channels; ++c) {
    JXL_DASSERT(state.cluster_logits[c].size() == num_cells * num_clusters);
  }

  const size_t cp_count = num_clusters * num_passes;
  const CompactACHistogramData& ac_hist = d.ACHistogram();
  const uint32_t ac_alpha = d.ACHistogramSize();
  constexpr uint32_t kZDC = kZeroDensityContextCount;
  const double inv_pass_t = 1.0 / state.pass_temperature;
  const double inv_thr_t = 1.0 / state.threshold_temperature;
  const double inv_cluster_t = 1.0 / state.cluster_temperature;
  const double inv_ctx_t = 1.0 / state.ctx_temperature;
  const uint32_t H = state.num_hists;
  JXL_DASSERT(state.ctx_logits.size() == num_passes * num_clusters * kZDC * H);
  const bool optimize_pass_count = state.pass_gates.size() == num_passes;
  const double* HWY_RESTRICT pass_gates =
      optimize_pass_count ? state.pass_gates.data() : nullptr;
  const uint32_t pass_group_count = GradientPassGroupCount(d);
  const double total_blocks = static_cast<double>(TotalChannelBlocks(d));
  const double tau_pass =
      std::max(1.0, total_blocks / static_cast<double>(num_passes));
  const double tau_group = std::max(
      1.0, total_blocks / static_cast<double>(pass_group_count * num_passes));
  JXL_DASSERT(ac_hist.dense_to_zdc.size() == ac_alpha);
  JXL_DASSERT(ac_hist.dense_to_token.size() == ac_alpha);
  for (uint32_t c = 0; c < d.channels; ++c) {
    JXL_DASSERT(aux.blocks[c].size() == d.num_blocks[c]);
  }

  // AC forward accumulators in di-major / zdc-major layouts so each
  // AC event triggers one VecAddVec(cp_count) rather than K*P scatter writes.
  std::vector<double>& ac_h = work.ac_h;
  std::vector<double>& ac_N = work.ac_N;
  std::fill(ac_h.begin(), ac_h.end(), 0.0);
  std::fill(ac_N.begin(), ac_N.end(), 0.0);
  // NZ uses `[bin][p][k]` / `[pb][p][k]`, making fixed-pass cluster updates
  // contiguous while preserving pass-dependent predictor buckets.
  std::vector<double>& nz_h = work.nz_h;
  std::vector<double>& nz_N = work.nz_N;
  std::fill(nz_h.begin(), nz_h.end(), 0.0);
  std::fill(nz_N.begin(), nz_N.end(), 0.0);

  // Precompute block-pi vectors for pass weights and NZ neighbor lookups.
  // Flat: `pi_cache[c][b * num_passes + p]`.
  auto& pi_cache = work.pi_cache;
  std::vector<double>& pass_mass = work.pass_mass;
  std::vector<double>& pass_group_mass = work.pass_group_mass;
  if (optimize_pass_count) {
    std::fill(pass_mass.begin(), pass_mass.end(), 0.0);
    std::fill(pass_group_mass.begin(), pass_group_mass.end(), 0.0);
  }
  for (uint32_t c = 0; c < d.channels; ++c) {
    const size_t nb = d.num_blocks[c];
    PassSoftmaxCacheVec(state.pass_logits[c].data(), pass_gates, nb, num_passes,
                        state.pass_temperature, pi_cache[c].data());
    if (optimize_pass_count) {
      for (size_t b = 0; b < nb; ++b) {
        const uint32_t group =
            GradientPassGroupIndex(d, c, static_cast<uint32_t>(b));
        const double* pi = &pi_cache[c][b * num_passes];
        for (uint32_t p = 0; p < num_passes; ++p) {
          pass_mass[p] += pi[p];
          pass_group_mass[p * pass_group_count + group] += pi[p];
        }
      }
    }
  }

  // Precompute per-(channel, cell) cluster-softmax `rho`. Iteration 5 softens
  // the old `cluster = ctx_map[c*num_cells + cell]` lookup into a distribution
  // over clusters: `rho_{c,cell,k} = softmax(cluster_logits[c][cell*K..],
  // cluster_temperature)[k]`.
  auto& rho_cache = work.rho_cache;
  for (uint32_t c = 0; c < d.channels; ++c) {
    for (size_t cell = 0; cell < num_cells; ++cell) {
      Softmax(&state.cluster_logits[c][cell * num_clusters], num_clusters,
              state.cluster_temperature, &rho_cache[c][cell * num_clusters]);
    }
  }

  // Soft context-map sigma: `softmax(ctx_logits[p,k,zdc,:], ctx_temperature)`.
  // Flat layout: `(p * K * kZDC + k * kZDC + zdc) * H + h`.
  std::vector<double>& sigma = work.sigma;
  for (uint32_t p = 0; p < num_passes; ++p) {
    for (uint32_t k = 0; k < num_clusters; ++k) {
      for (uint32_t zdc = 0; zdc < kZDC; ++zdc) {
        const size_t off = (p * num_clusters * kZDC + k * kZDC + zdc) * H;
        Softmax(&state.ctx_logits[off], H, state.ctx_temperature, &sigma[off]);
      }
    }
  }

  // Per-block scratch.
  std::vector<double>& w0 = work.w0;
  std::vector<double>& w1 = work.w1;
  std::vector<double>& w2 = work.w2;
  std::vector<double>& cell_weight = work.cell_weight;
  // Per-block per-cluster aggregate weight,
  //  `B[k] = sum_cell w_cell*rho[c,cell,k]`.
  // Reused by both AC and NZ forward and by the backward pass;
  // sized to fit any cluster axis count, allocated once per call.
  std::vector<double>& B = work.B;
  // `w_vec[k*P+p] = B[k]*pi[p]`; drives `VecAddVec` per AC event.
  std::vector<double>& w_vec = work.w_vec;

  // --- Forward pass ---------------------------------------------------------
  for (uint32_t c = 0; c < d.channels; ++c) {
    const uint32_t nb = d.num_blocks[c];
    const uint32_t grid_w = d.block_grid_w[c];
    for (uint32_t b = 0; b < nb; ++b) {
      const double* pi = &pi_cache[c][b * num_passes];
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

      const uint32_t ev_start = d.block_offsets[c][b];
      const uint32_t ev_end = d.block_offsets[c][b + 1];

      // Per-block per-cluster aggregate `B[k] = sum_cell w_cell*rho[c,cell,k]`.
      // This collapses the cell axis out of the inner accumulation, dropping a
      // factor of `num_cells` from the AC and NZ inner loops.
      ClusterWeightsVec(cell_weight.data(), rho_cache[c].data(), B.data(),
                        num_cells, num_clusters);

      // AC accumulation.
      // Compute `w_vec[k*P+p] = B[k]*pi[p]` once, then per event add
      // `w_vec` into the transposed accumulators with a single `VecAddVec`.
      // This replaces `K*P` random scatter writes with one sequential SIMD
      // add per event, and keeps `ac_h` footprint L2-resident.
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

      // NZ accumulation: `pb` depends only on `(block, pass)`, so precompute it
      // outside the cluster loop. Same `B[k]` aggregation collapses the cell
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
        const size_t N_off = (pb * num_passes + p) * num_clusters;
        const size_t h_real_off = (bin_real * num_passes + p) * num_clusters;
        const size_t h_zero_off = (bin_zero * num_passes + p) * num_clusters;
        VecAddVec(&nz_N[N_off], B.data(), num_clusters);
        AccumScaledVec(&nz_h[h_real_off], pi_p, B.data(), num_clusters);
        AccumScaledVec(&nz_h[h_zero_off], 1.0 - pi_p, B.data(), num_clusters);
      }
    }
  }

  // Context histograms: `ctx_h[p, token, h]` and `ctx_N[p, h]`.
  // Layouts: `ctx_h index = p * kACTokenCount * H + token * H + h`,
  //          `ctx_N index = p * H + h`.
  std::vector<double>& ctx_h = work.ctx_h;
  std::vector<double>& ctx_N = work.ctx_N;
  std::fill(ctx_h.begin(), ctx_h.end(), 0.0);
  std::fill(ctx_N.begin(), ctx_N.end(), 0.0);
  ContextForwardVec(ac_h.data(), sigma.data(), ac_hist.dense_to_zdc.data(),
                    ac_hist.dense_to_token.data(), ctx_h.data(), ctx_N.data(),
                    num_clusters, num_passes, ac_alpha, kZDC, kACTokenCount, H);

  // --- AC cost reduction ----------------------------------------------------
  double ac_cost = 0.0;
  for (uint32_t p = 0; p < num_passes; ++p) {
    const double N_sum = SoftFTabReduceVecFast(&ctx_N[p * H], H);
    const double h_sum =
        SoftFTabReduceVecFast(&ctx_h[p * kACTokenCount * H], kACTokenCount * H);
    ac_cost += N_sum - h_sum;
  }
  result.ac_cost_bits = ac_cost;

  // --- NZ cost + signalling overhead ---------------------------------------
  double nz_cost = SoftFTabReduceVecFast(nz_N.data(), nz_N.size()) -
                   SoftFTabReduceVecFast(nz_h.data(), nz_h.size());
  double overhead_bits = 0.0;
  uint32_t touched_slots = 0;
  for (uint32_t p = 0; p < num_passes; ++p) {
    for (uint32_t h = 0; h < H; ++h) {
      overhead_bits += ACSignallingOverheadBits(
          ctx_h.data(), p, h, H, &touched_slots, &work.overhead_hist);
    }
  }
  result.num_cp_slots = touched_slots;
  for (size_t slot = 0; slot < cp_count; ++slot) {
    overhead_bits += NZSignallingOverheadBits(nz_h.data(), slot, cp_count,
                                              &work.overhead_hist);
  }
  if (optimize_pass_count) {
    overhead_bits += SmoothPassOccupancyOverheadBits(
        pass_mass, pass_group_mass, num_passes, pass_group_count, tau_pass,
        tau_group, grad == nullptr ? nullptr : &work.pass_mass_grad,
        grad == nullptr ? nullptr : &work.pass_group_mass_grad);
  } else {
    overhead_bits += FlatPassOverheadBits(d) * num_passes;
  }
  result.nz_cost_bits = nz_cost;
  result.signalling_overhead_bits = overhead_bits;
  result.total_cost_bits = ac_cost + nz_cost + overhead_bits;

  if (grad == nullptr) return result;

  // --- Backward pass --------------------------------------------------------
  // Precompute upstream gradients wrt `ac_h/ac_N` accumulators.
  std::vector<double>& dL_dN = work.dL_dN;
  std::vector<double>& dL_dh = work.dL_dh;
  // Upstream from context histograms: `dL/dctx_N = +ftab', dL/dctx_h = -ftab'`.
  std::vector<double>& dL_dctx_N = work.dL_dctx_N;
  std::vector<double>& dL_dctx_h = work.dL_dctx_h;
  SoftFTabPrimeVecFast(ctx_N.data(), dL_dctx_N.data(), num_passes * H);
  SoftFTabPrimeNegVecFast(ctx_h.data(), dL_dctx_h.data(),
                          num_passes * kACTokenCount * H);

  // Propagate through `sigma` to get `dL/dh`, `dL/dN`, and `dL/dsigma`.
  JXL_DASSERT(grad->ctx_logits.size() == state.ctx_logits.size());
  std::vector<double>& dL_dsigma = work.dL_dsigma;
  std::fill(dL_dsigma.begin(), dL_dsigma.end(), 0.0);
  ContextBackwardVec(ac_h.data(), ac_N.data(), sigma.data(), dL_dctx_h.data(),
                     dL_dctx_N.data(), ac_hist.dense_to_zdc.data(),
                     ac_hist.dense_to_token.data(), dL_dh.data(), dL_dN.data(),
                     dL_dsigma.data(), num_clusters, num_passes, ac_alpha, kZDC,
                     kACTokenCount, H);

  // Fold the split entropy derivatives into the per-event derivative that the
  // block replay actually consumes:
  //   event(di, cp) = dL/dh(di, cp) + dL/dN(zdc(di), cp).
  std::vector<double>& dL_dac_event = work.dL_dac_event;
  for (uint32_t di = 0; di < ac_alpha; ++di) {
    double* HWY_RESTRICT dst = &dL_dac_event[di * cp_count];
    const uint32_t zdc = ac_hist.dense_to_zdc[di];
    for (uint32_t cp = 0; cp < cp_count; ++cp) {
      dst[cp] = dL_dh[cp * ac_alpha + di] + dL_dN[cp * kZDC + zdc];
    }
  }

  // `dL/dsigma` softmax Jacobian -> `grad->ctx_logits`.
  // Softmax Jacobian per `(p, k, zdc)` row.
  SoftmaxJacobianRowsVec(sigma.data(), dL_dsigma.data(),
                         grad->ctx_logits.data(), inv_ctx_t,
                         num_passes * num_clusters * kZDC, H);

  // Saved per-threshold sigmoid values for one block/axis:
  // `axis_sigma[a][j] = sigmoid((T[a][j] - DC) / tau)`.
  auto& axis_sigma = work.axis_sigma;
  std::vector<double>& dL_dcell = work.dL_dcell;
  std::vector<double>& dL_dpi = work.dL_dpi;
  auto& dL_dw_ax = work.dL_dw_ax;

  // Channel-wide accumulator for `dL/drho[c][cell * K + k]`. Accumulates
  // contributions from every block in channel `c`; converted to
  // `dL/dcluster_logits[c]` via the softmax Jacobian after the block loop.
  auto& dL_drho = work.dL_drho;
  for (uint32_t c = 0; c < d.channels; ++c) {
    std::fill(dL_drho[c].begin(), dL_drho[c].end(), 0.0);
  }

  // NZ upstream gradients.
  std::vector<double>& dL_dnz_N = work.dL_dnz_N;
  std::vector<double>& dL_dnz_event = work.dL_dnz_event;
  SoftFTabPrimeVecFast(nz_N.data(), dL_dnz_N.data(), nz_N.size());
  SoftFTabPrimeNegVecFast(nz_h.data(), dL_dnz_event.data(), nz_h.size());
  for (uint32_t pb = 0; pb < kJPEGNonZeroBuckets; ++pb) {
    for (uint32_t nz = 0; nz < kJPEGNonZeroRange; ++nz) {
      const uint32_t bin = NZHistogramIndex(pb, nz);
      for (uint32_t p = 0; p < num_passes; ++p) {
        VecAddVec(&dL_dnz_event[(bin * num_passes + p) * num_clusters],
                  &dL_dnz_N[(pb * num_passes + p) * num_clusters],
                  num_clusters);
      }
    }
  }

  // Per-block-per-(cluster, pass) scratch tables for the factored backward.
  // `delta_ac_kp[cp]` is the events-aggregated AC gradient at slot cp;
  // `nz_T_kp[cp]` and `nz_diff_event_kp[cp]` carry the NZ-derived per-(k,p)
  // factors. `block_D[k]` is the per-block per-cluster reduction over passes.
  std::vector<double>& delta_ac_kp = work.delta_ac_kp;
  std::vector<double>& nz_T_kp = work.nz_T_kp;
  std::vector<double>& nz_diff_event_kp = work.nz_diff_event_kp;
  std::vector<double>& block_D = work.block_D;

  for (uint32_t c = 0; c < d.channels; ++c) {
    const uint32_t nb = d.num_blocks[c];
    const uint32_t grid_w = d.block_grid_w[c];
    for (uint32_t b = 0; b < nb; ++b) {
      const double* pi = &pi_cache[c][b * num_passes];
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

      // AC contribution: per-(k, p) delta. Each event adds its combined
      // entropy derivative as a contiguous cp_count vector, mirroring forward.
      //   `dL/dpi[p]   = sum_k B[k] * delta_ac[k*P+p]`
      //   `dL/dcell    = sum_k rho_cell[k] * D[k], D[k] = sum_p pi[p]*delta`
      //   `dL/drho[c,cell,k] += w_cell * D[k]`
      std::fill(delta_ac_kp.begin(), delta_ac_kp.end(), 0.0);
      for (uint32_t e = ev_start; e < ev_end; ++e) {
        const CompactACEvent evt = d.FromBin(d.block_bins[c][e]);
        if (evt.hist_bin == kInvalidCompactH) continue;
        VecAddVec(delta_ac_kp.data(), &dL_dac_event[evt.hist_bin * cp_count],
                  cp_count);
      }

      // NZ contribution: `pb` is per-(block, pass), so precompute outside the
      // cluster loop. For each (k, p) we record:
      //   `nz_diff_h[cp] = event_real - event_zero`  (drives dL/dpi; the
      //                    shared N term cancels)
      //   `nz_T_kp[cp]   = pi_p*event_real + (1-pi_p)*event_zero`
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
        const double* HWY_RESTRICT event_real_row =
            &dL_dnz_event[(bin_real * num_passes + p) * num_clusters];
        const double* HWY_RESTRICT event_zero_row =
            &dL_dnz_event[(bin_zero * num_passes + p) * num_clusters];
        for (uint32_t k = 0; k < num_clusters; ++k) {
          const uint32_t cp = k * num_passes + p;
          const double event_real = event_real_row[k];
          const double event_zero = event_zero_row[k];
          nz_diff_event_kp[cp] = event_real - event_zero;
          nz_T_kp[cp] = pi_p * event_real + one_minus_pi_p * event_zero;
        }
      }

      // Build `D[k] = sum_p (pi[p] * delta_ac[cp] + nz_T_kp[cp])` and `pi`-axis
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

      // `dL/dpi[p]` from AC and NZ contributions, factored via `B[k]`.
      for (uint32_t k = 0; k < num_clusters; ++k) {
        const double Bk = B[k];
        if (Bk == 0.0) continue;
        for (uint32_t p = 0; p < num_passes; ++p) {
          const uint32_t cp = k * num_passes + p;
          dL_dpi[p] += Bk * delta_ac_kp[cp];
          dL_dpi[p] += Bk * nz_diff_event_kp[cp];
        }
      }
      if (optimize_pass_count) {
        const uint32_t group = GradientPassGroupIndex(d, c, b);
        const std::vector<double>& pass_mass_grad = work.pass_mass_grad;
        const std::vector<double>& pass_group_mass_grad =
            work.pass_group_mass_grad;
        for (uint32_t p = 0; p < num_passes; ++p) {
          dL_dpi[p] += pass_mass_grad[p];
          dL_dpi[p] += pass_group_mass_grad[p * pass_group_count + group];
        }
      }

      // `dL/dcell[cell] = sum_k rho_cell[k] * D[k]`;
      // `dL/drho[c,cell,k] += w_cell * D[k]`.
      // Single per-cell pass over clusters covers both.
      CellGradientVec(cell_weight.data(), rho_cache[c].data(), D.data(),
                      dL_dcell.data(), dL_drho[c].data(), num_cells,
                      num_clusters);

      // Softmax Jacobian: `dL/dlogit[q] = pi[q] * (dL/dpi[q] - s) / tau_pi`,
      // where `s = sum_p pi[p] * dL/dpi[p]`.
      double s = 0.0;
      for (uint32_t p = 0; p < num_passes; ++p) s += pi[p] * dL_dpi[p];
      double* grad_logits = &grad->pass_logits[c][b * num_passes];
      for (uint32_t q = 0; q < num_passes; ++q) {
        const double g = inv_pass_t * pi[q] * (dL_dpi[q] - s);
        grad_logits[q] += g;
        if (optimize_pass_count) grad->pass_gates[q] += g;
      }

      // Decompose `dL/dcell` into `dL/dw_axis`. Axis 0's weight at `k0` pairs
      // with `w1[k1]*w2[k2]` in the product, so that's the factor we multiply
      // by.
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

      // Threshold gradient per axis. Threshold `T[a][j]` affects buckets `j`
      // and `j+1` in opposite directions.
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

  // Softmax Jacobian for cluster logits: per `(c, cell)`, convert
  // `dL/drho[c][cell*K + k]` into `dL/dcluster_logits[c][cell*K + k]` via
  //   `dL/dlogit[m] = rho[m] * (dL/drho[m] - sum_k rho[k] * dL/drho[k])`
  //                  `/ cluster_temperature`.
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

HWY_EXPORT(SoftForwardBackwardManyPassImpl);

SoftCostResult SoftForwardBackwardManyPass(const JPEGOptData& d,
                                           const GradientAux& aux,
                                           const GradientState& state,
                                           GradientGrad* grad,
                                           GradientScratch* scratch) {
  return HWY_DYNAMIC_DISPATCH(SoftForwardBackwardManyPassImpl)(d, aux, state,
                                                               grad, scratch);
}

}  // namespace jxl

#endif  // HWY_ONCE
