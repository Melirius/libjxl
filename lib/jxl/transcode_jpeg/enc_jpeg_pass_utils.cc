// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_utils.h"

#include <array>
#include <vector>

#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"

namespace jxl {

PrunedCtxMapResult PruneDeadThresholdsFromCtxMap(const ThresholdSet& thresholds,
                                                 ContextMap ctx_map,
                                                 uint32_t channels) {
  PrunedCtxMapResult out;
  out.thresholds = thresholds;
  out.ctx_map = std::move(ctx_map);
  ThresholdSet& T = out.thresholds;

  if (channels == 1 && T.TCb().empty() && T.TCr().empty()) {
    const uint32_t old_n = static_cast<uint32_t>(T.TY().size() + 1);
    Thresholds new_thr;
    new_thr.reserve(T.TY().size());
    std::vector<uint32_t> old_from_new = {0};
    for (uint32_t t = 0; t < T.TY().size(); ++t) {
      if (out.ctx_map[t] != out.ctx_map[t + 1]) {
        old_from_new.push_back(t + 1);
        new_thr.push_back(T.TY()[t]);
      }
    }
    T.TY().swap(new_thr);
    const uint32_t new_n = static_cast<uint32_t>(T.TY().size() + 1);
    ContextMap new_ctx_map(channels * new_n, 0);
    for (uint32_t c = 0; c < channels; ++c) {
      const uint32_t old_base = c * old_n;
      const uint32_t new_base = c * new_n;
      for (uint32_t x = 0; x < new_n; ++x) {
        new_ctx_map[new_base + x] = out.ctx_map[old_base + old_from_new[x]];
      }
    }
    out.ctx_map.swap(new_ctx_map);
    return out;
  }

  const uint32_t sizes[3] = {static_cast<uint32_t>(T.TY().size() + 1),
                             static_cast<uint32_t>(T.TCb().size() + 1),
                             static_cast<uint32_t>(T.TCr().size() + 1)};
  const uint32_t num_cells_init = sizes[0] * sizes[1] * sizes[2];
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
      for (uint32_t c = 0; c < channels; ++c) {
        const uint32_t c_base = c * num_cells_init;
        for (uint32_t k1 = 0; k1 < sizes[ax1]; ++k1) {
          b[ax1] = k1;
          for (uint32_t k2 = 0; k2 < sizes[ax2]; ++k2) {
            b[ax2] = k2;
            const uint32_t gl = (b[1] * sizes[2] + b[2]) * sizes[0] + b[0];
            if (out.ctx_map[c_base + gl] !=
                out.ctx_map[c_base + gl + axis_stride[axis]]) {
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
  ContextMap new_ctx_map(channels * new_num_cells, 0);
  for (uint32_t c = 0; c < channels; ++c) {
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
          new_ctx_map[new_base + g_new] = out.ctx_map[old_base + g_old];
        }
      }
    }
  }
  out.ctx_map.swap(new_ctx_map);
  return out;
}

}  // namespace jxl
