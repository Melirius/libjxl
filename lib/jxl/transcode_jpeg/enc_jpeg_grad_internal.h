// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#ifndef LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_GRAD_INTERNAL_H_
#define LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_GRAD_INTERNAL_H_

#include <array>
#include <cstdint>
#include <vector>

#include "lib/jxl/transcode_jpeg/enc_jpeg_grad.h"

namespace jxl {

struct GradientJointBlockAux {
  std::array<int16_t, kNumCh> dc;
  uint16_t onepass_nz_pb = 0;
  uint16_t onepass_nz_bin_real = 0;
};

struct GradientJointAux {
  uint32_t ac_alpha = 0;
  std::vector<uint32_t> dense_to_zdc_lut;
  std::vector<uint32_t> dense_to_token_lut;
  std::array<std::vector<GradientJointBlockAux>, kNumCh> blocks;
};

GradientJointAux BuildGradientJointAux(const JPEGOptData& d);

SoftCostResult SoftForwardBackwardOnePass(const JPEGOptData& d,
                                          const GradientJointAux& aux,
                                          const GradientJointState& state,
                                          GradientJointGrad* grad);

}  // namespace jxl

#endif  // LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_GRAD_INTERNAL_H_
