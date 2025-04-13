// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include "lib/jxl/entropy_coder.h"

#include <jxl/memory_manager.h>

#include <cstdint>
#include <vector>

#include "lib/jxl/ac_context.h"
#include "lib/jxl/base/compiler_specific.h"
#include "lib/jxl/base/status.h"
#include "lib/jxl/coeff_order.h"
#include "lib/jxl/coeff_order_fwd.h"
#include "lib/jxl/dec_bit_reader.h"
#include "lib/jxl/dec_context_map.h"
#include "lib/jxl/fields.h"
#include "lib/jxl/pack_signed.h"

namespace jxl {

Status DecodeBlockCtxMap(JxlMemoryManager* memory_manager, BitReader* br,
                         BlockCtxMap* block_ctx_map) {
  auto& dct = block_ctx_map->dc_thresholds;
  auto& qft = block_ctx_map->qf_thresholds;
  bool is_default = static_cast<bool>(br->ReadFixedBits<1>());
  if (is_default) {
    *block_ctx_map = BlockCtxMap();
    return true;
  }
  block_ctx_map->num_dc_ctxs = 1;
  for (int j : {0, 1, 2}) {
    dct[j].resize(br->ReadFixedBits<4>());
    block_ctx_map->num_dc_ctxs *= dct[j].size() + 1;
    for (int& i : dct[j]) {
      i = UnpackSigned(U32Coder::Read(kDCThresholdDist, br));
    }
  }
  qft.resize(br->ReadFixedBits<4>());
  for (uint32_t& i : qft) {
    i = U32Coder::Read(kQFThresholdDist, br) + 1;
  }

  size_t num_contexts =
      3 * kNumOrders * block_ctx_map->num_dc_ctxs * (qft.size() + 1);
  if (num_contexts > 3 * kNumOrders * 64) {
    return JXL_FAILURE("Invalid block context map: too big");
  }

  JXL_ASSIGN_OR_RETURN(block_ctx_map->ctx_map,
                       DecodeContextMap(memory_manager, num_contexts,
                                        &block_ctx_map->num_ctxs, br));
  if (block_ctx_map->num_ctxs > 16) {
    return JXL_FAILURE("Invalid block context map: too many distinct contexts");
  }
  return true;
}

#if JXL_CXX_LANG < JXL_CXX_17
constexpr uint8_t BlockCtxMap::kDefaultCtxMap[];  // from ac_context.h
#endif

}  // namespace jxl
