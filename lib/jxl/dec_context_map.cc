// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include "lib/jxl/dec_context_map.h"

#include <jxl/memory_manager.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "lib/jxl/ac_context.h"
#include "lib/jxl/base/status.h"
#include "lib/jxl/dec_ans.h"
#include "lib/jxl/dec_bit_reader.h"
#include "lib/jxl/dec_context_map.h"
#include "lib/jxl/entropy_coder.h"
#include "lib/jxl/fields.h"
#include "lib/jxl/inverse_mtf-inl.h"
#include "lib/jxl/pack_signed.h"

namespace jxl {

namespace {

// Context map uses uint8_t.
constexpr size_t kMaxClusters = 256;

Status VerifyContextMap(const std::vector<uint8_t>& context_map,
                        const size_t num_htrees) {
  std::vector<bool> have_htree(num_htrees);
  size_t num_found = 0;
  for (const uint8_t htree : context_map) {
    if (htree >= num_htrees) {
      return JXL_FAILURE("Invalid histogram index in context map.");
    }
    if (!have_htree[htree]) {
      have_htree[htree] = true;
      ++num_found;
    }
  }
  if (num_found != num_htrees) {
    return JXL_FAILURE("Incomplete context map.");
  }
  return true;
}

}  // namespace

StatusOr<std::vector<uint8_t>> DecodeContextMap(
    JxlMemoryManager* memory_manager, size_t num_contexts, size_t* num_htrees,
    BitReader& input) {
  std::vector<uint8_t> context_map;
  context_map.reserve(num_contexts);
  bool is_simple = static_cast<bool>(input.ReadFixedBits<1>());
  if (is_simple) {
    int bits_per_entry = input.ReadFixedBits<2>();
    if (bits_per_entry != 0) {
      for (size_t i = 0; i < num_contexts; ++i) {
        context_map.push_back(input.ReadBits(bits_per_entry));
      }
    } else {
      context_map.assign(num_contexts, 0);
    }
  } else {
    bool use_mtf = static_cast<bool>(input.ReadFixedBits<1>());
    // Usage of LZ77 is disallowed if decoding only two symbols. This doesn't
    // make sense in non-malicious bitstreams, and could cause a stack overflow
    // in malicious bitstreams by making every context map require its own
    // context map.
    JXL_ASSIGN_OR_RETURN(ANSCode code,
                         DecodeHistograms(memory_manager, input, 1,
                                          /*disallow_lz77=*/num_contexts <= 2));
    ANSSymbolReader reader;
    JXL_RETURN_IF_ERROR(reader.Init(code, input));
    size_t i = 0;
    uint32_t maxsym = 0;
    while (i < num_contexts) {
      uint32_t sym = reader.ReadHybridUintInlined</*uses_lz77=*/true>(0);
      maxsym = sym > maxsym ? sym : maxsym;
      context_map.push_back(sym);
      i++;
    }
    if (maxsym >= kMaxClusters) {
      return JXL_FAILURE("Invalid cluster ID");
    }
    if (!reader.CheckANSFinalState()) {
      return JXL_FAILURE("Invalid context map");
    }
    if (use_mtf) {
      InverseMoveToFrontTransform(context_map.data(), context_map.size());
    }
  }
  *num_htrees = *std::max_element(context_map.begin(), context_map.end()) + 1;
  JXL_RETURN_IF_ERROR(VerifyContextMap(context_map, *num_htrees));
  return context_map;
}

Status BlockCtxMap::Decode(JxlMemoryManager* memory_manager, BitReader& br) {
  bool is_default = static_cast<bool>(br.ReadFixedBits<1>());
  if (is_default) {
    *this = BlockCtxMap();
    return true;
  }

  num_dc_ctxs = 1;
  for (int j : {0, 1, 2}) {
    dc_thresholds[j].resize(br.ReadFixedBits<4>());
    num_dc_ctxs *= dc_thresholds[j].size() + 1;
    for (int& i : dc_thresholds[j]) {
      i = UnpackSigned(U32Coder::Read(kDCThresholdDist, &br));
    }
  }

  qf_thresholds.resize(br.ReadFixedBits<4>());
  for (uint32_t& i : qf_thresholds) {
    i = U32Coder::Read(kQFThresholdDist, &br) + 1;
  }

  size_t num_contexts =
      3 * kNumOrders * num_dc_ctxs * (qf_thresholds.size() + 1);
  if (num_contexts > 3 * kNumOrders * 64) {
    return JXL_FAILURE("Invalid block context map: too big");
  }

  JXL_ASSIGN_OR_RETURN(
      ctx_map, DecodeContextMap(memory_manager, num_contexts, &num_ctxs, br));
  if (num_ctxs > 16) {
    return JXL_FAILURE("Invalid block context map: too many distinct contexts");
  }
  return true;
}

}  // namespace jxl
