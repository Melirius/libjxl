// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#ifndef LIB_JXL_DEC_CONTEXT_MAP_H_
#define LIB_JXL_DEC_CONTEXT_MAP_H_

#include <jxl/memory_manager.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "lib/jxl/base/status.h"
#include "lib/jxl/dec_bit_reader.h"

namespace jxl {

// Reads the context map from the bit stream.
// Sets *num_htrees to the number of different histogram ids in
// *context_map.
StatusOr<std::vector<uint8_t>> DecodeContextMap(
    JxlMemoryManager* memory_manager, size_t num_contexts, size_t* num_htrees,
    BitReader& input);

}  // namespace jxl

#endif  // LIB_JXL_DEC_CONTEXT_MAP_H_
