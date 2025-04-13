// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include "lib/jxl/entropy_coder.h"

#include <jxl/memory_manager.h>

#include <cstdint>
#include <vector>

#include "lib/jxl/ac_context.h"
#include "lib/jxl/base/data_parallel.h"
namespace jxl {

#if JXL_CXX_LANG < JXL_CXX_17
// TODO(Ivan) find better place for this
constexpr ThreadPoolNoInit NoInit;                // from data_parallel.h
constexpr uint8_t BlockCtxMap::kDefaultCtxMap[];  // from ac_context.h
#endif

}  // namespace jxl
