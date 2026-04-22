// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.


#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_stream.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "lib/jxl/base/data_parallel.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_assign.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_axis_maps.h"

namespace jxl {

// --- Pass-aware stream construction --------------------------------------------

struct EmitBin {
  ACBin raw_bin;
  uint32_t hist_key;
  uint32_t compact_id;
};

// Returns the top-left-anchored DC bucket indices for the current block in all
// components. This mirrors the coordinate convention used by the main
// clustering path and lets pass-aware code rebuild the same cell ids.
void BlockDCIndices(const JPEGOptData& d, uint32_t c, uint32_t b, uint32_t* dc0,
                    uint32_t* dc1, uint32_t* dc2) {
  const uint32_t y = b / d.block_grid_w[c];
  const uint32_t x = b % d.block_grid_w[c];
  const uint32_t b0 = MapTopLeftBlockIndex(d, c, y, x, 0);
  *dc0 = d.block_DC_idx[0][b0];
  if (d.channels == 1) {
    *dc1 = 0;
    *dc2 = 0;
    return;
  }
  const uint32_t b1 = MapTopLeftBlockIndex(d, c, y, x, 1);
  const uint32_t b2 = MapTopLeftBlockIndex(d, c, y, x, 2);
  *dc1 = d.block_DC_idx[1][b1];
  *dc2 = d.block_DC_idx[2][b2];
}

// Maps one block to its threshold cell id for the given threshold set. This is
// shared by the pass-aware evaluator and the biclustering lattice builder.
uint32_t BlockCell(const JPEGOptData& d, const AxisMaps& axis_maps,
                   const ThresholdSet& thresholds, uint32_t c, uint32_t b) {
  if (d.channels == 1) {
    return axis_maps.ax0_to_k[d.block_DC_idx[0][b]];
  }
  const uint32_t y = b / d.block_grid_w[c];
  const uint32_t x = b % d.block_grid_w[c];
  const uint32_t b0 = MapTopLeftBlockIndex(d, c, y, x, 0);
  const uint32_t b1 = MapTopLeftBlockIndex(d, c, y, x, 1);
  const uint32_t b2 = MapTopLeftBlockIndex(d, c, y, x, 2);
  const uint32_t n0 = static_cast<uint32_t>(thresholds.TY().size() + 1);
  return (axis_maps.ax1_row[d.block_DC_idx[1][b1]] +
          axis_maps.ax2_col[d.block_DC_idx[2][b2]]) *
             n0 +
         axis_maps.ax0_to_k[d.block_DC_idx[0][b0]];
}

// Metadata for one compact active bin while rebuilding the pass-local AC
// stream. `hist_key` matches the encoder's chosen AC histogram model so bins
// can be emitted in histogram-major order.
std::vector<EmitBin> BuildEmitBins(const JPEGOptData& d,
                                   const ActiveRawBins& active) {
  std::vector<EmitBin> emit_bins;
  emit_bins.reserve(active.active_bins.size());
  for (uint32_t compact_id = 0; compact_id < active.active_bins.size();
       ++compact_id) {
    const ACBin raw_bin = active.active_bins[compact_id];
    emit_bins.push_back({raw_bin, d.ACHistogramKey(raw_bin), compact_id});
  }
  if (d.AC_hist_model == JPEGTranscodeACModel::kToken420) {
    std::sort(emit_bins.begin(), emit_bins.end(),
              [](const EmitBin& a, const EmitBin& b) {
                if (a.hist_key != b.hist_key) return a.hist_key < b.hist_key;
                return a.raw_bin < b.raw_bin;
              });
  }
  return emit_bins;
}

// Rebuilds the AC stream after pass assignment. The resulting stream keeps the
// same packed-entry layout as the canonical optimizer stream, but is split into
// independent contiguous pass ranges recorded in `pass_offsets`.
StatusOr<std::vector<ACEntry>> BuildPassStream(
    const JPEGOptData& d, const ActiveRawBins& active,
    const PassAssignment& pass_assignment, uint32_t num_passes,
    std::vector<uint32_t>* pass_offsets, ThreadPool* pool) {
  const uint32_t M = static_cast<uint32_t>(active.active_bins.size());
  // Each AC entry is bucketed by (pass, compact_bin, dc0_high_bit). The high
  // bit of dc0 splits entries into two halves per (pass, bin), which enables
  // the `emit_half` lambda below to emit a dc0-base jump marker when crossing
  // from the low half to the high half.
  const uint32_t num_buckets = num_passes * M * 2;
  constexpr uint32_t kBlocksPerTask = 2048;
  std::array<uint32_t, kNumCh + 1> block_prefix = {};
  for (uint32_t c = 0; c < d.channels; ++c) {
    block_prefix[c + 1] = block_prefix[c] + d.num_blocks[c];
  }
  for (uint32_t c = d.channels; c < kNumCh; ++c) {
    block_prefix[c + 1] = block_prefix[c];
  }
  const uint32_t total_blocks = block_prefix[d.channels];
  const uint32_t num_tasks =
      (total_blocks + kBlocksPerTask - 1) / kBlocksPerTask;

  // Counting sort: first pass — compute bucket sizes.
  std::vector<uint32_t> bucket_start(static_cast<size_t>(num_buckets) + 1, 0);
  std::vector<std::vector<uint32_t>> thread_bucket_pos;
  JXL_RETURN_IF_ERROR(RunOnPool(
      pool, 0, num_tasks,
      [&](size_t num_threads) -> Status {
        thread_bucket_pos.assign(
            num_threads, std::vector<uint32_t>(num_buckets, 0));
        return true;
      },
      [&](uint32_t task, size_t thread) -> Status {
        std::vector<uint32_t>& local_counts = thread_bucket_pos[thread];
        const uint32_t begin = task * kBlocksPerTask;
        const uint32_t end = std::min(total_blocks, begin + kBlocksPerTask);
        uint32_t c = 0;
        while (c + 1 < block_prefix.size() && begin >= block_prefix[c + 1]) {
          ++c;
        }
        uint32_t global = begin;
        while (global < end && c < d.channels) {
          const uint32_t local_begin = global - block_prefix[c];
          const uint32_t local_end =
              std::min(end, block_prefix[c + 1]) - block_prefix[c];
          for (uint32_t b = local_begin; b < local_end; ++b) {
          uint32_t dc0 = 0;
          uint32_t dc1 = 0;
          uint32_t dc2 = 0;
          BlockDCIndices(d, c, b, &dc0, &dc1, &dc2);
          const uint32_t pass = pass_assignment[c][b];
          for (uint32_t pi = d.block_offsets[c][b];
               pi < d.block_offsets[c][b + 1]; ++pi) {
            const ACBin raw_bin = d.block_bins[c][pi];
            const uint32_t compact_id = active.raw_to_compact[raw_bin];
            ++local_counts[(static_cast<size_t>(pass) * M + compact_id) * 2 +
                           (dc0 >> 10)];
          }
          }
          global = block_prefix[c] + local_end;
          ++c;
        }
        return true;
      },
      "BuildPassStreamCount"));
  for (uint32_t bucket = 0; bucket < num_buckets; ++bucket) {
    uint32_t total = 0;
    for (size_t thread = 0; thread < thread_bucket_pos.size(); ++thread) {
      total += thread_bucket_pos[thread][bucket];
    }
    bucket_start[bucket + 1] = total;
  }
  // Prefix sum to turn counts into exclusive start positions.
  for (size_t i = 0; i < num_buckets; ++i) {
    bucket_start[i + 1] += bucket_start[i];
  }
  for (uint32_t bucket = 0; bucket < num_buckets; ++bucket) {
    uint32_t pos = bucket_start[bucket];
    for (size_t thread = 0; thread < thread_bucket_pos.size(); ++thread) {
      const uint32_t count = thread_bucket_pos[thread][bucket];
      thread_bucket_pos[thread][bucket] = pos;
      pos += count;
    }
  }

  // Counting sort: second pass — scatter entries into buckets.
  // Each entry packs (dc0_low_10_bits, dc1_11_bits, dc2_11_bits) into 32 bits.
  std::vector<uint32_t> flat(bucket_start.back());
  std::vector<std::atomic<uint32_t>> write_pos(num_buckets);
  for (uint32_t bucket = 0; bucket < num_buckets; ++bucket) {
    write_pos[bucket].store(bucket_start[bucket], std::memory_order_relaxed);
  }
  JXL_RETURN_IF_ERROR(RunOnPool(
      pool, 0, num_tasks, ThreadPool::NoInit,
      [&](uint32_t task, size_t /*thread*/) -> Status {
        const uint32_t begin = task * kBlocksPerTask;
        const uint32_t end = std::min(total_blocks, begin + kBlocksPerTask);
        uint32_t c = 0;
        while (c + 1 < block_prefix.size() && begin >= block_prefix[c + 1]) {
          ++c;
        }
        uint32_t global = begin;
        while (global < end && c < d.channels) {
          const uint32_t local_begin = global - block_prefix[c];
          const uint32_t local_end =
              std::min(end, block_prefix[c + 1]) - block_prefix[c];
          for (uint32_t b = local_begin; b < local_end; ++b) {
          uint32_t dc0 = 0;
          uint32_t dc1 = 0;
          uint32_t dc2 = 0;
          BlockDCIndices(d, c, b, &dc0, &dc1, &dc2);
          const uint32_t pass = pass_assignment[c][b];
          for (uint32_t pi = d.block_offsets[c][b];
               pi < d.block_offsets[c][b + 1]; ++pi) {
            const ACBin raw_bin = d.block_bins[c][pi];
            const uint32_t compact_id = active.raw_to_compact[raw_bin];
            const uint32_t bucket =
                (static_cast<size_t>(pass) * M + compact_id) * 2 + (dc0 >> 10);
            const uint32_t pos =
                write_pos[bucket].fetch_add(1, std::memory_order_relaxed);
            flat[pos] =
                ((dc0 & 0x3FFu) << 22) | (dc1 << 11) | dc2;
          }
          }
          global = block_prefix[c] + local_end;
          ++c;
        }
        return true;
      },
      "BuildPassStreamScatter"));

  // Sort within each bucket by the packed DC indices. This groups entries
  // with the same dc0/dc1/dc2 together, enabling run-length encoding in the
  // `emit_half` lambda below.
  JXL_RETURN_IF_ERROR(RunOnPool(
      pool, 0, num_buckets, ThreadPool::NoInit,
      [&](uint32_t bucket, size_t /*thread*/) -> Status {
        const uint32_t begin = bucket_start[bucket];
        const uint32_t end = bucket_start[bucket + 1];
        if (begin < end) {
          std::sort(flat.begin() + begin, flat.begin() + end);
        }
        return true;
      },
      "BuildPassStreamSort"));

  // Emit the final AC stream by iterating over passes and bins in
  // histogram-major order. Within each (pass, bin) group, entries are emitted
  // from the sorted buckets with run-length encoding and dc0 jump markers.
  const std::vector<EmitBin> emit_bins = BuildEmitBins(d, active);
  std::vector<ACEntry> stream;
  stream.reserve(flat.size() + flat.size() / 16);
  if (pass_offsets != nullptr) pass_offsets->assign(num_passes + 1, 0);

  for (uint32_t pass = 0; pass < num_passes; ++pass) {
    if (pass_offsets != nullptr) (*pass_offsets)[pass] = stream.size();
    uint32_t prev_hist_key = UINT32_MAX;
    uint32_t prev_ctx_key = UINT32_MAX;
    uint32_t cur_dc0 = 0;

    for (const EmitBin& emit_bin : emit_bins) {
      const uint32_t bucket_lo = (pass * M + emit_bin.compact_id) * 2;
      const uint32_t s_lo = bucket_start[bucket_lo];
      const uint32_t e_lo = bucket_start[bucket_lo + 1];
      const uint32_t s_hi = bucket_start[bucket_lo + 1];
      const uint32_t e_hi = bucket_start[bucket_lo + 2];
      if (s_lo == e_lo && s_hi == e_hi) continue;

      const uint32_t ctx_key = JPEGOptData::ACBinCZDC(emit_bin.raw_bin);
      const bool bin_change =
          prev_hist_key != UINT32_MAX && emit_bin.hist_key != prev_hist_key;
      const bool ctx_change = prev_ctx_key != UINT32_MAX && ctx_key != prev_ctx_key;

      auto emit_half = [&](uint32_t begin, uint32_t end, uint32_t dc0_base,
                           bool* first_in_bin) {
        uint32_t i = begin;
        while (i < end) {
          uint32_t j = i + 1;
          while (j < end && flat[j] == flat[i]) ++j;
          uint32_t run = j - i;
          const uint32_t dc0 = (flat[i] >> 22) | dc0_base;
          const uint32_t dc1 = (flat[i] >> 11) & 0x7FFu;
          const uint32_t dc2 = flat[i] & 0x7FFu;
          if (*first_in_bin || dc0 - cur_dc0 > 15u) {
            stream.push_back(
                (1u << 31) |
                (static_cast<uint32_t>(ctx_change && *first_in_bin) << 30) |
                (static_cast<uint32_t>(bin_change && *first_in_bin) << 29) |
                (emit_bin.raw_bin << 7) | (dc0 >> 4));
            cur_dc0 = (dc0 >> 4) << 4;
          }

          uint32_t delta_dc0 = dc0 - cur_dc0;
          uint32_t header = (delta_dc0 << 27) | (dc1 << 16) | (dc2 << 5);
          const uint32_t cont_header = (dc1 << 16) | (dc2 << 5);
          while (run > 32) {
            stream.push_back(header | 31u);
            run -= 32;
            header = cont_header;
          }
          stream.push_back(header | (run - 1));
          cur_dc0 = dc0;
          *first_in_bin = false;
          i = j;
        }
      };

      bool first_in_bin = true;
      emit_half(s_lo, e_lo, 0u, &first_in_bin);
      emit_half(s_hi, e_hi, 0x400u, &first_in_bin);
      prev_hist_key = emit_bin.hist_key;
      prev_ctx_key = ctx_key;
    }
  }
  if (pass_offsets != nullptr) (*pass_offsets)[num_passes] = stream.size();
  return stream;
}


FixedRows BuildFixedRows(const JPEGOptData& d, const ThresholdSet& thresholds) {
  FixedRows rows;
  AxisMaps axis_maps(d);
  axis_maps.Update(thresholds);
  const uint32_t num_cells = static_cast<uint32_t>((thresholds.TY().size() + 1) *
                                                   (thresholds.TCb().size() + 1) *
                                                   (thresholds.TCr().size() + 1));
  for (uint32_t c = 0; c < kNumCh; ++c) {
    rows[c].assign(d.num_blocks[c], 0);
  }
  for (uint32_t c = 0; c < d.channels; ++c) {
    for (uint32_t b = 0; b < d.num_blocks[c]; ++b) {
      rows[c][b] = static_cast<uint16_t>(
          c * num_cells + BlockCell(d, axis_maps, thresholds, c, b));
    }
  }
  return rows;
}

}  // namespace jxl
