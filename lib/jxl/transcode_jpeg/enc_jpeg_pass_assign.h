// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Greedy block-to-pass assignment for the experimental pass-aware context
// model search.
//
// `PassAssignmentCtx` holds all mutable state for one pass-assignment solve:
// AC/NZ histograms and the per-block NZ-predictor cache. The public API is
// `AssignPassesGreedy`, which runs the full initialization + refinement
// pipeline and returns the final assignment.

#ifndef LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_PASS_ASSIGN_H_
#define LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_PASS_ASSIGN_H_

#include <array>
#include <cstdint>
#include <vector>

#include "lib/jxl/transcode_jpeg/enc_jpeg_opt_data.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_passes.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_histogram.h"

namespace jxl {

// Dense remapping of the AC bins that are actually present in the image.
// `raw_to_compact` maps raw `ACBin` ids to a compact active-bin index, and
// `compact_to_czdc` stores the corresponding `(channel, zdc)` selector used by
// pass assignment.
struct ActiveRawBins {
  std::vector<ACBin> active_bins;
  std::vector<uint32_t> raw_to_compact;
  std::vector<uint16_t> compact_to_czdc;
};

// Scans the JPEG AC stream and compacts the raw `ACBin` space down to only the
// bins that actually occur in the image.
ActiveRawBins BuildActiveRawBins(const JPEGOptData& d);

// Iterates over all AC bins of one block, calling `fn(ACBin)` for each.
template <typename Func>
void ForEachBlockBin(const JPEGOptData& d, uint32_t c, uint32_t b, Func&& fn) {
  for (uint32_t pi = d.block_offsets[c][b]; pi < d.block_offsets[c][b + 1];
       ++pi) {
    fn(d.block_bins[c][pi]);
  }
}

// Timing breakdown for greedy block-to-pass assignment.
struct PassAssignmentTimings {
  int64_t batch_ns = 0;
  int64_t sequential_ns = 0;
  int64_t total_ns = 0;
  int32_t seq_iters = 0;
  int32_t batch_iters = 0;
};

// Result of greedy pass assignment plus phase timings.
struct AssignPassesResult {
  PassAssignment pass_assignment;
  PassAssignmentTimings timings;
};

// Range result for fused multi-K pass assignment. `results[i]` corresponds to
// `min_num_passes + i`.
struct AssignPassesRangeResult {
  uint32_t min_num_passes = 1;
  std::vector<AssignPassesResult> results;
  PassAssignmentTimings shared_timings;
};

// Lightweight (channel, block-index) pair used as the unit of the greedy
// pass-assignment solver. `c` is a channel index (0..2), `b` is the block
// index within that channel's raster order.
struct BlockRef {
  uint32_t c : 3;
  uint32_t b : 26;
};

struct PreDigestedBin {
  uint32_t compact_id;
  uint16_t czdc;
};

// Per-thread scratch space for `FindBestPass`. Allocated once per thread and
// reused across blocks to avoid per-block heap allocations.
struct AssignScratch {
  // Delta cost per pass: `delta[p]` is the cost change from moving the
  // current block to pass `p` (lower is better).
  std::vector<FixedPointCost> delta;
  // Tracks which `(channel, zdc)` slots were touched during delta computation
  // so they can be reset efficiently.
  std::vector<uint16_t> touched_czdc;
  // Temporary counts of AC bins per `(channel, zdc)` slot for the current
  // block, used to compute the czdc-level histogram delta.
  std::vector<uint32_t> czdc_counts;
};

// Result of one full sequential sweep: number of accepted moves and the exact
// accumulated proxy-cost delta from those moves.
struct SequentialSweepResult {
  uint32_t moves = 0;
  FixedPointCost delta_cost = 0;
};

// All mutable state for one pass-assignment solve: AC/NZ histograms and the
// per-block NZ-predictor cache. Read-only references to the input data and
// derived dimensional constants are bundled here to keep helper signatures
// short.
class PassAssignmentCtx {
 public:
  PassAssignmentCtx(const JPEGOptData& d, const ActiveRawBins& active_bins,
                    uint32_t num_passes);

  /// Initialization ///

  // Initializes pass assignment and AC histograms via seeded spatial growth
  // (or raster-order greedy scan when kLinearScan is true).
  void InitPassAssignmentHistogramAware();

  // Fills `nz_pred_bucket`, `nz_hist_h`, and `nz_hist_N` from the current
  // `pass_assignment`.
  void InitNZPredictorState();

  // Initializes pass assignment with a linear ramp for each component.
  void InitPassAssignmentSimple();

  /// Core primitives ///

  AssignScratch MakeScratch() const;

  // Predicts the nonzero-count bucket for block (c, b) under the assumption
  // that it is assigned to `pass`. The predictor uses only same-pass
  // neighbors: if the top or left neighbor is in a different pass, its NZ
  // count is treated as 0. An optional `override_ref`/`override_pass` pair
  // lets `FindBestPass` evaluate a hypothetical move without modifying
  // `pass_assignment` first.
  uint32_t PredictNZBucket(uint16_t c, uint32_t b, uint32_t pass,
                           const BlockRef* override_ref,
                           uint32_t override_pass) const;

  // Evaluates the cost delta of moving block `ref` from `cur_pass` to every
  // other pass, and returns the pass with the lowest delta.
  uint32_t FindBestPass(const BlockRef& ref, uint32_t cur_pass,
                        AssignScratch* scratch,
                        FixedPointCost* best_delta = nullptr) const;

  uint32_t FindBestPassCached(const BlockRef& ref, uint32_t cur_pass,
                              const PreDigestedBin* bins, size_t num_bins,
                              AssignScratch* scratch,
                              FixedPointCost* best_delta = nullptr) const;

  // Commits a block move: updates the AC histograms, NZ histograms, pass
  // assignment, and NZ predictor cache.
  void ApplyMove(const BlockRef& ref, uint32_t cur_pass, uint32_t new_pass);

  void ApplyMoveCached(const BlockRef& ref, uint32_t cur_pass,
                       uint32_t new_pass, const PreDigestedBin* bins,
                       size_t num_bins);

  // Computes the total entropy cost of the current pass assignment state.
  FixedPointCost TotalCost() const;

  /// Iterative solvers ///

  // One sequential sweep over all active blocks. Returns the number of blocks
  // moved and the exact accumulated proxy-cost delta of accepted moves.
  SequentialSweepResult SequentialIter(
      const std::vector<BlockRef>& active_blocks, AssignScratch& scratch);

  // Scores all active blocks in parallel; writes proposed best pass per block
  // into new_passes[]. Returns number of proposed changes.
  uint32_t ScoreBatchMoves(const std::vector<BlockRef>& active_blocks,
                           std::vector<uint8_t>& new_passes,
                           std::vector<AssignScratch>& scratch_pool,
                           ThreadPool* pool);

  // Applies the subset of `new_passes` moves where `i % stride == iter % stride`.
  uint32_t ApplyBatchMoves(const std::vector<BlockRef>& active_blocks,
                           const std::vector<uint8_t>& new_passes,
                           uint32_t stride, uint32_t iter);

  // Snapshot-batched parallel refinement phase for large images.
  // Returns elapsed nanoseconds; updates iter and seq_moves.
  int64_t RunBatchPassRefinement(const std::vector<BlockRef>& active_blocks,
                                 ThreadPool* pool, uint32_t min_moves,
                                 uint32_t& iter, uint32_t& seq_moves);

  // Serial sweep-to-convergence phase. Returns elapsed nanoseconds.
  int64_t RunSequentialPassRefinement(
      const std::vector<BlockRef>& active_blocks, ThreadPool* pool,
      uint32_t min_moves, uint32_t& iter);

  /// Public data ///

  const JPEGOptData& d;
  const ActiveRawBins& active;
  const uint32_t num_passes;
  const uint32_t M;          // active bin count
  const uint32_t czdc_size;  // `d.channels * kZeroDensityContextCount`

  PassAssignment pass_assignment;
  std::vector<uint32_t> hist_h;     // `[M * num_passes]`
  std::vector<uint32_t> hist_N;     // `[czdc_size * num_passes]`
  std::vector<uint32_t> nz_hist_h;  // `[kNZHistogramsSize * num_passes]`
  std::vector<uint32_t> nz_hist_N;  // `[kJPEGNonZeroBuckets * num_passes]`
  std::array<std::vector<uint8_t>, kNumCh> nz_pred_bucket;
};

// Greedy block-to-pass assignment:
//   1. Seeded spatial growth initializes histograms and pass_assignment.
//   2. NZ predictor state built from initial assignment.
//   3. One serial sweep warms up the refinement state.
//   4. Snapshot-batched parallel refinement (large images only).
//   5. Serial sweep-to-convergence.
AssignPassesResult AssignPassesGreedy(const JPEGOptData& d,
                                      const ActiveRawBins& active,
                                      uint32_t num_passes, ThreadPool* pool);

// Fused greedy block-to-pass assignment for a whole pass-count range.
// Runs the current "1 seq warm-up, 5 unconditional batch, batch with rare
// sequential rescues, then sequential with rare batch rescues" schedule across
// all `K = [min_num_passes, max_num_passes]` in one shared block traversal.
AssignPassesRangeResult AssignPassesGreedyAllK(
    const JPEGOptData& d, const ActiveRawBins& active,
    uint32_t min_num_passes, uint32_t max_num_passes, ThreadPool* pool);

inline uint32_t PassAssignmentCtx::PredictNZBucket(uint16_t c, uint32_t b,
                                            uint32_t pass,
                                            const BlockRef* override_ref,
                                            uint32_t override_pass) const {
  const uint32_t w = d.block_grid_w[c];
  const uint32_t x = b % w;
  const uint32_t y = b / w;
  const uint32_t b_top = (y > 0) ? b - w : 0;
  const uint32_t b_left = (x > 0) ? b - 1 : 0;
  auto PassOf = [&](uint32_t neighbor) -> uint32_t {
    if (override_ref != nullptr && override_ref->c == c &&
        override_ref->b == neighbor)
      return override_pass;
    return pass_assignment[c][neighbor];
  };
  const uint32_t pass_nz_top =
      (y > 0 && PassOf(b_top) == pass) ? d.block_nonzeros[c][b_top] : 0u;
  const uint32_t pass_nz_left =
      (x > 0 && PassOf(b_left) == pass) ? d.block_nonzeros[c][b_left] : 0u;
  uint32_t predicted_nz;
  if (x == 0 && y == 0) {
    predicted_nz = 32u;
  } else if (x == 0) {
    predicted_nz = pass_nz_top;
  } else if (y == 0) {
    predicted_nz = pass_nz_left;
  } else {
    predicted_nz = (pass_nz_top + pass_nz_left + 1u) / 2u;
  }
  return (predicted_nz < 8) ? predicted_nz : (4 + predicted_nz / 2);
}

inline uint32_t PassAssignmentCtx::FindBestPass(const BlockRef& ref,
                                         uint32_t cur_pass,
                                         AssignScratch* scratch,
                                         FixedPointCost* best_delta_out) const {
  std::fill(scratch->delta.begin(), scratch->delta.end(), 0);
  scratch->touched_czdc.clear();

  // --- Part 1: AC histogram delta ---
  ForEachBlockBin(d, ref.c, ref.b, [&](ACBin bin) {
    const uint32_t compact_id = active.raw_to_compact[bin];
    const uint32_t czdc = active.compact_to_czdc[compact_id];
    if (scratch->czdc_counts[czdc]++ == 0)
      scratch->touched_czdc.push_back(static_cast<uint16_t>(czdc));
    const uint32_t* h_row =
        &hist_h[static_cast<size_t>(compact_id) * num_passes];
    const FixedPointCost rm_cost =
        d.ftab[h_row[cur_pass] - 1] - d.ftab[h_row[cur_pass]];
    for (uint32_t p = 0; p < num_passes; ++p) {
      scratch->delta[p] -=
          (d.ftab[h_row[p] + 1] - d.ftab[h_row[p]]) + rm_cost;
    }
  });

  // CZDC-level count delta.
  for (uint16_t czdc : scratch->touched_czdc) {
    const uint32_t n = scratch->czdc_counts[czdc];
    scratch->czdc_counts[czdc] = 0;
    const uint32_t* n_row = &hist_N[static_cast<size_t>(czdc) * num_passes];
    const FixedPointCost rm_cost =
        d.ftab[n_row[cur_pass] - n] - d.ftab[n_row[cur_pass]];
    for (uint32_t p = 0; p < num_passes; ++p)
      scratch->delta[p] += rm_cost + d.ftab[n_row[p] + n] - d.ftab[n_row[p]];
  }

  // --- Part 2: NZ predictor delta ---
  std::array<BlockRef, 3> affected = {ref, ref, ref};
  size_t num_affected = 1;
  const uint32_t w = d.block_grid_w[ref.c];
  const uint32_t x = ref.b % w;
  if (x + 1 < w) affected[num_affected++] = {ref.c, ref.b + 1u};
  if (ref.b + w < d.num_blocks[ref.c])
    affected[num_affected++] = {ref.c, ref.b + w};

  scratch->delta[cur_pass] = 1;
  FixedPointCost best_delta = 0;
  uint32_t best_pass = cur_pass;
  for (uint32_t p = 0; p < num_passes; ++p) {
    if (p != cur_pass) {
      std::array<uint32_t, 6> n_keys = {};
      std::array<int32_t, 6> n_diff = {};
      size_t n_size = 0;
      std::array<uint32_t, 6> h_keys = {};
      std::array<int32_t, 6> h_diff = {};
      size_t h_size = 0;
      auto AddChange = [](uint32_t key, int32_t delta,
                          std::array<uint32_t, 6>* keys,
                          std::array<int32_t, 6>* diff, size_t* size) {
        for (size_t i = 0; i < *size; ++i) {
          if ((*keys)[i] == key) {
            (*diff)[i] += delta;
            return;
          }
        }
        (*keys)[*size] = key;
        (*diff)[*size] = delta;
        ++*size;
      };
      for (size_t i = 0; i < num_affected; ++i) {
        const BlockRef& ar = affected[i];
        const uint32_t old_pass = pass_assignment[ar.c][ar.b];
        const uint32_t old_pb = nz_pred_bucket[ar.c][ar.b];
        const uint32_t new_pass =
            (ar.c == ref.c && ar.b == ref.b) ? p : old_pass;
        const uint32_t new_pb =
            PredictNZBucket(ar.c, ar.b, new_pass, &ref, p);
        if (old_pass == new_pass && old_pb == new_pb) continue;
        const uint32_t nz = d.block_nonzeros[ar.c][ar.b];
        AddChange(old_pass * kJPEGNonZeroBuckets + old_pb, -1, &n_keys,
                  &n_diff, &n_size);
        AddChange(new_pass * kJPEGNonZeroBuckets + new_pb, +1, &n_keys,
                  &n_diff, &n_size);
        AddChange(old_pass * kNZHistogramsSize + NZHistogramIndex(old_pb, nz),
                  -1, &h_keys, &h_diff, &h_size);
        AddChange(new_pass * kNZHistogramsSize + NZHistogramIndex(new_pb, nz),
                  +1, &h_keys, &h_diff, &h_size);
      }
      for (size_t i = 0; i < n_size; ++i) {
        if (n_diff[i] == 0) continue;
        scratch->delta[p] +=
            d.NZFTab(static_cast<uint32_t>(
                static_cast<int32_t>(nz_hist_N[n_keys[i]]) + n_diff[i])) -
            d.NZFTab(nz_hist_N[n_keys[i]]);
      }
      for (size_t i = 0; i < h_size; ++i) {
        if (h_diff[i] == 0) continue;
        scratch->delta[p] -=
            d.NZFTab(static_cast<uint32_t>(
                static_cast<int32_t>(nz_hist_h[h_keys[i]]) + h_diff[i])) -
            d.NZFTab(nz_hist_h[h_keys[i]]);
      }
    }
    if (scratch->delta[p] < best_delta) {
      best_delta = scratch->delta[p];
      best_pass = p;
    }
  }
  if (best_delta_out != nullptr) *best_delta_out = best_delta;
  return best_pass;
}

inline uint32_t PassAssignmentCtx::FindBestPassCached(
    const BlockRef& ref, uint32_t cur_pass, const PreDigestedBin* bins,
    size_t num_bins, AssignScratch* scratch,
    FixedPointCost* best_delta_out) const {
  std::fill(scratch->delta.begin(), scratch->delta.end(), 0);
  scratch->touched_czdc.clear();

  // --- Part 1: AC histogram delta ---
  for (size_t i = 0; i < num_bins; ++i) {
    const uint32_t compact_id = bins[i].compact_id;
    const uint32_t czdc = bins[i].czdc;
    if (scratch->czdc_counts[czdc]++ == 0)
      scratch->touched_czdc.push_back(static_cast<uint16_t>(czdc));
    const uint32_t* h_row =
        &hist_h[static_cast<size_t>(compact_id) * num_passes];
    const FixedPointCost rm_cost =
        d.ftab[h_row[cur_pass] - 1] - d.ftab[h_row[cur_pass]];
    for (uint32_t p = 0; p < num_passes; ++p) {
      scratch->delta[p] -=
          (d.ftab[h_row[p] + 1] - d.ftab[h_row[p]]) + rm_cost;
    }
  }

  // CZDC-level count delta.
  for (uint16_t czdc : scratch->touched_czdc) {
    const uint32_t n = scratch->czdc_counts[czdc];
    scratch->czdc_counts[czdc] = 0;
    const uint32_t* n_row = &hist_N[static_cast<size_t>(czdc) * num_passes];
    const FixedPointCost rm_cost =
        d.ftab[n_row[cur_pass] - n] - d.ftab[n_row[cur_pass]];
    for (uint32_t p = 0; p < num_passes; ++p)
      scratch->delta[p] += rm_cost + d.ftab[n_row[p] + n] - d.ftab[n_row[p]];
  }

  // --- Part 2: NZ predictor delta ---
  std::array<BlockRef, 3> affected = {ref, ref, ref};
  size_t num_affected = 1;
  const uint32_t w = d.block_grid_w[ref.c];
  const uint32_t x = ref.b % w;
  if (x + 1 < w) affected[num_affected++] = {ref.c, ref.b + 1u};
  if (ref.b + w < d.num_blocks[ref.c])
    affected[num_affected++] = {ref.c, ref.b + w};

  scratch->delta[cur_pass] = 1;
  FixedPointCost best_delta = 0;
  uint32_t best_pass = cur_pass;
  for (uint32_t p = 0; p < num_passes; ++p) {
    if (p != cur_pass) {
      std::array<uint32_t, 6> n_keys = {};
      std::array<int32_t, 6> n_diff = {};
      size_t n_size = 0;
      std::array<uint32_t, 6> h_keys = {};
      std::array<int32_t, 6> h_diff = {};
      size_t h_size = 0;
      auto AddChange = [](uint32_t key, int32_t delta,
                          std::array<uint32_t, 6>* keys,
                          std::array<int32_t, 6>* diff, size_t* size) {
        for (size_t i = 0; i < *size; ++i) {
          if ((*keys)[i] == key) {
            (*diff)[i] += delta;
            return;
          }
        }
        (*keys)[*size] = key;
        (*diff)[*size] = delta;
        ++*size;
      };
      for (size_t i = 0; i < num_affected; ++i) {
        const BlockRef& ar = affected[i];
        const uint32_t old_pass = pass_assignment[ar.c][ar.b];
        const uint32_t old_pb = nz_pred_bucket[ar.c][ar.b];
        const uint32_t new_pass =
            (ar.c == ref.c && ar.b == ref.b) ? p : old_pass;
        const uint32_t new_pb =
            PredictNZBucket(ar.c, ar.b, new_pass, &ref, p);
        if (old_pass == new_pass && old_pb == new_pb) continue;
        const uint32_t nz = d.block_nonzeros[ar.c][ar.b];
        AddChange(old_pass * kJPEGNonZeroBuckets + old_pb, -1, &n_keys,
                  &n_diff, &n_size);
        AddChange(new_pass * kJPEGNonZeroBuckets + new_pb, +1, &n_keys,
                  &n_diff, &n_size);
        AddChange(old_pass * kNZHistogramsSize + NZHistogramIndex(old_pb, nz),
                  -1, &h_keys, &h_diff, &h_size);
        AddChange(new_pass * kNZHistogramsSize + NZHistogramIndex(new_pb, nz),
                  +1, &h_keys, &h_diff, &h_size);
      }
      for (size_t i = 0; i < n_size; ++i) {
        if (n_diff[i] == 0) continue;
        scratch->delta[p] +=
            d.NZFTab(static_cast<uint32_t>(
                static_cast<int32_t>(nz_hist_N[n_keys[i]]) + n_diff[i])) -
            d.NZFTab(nz_hist_N[n_keys[i]]);
      }
      for (size_t i = 0; i < h_size; ++i) {
        if (h_diff[i] == 0) continue;
        scratch->delta[p] -=
            d.NZFTab(static_cast<uint32_t>(
                static_cast<int32_t>(nz_hist_h[h_keys[i]]) + h_diff[i])) -
            d.NZFTab(nz_hist_h[h_keys[i]]);
      }
    }
    if (scratch->delta[p] < best_delta) {
      best_delta = scratch->delta[p];
      best_pass = p;
    }
  }
  if (best_delta_out != nullptr) *best_delta_out = best_delta;
  return best_pass;
}

inline void PassAssignmentCtx::ApplyMove(const BlockRef& ref, uint32_t cur_pass,
                                  uint32_t new_pass) {
  std::array<BlockRef, 3> affected = {ref, ref, ref};
  size_t num_affected = 1;
  const uint32_t w = d.block_grid_w[ref.c];
  const uint32_t x = ref.b % w;
  if (x + 1 < w)
    affected[num_affected++] = {ref.c, static_cast<uint32_t>(ref.b + 1)};
  if (ref.b + w < d.num_blocks[ref.c])
    affected[num_affected++] = {ref.c, static_cast<uint32_t>(ref.b + w)};

  struct NZState {
    BlockRef ref;
    uint32_t old_pass, old_pb, new_pass, new_pb;
  };
  std::array<NZState, 3> nz_states = {};
  for (size_t i = 0; i < num_affected; ++i) {
    const BlockRef& ar = affected[i];
    const uint32_t op = pass_assignment[ar.c][ar.b];
    const uint32_t np = (ar.c == ref.c && ar.b == ref.b) ? new_pass : op;
    nz_states[i] = {ar, op, nz_pred_bucket[ar.c][ar.b], np,
                    PredictNZBucket(ar.c, ar.b, np, &ref, new_pass)};
  }

  ForEachBlockBin(d, ref.c, ref.b, [&](ACBin bin) {
    const uint32_t compact_id = active.raw_to_compact[bin];
    const uint32_t czdc = active.compact_to_czdc[compact_id];
    --hist_h[static_cast<size_t>(compact_id) * num_passes + cur_pass];
    ++hist_h[static_cast<size_t>(compact_id) * num_passes + new_pass];
    --hist_N[static_cast<size_t>(czdc) * num_passes + cur_pass];
    ++hist_N[static_cast<size_t>(czdc) * num_passes + new_pass];
  });

  for (size_t i = 0; i < num_affected; ++i) {
    const auto& s = nz_states[i];
    --nz_hist_N[static_cast<size_t>(s.old_pass) * kJPEGNonZeroBuckets +
                s.old_pb];
    --nz_hist_h[static_cast<size_t>(s.old_pass) * kNZHistogramsSize +
                NZHistogramIndex(s.old_pb,
                                 d.block_nonzeros[s.ref.c][s.ref.b])];
  }
  pass_assignment[ref.c][ref.b] = static_cast<uint8_t>(new_pass);
  for (size_t i = 0; i < num_affected; ++i) {
    const auto& s = nz_states[i];
    ++nz_hist_N[static_cast<size_t>(s.new_pass) * kJPEGNonZeroBuckets +
                s.new_pb];
    ++nz_hist_h[static_cast<size_t>(s.new_pass) * kNZHistogramsSize +
                NZHistogramIndex(s.new_pb,
                                 d.block_nonzeros[s.ref.c][s.ref.b])];
    nz_pred_bucket[s.ref.c][s.ref.b] = static_cast<uint8_t>(s.new_pb);
  }
}

inline void PassAssignmentCtx::ApplyMoveCached(const BlockRef& ref, uint32_t cur_pass,
                                        uint32_t new_pass,
                                        const PreDigestedBin* bins,
                                        size_t num_bins) {
  std::array<BlockRef, 3> affected = {ref, ref, ref};
  size_t num_affected = 1;
  const uint32_t w = d.block_grid_w[ref.c];
  const uint32_t x = ref.b % w;
  if (x + 1 < w)
    affected[num_affected++] = {ref.c, static_cast<uint32_t>(ref.b + 1)};
  if (ref.b + w < d.num_blocks[ref.c])
    affected[num_affected++] = {ref.c, static_cast<uint32_t>(ref.b + w)};

  struct NZState {
    BlockRef ref;
    uint32_t old_pass, old_pb, new_pass, new_pb;
  };
  std::array<NZState, 3> nz_states = {};
  for (size_t i = 0; i < num_affected; ++i) {
    const BlockRef& ar = affected[i];
    const uint32_t op = pass_assignment[ar.c][ar.b];
    const uint32_t np = (ar.c == ref.c && ar.b == ref.b) ? new_pass : op;
    nz_states[i] = {ar, op, nz_pred_bucket[ar.c][ar.b], np,
                    PredictNZBucket(ar.c, ar.b, np, &ref, new_pass)};
  }

  for (size_t i = 0; i < num_bins; ++i) {
    const uint32_t compact_id = bins[i].compact_id;
    const uint32_t czdc = bins[i].czdc;
    --hist_h[static_cast<size_t>(compact_id) * num_passes + cur_pass];
    ++hist_h[static_cast<size_t>(compact_id) * num_passes + new_pass];
    --hist_N[static_cast<size_t>(czdc) * num_passes + cur_pass];
    ++hist_N[static_cast<size_t>(czdc) * num_passes + new_pass];
  }

  for (size_t i = 0; i < num_affected; ++i) {
    const auto& s = nz_states[i];
    --nz_hist_N[static_cast<size_t>(s.old_pass) * kJPEGNonZeroBuckets +
                s.old_pb];
    --nz_hist_h[static_cast<size_t>(s.old_pass) * kNZHistogramsSize +
                NZHistogramIndex(s.old_pb,
                                 d.block_nonzeros[s.ref.c][s.ref.b])];
  }
  pass_assignment[ref.c][ref.b] = static_cast<uint8_t>(new_pass);
  for (size_t i = 0; i < num_affected; ++i) {
    const auto& s = nz_states[i];
    ++nz_hist_N[static_cast<size_t>(s.new_pass) * kJPEGNonZeroBuckets +
                s.new_pb];
    ++nz_hist_h[static_cast<size_t>(s.new_pass) * kNZHistogramsSize +
                NZHistogramIndex(s.new_pb,
                                 d.block_nonzeros[s.ref.c][s.ref.b])];
    nz_pred_bucket[s.ref.c][s.ref.b] = static_cast<uint8_t>(s.new_pb);
  }
}

}  // namespace jxl

#endif  // LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_PASS_ASSIGN_H_
