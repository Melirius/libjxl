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

  // Commits a block move: updates the AC histograms, NZ histograms, pass
  // assignment, and NZ predictor cache.
  void ApplyMove(const BlockRef& ref, uint32_t cur_pass, uint32_t new_pass);

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

// Experimental variant that keeps the `(channel, cell)` row grid fixed and
// scores pass moves on `(row, pass)` token/NZ histograms.
AssignPassesRangeResult AssignPassesGreedyAllKFixedRows(
    const JPEGOptData& d, const ActiveRawBins& active,
    const FixedRows& fixed_rows, uint32_t num_rows,
    uint32_t min_num_passes, uint32_t max_num_passes, ThreadPool* pool);

}  // namespace jxl

#endif  // LIB_JXL_TRANSCODE_JPEG_ENC_JPEG_PASS_ASSIGN_H_
