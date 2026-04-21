// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Fused greedy block-to-pass assignment across a whole pass-count range.
//
// This translation unit contains the experimental all-K scheduler used by the
// pass-aware planner when it wants to evaluate multiple pass counts together.
// The single-K solver remains in `enc_jpeg_pass_assign.cc`.

#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_assign.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "lib/jxl/base/data_parallel.h"

namespace jxl {

namespace {

constexpr uint32_t kMaxIters = 100;
constexpr uint32_t kLargeImageThreshold = 1u << 15;
constexpr uint32_t kBatchChunkSize = 1u << 14;

using PlannerClock = std::chrono::high_resolution_clock;

int64_t ElapsedNanos(const PlannerClock::time_point& start,
                     const PlannerClock::time_point& end) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(end - start)
      .count();
}

double NanosToMs(int64_t ns) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::nanoseconds(ns))
      .count();
}

struct MultiKState {
  uint32_t num_passes;
  PassAssignmentCtx ctx;
  AssignScratch scratch;
  std::vector<uint8_t> new_passes;
  uint32_t min_moves;
  uint32_t seq_moves;
  FixedPointCost current_cost;
  bool finished;

  MultiKState(const JPEGOptData& d, const ActiveRawBins& active,
              uint32_t num_passes, size_t num_active_blocks)
      : num_passes(num_passes),
        ctx(d, active, num_passes),
        scratch(ctx.MakeScratch()),
        new_passes(num_active_blocks, 0),
        min_moves(std::max<uint32_t>(
            1, static_cast<uint32_t>(
                   std::max<size_t>(size_t{1}, num_active_blocks >> 13)))),
        seq_moves(std::numeric_limits<uint32_t>::max()),
        current_cost(0),
        finished(false) {}
};

bool AnyActiveK(const std::vector<MultiKState>& states) {
  for (const auto& state : states) {
    if (!state.finished && state.seq_moves > state.min_moves) return true;
  }
  return false;
}

bool GainWellEnough(FixedPointCost gain, FixedPointCost current_cost) {
  const FixedPointCost threshold =
      std::max<FixedPointCost>(1, current_cost >> 14);
  return gain >= threshold;
}

bool MajorityBatchImproved(std::vector<MultiKState>* states,
                           const std::vector<uint32_t>& batch_moves,
                           const std::vector<FixedPointCost>& new_costs) {
  size_t active = 0;
  size_t good = 0;
  for (size_t i = 0; i < states->size(); ++i) {
    MultiKState& state = (*states)[i];
    const FixedPointCost old_cost = state.current_cost;
    const FixedPointCost gain = std::max<FixedPointCost>(0, old_cost - new_costs[i]);
    const bool finished_now = (batch_moves[i] == 0);
    if (!state.finished && !finished_now && state.seq_moves > state.min_moves) {
      ++active;
      if (GainWellEnough(gain, old_cost)) {
        ++good;
      }
    }
    state.current_cost = new_costs[i];
    state.finished = state.finished || finished_now;
  }
  return active == 0 || good * 2 >= active;
}

bool MajoritySequentialImproved(std::vector<MultiKState>* states,
                                const std::vector<SequentialSweepResult>& seq_stats) {
  size_t active = 0;
  size_t good = 0;
  for (size_t i = 0; i < states->size(); ++i) {
    MultiKState& state = (*states)[i];
    const FixedPointCost old_cost = state.current_cost;
    state.seq_moves = seq_stats[i].moves;
    state.current_cost += seq_stats[i].delta_cost;
    const FixedPointCost gain =
        std::max<FixedPointCost>(0, -seq_stats[i].delta_cost);
    const bool finished_now = (seq_stats[i].moves == 0);
    if (!state.finished && !finished_now &&
        seq_stats[i].moves > state.min_moves) {
      ++active;
      if (GainWellEnough(gain, old_cost)) {
        ++good;
      }
    }
    state.finished = state.finished || finished_now;
  }
  // Be more permissive here than on the batch side: the sequential phase is
  // the more trustworthy local refiner, so we only consider it "not helping"
  // when clearly less than a third of the still-active K values improve well.
  return active == 0 || good * 3 >= active;
}

uint32_t TotalBlockCount(const JPEGOptData& d) {
  uint32_t total = 0;
  for (uint32_t c = 0; c < d.channels; ++c) total += d.num_blocks[c];
  return total;
}

void PrintMoveTableHeader(const std::vector<MultiKState>& states,
                          int move_width) {
  fprintf(stderr, "PLANNER: [all-k] %-12s", "step");
  for (const auto & state : states) {
    fprintf(stderr, " %*u", move_width, state.num_passes);
  }
  fprintf(stderr, "\n");
  fflush(stderr);
}

void PrintMoveTableRow(const char* label, const std::vector<MultiKState>& states,
                       const std::vector<uint32_t>& moves, int move_width) {
  fprintf(stderr, "PLANNER: [all-k] %-12s", label);
  for (size_t i = 0; i < states.size(); ++i) {
    fprintf(stderr, " %*u", move_width, moves[i]);
  }
  fprintf(stderr, "\n");
  fflush(stderr);
}

std::vector<SequentialSweepResult> FusedSequentialIter(
    std::vector<MultiKState>* states, const std::vector<BlockRef>& active_blocks,
    ThreadPool* pool) {
  std::vector<SequentialSweepResult> results(states->size());
  auto run_one_k = [&](size_t ki) {
    MultiKState& state = (*states)[ki];
    if (state.finished) return;
    SequentialSweepResult local_result;
    for (const BlockRef& ref : active_blocks) {
      const uint32_t cur = state.ctx.pass_assignment[ref.c][ref.b];
      FixedPointCost best_delta = 0;
      const uint32_t best =
          state.ctx.FindBestPass(ref, cur, &state.scratch, &best_delta);
      if (best == cur) continue;
      state.ctx.ApplyMove(ref, cur, best);
      ++local_result.moves;
      local_result.delta_cost += best_delta;
    }
    results[ki] = local_result;
  };
  if (pool != nullptr && states->size() > 1) {
    (void)RunOnPool(
        pool, 0, static_cast<uint32_t>(states->size()), ThreadPool::NoInit,
        [&](uint32_t ki, size_t /*thread*/) -> Status {
          run_one_k(ki);
          return true;
        },
        "AssignPassesGreedyAllKSeq");
  } else {
    for (size_t ki = 0; ki < states->size(); ++ki) {
      run_one_k(ki);
    }
  }
  return results;
}

std::vector<uint32_t> FusedScoreBatchMoves(
    std::vector<MultiKState>* states, const std::vector<BlockRef>& active_blocks,
    ThreadPool* pool) {
  const uint32_t num_chunks = static_cast<uint32_t>(
      (active_blocks.size() + kBatchChunkSize - 1) / kBatchChunkSize);
  std::vector<std::vector<uint32_t>> thread_moves;
  std::vector<std::vector<AssignScratch>> scratch_pool;
  if (!RunOnPool(
          pool, 0, num_chunks,
          [&](size_t num_threads) -> Status {
            thread_moves.assign(num_threads,
                                std::vector<uint32_t>(states->size(), 0));
            scratch_pool.resize(num_threads);
            for (size_t t = 0; t < num_threads; ++t) {
              scratch_pool[t].reserve(states->size());
              for (size_t k = 0; k < states->size(); ++k) {
                scratch_pool[t].push_back((*states)[k].ctx.MakeScratch());
              }
            }
            return true;
          },
          [&](uint32_t chunk, size_t thread_id) -> Status {
            std::vector<uint32_t>& local_moves = thread_moves[thread_id];
            const size_t begin = static_cast<size_t>(chunk) * kBatchChunkSize;
            const size_t end =
                std::min(begin + kBatchChunkSize, active_blocks.size());
            for (size_t i = begin; i < end; ++i) {
              const BlockRef& ref = active_blocks[i];
              for (size_t k = 0; k < states->size(); ++k) {
                MultiKState& state = (*states)[k];
                if (state.finished) continue;
                const uint32_t cur = state.ctx.pass_assignment[ref.c][ref.b];
                const uint32_t best = state.ctx.FindBestPass(
                    ref, cur, &scratch_pool[thread_id][k]);
                state.new_passes[i] = static_cast<uint8_t>(best);
                if (best != cur) ++local_moves[k];
              }
            }
            return true;
          },
          "AssignPassesGreedyAllKBatch")) {
    return std::vector<uint32_t>(states->size(), 0);
  }

  std::vector<uint32_t> batch_moves(states->size(), 0);
  for (const auto& move : thread_moves) {
    for (size_t k = 0; k < states->size(); ++k) {
      batch_moves[k] += move[k];
    }
  }
  return batch_moves;
}

std::vector<uint32_t> FusedApplyBatchMoves(
    std::vector<MultiKState>* states, const std::vector<BlockRef>& active_blocks,
    uint32_t stride, uint32_t iter) {
  std::vector<uint32_t> applied(states->size(), 0);
  for (size_t i = 0; i < active_blocks.size(); ++i) {
    if (i % stride != iter % stride) continue;
    const BlockRef& ref = active_blocks[i];
    for (size_t k = 0; k < states->size(); ++k) {
      MultiKState& state = (*states)[k];
      if (state.finished) continue;
      const uint32_t cur = state.ctx.pass_assignment[ref.c][ref.b];
      const uint32_t next = state.new_passes[i];
      if (next == cur) continue;
      state.ctx.ApplyMove(ref, cur, next);
      ++applied[k];
    }
  }
  return applied;
}

std::vector<FixedPointCost> ComputeCurrentCosts(std::vector<MultiKState>* states,
                                                ThreadPool* pool) {
  std::vector<FixedPointCost> costs(states->size(), 0);
  auto run_one_k = [&](size_t ki) {
    MultiKState& state = (*states)[ki];
    costs[ki] = state.finished ? state.current_cost : state.ctx.TotalCost();
  };
  if (pool != nullptr && states->size() > 1) {
    (void)RunOnPool(
        pool, 0, static_cast<uint32_t>(states->size()), ThreadPool::NoInit,
        [&](uint32_t ki, size_t /*thread*/) -> Status {
          run_one_k(ki);
          return true;
        },
        "AssignPassesGreedyAllKCost");
  } else {
    for (size_t ki = 0; ki < states->size(); ++ki) {
      run_one_k(ki);
    }
  }
  return costs;
}

}  // namespace

AssignPassesRangeResult AssignPassesGreedyAllK(
    const JPEGOptData& d, const ActiveRawBins& active,
    uint32_t min_num_passes, uint32_t max_num_passes, ThreadPool* pool) {
  AssignPassesRangeResult out;
  out.min_num_passes = min_num_passes;
  out.results.resize(max_num_passes - min_num_passes + 1);

  const auto start_total = PlannerClock::now();
  if (active.active_bins.empty()) {
    for (AssignPassesResult& result : out.results) {
      for (uint32_t c = 0; c < kNumCh; ++c) {
        result.pass_assignment[c].assign(d.num_blocks[c], 0);
      }
      result.timings.total_ns = ElapsedNanos(start_total, PlannerClock::now());
    }
    out.shared_timings.total_ns = ElapsedNanos(start_total, PlannerClock::now());
    return out;
  }

  std::vector<BlockRef> active_blocks;
  for (uint32_t c = 0; c < d.channels; ++c) {
    for (uint32_t b = 0; b < d.num_blocks[c]; ++b) {
      if (d.block_offsets[c][b] == d.block_offsets[c][b + 1]) continue;
      active_blocks.push_back({static_cast<uint16_t>(c), b});
    }
  }
  if (active_blocks.empty()) {
    for (AssignPassesResult& result : out.results) {
      for (uint32_t c = 0; c < kNumCh; ++c) {
        result.pass_assignment[c].assign(d.num_blocks[c], 0);
      }
      result.timings.total_ns = ElapsedNanos(start_total, PlannerClock::now());
    }
    out.shared_timings.total_ns = ElapsedNanos(start_total, PlannerClock::now());
    return out;
  }

  if (min_num_passes == 1 && max_num_passes >= 1) {
    AssignPassesResult& result = out.results[0];
    for (uint32_t c = 0; c < kNumCh; ++c) {
      result.pass_assignment[c].assign(d.num_blocks[c], 0);
    }
  }

  std::vector<MultiKState> states;
  const uint32_t first_fused_k = std::max<uint32_t>(2, min_num_passes);
  if (max_num_passes >= first_fused_k) {
    states.reserve(max_num_passes - first_fused_k + 1);
    for (uint32_t k = first_fused_k; k <= max_num_passes; ++k) {
      states.emplace_back(d, active, k, active_blocks.size());
    }
  }
  if (states.empty()) {
    out.shared_timings.total_ns = ElapsedNanos(start_total, PlannerClock::now());
    for (AssignPassesResult& result : out.results) {
      result.timings = out.shared_timings;
    }
    return out;
  }

  const auto start_init = PlannerClock::now();
  for (MultiKState& state : states) {
    state.ctx.InitPassAssignmentSimple();
    state.ctx.InitNZPredictorState();
  }
  {
    const std::vector<FixedPointCost> initial_costs =
        ComputeCurrentCosts(&states, pool);
    for (size_t i = 0; i < states.size(); ++i) {
      states[i].current_cost = initial_costs[i];
    }
  }
  fprintf(stderr, "PLANNER: [all-k] Initializing clusters took %.2f ms\n",
          NanosToMs(ElapsedNanos(start_init, PlannerClock::now())));
  fflush(stderr);

  const int move_width =
      static_cast<int>(std::to_string(TotalBlockCount(d)).size());
  const bool use_batch =
      (pool != nullptr && active_blocks.size() > kLargeImageThreshold);
  PrintMoveTableHeader(states, move_width);

  uint32_t iter = 0;
  {
    const auto start_seq = PlannerClock::now();
    const std::vector<SequentialSweepResult> seq_stats =
        FusedSequentialIter(&states, active_blocks, pool);
    out.shared_timings.sequential_ns +=
        ElapsedNanos(start_seq, PlannerClock::now());
    MajoritySequentialImproved(&states, seq_stats);
    ++iter;
    std::vector<uint32_t> seq_moves(states.size(), 0);
    for (size_t i = 0; i < states.size(); ++i) seq_moves[i] = seq_stats[i].moves;
    PrintMoveTableRow("seq-1", states, seq_moves, move_width);
  }

  constexpr uint32_t kUnconditionalBatchIterations = 5;
  constexpr uint32_t kBatchPatience = 2;
  constexpr uint32_t kBatchStride = 1;

  if (use_batch) {
    for (uint32_t n = 0;
         n < kUnconditionalBatchIterations && iter < kMaxIters &&
         AnyActiveK(states);
         ++n) {
      const auto start_batch = PlannerClock::now();
      const std::vector<uint32_t> batch_moves =
          FusedScoreBatchMoves(&states, active_blocks, pool);
      if (std::all_of(batch_moves.begin(), batch_moves.end(),
                      [](uint32_t v) { return v == 0; })) {
        break;
      }
      const std::vector<uint32_t> applied =
          FusedApplyBatchMoves(&states, active_blocks, kBatchStride, iter);
      out.shared_timings.batch_ns +=
          ElapsedNanos(start_batch, PlannerClock::now());
      const std::vector<FixedPointCost> new_costs =
          ComputeCurrentCosts(&states, pool);
      MajorityBatchImproved(&states, applied, new_costs);
      ++iter;
      char label[32];
      std::snprintf(label, sizeof(label), "b-batch-%u", iter);
      PrintMoveTableRow(label, states, applied, move_width);
    }

    uint32_t batch_stale_count = 0;
    while (iter < kMaxIters && AnyActiveK(states) &&
           batch_stale_count < kBatchPatience) {
      const auto start_batch = PlannerClock::now();
      const std::vector<uint32_t> batch_moves =
          FusedScoreBatchMoves(&states, active_blocks, pool);
      if (std::all_of(batch_moves.begin(), batch_moves.end(),
                      [](uint32_t v) { return v == 0; })) {
        break;
      }
      const std::vector<uint32_t> applied =
          FusedApplyBatchMoves(&states, active_blocks, kBatchStride, iter);
      out.shared_timings.batch_ns +=
          ElapsedNanos(start_batch, PlannerClock::now());
      const std::vector<FixedPointCost> new_costs =
          ComputeCurrentCosts(&states, pool);
      const bool good_batch = MajorityBatchImproved(&states, applied, new_costs);
      ++iter;
      char batch_label[32];
      std::snprintf(batch_label, sizeof(batch_label), "b-batch-%u", iter);
      PrintMoveTableRow(batch_label, states, applied, move_width);
      if (good_batch) {
        batch_stale_count = 0;
        continue;
      }

      ++batch_stale_count;
      if (iter >= kMaxIters) break;
      const auto start_seq = PlannerClock::now();
      const std::vector<SequentialSweepResult> seq_stats =
          FusedSequentialIter(&states, active_blocks, pool);
      out.shared_timings.sequential_ns +=
          ElapsedNanos(start_seq, PlannerClock::now());
      MajoritySequentialImproved(&states, seq_stats);
      ++iter;
      std::vector<uint32_t> seq_moves(states.size(), 0);
      for (size_t i = 0; i < states.size(); ++i) seq_moves[i] = seq_stats[i].moves;
      char seq_label[32];
      std::snprintf(seq_label, sizeof(seq_label), "b-seq-%u", iter);
      PrintMoveTableRow(seq_label, states, seq_moves, move_width);
    }
  }

  uint32_t seq_bad_streak = 0;
  while (iter < kMaxIters && AnyActiveK(states)) {
    const auto start_seq = PlannerClock::now();
    const std::vector<SequentialSweepResult> seq_stats =
        FusedSequentialIter(&states, active_blocks, pool);
    out.shared_timings.sequential_ns +=
        ElapsedNanos(start_seq, PlannerClock::now());
    const bool good_seq = MajoritySequentialImproved(&states, seq_stats);
    ++iter;
    std::vector<uint32_t> seq_moves(states.size(), 0);
    for (size_t i = 0; i < states.size(); ++i) seq_moves[i] = seq_stats[i].moves;
    char seq_label[32];
    std::snprintf(seq_label, sizeof(seq_label), "s-seq-%u", iter);
    PrintMoveTableRow(seq_label, states, seq_moves, move_width);
    if (iter >= kMaxIters || !AnyActiveK(states)) break;

    if (good_seq) {
      seq_bad_streak = 0;
      continue;
    }

    ++seq_bad_streak;
    if (use_batch && seq_bad_streak >= 2) {
      const auto start_batch = PlannerClock::now();
      const std::vector<uint32_t> batch_moves =
          FusedScoreBatchMoves(&states, active_blocks, pool);
      if (std::all_of(batch_moves.begin(), batch_moves.end(),
                      [](uint32_t v) { return v == 0; })) {
        break;
      }
      const std::vector<uint32_t> applied =
          FusedApplyBatchMoves(&states, active_blocks, kBatchStride, iter);
      out.shared_timings.batch_ns +=
          ElapsedNanos(start_batch, PlannerClock::now());
      const std::vector<FixedPointCost> new_costs =
          ComputeCurrentCosts(&states, pool);
      MajorityBatchImproved(&states, applied, new_costs);
      ++iter;
      char batch_label[32];
      std::snprintf(batch_label, sizeof(batch_label), "s-batch-%u", iter);
      PrintMoveTableRow(batch_label, states, applied, move_width);
      seq_bad_streak = 0;
    }
  }

  out.shared_timings.total_ns = ElapsedNanos(start_total, PlannerClock::now());
  for (MultiKState& state : states) {
    AssignPassesResult& result =
        out.results[state.num_passes - min_num_passes];
    result.pass_assignment = std::move(state.ctx.pass_assignment);
    result.timings = out.shared_timings;
  }
  for (AssignPassesResult& result : out.results) {
    result.timings = out.shared_timings;
  }
  return out;
}

}  // namespace jxl
