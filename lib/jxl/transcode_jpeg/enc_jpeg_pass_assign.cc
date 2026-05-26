// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_assign.h"

#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cstdio>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "lib/jxl/base/data_parallel.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_histogram.h"

namespace jxl {

namespace {

// Iteration cap for the sequential/batch refinement loops.
constexpr uint32_t kMaxIters = 100;
// Above this many active blocks, the batch parallel refinement phase is used.
constexpr uint32_t kLargeImageThreshold = 1u << 15;
// Number of blocks scored per chunk in `ScoreBatchMoves`.
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

}  // namespace

ActiveRawBins BuildActiveRawBins(const JPEGOptData& d) {
  ActiveRawBins out;
  const size_t raw_bin_count = static_cast<size_t>(d.channels) * kMaxACSymbolCount;
  out.raw_to_compact.assign(raw_bin_count, kInvalidCompactH);
  out.active_bins.reserve(d.ACHistogram().dense_to_zdcvalue.size());
  for (uint32_t c = 0; c < d.channels; ++c) {
    for (uint32_t b = 0; b < d.num_blocks[c]; ++b) {
      for (uint32_t pi = d.block_offsets[c][b];
           pi < d.block_offsets[c][b + 1]; ++pi) {
        const ACBin bin = d.block_bins[c][pi];
        if (out.raw_to_compact[bin] != kInvalidCompactH) continue;
        out.raw_to_compact[bin] = static_cast<uint32_t>(out.active_bins.size());
        out.active_bins.push_back(bin);
        out.compact_to_czdc.push_back(
            static_cast<uint16_t>(JPEGOptData::ACBinCZDC(bin)));
      }
    }
  }
  return out;
}

// --- PassAssignmentCtx ---

PassAssignmentCtx::PassAssignmentCtx(const JPEGOptData& d,
                                     const ActiveRawBins& active,
                                     uint32_t num_passes)
    : d(d),
      active(active),
      num_passes(num_passes),
      M(static_cast<uint32_t>(active.active_bins.size())),
      czdc_size(d.channels * kZeroDensityContextCount),
      hist_h(static_cast<size_t>(M) * num_passes, 0),
      hist_N(static_cast<size_t>(czdc_size) * num_passes, 0),
      nz_hist_h(static_cast<size_t>(kNZHistogramsSize) * num_passes, 0),
      nz_hist_N(static_cast<size_t>(kJPEGNonZeroBuckets) * num_passes, 0) {
  for (uint32_t c = 0; c < kNumCh; ++c)
    pass_assignment[c].assign(d.num_blocks[c], /*0*/ c);
}

void PassAssignmentCtx::InitPassAssignmentSimple() {
  for (uint32_t c = 0; c < d.channels; ++c) {
    for (uint32_t b = 0; b < d.num_blocks[c]; ++b) {
      const uint32_t pass = b * num_passes / d.num_blocks[c];  // c;//
      pass_assignment[c][b] = static_cast<uint8_t>(pass);
      ForEachBlockBin(d, c, b, [&](ACBin bin) {
        const uint32_t compact_id = active.raw_to_compact[bin];
        const uint32_t czdc = active.compact_to_czdc[compact_id];
        ++hist_h[static_cast<size_t>(compact_id) * num_passes + pass];
        ++hist_N[static_cast<size_t>(czdc) * num_passes + pass];
      });
    }
  }
}

void PassAssignmentCtx::InitPassAssignmentHistogramAware() {
  struct GrowthScratch {
    std::vector<uint16_t> touched_czdc;
    std::vector<uint32_t> czdc_counts;
  };

  for (uint32_t c = 0; c < d.channels; ++c) {
    if (d.num_blocks[c] == 0) continue;

    const uint32_t w = d.block_grid_w[c];
    const uint32_t h = d.block_grid_h[c];
    const uint32_t seeded_passes =
        std::min<uint32_t>(num_passes, d.num_blocks[c]);
    std::vector<uint8_t> assigned(d.num_blocks[c], 0);
    std::vector<uint32_t> pass_sizes(num_passes, 0);
    std::vector<GrowthScratch> growth_scratch(num_passes);
    for (uint32_t pass = 0; pass < num_passes; ++pass) {
      growth_scratch[pass].touched_czdc.reserve(64);
      growth_scratch[pass].czdc_counts.assign(czdc_size, 0);
    }

    auto growth_cost = [&](uint32_t b, uint32_t pass,
                           GrowthScratch* scratch) -> FixedPointCost {
      FixedPointCost cost = 0;
      scratch->touched_czdc.clear();
      ForEachBlockBin(d, c, b, [&](ACBin bin) {
        const uint32_t compact_id = active.raw_to_compact[bin];
        const uint32_t czdc = active.compact_to_czdc[compact_id];
        const uint32_t h_count =
            hist_h[static_cast<size_t>(compact_id) * num_passes + pass];
        cost += d.ftab[h_count + 1] - d.ftab[h_count];
        if (scratch->czdc_counts[czdc]++ == 0) {
          scratch->touched_czdc.push_back(static_cast<uint16_t>(czdc));
        }
      });
      for (uint16_t czdc : scratch->touched_czdc) {
        const uint32_t count = scratch->czdc_counts[czdc];
        scratch->czdc_counts[czdc] = 0;
        const uint32_t n_count =
            hist_N[static_cast<size_t>(czdc) * num_passes + pass];
        cost += d.ftab[n_count + count] - d.ftab[n_count];
      }
      return cost;
    };

    // Set to true to use O(N·P) raster-order greedy scan instead of
    // O(N log N) seeded spatial growth.
    static constexpr bool kLinearScan = false;

    if (kLinearScan) {
      for (uint32_t b = 0; b < d.num_blocks[c]; ++b) {
        uint32_t best_pass = 0;
        FixedPointCost best_cost = growth_cost(b, 0, &growth_scratch[0]);
        for (uint32_t pass = 1; pass < num_passes; ++pass) {
          const FixedPointCost cost =
              growth_cost(b, pass, &growth_scratch[pass]);
          if (cost < best_cost) {
            best_cost = cost;
            best_pass = pass;
          }
        }
        pass_assignment[c][b] = static_cast<uint8_t>(best_pass);
        ++pass_sizes[best_pass];
        ForEachBlockBin(d, c, b, [&](ACBin bin) {
          const uint32_t compact_id = active.raw_to_compact[bin];
          const uint32_t czdc = active.compact_to_czdc[compact_id];
          ++hist_h[static_cast<size_t>(compact_id) * num_passes + best_pass];
          ++hist_N[static_cast<size_t>(czdc) * num_passes + best_pass];
        });
      }
    } else {
      using FrontierEntry = std::pair<FixedPointCost, uint32_t>;
      using FrontierHeap =
          std::priority_queue<FrontierEntry, std::vector<FrontierEntry>,
                              std::greater<FrontierEntry>>;
      std::vector<FrontierHeap> frontier(num_passes);

      auto add_block_to_pass = [&](uint32_t b, uint32_t pass) {
        assigned[b] = 1;
        pass_assignment[c][b] = static_cast<uint8_t>(pass);
        ++pass_sizes[pass];
        ForEachBlockBin(d, c, b, [&](ACBin bin) {
          const uint32_t compact_id = active.raw_to_compact[bin];
          const uint32_t czdc = active.compact_to_czdc[compact_id];
          ++hist_h[static_cast<size_t>(compact_id) * num_passes + pass];
          ++hist_N[static_cast<size_t>(czdc) * num_passes + pass];
        });
        const uint32_t x = b % w;
        const uint32_t y = b / w;
        auto push_neighbor = [&](uint32_t nx, uint32_t ny) {
          const uint32_t nb = ny * w + nx;
          if (!assigned[nb]) {
            frontier[pass].emplace(growth_cost(nb, pass, &growth_scratch[pass]),
                                   nb);
          }
        };
        if (x > 0) push_neighbor(x - 1, y);
        if (x + 1 < w) push_neighbor(x + 1, y);
        if (y > 0) push_neighbor(x, y - 1);
        if (y + 1 < h) push_neighbor(x, y + 1);
      };

      auto choose_seed = [&](uint32_t pass) {
        const uint32_t target = std::min<uint32_t>(
            d.num_blocks[c] - 1,
            ((2 * pass + 1) * d.num_blocks[c]) / (2 * seeded_passes));
        if (!assigned[target]) return target;
        for (uint32_t radius = 1; radius < d.num_blocks[c]; ++radius) {
          if (target >= radius && !assigned[target - radius])
            return target - radius;
          if (target + radius < d.num_blocks[c] && !assigned[target + radius])
            return target + radius;
        }
        return target;
      };

      uint32_t assigned_count = 0;
      for (uint32_t pass = 0; pass < seeded_passes; ++pass) {
        const uint32_t seed = choose_seed(pass);
        if (assigned[seed]) continue;
        add_block_to_pass(seed, pass);
        ++assigned_count;
      }

      auto refresh_frontier_top = [&](uint32_t pass) {
        while (!frontier[pass].empty()) {
          const uint32_t block = frontier[pass].top().second;
          const FixedPointCost stored_cost = frontier[pass].top().first;
          if (assigned[block]) {
            frontier[pass].pop();
            continue;
          }
          const FixedPointCost refreshed_cost =
              growth_cost(block, pass, &growth_scratch[pass]);
          if (refreshed_cost != stored_cost) {
            frontier[pass].pop();
            frontier[pass].emplace(refreshed_cost, block);
            continue;
          }
          break;
        }
      };

      while (assigned_count < d.num_blocks[c]) {
        uint32_t best_pass = 0;
        for (uint32_t p = 1; p < seeded_passes; ++p)
          if (pass_sizes[p] < pass_sizes[best_pass]) best_pass = p;

        refresh_frontier_top(best_pass);

        if (!frontier[best_pass].empty()) {
          const uint32_t best_block = frontier[best_pass].top().second;
          frontier[best_pass].pop();
          add_block_to_pass(best_block, best_pass);
        } else {
          for (uint32_t b = 0; b < d.num_blocks[c]; ++b) {
            if (!assigned[b]) {
              add_block_to_pass(b, best_pass);
              break;
            }
          }
        }
        ++assigned_count;
      }
    }
  }
}

void PassAssignmentCtx::InitNZPredictorState() {
  for (uint32_t c = 0; c < d.channels; ++c) {
    nz_pred_bucket[c].resize(d.num_blocks[c], 0);
    for (uint32_t b = 0; b < d.num_blocks[c]; ++b) {
      const uint32_t pass = pass_assignment[c][b];
      const uint32_t pb = PredictNZBucket(c, b, pass, nullptr, 0);
      nz_pred_bucket[c][b] = static_cast<uint8_t>(pb);
      ++nz_hist_N[static_cast<size_t>(pass) * kJPEGNonZeroBuckets + pb];
      ++nz_hist_h[static_cast<size_t>(pass) * kNZHistogramsSize +
                  NZHistogramIndex(pb, d.block_nonzeros[c][b])];
    }
  }
}

AssignScratch PassAssignmentCtx::MakeScratch() const {
  AssignScratch s;
  s.delta.resize(num_passes);
  s.touched_czdc.reserve(128);
  s.czdc_counts.assign(czdc_size, 0);
  return s;
}

FixedPointCost PassAssignmentCtx::TotalCost() const {
  FixedPointCost cost = 0;
  for (uint32_t czdc = 0; czdc < czdc_size; ++czdc) {
    const uint32_t* n_row = &hist_N[static_cast<size_t>(czdc) * num_passes];
    for (uint32_t pass = 0; pass < num_passes; ++pass)
      if (n_row[pass] != 0) cost += d.ftab[n_row[pass]];
  }
  for (uint32_t compact_id = 0; compact_id < M; ++compact_id) {
    const uint32_t* h_row =
        &hist_h[static_cast<size_t>(compact_id) * num_passes];
    for (uint32_t pass = 0; pass < num_passes; ++pass)
      if (h_row[pass] != 0) cost -= d.ftab[h_row[pass]];
  }
  for (uint32_t pass = 0; pass < num_passes; ++pass) {
    const uint32_t* nz_n =
        &nz_hist_N[static_cast<size_t>(pass) * kJPEGNonZeroBuckets];
    for (uint32_t pb = 0; pb < kJPEGNonZeroBuckets; ++pb)
      if (nz_n[pb] != 0) cost += d.NZFTab(nz_n[pb]);
    const uint32_t* nz_h =
        &nz_hist_h[static_cast<size_t>(pass) * kNZHistogramsSize];
    for (uint32_t idx = 0; idx < kNZHistogramsSize; ++idx)
      if (nz_h[idx] != 0) cost -= d.NZFTab(nz_h[idx]);
  }
  return cost;
}

SequentialSweepResult PassAssignmentCtx::SequentialIter(
    const std::vector<BlockRef>& active_blocks, AssignScratch& scratch) {
  SequentialSweepResult result;
  for (const BlockRef& ref : active_blocks) {
    const uint32_t cur = pass_assignment[ref.c][ref.b];
    FixedPointCost best_delta = 0;
    const uint32_t best = FindBestPass(ref, cur, &scratch, &best_delta);
    if (best == cur) continue;
    ApplyMove(ref, cur, best);
    ++result.moves;
    result.delta_cost += best_delta;
  }
  return result;
}

uint32_t PassAssignmentCtx::ScoreBatchMoves(
    const std::vector<BlockRef>& active_blocks,
    std::vector<uint8_t>& new_passes,
    std::vector<AssignScratch>& scratch_pool, ThreadPool* pool) {
  std::atomic<uint32_t> batch_moves(0);
  const uint32_t num_chunks = static_cast<uint32_t>(
      (active_blocks.size() + kBatchChunkSize - 1) / kBatchChunkSize);
  if (!RunOnPool(
          pool, 0, num_chunks,
          [&](size_t num_threads) -> Status {
            scratch_pool.clear();
            scratch_pool.reserve(num_threads);
            for (size_t i = 0; i < num_threads; ++i)
              scratch_pool.push_back(MakeScratch());
            return true;
          },
          [&](uint32_t chunk, size_t thread_id) -> Status {
            uint32_t chunk_moves = 0;
            AssignScratch& ts = scratch_pool[thread_id];
            const size_t begin = static_cast<size_t>(chunk) * kBatchChunkSize;
            const size_t end =
                std::min(begin + kBatchChunkSize, active_blocks.size());
            for (size_t i = begin; i < end; ++i) {
              const BlockRef& ref = active_blocks[i];
              const uint32_t cur = pass_assignment[ref.c][ref.b];
              const uint32_t best = FindBestPass(ref, cur, &ts);
              new_passes[i] = static_cast<uint8_t>(best);
              if (best != cur) ++chunk_moves;
            }
            batch_moves.fetch_add(chunk_moves, std::memory_order_relaxed);
            return true;
          },
          "AssignPassesGreedyBatch")) {
    return 0;
  }
  return batch_moves.load(std::memory_order_seq_cst);
}

uint32_t PassAssignmentCtx::ApplyBatchMoves(
    const std::vector<BlockRef>& active_blocks,
    const std::vector<uint8_t>& new_passes, uint32_t stride, uint32_t iter) {
  uint32_t n = 0;
  for (size_t i = 0; i < active_blocks.size(); ++i) {
    if (i % stride != iter % stride) continue;
    const BlockRef& ref = active_blocks[i];
    const uint32_t cur = pass_assignment[ref.c][ref.b];
    if (new_passes[i] == cur) continue;
    ApplyMove(ref, cur, new_passes[i]);
    ++n;
  }
  return n;
}

int64_t PassAssignmentCtx::RunBatchPassRefinement(
    const std::vector<BlockRef>& active_blocks, ThreadPool* pool,
    uint32_t min_moves, uint32_t& iter, uint32_t& seq_moves) {
  constexpr uint32_t kUnconditionalBatchIterations = 5;
  constexpr uint32_t kBatchPatience = 2;
  constexpr uint32_t kBatchStride = 1;

  const auto start = PlannerClock::now();

  std::vector<uint8_t> new_passes(active_blocks.size(), 0);
  std::vector<AssignScratch> scratch_pool;
  FixedPointCost best_cost = TotalCost();
  FixedPointCost current_cost = best_cost;
  PassAssignment snap_assignment = pass_assignment;
  std::vector<uint32_t> snap_hist_h = hist_h;
  std::vector<uint32_t> snap_hist_N = hist_N;
  uint32_t stale_count = 0;
  uint32_t applied = 1;
  uint32_t best_moves = std::numeric_limits<uint32_t>::max();

  while (iter < kMaxIters && applied > 0 && stale_count < kBatchPatience &&
         seq_moves > min_moves) {
    const uint32_t batch_moves =
        ScoreBatchMoves(active_blocks, new_passes, scratch_pool, pool);
    if (batch_moves == 0) break;

    applied =
        ApplyBatchMoves(active_blocks, new_passes, kBatchStride, iter);
    ++iter;
    fprintf(stderr,
            "PLANNER: [bicluster] Batch phase %u took %.2f ms for %u moves\n",
            iter, NanosToMs(ElapsedNanos(start, PlannerClock::now())), applied);
    fflush(stderr);
    if (iter < kUnconditionalBatchIterations) continue;

    if (batch_moves < best_moves) {
      if (batch_moves < best_moves - (best_moves >> 4)) {
        stale_count = 0;
        best_moves = batch_moves;
        current_cost = TotalCost();
        if (current_cost < best_cost) {
          best_cost = current_cost;
          snap_assignment = pass_assignment;
          snap_hist_h = hist_h;
          snap_hist_N = hist_N;
        }
        continue;
      }
      best_moves = batch_moves;
    } else {
      ++stale_count;
    }

    AssignScratch scratch = MakeScratch();
    seq_moves = SequentialIter(active_blocks, scratch).moves;
    ++iter;
    fprintf(
        stderr,
        "PLANNER: [bicluster] Sequential phase %u took %.2f ms for %u moves\n",
        iter, NanosToMs(ElapsedNanos(start, PlannerClock::now())), seq_moves);
    fflush(stderr);

    current_cost = TotalCost();
    if (current_cost < best_cost) {
      best_cost = current_cost;
      snap_assignment = pass_assignment;
      snap_hist_h = hist_h;
      snap_hist_N = hist_N;
    }
  }

  if (current_cost > best_cost) {
    pass_assignment = std::move(snap_assignment);
    hist_h = std::move(snap_hist_h);
    hist_N = std::move(snap_hist_N);
  }
  return ElapsedNanos(start, PlannerClock::now());
}

int64_t PassAssignmentCtx::RunSequentialPassRefinement(
    const std::vector<BlockRef>& active_blocks, ThreadPool* pool,
    uint32_t min_moves, uint32_t& iter) {
  constexpr uint32_t kBatchStride = 1;
  std::vector<uint8_t> new_passes(active_blocks.size(), 0);
  std::vector<AssignScratch> scratch_pool;

  const auto start = PlannerClock::now();
  AssignScratch scratch = MakeScratch();
  uint32_t best_moves = std::numeric_limits<uint32_t>::max();
  uint32_t seq_moves = min_moves + 1;

  while (iter < kMaxIters && seq_moves > min_moves) {
    seq_moves = SequentialIter(active_blocks, scratch).moves;
    ++iter;
    fprintf(stderr,
            "PLANNER: [bicluster] Sequential global phase %u took %.2f ms for "
            "%u moves\n",
            iter, NanosToMs(ElapsedNanos(start, PlannerClock::now())),
            seq_moves);
    fflush(stderr);

    if (seq_moves > best_moves) {
      const uint32_t batch_moves =
          ScoreBatchMoves(active_blocks, new_passes, scratch_pool, pool);
      if (batch_moves == 0) break;
      const uint32_t applied =
          ApplyBatchMoves(active_blocks, new_passes, kBatchStride, iter);
      ++iter;
      fprintf(stderr,
              "PLANNER: [bicluster] Batch global phase %u took %.2f ms for %u "
              "moves\n",
              iter, NanosToMs(ElapsedNanos(start, PlannerClock::now())),
              applied);
      fflush(stderr);
    } else {
      best_moves = seq_moves;
    }
  }
  return ElapsedNanos(start, PlannerClock::now());
}

// --- AssignPassesGreedy ---

AssignPassesResult AssignPassesGreedy(const JPEGOptData& d,
                                      const ActiveRawBins& active,
                                      uint32_t num_passes, ThreadPool* pool) {
  const auto start_total = PlannerClock::now();
  AssignPassesResult result;
  if (num_passes <= 1 || active.active_bins.empty()) {
    for (uint32_t c = 0; c < kNumCh; ++c)
      result.pass_assignment[c].assign(d.num_blocks[c], 0);
    result.timings.total_ns = ElapsedNanos(start_total, PlannerClock::now());
    return result;
  }

  std::vector<BlockRef> active_blocks;
  for (uint32_t c = 0; c < d.channels; ++c) {
    for (uint32_t b = 0; b < d.num_blocks[c]; ++b) {
      if (d.block_offsets[c][b] == d.block_offsets[c][b + 1]) continue;
      active_blocks.push_back({static_cast<uint16_t>(c), b});
    }
  }

  PassAssignmentCtx ctx(d, active, num_passes);
  if (active_blocks.empty() /* || num_passes == 3*/) {
    result.pass_assignment = std::move(ctx.pass_assignment);
    result.timings.total_ns = ElapsedNanos(start_total, PlannerClock::now());
    return result;
  }

  const uint32_t min_moves = std::max<uint32_t>(1, active_blocks.size() >> 13);

  const auto start_init = PlannerClock::now();
  // ctx.InitPassAssignmentHistogramAware();
  ctx.InitPassAssignmentSimple();
  ctx.InitNZPredictorState();
  fprintf(stderr, "PLANNER: [bicluster] Initializing clusters took %.2f ms\n",
          NanosToMs(ElapsedNanos(start_init, PlannerClock::now())));
  fflush(stderr);

  AssignScratch scratch = ctx.MakeScratch();
  uint32_t iter = 0;
  uint32_t seq_moves = ctx.SequentialIter(active_blocks, scratch).moves;
  fprintf(stderr,
          "PLANNER: [bicluster] Initial phase 1 took %.2f ms for %u moves\n",
          NanosToMs(ElapsedNanos(start_init, PlannerClock::now())), seq_moves);
  fflush(stderr);
  ++iter;

  if (pool != nullptr && active_blocks.size() > kLargeImageThreshold) {
    result.timings.batch_ns = ctx.RunBatchPassRefinement(
        active_blocks, pool, min_moves, iter, seq_moves);
  }

  result.timings.sequential_ns =
      ctx.RunSequentialPassRefinement(active_blocks, pool, min_moves, iter);

  result.pass_assignment = std::move(ctx.pass_assignment);
  result.timings.total_ns = ElapsedNanos(start_total, PlannerClock::now());
  return result;
}

}  // namespace jxl
