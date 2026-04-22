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
#include <cmath>
#include <cstring>
#include <cstdio>
#include <limits>
#include <vector>

#include "lib/jxl/base/data_parallel.h"
#include "lib/jxl/enc_ans_params.h"
#include "lib/jxl/enc_cluster.h"
#include "lib/jxl/enc_context_map.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_cluster.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_utils.h"

namespace jxl {

namespace {

constexpr double kSequentialStopDropPct = 0.0005;

struct MultiKState {
  uint32_t num_passes;
  PassAssignmentCtx ctx;
  AssignScratch scratch;
  std::vector<uint8_t> new_passes;
  uint32_t min_moves;
  uint32_t seq_moves;
  FixedPointCost initial_cost;
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
        initial_cost(0),
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

double CostDropPercent(FixedPointCost old_cost, FixedPointCost new_cost) {
  if (old_cost <= 0) return 0.0;
  return 100.0 * static_cast<double>(old_cost - new_cost) /
         static_cast<double>(old_cost);
}

void PrintMoveTableHeader(const std::vector<MultiKState>& states,
                          int column_width) {
  fprintf(stderr, "PLANNER: [all-k] %-12s", "step/passes");
  for (const auto& state : states) {
    fprintf(stderr, " %*u", column_width, state.num_passes);
  }
  fprintf(stderr, "\n");
  fflush(stderr);
}

void PrintMoveTableRow(const char* label, const std::vector<MultiKState>& states,
                       const std::vector<double>& drop_pct, int column_width) {
  fprintf(stderr, "PLANNER: [all-k] %-12s", label);
  for (size_t i = 0; i < states.size(); ++i) {
    fprintf(stderr, " %*.3f%%", column_width - 1, drop_pct[i]);
  }
  fprintf(stderr, "\n");
  fflush(stderr);
}

void PrintFinalCostRow(const std::vector<MultiKState>& states,
                       const FixedPointCost* one_pass_cost) {
  int column_width = 1;
  if (one_pass_cost != nullptr) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.2f", bit_cost(*one_pass_cost));
    column_width = std::max<int>(column_width, static_cast<int>(std::strlen(buf)));
  }
  for (const auto& state : states) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.2f", bit_cost(state.current_cost));
    column_width = std::max<int>(column_width, static_cast<int>(std::strlen(buf)));
  }

  fprintf(stderr, "PLANNER: [all-k] %-12s", "cost(bits)");
  if (one_pass_cost != nullptr) {
    fprintf(stderr, " 1=%*.2f", column_width, bit_cost(*one_pass_cost));
  }
  for (const auto& state : states) {
    fprintf(stderr, " %*.2f", column_width, bit_cost(state.current_cost));
  }
  fprintf(stderr, "\n");
  fflush(stderr);
}

FixedPointCost ClusteredHistogramProxyCost(const PassAssignmentCtx& ctx) {
  HistogramParams params;
  params.clustering = HistogramParams::ClusteringType::kBest;

  std::vector<std::array<uint32_t, kACTokenCount>> token_counts(ctx.czdc_size);
  const auto& dense_to_symbol = ctx.d.ACHistogram().dense_to_zdcvalue;

  FixedPointCost unclustered_token_cost = 0;
  FixedPointCost clustered_token_cost = 0;
  std::vector<Histogram> histograms;
  std::vector<Histogram> clustered;
  std::vector<uint32_t> histogram_symbols;
  for (uint32_t pass = 0; pass < ctx.num_passes; ++pass) {
    histograms.clear();
    histograms.reserve(ctx.czdc_size + kJPEGNonZeroBuckets);
    std::fill(token_counts.begin(), token_counts.end(),
              std::array<uint32_t, kACTokenCount>{});
    for (uint32_t compact_id = 0; compact_id < ctx.M; ++compact_id) {
      const uint32_t freq =
          ctx.hist_h[static_cast<size_t>(compact_id) * ctx.num_passes + pass];
      if (freq == 0) continue;
      const uint16_t czdc = ctx.active.compact_to_czdc[compact_id];
      const SignallingHistSymbol sym = ctx.d.SignallingHistSymbolFromSymbol(
          dense_to_symbol[ctx.active.active_bins[compact_id]]);
      token_counts[czdc][sym.token] += freq;
    }

    for (uint32_t czdc = 0; czdc < ctx.czdc_size; ++czdc) {
      uint32_t max_token = 0;
      size_t total = 0;
      for (uint32_t token = 0; token < kACTokenCount; ++token) {
        if (token_counts[czdc][token] == 0) continue;
        max_token = token;
        total += token_counts[czdc][token];
      }
      if (total == 0) continue;

      Histogram h(max_token + 1);
      h.total_count = total;
      for (uint32_t token = 0; token <= max_token; ++token) {
        h.counts[token] = static_cast<ANSHistBin>(token_counts[czdc][token]);
      }
      unclustered_token_cost +=
          static_cast<FixedPointCost>(h.ShannonEntropy() * kFScale);
      histograms.push_back(std::move(h));
    }

    for (uint32_t pb = 0; pb < kJPEGNonZeroBuckets; ++pb) {
      const uint32_t total = ctx.nz_hist_N[static_cast<size_t>(pass) *
                                               kJPEGNonZeroBuckets +
                                           pb];
      if (total == 0) continue;

      uint32_t max_nz = 0;
      const size_t base =
          static_cast<size_t>(pass) * kNZHistogramsSize + pb * kJPEGNonZeroRange;
      for (uint32_t nz = 0; nz < kJPEGNonZeroRange; ++nz) {
        if (ctx.nz_hist_h[base + nz] != 0) max_nz = nz;
      }
      Histogram h(max_nz + 1);
      h.total_count = total;
      for (uint32_t nz = 0; nz <= max_nz; ++nz) {
        h.counts[nz] = static_cast<ANSHistBin>(ctx.nz_hist_h[base + nz]);
      }
      unclustered_token_cost +=
          static_cast<FixedPointCost>(h.ShannonEntropy() * kFScale);
      histograms.push_back(std::move(h));
    }

    if (histograms.empty()) continue;

    clustered.clear();
    histogram_symbols.clear();
    if (!ClusterHistograms(params, histograms, kClustersLimit, &clustered,
                           &histogram_symbols)) {
      JXL_WARNING("ClusterHistograms failed in AssignPassesGreedyAllK proxy");
      return ctx.TotalCost();
    }

    for (const auto& h : clustered) {
      clustered_token_cost +=
          static_cast<FixedPointCost>(h.ShannonEntropy() * kFScale);
      StatusOr<FixedPointCost> header_cost = HistogramHeaderCost(h);
      if (!header_cost.ok()) {
        JXL_WARNING("HistogramHeaderCost failed in AssignPassesGreedyAllK proxy");
        return ctx.TotalCost();
      }
      clustered_token_cost += std::move(header_cost).value_();
    }
    if (clustered.size() > 1) {
      const double ctx_map_bits =
          static_cast<double>(histograms.size()) *
          std::log2(static_cast<double>(clustered.size()));
      clustered_token_cost += static_cast<FixedPointCost>(ctx_map_bits * kFScale);
    }
  }

  const FixedPointCost penalty =
      std::max<FixedPointCost>(0, clustered_token_cost - unclustered_token_cost);
  return ctx.TotalCost() + penalty;
}

std::vector<FixedPointCost> ComputeClusteredProxyCosts(
    std::vector<MultiKState>* states, ThreadPool* pool) {
  std::vector<FixedPointCost> costs(states->size(), 0);
  auto run_one_k = [&](size_t ki) {
    costs[ki] = ClusteredHistogramProxyCost((*states)[ki].ctx);
  };
  if (pool != nullptr && states->size() > 1) {
    (void)RunOnPool(
        pool, 0, static_cast<uint32_t>(states->size()), ThreadPool::NoInit,
        [&](uint32_t ki, size_t /*thread*/) -> Status {
          run_one_k(ki);
          return true;
        },
        "AssignPassesGreedyAllKClusteredCost");
  } else {
    for (size_t ki = 0; ki < states->size(); ++ki) {
      run_one_k(ki);
    }
  }
  return costs;
}

void PrintClusteredCostRow(const std::vector<MultiKState>& states,
                           const FixedPointCost* one_pass_cost,
                           const std::vector<FixedPointCost>& clustered_costs) {
  int column_width = 1;
  if (one_pass_cost != nullptr) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.2f", bit_cost(*one_pass_cost));
    column_width =
        std::max<int>(column_width, static_cast<int>(std::strlen(buf)));
  }
  for (FixedPointCost cost : clustered_costs) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.2f", bit_cost(cost));
    column_width =
        std::max<int>(column_width, static_cast<int>(std::strlen(buf)));
  }

  fprintf(stderr, "PLANNER: [all-k] %-12s", "cluster(bits)");
  if (one_pass_cost != nullptr) {
    fprintf(stderr, " 1=%*.2f", column_width, bit_cost(*one_pass_cost));
  }
  for (FixedPointCost cost : clustered_costs) {
    fprintf(stderr, " %*.2f", column_width, bit_cost(cost));
  }
  fprintf(stderr, "\n");
  fflush(stderr);
}

bool AllActiveSequentialGainBelowThreshold(
    const std::vector<MultiKState>& states, const std::vector<double>& drop_pct,
    double threshold_pct) {
  bool any_active = false;
  for (size_t i = 0; i < states.size(); ++i) {
    const MultiKState& state = states[i];
    if (state.finished || state.seq_moves <= state.min_moves) continue;
    any_active = true;
    if (drop_pct[i] >= threshold_pct) return false;
  }
  return any_active;
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
  bool have_one_pass_cost = false;
  FixedPointCost one_pass_cost = 0;
  bool have_one_pass_clustered_cost = false;
  FixedPointCost one_pass_clustered_cost = 0;
  if (min_num_passes <= 1 && 1 <= max_num_passes) {
    PassAssignmentCtx one_pass_ctx(d, active, 1);
    one_pass_ctx.InitPassAssignmentSimple();
    one_pass_ctx.InitNZPredictorState();
    one_pass_cost = one_pass_ctx.TotalCost();
    one_pass_clustered_cost = ClusteredHistogramProxyCost(one_pass_ctx);
    have_one_pass_cost = true;
    have_one_pass_clustered_cost = true;
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
      states[i].initial_cost = initial_costs[i];
      states[i].current_cost = initial_costs[i];
    }
  }
  fprintf(stderr, "PLANNER: [all-k] Initializing clusters took %.2f ms\n",
          NanosToMs(ElapsedNanos(start_init, PlannerClock::now())));
  fflush(stderr);

  constexpr int kPctColumnWidth = 7;
  const bool use_batch =
      (pool != nullptr && active_blocks.size() > kLargeImageThreshold);
  PrintMoveTableHeader(states, kPctColumnWidth);

  uint32_t iter = 0;
  {
    const auto start_seq = PlannerClock::now();
    const std::vector<SequentialSweepResult> seq_stats =
        FusedSequentialIter(&states, active_blocks, pool);
    out.shared_timings.sequential_ns +=
        ElapsedNanos(start_seq, PlannerClock::now());
    std::vector<double> seq_drop_pct(states.size(), 0.0);
    for (size_t i = 0; i < states.size(); ++i) {
      const FixedPointCost old_cost = states[i].current_cost;
      const FixedPointCost new_cost = old_cost + seq_stats[i].delta_cost;
      seq_drop_pct[i] = CostDropPercent(old_cost, new_cost);
    }
    MajoritySequentialImproved(&states, seq_stats);
    ++iter;
    PrintMoveTableRow("seq-1", states, seq_drop_pct, kPctColumnWidth);
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
      std::vector<double> batch_drop_pct(states.size(), 0.0);
      for (size_t i = 0; i < states.size(); ++i) {
        batch_drop_pct[i] =
            CostDropPercent(states[i].current_cost, new_costs[i]);
      }
      MajorityBatchImproved(&states, applied, new_costs);
      ++iter;
      char label[32];
      std::snprintf(label, sizeof(label), "b-batch-%u", iter);
      PrintMoveTableRow(label, states, batch_drop_pct, kPctColumnWidth);
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
      std::vector<double> batch_drop_pct(states.size(), 0.0);
      for (size_t i = 0; i < states.size(); ++i) {
        batch_drop_pct[i] =
            CostDropPercent(states[i].current_cost, new_costs[i]);
      }
      const bool good_batch = MajorityBatchImproved(&states, applied, new_costs);
      ++iter;
      char batch_label[32];
      std::snprintf(batch_label, sizeof(batch_label), "b-batch-%u", iter);
      PrintMoveTableRow(batch_label, states, batch_drop_pct, kPctColumnWidth);
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
      std::vector<double> seq_drop_pct(states.size(), 0.0);
      for (size_t i = 0; i < states.size(); ++i) {
        const FixedPointCost old_cost = states[i].current_cost;
        const FixedPointCost new_cost = old_cost + seq_stats[i].delta_cost;
        seq_drop_pct[i] = CostDropPercent(old_cost, new_cost);
      }
      MajoritySequentialImproved(&states, seq_stats);
      ++iter;
      char seq_label[32];
      std::snprintf(seq_label, sizeof(seq_label), "b-seq-%u", iter);
      PrintMoveTableRow(seq_label, states, seq_drop_pct, kPctColumnWidth);
    }
  }

  uint32_t seq_bad_streak = 0;
  while (iter < kMaxIters && AnyActiveK(states)) {
    const auto start_seq = PlannerClock::now();
    const std::vector<SequentialSweepResult> seq_stats =
        FusedSequentialIter(&states, active_blocks, pool);
    out.shared_timings.sequential_ns +=
        ElapsedNanos(start_seq, PlannerClock::now());
    std::vector<double> seq_drop_pct(states.size(), 0.0);
    for (size_t i = 0; i < states.size(); ++i) {
      const FixedPointCost old_cost = states[i].current_cost;
      const FixedPointCost new_cost = old_cost + seq_stats[i].delta_cost;
      seq_drop_pct[i] = CostDropPercent(old_cost, new_cost);
    }
    const bool good_seq = MajoritySequentialImproved(&states, seq_stats);
    ++iter;
    char seq_label[32];
    std::snprintf(seq_label, sizeof(seq_label), "s-seq-%u", iter);
    PrintMoveTableRow(seq_label, states, seq_drop_pct, kPctColumnWidth);
    if (AllActiveSequentialGainBelowThreshold(states, seq_drop_pct,
                                              kSequentialStopDropPct)) {
      break;
    }
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
      std::vector<double> batch_drop_pct(states.size(), 0.0);
      for (size_t i = 0; i < states.size(); ++i) {
        batch_drop_pct[i] =
            CostDropPercent(states[i].current_cost, new_costs[i]);
      }
      MajorityBatchImproved(&states, applied, new_costs);
      ++iter;
      char batch_label[32];
      std::snprintf(batch_label, sizeof(batch_label), "s-batch-%u", iter);
      PrintMoveTableRow(batch_label, states, batch_drop_pct, kPctColumnWidth);
      seq_bad_streak = 0;
    }
  }

  std::vector<double> total_drop_pct(states.size(), 0.0);
  for (size_t i = 0; i < states.size(); ++i) {
    total_drop_pct[i] =
        CostDropPercent(states[i].initial_cost, states[i].current_cost);
  }
  PrintMoveTableRow("total", states, total_drop_pct, kPctColumnWidth);
  PrintFinalCostRow(states, have_one_pass_cost ? &one_pass_cost : nullptr);
  PrintClusteredCostRow(
      states,
      have_one_pass_clustered_cost ? &one_pass_clustered_cost : nullptr,
      ComputeClusteredProxyCosts(&states, pool));

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

namespace {

struct FixedRowsAssignScratch {
  std::vector<FixedPointCost> delta;
  std::array<uint16_t, kNumPos> ac_slice_keys = {};
  std::array<uint32_t, kNumPos> ac_slice_counts = {};
  size_t ac_slice_size = 0;
  std::array<uint16_t, kNumPos> ac_token_keys = {};
  std::array<uint32_t, kNumPos> ac_token_counts = {};
  size_t ac_token_size = 0;
};

class FixedRowsPassAssignmentCtx {
 public:
  FixedRowsPassAssignmentCtx(const JPEGOptData& d, const FixedRows& fixed_rows,
                             uint32_t num_rows, uint32_t num_passes)
      : d(d),
        fixed_rows(fixed_rows),
        num_rows(num_rows),
        num_passes(num_passes),
        ac_total(static_cast<size_t>(num_rows) * num_passes *
                     kZeroDensityContextCount,
                 0),
        ac_hist(static_cast<size_t>(num_rows) * num_passes *
                    kZeroDensityContextCount * kACTokenCount,
                0),
        nz_total(static_cast<size_t>(num_rows) * num_passes *
                     kJPEGNonZeroBuckets,
                 0),
        nz_hist(static_cast<size_t>(num_rows) * num_passes *
                    kNZHistogramsSize,
                0) {
    for (uint32_t c = 0; c < kNumCh; ++c) {
      pass_assignment[c].assign(d.num_blocks[c], 0);
    }
  }

  FixedRowsAssignScratch MakeScratch() const {
    FixedRowsAssignScratch s;
    s.delta.resize(num_passes);
    return s;
  }

  void InitPassAssignmentSimple() {
    for (uint32_t c = 0; c < d.channels; ++c) {
      for (uint32_t b = 0; b < d.num_blocks[c]; ++b) {
        const uint32_t pass = b * num_passes / d.num_blocks[c];
        pass_assignment[c][b] = static_cast<uint8_t>(pass);
        AddACBlockToPass(c, b, pass);
      }
    }
  }

  void InitNZPredictorState() {
    for (uint32_t c = 0; c < d.channels; ++c) {
      nz_pred_bucket[c].resize(d.num_blocks[c], 0);
      for (uint32_t b = 0; b < d.num_blocks[c]; ++b) {
        const uint32_t pass = pass_assignment[c][b];
        const uint32_t row = fixed_rows[c][b];
        const uint32_t pb = PredictNZBucket(c, b, pass, nullptr, 0);
        nz_pred_bucket[c][b] = static_cast<uint8_t>(pb);
        const size_t slice = NZSliceIndex(row, pass, pb);
        ++nz_total[slice];
        ++nz_hist[slice * kJPEGNonZeroRange + d.block_nonzeros[c][b]];
      }
    }
  }

  uint32_t FindBestPass(const BlockRef& ref, uint32_t cur_pass,
                        FixedRowsAssignScratch* scratch,
                        FixedPointCost* best_delta_out = nullptr) const {
    const uint32_t row = fixed_rows[ref.c][ref.b];
    std::fill(scratch->delta.begin(), scratch->delta.end(), 0);
    scratch->ac_slice_size = 0;
    scratch->ac_token_size = 0;

    auto add_slice = [&](uint16_t zdc) {
      for (size_t i = 0; i < scratch->ac_slice_size; ++i) {
        if (scratch->ac_slice_keys[i] == zdc) {
          ++scratch->ac_slice_counts[i];
          return;
        }
      }
      scratch->ac_slice_keys[scratch->ac_slice_size] = zdc;
      scratch->ac_slice_counts[scratch->ac_slice_size] = 1;
      ++scratch->ac_slice_size;
    };
    auto add_token = [&](uint16_t key) {
      for (size_t i = 0; i < scratch->ac_token_size; ++i) {
        if (scratch->ac_token_keys[i] == key) {
          ++scratch->ac_token_counts[i];
          return;
        }
      }
      scratch->ac_token_keys[scratch->ac_token_size] = key;
      scratch->ac_token_counts[scratch->ac_token_size] = 1;
      ++scratch->ac_token_size;
    };

    ForEachBlockBin(d, ref.c, ref.b, [&](ACBin bin) {
      const CompactACEvent ac_event = d.FromBin(bin);
      const SignallingHistSymbol sym = d.SignallingHistSymbolFromSymbol(
          d.ACHistogram().dense_to_zdcvalue[ac_event.hist_bin]);
      add_slice(static_cast<uint16_t>(sym.zdc));
      add_token(static_cast<uint16_t>(sym.zdc * kACTokenCount + sym.token));
    });

    FixedPointCost best_delta = 0;
    uint32_t best_pass = cur_pass;
    for (uint32_t pass = 0; pass < num_passes; ++pass) {
      if (pass == cur_pass) continue;
      FixedPointCost delta = 0;
      for (size_t i = 0; i < scratch->ac_slice_size; ++i) {
        const uint16_t zdc = scratch->ac_slice_keys[i];
        const uint32_t count = scratch->ac_slice_counts[i];
        const size_t old_slice = ACSliceIndex(row, cur_pass, zdc);
        const size_t new_slice = ACSliceIndex(row, pass, zdc);
        delta += d.ftab[ac_total[new_slice] + count] - d.ftab[ac_total[new_slice]];
        delta += d.ftab[ac_total[old_slice] - count] - d.ftab[ac_total[old_slice]];
      }
      for (size_t i = 0; i < scratch->ac_token_size; ++i) {
        const uint16_t key = scratch->ac_token_keys[i];
        const uint16_t zdc = key / kACTokenCount;
        const uint16_t token = key % kACTokenCount;
        const uint32_t count = scratch->ac_token_counts[i];
        const size_t old_hist =
            ACSliceIndex(row, cur_pass, zdc) * kACTokenCount + token;
        const size_t new_hist =
            ACSliceIndex(row, pass, zdc) * kACTokenCount + token;
        delta -= d.ftab[ac_hist[new_hist] + count] - d.ftab[ac_hist[new_hist]];
        delta -= d.ftab[ac_hist[old_hist] - count] - d.ftab[ac_hist[old_hist]];
      }

      std::array<BlockRef, 3> affected = {ref, ref, ref};
      size_t num_affected = 1;
      const uint32_t w = d.block_grid_w[ref.c];
      const uint32_t x = ref.b % w;
      if (x + 1 < w) affected[num_affected++] = {ref.c, ref.b + 1u};
      if (ref.b + w < d.num_blocks[ref.c]) {
        affected[num_affected++] = {ref.c, ref.b + w};
      }

      std::array<uint32_t, 6> n_keys = {};
      std::array<int32_t, 6> n_diff = {};
      size_t n_size = 0;
      std::array<uint32_t, 6> h_keys = {};
      std::array<int32_t, 6> h_diff = {};
      size_t h_size = 0;
      auto add_change = [](uint32_t key, int32_t diff,
                           std::array<uint32_t, 6>* keys,
                           std::array<int32_t, 6>* vals, size_t* size) {
        for (size_t i = 0; i < *size; ++i) {
          if ((*keys)[i] == key) {
            (*vals)[i] += diff;
            return;
          }
        }
        (*keys)[*size] = key;
        (*vals)[*size] = diff;
        ++*size;
      };
      for (size_t i = 0; i < num_affected; ++i) {
        const BlockRef& ar = affected[i];
        const uint32_t ar_row = fixed_rows[ar.c][ar.b];
        const uint32_t old_pass = pass_assignment[ar.c][ar.b];
        const uint32_t old_pb = nz_pred_bucket[ar.c][ar.b];
        const uint32_t new_pass = (ar.c == ref.c && ar.b == ref.b) ? pass : old_pass;
        const uint32_t new_pb = PredictNZBucket(ar.c, ar.b, new_pass, &ref, pass);
        if (old_pass == new_pass && old_pb == new_pb) continue;
        const uint32_t nz = d.block_nonzeros[ar.c][ar.b];
        add_change(NZSliceIndex(ar_row, old_pass, old_pb), -1, &n_keys, &n_diff,
                   &n_size);
        add_change(NZSliceIndex(ar_row, new_pass, new_pb), +1, &n_keys, &n_diff,
                   &n_size);
        add_change(NZSliceIndex(ar_row, old_pass, old_pb) * kJPEGNonZeroRange +
                       nz,
                   -1, &h_keys, &h_diff, &h_size);
        add_change(NZSliceIndex(ar_row, new_pass, new_pb) * kJPEGNonZeroRange +
                       nz,
                   +1, &h_keys, &h_diff, &h_size);
      }
      for (size_t i = 0; i < n_size; ++i) {
        if (n_diff[i] == 0) continue;
        delta += d.NZFTab(static_cast<uint32_t>(
                     static_cast<int32_t>(nz_total[n_keys[i]]) + n_diff[i])) -
                 d.NZFTab(nz_total[n_keys[i]]);
      }
      for (size_t i = 0; i < h_size; ++i) {
        if (h_diff[i] == 0) continue;
        delta -= d.NZFTab(static_cast<uint32_t>(
                     static_cast<int32_t>(nz_hist[h_keys[i]]) + h_diff[i])) -
                 d.NZFTab(nz_hist[h_keys[i]]);
      }
      scratch->delta[pass] = delta;
      if (delta < best_delta) {
        best_delta = delta;
        best_pass = pass;
      }
    }
    if (best_delta_out != nullptr) *best_delta_out = best_delta;
    return best_pass;
  }

  void ApplyMove(const BlockRef& ref, uint32_t cur_pass, uint32_t new_pass) {
    std::array<BlockRef, 3> affected = {ref, ref, ref};
    size_t num_affected = 1;
    const uint32_t w = d.block_grid_w[ref.c];
    const uint32_t x = ref.b % w;
    if (x + 1 < w) {
      affected[num_affected++] = {ref.c, static_cast<uint32_t>(ref.b + 1)};
    }
    if (ref.b + w < d.num_blocks[ref.c]) {
      affected[num_affected++] = {ref.c, static_cast<uint32_t>(ref.b + w)};
    }

    struct NZState {
      BlockRef ref;
      uint32_t row;
      uint32_t old_pass;
      uint32_t old_pb;
      uint32_t new_pass;
      uint32_t new_pb;
    };
    std::array<NZState, 3> nz_states = {};
    for (size_t i = 0; i < num_affected; ++i) {
      const BlockRef& ar = affected[i];
      const uint32_t op = pass_assignment[ar.c][ar.b];
      const uint32_t np = (ar.c == ref.c && ar.b == ref.b) ? new_pass : op;
      nz_states[i] = {ar,
                      fixed_rows[ar.c][ar.b],
                      op,
                      nz_pred_bucket[ar.c][ar.b],
                      np,
                      PredictNZBucket(ar.c, ar.b, np, &ref, new_pass)};
    }

    RemoveACBlockFromPass(ref.c, ref.b, cur_pass);
    AddACBlockToPass(ref.c, ref.b, new_pass);

    for (size_t i = 0; i < num_affected; ++i) {
      const NZState& s = nz_states[i];
      const size_t old_slice = NZSliceIndex(s.row, s.old_pass, s.old_pb);
      --nz_total[old_slice];
      --nz_hist[old_slice * kJPEGNonZeroRange +
                d.block_nonzeros[s.ref.c][s.ref.b]];
    }
    pass_assignment[ref.c][ref.b] = static_cast<uint8_t>(new_pass);
    for (size_t i = 0; i < num_affected; ++i) {
      const NZState& s = nz_states[i];
      const size_t new_slice = NZSliceIndex(s.row, s.new_pass, s.new_pb);
      ++nz_total[new_slice];
      ++nz_hist[new_slice * kJPEGNonZeroRange +
                d.block_nonzeros[s.ref.c][s.ref.b]];
      nz_pred_bucket[s.ref.c][s.ref.b] = static_cast<uint8_t>(s.new_pb);
    }
  }

  FixedPointCost TotalCost() const {
    FixedPointCost cost = 0;
    for (uint32_t total : ac_total) {
      if (total != 0) cost += d.ftab[total];
    }
    for (uint32_t count : ac_hist) {
      if (count != 0) cost -= d.ftab[count];
    }
    for (uint32_t total : nz_total) {
      if (total != 0) cost += d.NZFTab(total);
    }
    for (uint32_t count : nz_hist) {
      if (count != 0) cost -= d.NZFTab(count);
    }
    return cost;
  }

  uint32_t PredictNZBucket(uint16_t c, uint32_t b, uint32_t pass,
                           const BlockRef* override_ref,
                           uint32_t override_pass) const {
    const uint32_t w = d.block_grid_w[c];
    const uint32_t x = b % w;
    const uint32_t y = b / w;
    const uint32_t b_top = (y > 0) ? b - w : 0;
    const uint32_t b_left = (x > 0) ? b - 1 : 0;
    auto pass_of = [&](uint32_t neighbor) -> uint32_t {
      if (override_ref != nullptr && override_ref->c == c &&
          override_ref->b == neighbor) {
        return override_pass;
      }
      return pass_assignment[c][neighbor];
    };
    const uint32_t pass_nz_top =
        (y > 0 && pass_of(b_top) == pass) ? d.block_nonzeros[c][b_top] : 0u;
    const uint32_t pass_nz_left =
        (x > 0 && pass_of(b_left) == pass) ? d.block_nonzeros[c][b_left] : 0u;
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

  size_t ACSliceIndex(uint32_t row, uint32_t pass, uint32_t zdc) const {
    return (static_cast<size_t>(row) * num_passes + pass) *
               kZeroDensityContextCount +
           zdc;
  }

  size_t NZSliceIndex(uint32_t row, uint32_t pass, uint32_t pb) const {
    return (static_cast<size_t>(row) * num_passes + pass) *
               kJPEGNonZeroBuckets +
           pb;
  }

  void AddACBlockToPass(uint32_t c, uint32_t b, uint32_t pass) {
    const uint32_t row = fixed_rows[c][b];
    ForEachBlockBin(d, c, b, [&](ACBin bin) {
      const CompactACEvent ac_event = d.FromBin(bin);
      const SignallingHistSymbol sym = d.SignallingHistSymbolFromSymbol(
          d.ACHistogram().dense_to_zdcvalue[ac_event.hist_bin]);
      const size_t slice = ACSliceIndex(row, pass, sym.zdc);
      ++ac_total[slice];
      ++ac_hist[slice * kACTokenCount + sym.token];
    });
  }

  void RemoveACBlockFromPass(uint32_t c, uint32_t b, uint32_t pass) {
    const uint32_t row = fixed_rows[c][b];
    ForEachBlockBin(d, c, b, [&](ACBin bin) {
      const CompactACEvent ac_event = d.FromBin(bin);
      const SignallingHistSymbol sym = d.SignallingHistSymbolFromSymbol(
          d.ACHistogram().dense_to_zdcvalue[ac_event.hist_bin]);
      const size_t slice = ACSliceIndex(row, pass, sym.zdc);
      --ac_total[slice];
      --ac_hist[slice * kACTokenCount + sym.token];
    });
  }

  const JPEGOptData& d;
  const FixedRows& fixed_rows;
  const uint32_t num_rows;
  const uint32_t num_passes;

  PassAssignment pass_assignment;
  std::vector<uint32_t> ac_total;
  std::vector<uint32_t> ac_hist;
  std::vector<uint32_t> nz_total;
  std::vector<uint32_t> nz_hist;
  std::array<std::vector<uint8_t>, kNumCh> nz_pred_bucket;
};

struct FixedRowsMultiKState {
  uint32_t num_passes;
  FixedRowsPassAssignmentCtx ctx;
  FixedRowsAssignScratch scratch;
  std::vector<uint8_t> new_passes;
  uint32_t min_moves;
  uint32_t seq_moves;
  FixedPointCost initial_cost;
  FixedPointCost current_cost;
  bool finished;

  FixedRowsMultiKState(const JPEGOptData& d, const FixedRows& fixed_rows,
                       uint32_t num_rows, uint32_t num_passes,
                       size_t num_active_blocks)
      : num_passes(num_passes),
        ctx(d, fixed_rows, num_rows, num_passes),
        scratch(ctx.MakeScratch()),
        new_passes(num_active_blocks, 0),
        min_moves(std::max<uint32_t>(
            1, static_cast<uint32_t>(
                   std::max<size_t>(size_t{1}, num_active_blocks >> 13)))),
        seq_moves(std::numeric_limits<uint32_t>::max()),
        initial_cost(0),
        current_cost(0),
        finished(false) {}
};

}  // namespace

AssignPassesRangeResult AssignPassesGreedyAllKFixedRows(
    const JPEGOptData& d, const ActiveRawBins& /*active*/,
    const FixedRows& fixed_rows, uint32_t num_rows,
    uint32_t min_num_passes, uint32_t max_num_passes, ThreadPool* pool) {
  AssignPassesRangeResult out;
  out.min_num_passes = min_num_passes;
  out.results.resize(max_num_passes - min_num_passes + 1);

  const auto start_total = PlannerClock::now();
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

  if (min_num_passes <= 1 && 1 <= max_num_passes) {
    AssignPassesResult& result = out.results[1 - min_num_passes];
    for (uint32_t c = 0; c < kNumCh; ++c) {
      result.pass_assignment[c].assign(d.num_blocks[c], 0);
    }
  }

  bool have_one_pass_cost = false;
  FixedPointCost one_pass_cost = 0;
  if (min_num_passes <= 1 && 1 <= max_num_passes) {
    FixedRowsPassAssignmentCtx one_pass_ctx(d, fixed_rows, num_rows, 1);
    one_pass_ctx.InitPassAssignmentSimple();
    one_pass_ctx.InitNZPredictorState();
    one_pass_cost = one_pass_ctx.TotalCost();
    have_one_pass_cost = true;
  }

  const uint32_t first_fused_k = std::max<uint32_t>(2, min_num_passes);
  std::vector<FixedRowsMultiKState> states;
  if (max_num_passes >= first_fused_k) {
    states.reserve(max_num_passes - first_fused_k + 1);
    for (uint32_t k = first_fused_k; k <= max_num_passes; ++k) {
      states.emplace_back(d, fixed_rows, num_rows, k, active_blocks.size());
    }
  }
  if (states.empty()) {
    out.shared_timings.total_ns = ElapsedNanos(start_total, PlannerClock::now());
    for (AssignPassesResult& result : out.results) result.timings = out.shared_timings;
    return out;
  }

  const auto start_init = PlannerClock::now();
  for (FixedRowsMultiKState& state : states) {
    state.ctx.InitPassAssignmentSimple();
    state.ctx.InitNZPredictorState();
    state.initial_cost = state.ctx.TotalCost();
    state.current_cost = state.initial_cost;
  }
  fprintf(stderr,
          "PLANNER: [all-k fixed] Initializing clusters took %.2f ms\n",
          NanosToMs(ElapsedNanos(start_init, PlannerClock::now())));
  fflush(stderr);

  constexpr int kPctColumnWidth = 7;
  const bool use_batch =
      (pool != nullptr && active_blocks.size() > kLargeImageThreshold);
  fprintf(stderr, "PLANNER: [all-k fixed] %-12s", "step/passes");
  for (const auto& state : states) {
    fprintf(stderr, " %*u", kPctColumnWidth, state.num_passes);
  }
  fprintf(stderr, "\n");
  fflush(stderr);

  auto any_active = [&]() {
    for (const auto& state : states) {
      if (!state.finished && state.seq_moves > state.min_moves) return true;
    }
    return false;
  };
  auto gain_well_enough = [](FixedPointCost gain, FixedPointCost current_cost) {
    const FixedPointCost threshold =
        std::max<FixedPointCost>(1, current_cost >> 14);
    return gain >= threshold;
  };
  auto majority_seq_improved =
      [&](const std::vector<SequentialSweepResult>& seq_stats) {
        size_t active = 0;
        size_t good = 0;
        for (size_t i = 0; i < states.size(); ++i) {
          FixedRowsMultiKState& state = states[i];
          const FixedPointCost old_cost = state.current_cost;
          state.seq_moves = seq_stats[i].moves;
          state.current_cost += seq_stats[i].delta_cost;
          const FixedPointCost gain =
              std::max<FixedPointCost>(0, -seq_stats[i].delta_cost);
          const bool finished_now = (seq_stats[i].moves == 0);
          if (!state.finished && !finished_now &&
              seq_stats[i].moves > state.min_moves) {
            ++active;
            if (gain_well_enough(gain, old_cost)) ++good;
          }
          state.finished = state.finished || finished_now;
        }
        return active == 0 || good * 3 >= active;
      };
  auto majority_batch_improved =
      [&](const std::vector<uint32_t>& batch_moves,
          const std::vector<FixedPointCost>& new_costs) {
        size_t active = 0;
        size_t good = 0;
        for (size_t i = 0; i < states.size(); ++i) {
          FixedRowsMultiKState& state = states[i];
          const FixedPointCost old_cost = state.current_cost;
          const FixedPointCost gain =
              std::max<FixedPointCost>(0, old_cost - new_costs[i]);
          const bool finished_now = (batch_moves[i] == 0);
          if (!state.finished && !finished_now &&
              state.seq_moves > state.min_moves) {
            ++active;
            if (gain_well_enough(gain, old_cost)) ++good;
          }
          state.current_cost = new_costs[i];
          state.finished = state.finished || finished_now;
        }
        return active == 0 || good * 2 >= active;
      };
  auto print_row = [&](const char* label, const std::vector<double>& drop_pct) {
    fprintf(stderr, "PLANNER: [all-k fixed] %-12s", label);
    for (size_t i = 0; i < states.size(); ++i) {
      fprintf(stderr, " %*.3f%%", kPctColumnWidth - 1, drop_pct[i]);
    }
    fprintf(stderr, "\n");
    fflush(stderr);
  };
  auto print_final_cost_row = [&]() {
    int column_width = 1;
    if (have_one_pass_cost) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%.2f", bit_cost(one_pass_cost));
      column_width =
          std::max<int>(column_width, static_cast<int>(std::strlen(buf)));
    }
    for (const auto& state : states) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%.2f", bit_cost(state.current_cost));
      column_width =
          std::max<int>(column_width, static_cast<int>(std::strlen(buf)));
    }
    fprintf(stderr, "PLANNER: [all-k fixed] %-12s", "cost(bits)");
    if (have_one_pass_cost) {
      fprintf(stderr, " 1=%*.2f", column_width, bit_cost(one_pass_cost));
    }
    for (const auto& state : states) {
      fprintf(stderr, " %*.2f", column_width, bit_cost(state.current_cost));
    }
    fprintf(stderr, "\n");
    fflush(stderr);
  };
  auto all_active_seq_below_threshold =
      [&](const std::vector<double>& drop_pct) {
        bool any = false;
        for (size_t i = 0; i < states.size(); ++i) {
          if (states[i].finished || states[i].seq_moves <= states[i].min_moves) {
            continue;
          }
          any = true;
          if (drop_pct[i] >= kSequentialStopDropPct) return false;
        }
        return any;
      };
  auto fused_seq_iter = [&]() {
    std::vector<SequentialSweepResult> results(states.size());
    auto run_one_k = [&](size_t ki) {
      FixedRowsMultiKState& state = states[ki];
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
    if (pool != nullptr && states.size() > 1) {
      (void)RunOnPool(
          pool, 0, static_cast<uint32_t>(states.size()), ThreadPool::NoInit,
          [&](uint32_t ki, size_t /*thread*/) -> Status {
            run_one_k(ki);
            return true;
          },
          "AssignPassesGreedyAllKFixedRowsSeq");
    } else {
      for (size_t ki = 0; ki < states.size(); ++ki) run_one_k(ki);
    }
    return results;
  };
  auto fused_score_batch = [&]() {
    const uint32_t num_chunks = static_cast<uint32_t>(
        (active_blocks.size() + kBatchChunkSize - 1) / kBatchChunkSize);
    std::vector<std::vector<uint32_t>> thread_moves;
    std::vector<std::vector<FixedRowsAssignScratch>> scratch_pool;
    if (!RunOnPool(
            pool, 0, num_chunks,
            [&](size_t num_threads) -> Status {
              thread_moves.assign(num_threads,
                                  std::vector<uint32_t>(states.size(), 0));
              scratch_pool.resize(num_threads);
              for (size_t t = 0; t < num_threads; ++t) {
                scratch_pool[t].reserve(states.size());
                for (size_t k = 0; k < states.size(); ++k) {
                  scratch_pool[t].push_back(states[k].ctx.MakeScratch());
                }
              }
              return true;
            },
            [&](uint32_t chunk, size_t thread_id) -> Status {
              std::vector<uint32_t>& local_moves = thread_moves[thread_id];
              const size_t begin =
                  static_cast<size_t>(chunk) * kBatchChunkSize;
              const size_t end =
                  std::min(begin + kBatchChunkSize, active_blocks.size());
              for (size_t i = begin; i < end; ++i) {
                const BlockRef& ref = active_blocks[i];
                for (size_t k = 0; k < states.size(); ++k) {
                  FixedRowsMultiKState& state = states[k];
                  if (state.finished) continue;
                  const uint32_t cur = state.ctx.pass_assignment[ref.c][ref.b];
                  const uint32_t best =
                      state.ctx.FindBestPass(ref, cur,
                                             &scratch_pool[thread_id][k]);
                  state.new_passes[i] = static_cast<uint8_t>(best);
                  if (best != cur) ++local_moves[k];
                }
              }
              return true;
            },
            "AssignPassesGreedyAllKFixedRowsBatch")) {
      return std::vector<uint32_t>(states.size(), 0);
    }
    std::vector<uint32_t> batch_moves(states.size(), 0);
    for (const auto& move : thread_moves) {
      for (size_t k = 0; k < states.size(); ++k) batch_moves[k] += move[k];
    }
    return batch_moves;
  };
  auto fused_apply_batch = [&](uint32_t stride, uint32_t iter) {
    std::vector<uint32_t> applied(states.size(), 0);
    for (size_t i = 0; i < active_blocks.size(); ++i) {
      if (i % stride != iter % stride) continue;
      const BlockRef& ref = active_blocks[i];
      for (size_t k = 0; k < states.size(); ++k) {
        FixedRowsMultiKState& state = states[k];
        if (state.finished) continue;
        const uint32_t cur = state.ctx.pass_assignment[ref.c][ref.b];
        const uint32_t next = state.new_passes[i];
        if (next == cur) continue;
        state.ctx.ApplyMove(ref, cur, next);
        ++applied[k];
      }
    }
    return applied;
  };
  auto compute_current_costs = [&]() {
    std::vector<FixedPointCost> costs(states.size(), 0);
    auto run_one_k = [&](size_t ki) {
      FixedRowsMultiKState& state = states[ki];
      costs[ki] = state.finished ? state.current_cost : state.ctx.TotalCost();
    };
    if (pool != nullptr && states.size() > 1) {
      (void)RunOnPool(
          pool, 0, static_cast<uint32_t>(states.size()), ThreadPool::NoInit,
          [&](uint32_t ki, size_t /*thread*/) -> Status {
            run_one_k(ki);
            return true;
          },
          "AssignPassesGreedyAllKFixedRowsCost");
    } else {
      for (size_t ki = 0; ki < states.size(); ++ki) run_one_k(ki);
    }
    return costs;
  };

  uint32_t iter = 0;
  {
    const auto start_seq = PlannerClock::now();
    const std::vector<SequentialSweepResult> seq_stats = fused_seq_iter();
    out.shared_timings.sequential_ns +=
        ElapsedNanos(start_seq, PlannerClock::now());
    std::vector<double> seq_drop_pct(states.size(), 0.0);
    for (size_t i = 0; i < states.size(); ++i) {
      const FixedPointCost old_cost = states[i].current_cost;
      const FixedPointCost new_cost = old_cost + seq_stats[i].delta_cost;
      seq_drop_pct[i] = CostDropPercent(old_cost, new_cost);
    }
    majority_seq_improved(seq_stats);
    ++iter;
    print_row("seq-1", seq_drop_pct);
  }

  constexpr uint32_t kUnconditionalBatchIterations = 5;
  constexpr uint32_t kBatchPatience = 2;
  constexpr uint32_t kBatchStride = 1;

  if (use_batch) {
    for (uint32_t n = 0;
         n < kUnconditionalBatchIterations && iter < kMaxIters && any_active();
         ++n) {
      const auto start_batch = PlannerClock::now();
      const std::vector<uint32_t> batch_moves = fused_score_batch();
      if (std::all_of(batch_moves.begin(), batch_moves.end(),
                      [](uint32_t v) { return v == 0; })) {
        break;
      }
      const std::vector<uint32_t> applied =
          fused_apply_batch(kBatchStride, iter);
      out.shared_timings.batch_ns +=
          ElapsedNanos(start_batch, PlannerClock::now());
      const std::vector<FixedPointCost> new_costs = compute_current_costs();
      std::vector<double> batch_drop_pct(states.size(), 0.0);
      for (size_t i = 0; i < states.size(); ++i) {
        batch_drop_pct[i] =
            CostDropPercent(states[i].current_cost, new_costs[i]);
      }
      majority_batch_improved(applied, new_costs);
      ++iter;
      char label[32];
      std::snprintf(label, sizeof(label), "b-batch-%u", iter);
      print_row(label, batch_drop_pct);
    }

    uint32_t batch_stale_count = 0;
    while (iter < kMaxIters && any_active() &&
           batch_stale_count < kBatchPatience) {
      const auto start_batch = PlannerClock::now();
      const std::vector<uint32_t> batch_moves = fused_score_batch();
      if (std::all_of(batch_moves.begin(), batch_moves.end(),
                      [](uint32_t v) { return v == 0; })) {
        break;
      }
      const std::vector<uint32_t> applied =
          fused_apply_batch(kBatchStride, iter);
      out.shared_timings.batch_ns +=
          ElapsedNanos(start_batch, PlannerClock::now());
      const std::vector<FixedPointCost> new_costs = compute_current_costs();
      std::vector<double> batch_drop_pct(states.size(), 0.0);
      for (size_t i = 0; i < states.size(); ++i) {
        batch_drop_pct[i] =
            CostDropPercent(states[i].current_cost, new_costs[i]);
      }
      const bool good_batch = majority_batch_improved(applied, new_costs);
      ++iter;
      char batch_label[32];
      std::snprintf(batch_label, sizeof(batch_label), "b-batch-%u", iter);
      print_row(batch_label, batch_drop_pct);
      if (good_batch) {
        batch_stale_count = 0;
        continue;
      }

      ++batch_stale_count;
      if (iter >= kMaxIters) break;
      const auto start_seq = PlannerClock::now();
      const std::vector<SequentialSweepResult> seq_stats = fused_seq_iter();
      out.shared_timings.sequential_ns +=
          ElapsedNanos(start_seq, PlannerClock::now());
      std::vector<double> seq_drop_pct(states.size(), 0.0);
      for (size_t i = 0; i < states.size(); ++i) {
        const FixedPointCost old_cost = states[i].current_cost;
        const FixedPointCost new_cost = old_cost + seq_stats[i].delta_cost;
        seq_drop_pct[i] = CostDropPercent(old_cost, new_cost);
      }
      majority_seq_improved(seq_stats);
      ++iter;
      char seq_label[32];
      std::snprintf(seq_label, sizeof(seq_label), "b-seq-%u", iter);
      print_row(seq_label, seq_drop_pct);
    }
  }

  uint32_t seq_bad_streak = 0;
  while (iter < kMaxIters && any_active()) {
    const auto start_seq = PlannerClock::now();
    const std::vector<SequentialSweepResult> seq_stats = fused_seq_iter();
    out.shared_timings.sequential_ns +=
        ElapsedNanos(start_seq, PlannerClock::now());
    std::vector<double> seq_drop_pct(states.size(), 0.0);
    for (size_t i = 0; i < states.size(); ++i) {
      const FixedPointCost old_cost = states[i].current_cost;
      const FixedPointCost new_cost = old_cost + seq_stats[i].delta_cost;
      seq_drop_pct[i] = CostDropPercent(old_cost, new_cost);
    }
    const bool good_seq = majority_seq_improved(seq_stats);
    ++iter;
    char seq_label[32];
    std::snprintf(seq_label, sizeof(seq_label), "s-seq-%u", iter);
    print_row(seq_label, seq_drop_pct);
    if (all_active_seq_below_threshold(seq_drop_pct)) break;
    if (iter >= kMaxIters || !any_active()) break;
    if (good_seq) {
      seq_bad_streak = 0;
      continue;
    }

    ++seq_bad_streak;
    if (use_batch && seq_bad_streak >= 2) {
      const auto start_batch = PlannerClock::now();
      const std::vector<uint32_t> batch_moves = fused_score_batch();
      if (std::all_of(batch_moves.begin(), batch_moves.end(),
                      [](uint32_t v) { return v == 0; })) {
        break;
      }
      const std::vector<uint32_t> applied =
          fused_apply_batch(kBatchStride, iter);
      out.shared_timings.batch_ns +=
          ElapsedNanos(start_batch, PlannerClock::now());
      const std::vector<FixedPointCost> new_costs = compute_current_costs();
      std::vector<double> batch_drop_pct(states.size(), 0.0);
      for (size_t i = 0; i < states.size(); ++i) {
        batch_drop_pct[i] =
            CostDropPercent(states[i].current_cost, new_costs[i]);
      }
      majority_batch_improved(applied, new_costs);
      ++iter;
      char batch_label[32];
      std::snprintf(batch_label, sizeof(batch_label), "s-batch-%u", iter);
      print_row(batch_label, batch_drop_pct);
      seq_bad_streak = 0;
    }
  }

  std::vector<double> total_drop_pct(states.size(), 0.0);
  for (size_t i = 0; i < states.size(); ++i) {
    total_drop_pct[i] =
        CostDropPercent(states[i].initial_cost, states[i].current_cost);
  }
  print_row("total", total_drop_pct);
  print_final_cost_row();

  out.shared_timings.total_ns = ElapsedNanos(start_total, PlannerClock::now());
  for (FixedRowsMultiKState& state : states) {
    AssignPassesResult& result = out.results[state.num_passes - min_num_passes];
    result.pass_assignment = std::move(state.ctx.pass_assignment);
    result.timings = out.shared_timings;
  }
  for (AssignPassesResult& result : out.results) result.timings = out.shared_timings;
  return out;
}

}  // namespace jxl
