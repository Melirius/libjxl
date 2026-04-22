// Copyright (c) the JPEG XL Project Authors. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

// Experimental pass-aware context-model search for JPEG lossless
// recompression.
//
// The classic optimizer in `enc_jpeg_frame.cc` searches one global context map
// over the canonical AC stream. This file implements two related experimental
// planner lanes that instead reason about multiple progressive AC passes:
//
// `SearchPassAwareContextModel`
//   Assigns blocks to passes, rebuilds a pass-local AC stream, clusters the
//   resulting `(cell, pass)` contexts, and evaluates thresholds on that
//   pass-aware model.
//
// `SearchBiclusteredContextModel`
//   Reuses the same pass assignment and threshold search, but also materializes
//   a fixed `(row, pass, slice)` histogram lattice used to score an initial
//   biclustering-style objective. In the current prototype this still reuses
//   the pass-aware row clustering path; it is the scaffolding for the more
//   ambitious hierarchical biclustering experiment described in
//   `plans/Passes_histo_clustering.md`.
//
// Internal helpers are grouped into four layers:
//
// `ActiveRawBins`, `AssignPassesGreedy`, `BuildPassStream`
//   Build the pass-local view of the AC stream and the block->pass assignment.
//
// `ClusterContextsPassAware`, `EvaluatePassAwareModel`
//   Cluster and score the pass-aware `(cell, pass)` contexts.
//
// `RowSliceHistograms`, `BuildRowSliceHistograms`, `EvaluateBiclusterState`
//   Materialize and score the richer `(row, pass, zdc/pb)` biclustering state.
//
// `SearchPassAwareContextModel`, `SearchBiclusteredContextModel`
//   Drive the candidate search over thresholds and pick the best-scoring model.

#include "lib/jxl/transcode_jpeg/enc_jpeg_passes.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "lib/jxl/base/data_parallel.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_bicluster.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_assign.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_cluster.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_stream.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_pass_utils.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_search.h"
#include "lib/jxl/transcode_jpeg/enc_jpeg_threshold.h"

namespace jxl {

namespace {

using SparseHistogram = std::vector<std::unordered_map<uint32_t, uint32_t>>;

// Optional threshold refinement stage reused by both experimental searches.
ThresholdSet RefinePassAwareThresholds(
    PartitioningCtx& ctx, const ThresholdSet& thresholds,
    const std::vector<ACEntry>& pass_stream,
    const JPEGCtxEffortParams& effort) {
  if (effort.refine_iters == 0) return thresholds;
  FixedPointCost ignored_cost = 0;
  return ctx.OptimizeThresholds(thresholds, pass_stream, effort.main_m_target,
                                effort.refine_iters, &ignored_cost);
}

// Upper bound for the number of progressive passes worth considering from
// the image size. 11 is a hard limit by the standard, and number of
// histogram clusters is limited by max `num_hf_presets` which is written by
// `u(ceil(log2(num_groups))) + 1`.
uint32_t ComputeMaxNumPasses(const JPEGOptData& d) {
  const double groups_x = static_cast<double>((d.w_max + 31) / 32);
  const double groups_y = static_cast<double>((d.h_max + 31) / 32);
  const double groups = std::max(1.0, groups_x * groups_y);
  return static_cast<uint32_t>(
      std::min(11.0, std::ceil(std::log2(groups)) + 1.0));
}

}  // namespace (close anonymous namespace - exported functions follow)

StatusOr<PassSearchResult> SearchPassAwareContextModel(
    std::shared_ptr<const JPEGOptData> opt_data,
    const std::vector<FactorizationCandidate>& candidates,
    const JPEGCtxEffortParams& effort, ThreadPool* pool) {
  if (candidates.empty()) {
    return JXL_FAILURE("Pass-aware search requires at least one candidate");
  }

  const JPEGOptData& d = *opt_data;
  const ActiveRawBins active = BuildActiveRawBins(d);
  const uint32_t min_passes =
      effort.optimize_passes_num <= 0
          ? 1
          : static_cast<uint32_t>(effort.optimize_passes_num);
  const uint32_t max_img_passes = ComputeMaxNumPasses(d);
  const uint32_t max_passes =
      effort.optimize_passes_num < 0
          ? 1
          : (effort.optimize_passes_num == 0
                 ? max_img_passes
                 : std::min<uint32_t>(effort.optimize_passes_num,
                                      max_img_passes));
  const uint32_t target_clusters =
      kMaxClusters - static_cast<uint32_t>(d.channels == 1);

  std::unique_ptr<AssignPassesRangeResult> assign_range_result;
  if (max_passes > min_passes) {
    assign_range_result = jxl::make_unique<AssignPassesRangeResult>(
        AssignPassesGreedyAllK(d, active, min_passes, max_passes, pool));
    fprintf(stderr,
            "PLANNER: AssignPassesGreedyAllK took %.2f ms total "
            "(batch %.2f ms, sequential %.2f ms)\n",
            NanosToMs(assign_range_result->shared_timings.total_ns),
            NanosToMs(assign_range_result->shared_timings.batch_ns),
            NanosToMs(assign_range_result->shared_timings.sequential_ns));
    fflush(stderr);
  }

  // Accumulates the best result across all pass configurations. Updated by
  // deterministic reduction after each pass loop.
  PassSearchResult best_result;
  best_result.total_cost = std::numeric_limits<FixedPointCost>::max();
  // Tiebreaker for deterministic results: among equal-cost candidates the
  // one with the lowest candidate index wins, independent of thread count.
  uint32_t overall_best_idx = std::numeric_limits<uint32_t>::max();

  for (uint32_t num_passes = min_passes; num_passes <= max_passes;
       ++num_passes) {
    auto start_pass_config = std::chrono::high_resolution_clock::now();
    fprintf(stderr, "PLANNER: Testing configuration with %u passes\n", num_passes);
    fflush(stderr);
    AssignPassesResult assign_result_storage;
    const AssignPassesResult* assign_result = nullptr;
    if (assign_range_result != nullptr) {
      assign_result =
          &assign_range_result->results[num_passes - min_passes];
    } else {
      assign_result_storage = AssignPassesGreedy(d, active, num_passes, pool);
      assign_result = &assign_result_storage;
      fprintf(stderr,
              "PLANNER: AssignPassesGreedy took %.2f ms total "
              "(batch %.2f ms, sequential %.2f ms)\n",
              NanosToMs(assign_result->timings.total_ns),
              NanosToMs(assign_result->timings.batch_ns),
              NanosToMs(assign_result->timings.sequential_ns));
      fflush(stderr);
    }
    const PassAssignment& pass_assignment = assign_result->pass_assignment;

    auto start_build_stream = PlannerClock::now();
    std::vector<uint32_t> pass_offsets;
    JXL_ASSIGN_OR_RETURN(std::vector<ACEntry> pass_stream,
                         BuildPassStream(d, active, pass_assignment, num_passes,
                                         &pass_offsets, pool));
    auto end_build_stream = PlannerClock::now();
    fprintf(stderr, "PLANNER: BuildPassStream took %.2f ms\n",
            NanosToMs(ElapsedNanos(start_build_stream, end_build_stream)));
    fflush(stderr);

    // Per-stage timing accumulators (atomics for lock-free accumulation from
    // multiple threads; relaxed ordering is sufficient — visibility is ensured
    // by the `RunOnPool` join barrier).
    std::atomic<int64_t> rough_opt_ns(0);
    std::atomic<int64_t> cluster_ns(0);
    std::atomic<int64_t> rough_eval_ns(0);
    std::atomic<int64_t> refine_ns(0);
    std::atomic<int64_t> refined_eval_ns(0);
    std::atomic<uint32_t> processed_candidates(0);

    // Per-thread best result. Each thread writes only its own slot
    // `thread_best[thread_id]`, so no synchronization is needed. After
    // `RunOnPool` returns, a single-threaded reduction picks the overall best.
    struct PassThreadBest {
      PassSearchResult result;
      uint32_t best_idx = std::numeric_limits<uint32_t>::max();
    };
    std::vector<PassThreadBest> thread_best;
    // One `PartitioningCtx` per thread.
    std::vector<PartitioningCtx> ctx_pool;
    auto start_candidate_loop = PlannerClock::now();
    JXL_RETURN_IF_ERROR(RunOnPool(
        pool, 0, static_cast<uint32_t>(candidates.size()),
        [&](size_t num_threads) -> Status {
          ctx_pool.reserve(num_threads);
          for (size_t i = 0; i < num_threads; ++i) {
            ctx_pool.emplace_back(opt_data);
          }
          thread_best.resize(num_threads);
          return true;
        },
        [&](uint32_t idx, size_t thread_id) -> Status {
          PartitioningCtx& ctx = ctx_pool[thread_id];
          const FactorizationCandidate& candidate = candidates[idx];

          // Stage 1: rough threshold optimization from the candidate's init
          // point. Produces an initial `ThresholdSet`.
          auto start_rough_opt = PlannerClock::now();
          FixedPointCost rough_unclustered_cost = 0;
          ThresholdSet rough_thresholds =
              ctx.OptimizeThresholds(candidate.init, pass_stream,
                                     effort.main_m_target, effort.main_iters,
                                     &rough_unclustered_cost);
          auto end_rough_opt = PlannerClock::now();
          rough_opt_ns.fetch_add(
              ElapsedNanos(start_rough_opt, end_rough_opt),
              std::memory_order_relaxed);

          // Stage 2: cluster the (cell, pass) contexts implied by the rough
          // thresholds into at most `target_clusters` groups.
          auto start_cluster = PlannerClock::now();
          JXL_ASSIGN_OR_RETURN(
              ClusterResult cluster_result,
              ClusterContextsPassAware(d, rough_thresholds, pass_stream,
                                       pass_offsets, num_passes,
                                       target_clusters));
          auto end_cluster = PlannerClock::now();
          cluster_ns.fetch_add(ElapsedNanos(start_cluster, end_cluster),
                               std::memory_order_relaxed);

          auto start_rough_eval = PlannerClock::now();
          JXL_ASSIGN_OR_RETURN(
              ModelEvaluation rough_eval,
              EvaluatePassAwareModel(d, rough_thresholds, cluster_result.ctx_map,
                                     cluster_result.num_clusters, pass_assignment,
                                     num_passes, pass_stream, pass_offsets));
          auto end_rough_eval = PlannerClock::now();
          rough_eval_ns.fetch_add(
              ElapsedNanos(start_rough_eval, end_rough_eval),
              std::memory_order_relaxed);

          // Stage 3: refine thresholds around the rough optimum.
          auto start_refine = PlannerClock::now();
          ThresholdSet refined_thresholds =
              RefinePassAwareThresholds(ctx, rough_thresholds, pass_stream,
                                        effort);
          auto end_refine = PlannerClock::now();
          refine_ns.fetch_add(ElapsedNanos(start_refine, end_refine),
                              std::memory_order_relaxed);

          auto start_refined_eval = PlannerClock::now();
          JXL_ASSIGN_OR_RETURN(
              ModelEvaluation refined_eval,
              EvaluatePassAwareModel(d, refined_thresholds, cluster_result.ctx_map,
                                     cluster_result.num_clusters, pass_assignment,
                                     num_passes, pass_stream, pass_offsets));
          auto end_refined_eval = PlannerClock::now();
          refined_eval_ns.fetch_add(
              ElapsedNanos(start_refined_eval, end_refined_eval),
              std::memory_order_relaxed);
          // relaxed is safe: only used for diagnostic printing after join.
          processed_candidates.fetch_add(1, std::memory_order_relaxed);

          // Pick whichever of rough/refined scored better.
          const bool refined_is_better =
              refined_eval.total_cost() < rough_eval.total_cost();
          const ThresholdSet& best_thresholds =
              refined_is_better ? refined_thresholds : rough_thresholds;
          const ModelEvaluation& best_eval =
              refined_is_better ? refined_eval : rough_eval;
          // Per-thread best; no lock needed. Tiebreaker: lowest candidate
          // index for deterministic results independent of thread count.
          auto& local = thread_best[thread_id];
          if (best_eval.total_cost() < local.result.total_cost ||
              (best_eval.total_cost() == local.result.total_cost &&
               idx < local.best_idx)) {
            local.result.thresholds = best_thresholds;
            local.result.ctx_map = cluster_result.ctx_map;
            local.result.pass_assignment = pass_assignment;
            local.result.num_passes = num_passes;
            local.result.num_clusters = cluster_result.num_clusters;
            local.result.ac_cost = best_eval.ac_cost;
            local.result.nz_cost = best_eval.nz_cost;
            local.result.signalling_overhead = best_eval.signalling_overhead;
            local.result.total_cost = best_eval.total_cost();
            local.best_idx = idx;
          }
          return true;
        },
        "JpegCtxPasses"));
    // Deterministic reduction: merge per-thread bests into the overall
    // best_result. Tiebreaker: lowest candidate index. This also computes
    // best_pass_cost (the best cost for this particular pass configuration)
    // for diagnostic printing below.
    FixedPointCost best_pass_cost = std::numeric_limits<FixedPointCost>::max();
    for (auto& tb : thread_best) {
      best_pass_cost = std::min(tb.result.total_cost, best_pass_cost);
      if (tb.result.total_cost < best_result.total_cost ||
          (tb.result.total_cost == best_result.total_cost &&
           tb.best_idx < overall_best_idx)) {
        best_result = std::move(tb.result);
        overall_best_idx = tb.best_idx;
      }
    }

    auto end_candidate_loop = PlannerClock::now();
    const uint32_t num_processed = processed_candidates.load(
        std::memory_order_relaxed);
    fprintf(stderr,
            "PLANNER: Candidate loop took %.2f ms wall time (%u candidates)\n",
            NanosToMs(ElapsedNanos(start_candidate_loop, end_candidate_loop)),
            num_processed);
    if (num_processed != 0) {
      // relaxed is safe here: RunOnPool join establishes happens-before, so
      // all fetch_add writes from worker threads are visible.
      const double rough_opt_ms =
          NanosToMs(rough_opt_ns.load(std::memory_order_relaxed));
      const double cluster_ms =
          NanosToMs(cluster_ns.load(std::memory_order_relaxed));
      const double rough_eval_ms =
          NanosToMs(rough_eval_ns.load(std::memory_order_relaxed));
      const double refine_ms =
          NanosToMs(refine_ns.load(std::memory_order_relaxed));
      const double refined_eval_ms =
          NanosToMs(refined_eval_ns.load(std::memory_order_relaxed));
      fprintf(stderr,
              "PLANNER: Candidate stages (sum/avg ms): \nrough_opt=%.2f/%.2f "
              "\ncluster=%.2f/%.2f \nrough_eval=%.2f/%.2f "
              "\nrefine=%.2f/%.2f \nrefined_eval=%.2f/%.2f\n",
              rough_opt_ms, rough_opt_ms / num_processed, cluster_ms,
              cluster_ms / num_processed, rough_eval_ms,
              rough_eval_ms / num_processed, refine_ms,
              refine_ms / num_processed, refined_eval_ms,
              refined_eval_ms / num_processed);
    }
    fflush(stderr);
    if (best_pass_cost != std::numeric_limits<FixedPointCost>::max()) {
      fprintf(stderr,
              "PLANNER: Best cost for %u passes = %.2f bits\n",
              num_passes, bit_cost(best_pass_cost));
      fflush(stderr);
    }
    auto end_pass_config = std::chrono::high_resolution_clock::now();
    fprintf(stderr, "PLANNER: Pass configuration %u took %.2f ms\n", num_passes,
            std::chrono::duration<double, std::milli>(end_pass_config - start_pass_config).count());
    fflush(stderr);
  }

  if (best_result.total_cost == std::numeric_limits<FixedPointCost>::max()) {
    return JXL_FAILURE("Pass-aware search did not produce a result");
  }
  return best_result;
}

// Convenience overload that ranks/trims factorization candidates first.
StatusOr<PassSearchResult> SearchPassAwareContextModel(
    std::shared_ptr<const JPEGOptData> opt_data,
    const JPEGCtxEffortParams& effort, ThreadPool* pool) {
  auto start_rank = std::chrono::high_resolution_clock::now();
  JXL_ASSIGN_OR_RETURN(std::vector<FactorizationCandidate> candidates,
                       RankAndTrimFactorizations(opt_data, effort, pool));
  auto end_rank = std::chrono::high_resolution_clock::now();
  fprintf(stderr, "PLANNER: RankAndTrimFactorizations took %.2f ms (found %zu candidates)\n",
          std::chrono::duration<double, std::milli>(end_rank - start_rank).count(),
          candidates.size());
  fflush(stderr);
  return ::jxl::SearchPassAwareContextModel(opt_data, candidates, effort, pool);
}

StatusOr<BiclusterSearchResult> SearchBiclusteredContextModelThresholdFirst(
    std::shared_ptr<const JPEGOptData> opt_data,
    const std::vector<FactorizationCandidate>& candidates,
    const JPEGCtxEffortParams& effort, ThreadPool* pool) {
  if (candidates.empty()) {
    return JXL_FAILURE("Biclustered search requires at least one candidate");
  }

  const JPEGOptData& d = *opt_data;
  const ActiveRawBins active = BuildActiveRawBins(d);
  const uint32_t min_num_passes =
      effort.optimize_passes_num <= 0
          ? 1
          : static_cast<uint32_t>(effort.optimize_passes_num);
  const uint32_t max_num_passes =
      effort.optimize_passes_num <= 0
          ? ComputeMaxNumPasses(d)
          : static_cast<uint32_t>(effort.optimize_passes_num);
  const uint32_t target_clusters =
      kMaxClusters - static_cast<uint32_t>(d.channels == 1);

  fprintf(stderr,
          "PLANNER: [bicluster-threshold-first] Selecting fixed thresholds with 1-pass search\n");
  fflush(stderr);
  auto start_seed = PlannerClock::now();
  JPEGCtxEffortParams seed_effort = effort;
  seed_effort.use_bicluster_search = false;
  seed_effort.optimize_passes_num = 1;
  seed_effort.bicluster_threshold_first = false;
  JXL_ASSIGN_OR_RETURN(
      PassSearchResult threshold_seed,
      ::jxl::SearchPassAwareContextModel(opt_data, candidates, seed_effort, pool));
  auto end_seed = PlannerClock::now();
  fprintf(stderr,
          "PLANNER: [bicluster-threshold-first] Seed search took %.2f ms\n",
          NanosToMs(ElapsedNanos(start_seed, end_seed)));
  fflush(stderr);

  PrunedCtxMapResult pruned_seed = PruneDeadThresholdsFromCtxMap(
      threshold_seed.thresholds, threshold_seed.ctx_map, d.channels);
  ThresholdSet seed_thresholds = std::move(pruned_seed.thresholds);
  const uint32_t seed_num_cells = static_cast<uint32_t>(
      (seed_thresholds.TY().size() + 1) * (seed_thresholds.TCb().size() + 1) *
      (seed_thresholds.TCr().size() + 1));
  const FixedRows fixed_rows = BuildFixedRows(d, seed_thresholds);
  fprintf(stderr,
          "PLANNER: [bicluster-threshold-first] Using seed threshold grid with %u cells\n",
          seed_num_cells);
  fflush(stderr);

  auto start_assign = PlannerClock::now();
  AssignPassesRangeResult assign_range_result = AssignPassesGreedyAllKFixedRows(
      d, active, fixed_rows, d.channels * seed_num_cells, min_num_passes,
      max_num_passes, pool);
  auto end_assign = PlannerClock::now();
  fprintf(stderr,
          "PLANNER: [bicluster-threshold-first] AssignPassesGreedyAllKFixedRows took %.2f ms total "
          "(batch %.2f ms, sequential %.2f ms)\n",
          NanosToMs(assign_range_result.shared_timings.total_ns),
          NanosToMs(assign_range_result.shared_timings.batch_ns),
          NanosToMs(assign_range_result.shared_timings.sequential_ns));
  fprintf(stderr,
          "PLANNER: [bicluster-threshold-first] Fixed-row pass assignment stage took %.2f ms wall time\n",
          NanosToMs(ElapsedNanos(start_assign, end_assign)));
  fflush(stderr);

  BiclusterSearchResult best_result;
  best_result.total_cost = std::numeric_limits<FixedPointCost>::max();
  for (uint32_t num_passes = min_num_passes; num_passes <= max_num_passes;
       ++num_passes) {
    auto start_pass_config = PlannerClock::now();
    fprintf(stderr,
            "PLANNER: [bicluster-threshold-first] Testing configuration with %u passes\n",
            num_passes);
    fflush(stderr);

    const PassAssignment& pass_assignment =
        assign_range_result.results[num_passes - min_num_passes].pass_assignment;
    const NZBlockCache nz_cache =
        BuildNZBlockCache(d, pass_assignment, num_passes);

    auto start_build_stream = PlannerClock::now();
    std::vector<uint32_t> pass_offsets;
    JXL_ASSIGN_OR_RETURN(
        std::vector<ACEntry> pass_stream,
        BuildPassStream(d, active, pass_assignment, num_passes, &pass_offsets,
                        pool));
    auto end_build_stream = PlannerClock::now();

    auto start_rough_opt = PlannerClock::now();
    PartitioningCtx ctx(opt_data);
    FixedPointCost rough_unclustered_cost = 0;
    ThresholdSet rough_thresholds =
        ctx.OptimizeThresholds(seed_thresholds, pass_stream,
                               effort.main_m_target, effort.main_iters,
                               &rough_unclustered_cost);
    auto end_rough_opt = PlannerClock::now();

    auto start_build_rows = PlannerClock::now();
    JXL_ASSIGN_OR_RETURN(
        RowSliceState rough_state,
        BuildRowSliceState(d, rough_thresholds, pass_assignment, num_passes,
                           nz_cache));
    auto end_build_rows = PlannerClock::now();

    auto start_row_cluster = PlannerClock::now();
    JXL_ASSIGN_OR_RETURN(
        ClusterResult rough_cluster_result,
        ClusterRowsBiclustered(d, rough_state.rows,
                               std::min(target_clusters,
                                        effort.bicluster_row_budget)));
    auto end_row_cluster = PlannerClock::now();

    std::vector<uint32_t> rough_num_prototypes;
    auto start_eval = PlannerClock::now();
    JXL_ASSIGN_OR_RETURN(
        ModelEvaluation rough_eval,
        EvaluateBiclusterState(
            d, rough_thresholds, rough_cluster_result.ctx_map,
            rough_cluster_result.num_clusters, pass_assignment, num_passes,
            effort.bicluster_proto_budget_per_pass, rough_state.rows,
            &rough_num_prototypes, best_result.total_cost));
    auto end_eval = PlannerClock::now();

    bool refined_is_better = false;
    ThresholdSet refined_thresholds;
    ClusterResult refined_cluster_result;
    ModelEvaluation refined_eval;
    std::vector<uint32_t> refined_num_prototypes;
    if (effort.bicluster_refine_thresholds) {
      refined_thresholds = RefinePassAwareThresholds(ctx, rough_thresholds,
                                                     pass_stream, effort);
      JXL_ASSIGN_OR_RETURN(
          RowSliceState refined_state,
          RefineRowSliceState(d, refined_thresholds, pass_assignment, num_passes,
                              rough_state, nz_cache));
      JXL_ASSIGN_OR_RETURN(
          refined_cluster_result,
          ClusterRowsBiclustered(d, refined_state.rows,
                                 std::min(target_clusters,
                                          effort.bicluster_row_budget)));
      JXL_ASSIGN_OR_RETURN(
          refined_eval,
          EvaluateBiclusterState(
              d, refined_thresholds, refined_cluster_result.ctx_map,
              refined_cluster_result.num_clusters, pass_assignment, num_passes,
              effort.bicluster_proto_budget_per_pass, refined_state.rows,
              &refined_num_prototypes,
              std::min(best_result.total_cost, rough_eval.total_cost())));
      refined_is_better =
          refined_eval.total_cost() < rough_eval.total_cost();
    }

    const ThresholdSet& best_thresholds =
        refined_is_better ? refined_thresholds : rough_thresholds;
    const ClusterResult& best_cluster_result =
        refined_is_better ? refined_cluster_result : rough_cluster_result;
    const ModelEvaluation& best_eval =
        refined_is_better ? refined_eval : rough_eval;
    const std::vector<uint32_t>& best_num_prototypes =
        refined_is_better ? refined_num_prototypes : rough_num_prototypes;
    if (best_eval.total_cost() < best_result.total_cost) {
      best_result.thresholds = best_thresholds;
      best_result.ctx_map = best_cluster_result.ctx_map;
      best_result.pass_assignment = pass_assignment;
      best_result.num_passes = num_passes;
      best_result.num_cells = static_cast<uint32_t>(
          (best_thresholds.TY().size() + 1) *
          (best_thresholds.TCb().size() + 1) *
          (best_thresholds.TCr().size() + 1));
      best_result.num_row_clusters = best_cluster_result.num_clusters;
      best_result.num_prototypes_per_pass = best_num_prototypes;
      best_result.total_num_prototypes = 0;
      for (uint32_t n : best_num_prototypes) best_result.total_num_prototypes += n;
      best_result.ac_cost =
          best_eval.corrected_entropy_cost >= 0 ? best_eval.corrected_entropy_cost
                                                : best_eval.ac_cost;
      best_result.nz_cost = best_eval.nz_cost;
      best_result.signalling_overhead = best_eval.signalling_overhead;
      best_result.total_cost = best_eval.total_cost();
    }

    fprintf(stderr,
            "PLANNER: [bicluster-threshold-first] Stages: build_stream=%.2f ms rough_opt=%.2f ms build_rows=%.2f ms row_cluster=%.2f ms eval=%.2f ms\n",
            NanosToMs(ElapsedNanos(start_build_stream, end_build_stream)),
            NanosToMs(ElapsedNanos(start_rough_opt, end_rough_opt)),
            NanosToMs(ElapsedNanos(start_build_rows, end_build_rows)),
            NanosToMs(ElapsedNanos(start_row_cluster, end_row_cluster)),
            NanosToMs(ElapsedNanos(start_eval, end_eval)));
    fprintf(stderr,
            "PLANNER: [bicluster-threshold-first] Best cost for %u passes = %.2f bits\n",
            num_passes, bit_cost(best_eval.total_cost()));
    auto end_pass_config = PlannerClock::now();
    fprintf(stderr,
            "PLANNER: [bicluster-threshold-first] Pass configuration %u took %.2f ms\n",
            num_passes,
            std::chrono::duration<double, std::milli>(end_pass_config -
                                                      start_pass_config)
                .count());
    fflush(stderr);
  }

  if (best_result.total_cost == std::numeric_limits<FixedPointCost>::max()) {
    return JXL_FAILURE(
        "Threshold-first biclustered search did not produce a result");
  }
  return best_result;
}

// Biclustering-prototype search on a fixed candidate list. This currently
// shares the pass-aware threshold optimization and row clustering steps, then
// re-scores each candidate on the richer `(row, pass, slice)` lattice.
StatusOr<BiclusterSearchResult> SearchBiclusteredContextModel(
    std::shared_ptr<const JPEGOptData> opt_data,
    const std::vector<FactorizationCandidate>& candidates,
    const JPEGCtxEffortParams& effort, ThreadPool* pool) {
  if (effort.bicluster_threshold_first) {
    return SearchBiclusteredContextModelThresholdFirst(opt_data, candidates,
                                                       effort, pool);
  }
  if (candidates.empty()) {
    return JXL_FAILURE("Biclustered search requires at least one candidate");
  }

  const JPEGOptData& d = *opt_data;
  const ActiveRawBins active = BuildActiveRawBins(d);
  const uint32_t min_num_passes =
      effort.optimize_passes_num <= 0
          ? 1
          : static_cast<uint32_t>(effort.optimize_passes_num);
  const uint32_t max_num_passes =
      effort.optimize_passes_num <= 0
          ? ComputeMaxNumPasses(d)
          : static_cast<uint32_t>(effort.optimize_passes_num);
  const uint32_t target_clusters =
      kMaxClusters - static_cast<uint32_t>(d.channels == 1);

  std::unique_ptr<AssignPassesRangeResult> assign_range_result;
  if (max_num_passes > min_num_passes) {
    assign_range_result = jxl::make_unique<AssignPassesRangeResult>(
        AssignPassesGreedyAllK(d, active, min_num_passes, max_num_passes,
                               pool));
    fprintf(stderr,
            "PLANNER: [bicluster] AssignPassesGreedyAllK took %.2f ms total "
            "(batch %.2f ms, sequential %.2f ms)\n",
            NanosToMs(assign_range_result->shared_timings.total_ns),
            NanosToMs(assign_range_result->shared_timings.batch_ns),
            NanosToMs(assign_range_result->shared_timings.sequential_ns));
    fflush(stderr);
  }
  // Accumulates the best result across all pass configurations. Updated by
  // deterministic reduction after each pass loop.
  BiclusterSearchResult best_result;
  best_result.total_cost = std::numeric_limits<FixedPointCost>::max();
  // Tiebreaker: lowest candidate index for deterministic results.
  uint32_t overall_best_idx = std::numeric_limits<uint32_t>::max();

  for (uint32_t num_passes = min_num_passes; num_passes <= max_num_passes;
       ++num_passes) {
    auto start_pass_config = std::chrono::high_resolution_clock::now();
    fprintf(stderr, "PLANNER: [bicluster] Testing configuration with %u passes\n",
            num_passes);
    fflush(stderr);

    AssignPassesResult assign_result_storage;
    const AssignPassesResult* assign_result = nullptr;
    if (assign_range_result != nullptr) {
      assign_result =
          &assign_range_result->results[num_passes - min_num_passes];
    } else {
      assign_result_storage = AssignPassesGreedy(d, active, num_passes, pool);
      assign_result = &assign_result_storage;
      fprintf(stderr,
              "PLANNER: [bicluster] AssignPassesGreedy took %.2f ms total "
              "(batch %.2f ms, sequential %.2f ms)\n",
              NanosToMs(assign_result->timings.total_ns),
              NanosToMs(assign_result->timings.batch_ns),
              NanosToMs(assign_result->timings.sequential_ns));
      fflush(stderr);
    }
    const PassAssignment& pass_assignment = assign_result->pass_assignment;
    const NZBlockCache nz_cache =
        BuildNZBlockCache(d, pass_assignment, num_passes);

    auto start_build_stream = PlannerClock::now();
    std::vector<uint32_t> pass_offsets;
    JXL_ASSIGN_OR_RETURN(std::vector<ACEntry> pass_stream,
                         BuildPassStream(d, active, pass_assignment,
                                         num_passes, &pass_offsets, pool));
    auto end_build_stream = PlannerClock::now();
    fprintf(stderr, "PLANNER: [bicluster] BuildPassStream took %.2f ms\n",
            NanosToMs(ElapsedNanos(start_build_stream, end_build_stream)));
    fflush(stderr);

    // Per-stage timing accumulators (relaxed atomics; join ensures visibility).
    std::atomic<int64_t> rough_opt_ns(0);
    std::atomic<int64_t> rough_row_cluster_ns(0);
    std::atomic<int64_t> rough_build_rows_ns(0);
    std::atomic<int64_t> rough_eval_ns(0);
    std::atomic<int64_t> refine_ns(0);
    std::atomic<int64_t> refined_row_cluster_ns(0);
    std::atomic<int64_t> refined_build_rows_ns(0);
    std::atomic<int64_t> refined_eval_ns(0);
    std::atomic<uint32_t> processed_candidates(0);

    // Per-thread best result.
    struct BiclusterThreadBest {
      BiclusterSearchResult result;
      uint32_t best_idx = std::numeric_limits<uint32_t>::max();
    };
    std::vector<BiclusterThreadBest> thread_best;
    std::vector<PartitioningCtx> ctx_pool;
    auto start_candidate_loop = PlannerClock::now();
    JXL_RETURN_IF_ERROR(RunOnPool(
        pool, 0, static_cast<uint32_t>(candidates.size()),
        [&](size_t num_threads) -> Status {
          ctx_pool.reserve(num_threads);
          for (size_t i = 0; i < num_threads; ++i) {
            ctx_pool.emplace_back(opt_data);
          }
          thread_best.resize(num_threads);
          return true;
        },
        [&](uint32_t idx, size_t thread_id) -> Status {
          PartitioningCtx& ctx = ctx_pool[thread_id];
          const FactorizationCandidate& candidate = candidates[idx];
          auto& local = thread_best[thread_id];

          // Stage 1: rough threshold optimization.
          auto start_rough_opt = PlannerClock::now();
          FixedPointCost rough_unclustered_cost = 0;
          ThresholdSet rough_thresholds =
              ctx.OptimizeThresholds(candidate.init, pass_stream,
                                     effort.main_m_target, effort.main_iters,
                                     &rough_unclustered_cost);
          auto end_rough_opt = PlannerClock::now();
          rough_opt_ns.fetch_add(ElapsedNanos(start_rough_opt, end_rough_opt),
                                 std::memory_order_relaxed);

          // Shared helper: given a row-slice state, cluster rows biclustered
          // and evaluate the full bicluster model. The refined path can reuse
          // the rough state and update only the blocks whose row changed.
          std::vector<uint32_t> rough_num_prototypes;
          ClusterResult rough_cluster_result;
          auto evaluate_state =
              [&](const ThresholdSet& thresholds, const RowSliceState& state,
                  std::atomic<int64_t>* row_cluster_ns,
                  std::atomic<int64_t>* eval_ns,
                  ClusterResult* cluster_result,
                  std::vector<uint32_t>* num_prototypes,
                  FixedPointCost cutoff)
              -> StatusOr<ModelEvaluation> {
            auto start_row_cluster = PlannerClock::now();
            JXL_ASSIGN_OR_RETURN(
                *cluster_result,
                ClusterRowsBiclustered(d, state.rows,
                                       std::min(target_clusters,
                                                effort.bicluster_row_budget)));
            auto end_row_cluster = PlannerClock::now();
            row_cluster_ns->fetch_add(
                ElapsedNanos(start_row_cluster, end_row_cluster),
                std::memory_order_relaxed);

            auto start_eval = PlannerClock::now();
            JXL_ASSIGN_OR_RETURN(ModelEvaluation eval, EvaluateBiclusterState(
                d, thresholds, cluster_result->ctx_map,
                cluster_result->num_clusters, pass_assignment,
                num_passes, effort.bicluster_proto_budget_per_pass, state.rows,
                num_prototypes, cutoff));
            auto end_eval = PlannerClock::now();
            eval_ns->fetch_add(ElapsedNanos(start_eval, end_eval),
                               std::memory_order_relaxed);
            return eval;
          };
          auto start_build_rows = PlannerClock::now();
          JXL_ASSIGN_OR_RETURN(
              RowSliceState rough_state,
              BuildRowSliceState(d, rough_thresholds, pass_assignment,
                                 num_passes, nz_cache));
          auto end_build_rows = PlannerClock::now();
          rough_build_rows_ns.fetch_add(
              ElapsedNanos(start_build_rows, end_build_rows),
              std::memory_order_relaxed);
          JXL_ASSIGN_OR_RETURN(ModelEvaluation rough_eval,
                               evaluate_state(rough_thresholds,
                                              rough_state,
                                              &rough_row_cluster_ns,
                                              &rough_eval_ns,
                                              &rough_cluster_result,
                                              &rough_num_prototypes,
                                              local.result.total_cost));
          PrunedCtxMapResult pruned_rough =
              PruneDeadThresholdsFromCtxMap(rough_thresholds,
                                           rough_cluster_result.ctx_map,
                                           d.channels);
          const bool rough_pruned =
              (pruned_rough.thresholds.T != rough_thresholds.T);
          ThresholdSet rough_output_thresholds = rough_thresholds;
          ClusterResult rough_output_cluster_result = rough_cluster_result;
          if (rough_pruned) {
            rough_output_thresholds = std::move(pruned_rough.thresholds);
            rough_output_cluster_result.ctx_map = std::move(pruned_rough.ctx_map);
          }

          auto start_refine = PlannerClock::now();
          ThresholdSet refined_thresholds =
              RefinePassAwareThresholds(ctx,
                                        rough_pruned ? rough_output_thresholds
                                                     : rough_thresholds,
                                        pass_stream, effort);
          auto end_refine = PlannerClock::now();
          refine_ns.fetch_add(ElapsedNanos(start_refine, end_refine),
                              std::memory_order_relaxed);
          auto start_refined_build_rows = PlannerClock::now();
          RowSliceState refined_seed_state;
          const RowSliceState* refine_base_state = &rough_state;
          if (rough_pruned) {
            JXL_ASSIGN_OR_RETURN(
                refined_seed_state,
                RefineRowSliceState(d, rough_output_thresholds, pass_assignment,
                                    num_passes, rough_state, nz_cache));
            refine_base_state = &refined_seed_state;
          }
          JXL_ASSIGN_OR_RETURN(
              RowSliceState refined_state,
              RefineRowSliceState(d, refined_thresholds, pass_assignment,
                                  num_passes, *refine_base_state, nz_cache));
          auto end_refined_build_rows = PlannerClock::now();
          refined_build_rows_ns.fetch_add(
              ElapsedNanos(start_refined_build_rows, end_refined_build_rows),
              std::memory_order_relaxed);
          std::vector<uint32_t> refined_num_prototypes;
          ClusterResult refined_cluster_result;
          JXL_ASSIGN_OR_RETURN(ModelEvaluation refined_eval,
                               evaluate_state(refined_thresholds,
                                              refined_state,
                                              &refined_row_cluster_ns,
                                              &refined_eval_ns,
                                              &refined_cluster_result,
                                              &refined_num_prototypes,
                                              std::min(local.result.total_cost,
                                                       rough_eval.total_cost())));
          processed_candidates.fetch_add(1, std::memory_order_relaxed);

          // Pick whichever of rough/refined scored better.
          const bool refined_is_better =
              refined_eval.total_cost() < rough_eval.total_cost();
          const ThresholdSet& best_thresholds =
              refined_is_better ? refined_thresholds : rough_output_thresholds;
          const ModelEvaluation& best_eval =
              refined_is_better ? refined_eval : rough_eval;
          const ClusterResult& best_cluster_result =
              refined_is_better ? refined_cluster_result
                                : rough_output_cluster_result;
          const std::vector<uint32_t>& best_num_prototypes =
              refined_is_better ? refined_num_prototypes : rough_num_prototypes;
          // Per-thread best; no lock needed. Tiebreaker: lowest candidate
          // index for deterministic results independent of thread count.
          if (best_eval.total_cost() < local.result.total_cost ||
              (best_eval.total_cost() == local.result.total_cost &&
               idx < local.best_idx)) {
            local.result.thresholds = best_thresholds;
            local.result.ctx_map = best_cluster_result.ctx_map;
            local.result.pass_assignment = pass_assignment;
            local.result.num_passes = num_passes;
            local.result.num_cells = static_cast<uint32_t>(
                best_cluster_result.ctx_map.size() / d.channels);
            local.result.num_row_clusters = best_cluster_result.num_clusters;
            local.result.num_prototypes_per_pass = best_num_prototypes;
            local.result.total_num_prototypes = 0;
            for (uint32_t n : best_num_prototypes) {
              local.result.total_num_prototypes += n;
            }
            // Prefer the ANS-clustering-corrected cost when available
            // (non-negative); fall back to the raw AC entropy cost.
            local.result.ac_cost = best_eval.corrected_entropy_cost >= 0
                                       ? best_eval.corrected_entropy_cost
                                       : best_eval.ac_cost;
            local.result.nz_cost = best_eval.nz_cost;
            local.result.signalling_overhead = best_eval.signalling_overhead;
            local.result.total_cost = best_eval.total_cost();
            local.best_idx = idx;
          }
          return true;
        },
        "JpegCtxBicluster"));
    // Deterministic reduction: pick best across all threads.
    // Tiebreaker: lowest candidate index.
    FixedPointCost best_pass_cost = std::numeric_limits<FixedPointCost>::max();
    for (auto& tb : thread_best) {
      best_pass_cost = std::min(tb.result.total_cost, best_pass_cost);
      if (tb.result.total_cost < best_result.total_cost ||
          (tb.result.total_cost == best_result.total_cost &&
           tb.best_idx < overall_best_idx)) {
        best_result = std::move(tb.result);
        overall_best_idx = tb.best_idx;
      }
    }

    auto end_candidate_loop = PlannerClock::now();
    const uint32_t num_processed =
        processed_candidates.load(std::memory_order_relaxed);
    fprintf(stderr,
            "PLANNER: [bicluster] Candidate loop took %.2f ms wall time (%u candidates)\n",
            NanosToMs(ElapsedNanos(start_candidate_loop, end_candidate_loop)),
            num_processed);
    if (num_processed != 0) {
      // relaxed is safe here: `RunOnPool` join establishes happens-before, so
      // all `fetch_add` writes from worker threads are visible.
      const double rough_opt_ms =
          NanosToMs(rough_opt_ns.load(std::memory_order_relaxed));
      const double rough_row_cluster_ms =
          NanosToMs(rough_row_cluster_ns.load(std::memory_order_relaxed));
      const double rough_build_rows_ms =
          NanosToMs(rough_build_rows_ns.load(std::memory_order_relaxed));
      const double rough_eval_ms =
          NanosToMs(rough_eval_ns.load(std::memory_order_relaxed));
      const double refine_ms =
          NanosToMs(refine_ns.load(std::memory_order_relaxed));
      const double refined_row_cluster_ms =
          NanosToMs(refined_row_cluster_ns.load(std::memory_order_relaxed));
      const double refined_build_rows_ms =
          NanosToMs(refined_build_rows_ns.load(std::memory_order_relaxed));
      const double refined_eval_ms =
          NanosToMs(refined_eval_ns.load(std::memory_order_relaxed));
      fprintf(stderr,
              "PLANNER: [bicluster] Candidate stages (sum/avg ms): "
              "rough_opt=%.2f/%.2f rough_row_cluster=%.2f/%.2f "
              "rough_build_rows=%.2f/%.2f rough_eval=%.2f/%.2f "
              "refine=%.2f/%.2f refined_row_cluster=%.2f/%.2f "
              "refined_build_rows=%.2f/%.2f "
              "refined_eval=%.2f/%.2f\n",
              rough_opt_ms, rough_opt_ms / num_processed, rough_row_cluster_ms,
              rough_row_cluster_ms / num_processed, rough_build_rows_ms,
              rough_build_rows_ms / num_processed, rough_eval_ms,
              rough_eval_ms / num_processed, refine_ms,
              refine_ms / num_processed, refined_row_cluster_ms,
              refined_row_cluster_ms / num_processed, refined_build_rows_ms,
              refined_build_rows_ms / num_processed, refined_eval_ms,
              refined_eval_ms / num_processed);
    }
    fflush(stderr);
    if (best_pass_cost != std::numeric_limits<FixedPointCost>::max()) {
      fprintf(stderr,
              "PLANNER: [bicluster] Best cost for %u passes = %.2f bits\n",
              num_passes, bit_cost(best_pass_cost));
      fflush(stderr);
    }
    auto end_pass_config = std::chrono::high_resolution_clock::now();
    fprintf(stderr, "PLANNER: [bicluster] Pass configuration %u took %.2f ms\n",
            num_passes,
            std::chrono::duration<double, std::milli>(end_pass_config -
                                                      start_pass_config)
                .count());
    fflush(stderr);
  }

  if (best_result.total_cost == std::numeric_limits<FixedPointCost>::max()) {
    return JXL_FAILURE("Biclustered search did not produce a result");
  }
  return best_result;
}

// Convenience overload that ranks/trims factorization candidates first.
StatusOr<BiclusterSearchResult> SearchBiclusteredContextModel(
    std::shared_ptr<const JPEGOptData> opt_data,
    const JPEGCtxEffortParams& effort, ThreadPool* pool) {
  JXL_ASSIGN_OR_RETURN(std::vector<FactorizationCandidate> candidates,
                       RankAndTrimFactorizations(opt_data, effort, pool));
  return ::jxl::SearchBiclusteredContextModel(opt_data, candidates, effort, pool);
}

}  // namespace jxl
