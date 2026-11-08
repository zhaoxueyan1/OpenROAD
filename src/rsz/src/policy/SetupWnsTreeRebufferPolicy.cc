// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "SetupWnsTreeRebufferPolicy.hh"

#include <algorithm>
#include <map>
#include <optional>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "MoveCommitter.hh"
#include "OptimizerTypes.hh"
#include "Rebuffer.hh"
#include "RepairTargetCollector.hh"
#include "db_sta/dbNetwork.hh"
#include "est/EstimateParasitics.h"
#include "odb/db.h"
#include "odb/geom.h"
#include "policy/OptimizationPolicy.hh"
#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/Fuzzy.hh"
#include "sta/Graph.hh"
#include "sta/GraphClass.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/Path.hh"
#include "sta/PathExpanded.hh"
#include "sta/Scene.hh"
#include "sta/TimingArc.hh"
#include "utl/Logger.h"
#include "utl/timer.h"

namespace rsz {

using std::pair;
using utl::RSZ;

namespace {
constexpr int kDelayDigits = 3;
}  // namespace

void SetupWnsTreeRebufferPolicy::buildMainMoveSequence(const bool log_sequence)
{
  move_sequence_.clear();
  pushMoveIfEnabled(!config_.skip_buffering, MoveType::kTreeRebuffer);
  pushMoveIfEnabled(!config_.skip_vt_swap && hasVtSwapCells(),
                    MoveType::kVtSwap);
  move_sequence_.push_back(MoveType::kSizeUp);
  activateMoveSequence(log_sequence);
}

std::vector<SuperStageDriver>
SetupWnsTreeRebufferPolicy::rankSuperStageRootDrivers(
    sta::PathExpanded& expanded,
    const sta::Scene* corner,
    const int lib_ap) const
{
  std::vector<SuperStageDriver> drivers;
  const int start_index = expanded.startIndex();
  const int path_length = expanded.size();
  const int dcalc_ap = corner->dcalcAnalysisPtIndex(max_);
  std::unordered_set<const sta::Pin*> seen_roots;

  for (int i = start_index; i < path_length; ++i) {
    const sta::Path* path_node = expanded.path(i);
    sta::Vertex* path_vertex = path_node->vertex(sta_);
    const sta::Pin* path_pin = path_node->pin(sta_);
    if (i <= 0 || !path_vertex->isDriver(network_)
        || network_->isTopLevelPort(path_pin)) {
      continue;
    }

    const sta::Pin* root_pin
        = resizer_.rebuffer().findBufferTreeRootDriver(path_pin);
    if (!root_pin || network_->isTopLevelPort(root_pin)) {
      root_pin = path_pin;
    }

    // If this driver is an internal buffer whose root driver is already handled
    // (or will be handled at its own stage), skip it.
    if (root_pin != path_pin && seen_roots.contains(root_pin)) {
      continue;
    }

    const sta::TimingArc* prev_arc = path_node->prevArc(sta_);
    sta::Edge* prev_edge = path_node->prevEdge(sta_);
    if (!prev_arc || !prev_edge) {
      continue;
    }
    const sta::TimingArc* corner_arc = prev_arc->sceneArc(lib_ap);
    const sta::Delay root_load_delay
        = graph_->arcDelay(prev_edge, prev_arc, dcalc_ap)
          - (corner_arc ? corner_arc->intrinsicDelay() : sta::Delay(0.0f));

    int cur_drvr_idx = i;
    int tree_buf_stages = 0;
    sta::Delay total_wire_delay = 0.0;
    sta::Delay total_buf_gate_delay = 0.0;
    const sta::Pin* crit_sink_pin = nullptr;
    int64_t actual_routed_wl_dbu = 0;
    int64_t hop_manhattan_wl_dbu = 0;

    while (cur_drvr_idx + 1 < path_length) {
      const sta::Path* cur_drvr_node = expanded.path(cur_drvr_idx);
      const sta::Pin* cur_drvr_pin = cur_drvr_node->pin(sta_);
      const sta::Path* load_node = expanded.path(cur_drvr_idx + 1);
      sta::Edge* wire_edge = load_node->prevEdge(sta_);
      const sta::TimingArc* wire_arc = load_node->prevArc(sta_);
      if (wire_edge && wire_edge->isWire() && wire_arc) {
        total_wire_delay
            = total_wire_delay + graph_->arcDelay(wire_edge, wire_arc, dcalc_ap);
      }

      const sta::Pin* load_pin = load_node->pin(sta_);
      crit_sink_pin = load_pin;
      if (cur_drvr_pin && load_pin) {
        const odb::Point p_drvr = resizer_.dbNetwork()->location(cur_drvr_pin);
        const odb::Point p_load = resizer_.dbNetwork()->location(load_pin);
        const int hop_dist = odb::Point::manhattanDistance(p_drvr, p_load);
        hop_manhattan_wl_dbu += hop_dist;
        odb::dbNet* hop_db_net = resizer_.dbNetwork()->flatNet(cur_drvr_pin);
        if (hop_db_net && hop_db_net->getWire()) {
          actual_routed_wl_dbu += std::max<int64_t>(
              hop_dist, hop_db_net->getWire()->length());
        } else {
          actual_routed_wl_dbu += hop_dist;
        }
      }

      sta::Instance* load_inst
          = load_pin ? network_->instance(load_pin) : nullptr;
      if (load_inst && resizer_.rebuffer().isRemovableTreeBuffer(load_inst)
          && cur_drvr_idx + 2 < path_length) {
        const sta::Path* buf_drvr_node = expanded.path(cur_drvr_idx + 2);
        if (network_->instance(buf_drvr_node->pin(sta_)) == load_inst) {
          sta::Edge* buf_edge = buf_drvr_node->prevEdge(sta_);
          const sta::TimingArc* buf_arc = buf_drvr_node->prevArc(sta_);
          if (buf_edge && buf_arc) {
            total_buf_gate_delay
                = total_buf_gate_delay
                  + graph_->arcDelay(buf_edge, buf_arc, dcalc_ap);
          }
          tree_buf_stages++;
          cur_drvr_idx += 2;
          continue;
        }
      }
      break;
    }

    const sta::Delay super_stage_delay
        = root_load_delay + total_wire_delay + total_buf_gate_delay;

    // Compute Ideal Direct Placement Delay & Detour Excess Delay between
    // root_pin and crit_sink_pin.
    sta::Delay detour_excess_delay = 0.0f;
    float detour_ratio = 1.0f;
    if (crit_sink_pin) {
      const odb::Point root_loc = resizer_.dbNetwork()->location(path_pin);
      const odb::Point sink_loc = resizer_.dbNetwork()->location(crit_sink_pin);
      const int direct_dist_dbu
          = odb::Point::manhattanDistance(root_loc, sink_loc);
      const double direct_meters = resizer_.dbuToMeters(direct_dist_dbu);

      double wire_res = 0.0;
      double wire_cap = 0.0;
      estimate_parasitics_->wireSignalRC(corner, wire_res, wire_cap);

      sta::LibertyPort* root_port = network_->libertyPort(path_pin);
      sta::LibertyPort* sink_port = network_->libertyPort(crit_sink_pin);
      const float r_drvr = root_port ? root_port->driveResistance() : 0.0f;
      const float c_sink
          = sink_port ? resizer_.portCapacitance(sink_port, corner) : 0.0f;

      const double ideal_wire_cap = direct_meters * wire_cap;
      const double ideal_wire_res = direct_meters * wire_res;
      const sta::Delay ideal_direct_delay = static_cast<float>(
          r_drvr * (ideal_wire_cap + c_sink)
          + ideal_wire_res * (0.5 * ideal_wire_cap + c_sink));

      if (super_stage_delay > ideal_direct_delay) {
        detour_excess_delay = super_stage_delay - ideal_direct_delay;
      }
      const int64_t denom_dbu
          = std::max<int64_t>(direct_dist_dbu, resizer_.metersToDbu(1e-6));
      detour_ratio = static_cast<float>(std::max(actual_routed_wl_dbu,
                                                 hop_manhattan_wl_dbu))
                     / static_cast<float>(denom_dbu);
      if (resizer_.blockageAwareSteiner()) {
        resizer_.ensurePlacementBlockagesInitialized();
        if (resizer_.segmentCrossesPlacementBlockage(root_loc, sink_loc)) {
          detour_ratio = std::max(detour_ratio, 1.5f);
        }
      }
    }

    // Combine total super-stage delay and recoverable detour excess delay.
    const sta::Delay ranking_score = super_stage_delay + detour_excess_delay;

    seen_roots.insert(path_pin);
    drivers.push_back(SuperStageDriver{
        .path_index = i,
        .root_pin = path_pin,
        .root_load_delay = root_load_delay,
        .total_wire_delay = total_wire_delay,
        .total_buf_gate_delay = total_buf_gate_delay,
        .super_stage_delay = super_stage_delay,
        .detour_excess_delay = detour_excess_delay,
        .ranking_score = ranking_score,
        .detour_ratio = detour_ratio,
        .tree_buf_stages = tree_buf_stages,
    });

    // Skip over intermediate buffer stages on this path that belong to this
    // root driver's buffer tree.
    i = cur_drvr_idx;
  }

  std::ranges::sort(
      drivers, [](const SuperStageDriver& lhs, const SuperStageDriver& rhs) {
        return lhs.ranking_score > rhs.ranking_score
               || (lhs.ranking_score == rhs.ranking_score
                   && lhs.path_index > rhs.path_index);
      });

  return drivers;
}

void SetupWnsTreeRebufferPolicy::iterate()
{
  buildMainMoveSequence(/*log_sequence=*/true);
  repairSetupWnsTreeRebuffer(
      config_.setup_slack_margin, config_.max_passes, config_.verbose);
  committer_.printTrackerPhaseSummary(
      "WNS_TREE_REBUFFER Phase Summary",
      "WNS_TREE_REBUFFER Phase Endpoint Profiler",
      true);
  markRunComplete(true);
}

void SetupWnsTreeRebufferPolicy::repairSetupWnsTreeRebuffer(
    const float setup_slack_margin,
    const int max_passes_per_endpoint,
    const bool verbose)
{
  int& opto_iteration = setup_context_.iteration;
  const char phase_marker = phaseMarkerForIndex(setup_context_.phase_index);
  const utl::DebugScopedTimer timer(
      logger_,
      RSZ,
      "repair_setup",
      10,
      fmt::format("WNS_TREE_REBUFFER{} Phase Time: {{}}", phase_marker));

  committer_.capturePrePhaseSlack();
  rejected_pin_moves_current_endpoint_.clear();
  target_collector_->resetEndpointPasses();

  ViolatingEnds violating_ends = collectViolatingEndpoints(setup_slack_margin);
  if (violating_ends.empty()) {
    return;
  }

  std::map<const sta::Pin*, int> endpoint_pass_counts;
  std::set<const sta::Pin*> endpoints_exhausted_this_round;
  std::unordered_map<const sta::Pin*,
                     std::unordered_map<const sta::Pin*,
                                        std::unordered_set<MoveType>>>
      rejected_moves_per_endpoint;
  std::unordered_map<const sta::Pin*, std::unordered_set<MoveType>>
      global_exhausted_moves_per_root;
  std::unordered_set<const sta::Pin*> completed_tree_rebuffer_roots;
  int consecutive_exhausted_endpoints = 0;
  constexpr int kMaxConsecutiveExhaustedEndpoints = 64;

  printProgress(opto_iteration, false, phase_marker);

  while (true) {
    if (resizer_.overMaxArea()
        || reachedIterationLimit(opto_iteration, config_.max_iterations)
        || consecutive_exhausted_endpoints >= kMaxConsecutiveExhaustedEndpoints) {
      break;
    }

    sta::Slack worst_slack = 0.0;
    sta::Vertex* worst_endpoint = nullptr;
    sta_->worstSlack(max_, worst_slack, worst_endpoint);
    if (worst_endpoint == nullptr
        || sta::fuzzyGreaterEqual(worst_slack, setup_slack_margin)) {
      break;
    }

    // Pick the current endpoint: prefer the global worst_endpoint if not yet
    // exhausted in this round, otherwise pick the worst remaining endpoint in
    // violating_ends (refreshing violating_ends if the initial slice is done).
    sta::Vertex* current_endpoint = nullptr;
    if (!endpoints_exhausted_this_round.contains(worst_endpoint->pin())) {
      current_endpoint = worst_endpoint;
    } else {
      auto pick_from_list = [&]() -> sta::Vertex* {
        for (size_t idx = 0; idx < violating_ends.size(); ++idx) {
          sta::Vertex* cand = violating_ends[idx].first;
          if (cand == nullptr) {
            continue;
          }
          const sta::Pin* cand_pin = cand->pin();
          if (endpoints_exhausted_this_round.contains(cand_pin)) {
            continue;
          }
          const sta::Slack cand_slack = sta_->slack(cand, max_);
          if (sta::fuzzyLess(cand_slack, setup_slack_margin)) {
            return cand;
          }
          endpoints_exhausted_this_round.insert(cand_pin);
        }
        return nullptr;
      };
      current_endpoint = pick_from_list();
      if (current_endpoint == nullptr) {
        violating_ends = collectViolatingEndpoints(setup_slack_margin);
        current_endpoint = pick_from_list();
      }
    }

    if (current_endpoint == nullptr) {
      break;
    }

    const sta::Pin* endpoint_pin = current_endpoint->pin();
    if (endpoint_pass_counts[endpoint_pin] >= max_passes_per_endpoint) {
      endpoints_exhausted_this_round.insert(endpoint_pin);
      consecutive_exhausted_endpoints++;
      continue;
    }

    target_collector_->useWorstEndpoint(current_endpoint);
    committer_.setCurrentEndpoint(endpoint_pin);
    auto& rejected_for_endpoint = rejected_moves_per_endpoint[endpoint_pin];

    sta::Path* focus_path = sta_->vertexWorstSlackPath(current_endpoint, max_);
    if (focus_path == nullptr || focus_path->minMax(sta_) != max_) {
      endpoints_exhausted_this_round.insert(endpoint_pin);
      consecutive_exhausted_endpoints++;
      continue;
    }

    sta::PathExpanded expanded(focus_path, sta_);
    if (expanded.size() <= 1) {
      endpoints_exhausted_this_round.insert(endpoint_pin);
      consecutive_exhausted_endpoints++;
      continue;
    }

    const sta::Scene* corner = focus_path->scene(sta_);
    const int lib_ap = corner->libertyIndex(max_);
    const std::vector<SuperStageDriver> ranked_roots
        = rankSuperStageRootDrivers(expanded, corner, lib_ap);

    const sta::Slack prev_end_slack = sta_->slack(current_endpoint, max_);
    const sta::Slack prev_worst_slack = worst_slack;
    const float prev_tns_local = sta_->totalNegativeSlack(max_);

    bool attempted_any_stage = false;

    for (const SuperStageDriver& stage : ranked_roots) {
      const sta::Pin* root_pin = stage.root_pin;
      auto& rejected_set = rejected_for_endpoint[root_pin];
      if (completed_tree_rebuffer_roots.contains(root_pin)) {
        rejected_set.insert(MoveType::kTreeRebuffer);
      }
      if (const auto glob_it = global_exhausted_moves_per_root.find(root_pin);
          glob_it != global_exhausted_moves_per_root.end()) {
        rejected_set.insert(glob_it->second.begin(), glob_it->second.end());
      }
      const std::unordered_set<MoveType>* rejected_types = &rejected_set;

      // Skip if all configured move types have already been rejected for this
      // root driver on this endpoint.
      bool all_rejected = true;
      for (const MoveType move_type : move_sequence_) {
        if (!rejected_types->contains(move_type)) {
          all_rejected = false;
          break;
        }
      }
      if (all_rejected) {
        continue;
      }

      Target target;
      makePathDriverTarget(
          focus_path, expanded, stage.path_index, prev_end_slack, target);

      committer_.beginJournal();
      int changed = 0;
      std::optional<MoveType> accepted_type;
      const bool applied = tryRepairTarget(
          target, /*repairs_per_pass=*/1, changed, rejected_types, accepted_type);

      if (!applied || changed <= 0 || !accepted_type.has_value()) {
        committer_.commitJournal();
        for (const MoveType move_type : move_sequence_) {
          rejected_for_endpoint[root_pin].insert(move_type);
          global_exhausted_moves_per_root[root_pin].insert(move_type);
        }
        completed_tree_rebuffer_roots.insert(root_pin);
        continue;
      }

      // If tryRepairTarget fell through MoveType::kTreeRebuffer to a later move
      // (e.g. kSizeUp), kTreeRebuffer produced no improvement on this root_pin.
      if (*accepted_type != MoveType::kTreeRebuffer) {
        rejected_for_endpoint[root_pin].insert(MoveType::kTreeRebuffer);
        completed_tree_rebuffer_roots.insert(root_pin);
      }

      attempted_any_stage = true;
      opto_iteration++;
      if (verbose || opto_iteration % print_interval_ == 0) {
        printProgress(opto_iteration, false, phase_marker);
      }

      estimate_parasitics_->updateParasitics();
      sta_->findRequireds();

      const sta::Slack new_end_slack = sta_->slack(current_endpoint, max_);
      sta::Slack new_wns = 0.0;
      sta::Vertex* new_worst_vertex = nullptr;
      sta_->worstSlack(max_, new_wns, new_worst_vertex);
      const float new_tns = sta_->totalNegativeSlack(max_);

      const bool wns_improved = sta::fuzzyGreater(new_wns, prev_worst_slack);
      const bool wns_same = sta::fuzzyEqual(new_wns, prev_worst_slack);
      const bool endpoint_improved
          = sta::fuzzyGreater(new_end_slack, prev_end_slack);
      const bool tns_improved = sta::fuzzyGreater(new_tns, prev_tns_local);
      const bool better
          = wns_improved || (wns_same && (endpoint_improved || tns_improved));

      if (better) {
        committer_.commitJournal();
        endpoint_pass_counts[endpoint_pin]++;
        consecutive_exhausted_endpoints = 0;
        if (*accepted_type == MoveType::kTreeRebuffer) {
          // Once a root driver's buffer tree has been rebuilt, do not rebuild
          // the same root tree again; subsequent passes can size up the root.
          rejected_for_endpoint[root_pin].insert(MoveType::kTreeRebuffer);
          completed_tree_rebuffer_roots.insert(root_pin);
        }
        if (wns_improved) {
          endpoints_exhausted_this_round.clear();
          global_exhausted_moves_per_root.clear();
        }
        debugPrint(logger_,
                   RSZ,
                   "repair_setup",
                   1,
                   "WNS_TREE_REBUFFER{} ACCEPT {} ({}) on root {}: "
                   "score={} super_delay={} detour_excess={} detour_ratio={:.2f} "
                   "(load={}, wire={}, buf={}x{}) "
                   "end_slack {} -> {}, WNS {} -> {}",
                   phase_marker,
                   moveName(*accepted_type),
                   stage.path_index,
                   network_->pathName(root_pin),
                   delayAsString(stage.ranking_score, kDelayDigits, sta_),
                   delayAsString(stage.super_stage_delay, kDelayDigits, sta_),
                   delayAsString(stage.detour_excess_delay, kDelayDigits, sta_),
                   stage.detour_ratio,
                   delayAsString(stage.root_load_delay, kDelayDigits, sta_),
                   delayAsString(stage.total_wire_delay, kDelayDigits, sta_),
                   stage.tree_buf_stages,
                   delayAsString(stage.total_buf_gate_delay, kDelayDigits, sta_),
                   delayAsString(prev_end_slack, kDelayDigits, sta_),
                   delayAsString(new_end_slack, kDelayDigits, sta_),
                   delayAsString(prev_worst_slack, kDelayDigits, sta_),
                   delayAsString(new_wns, kDelayDigits, sta_));
      } else {
        committer_.restoreJournal();
        rejected_for_endpoint[root_pin].insert(*accepted_type);
        global_exhausted_moves_per_root[root_pin].insert(*accepted_type);
        if (*accepted_type == MoveType::kTreeRebuffer) {
          completed_tree_rebuffer_roots.insert(root_pin);
        }
      }
      // Always break after findRequireds() / restoreJournal() because
      // focus_path and expanded are invalidated by STA timing updates.
      break;
    }

    if (!attempted_any_stage) {
      endpoints_exhausted_this_round.insert(endpoint_pin);
      consecutive_exhausted_endpoints++;
    }
  }

  printProgress(opto_iteration, true, phase_marker);
}

}  // namespace rsz
