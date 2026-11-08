// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <utility>
#include <vector>

#include "MoveCommitter.hh"
#include "OptimizerTypes.hh"
#include "RepairSetupContext.hh"
#include "SetupLegacyBase.hh"
#include "sta/Delay.hh"
#include "sta/Path.hh"

namespace rsz {

struct SuperStageDriver
{
  int path_index{-1};
  const sta::Pin* root_pin{nullptr};
  sta::Delay root_load_delay{0.0};
  sta::Delay total_wire_delay{0.0};
  sta::Delay total_buf_gate_delay{0.0};
  sta::Delay super_stage_delay{0.0};
  sta::Delay detour_excess_delay{0.0};
  sta::Delay ranking_score{0.0};
  float detour_ratio{1.0f};
  int tree_buf_stages{0};
};

// WNS_TREE_REBUFFER phase: collapses multi-stage buffer chains along critical
// paths back to their root logic drivers, ranks root drivers by Super-Stage
// Total Delay (root load delay + total wire delay + intermediate buffer gate
// delays), and re-synthesizes each buffer tree via Rebuffer::rebufferTreePin.
class SetupWnsTreeRebufferPolicy : public SetupLegacyBase
{
 public:
  SetupWnsTreeRebufferPolicy(Resizer& resizer,
                             MoveCommitter& committer,
                             RepairSetupContext& setup_context,
                             const OptimizerRunConfig& config)
      : SetupLegacyBase(resizer, committer, setup_context, config)
  {
  }

  const char* name() const override { return "SetupWnsTreeRebufferPolicy"; }
  void iterate() override;

 protected:
  void buildMainMoveSequence(bool log_sequence) override;

 private:
  std::vector<SuperStageDriver> rankSuperStageRootDrivers(
      sta::PathExpanded& expanded,
      const sta::Scene* corner,
      int lib_ap) const;

  void repairSetupWnsTreeRebuffer(float setup_slack_margin,
                                  int max_passes_per_endpoint,
                                  bool verbose);
};

}  // namespace rsz
