// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include "MoveCandidate.hh"
#include "OptimizerTypes.hh"
#include "rsz/Resizer.hh"

namespace sta {
class Pin;
}  // namespace sta

namespace rsz {

// Candidate that invokes Rebuffer::rebufferTreePin on a root driver pin,
// merging and re-synthesizing the entire downstream buffer tree.
class TreeRebufferCandidate : public MoveCandidate
{
 public:
  TreeRebufferCandidate(Resizer& resizer,
                        const Target& target,
                        sta::Pin* driver_pin);

  MoveResult apply() override;
  MoveType type() const override { return MoveType::kTreeRebuffer; }

 private:
  sta::Pin* driver_pin_{nullptr};
};

}  // namespace rsz
