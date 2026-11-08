// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "TreeRebufferCandidate.hh"

#include <vector>

#include "MoveCandidate.hh"
#include "OptimizerTypes.hh"
#include "Rebuffer.hh"
#include "rsz/Resizer.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

TreeRebufferCandidate::TreeRebufferCandidate(Resizer& resizer,
                                             const Target& target,
                                             sta::Pin* driver_pin)
    : MoveCandidate(resizer, target), driver_pin_(driver_pin)
{
}

MoveResult TreeRebufferCandidate::apply()
{
  const int rebuffer_count = resizer_.rebuffer().rebufferTreePin(driver_pin_);
  if (rebuffer_count <= 0) {
    debugPrint(resizer_.logger(),
               RSZ,
               "buffer_move",
               2,
               "REJECT TreeRebufferMove {}: No tree improvement",
               resizer_.network()->pathName(driver_pin_));
    return rejectedMove();
  }

  sta::Instance* drvr_inst = resizer_.network()->instance(driver_pin_);
  debugPrint(resizer_.logger(),
             RSZ,
             "buffer_move",
             1,
             "ACCEPT TreeRebufferMove {}: Changed {} buffers in tree",
             resizer_.network()->pathName(driver_pin_),
             rebuffer_count);
  return {
      .accepted = true,
      .type = MoveType::kTreeRebuffer,
      .move_count = rebuffer_count,
      .touched_instances = {drvr_inst},
  };
}

}  // namespace rsz
