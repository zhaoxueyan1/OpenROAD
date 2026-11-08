// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "TreeRebufferGenerator.hh"

#include <memory>
#include <vector>

#include "MoveCandidate.hh"
#include "MoveGenerator.hh"
#include "OptimizerTypes.hh"
#include "TreeRebufferCandidate.hh"
#include "rsz/Resizer.hh"

namespace rsz {

namespace {

constexpr int kTreeRebufferMaxFanout = 64;

}  // namespace

TreeRebufferGenerator::TreeRebufferGenerator(const GeneratorContext& context)
    : MoveGenerator(context)
{
}

bool TreeRebufferGenerator::isApplicable(const Target& target) const
{
  return MoveGenerator::isApplicable(target) && target.fanout > 0
         && target.fanout < kTreeRebufferMaxFanout
         && resizer_.okToBufferNet(target.driver_pin);
}

std::vector<std::unique_ptr<MoveCandidate>> TreeRebufferGenerator::generate(
    const Target& target)
{
  std::vector<std::unique_ptr<MoveCandidate>> candidates;
  candidates.push_back(std::make_unique<TreeRebufferCandidate>(
      resizer_, target, target.driver_pin));
  return candidates;
}

}  // namespace rsz
