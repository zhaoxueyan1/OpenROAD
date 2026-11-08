// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <memory>
#include <vector>

#include "MoveCandidate.hh"
#include "MoveGenerator.hh"
#include "OptimizerTypes.hh"
#include "rsz/Resizer.hh"

namespace rsz {

class TreeRebufferGenerator : public MoveGenerator
{
 public:
  explicit TreeRebufferGenerator(const GeneratorContext& context);

  MoveType type() const override { return MoveType::kTreeRebuffer; }
  bool isApplicable(const Target& target) const override;
  std::vector<std::unique_ptr<MoveCandidate>> generate(
      const Target& target) override;
};

}  // namespace rsz
