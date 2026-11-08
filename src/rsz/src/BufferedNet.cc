// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "BufferedNet.hh"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "est/EstimateParasitics.h"
#include "est/SteinerTree.h"
#include "grt/GRoute.h"
#include "odb/db.h"
#include "odb/dbWireCodec.h"
#include "odb/geom.h"
#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/MinMax.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/Scene.hh"
#include "sta/Transition.hh"
#include "stt/SteinerTreeBuilder.h"
// Use spdlog fmt::format until c++20 that supports std::format.
#include "spdlog/fmt/fmt.h"
#include "sta/Fuzzy.hh"
#include "sta/Liberty.hh"
#include "sta/Sdc.hh"
#include "sta/Units.hh"
#include "utl/Logger.h"

// Resizer::makeBufferedNetGroute
#include "db_sta/dbNetwork.hh"
#include "grt/GlobalRouter.h"
#include "grt/PinGridLocation.h"
#include "grt/RoutePt.h"
#include "sta/Hash.hh"

namespace sta {
class Port;
class Pin;
}  // namespace sta

namespace rsz {

using sta::MinMax;
using sta::Pin;
using sta::Port;
using sta::Scene;
using std::make_shared;
using std::max;
using std::min;
using utl::RSZ;

static const char* to_string(BufferedNetType type);

////////////////////////////////////////////////////////////////

const FixedDelay FixedDelay::INF = FixedDelay(100.0f, nullptr);
const FixedDelay FixedDelay::ZERO = FixedDelay(0.0f, nullptr);

FixedDelay::FixedDelay() : value_fs_(0)
{
}

FixedDelay::FixedDelay(sta::Delay float_value, Resizer* resizer)
{
  if (resizer) {
    if (float_value > (FixedDelay::INF).toSeconds()) {
      float_value = (FixedDelay::INF).toSeconds();
    } else if (float_value < (-FixedDelay::INF).toSeconds()) {
      float_value = (-FixedDelay::INF).toSeconds();
    }
  }

  value_fs_ = float_value * second_;
}

////////////////////////////////////////////////////////////////

// load
BufferedNet::BufferedNet(const BufferedNetType type,
                         const odb::Point& location,
                         const sta::Pin* load_pin,
                         const sta::Scene* corner,
                         const Resizer* resizer)
{
  if (type != BufferedNetType::load) {
    resizer->logger()->critical(
        RSZ, 78, "incorrect BufferedNet type {}", rsz::to_string(type));
  }
  type_ = BufferedNetType::load;
  location_ = location;
  load_pin_ = load_pin;
  corner_ = corner;

  sta::Network* network = resizer->network();
  sta::LibertyPort* load_port = network->libertyPort(load_pin);
  if (load_port) {
    cap_ = resizer->portCapacitance(load_port, corner);
    fanout_ = resizer->portFanoutLoad(load_port);
    max_load_slew_ = resizer->maxInputSlew(load_port, corner);
  } else if (network->isTopLevelPort(load_pin)) {
    Port* port = network->port(load_pin);
    for (auto rf : sta::RiseFall::range()) {
      float pin_cap, wire_cap;
      int fanout;
      bool has_pin_cap, has_wire_cap, has_fanout;
      corner->sdc()->portExtCap(port,
                                rf,
                                MinMax::max(),
                                pin_cap,
                                has_pin_cap,
                                wire_cap,
                                has_wire_cap,
                                fanout,
                                has_fanout);
      if (has_pin_cap) {
        cap_ = std::max(cap_, pin_cap);
      }
    }
  }
}

// junc
BufferedNet::BufferedNet(const BufferedNetType type,
                         const odb::Point& location,
                         const BufferedNetPtr& ref,
                         const BufferedNetPtr& ref2,
                         const Resizer* resizer)
{
  if (type != BufferedNetType::junction) {
    resizer->logger()->critical(
        RSZ, 79, "incorrect BufferedNet type {}", rsz::to_string(type));
  }
  type_ = BufferedNetType::junction;
  location_ = location;
  layer_ = null_layer;
  ref_ = ref;
  ref2_ = ref2;
  corner_ = ref->corner();

  cap_ = ref->cap() + ref2->cap();
  fanout_ = ref->fanout() + ref2->fanout();
  max_load_slew_ = min(ref->maxLoadSlew(), ref2->maxLoadSlew());

  area_ = ref->area() + ref2->area();
}

// wire
BufferedNet::BufferedNet(const BufferedNetType type,
                         const odb::Point& location,
                         const int layer,
                         const BufferedNetPtr& ref,
                         const sta::Scene* corner,
                         const Resizer* resizer,
                         const est::EstimateParasitics* estimate_parasitics)
{
  if (type != BufferedNetType::wire) {
    resizer->logger()->critical(
        RSZ, 80, "incorrect BufferedNet type {}", rsz::to_string(type));
  }
  type_ = BufferedNetType::wire;
  location_ = location;
  layer_ = layer;
  ref_ = ref;
  corner_ = corner;

  double wire_res, wire_cap;
  wireRC(corner, resizer, estimate_parasitics, wire_res, wire_cap);
  cap_ = ref->cap() + resizer->dbuToMeters(length()) * wire_cap;
  fanout_ = ref->fanout();
  max_load_slew_ = ref->maxLoadSlew();

  area_ = ref->area();
}

// via
BufferedNet::BufferedNet(const BufferedNetType type,
                         const odb::Point& location,
                         const int layer,
                         const int ref_layer,
                         const BufferedNetPtr& ref,
                         const Scene* corner,
                         const Resizer* resizer)
{
  if (type != BufferedNetType::via) {
    resizer->logger()->critical(
        RSZ, 87, "incorrect BufferedNet type {}", rsz::to_string(type));
  }
  type_ = BufferedNetType::via;
  location_ = location;
  layer_ = layer;
  ref_layer_ = ref_layer;
  ref_ = ref;
  corner_ = corner;

  cap_ = ref->cap();
  fanout_ = ref->fanout();
  max_load_slew_ = ref->maxLoadSlew();

  area_ = ref->area();
}

// buffer
BufferedNet::BufferedNet(const BufferedNetType type,
                         const odb::Point& location,
                         sta::LibertyCell* buffer_cell,
                         const BufferedNetPtr& ref,
                         const sta::Scene* corner,
                         const Resizer* resizer,
                         const est::EstimateParasitics* estimate_parasitics)
{
  if (type != BufferedNetType::buffer) {
    resizer->logger()->critical(
        RSZ, 81, "incorrect BufferedNet type {}", rsz::to_string(type));
  }
  type_ = BufferedNetType::buffer;
  location_ = location;
  buffer_cell_ = buffer_cell;
  layer_ = null_layer;
  ref_ = ref;
  corner_ = corner;

  sta::LibertyPort *input, *output;
  buffer_cell->bufferPorts(input, output);
  cap_ = resizer->portCapacitance(input, corner);
  fanout_ = resizer->portFanoutLoad(input);
  max_load_slew_ = resizer->maxInputSlew(input, corner);

  area_ = ref->area() + buffer_cell_->area();
}

void BufferedNet::reportTree(const Resizer* resizer) const
{
  reportTree(0, resizer);
}

void BufferedNet::reportTree(const int level, const Resizer* resizer) const
{
  resizer->logger()->report("{:{}s}{}", "", level, to_string(resizer));
  switch (type_) {
    case BufferedNetType::load:
      break;
    case BufferedNetType::buffer:
    case BufferedNetType::wire:
    case BufferedNetType::via:
      ref_->reportTree(level + 1, resizer);
      break;
    case BufferedNetType::junction:
      ref_->reportTree(level + 1, resizer);
      ref2_->reportTree(level + 1, resizer);
      break;
  }
}

std::string BufferedNet::to_string(const Resizer* resizer) const
{
  sta::Network* sdc_network = resizer->sdcNetwork();
  sta::Units* units = resizer->units();
  sta::Unit* dist_unit = units->distanceUnit();
  const std::string x
      = dist_unit->asString(resizer->dbuToMeters(location_.x()), 2);
  const std::string y
      = dist_unit->asString(resizer->dbuToMeters(location_.y()), 2);
  const std::string cap = units->capacitanceUnit()->asString(cap_);

  switch (type_) {
    case BufferedNetType::load:
      // {:{}s} format indents level spaces.
      return fmt::format("load {} ({}, {}) cap {} slack {} load sl {}",
                         sdc_network->pathName(load_pin_),
                         x,
                         y,
                         cap,
                         delayAsString(slack().toSeconds(), resizer),
                         delayAsString(maxLoadSlew(), resizer));
    case BufferedNetType::wire:
      return fmt::format("wire ({}, {}) cap {} slack {} buffers {} load sl {}",
                         x,
                         y,
                         cap,
                         delayAsString(slack().toSeconds(), resizer),
                         bufferCount(),
                         delayAsString(maxLoadSlew(), resizer));
    case BufferedNetType::via:
      return fmt::format(
          "via ({}, {}) layer {} -> {}", x, y, layer_, ref_layer_);
    case BufferedNetType::buffer:
      return fmt::format(
          "buffer ({}, {}) {} cap {} slack {} buffers {} load sl {}",
          x,
          y,
          buffer_cell_->name(),
          cap,
          delayAsString(slack().toSeconds(), resizer),
          bufferCount(),
          delayAsString(maxLoadSlew(), resizer));
    case BufferedNetType::junction:
      return fmt::format(
          "junction ({}, {}) cap {} slack {} buffers {} load sl {}",
          x,
          y,
          cap,
          delayAsString(slack().toSeconds(), resizer),
          bufferCount(),
          delayAsString(maxLoadSlew(), resizer));
  }
  // suppress gcc warning
  return "";
}

int BufferedNet::length() const
{
  return odb::Point::manhattanDistance(location_, ref_->location());
}

void BufferedNet::setCapacitance(float cap)
{
  cap_ = cap;
}

void BufferedNet::setFanout(float fanout)
{
  fanout_ = fanout;
}

void BufferedNet::setMaxLoadSlew(float max_slew)
{
  max_load_slew_ = max_slew;
}

void BufferedNet::setSlackTransition(const sta::RiseFallBoth* transitions)
{
  slack_transitions_ = transitions;
}

void BufferedNet::setSlack(FixedDelay slack)
{
  slack_ = slack;
}

void BufferedNet::setDelay(FixedDelay delay)
{
  delay_ = delay;
}

void BufferedNet::setArrivalDelay(FixedDelay delay)
{
  arrival_delay_ = delay;
}

int BufferedNet::bufferCount() const
{
  switch (type_) {
    case BufferedNetType::buffer:
      return ref_->bufferCount() + 1;
    case BufferedNetType::wire:
    case BufferedNetType::via:
      return ref_->bufferCount();
    case BufferedNetType::junction:
      return ref_->bufferCount() + ref2_->bufferCount();
    case BufferedNetType::load:
      return 0;
  }
  return 0;
}

int BufferedNet::loadCount() const
{
  switch (type_) {
    case BufferedNetType::buffer:
      return ref_->loadCount();
    case BufferedNetType::wire:
    case BufferedNetType::via:
      return ref_->loadCount();
    case BufferedNetType::junction:
      return ref_->loadCount() + ref2_->loadCount();
    case BufferedNetType::load:
      return 1;
  }
  return 0;
}

int BufferedNet::maxLoadWireLength() const
{
  switch (type_) {
    case BufferedNetType::wire:
      return length() + ref_->maxLoadWireLength();
    case BufferedNetType::via:
      return ref_->maxLoadWireLength();
    case BufferedNetType::junction:
      return max(ref_->maxLoadWireLength(), ref2_->maxLoadWireLength());
    case BufferedNetType::load:
      return 0;
    case BufferedNetType::buffer:
      return 0;
  }
  return 0;
}

void BufferedNet::wireRC(const sta::Scene* corner,
                         const Resizer* resizer,
                         const est::EstimateParasitics* estimate_parasitics,
                         // Return values.
                         double& res,
                         double& cap)
{
  if (type_ != BufferedNetType::wire) {
    resizer->logger()->critical(RSZ, 82, "wireRC called for non-wire");
  }
  if (layer_ == BufferedNet::null_layer) {
    if (length() == 0) {
      res = 0;
      cap = 0;
    } else {
      const double dx
          = resizer->dbuToMeters(std::abs(location_.x() - ref_->location().x()))
            / resizer->dbuToMeters(length());
      const double dy
          = resizer->dbuToMeters(std::abs(location_.y() - ref_->location().y()))
            / resizer->dbuToMeters(length());
      res = dx * estimate_parasitics->wireSignalHResistance(corner)
            + dy * estimate_parasitics->wireSignalVResistance(corner);
      cap = dx * estimate_parasitics->wireSignalHCapacitance(corner)
            + dy * estimate_parasitics->wireSignalVCapacitance(corner);
    }
  } else {
    odb::dbTech* tech = resizer->db_->getTech();
    estimate_parasitics->layerRC(
        tech->findRoutingLayer(layer_), corner, res, cap);
  }
  if (estimate_parasitics && estimate_parasitics->hasSpefCalibration()) {
    const auto& rc_scale = resizer->activeNetRcScale();
    const bool is_steiner_seg = (layer_ == BufferedNet::null_layer);
    const double cap_scale
        = rc_scale.valid ? (is_steiner_seg ? rc_scale.cap_scale
                                           : rc_scale.unit_cap_scale)
                         : estimate_parasitics->defaultUnitCapScale();
    const double res_scale
        = rc_scale.valid ? (is_steiner_seg ? rc_scale.res_scale
                                           : rc_scale.unit_res_scale)
                         : estimate_parasitics->defaultUnitResScale();
    cap *= cap_scale;
    res *= res_scale;
  }
}

double BufferedNet::viaResistance(
    const Scene* corner,
    const Resizer* resizer,
    const est::EstimateParasitics* estimate_parasitics)
{
  if (type_ != BufferedNetType::via) {
    resizer->logger()->critical(RSZ, 92, "viaResistance called for non-via");
  }
  double total_resist = 0;
  odb::dbTech* tech = resizer->db_->getTech();
  for (int i = std::min(layer_, ref_layer_); i < std::max(layer_, ref_layer_);
       i++) {
    odb::dbTechLayer* tech_layer = tech->findRoutingLayer(i);
    odb::dbTechLayer* cut_layer
        = tech_layer ? tech_layer->getUpperLayer() : nullptr;
    if (cut_layer) {
      double res = 0.0;
      double cap = 0.0;
      estimate_parasitics->layerRC(cut_layer, corner, res, cap);
      if (res == 0.0) {
        res = cut_layer->getResistance();
      }
      total_resist += res;
    } else {
      resizer->logger()->error(
          RSZ,
          93,
          "cannot find resistance of via layer above routing layer {}",
          i);
    }
  }
  if (estimate_parasitics && estimate_parasitics->hasSpefCalibration()) {
    const auto& rc_scale = resizer->activeNetRcScale();
    const double res_scale
        = rc_scale.valid ? rc_scale.unit_res_scale
                         : estimate_parasitics->defaultUnitResScale();
    total_resist *= res_scale;
  }
  return total_resist;
}

static const char* to_string(const BufferedNetType type)
{
  switch (type) {
    case BufferedNetType::load:
      return "load";
    case BufferedNetType::junction:
      return "junction";
    case BufferedNetType::wire:
      return "wire";
    case BufferedNetType::via:
      return "via";
    case BufferedNetType::buffer:
      return "buffer";
  }
  return "??";
}

////////////////////////////////////////////////////////////////

BufferedNetPtr Resizer::makeBufferedNet(const sta::Pin* drvr_pin,
                                        const sta::Scene* corner)
{
  odb::dbNet* db_net = db_network_->findFlatDbNet(drvr_pin);
  setActiveNetRcScale(db_net ? db_network_->dbToSta(db_net) : nullptr);
  if (db_net != nullptr && db_net->getWire() != nullptr
      && !db_net->isSpecial()) {
    if (BufferedNetPtr bnet = makeBufferedNetDroute(drvr_pin, corner)) {
      return bnet;
    }
  }
  switch (estimate_parasitics_->getParasiticsSrc()) {
    case est::ParasiticsSrc::kPlacement:
      return makeBufferedNetSteiner(drvr_pin, corner);
    case est::ParasiticsSrc::kGlobalRouting:
    case est::ParasiticsSrc::kDetailedRouting:
      return makeBufferedNetGroute(drvr_pin, corner);
    case est::ParasiticsSrc::kNone:
      return nullptr;
  }
  return nullptr;
}

using SteinerPtAdjacents = std::vector<std::vector<SteinerPt>>;
using SteinerPtPinVisited
    = std::unordered_set<odb::Point, est::PointHash, est::PointEqual>;

static BufferedNetPtr makeBufferedNetFromTree(
    const est::SteinerTree* tree,
    const est::SteinerPt from,
    const est::SteinerPt to,
    const SteinerPtAdjacents& adjacents,
    const int level,
    SteinerPtPinVisited& pins_visited,
    const sta::Scene* corner,
    const Resizer* resizer,
    const est::EstimateParasitics* estimate_parasitics,
    utl::Logger* logger,
    const sta::Network* network)
{
  BufferedNetPtr bnet = nullptr;
  const sta::PinSeq* pins = tree->pins(to);
  const odb::Point to_loc = tree->location(to);
  // If there is more than one node at a location we don't want to
  // add the pins repeatedly.  The first node wins and the rest are skipped.
  if (pins && !pins_visited.contains(to_loc)) {
    pins_visited.insert(to_loc);
    for (const sta::Pin* pin : *pins) {
      if (network->isLoad(pin)) {
        BufferedNetPtr bnet1 = make_shared<BufferedNet>(
            BufferedNetType::load, tree->location(to), pin, corner, resizer);
        if (bnet1) {
          debugPrint(logger,
                     RSZ,
                     "make_buffered_net",
                     4,
                     "{:{}s}{}",
                     "",
                     level,
                     bnet1->to_string(resizer));
          if (bnet) {
            bnet = make_shared<BufferedNet>(BufferedNetType::junction,
                                            tree->location(to),
                                            bnet,
                                            bnet1,
                                            resizer);
          } else {
            bnet = std::move(bnet1);
          }
        }
      }
    }
  }
  // Steiner pt.
  for (const int adj : adjacents[to]) {
    if (adj != from) {
      BufferedNetPtr bnet1 = makeBufferedNetFromTree(tree,
                                                     to,
                                                     adj,
                                                     adjacents,
                                                     level + 1,
                                                     pins_visited,
                                                     corner,
                                                     resizer,
                                                     estimate_parasitics,
                                                     logger,
                                                     network);
      if (bnet1) {
        if (bnet) {
          bnet = make_shared<BufferedNet>(BufferedNetType::junction,
                                          tree->location(to),
                                          bnet,
                                          bnet1,
                                          resizer);
        } else {
          bnet = std::move(bnet1);
        }
      }
    }
  }
  if (bnet && from != est::SteinerTree::kNullPt
      && tree->location(to) != tree->location(from)) {
    bnet = make_shared<BufferedNet>(BufferedNetType::wire,
                                    tree->location(from),
                                    BufferedNet::null_layer,
                                    bnet,
                                    corner,
                                    resizer,
                                    estimate_parasitics);
  }
  return bnet;
}

// Make BufferedNet from steiner tree.
BufferedNetPtr Resizer::makeBufferedNetSteiner(const sta::Pin* drvr_pin,
                                               const sta::Scene* corner)
{
  BufferedNetPtr bnet;
  est::SteinerTree* tree = estimate_parasitics_->makeSteinerTree(drvr_pin);
  if (tree) {
    const SteinerPt drvr_pt = tree->drvrPt();
    if (drvr_pt != est::SteinerTree::kNullPt) {
      const int branch_count = tree->branchCount();
      SteinerPtAdjacents adjacents(branch_count);
      for (int i = 0; i < branch_count; i++) {
        const stt::Branch& branch_pt = tree->branch(i);
        const SteinerPt j = branch_pt.n;
        if (j != i) {
          adjacents[i].push_back(j);
          adjacents[j].push_back(i);
        }
      }
      SteinerPtPinVisited pins_visited;
      bnet = rsz::makeBufferedNetFromTree(tree,
                                          est::SteinerTree::kNullPt,
                                          drvr_pt,
                                          adjacents,
                                          0,
                                          pins_visited,
                                          corner,
                                          this,
                                          estimate_parasitics_,
                                          logger_,
                                          network_);
    }
    delete tree;
  }
  if (bnet && blockage_aware_steiner_) {
    bnet = rerouteBufferedNetAroundBlockages(bnet, corner);
  }
  return bnet;
}

// helper for makeBufferedNetSteinerOverBnets
static BufferedNetPtr makeBufferedNetFromTree2(
    const est::SteinerTree* tree,
    const est::SteinerPt from,
    const est::SteinerPt to,
    const SteinerPtAdjacents& adjacents,
    const int level,
    SteinerPtPinVisited& pins_visited,
    const sta::Scene* corner,
    const Resizer* resizer,
    const est::EstimateParasitics* estimate_parasitics,
    utl::Logger* logger,
    const sta::Network* network,
    std::map<odb::Point, std::vector<BufferedNetPtr>>& sink_map)
{
  BufferedNetPtr bnet = nullptr;
  const odb::Point to_loc = tree->location(to);
  // If there is more than one node at a location we don't want to
  // add the pins repeatedly.  The first node wins and the rest are skipped.
  if (sink_map.contains(to_loc) && !pins_visited.contains(to_loc)) {
    pins_visited.insert(to_loc);
    for (BufferedNetPtr sink : sink_map[to_loc]) {
      if (bnet) {
        bnet = make_shared<BufferedNet>(
            BufferedNetType::junction, to_loc, bnet, sink, resizer);
      } else {
        bnet = std::move(sink);
      }
    }
  }
  // Steiner pt.
  for (int adj : adjacents[to]) {
    if (adj != from) {
      BufferedNetPtr bnet1 = makeBufferedNetFromTree2(tree,
                                                      to,
                                                      adj,
                                                      adjacents,
                                                      level + 1,
                                                      pins_visited,
                                                      corner,
                                                      resizer,
                                                      estimate_parasitics,
                                                      logger,
                                                      network,
                                                      sink_map);
      if (bnet1) {
        if (bnet) {
          bnet = make_shared<BufferedNet>(BufferedNetType::junction,
                                          tree->location(to),
                                          bnet,
                                          bnet1,
                                          resizer);
        } else {
          bnet = std::move(bnet1);
        }
      }
    }
  }
  if (bnet && from != est::SteinerTree::kNullPt
      && tree->location(to) != tree->location(from)) {
    bnet = make_shared<BufferedNet>(BufferedNetType::wire,
                                    tree->location(from),
                                    BufferedNet::null_layer,
                                    bnet,
                                    corner,
                                    resizer,
                                    estimate_parasitics);
  }
  return bnet;
}

////////////////////////////////////////////////////////////////

// Make BufferedNet from Steiner tree. This is similar to
// makeBufferedNetSteiner but supports sinks of type BufferedNetPtr
BufferedNetPtr Resizer::makeBufferedNetSteinerOverBnets(
    odb::Point root,
    const std::vector<BufferedNetPtr>& sinks,
    const sta::Scene* corner)
{
  BufferedNetPtr bnet = nullptr;
  std::vector<odb::Point> sink_points;
  std::map<odb::Point, std::vector<BufferedNetPtr>> sink_map;
  for (const auto& sink : sinks) {
    sink_points.push_back(sink->location());
    sink_map[sink->location()].push_back(sink);
  }
  est::SteinerTree* tree
      = estimate_parasitics_->makeSteinerTree(root, sink_points);
  if (tree) {
    SteinerPt drvr_pt = tree->drvrPt();
    if (drvr_pt != est::SteinerTree::kNullPt) {
      int branch_count = tree->branchCount();
      SteinerPtAdjacents adjacents(branch_count);
      for (int i = 0; i < branch_count; i++) {
        stt::Branch& branch_pt = tree->branch(i);
        SteinerPt j = branch_pt.n;
        if (j != i) {
          adjacents[i].push_back(j);
          adjacents[j].push_back(i);
        }
      }
      SteinerPtPinVisited pins_visited;
      bnet = rsz::makeBufferedNetFromTree2(tree,
                                           est::SteinerTree::kNullPt,
                                           drvr_pt,
                                           adjacents,
                                           0,
                                           pins_visited,
                                           corner,
                                           this,
                                           estimate_parasitics_,
                                           logger_,
                                           network_,
                                           sink_map);
    }
    delete tree;
  }
  if (bnet && blockage_aware_steiner_) {
    ensurePlacementBlockagesInitialized();
    if (treeTouchesPlacementBlockage(bnet)) {
      BufferedNetPtr aware_bnet
          = makeBufferedNetBlockageAwareSteinerOverBnets(root, sinks, corner);
      if (aware_bnet) {
        return aware_bnet;
      }
      bnet = rerouteBufferedNetAroundBlockages(bnet, corner);
    }
  }
  return bnet;
}

////////////////////////////////////////////////////////////////

BufferedNetPtr Resizer::stitchTrees(const BufferedNetPtr& outer_tree,
                                    Pin* stitching_load,
                                    const BufferedNetPtr& inner_tree)
{
  using BnetType = BufferedNetType;
  using BnetPtr = BufferedNetPtr;
  return visitTree(
      [this, stitching_load, inner_tree](
          auto& recurse, int level, const BnetPtr& node) -> BufferedNetPtr {
        if (!node) {
          return nullptr;
        }
        switch (node->type()) {
          case BnetType::via: {
            BnetPtr new_ref = recurse(node->ref());
            if (!new_ref) {
              return nullptr;
            }
            if (new_ref == node->ref()) {
              // Optimization: do not create a new node if it would be
              // equivalent to the existing one; analogously below for wire and
              // junction
              return node;
            }
            return std::make_shared<BufferedNet>(BnetType::via,
                                                 node->location(),
                                                 node->layer(),
                                                 node->refLayer(),
                                                 new_ref,
                                                 node->corner(),
                                                 this);
          }
          case BnetType::wire: {
            BnetPtr new_ref = recurse(node->ref());
            if (!new_ref) {
              return nullptr;
            }
            if (new_ref == node->ref()) {
              return node;
            }
            return std::make_shared<BufferedNet>(
                BnetType::wire,
                node->location(),
                node->layer(),
                new_ref,  // Fixed: was recurse(node->ref())
                node->corner(),
                this,
                estimate_parasitics_);
          }
          case BnetType::junction: {
            BnetPtr new_ref = recurse(node->ref());
            BnetPtr new_ref2 = recurse(node->ref2());
            if (!new_ref || !new_ref2) {
              return nullptr;
            }
            if (new_ref == node->ref() && new_ref2 == node->ref2()) {
              return node;
            }
            return std::make_shared<BufferedNet>(
                BnetType::junction, node->location(), new_ref, new_ref2, this);
          }
          case BnetType::load: {
            if (node->loadPin() == stitching_load) {
              // This is the buffer input pin we want to replace
              // Connect buffer input location to buffer output location with
              // wire
              odb::Point buffer_in_loc = node->location();
              int buffer_in_layer = node->layer();
              odb::Point buffer_out_loc = inner_tree->location();
              int buffer_out_layer = inner_tree->layer();

              // Check if we need to add connection
              if (buffer_in_loc == buffer_out_loc
                  && buffer_in_layer == buffer_out_layer) {
                // Same location and layer - just splice in inner_tree
                return inner_tree;
              }
              if (buffer_in_layer == buffer_out_layer) {
                // Same layer but different location - add wire segment
                debugPrint(logger_,
                           RSZ,
                           "buffer_removal",
                           2,
                           "Adding wire from buffer input ({}, {}) to output "
                           "({}, {}) on layer {}",
                           buffer_in_loc.getX(),
                           buffer_in_loc.getY(),
                           buffer_out_loc.getX(),
                           buffer_out_loc.getY(),
                           buffer_in_layer);

                return std::make_shared<BufferedNet>(BnetType::wire,
                                                     buffer_in_loc,
                                                     buffer_in_layer,
                                                     inner_tree,
                                                     node->corner(),
                                                     this,
                                                     estimate_parasitics_);
              }
              // Different layers - need via(s) + wire
              debugPrint(logger_,
                         RSZ,
                         "buffer_removal",
                         2,
                         "Adding via+wire from buffer input ({}, {}, layer "
                         "{}) to output ({}, {}, layer {})",
                         buffer_in_loc.getX(),
                         buffer_in_loc.getY(),
                         buffer_in_layer,
                         buffer_out_loc.getX(),
                         buffer_out_loc.getY(),
                         buffer_out_layer);

              // Validate layer range
              if (buffer_in_layer < 1 || buffer_out_layer < 1) {
                // layer assignment is incomplete
                return inner_tree;
              }

              // Build connection: start from inner_tree and work backwards
              BnetPtr current = inner_tree;

              // Add via stack from buffer_out_layer to buffer_in_layer
              int layer_step = (buffer_in_layer > buffer_out_layer) ? 1 : -1;
              odb::Point current_loc = buffer_out_loc;

              debugPrint(
                  logger_,
                  RSZ,
                  "buffer_removal",
                  1,
                  "Creating via stack: from_layer={} to_layer={} step={}",
                  buffer_out_layer,
                  buffer_in_layer,
                  layer_step);
              for (int layer = buffer_out_layer; layer != buffer_in_layer;
                   layer += layer_step) {
                int next_layer = layer + layer_step;
                debugPrint(logger_,
                           RSZ,
                           "buffer_removal",
                           2,
                           "Creating via from layer {} to layer {}",
                           layer,
                           next_layer);
                current = std::make_shared<BufferedNet>(BnetType::via,
                                                        current_loc,
                                                        layer,
                                                        next_layer,
                                                        current,
                                                        node->corner(),
                                                        this);
              }
              // Add wire if locations differ
              if (buffer_in_loc != buffer_out_loc) {
                current = std::make_shared<BufferedNet>(BnetType::wire,
                                                        buffer_in_loc,
                                                        buffer_in_layer,
                                                        current,
                                                        node->corner(),
                                                        this,
                                                        estimate_parasitics_);
              }

              return current;
            }
            // Not the stitching load, return as-is
            return node;
          }
          default:
            logger_->critical(RSZ, 130, "unhandled BufferedNet type");
        }
      },
      outer_tree);
}

////////////////////////////////////////////////////////////////

using grt::RoutePt;

class RoutePtHash
{
 public:
  size_t operator()(const RoutePt& pt) const;
};

class RoutePtEqual
{
 public:
  bool operator()(const RoutePt& pt1, const RoutePt& pt2) const;
};

using GRoutePtAdjacents
    = std::unordered_map<RoutePt, std::vector<RoutePt>, RoutePtHash>;

size_t RoutePtHash::operator()(const RoutePt& pt) const
{
  size_t hash = sta::hash_init_value;
  sta::hashIncr(hash, pt.x());
  sta::hashIncr(hash, pt.y());
  sta::hashIncr(hash, pt.layer());
  return hash;
}

bool RoutePtEqual::operator()(const RoutePt& pt1, const RoutePt& pt2) const
{
  return pt1.x() == pt2.x() && pt1.y() == pt2.y() && pt1.layer() == pt2.layer();
}

static const RoutePt route_pt_null(0, 0, 0);

using RoutePtSet = std::unordered_set<RoutePt, RoutePtHash, RoutePtEqual>;
using RoutePtPinMap = std::unordered_map<RoutePt, sta::PinSeq, RoutePtHash>;

static BufferedNetPtr makeBufferedNet(
    RoutePt& from,
    RoutePt& to,
    GRoutePtAdjacents& adjacents,
    RoutePtPinMap& loc_pin_map,
    int level,
    const sta::Scene* corner,
    const Resizer* resizer,
    const est::EstimateParasitics* estimate_parasitics,
    utl::Logger* logger,
    sta::dbNetwork* db_network,
    RoutePtSet& visited)
{
  if (visited.contains(to)) {
    debugPrint(logger, RSZ, "groute_bnet", 2, "Loop found in groute");
    return nullptr;
  }
  visited.insert(to);

  odb::Point from_pt(from.x(), from.y());
  odb::Point to_pt(to.x(), to.y());

  BufferedNetPtr bnet = nullptr;
  const sta::PinSeq& pins = loc_pin_map[to];
  for (const sta::Pin* pin : pins) {
    if (db_network->isLoad(pin)) {
      auto load_bnet = make_shared<BufferedNet>(
          BufferedNetType::load, to_pt, pin, corner, resizer);
      debugPrint(logger,
                 RSZ,
                 "groute_bnet",
                 2,
                 "{:{}s}{}",
                 "",
                 level,
                 load_bnet->to_string(resizer));
      if (bnet) {
        bnet = make_shared<BufferedNet>(
            BufferedNetType::junction, to_pt, bnet, load_bnet, resizer);
      } else {
        bnet = std::move(load_bnet);
      }
    }
  }

  for (RoutePt& adj : adjacents[to]) {
    if (adj != from) {
      BufferedNetPtr bnet1 = makeBufferedNet(to,
                                             adj,
                                             adjacents,
                                             loc_pin_map,
                                             level + 1,
                                             corner,
                                             resizer,
                                             estimate_parasitics,
                                             logger,
                                             db_network,
                                             visited);
      if (bnet1) {
        if (bnet) {
          bnet = make_shared<BufferedNet>(
              BufferedNetType::junction, to_pt, bnet, bnet1, resizer);
        } else {
          bnet = std::move(bnet1);
        }
      }
    }
  }

  if (bnet && from != route_pt_null) {
    if (from_pt != to_pt) {
      bnet = make_shared<BufferedNet>(BufferedNetType::wire,
                                      from_pt,
                                      to.layer(),
                                      bnet,
                                      corner,
                                      resizer,
                                      estimate_parasitics);
    } else {
      bnet = make_shared<BufferedNet>(BufferedNetType::via,
                                      from_pt,
                                      from.layer(),
                                      to.layer(),
                                      bnet,
                                      corner,
                                      resizer);
    }
  }
  return bnet;
}

BufferedNetPtr Resizer::makeBufferedNetGroute(const sta::Pin* drvr_pin,
                                              const sta::Scene* corner)
{
  odb::dbNet* db_net = db_network_->findFlatDbNet(drvr_pin);
  if (db_net == nullptr) {
    logger_->error(RSZ,
                   106,
                   "Net for driver pin {} not found.",
                   db_network_->pathName(drvr_pin));
  }

  if (db_net->getTermCount() == 1) {
    logger_->warn(
        RSZ,
        104,
        "Net {} only has one pin. Check if it is intended to be dangling.",
        db_net->getName());
    return nullptr;
  }
  const sta::Net* net = db_network_->dbToSta(db_net);

  std::vector<grt::PinGridLocation> pin_grid_locs
      = global_router_->getPinGridPositions(db_net);

  bool found_drvr_route_pt = false;
  RoutePt drvr_route_pt;
  RoutePtPinMap loc_pin_map;

  for (grt::PinGridLocation& pin_loc : pin_grid_locs) {
    sta::Pin* pin = pin_loc.iterm ? db_network_->dbToSta(pin_loc.iterm)
                                  : db_network_->dbToSta(pin_loc.bterm);
    RoutePt pin_route_pt(
        pin_loc.grid_pt.getX(), pin_loc.grid_pt.getY(), pin_loc.conn_layer);
    debugPrint(logger_,
               RSZ,
               "repair_net",
               2,
               "pin {}{} grid ({} {}) layer {}",
               network_->pathName(pin),
               (pin == drvr_pin) ? " drvr" : "",
               pin_route_pt.x(),
               pin_route_pt.y(),
               pin_route_pt.layer());
    loc_pin_map[pin_route_pt].push_back(pin);
    if (pin == drvr_pin) {
      drvr_route_pt = pin_route_pt;
      found_drvr_route_pt = true;
    }
  }

  if (found_drvr_route_pt) {
    grt::NetRouteMap& route_map = global_router_->getRoutes();
    grt::GRoute& route = route_map[db_network_->staToDb(net)];
    GRoutePtAdjacents adjacents(route.size());
    for (grt::GSegment& seg : route) {
      RoutePt from(seg.init_x, seg.init_y, seg.init_layer);
      RoutePt to(seg.final_x, seg.final_y, seg.final_layer);
      debugPrint(logger_,
                 RSZ,
                 "repair_net",
                 2,
                 "route {} {} {} -> {} {} {}",
                 seg.init_x,
                 seg.init_y,
                 seg.init_layer,
                 seg.final_x,
                 seg.final_y,
                 seg.final_layer);
      adjacents[from].push_back(to);
      adjacents[to].push_back(from);
    }
    RoutePt null_route_pt(0, 0, 0);
    RoutePtSet visited;
    BufferedNetPtr bnet = rsz::makeBufferedNet(null_route_pt,
                                               drvr_route_pt,
                                               adjacents,
                                               loc_pin_map,
                                               0,
                                               corner,
                                               this,
                                               estimate_parasitics_,
                                               logger_,
                                               db_network_,
                                               visited);
    if (bnet) {
      if (bnet->loadCount() != pin_grid_locs.size() - 1) {
        // we are subtracting one to account for driver at the root of the
        // tree
        logger_->error(RSZ,
                       74,
                       "Failed to build tree from global routes for pin '{}' "
                       "and net '{}' at grid ({}, {}): found route to {} "
                       "pins, expected {}",
                       db_network_->pathName(drvr_pin),
                       db_net->getName(),
                       drvr_route_pt.x(),
                       drvr_route_pt.y(),
                       bnet->loadCount(),
                       pin_grid_locs.size() - 1);
      }
    }
    return bnet;
  }
  logger_->warn(RSZ,
                73,
                "driver pin {} not found in global routes",
                db_network_->pathName(drvr_pin));
  return nullptr;
}

BufferedNetPtr Resizer::makeBufferedNetDroute(const sta::Pin* drvr_pin,
                                              const sta::Scene* corner)
{
  if (drvr_pin == nullptr) {
    return nullptr;
  }
  odb::dbNet* db_net = db_network_->findFlatDbNet(drvr_pin);
  if (db_net == nullptr || db_net->isSpecial() || db_net->getTermCount() <= 1) {
    return nullptr;
  }
  odb::dbWire* wire = db_net->getWire();
  if (wire == nullptr || wire->length() == 0) {
    return nullptr;
  }

  struct DrouteWireSeg
  {
    int x1;
    int y1;
    int x2;
    int y2;
    int layer;
  };
  struct DrouteViaSeg
  {
    int x;
    int y;
    int bot_layer;
    int top_layer;
  };

  std::vector<DrouteWireSeg> wire_segs;
  std::vector<DrouteViaSeg> via_segs;

  odb::dbWireDecoder decoder;
  decoder.begin(wire);
  bool has_prev_pt = false;
  int prev_x = 0;
  int prev_y = 0;
  int cur_layer = 0;

  while (true) {
    const odb::dbWireDecoder::OpCode opcode = decoder.next();
    if (opcode == odb::dbWireDecoder::END_DECODE) {
      break;
    }
    switch (opcode) {
      case odb::dbWireDecoder::PATH:
      case odb::dbWireDecoder::JUNCTION:
      case odb::dbWireDecoder::SHORT:
      case odb::dbWireDecoder::VWIRE: {
        has_prev_pt = false;
        odb::dbTechLayer* layer = decoder.getLayer();
        cur_layer = (layer != nullptr) ? layer->getRoutingLevel() : 0;
        break;
      }
      case odb::dbWireDecoder::POINT:
      case odb::dbWireDecoder::POINT_EXT: {
        int x = 0;
        int y = 0;
        decoder.getPoint(x, y);
        odb::dbTechLayer* layer = decoder.getLayer();
        if (layer != nullptr) {
          cur_layer = layer->getRoutingLevel();
        }
        if (cur_layer > 0) {
          if (has_prev_pt && (x != prev_x || y != prev_y)) {
            wire_segs.push_back({prev_x, prev_y, x, y, cur_layer});
            if (wire_segs.size() > 5000) {
              return nullptr;
            }
          }
          prev_x = x;
          prev_y = y;
          has_prev_pt = true;
        }
        break;
      }
      case odb::dbWireDecoder::VIA: {
        odb::dbVia* via = decoder.getVia();
        if (has_prev_pt && via != nullptr) {
          odb::dbTechLayer* bot = via->getBottomLayer();
          odb::dbTechLayer* top = via->getTopLayer();
          if (bot != nullptr && top != nullptr) {
            const int l1 = bot->getRoutingLevel();
            const int l2 = top->getRoutingLevel();
            if (l1 > 0 && l2 > 0 && l1 != l2) {
              via_segs.push_back(
                  {prev_x, prev_y, std::min(l1, l2), std::max(l1, l2)});
            }
          }
        }
        odb::dbTechLayer* exit_layer = decoder.getLayer();
        if (exit_layer != nullptr) {
          cur_layer = exit_layer->getRoutingLevel();
        }
        break;
      }
      case odb::dbWireDecoder::TECH_VIA: {
        odb::dbTechVia* via = decoder.getTechVia();
        if (has_prev_pt && via != nullptr) {
          odb::dbTechLayer* bot = via->getBottomLayer();
          odb::dbTechLayer* top = via->getTopLayer();
          if (bot != nullptr && top != nullptr) {
            const int l1 = bot->getRoutingLevel();
            const int l2 = top->getRoutingLevel();
            if (l1 > 0 && l2 > 0 && l1 != l2) {
              via_segs.push_back(
                  {prev_x, prev_y, std::min(l1, l2), std::max(l1, l2)});
            }
          }
        }
        odb::dbTechLayer* exit_layer = decoder.getLayer();
        if (exit_layer != nullptr) {
          cur_layer = exit_layer->getRoutingLevel();
        }
        break;
      }
      default:
        break;
    }
  }

  if (wire_segs.empty() && via_segs.empty()) {
    return nullptr;
  }

  struct PinBox
  {
    odb::Rect box;
    int layer;
  };
  struct PinEntry
  {
    sta::Pin* pin;
    bool is_drvr;
    std::vector<PinBox> boxes;
  };

  std::vector<PinEntry> pin_entries;
  int expected_load_count = 0;

  for (odb::dbITerm* iterm : db_net->getITerms()) {
    if (iterm == nullptr || iterm->getSigType().isSupply()) {
      continue;
    }
    sta::Pin* pin = db_network_->dbToSta(iterm);
    if (pin == nullptr) {
      continue;
    }
    const bool is_drvr = (pin == drvr_pin);
    const bool is_load = db_network_->isLoad(pin);
    if (!is_drvr && !is_load) {
      continue;
    }
    if (is_load && !is_drvr) {
      ++expected_load_count;
    }
    PinEntry entry{pin, is_drvr, {}};
    for (const auto& [layer, rect] : iterm->getGeometries()) {
      if (layer != nullptr && layer->getRoutingLevel() > 0) {
        entry.boxes.push_back({rect, layer->getRoutingLevel()});
      }
    }
    if (entry.boxes.empty()) {
      const odb::Point loc = db_network_->location(pin);
      entry.boxes.push_back({odb::Rect(loc.x(), loc.y(), loc.x(), loc.y()), 1});
    }
    pin_entries.push_back(std::move(entry));
  }

  for (odb::dbBTerm* bterm : db_net->getBTerms()) {
    if (bterm == nullptr || bterm->getSigType().isSupply()) {
      continue;
    }
    sta::Pin* pin = db_network_->dbToSta(bterm);
    if (pin == nullptr) {
      continue;
    }
    const bool is_drvr = (pin == drvr_pin);
    const bool is_load = db_network_->isLoad(pin);
    if (!is_drvr && !is_load) {
      continue;
    }
    if (is_load && !is_drvr) {
      ++expected_load_count;
    }
    PinEntry entry{pin, is_drvr, {}};
    for (odb::dbBPin* bpin : bterm->getBPins()) {
      for (odb::dbBox* box : bpin->getBoxes()) {
        odb::dbTechLayer* layer = box->getTechLayer();
        if (layer != nullptr && layer->getRoutingLevel() > 0) {
          entry.boxes.push_back({box->getBox(), layer->getRoutingLevel()});
        }
      }
    }
    if (entry.boxes.empty()) {
      const odb::Point loc = db_network_->location(pin);
      entry.boxes.push_back({odb::Rect(loc.x(), loc.y(), loc.x(), loc.y()), 1});
    }
    pin_entries.push_back(std::move(entry));
  }

  if (expected_load_count == 0) {
    return nullptr;
  }

  const int64_t layer_penalty = std::max(100, dbu_ / 2);
  const int64_t max_pin_xy_dist = static_cast<int64_t>(std::max(1000, dbu_)) * 20;

  bool found_drvr_route_pt = false;
  RoutePt drvr_route_pt;
  RoutePtPinMap loc_pin_map;
  std::map<int, std::vector<odb::Point>> layer_split_pts;

  auto add_split_pt = [&](int layer, int x, int y) {
    if (layer > 0) {
      layer_split_pts[layer].emplace_back(x, y);
    }
  };

  for (const DrouteWireSeg& s : wire_segs) {
    add_split_pt(s.layer, s.x1, s.y1);
    add_split_pt(s.layer, s.x2, s.y2);
  }
  for (const DrouteViaSeg& v : via_segs) {
    for (int l = v.bot_layer; l <= v.top_layer; ++l) {
      add_split_pt(l, v.x, v.y);
    }
  }

  for (const PinEntry& entry : pin_entries) {
    int64_t best_cost = std::numeric_limits<int64_t>::max();
    int64_t best_xy_dist = std::numeric_limits<int64_t>::max();
    RoutePt best_pt;
    bool found_pt = false;

    for (const PinBox& pb : entry.boxes) {
      const int bx1 = pb.box.xMin();
      const int by1 = pb.box.yMin();
      const int bx2 = pb.box.xMax();
      const int by2 = pb.box.yMax();
      const int bcx = (bx1 + bx2) / 2;
      const int bcy = (by1 + by2) / 2;

      auto eval_point = [&](int px, int py, int player, bool is_endpoint) {
        const int64_t dx = std::max({0, bx1 - px, px - bx2});
        const int64_t dy = std::max({0, by1 - py, py - by2});
        const int64_t xy_dist = dx + dy;
        const int64_t l_diff = std::abs(player - pb.layer);
        const int64_t cost
            = (xy_dist + l_diff * layer_penalty) * 2 + (is_endpoint ? 0 : 1);
        if (cost < best_cost) {
          best_cost = cost;
          best_xy_dist = xy_dist;
          best_pt = RoutePt(px, py, player);
          found_pt = true;
        }
      };

      for (const DrouteWireSeg& s : wire_segs) {
        eval_point(s.x1, s.y1, s.layer, true);
        eval_point(s.x2, s.y2, s.layer, true);
        if (s.x1 == s.x2 || s.y1 == s.y2) {
          const int sx1 = std::min(s.x1, s.x2);
          const int sx2 = std::max(s.x1, s.x2);
          const int sy1 = std::min(s.y1, s.y2);
          const int sy2 = std::max(s.y1, s.y2);
          const int qx = std::clamp(bcx, sx1, sx2);
          const int qy = std::clamp(bcy, sy1, sy2);
          const bool is_ep
              = (qx == s.x1 && qy == s.y1) || (qx == s.x2 && qy == s.y2);
          eval_point(qx, qy, s.layer, is_ep);
        }
      }
      for (const DrouteViaSeg& v : via_segs) {
        eval_point(v.x, v.y, v.bot_layer, true);
        eval_point(v.x, v.y, v.top_layer, true);
      }
    }

    if (!found_pt || best_xy_dist > max_pin_xy_dist) {
      return nullptr;
    }

    add_split_pt(best_pt.layer(), best_pt.x(), best_pt.y());
    if (entry.is_drvr) {
      drvr_route_pt = best_pt;
      found_drvr_route_pt = true;
    }
    if (db_network_->isLoad(entry.pin) && !entry.is_drvr) {
      loc_pin_map[best_pt].push_back(entry.pin);
    }
  }

  if (!found_drvr_route_pt) {
    return nullptr;
  }

  // Also split perpendicular crossing segments on the same layer.
  for (size_t i = 0; i < wire_segs.size(); ++i) {
    const DrouteWireSeg& s1 = wire_segs[i];
    const bool s1_horiz = (s1.y1 == s1.y2);
    const bool s1_vert = (s1.x1 == s1.x2);
    if (!s1_horiz && !s1_vert) {
      continue;
    }
    for (size_t j = i + 1; j < wire_segs.size(); ++j) {
      const DrouteWireSeg& s2 = wire_segs[j];
      if (s1.layer != s2.layer) {
        continue;
      }
      if (s1_horiz && s2.x1 == s2.x2) {
        const int hx1 = std::min(s1.x1, s1.x2);
        const int hx2 = std::max(s1.x1, s1.x2);
        const int vy1 = std::min(s2.y1, s2.y2);
        const int vy2 = std::max(s2.y1, s2.y2);
        if (s2.x1 >= hx1 && s2.x1 <= hx2 && s1.y1 >= vy1 && s1.y1 <= vy2) {
          add_split_pt(s1.layer, s2.x1, s1.y1);
        }
      } else if (s1_vert && s2.y1 == s2.y2) {
        const int vy1 = std::min(s1.y1, s1.y2);
        const int vy2 = std::max(s1.y1, s1.y2);
        const int hx1 = std::min(s2.x1, s2.x2);
        const int hx2 = std::max(s2.x1, s2.x2);
        if (s1.x1 >= hx1 && s1.x1 <= hx2 && s2.y1 >= vy1 && s2.y1 <= vy2) {
          add_split_pt(s1.layer, s1.x1, s2.y1);
        }
      }
    }
  }

  for (auto& [layer, pts] : layer_split_pts) {
    std::sort(pts.begin(), pts.end(), [](const odb::Point& a, const odb::Point& b) {
      return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });
    pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
  }

  GRoutePtAdjacents raw_adjacents;
  auto add_edge = [&](const RoutePt& u, const RoutePt& v) {
    if (RoutePtEqual()(u, v)) {
      return;
    }
    raw_adjacents[u].push_back(v);
    raw_adjacents[v].push_back(u);
  };

  for (const DrouteWireSeg& s : wire_segs) {
    const auto& pts = layer_split_pts[s.layer];
    if (s.x1 == s.x2) {
      const int x = s.x1;
      const int y_min = std::min(s.y1, s.y2);
      const int y_max = std::max(s.y1, s.y2);
      auto it = std::lower_bound(
          pts.begin(),
          pts.end(),
          odb::Point(x, y_min),
          [](const odb::Point& a, const odb::Point& b) {
            return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
          });
      int prev_py = y_min;
      bool first = true;
      for (; it != pts.end() && it->x() == x && it->y() <= y_max; ++it) {
        if (!first && it->y() != prev_py) {
          add_edge(RoutePt(x, prev_py, s.layer), RoutePt(x, it->y(), s.layer));
        }
        prev_py = it->y();
        first = false;
      }
    } else if (s.y1 == s.y2) {
      const int y = s.y1;
      const int x_min = std::min(s.x1, s.x2);
      const int x_max = std::max(s.x1, s.x2);
      int prev_px = x_min;
      bool first = true;
      for (const odb::Point& p : pts) {
        if (p.x() < x_min) {
          continue;
        }
        if (p.x() > x_max) {
          break;
        }
        if (p.y() == y) {
          if (!first && p.x() != prev_px) {
            add_edge(RoutePt(prev_px, y, s.layer), RoutePt(p.x(), y, s.layer));
          }
          prev_px = p.x();
          first = false;
        }
      }
    } else {
      add_edge(RoutePt(s.x1, s.y1, s.layer), RoutePt(s.x2, s.y2, s.layer));
    }
  }

  for (const DrouteViaSeg& v : via_segs) {
    for (int l = v.bot_layer; l < v.top_layer; ++l) {
      add_edge(RoutePt(v.x, v.y, l), RoutePt(v.x, v.y, l + 1));
    }
  }

  // Ensure drvr_route_pt and all load pin RoutePts exist in raw_adjacents.
  raw_adjacents[drvr_route_pt];
  for (const auto& [pt, pins] : loc_pin_map) {
    raw_adjacents[pt];
  }

  for (auto& [u, nbrs] : raw_adjacents) {
    std::sort(nbrs.begin(), nbrs.end());
    nbrs.erase(std::unique(nbrs.begin(), nbrs.end(), RoutePtEqual()),
               nbrs.end());
  }

  // If the decoded wire graph has multiple connected components (e.g. a
  // multi-shape pin internally bridges two segments), connect disconnected
  // components to the driver component using pure wire/via edges.
  RoutePtSet comp_visited;
  std::queue<RoutePt> bfs_q;
  comp_visited.insert(drvr_route_pt);
  bfs_q.push(drvr_route_pt);
  while (!bfs_q.empty()) {
    const RoutePt cur = bfs_q.front();
    bfs_q.pop();
    for (const RoutePt& nxt : raw_adjacents[cur]) {
      if (!comp_visited.contains(nxt)) {
        comp_visited.insert(nxt);
        bfs_q.push(nxt);
      }
    }
  }

  while (comp_visited.size() < raw_adjacents.size()) {
    int64_t best_dist = std::numeric_limits<int64_t>::max();
    RoutePt best_u;
    RoutePt best_v;
    bool found_bridge = false;
    for (const RoutePt& u : comp_visited) {
      for (const auto& [v, _] : raw_adjacents) {
        if (comp_visited.contains(v)) {
          continue;
        }
        const int64_t dist
            = std::abs(u.x() - v.x()) + std::abs(u.y() - v.y())
              + std::abs(u.layer() - v.layer()) * layer_penalty;
        if (dist < best_dist) {
          best_dist = dist;
          best_u = u;
          best_v = v;
          found_bridge = true;
        }
      }
    }
    if (!found_bridge) {
      break;
    }
    RoutePt cur = best_u;
    if (cur.x() != best_v.x() || cur.y() != best_v.y()) {
      const RoutePt xy_pt(best_v.x(), best_v.y(), cur.layer());
      add_edge(cur, xy_pt);
      comp_visited.insert(xy_pt);
      cur = xy_pt;
    }
    while (cur.layer() != best_v.layer()) {
      const int next_l
          = (best_v.layer() > cur.layer()) ? (cur.layer() + 1) : (cur.layer() - 1);
      const RoutePt via_pt(cur.x(), cur.y(), next_l);
      add_edge(cur, via_pt);
      comp_visited.insert(via_pt);
      cur = via_pt;
    }
    bfs_q.push(best_v);
    comp_visited.insert(best_v);
    while (!bfs_q.empty()) {
      const RoutePt c = bfs_q.front();
      bfs_q.pop();
      for (const RoutePt& nxt : raw_adjacents[c]) {
        if (!comp_visited.contains(nxt)) {
          comp_visited.insert(nxt);
          bfs_q.push(nxt);
        }
      }
    }
  }

  // Extract a cycle-free shortest-path tree rooted at drvr_route_pt.
  using DistEntry = std::pair<int64_t, RoutePt>;
  std::priority_queue<DistEntry, std::vector<DistEntry>, std::greater<DistEntry>>
      pq;
  std::unordered_map<RoutePt, int64_t, RoutePtHash, RoutePtEqual> dist_map;
  std::unordered_map<RoutePt, RoutePt, RoutePtHash, RoutePtEqual> parent_map;

  dist_map[drvr_route_pt] = 0;
  pq.emplace(0, drvr_route_pt);

  while (!pq.empty()) {
    const auto [d, u] = pq.top();
    pq.pop();
    if (d > dist_map[u]) {
      continue;
    }
    for (const RoutePt& v : raw_adjacents[u]) {
      const int64_t w
          = std::abs(u.x() - v.x()) + std::abs(u.y() - v.y())
            + std::abs(u.layer() - v.layer()) * 10;
      const int64_t nd = d + std::max<int64_t>(1, w);
      auto it = dist_map.find(v);
      if (it == dist_map.end() || nd < it->second) {
        dist_map[v] = nd;
        parent_map[v] = u;
        pq.emplace(nd, v);
      }
    }
  }

  GRoutePtAdjacents tree_adjacents;
  for (const auto& [v, p] : parent_map) {
    tree_adjacents[p].push_back(v);
    tree_adjacents[v].push_back(p);
  }

  RoutePt null_route_pt(0, 0, 0);
  RoutePtSet visited;
  BufferedNetPtr bnet = rsz::makeBufferedNet(null_route_pt,
                                             drvr_route_pt,
                                             tree_adjacents,
                                             loc_pin_map,
                                             0,
                                             corner,
                                             this,
                                             estimate_parasitics_,
                                             logger_,
                                             db_network_,
                                             visited);
  if (bnet && bnet->loadCount() == expected_load_count) {
    return bnet;
  }
  return nullptr;
}

bool BufferedNet::fitsEnvelope(Metrics target)
{
  return maxLoadWireLength() <= target.max_load_wl && slack() >= target.slack
         && sta::fuzzyLessEqual(cap(), target.cap)
         && sta::fuzzyGreaterEqual(maxLoadSlew(), target.max_load_slew)
         && sta::fuzzyLessEqual(fanout(), target.fanout);
}

void Resizer::ensurePlacementBlockagesInitialized()
{
  if (!placement_blockages_initialized_) {
    initPlacementBlockages();
  }
}

void Resizer::initPlacementBlockages()
{
  placement_blockages_.clear();
  blockage_corners_.clear();
  blockage_corner_adj_.clear();
  placement_blockages_initialized_ = true;
  placement_row_height_ = 1400;

  if (const char* env_flag = std::getenv("SIZER_BLOCKAGE_AWARE_STEINER")) {
    if (std::string(env_flag) == "0" || std::string(env_flag) == "false") {
      blockage_aware_steiner_ = false;
    } else if (std::string(env_flag) == "1"
               || std::string(env_flag) == "true") {
      blockage_aware_steiner_ = true;
    }
  }

  if (!db_ || !db_->getChip()) {
    return;
  }
  odb::dbBlock* block = db_->getChip()->getBlock();
  if (!block) {
    return;
  }

  placement_core_area_ = block->getCoreArea();
  int min_row_h = std::numeric_limits<int>::max();
  struct RowBand
  {
    int x1;
    int x2;
  };
  std::map<std::pair<int, int>, std::vector<RowBand>> rows_by_y;

  for (odb::dbRow* row : block->getRows()) {
    if (row->getSite()) {
      min_row_h = std::min(min_row_h, row->getSite()->getHeight());
    }
    odb::Rect rbox = row->getBBox();
    if (!rbox.isInverted() && rbox.dx() > 0 && rbox.dy() > 0) {
      rows_by_y[{rbox.yMin(), rbox.yMax()}].push_back(
          {rbox.xMin(), rbox.xMax()});
    }
  }
  if (min_row_h < std::numeric_limits<int>::max() && min_row_h > 0) {
    placement_row_height_ = min_row_h;
  }

  // 1. Extract un-rowed rectangular cutouts inside core_area_ from dbRows.
  if (!rows_by_y.empty() && !placement_core_area_.isInverted()) {
    const int cx1 = placement_core_area_.xMin();
    const int cx2 = placement_core_area_.xMax();
    std::map<std::pair<int, int>, int> active_holes;
    int prev_y2 = placement_core_area_.yMin();

    auto flush_band
        = [&](int y1, const std::set<std::pair<int, int>>& cur_holes) {
            for (auto it = active_holes.begin(); it != active_holes.end();) {
              if (!cur_holes.contains(it->first)) {
                odb::Rect rect(
                    it->first.first, it->second, it->first.second, y1);
                if (rect.dx() > placement_row_height_
                    && rect.dy() > placement_row_height_) {
                  placement_blockages_.push_back(rect);
                }
                it = active_holes.erase(it);
              } else {
                ++it;
              }
            }
            for (const auto& h : cur_holes) {
              if (!active_holes.contains(h)) {
                active_holes[h] = y1;
              }
            }
          };

    for (auto& [y_band, x_segs] : rows_by_y) {
      const int y1 = y_band.first;
      const int y2 = y_band.second;
      if (y1 > prev_y2) {
        std::set<std::pair<int, int>> full_hole{{cx1, cx2}};
        flush_band(prev_y2, full_hole);
      }
      std::sort(x_segs.begin(),
                x_segs.end(),
                [](const RowBand& a, const RowBand& b) {
                  return a.x1 < b.x1 || (a.x1 == b.x1 && a.x2 < b.x2);
                });
      std::set<std::pair<int, int>> cur_holes;
      int cur_x = cx1;
      for (const RowBand& seg : x_segs) {
        const int sx1 = std::max(cx1, seg.x1);
        const int sx2 = std::min(cx2, seg.x2);
        if (sx2 <= sx1) {
          continue;
        }
        if (sx1 > cur_x) {
          cur_holes.insert({cur_x, sx1});
        }
        cur_x = std::max(cur_x, sx2);
      }
      if (cur_x < cx2) {
        cur_holes.insert({cur_x, cx2});
      }
      flush_band(y1, cur_holes);
      prev_y2 = std::max(prev_y2, y2);
    }
    if (prev_y2 < placement_core_area_.yMax()) {
      std::set<std::pair<int, int>> full_hole{{cx1, cx2}};
      flush_band(prev_y2, full_hole);
    }
    std::set<std::pair<int, int>> empty_holes;
    flush_band(placement_core_area_.yMax(), empty_holes);
  }

  // 2. Also include any placed BLOCK macro or hard dbBlockage not already
  // contained inside an extracted un-rowed cutout.
  const int margin = std::max(placement_row_height_ / 2, 100);
  auto is_covered = [&](const odb::Rect& box) {
    for (const odb::Rect& existing : placement_blockages_) {
      if (box.xMin() >= existing.xMin() && box.xMax() <= existing.xMax()
          && box.yMin() >= existing.yMin() && box.yMax() <= existing.yMax()) {
        return true;
      }
    }
    return false;
  };

  for (odb::dbInst* db_inst : block->getInsts()) {
    if (!db_inst->isBlock() || !db_inst->isPlaced()) {
      continue;
    }
    odb::Rect box = db_inst->getBBox()->getBox();
    if (!is_covered(box)) {
      box.init(box.xMin() - margin,
               box.yMin() - margin,
               box.xMax() + margin,
               box.yMax() + margin);
      placement_blockages_.push_back(box);
    }
  }

  for (odb::dbBlockage* blockage : block->getBlockages()) {
    if (!blockage->isSoft() && blockage->getBBox()) {
      odb::Rect box = blockage->getBBox()->getBox();
      if (box.dx() > 2 * placement_row_height_
          && box.dy() > 2 * placement_row_height_ && !is_covered(box)) {
        placement_blockages_.push_back(box);
      }
    }
  }

  if (placement_blockages_.empty()) {
    return;
  }

  // 3. Build padded exterior corners and precompute the blockage-free L-path
  // visibility graph between corners.
  const int pad = std::max(placement_row_height_ / 2, 100);
  for (const odb::Rect& box : placement_blockages_) {
    const int xs[2] = {box.xMin() - pad, box.xMax() + pad};
    const int ys[2] = {box.yMin() - pad, box.yMax() + pad};
    for (int cx : xs) {
      for (int cy : ys) {
        odb::Point pt(cx, cy);
        if (!isPointInPlacementBlockage(pt)) {
          blockage_corners_.push_back(pt);
        }
      }
    }
  }
  std::sort(blockage_corners_.begin(),
            blockage_corners_.end(),
            [](const odb::Point& a, const odb::Point& b) {
              return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
            });
  blockage_corners_.erase(
      std::unique(blockage_corners_.begin(), blockage_corners_.end()),
      blockage_corners_.end());

  const int nc = static_cast<int>(blockage_corners_.size());
  blockage_corner_adj_.resize(nc);
  for (int i = 0; i < nc; ++i) {
    for (int j = i + 1; j < nc; ++j) {
      if (findBlockageFreeLPath(blockage_corners_[i],
                                blockage_corners_[j],
                                nullptr)) {
        const int64_t w = odb::Point::manhattanDistance(blockage_corners_[i],
                                                        blockage_corners_[j]);
        blockage_corner_adj_[i].emplace_back(j, w);
        blockage_corner_adj_[j].emplace_back(i, w);
      }
    }
  }
}

bool Resizer::isPointInPlacementBlockage(const odb::Point& pt,
                                         odb::Rect* hit_box) const
{
  if (!placement_core_area_.isInverted()) {
    if (pt.x() < placement_core_area_.xMin()
        || pt.x() > placement_core_area_.xMax()
        || pt.y() < placement_core_area_.yMin()
        || pt.y() > placement_core_area_.yMax()) {
      return true;
    }
  }
  for (const odb::Rect& box : placement_blockages_) {
    if (pt.x() > box.xMin() && pt.x() < box.xMax() && pt.y() > box.yMin()
        && pt.y() < box.yMax()) {
      if (hit_box) {
        *hit_box = box;
      }
      return true;
    }
  }
  return false;
}

bool Resizer::segmentCrossesPlacementBlockage(const odb::Point& p1,
                                              const odb::Point& p2,
                                              odb::Rect* hit_box) const
{
  if (placement_blockages_.empty() || p1 == p2) {
    return false;
  }
  const int seg_xmin = std::min(p1.x(), p2.x());
  const int seg_xmax = std::max(p1.x(), p2.x());
  const int seg_ymin = std::min(p1.y(), p2.y());
  const int seg_ymax = std::max(p1.y(), p2.y());

  const double dx = p2.x() - p1.x();
  const double dy = p2.y() - p1.y();

  for (const odb::Rect& box : placement_blockages_) {
    if (seg_xmax <= box.xMin() || seg_xmin >= box.xMax()
        || seg_ymax <= box.yMin() || seg_ymin >= box.yMax()) {
      continue;
    }
    double t0 = 0.0;
    double t1 = 1.0;
    auto clip = [&](double p, double q) -> bool {
      if (std::abs(p) < 1e-9) {
        return q > 0.0;
      }
      const double r = q / p;
      if (p < 0.0) {
        if (r >= t1) {
          return false;
        }
        if (r > t0) {
          t0 = r;
        }
      } else {
        if (r <= t0) {
          return false;
        }
        if (r < t1) {
          t1 = r;
        }
      }
      return true;
    };
    if (clip(-dx, p1.x() - box.xMin()) && clip(dx, box.xMax() - p1.x())
        && clip(-dy, p1.y() - box.yMin()) && clip(dy, box.yMax() - p1.y())
        && t0 < t1) {
      if (hit_box) {
        *hit_box = box;
      }
      return true;
    }
  }
  return false;
}

odb::Point Resizer::snapPointOutsidePlacementBlockage(
    const odb::Point& pt,
    const odb::Point& ref_pt) const
{
  odb::Point cur = pt;
  const int pad = std::max(placement_row_height_ / 2, 100);
  if (!placement_core_area_.isInverted()) {
    cur.setX(std::clamp(cur.x(),
                        placement_core_area_.xMin() + pad,
                        placement_core_area_.xMax() - pad));
    cur.setY(std::clamp(cur.y(),
                        placement_core_area_.yMin() + pad,
                        placement_core_area_.yMax() - pad));
  }
  for (int pass = 0; pass < 4; ++pass) {
    odb::Rect box;
    if (!isPointInPlacementBlockage(cur, &box)) {
      return cur;
    }
    const odb::Point cands[4] = {
        odb::Point(box.xMin() - pad, cur.y()),
        odb::Point(box.xMax() + pad, cur.y()),
        odb::Point(cur.x(), box.yMin() - pad),
        odb::Point(cur.x(), box.yMax() + pad),
    };
    odb::Point best_pt = cur;
    int64_t best_cost = std::numeric_limits<int64_t>::max();
    bool found = false;
    for (const odb::Point& cand : cands) {
      if (!isPointInPlacementBlockage(cand)) {
        const int64_t cost
            = odb::Point::manhattanDistance(cand, cur)
              + odb::Point::manhattanDistance(cand, ref_pt) / 4;
        if (cost < best_cost) {
          best_cost = cost;
          best_pt = cand;
          found = true;
        }
      }
    }
    if (found) {
      return best_pt;
    }
    for (const odb::Point& corner : blockage_corners_) {
      const int64_t cost
          = odb::Point::manhattanDistance(corner, cur)
            + odb::Point::manhattanDistance(corner, ref_pt) / 4;
      if (cost < best_cost) {
        best_cost = cost;
        best_pt = corner;
        found = true;
      }
    }
    if (found) {
      return best_pt;
    }
    break;
  }
  return cur;
}

bool Resizer::findBlockageFreeLPath(const odb::Point& u,
                                    const odb::Point& v,
                                    std::vector<odb::Point>* out_pts) const
{
  if (u == v) {
    if (out_pts) {
      *out_pts = {u};
    }
    return true;
  }
  if ((u.x() == v.x() || u.y() == v.y())
      && !segmentCrossesPlacementBlockage(u, v)) {
    if (out_pts) {
      *out_pts = {u, v};
    }
    return true;
  }
  const odb::Point m1(v.x(), u.y());
  if (!isPointInPlacementBlockage(m1)
      && !segmentCrossesPlacementBlockage(u, m1)
      && !segmentCrossesPlacementBlockage(m1, v)) {
    if (out_pts) {
      *out_pts = {u, m1, v};
    }
    return true;
  }
  const odb::Point m2(u.x(), v.y());
  if (!isPointInPlacementBlockage(m2)
      && !segmentCrossesPlacementBlockage(u, m2)
      && !segmentCrossesPlacementBlockage(m2, v)) {
    if (out_pts) {
      *out_pts = {u, m2, v};
    }
    return true;
  }
  return false;
}

std::vector<odb::Point> Resizer::findBlockageDetourPath(
    const odb::Point& p1,
    const odb::Point& p2) const
{
  if (placement_blockages_.empty() || p1 == p2) {
    return {p1, p2};
  }
  const bool p1_in_blk = isPointInPlacementBlockage(p1);
  const bool p2_in_blk = isPointInPlacementBlockage(p2);
  if (!p1_in_blk && !p2_in_blk && !segmentCrossesPlacementBlockage(p1, p2)) {
    return {p1, p2};
  }

  const odb::Point s_out
      = p1_in_blk ? snapPointOutsidePlacementBlockage(p1, p2) : p1;
  const odb::Point e_out
      = p2_in_blk ? snapPointOutsidePlacementBlockage(p2, p1) : p2;

  std::vector<odb::Point> mid_pts;
  if (!findBlockageFreeLPath(s_out, e_out, &mid_pts)) {
    const int nc = static_cast<int>(blockage_corners_.size());
    const int s_idx = nc;
    const int e_idx = nc + 1;
    const int tot = nc + 2;
    constexpr int64_t kInf = std::numeric_limits<int64_t>::max() / 4;
    std::vector<int64_t> dist(tot, kInf);
    std::vector<int> parent(tot, -1);

    using QItem = std::tuple<int64_t, int64_t, int>;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
    dist[s_idx] = 0;
    pq.emplace(odb::Point::manhattanDistance(s_out, e_out), 0, s_idx);

    while (!pq.empty()) {
      const auto [f, d, u] = pq.top();
      pq.pop();
      if (u == e_idx) {
        break;
      }
      if (d > dist[u]) {
        continue;
      }
      if (u == s_idx) {
        for (int i = 0; i < nc; ++i) {
          const int64_t w
              = odb::Point::manhattanDistance(s_out, blockage_corners_[i]);
          if (d + w < dist[i]
              && findBlockageFreeLPath(s_out, blockage_corners_[i], nullptr)) {
            dist[i] = d + w;
            parent[i] = u;
            const int64_t h
                = odb::Point::manhattanDistance(blockage_corners_[i], e_out);
            pq.emplace(dist[i] + h, dist[i], i);
          }
        }
      } else {
        const int64_t w_end
            = odb::Point::manhattanDistance(blockage_corners_[u], e_out);
        if (d + w_end < dist[e_idx]
            && findBlockageFreeLPath(blockage_corners_[u], e_out, nullptr)) {
          dist[e_idx] = d + w_end;
          parent[e_idx] = u;
          pq.emplace(dist[e_idx], dist[e_idx], e_idx);
        }
        for (const auto& [v, w] : blockage_corner_adj_[u]) {
          if (d + w < dist[v]) {
            dist[v] = d + w;
            parent[v] = u;
            const int64_t h
                = odb::Point::manhattanDistance(blockage_corners_[v], e_out);
            pq.emplace(dist[v] + h, dist[v], v);
          }
        }
      }
    }

    if (parent[e_idx] == -1) {
      mid_pts = {s_out, e_out};
    } else {
      std::vector<int> idx_path;
      for (int cur = e_idx; cur != -1; cur = parent[cur]) {
        idx_path.push_back(cur);
      }
      std::reverse(idx_path.begin(), idx_path.end());
      auto get_pt = [&](int idx) -> odb::Point {
        if (idx == s_idx) {
          return s_out;
        }
        if (idx == e_idx) {
          return e_out;
        }
        return blockage_corners_[idx];
      };
      mid_pts.push_back(s_out);
      for (size_t k = 0; k + 1 < idx_path.size(); ++k) {
        std::vector<odb::Point> seg;
        if (findBlockageFreeLPath(
                get_pt(idx_path[k]), get_pt(idx_path[k + 1]), &seg)) {
          for (size_t m = 1; m < seg.size(); ++m) {
            if (seg[m] != mid_pts.back()) {
              mid_pts.push_back(seg[m]);
            }
          }
        }
      }
      // Simplify collinear points where direct segment does not cross blockages
      if (mid_pts.size() > 2) {
        std::vector<odb::Point> simplified;
        simplified.push_back(mid_pts.front());
        for (size_t i = 1; i + 1 < mid_pts.size(); ++i) {
          const odb::Point& p0 = simplified.back();
          const odb::Point& p1_pt = mid_pts[i];
          const odb::Point& p2_pt = mid_pts[i + 1];
          const bool collinear
              = (p0.x() == p1_pt.x() && p1_pt.x() == p2_pt.x())
                || (p0.y() == p1_pt.y() && p1_pt.y() == p2_pt.y());
          if (collinear && !segmentCrossesPlacementBlockage(p0, p2_pt)) {
            continue;
          }
          simplified.push_back(p1_pt);
        }
        simplified.push_back(mid_pts.back());
        mid_pts = std::move(simplified);
      }
    }
  }

  std::vector<odb::Point> full_pts;
  if (p1 != s_out) {
    full_pts.push_back(p1);
  }
  for (const odb::Point& pt : mid_pts) {
    if (full_pts.empty() || pt != full_pts.back()) {
      full_pts.push_back(pt);
    }
  }
  if (p2 != e_out && (full_pts.empty() || p2 != full_pts.back())) {
    full_pts.push_back(p2);
  }
  return full_pts;
}

bool Resizer::segmentCrossesPlacementBlockageInterior(
    const odb::Point& p1,
    const odb::Point& p2) const
{
  if (placement_blockages_.empty() || p1 == p2) {
    return false;
  }
  const int inner_margin = std::max(placement_row_height_ * 3, 8400);
  const int min_cross_len = std::max(placement_row_height_ * 4, 11200);
  const int seg_xmin = std::min(p1.x(), p2.x());
  const int seg_xmax = std::max(p1.x(), p2.x());
  const int seg_ymin = std::min(p1.y(), p2.y());
  const int seg_ymax = std::max(p1.y(), p2.y());
  const double dx = p2.x() - p1.x();
  const double dy = p2.y() - p1.y();
  const double total_len = odb::Point::manhattanDistance(p1, p2);

  for (const odb::Rect& raw_box : placement_blockages_) {
    if (raw_box.dx() <= inner_margin * 2 + min_cross_len
        || raw_box.dy() <= inner_margin * 2 + min_cross_len) {
      continue;
    }
    const odb::Rect box(raw_box.xMin() + inner_margin,
                        raw_box.yMin() + inner_margin,
                        raw_box.xMax() - inner_margin,
                        raw_box.yMax() - inner_margin);
    if (seg_xmax <= box.xMin() || seg_xmin >= box.xMax()
        || seg_ymax <= box.yMin() || seg_ymin >= box.yMax()) {
      continue;
    }
    double t0 = 0.0;
    double t1 = 1.0;
    auto clip = [&](double p, double q) -> bool {
      if (std::abs(p) < 1e-9) {
        return q > 0.0;
      }
      const double r = q / p;
      if (p < 0.0) {
        if (r >= t1) {
          return false;
        }
        if (r > t0) {
          t0 = r;
        }
      } else {
        if (r <= t0) {
          return false;
        }
        if (r < t1) {
          t1 = r;
        }
      }
      return true;
    };
    if (clip(-dx, p1.x() - box.xMin()) && clip(dx, box.xMax() - p1.x())
        && clip(-dy, p1.y() - box.yMin()) && clip(dy, box.yMax() - p1.y())
        && t0 < t1) {
      if ((t1 - t0) * total_len >= min_cross_len) {
        return true;
      }
    }
  }
  return false;
}

bool Resizer::treeCrossesPlacementBlockage(const BufferedNetPtr& tree) const
{
  if (!tree || placement_blockages_.empty()) {
    return false;
  }

  std::vector<odb::Point> pin_locs;
  pin_locs.push_back(tree->location());
  std::function<void(const BufferedNet*)> collect_pins
      = [&](const BufferedNet* node) {
          if (!node) {
            return;
          }
          switch (node->type()) {
            case BufferedNetType::load:
              pin_locs.push_back(node->location());
              break;
            case BufferedNetType::wire:
            case BufferedNetType::via:
            case BufferedNetType::buffer:
              collect_pins(node->ref().get());
              break;
            case BufferedNetType::junction:
              collect_pins(node->ref().get());
              collect_pins(node->ref2().get());
              break;
          }
        };
  collect_pins(tree.get());

  std::vector<bool> box_has_pin(placement_blockages_.size(), false);
  for (size_t i = 0; i < placement_blockages_.size(); ++i) {
    const odb::Rect& b = placement_blockages_[i];
    for (const odb::Point& pt : pin_locs) {
      if (pt.x() >= b.xMin() && pt.x() <= b.xMax() && pt.y() >= b.yMin()
          && pt.y() <= b.yMax()) {
        box_has_pin[i] = true;
        break;
      }
    }
  }

  // Halo width around FIXED BLOCK macros is ~3.1-4.5 um (6200-9000 dbu).
  // For a macro where this net has NO pins (!box_has_pin[i]), any segment
  // entering deeper than the halo (>= 3.5 um = 7000 dbu) is crossing the
  // macro body itself.
  const int no_pin_margin = std::max(placement_row_height_ * 5 / 2, 7000);
  const int no_pin_min_len = std::max(placement_row_height_ * 2, 5600);

  auto seg_crosses = [&](const odb::Point& p1, const odb::Point& p2) -> bool {
    if (p1 == p2) {
      return false;
    }
    if (segmentCrossesPlacementBlockageInterior(p1, p2)) {
      return true;
    }
    const int seg_xmin = std::min(p1.x(), p2.x());
    const int seg_xmax = std::max(p1.x(), p2.x());
    const int seg_ymin = std::min(p1.y(), p2.y());
    const int seg_ymax = std::max(p1.y(), p2.y());
    const double dx = p2.x() - p1.x();
    const double dy = p2.y() - p1.y();
    const double total_len = odb::Point::manhattanDistance(p1, p2);

    for (size_t i = 0; i < placement_blockages_.size(); ++i) {
      if (box_has_pin[i]) {
        continue;
      }
      const odb::Rect& raw_box = placement_blockages_[i];
      if (raw_box.dx() <= no_pin_margin * 2
          || raw_box.dy() <= no_pin_margin * 2) {
        continue;
      }
      const odb::Rect box(raw_box.xMin() + no_pin_margin,
                          raw_box.yMin() + no_pin_margin,
                          raw_box.xMax() - no_pin_margin,
                          raw_box.yMax() - no_pin_margin);
      if (seg_xmax <= box.xMin() || seg_xmin >= box.xMax()
          || seg_ymax <= box.yMin() || seg_ymin >= box.yMax()) {
        continue;
      }
      double t0 = 0.0;
      double t1 = 1.0;
      auto clip = [&](double p, double q) -> bool {
        if (std::abs(p) < 1e-9) {
          return q > 0.0;
        }
        const double r = q / p;
        if (p < 0.0) {
          if (r >= t1) {
            return false;
          }
          if (r > t0) {
            t0 = r;
          }
        } else {
          if (r <= t0) {
            return false;
          }
          if (r < t1) {
            t1 = r;
          }
        }
        return true;
      };
      if (clip(-dx, p1.x() - box.xMin()) && clip(dx, box.xMax() - p1.x())
          && clip(-dy, p1.y() - box.yMin()) && clip(dy, box.yMax() - p1.y())
          && t0 < t1) {
        if ((t1 - t0) * total_len >= no_pin_min_len) {
          return true;
        }
      }
    }
    return false;
  };

  std::function<bool(const BufferedNet*)> check
      = [&](const BufferedNet* node) -> bool {
    if (!node) {
      return false;
    }
    switch (node->type()) {
      case BufferedNetType::load:
        return false;
      case BufferedNetType::wire: {
        const BufferedNet* ref = node->ref().get();
        if (!ref) {
          return false;
        }
        if (seg_crosses(node->location(), ref->location())) {
          return true;
        }
        return check(ref);
      }
      case BufferedNetType::via:
        return check(node->ref().get());
      case BufferedNetType::buffer:
        if (isPointInPlacementBlockage(node->location())) {
          return true;
        }
        return check(node->ref().get());
      case BufferedNetType::junction:
        if (node->ref()
            && seg_crosses(node->location(), node->ref()->location())) {
          return true;
        }
        if (node->ref2()
            && seg_crosses(node->location(), node->ref2()->location())) {
          return true;
        }
        return check(node->ref().get()) || check(node->ref2().get());
    }
    return false;
  };
  return check(tree.get());
}

bool Resizer::treeTouchesPlacementBlockage(const BufferedNetPtr& tree) const
{
  if (!tree || placement_blockages_.empty()) {
    return false;
  }
  if (isPointInPlacementBlockage(tree->location())) {
    return true;
  }
  std::function<bool(const BufferedNet*)> check
      = [&](const BufferedNet* node) -> bool {
    if (!node) {
      return false;
    }
    switch (node->type()) {
      case BufferedNetType::load:
        return isPointInPlacementBlockage(node->location());
      case BufferedNetType::wire: {
        const BufferedNet* ref = node->ref().get();
        if (!ref) {
          return false;
        }
        if (segmentCrossesPlacementBlockage(node->location(),
                                            ref->location())) {
          return true;
        }
        if (ref->type() != BufferedNetType::load
            && isPointInPlacementBlockage(ref->location())) {
          return true;
        }
        return check(ref);
      }
      case BufferedNetType::via:
        return check(node->ref().get());
      case BufferedNetType::buffer:
        if (isPointInPlacementBlockage(node->location())) {
          return true;
        }
        return check(node->ref().get());
      case BufferedNetType::junction:
        if (isPointInPlacementBlockage(node->location())) {
          return true;
        }
        if (node->ref()
            && segmentCrossesPlacementBlockage(node->location(),
                                               node->ref()->location())) {
          return true;
        }
        if (node->ref2()
            && segmentCrossesPlacementBlockage(node->location(),
                                               node->ref2()->location())) {
          return true;
        }
        return check(node->ref().get()) || check(node->ref2().get());
    }
    return false;
  };
  return check(tree.get());
}

BufferedNetPtr Resizer::makeBufferedNetBlockageAwareSteinerOverBnets(
    odb::Point root,
    const std::vector<BufferedNetPtr>& sinks,
    const sta::Scene* scene)
{
  if (sinks.empty() || sinks.size() > 128) {
    return nullptr;
  }
  ensurePlacementBlockagesInitialized();
  if (placement_blockages_.empty()) {
    return nullptr;
  }

  const odb::Point root_access
      = isPointInPlacementBlockage(root)
            ? snapPointOutsidePlacementBlockage(root, root)
            : root;

  // Map each sink to its valid outside-blockage access point
  struct SinkAccessEntry
  {
    odb::Point access_pt;
    BufferedNetPtr bnet;
    int64_t dist_to_root = 0;
  };
  std::vector<SinkAccessEntry> sink_entries;
  sink_entries.reserve(sinks.size());

  odb::Rect net_bbox(root_access, root_access);
  for (const BufferedNetPtr& sink : sinks) {
    if (!sink) {
      continue;
    }
    const odb::Point pt = sink->location();
    const odb::Point acc
        = isPointInPlacementBlockage(pt)
              ? snapPointOutsidePlacementBlockage(pt, pt)
              : pt;
    BufferedNetPtr wrapped = sink;
    if (acc != pt) {
      wrapped = std::make_shared<BufferedNet>(BufferedNetType::wire,
                                              acc,
                                              BufferedNet::null_layer,
                                              sink,
                                              scene,
                                              this,
                                              estimate_parasitics_);
    }
    sink_entries.push_back(
        {acc, wrapped, odb::Point::manhattanDistance(root_access, acc)});
    net_bbox.merge(odb::Rect(acc, acc));
  }
  if (sink_entries.empty()) {
    return nullptr;
  }

  const int pad = std::max(placement_row_height_ / 2, 100);
  const int margin = std::max(placement_row_height_ * 40, 20000);
  odb::Rect search_bbox(net_bbox.xMin() - margin,
                        net_bbox.yMin() - margin,
                        net_bbox.xMax() + margin,
                        net_bbox.yMax() + margin);

  // Expand search_bbox to include the full closure of any intersecting blockages
  std::vector<bool> blk_included(placement_blockages_.size(), false);
  bool expanded = true;
  while (expanded) {
    expanded = false;
    for (size_t i = 0; i < placement_blockages_.size(); ++i) {
      if (blk_included[i]) {
        continue;
      }
      const odb::Rect& box = placement_blockages_[i];
      if (search_bbox.intersects(box)) {
        blk_included[i] = true;
        search_bbox.merge(odb::Rect(box.xMin() - 2 * pad,
                                    box.yMin() - 2 * pad,
                                    box.xMax() + 2 * pad,
                                    box.yMax() + 2 * pad));
        expanded = true;
      }
    }
  }

  auto build_on_hanan_grid = [&](bool use_all_blockages) -> BufferedNetPtr {
    std::vector<int> xs;
    std::vector<int> ys;
    xs.reserve(sink_entries.size() + placement_blockages_.size() * 2 + 2);
    ys.reserve(sink_entries.size() + placement_blockages_.size() * 2 + 2);

    xs.push_back(root_access.x());
    ys.push_back(root_access.y());
    for (const auto& se : sink_entries) {
      xs.push_back(se.access_pt.x());
      ys.push_back(se.access_pt.y());
    }
    for (size_t i = 0; i < placement_blockages_.size(); ++i) {
      if (!use_all_blockages && !blk_included[i]) {
        continue;
      }
      const odb::Rect& box = placement_blockages_[i];
      const int x0 = box.xMin() - pad;
      const int x1 = box.xMax() + pad;
      const int y0 = box.yMin() - pad;
      const int y1 = box.yMax() + pad;
      if (placement_core_area_.isInverted()
          || (x0 >= placement_core_area_.xMin()
              && x0 <= placement_core_area_.xMax())) {
        xs.push_back(x0);
      }
      if (placement_core_area_.isInverted()
          || (x1 >= placement_core_area_.xMin()
              && x1 <= placement_core_area_.xMax())) {
        xs.push_back(x1);
      }
      if (placement_core_area_.isInverted()
          || (y0 >= placement_core_area_.yMin()
              && y0 <= placement_core_area_.yMax())) {
        ys.push_back(y0);
      }
      if (placement_core_area_.isInverted()
          || (y1 >= placement_core_area_.yMin()
              && y1 <= placement_core_area_.yMax())) {
        ys.push_back(y1);
      }
    }

    std::sort(xs.begin(), xs.end());
    xs.erase(std::unique(xs.begin(), xs.end()), xs.end());
    std::sort(ys.begin(), ys.end());
    ys.erase(std::unique(ys.begin(), ys.end()), ys.end());

    const int nx = static_cast<int>(xs.size());
    const int ny = static_cast<int>(ys.size());
    const int tot = nx * ny;
    if (tot <= 0) {
      return nullptr;
    }

    auto find_idx = [](const std::vector<int>& arr, int val) -> int {
      auto it = std::lower_bound(arr.begin(), arr.end(), val);
      return static_cast<int>(std::distance(arr.begin(), it));
    };
    auto node_id = [ny](int ix, int iy) -> int { return ix * ny + iy; };
    auto grid_pt = [&xs, &ys, ny](int u) -> odb::Point {
      return odb::Point(xs[u / ny], ys[u % ny]);
    };

    const int root_id
        = node_id(find_idx(xs, root_access.x()), find_idx(ys, root_access.y()));

    std::vector<uint8_t> valid_node(tot, 0);
    for (int ix = 0; ix < nx; ++ix) {
      for (int iy = 0; iy < ny; ++iy) {
        const int u = node_id(ix, iy);
        if (!isPointInPlacementBlockage(odb::Point(xs[ix], ys[iy]))) {
          valid_node[u] = 1;
        }
      }
    }
    valid_node[root_id] = 1;

    std::vector<std::vector<BufferedNetPtr>> node_sinks(tot);
    struct TargetSink
    {
      int id;
      int64_t dist_to_root;
    };
    std::vector<TargetSink> targets;
    targets.reserve(sink_entries.size());
    for (const auto& se : sink_entries) {
      const int sid = node_id(find_idx(xs, se.access_pt.x()),
                              find_idx(ys, se.access_pt.y()));
      valid_node[sid] = 1;
      if (node_sinks[sid].empty() && sid != root_id) {
        targets.push_back({sid, se.dist_to_root});
      }
      node_sinks[sid].push_back(se.bnet);
    }

    // Sort targets from closest to root to farthest so trunk grows outward
    std::sort(targets.begin(),
              targets.end(),
              [](const TargetSink& a, const TargetSink& b) {
                return a.dist_to_root < b.dist_to_root;
              });

    // Precompute valid elementary horizontal and vertical Hanan grid segments
    std::vector<uint8_t> valid_h(tot, 0);
    std::vector<uint8_t> valid_v(tot, 0);
    for (int ix = 0; ix < nx; ++ix) {
      for (int iy = 0; iy < ny; ++iy) {
        const int u = node_id(ix, iy);
        if (!valid_node[u]) {
          continue;
        }
        const odb::Point p1(xs[ix], ys[iy]);
        if (ix + 1 < nx && valid_node[node_id(ix + 1, iy)]) {
          const odb::Point p2(xs[ix + 1], ys[iy]);
          const odb::Point mid((xs[ix] + xs[ix + 1]) / 2, ys[iy]);
          if (!isPointInPlacementBlockage(mid)
              && !segmentCrossesPlacementBlockage(p1, p2)) {
            valid_h[u] = 1;
          }
        }
        if (iy + 1 < ny && valid_node[node_id(ix, iy + 1)]) {
          const odb::Point p2(xs[ix], ys[iy + 1]);
          const odb::Point mid(xs[ix], (ys[iy] + ys[iy + 1]) / 2);
          if (!isPointInPlacementBlockage(mid)
              && !segmentCrossesPlacementBlockage(p1, p2)) {
            valid_v[u] = 1;
          }
        }
      }
    }

    // Multi-Pin Prim-Dijkstra / A* on the Hanan Grid
    std::vector<uint8_t> in_tree(tot, 0);
    std::vector<int> tree_parent(tot, -1);
    std::vector<std::vector<int>> tree_children(tot);
    std::vector<int64_t> root_dist(tot, 0);
    std::vector<int> tree_nodes_list;
    tree_nodes_list.reserve(tot);

    in_tree[root_id] = 1;
    tree_nodes_list.push_back(root_id);

    constexpr int64_t kInf = std::numeric_limits<int64_t>::max() / 4;
    std::vector<int64_t> dist(tot, kInf);
    std::vector<int> parent(tot, -1);
    using QItem = std::tuple<int64_t, int64_t, int>;

    for (const TargetSink& target : targets) {
      const int t_id = target.id;
      if (in_tree[t_id]) {
        continue;
      }
      const odb::Point t_pt = grid_pt(t_id);
      std::fill(dist.begin(), dist.end(), kInf);
      std::fill(parent.begin(), parent.end(), -1);

      std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
      for (int u : tree_nodes_list) {
        const int64_t g_u = root_dist[u] / 32;
        const int64_t h_u = odb::Point::manhattanDistance(grid_pt(u), t_pt);
        dist[u] = g_u;
        pq.emplace(g_u + h_u, g_u, u);
      }

      while (!pq.empty()) {
        const auto [f, d, u] = pq.top();
        pq.pop();
        if (u == t_id) {
          break;
        }
        if (d > dist[u]) {
          continue;
        }
        const int ix = u / ny;
        const int iy = u % ny;

        auto try_edge = [&](int v, int64_t w) {
          if (d + w < dist[v]) {
            dist[v] = d + w;
            parent[v] = u;
            const int64_t h_v = odb::Point::manhattanDistance(grid_pt(v), t_pt);
            pq.emplace(dist[v] + h_v, dist[v], v);
          }
        };

        if (ix + 1 < nx && valid_h[u]) {
          try_edge(u + ny, xs[ix + 1] - xs[ix]);
        }
        if (ix - 1 >= 0 && valid_h[u - ny]) {
          try_edge(u - ny, xs[ix] - xs[ix - 1]);
        }
        if (iy + 1 < ny && valid_v[u]) {
          try_edge(u + 1, ys[iy + 1] - ys[iy]);
        }
        if (iy - 1 >= 0 && valid_v[u - 1]) {
          try_edge(u - 1, ys[iy] - ys[iy - 1]);
        }
      }

      if (parent[t_id] == -1) {
        if (!use_all_blockages) {
          return nullptr;  // Retry with all blockages
        }
        // Fallback: direct connection from root_id so sink is never lost
        in_tree[t_id] = 1;
        tree_parent[t_id] = root_id;
        tree_children[root_id].push_back(t_id);
        root_dist[t_id]
            = odb::Point::manhattanDistance(root_access, grid_pt(t_id));
        tree_nodes_list.push_back(t_id);
      } else {
        std::vector<int> path;
        for (int cur = t_id; cur != -1 && !in_tree[cur]; cur = parent[cur]) {
          path.push_back(cur);
        }
        std::reverse(path.begin(), path.end());
        for (int v : path) {
          const int p = parent[v];
          in_tree[v] = 1;
          tree_parent[v] = p;
          tree_children[p].push_back(v);
          root_dist[v] = root_dist[p]
                         + odb::Point::manhattanDistance(grid_pt(p),
                                                         grid_pt(v));
          tree_nodes_list.push_back(v);
        }
      }
    }

    // Build BufferedNet tree from tree_children, contracting collinear degree-2
    // grid nodes
    std::function<BufferedNetPtr(int)> build_bnet
        = [&](int u) -> BufferedNetPtr {
      BufferedNetPtr bnet = nullptr;
      const odb::Point u_pt = grid_pt(u);
      for (const BufferedNetPtr& s : node_sinks[u]) {
        if (bnet) {
          bnet = std::make_shared<BufferedNet>(
              BufferedNetType::junction, u_pt, bnet, s, this);
        } else {
          bnet = s;
        }
      }
      for (int child : tree_children[u]) {
        int v = child;
        while (node_sinks[v].empty() && tree_children[v].size() == 1) {
          const int next_v = tree_children[v][0];
          const odb::Point next_pt = grid_pt(next_v);
          const bool collinear
              = (u_pt.x() == next_pt.x()) || (u_pt.y() == next_pt.y());
          if (!collinear || segmentCrossesPlacementBlockage(u_pt, next_pt)) {
            break;
          }
          v = next_v;
        }
        BufferedNetPtr child_bnet = build_bnet(v);
        if (!child_bnet) {
          continue;
        }
        if (child_bnet->location() != u_pt) {
          child_bnet = std::make_shared<BufferedNet>(BufferedNetType::wire,
                                                     u_pt,
                                                     BufferedNet::null_layer,
                                                     child_bnet,
                                                     scene,
                                                     this,
                                                     estimate_parasitics_);
        }
        if (bnet) {
          bnet = std::make_shared<BufferedNet>(
              BufferedNetType::junction, u_pt, bnet, child_bnet, this);
        } else {
          bnet = std::move(child_bnet);
        }
      }
      return bnet;
    };

    BufferedNetPtr result = build_bnet(root_id);
    if (result && result->location() != root) {
      result = std::make_shared<BufferedNet>(BufferedNetType::wire,
                                             root,
                                             BufferedNet::null_layer,
                                             result,
                                             scene,
                                             this,
                                             estimate_parasitics_);
    }
    return result;
  };

  BufferedNetPtr res = build_on_hanan_grid(false);
  if (!res) {
    res = build_on_hanan_grid(true);
  }
  return res;
}

static const BufferedNet* stripLeadingWires(const BufferedNet* node)
{
  while (node && node->type() == BufferedNetType::wire && node->ref()) {
    node = node->ref().get();
  }
  return node;
}

static bool hasDirectLoadAtLoc(const BufferedNet* node, const odb::Point& loc)
{
  if (!node) {
    return false;
  }
  if (node->type() == BufferedNetType::load) {
    return node->location() == loc;
  }
  if (node->type() == BufferedNetType::junction) {
    return hasDirectLoadAtLoc(node->ref().get(), loc)
           || hasDirectLoadAtLoc(node->ref2().get(), loc);
  }
  return false;
}

BufferedNetPtr Resizer::rerouteBufferedNetAroundBlockages(
    const BufferedNetPtr& tree,
    const sta::Scene* scene)
{
  if (!tree || !blockage_aware_steiner_) {
    return tree;
  }
  ensurePlacementBlockagesInitialized();
  if (placement_blockages_.empty() || !treeTouchesPlacementBlockage(tree)) {
    return tree;
  }

  // When an unbuffered tree genuinely crosses the interior of a placement
  // blockage (e.g. a straight MET5 route across a macro), rebuild its topology
  // with the Multi-Pin Hanan Grid A* obstacle-avoiding Steiner builder.
  if (treeCrossesPlacementBlockage(tree)) {
    bool has_buffer = false;
    std::vector<BufferedNetPtr> load_sinks;
    std::function<void(const BufferedNetPtr&)> collect_loads
        = [&](const BufferedNetPtr& node) {
            if (!node || has_buffer) {
              return;
            }
            switch (node->type()) {
              case BufferedNetType::load:
                load_sinks.push_back(node);
                break;
              case BufferedNetType::buffer:
                has_buffer = true;
                break;
              case BufferedNetType::wire:
              case BufferedNetType::via:
                collect_loads(node->ref());
                break;
              case BufferedNetType::junction:
                collect_loads(node->ref());
                collect_loads(node->ref2());
                break;
            }
          };
    collect_loads(tree);
    if (!has_buffer && !load_sinks.empty() && load_sinks.size() <= 128) {
      BufferedNetPtr astar_tree = makeBufferedNetBlockageAwareSteinerOverBnets(
          tree->location(), load_sinks, scene);
      if (astar_tree && !treeCrossesPlacementBlockage(astar_tree)) {
        return astar_tree;
      }
    }
  }

  const int pad = std::max(placement_row_height_ / 2, 100);

  auto choose_best_junc_loc = [&](const odb::Point& junc_loc,
                                  const odb::Point& parent_loc,
                                  const odb::Point& c1_loc,
                                  const odb::Point& c2_loc) -> odb::Point {
    odb::Rect hit_box;
    const bool in_blk = isPointInPlacementBlockage(junc_loc, &hit_box);
    const bool cross_p
        = !in_blk && segmentCrossesPlacementBlockage(parent_loc, junc_loc, &hit_box);
    const bool cross_c1
        = !in_blk && !cross_p
          && segmentCrossesPlacementBlockage(junc_loc, c1_loc, &hit_box);
    const bool cross_c2
        = !in_blk && !cross_p && !cross_c1
          && segmentCrossesPlacementBlockage(junc_loc, c2_loc, &hit_box);
    if (!in_blk && !cross_p && !cross_c1 && !cross_c2) {
      return junc_loc;
    }

    std::vector<odb::Point> cands;
    if (!in_blk) {
      cands.push_back(junc_loc);
    }
    const odb::Point box_cands[4] = {
        odb::Point(hit_box.xMin() - pad, junc_loc.y()),
        odb::Point(hit_box.xMax() + pad, junc_loc.y()),
        odb::Point(junc_loc.x(), hit_box.yMin() - pad),
        odb::Point(junc_loc.x(), hit_box.yMax() + pad),
    };
    for (const odb::Point& bc : box_cands) {
      if (!isPointInPlacementBlockage(bc)) {
        cands.push_back(bc);
      }
    }
    if (cands.empty()) {
      return snapPointOutsidePlacementBlockage(junc_loc, parent_loc);
    }

    const odb::Point p_ref
        = isPointInPlacementBlockage(parent_loc)
              ? snapPointOutsidePlacementBlockage(parent_loc, junc_loc)
              : parent_loc;
    const odb::Point c1_ref
        = isPointInPlacementBlockage(c1_loc)
              ? snapPointOutsidePlacementBlockage(c1_loc, junc_loc)
              : c1_loc;
    const odb::Point c2_ref
        = isPointInPlacementBlockage(c2_loc)
              ? snapPointOutsidePlacementBlockage(c2_loc, junc_loc)
              : c2_loc;

    odb::Point best_pt = cands.front();
    int64_t best_score = std::numeric_limits<int64_t>::max();
    for (const odb::Point& cand : cands) {
      int64_t score = odb::Point::manhattanDistance(p_ref, cand)
                      + odb::Point::manhattanDistance(cand, c1_ref)
                      + odb::Point::manhattanDistance(cand, c2_ref);
      if (!findBlockageFreeLPath(p_ref, cand, nullptr)) {
        score += 500000;
      }
      if (!findBlockageFreeLPath(cand, c1_ref, nullptr)) {
        score += 500000;
      }
      if (!findBlockageFreeLPath(cand, c2_ref, nullptr)) {
        score += 500000;
      }
      if (score < best_score) {
        best_score = score;
        best_pt = cand;
      }
    }
    return best_pt;
  };

  auto connect_detour = [&](const BufferedNetPtr& child,
                            const odb::Point& from_loc,
                            int orig_layer) -> BufferedNetPtr {
    if (!child || child->location() == from_loc) {
      return child;
    }
    const std::vector<odb::Point> pts
        = findBlockageDetourPath(from_loc, child->location());
    if (pts.size() <= 2 && pts.front() == from_loc
        && pts.back() == child->location()) {
      return std::make_shared<BufferedNet>(BufferedNetType::wire,
                                           from_loc,
                                           orig_layer,
                                           child,
                                           scene,
                                           this,
                                           estimate_parasitics_);
    }
    BufferedNetPtr cur = child;
    for (int k = static_cast<int>(pts.size()) - 2; k >= 0; --k) {
      if (pts[k] != cur->location()) {
        cur = std::make_shared<BufferedNet>(BufferedNetType::wire,
                                            pts[k],
                                            BufferedNet::null_layer,
                                            cur,
                                            scene,
                                            this,
                                            estimate_parasitics_);
      }
    }
    return cur;
  };

  std::function<BufferedNetPtr(
      const BufferedNetPtr&, const odb::Point&, int)>
      transform = [&](const BufferedNetPtr& node,
                      const odb::Point& parent_loc,
                      int level) -> BufferedNetPtr {
    if (!node) {
      return nullptr;
    }
    switch (node->type()) {
      case BufferedNetType::load:
        return node;
      case BufferedNetType::buffer: {
        odb::Point buf_loc = node->location();
        if (isPointInPlacementBlockage(buf_loc)) {
          buf_loc = snapPointOutsidePlacementBlockage(buf_loc, parent_loc);
        }
        BufferedNetPtr inner = transform(node->ref(), buf_loc, level + 1);
        if (!inner) {
          return nullptr;
        }
        inner = connect_detour(inner, buf_loc, BufferedNet::null_layer);
        return std::make_shared<BufferedNet>(BufferedNetType::buffer,
                                             buf_loc,
                                             node->bufferCell(),
                                             inner,
                                             scene,
                                             this,
                                             estimate_parasitics_);
      }
      case BufferedNetType::via: {
        BufferedNetPtr inner
            = transform(node->ref(), node->location(), level + 1);
        if (!inner) {
          return nullptr;
        }
        if (inner == node->ref() && inner->location() == node->location()) {
          return node;
        }
        return std::make_shared<BufferedNet>(BufferedNetType::via,
                                             inner->location(),
                                             node->layer(),
                                             node->refLayer(),
                                             inner,
                                             scene,
                                             this);
      }
      case BufferedNetType::wire: {
        const odb::Point wire_loc = node->location();
        if (level > 0 && isPointInPlacementBlockage(wire_loc)) {
          return transform(node->ref(), parent_loc, level + 1);
        }
        BufferedNetPtr inner = transform(node->ref(), wire_loc, level + 1);
        if (!inner) {
          return nullptr;
        }
        if (inner == node->ref() && !isPointInPlacementBlockage(wire_loc)
            && !isPointInPlacementBlockage(inner->location())
            && !segmentCrossesPlacementBlockage(wire_loc, inner->location())) {
          return node;
        }
        return connect_detour(inner, wire_loc, node->layer());
      }
      case BufferedNetType::junction: {
        const odb::Point orig_junc = node->location();
        odb::Point junc_loc = orig_junc;
        if (level > 0 && !hasDirectLoadAtLoc(node.get(), orig_junc)) {
          const BufferedNet* c1 = stripLeadingWires(node->ref().get());
          const BufferedNet* c2 = stripLeadingWires(node->ref2().get());
          const odb::Point c1_loc = c1 ? c1->location() : orig_junc;
          const odb::Point c2_loc = c2 ? c2->location() : orig_junc;
          junc_loc
              = choose_best_junc_loc(orig_junc, parent_loc, c1_loc, c2_loc);
        }

        auto process_branch
            = [&](BufferedNetPtr branch) -> BufferedNetPtr {
          int branch_layer = BufferedNet::null_layer;
          if (junc_loc != orig_junc && branch
              && branch->type() == BufferedNetType::wire
              && branch->location() == orig_junc && branch->ref()) {
            branch_layer = branch->layer();
            branch = branch->ref();
          }
          BufferedNetPtr new_branch = transform(branch, junc_loc, level + 1);
          if (!new_branch) {
            return nullptr;
          }
          if (new_branch->location() != junc_loc) {
            new_branch = connect_detour(new_branch, junc_loc, branch_layer);
          }
          return new_branch;
        };

        BufferedNetPtr left = process_branch(node->ref());
        BufferedNetPtr right = process_branch(node->ref2());
        if (!left || !right) {
          return nullptr;
        }
        if (junc_loc == orig_junc && left == node->ref()
            && right == node->ref2()) {
          return node;
        }
        return std::make_shared<BufferedNet>(
            BufferedNetType::junction, junc_loc, left, right, this);
      }
    }
    return node;
  };

  return transform(tree, tree->location(), 0);
}

}  // namespace rsz
