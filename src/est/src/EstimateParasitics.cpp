// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "est/EstimateParasitics.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <limits>
#include <map>
#include <memory>
#include <ostream>
#include <set>
#include <utility>
#include <vector>

#include "AbstractSteinerRenderer.h"
#include "MakeWireParasitics.h"
#include "OdbCallBack.h"
#include "db_sta/SpefWriter.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "est/ParasiticsService.h"
#include "est/SteinerTree.h"
#include "grt/GRoute.h"
#include "grt/GlobalRouter.h"
#include "odb/db.h"
#include "odb/dbSet.h"
#include "odb/dbShape.h"
#include "odb/dbTypes.h"
#include "odb/geom.h"
#include "sta/ArcDelayCalc.hh"
#include "sta/Liberty.hh"
#include "sta/MinMax.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/Parasitics.hh"
#include "sta/ParasiticsClass.hh"
#include "sta/Report.hh"
#include "sta/Scene.hh"
#include "sta/Sdc.hh"
#include "sta/Transition.hh"
#include "sta/Units.hh"
#include "stt/SteinerTreeBuilder.h"
#include "utl/Logger.h"
#include "utl/ServiceRegistry.h"

namespace est {

using utl::EST;

using sta::NetConnectedPinIterator;
using sta::NetIterator;
using sta::Parasitic;
using sta::Parasitics;
using sta::Pin;
using sta::PinSet;
using sta::Scene;

using odb::dbInst;
using odb::dbMasterType;
using odb::dbModInst;

EstimateParasitics::EstimateParasitics(utl::Logger* logger,
                                       utl::ServiceRegistry* service_registry,
                                       odb::dbDatabase* db,
                                       sta::dbSta* sta,
                                       stt::SteinerTreeBuilder* stt_builder,
                                       grt::GlobalRouter* global_router)
    : logger_(logger),
      service_registry_(service_registry),
      stt_builder_(stt_builder),
      global_router_(global_router),
      db_network_(sta->getDbNetwork()),
      db_(db),
      db_cbk_(std::make_unique<OdbCallBack>(this, network_, db_network_)),
      wire_signal_res_(0.0),
      wire_signal_cap_(0.0),
      wire_clk_res_(0.0),
      wire_clk_cap_(0.0)
{
  dbStaState::init(sta);
  db_cbk_ = std::make_unique<OdbCallBack>(this, network_, db_network_);
  service_registry_->provide<ParasiticsService>(this);
}

EstimateParasitics::~EstimateParasitics()
{
  service_registry_->withdraw<ParasiticsService>(this);
}

void EstimateParasitics::estimateAllGlobalRouteParasitics()
{
  clearParasitics();
  for (auto& [db_net, route] : global_router_->getPartialRoutes()) {
    estimateGlobalRouteParasitics(db_net, route);
  }
}

void EstimateParasitics::initSteinerRenderer(
    std::unique_ptr<est::AbstractSteinerRenderer> steiner_renderer)
{
  if (steiner_renderer_) {
    logger_->warn(EST, 4, "Steiner renderer already initialized.");
    return;
  }
  steiner_renderer_ = std::move(steiner_renderer);
}

////////////////////////////////////////////////////////////////

void EstimateParasitics::setLayerRC(odb::dbTechLayer* layer,
                                    const sta::Scene* corner,
                                    double res,
                                    double cap)
{
  if (layer_res_.empty()) {
    int layer_count = db_->getTech()->getLayerCount();
    int corner_count = sta_->scenes().size();
    layer_res_.resize(layer_count);
    layer_cap_.resize(layer_count);
    for (int i = 0; i < layer_count; i++) {
      layer_res_[i].resize(corner_count);
      layer_cap_[i].resize(corner_count);
    }
  }

  layer_res_[layer->getNumber()][corner->index()] = res;
  layer_cap_[layer->getNumber()][corner->index()] = cap;
}

void EstimateParasitics::layerRC(odb::dbTechLayer* layer,
                                 const sta::Scene* corner,
                                 // Return values.
                                 double& res,
                                 double& cap) const
{
  if (layer_res_.empty()) {
    res = 0.0;
    cap = 0.0;
  } else {
    const int layer_level = layer->getNumber();
    res = layer_res_[layer_level][corner->index()];
    cap = layer_cap_[layer_level][corner->index()];
  }
}

////////////////////////////////////////////////////////////////

void EstimateParasitics::addClkLayer(odb::dbTechLayer* layer)
{
  clk_layers_.push_back(layer);
}

void EstimateParasitics::addSignalLayer(odb::dbTechLayer* layer)
{
  signal_layers_.push_back(layer);
}

void EstimateParasitics::sortClkAndSignalLayers()
{
  auto sort_layers = [](const odb::dbTechLayer* a, const odb::dbTechLayer* b) {
    return a->getNumber() < b->getNumber();
  };

  std::ranges::sort(clk_layers_, sort_layers);
  std::ranges::sort(signal_layers_, sort_layers);
}

void EstimateParasitics::setHWireSignalRC(const sta::Scene* scene,
                                          double res,
                                          double cap)
{
  wire_signal_res_.resize(sta_->scenes().size());
  wire_signal_cap_.resize(sta_->scenes().size());
  wire_signal_res_[scene->index()].h_res = res;
  wire_signal_cap_[scene->index()].h_cap = cap;
}
void EstimateParasitics::setVWireSignalRC(const sta::Scene* scene,
                                          double res,
                                          double cap)
{
  wire_signal_res_.resize(sta_->scenes().size());
  wire_signal_cap_.resize(sta_->scenes().size());
  wire_signal_res_[scene->index()].v_res = res;
  wire_signal_cap_[scene->index()].v_cap = cap;
}

double EstimateParasitics::wireSignalResistance(const sta::Scene* scene) const
{
  if (wire_signal_res_.empty()) {
    return 0.0;
  }

  return (wire_signal_res_[scene->index()].h_res
          + wire_signal_res_[scene->index()].v_res)
         / 2;
}

double EstimateParasitics::wireSignalHResistance(const sta::Scene* scene) const
{
  if (wire_signal_res_.empty()) {
    return 0.0;
  }
  return wire_signal_res_[scene->index()].h_res;
}

double EstimateParasitics::wireSignalVResistance(const sta::Scene* scene) const
{
  if (wire_signal_res_.empty()) {
    return 0.0;
  }
  return wire_signal_res_[scene->index()].v_res;
}

double EstimateParasitics::wireSignalCapacitance(const sta::Scene* scene) const
{
  if (wire_signal_cap_.empty()) {
    return 0.0;
  }

  return (wire_signal_cap_[scene->index()].h_cap
          + wire_signal_cap_[scene->index()].v_cap)
         / 2;
}

double EstimateParasitics::wireSignalHCapacitance(const sta::Scene* scene) const
{
  if (wire_signal_cap_.empty()) {
    return 0.0;
  }
  return wire_signal_cap_[scene->index()].h_cap;
}

double EstimateParasitics::wireSignalVCapacitance(const sta::Scene* scene) const
{
  if (wire_signal_cap_.empty()) {
    return 0.0;
  }
  return wire_signal_cap_[scene->index()].v_cap;
}

void EstimateParasitics::wireSignalRC(const sta::Scene* scene,
                                      // Return values.
                                      double& res,
                                      double& cap) const
{
  if (wire_signal_res_.empty()) {
    res = 0.0;
  } else {
    auto resistance = wire_signal_res_[scene->index()];
    res = (resistance.h_res + resistance.v_res) / 2;
  }
  if (wire_signal_cap_.empty()) {
    cap = 0.0;
  } else {
    auto capacitance = wire_signal_cap_[scene->index()];
    cap = (capacitance.h_cap + capacitance.v_cap) / 2;
  }
}

void EstimateParasitics::setHWireClkRC(const sta::Scene* scene,
                                       double res,
                                       double cap)
{
  wire_clk_res_.resize(sta_->scenes().size());
  wire_clk_cap_.resize(sta_->scenes().size());
  wire_clk_res_[scene->index()].h_res = res;
  wire_clk_cap_[scene->index()].h_cap = cap;
}

void EstimateParasitics::setVWireClkRC(const sta::Scene* scene,
                                       double res,
                                       double cap)
{
  wire_clk_res_.resize(sta_->scenes().size());
  wire_clk_cap_.resize(sta_->scenes().size());
  wire_clk_res_[scene->index()].v_res = res;
  wire_clk_cap_[scene->index()].v_cap = cap;
}

double EstimateParasitics::wireClkResistance(const sta::Scene* scene) const
{
  if (wire_clk_res_.empty()) {
    return 0.0;
  }

  return (wire_clk_res_[scene->index()].h_res
          + wire_clk_res_[scene->index()].v_res)
         / 2;
}

double EstimateParasitics::wireClkHResistance(const sta::Scene* scene) const
{
  if (wire_clk_res_.empty()) {
    return 0.0;
  }

  return wire_clk_res_[scene->index()].h_res;
}

double EstimateParasitics::wireClkVResistance(const sta::Scene* scene) const
{
  if (wire_clk_res_.empty()) {
    return 0.0;
  }

  return wire_clk_res_[scene->index()].v_res;
}

double EstimateParasitics::wireClkCapacitance(const sta::Scene* scene) const
{
  if (wire_clk_cap_.empty()) {
    return 0.0;
  }

  return (wire_clk_cap_[scene->index()].h_cap
          + wire_clk_cap_[scene->index()].v_cap)
         / 2;
}

double EstimateParasitics::wireClkHCapacitance(const sta::Scene* scene) const
{
  if (wire_clk_cap_.empty()) {
    return 0.0;
  }

  return wire_clk_cap_[scene->index()].h_cap;
}

double EstimateParasitics::wireClkVCapacitance(const sta::Scene* scene) const
{
  if (wire_clk_cap_.empty()) {
    return 0.0;
  }

  return wire_clk_cap_[scene->index()].v_cap;
}

////////////////////////////////////////////////////////////////

void EstimateParasitics::setDbCbkOwner(odb::dbBlock* block)
{
  db_cbk_->addOwner(block);
}
void EstimateParasitics::removeDbCbkOwner()
{
  db_cbk_->removeOwner();
}

// block_ indicates core_, design_area_, db_network_ etc valid.
void EstimateParasitics::initBlock()
{
  if (db_->getChip() == nullptr) {
    logger_->error(EST, 162, "Database does not have a loaded design");
  }

  if (block_ == nullptr) {
    block_ = db_->getChip()->getBlock();
  }

  if (block_ == nullptr) {
    logger_->error(EST, 163, "Database has no block");
  }

  dbu_ = db_->getTech()->getDbUnitsPerMicron();
  if (!db_cbk_->hasOwner()) {
    db_cbk_->addOwner(block_);
  }
}

void EstimateParasitics::ensureParasitics()
{
  estimateParasitics(global_router_->haveRoutes()
                         ? ParasiticsSrc::kGlobalRouting
                         : ParasiticsSrc::kPlacement);
}

void EstimateParasitics::estimateParasitics(ParasiticsSrc src)
{
  std::map<sta::Scene*, std::ostream*> spef_streams;
  estimateParasitics(src, spef_streams);
}

void EstimateParasitics::estimateParasitics(
    ParasiticsSrc src,
    std::map<sta::Scene*, std::ostream*>& spef_streams)
{
  initBlock();
  std::unique_ptr<sta::SpefWriter> spef_writer;
  if (!spef_streams.empty()) {
    spef_writer
        = std::make_unique<sta::SpefWriter>(logger_, sta_, spef_streams);
  }

  switch (src) {
    case ParasiticsSrc::kPlacement:
      estimateWireParasitics(spef_writer.get());
      parasitics_src_ = ParasiticsSrc::kPlacement;
      break;
    case ParasiticsSrc::kGlobalRouting:
      estimateGlobalRouteRC(spef_writer.get());
      parasitics_src_ = ParasiticsSrc::kGlobalRouting;
      break;
    case ParasiticsSrc::kDetailedRouting:
      // TODO: call rcx to extract parasitics and load them to STA
      parasitics_src_ = ParasiticsSrc::kDetailedRouting;
      break;
    case ParasiticsSrc::kNone:
      break;
  }
}

bool EstimateParasitics::haveEstimatedParasitics() const
{
  return parasitics_src_ != ParasiticsSrc::kNone;
}

void EstimateParasitics::updateParasitics()
{
  if (!incremental_parasitics_enabled_) {
    logger_->error(
        EST,
        109,
        "updateParasitics() called with incremental parasitics disabled");
  }

  // Call clearNetDrvrPinMap only without full blown ConcreteNetwork::clear()
  // This is because netlist changes may invalidate cached net driver pin data
  sta::LibertyLibrary* default_lib = network_->defaultLibertyLibrary();
  network_->sta::Network::clear();
  network_->setDefaultLibertyLibrary(default_lib);

  switch (parasitics_src_) {
    case ParasiticsSrc::kPlacement:
      for (const sta::Net* net : parasitics_invalid_) {
        if (isIdealClockNet(net)) {
          continue;
        }
        //
        // TODO: remove this check (we expect all to be flat net)
        //
        if (!(db_network_->isFlat(net))) {
          debugPrint(logger_,
                     EST,
                     "estimate_parasitics",
                     1,
                     "non-flat net {} is skipped",
                     sdc_network_->pathName(net));
          continue;
        }
        debugPrint(logger_,
                   EST,
                   "estimate_parasitics",
                   1,
                   "net {} para is estimated for placement",
                   sdc_network_->pathName(net));
        estimateWireParasitic(net);
      }
      break;
    case ParasiticsSrc::kGlobalRouting:
    case ParasiticsSrc::kDetailedRouting: {
      // TODO: update detailed route for modified nets
      incr_groute_->updateRoutes();
      for (const sta::Net* net : parasitics_invalid_) {
        if (isIdealClockNet(net)) {
          continue;
        }
        debugPrint(logger_,
                   EST,
                   "estimate_parasitics",
                   1,
                   "net {} para is estimated for GR or DR",
                   sdc_network_->pathName(net));
        estimateGlobalRouteRC(db_network_->staToDb(net));
      }
      break;
    }
    case ParasiticsSrc::kNone:
      break;
  }

  // Router calls into the timer. This means the timer could be caching
  // delays calculated in the interim period before we had put new parasitic
  // annotations on the nets affected by a network edit. We need to explicitly
  // invalidate those delays. Do it in bulk instead of interleaving with each
  // groute call.
  if (parasitics_src_ != ParasiticsSrc::kNone) {
    for (const sta::Net* net : parasitics_invalid_) {
      if (isIdealClockNet(net)) {
        continue;
      }
      debugPrint(logger_,
                 EST,
                 "estimate_parasitics",
                 1,
                 "net {} delays from fanin invalidated",
                 sdc_network_->pathName(net));
      sta_->delaysInvalidFromFanin(net);
    }
  }
  parasitics_invalid_.clear();
}

bool EstimateParasitics::parasiticsValid() const
{
  return parasitics_invalid_.empty();
}

void EstimateParasitics::ensureWireParasitic(const sta::Pin* drvr_pin)
{
  const sta::Net* net = db_network_->dbToSta(db_network_->flatNet(drvr_pin));

  if (net) {
    ensureWireParasitic(drvr_pin, net);
  }
}

void EstimateParasitics::ensureWireParasitic(const sta::Pin* drvr_pin,
                                             const sta::Net* net)
{
  // Sufficient to check for parasitic for one corner because
  // they are all made at the same time.
  const Scene* corner = sta_->scenes().front();
  Parasitics* parasitics = corner->parasitics(max_);
  if (!parasitics_invalid_.contains(net)
      && (isSpefNet(net) || parasitics->findParasiticNetwork(net) != nullptr)) {
    if (parasitics->findPiElmore(drvr_pin, sta::RiseFall::rise(), max_)
            == nullptr
        && arc_delay_calc_->reduceSupported()) {
      for (Scene* scene : sta_->scenes()) {
        for (const sta::MinMax* min_max : sta::MinMax::range()) {
          Parasitics* scene_parasitics = scene->parasitics(min_max);
          if (scene_parasitics == nullptr) {
            continue;
          }
          sta::Parasitic* network
              = scene_parasitics->findParasiticNetwork(net);
          if (network != nullptr) {
            scene_parasitics->reduceToPiElmore(network,
                                               drvr_pin,
                                               sta::RiseFall::rise(),
                                               scene,
                                               min_max);
            scene_parasitics->reduceToPiElmore(network,
                                               drvr_pin,
                                               sta::RiseFall::fall(),
                                               scene,
                                               min_max);
          }
        }
      }
    }
    return;
  }
  if (parasitics_invalid_.contains(net)
      || parasitics->findPiElmore(drvr_pin, sta::RiseFall::rise(), max_)
             == nullptr) {
    switch (parasitics_src_) {
      case ParasiticsSrc::kPlacement:
        estimateWireParasitic(drvr_pin, net);
        parasitics_invalid_.erase(net);
        break;
      case ParasiticsSrc::kGlobalRouting: {
        incr_groute_->updateRoutes();
        estimateGlobalRouteRC(db_network_->staToDb(net));
        parasitics_invalid_.erase(net);
        break;
      }
      case ParasiticsSrc::kDetailedRouting:
        // TODO: call incremental drt for the modified net
        break;
      case ParasiticsSrc::kNone:
        break;
    }
  }
}

void EstimateParasitics::estimateGlobalRouteRC(sta::SpefWriter* spef_writer)
{
  if (spef_writer == nullptr && hasSpefNets()) {
    refreshNonSpefParasitics(true);
    return;
  }

  sta_->deleteParasitics();
  for (Scene* scene : scenes_) {
    Parasitics* parasitics = sta_->makeConcreteParasitics(scene->name(), "");
    scene->setParasitics(parasitics, sta::MinMaxAll::minMax());
  }

  MakeWireParasitics builder(
      logger_, this, sta_, db_->getTech(), block_, global_router_);

  for (auto& [db_net, route] : global_router_->getRoutes()) {
    if (!route.empty()) {
      builder.estimateParasitics(db_net, route, spef_writer);
    }
  }
}

void EstimateParasitics::estimateGlobalRouteRC(odb::dbNet* db_net)
{
  MakeWireParasitics builder(
      logger_, this, sta_, db_->getTech(), block_, global_router_);
  auto& routes = global_router_->getRoutes();
  auto iter = routes.find(db_net);
  if (iter == routes.end()) {
    return;
  }
  grt::GRoute& route = iter->second;
  if (!route.empty()) {
    builder.estimateParasitics(db_net, route, nullptr);
  }
}

void EstimateParasitics::estimateGlobalRouteParasitics(odb::dbNet* net,
                                                       grt::GRoute& route)
{
  initBlock();
  MakeWireParasitics builder(
      logger_, this, sta_, db_->getTech(), block_, global_router_);

  // Check if we are estimating parasitics after layer assignment
  if (route.at(0).is3DRoute()) {
    builder.estimateParasitics(net, route, nullptr);
  } else {
    builder.estimateParasitics(net, route);
  }
}

void EstimateParasitics::clearParasitics()
{
  MakeWireParasitics builder(
      logger_, this, sta_, db_->getTech(), block_, global_router_);
  builder.clearParasitics();
}

void EstimateParasitics::clearSpefNet(const sta::Net* net)
{
  if (net != nullptr) {
    if (spef_nets_.contains(net)) {
      ensureNetRcScale(net);
      spef_nets_.erase(net);
    }
  }
}

void EstimateParasitics::clearSpefNet(const odb::dbNet* net)
{
  if (net != nullptr) {
    clearSpefNet(db_network_->dbToSta(net));
  }
}

bool EstimateParasitics::hasNetRcScale(const sta::Net* net) const
{
  if (net == nullptr) {
    return false;
  }
  auto it = net_rc_scales_.find(net);
  return it != net_rc_scales_.end() && it->second.valid;
}

void EstimateParasitics::setNetRcScale(const sta::Net* net,
                                       const NetRcScale& scale)
{
  if (net != nullptr) {
    net_rc_scales_[net] = scale;
  }
}

void EstimateParasitics::copyNetRcScale(const sta::Net* src_net,
                                        const sta::Net* dst_net)
{
  if (src_net == nullptr || dst_net == nullptr || src_net == dst_net) {
    return;
  }
  NetRcScale scale = ensureNetRcScale(src_net);
  if (scale.valid) {
    net_rc_scales_[dst_net] = scale;
  }
}

double EstimateParasitics::computeViaStackResistance(
    odb::dbTechLayer* pin_layer,
    odb::dbTechLayer* tree_layer,
    const sta::Scene* corner) const
{
  if (pin_layer == nullptr || tree_layer == nullptr || layer_res_.empty()) {
    return 0.0;
  }
  const int pin_layer_idx = pin_layer->getNumber();
  const int tree_layer_idx = tree_layer->getNumber();
  if (pin_layer_idx == tree_layer_idx) {
    return 1.0e-3;
  }
  if (std::abs(pin_layer_idx - tree_layer_idx) == 2) {
    const int cut_layer_idx = pin_layer_idx < tree_layer_idx
                                  ? pin_layer_idx + 1
                                  : pin_layer_idx - 1;
    return std::max<double>(layer_res_[cut_layer_idx][corner->index()], 1.0e-3);
  }
  const auto [start_idx, end_idx] = std::minmax(pin_layer_idx, tree_layer_idx);
  double total_res = 0.0;
  for (int layer_idx = start_idx; layer_idx < end_idx; layer_idx++) {
    odb::dbTechLayer* cut_layer = db_->getTech()->findLayer(layer_idx);
    if (cut_layer && cut_layer->getType() == odb::dbTechLayerType::CUT) {
      total_res
          += std::max<double>(layer_res_[layer_idx][corner->index()], 1.0e-3);
    }
  }
  return total_res;
}

NetRcScale EstimateParasitics::ensureNetRcScale(const sta::Net* net)
{
  if (net == nullptr) {
    return {};
  }
  auto it = net_rc_scales_.find(net);
  if (it != net_rc_scales_.end() && it->second.valid) {
    return it->second;
  }

  auto base_it = spef_net_baselines_.find(net);
  if (base_it == spef_net_baselines_.end()) {
    NetRcScale fallback;
    if (hasSpefCalibration()) {
      fallback.cap_scale = default_unit_cap_scale_;
      fallback.res_scale = default_unit_res_scale_;
      fallback.unit_cap_scale = default_unit_cap_scale_;
      fallback.unit_res_scale = default_unit_res_scale_;
      fallback.detour_ratio = 1.0f;
      fallback.valid = true;
    }
    return fallback;
  }

  const float c_spef = base_it->second.c_spef;
  const float r_spef = base_it->second.r_spef;

  NetRcScale scale;
  scale.cap_scale = default_unit_cap_scale_;
  scale.res_scale = default_unit_res_scale_;
  scale.unit_cap_scale = default_unit_cap_scale_;
  scale.unit_res_scale = default_unit_res_scale_;
  scale.detour_ratio = 1.0f;
  scale.valid = true;

  sta::PinSet* drivers = network_->drivers(net);
  if (drivers == nullptr || drivers->empty() || sta_->scenes().empty()) {
    net_rc_scales_[net] = scale;
    return scale;
  }

  const sta::Pin* drvr_pin = *drivers->begin();
  if (isIdealClockPin(drvr_pin)) {
    net_rc_scales_[net] = scale;
    return scale;
  }

  SteinerTree* tree = makeSteinerTree(drvr_pin);
  if (tree == nullptr) {
    net_rc_scales_[net] = scale;
    return scale;
  }

  sta::Scene* corner = sta_->scenes().front();
  odb::dbNet* db_net = db_network_->staToDb(net);
  const bool is_clk
      = db_net != nullptr && global_router_->isNonLeafClock(db_net);
  odb::dbTechNonDefaultRule* ndr
      = db_net != nullptr ? db_net->getNonDefaultRule() : nullptr;
  float ndr_ratio = 1.0f;
  if (ndr) {
    std::vector<odb::dbTechLayerRule*> layer_rules;
    ndr->getLayerRules(layer_rules);
    if (!layer_rules.empty() && layer_rules.at(0)->getLayer()->getWidth() > 0) {
      ndr_ratio = static_cast<float>(layer_rules.at(0)->getWidth())
                  / layer_rules.at(0)->getLayer()->getWidth();
    }
  }

  double c_steiner_lef = 0.0;
  double r_steiner_wire_lef = 0.0;
  double r_steiner_via_lef = 0.0;
  int steiner_wl_dbu = 0;

  odb::dbTechLayer* tree_layer = nullptr;
  if (is_clk) {
    tree_layer = clk_layers_.empty() ? nullptr : clk_layers_[0];
  } else {
    tree_layer = signal_layers_.empty() ? nullptr : signal_layers_[0];
  }
  std::set<const sta::Pin*> connected_pins;

  const int branch_count = tree->branchCount();
  for (int i = 0; i < branch_count; i++) {
    odb::Point pt1, pt2;
    SteinerPt steiner_pt1, steiner_pt2;
    int wire_length_dbu = 0;
    tree->branch(i, pt1, steiner_pt1, pt2, steiner_pt2, wire_length_dbu);
    if (wire_length_dbu > 0) {
      steiner_wl_dbu += wire_length_dbu;
      const double dx = dbuToMeters(std::abs(pt1.x() - pt2.x()))
                        / dbuToMeters(wire_length_dbu);
      const double dy = dbuToMeters(std::abs(pt1.y() - pt2.y()))
                        / dbuToMeters(wire_length_dbu);
      double unit_cap = 0.0;
      double unit_res = 0.0;
      if (is_clk) {
        unit_cap = dx * wireClkHCapacitance(corner)
                   + dy * wireClkVCapacitance(corner);
        unit_res
            = dx * wireClkHResistance(corner) + dy * wireClkVResistance(corner);
      } else {
        unit_cap = dx * wireSignalHCapacitance(corner)
                   + dy * wireSignalVCapacitance(corner);
        unit_res = dx * wireSignalHResistance(corner)
                   + dy * wireSignalVResistance(corner);
      }
      const double length = dbuToMeters(wire_length_dbu);
      c_steiner_lef += length * unit_cap;
      double branch_res = length * unit_res;
      if (ndr_ratio > 1.0f) {
        branch_res /= ndr_ratio;
      }
      r_steiner_wire_lef += branch_res;
    }

    for (SteinerPt pt : {steiner_pt1, steiner_pt2}) {
      const sta::PinSeq* pins = tree->pins(pt);
      if (pins) {
        for (const sta::Pin* pin : *pins) {
          if (connected_pins.insert(pin).second) {
            if (tree_layer != nullptr && !layer_res_.empty()) {
              odb::dbTechLayer* pin_layer = getPinLayer(pin);
              r_steiner_via_lef
                  += computeViaStackResistance(pin_layer, tree_layer, corner);
            } else {
              r_steiner_via_lef
                  += std::max(computeAverageCutResistance(corner), 1.0e-3);
            }
          }
        }
      }
    }
  }
  delete tree;

  scale.orig_steiner_wl_dbu = steiner_wl_dbu;
  int routed_wl_dbu = steiner_wl_dbu;
  if (db_net != nullptr && db_net->getWire() != nullptr) {
    const int dbwire_wl = static_cast<int>(db_net->getWire()->getLength());
    if (dbwire_wl > 0) {
      routed_wl_dbu = dbwire_wl;
    }
  }

  const float ref_wl_dbu = std::max(1.0f, 5.0f * static_cast<float>(dbu_));
  const float length_weight = std::clamp(
      static_cast<float>(routed_wl_dbu) / ref_wl_dbu, 0.0f, 1.0f);

  double wl_ratio = 1.0;
  if (steiner_wl_dbu > 0 && routed_wl_dbu > steiner_wl_dbu) {
    wl_ratio = static_cast<double>(routed_wl_dbu)
               / static_cast<double>(steiner_wl_dbu);
    const float raw_detour
        = std::clamp(static_cast<float>(wl_ratio), 1.0f, 1.2f);
    scale.detour_ratio
        = std::clamp(1.0f + length_weight * (raw_detour - 1.0f), 1.0f, 1.2f);
  } else {
    scale.detour_ratio = 1.0f;
  }

  if (c_steiner_lef > 1.0e-18 && c_spef > 0.0f) {
    const float raw_unit_cap
        = static_cast<float>(c_spef / (c_steiner_lef * wl_ratio));
    scale.unit_cap_scale = std::clamp(
        length_weight * raw_unit_cap
            + (1.0f - length_weight) * default_unit_cap_scale_,
        0.7f,
        2.5f);
    scale.cap_scale = std::clamp(
        scale.unit_cap_scale * scale.detour_ratio, 0.85f, 5.0f);
  }

  const double r_steiner_total_lef = r_steiner_wire_lef + r_steiner_via_lef;
  if (r_steiner_total_lef > 1.0e-3 && r_spef > 0.0f) {
    const float raw_res_scale
        = static_cast<float>(r_spef / r_steiner_total_lef);
    const double r_routed_total_lef
        = r_steiner_wire_lef * wl_ratio + r_steiner_via_lef;
    const float raw_unit_res
        = r_routed_total_lef > 1.0e-3
              ? static_cast<float>(r_spef / r_routed_total_lef)
              : raw_res_scale;
    scale.unit_res_scale = std::clamp(
        length_weight * raw_unit_res
            + (1.0f - length_weight) * default_unit_res_scale_,
        0.7f,
        2.5f);
    scale.res_scale = std::clamp(
        scale.unit_res_scale * scale.detour_ratio, 0.85f, 5.0f);
  }

  net_rc_scales_[net] = scale;
  return scale;
}

NetRcScale EstimateParasitics::netRcScale(const sta::Net* net) const
{
  if (net != nullptr) {
    auto it = net_rc_scales_.find(net);
    if (it != net_rc_scales_.end() && it->second.valid) {
      return it->second;
    }
  }
  NetRcScale fallback;
  if (hasSpefCalibration()) {
    fallback.cap_scale = default_unit_cap_scale_;
    fallback.res_scale = default_unit_res_scale_;
    fallback.unit_cap_scale = default_unit_cap_scale_;
    fallback.unit_res_scale = default_unit_res_scale_;
    fallback.detour_ratio = 1.0f;
    fallback.valid = true;
  }
  return fallback;
}

NetRcScale EstimateParasitics::calibrateTreeNets(
    const sta::Net* root_net,
    const std::vector<const sta::Net*>& tree_nets,
    int unbuffered_steiner_wl_dbu)
{
  if (root_net == nullptr || tree_nets.empty() || !hasSpefCalibration()) {
    return {};
  }
  if (tree_nets.size() == 1 && tree_nets.front() == root_net) {
    return ensureNetRcScale(root_net);
  }

  double sum_unit_cap_weighted = 0.0;
  double sum_unit_res_weighted = 0.0;
  double sum_routed_wl = 0.0;
  double sum_steiner_wl = 0.0;

  for (const sta::Net* tnet : tree_nets) {
    if (tnet == nullptr) {
      continue;
    }
    const NetRcScale s = ensureNetRcScale(tnet);
    const double net_steiner_wl = std::max(s.orig_steiner_wl_dbu, dbu_);
    const double net_routed_wl = net_steiner_wl * s.detour_ratio;
    sum_unit_cap_weighted += s.unit_cap_scale * net_routed_wl;
    sum_unit_res_weighted += s.unit_res_scale * net_routed_wl;
    sum_routed_wl += net_routed_wl;
    sum_steiner_wl += net_steiner_wl;
  }

  if (sum_routed_wl <= 0.0) {
    return ensureNetRcScale(root_net);
  }

  NetRcScale agg = ensureNetRcScale(root_net);
  agg.unit_cap_scale = std::clamp(
      static_cast<float>(sum_unit_cap_weighted / sum_routed_wl), 0.7f, 2.5f);
  agg.unit_res_scale = std::clamp(
      static_cast<float>(sum_unit_res_weighted / sum_routed_wl), 0.7f, 2.5f);
  const int ref_steiner_wl
      = unbuffered_steiner_wl_dbu > 0
            ? unbuffered_steiner_wl_dbu
            : static_cast<int>(sum_steiner_wl);
  agg.orig_steiner_wl_dbu = ref_steiner_wl;
  if (ref_steiner_wl > 0 && sum_routed_wl > ref_steiner_wl) {
    agg.detour_ratio = std::clamp(
        static_cast<float>(sum_routed_wl / ref_steiner_wl), 1.0f, 1.2f);
  } else {
    agg.detour_ratio = 1.0f;
  }
  agg.cap_scale
      = std::clamp(agg.unit_cap_scale * agg.detour_ratio, 0.85f, 5.0f);
  agg.res_scale
      = std::clamp(agg.unit_res_scale * agg.detour_ratio, 0.85f, 5.0f);
  agg.valid = true;
  return agg;
}

void EstimateParasitics::spefNetMasterSwapped(const sta::Net* net)
{
  if (net == nullptr || !isSpefNet(net)) {
    return;
  }
  if (arc_delay_calc_->reduceSupported()) {
    sta::PinSet* drivers = network_->drivers(net);
    if (drivers && !drivers->empty()) {
      for (const sta::Pin* drvr_pin : *drivers) {
        for (Scene* scene : sta_->scenes()) {
          for (const sta::MinMax* min_max : sta::MinMax::range()) {
            Parasitics* parasitics = scene->parasitics(min_max);
            if (parasitics == nullptr) {
              continue;
            }
            sta::Parasitic* pnet = parasitics->findParasiticNetwork(net);
            if (pnet != nullptr) {
              parasitics->deleteReducedParasitics(net);
              parasitics->reduceToPiElmore(pnet,
                                           drvr_pin,
                                           sta::RiseFall::rise(),
                                           scene,
                                           min_max);
              parasitics->reduceToPiElmore(pnet,
                                           drvr_pin,
                                           sta::RiseFall::fall(),
                                           scene,
                                           min_max);
            }
          }
        }
      }
    }
  }
  sta_->delaysInvalidFromFanin(net);
}

void EstimateParasitics::initSpefParasitics()
{
  initBlock();
  for (auto mode : sta_->modes()) {
    sta_->ensureClkNetwork(mode);
  }
  for (Scene* scene : scenes_) {
    if (scene->parasitics(max_) == nullptr) {
      Parasitics* parasitics = sta_->makeConcreteParasitics(scene->name(), "");
      scene->setParasitics(parasitics, sta::MinMaxAll::minMax());
    }
  }
  sortClkAndSignalLayers();

  spef_nets_.clear();
  spef_net_baselines_.clear();
  net_rc_scales_.clear();
  default_unit_cap_scale_ = 1.0;
  default_unit_res_scale_ = 1.0;
  parasitics_invalid_.clear();
  const Scene* corner = sta_->scenes().front();
  Parasitics* parasitics = corner->parasitics(max_);
  const double unit_cap_lef
      = !wire_signal_cap_.empty() ? wireSignalCapacitance(corner) : 0.0;
  const double unit_res_lef
      = !wire_signal_res_.empty() ? wireSignalResistance(corner) : 0.0;
  odb::dbTechLayer* pin_layer
      = db_->getTech() ? db_->getTech()->findRoutingLayer(1) : nullptr;
  odb::dbTechLayer* tree_layer
      = signal_layers_.empty() ? nullptr : signal_layers_[0];
  const double via_stack_res
      = computeViaStackResistance(pin_layer, tree_layer, corner);
  double sum_c_spef = 0.0;
  double sum_c_lef = 0.0;
  double sum_r_spef = 0.0;
  double sum_r_lef = 0.0;
  int calibrated_samples = 0;

  odb::dbSet<odb::dbNet> nets = block_->getNets();
  for (auto db_net : nets) {
    sta::Net* cur_net = db_network_->dbToSta(db_net);
    if (cur_net == nullptr) {
      continue;
    }
    sta::Parasitic* pn
        = (parasitics != nullptr) ? parasitics->findParasiticNetwork(cur_net)
                                  : nullptr;
    bool has_spef = (pn != nullptr);
    double c_spef = 0.0;
    double r_spef = 0.0;
    if (pn != nullptr) {
      c_spef = parasitics->capacitance(pn);
      for (sta::ParasiticResistor* res : parasitics->resistors(pn)) {
        r_spef += parasitics->value(res);
      }
    } else if (parasitics != nullptr) {
      sta::PinSet* drivers = network_->drivers(cur_net);
      if (drivers && !drivers->empty()) {
        const sta::Pin* drvr_pin = *drivers->begin();
        sta::Parasitic* pi_elmore
            = parasitics->findPiElmore(drvr_pin, sta::RiseFall::rise(), max_);
        if (pi_elmore != nullptr) {
          has_spef = true;
          c_spef = parasitics->capacitance(pi_elmore);
          float c2 = 0.0f, rpi = 0.0f, c1 = 0.0f;
          parasitics->piModel(pi_elmore, c2, rpi, c1);
          r_spef = rpi;
        }
      }
    }
    if (has_spef) {
      spef_nets_.insert(cur_net);
      spef_net_baselines_[cur_net]
          = {static_cast<float>(c_spef), static_cast<float>(r_spef)};
      if (odb::dbWire* wire = db_net->getWire()) {
        const double wl_m = dbuToMeters(wire->getLength());
        if (wl_m >= 5e-6 && unit_cap_lef > 0.0 && c_spef > 0.0) {
          const double c_lef = wl_m * unit_cap_lef;
          const double ratio_c = c_spef / c_lef;
          if (ratio_c >= 0.5 && ratio_c <= 8.0) {
            sum_c_spef += c_spef;
            sum_c_lef += c_lef;
            calibrated_samples++;
          }
          if (unit_res_lef > 0.0 && r_spef > 0.0) {
            const size_t pin_count
                = db_net->getITerms().size() + db_net->getBTerms().size();
            const double r_lef = wl_m * unit_res_lef + pin_count * via_stack_res;
            if (r_lef > 0.0) {
              const double ratio_r = r_spef / r_lef;
              if (ratio_r >= 0.25 && ratio_r <= 8.0) {
                sum_r_spef += r_spef;
                sum_r_lef += r_lef;
              }
            }
          }
        }
      }
    } else if (!wire_signal_cap_.empty()) {
      estimateWireParasitic(cur_net, nullptr);
    }
  }
  if (sum_c_lef > 0.0 && calibrated_samples >= 10) {
    default_unit_cap_scale_ = std::clamp(sum_c_spef / sum_c_lef, 0.7, 2.0);
  }
  if (sum_r_lef > 0.0 && calibrated_samples >= 10) {
    default_unit_res_scale_ = std::clamp(sum_r_spef / sum_r_lef, 0.7, 2.0);
  }
  logger_->info(EST,
                200,
                "Initialized SPEF parasitics on {} nets ({} total nets, "
                "default SPEF/LEF unit scales: cap={:.3f}x, res={:.3f}x from {} nets).",
                spef_nets_.size(),
                nets.size(),
                default_unit_cap_scale_,
                default_unit_res_scale_,
                calibrated_samples);
  parasitics_src_ = ParasiticsSrc::kPlacement;
  setDbCbkOwner(block_);
}

void EstimateParasitics::refreshNonSpefParasitics(bool use_grt)
{
  initBlock();
  for (auto mode : sta_->modes()) {
    sta_->ensureClkNetwork(mode);
  }
  sta::LibertyLibrary* default_lib = network_->defaultLibertyLibrary();
  network_->sta::Network::clear();
  network_->setDefaultLibertyLibrary(default_lib);

  if (use_grt) {
    MakeWireParasitics builder(
        logger_, this, sta_, db_->getTech(), block_, global_router_);
    for (auto& [db_net, route] : global_router_->getRoutes()) {
      sta::Net* cur_net = db_network_->dbToSta(db_net);
      if (cur_net != nullptr && isSpefNet(cur_net)) {
        continue;
      }
      if (!route.empty()) {
        builder.estimateParasitics(db_net, route, nullptr);
        if (cur_net != nullptr && !isIdealClockNet(cur_net)) {
          sta_->delaysInvalidFromFanin(cur_net);
        }
      }
    }
    parasitics_src_ = ParasiticsSrc::kGlobalRouting;
  } else if (!wire_signal_cap_.empty()) {
    sortClkAndSignalLayers();
    odb::dbSet<odb::dbNet> nets = block_->getNets();
    for (auto db_net : nets) {
      sta::Net* cur_net = db_network_->dbToSta(db_net);
      if (cur_net == nullptr || isSpefNet(cur_net)) {
        continue;
      }
      estimateWireParasitic(cur_net, nullptr);
      if (!isIdealClockNet(cur_net)) {
        sta_->delaysInvalidFromFanin(cur_net);
      }
    }
    parasitics_src_ = ParasiticsSrc::kPlacement;
  }
  parasitics_invalid_.clear();
  setDbCbkOwner(block_);
}

////////////////////////////////////////////////////////////////

void EstimateParasitics::estimateWireParasitics(sta::SpefWriter* spef_writer)
{
  if (spef_writer == nullptr && hasSpefNets()) {
    refreshNonSpefParasitics(false);
    return;
  }
  initBlock();
  if (!wire_signal_cap_.empty()) {
    for (auto mode : sta_->modes()) {
      sta_->ensureClkNetwork(mode);
    }
    // Make separate parasitics for each corner
    sta_->deleteParasitics();
    for (Scene* scene : scenes_) {
      Parasitics* parasitics = sta_->makeConcreteParasitics(scene->name(), "");
      scene->setParasitics(parasitics, sta::MinMaxAll::minMax());
    }

    sta::LibertyLibrary* default_lib = network_->defaultLibertyLibrary();
    // Call clearNetDrvrPinMap only without full blown ConcreteNetwork::clear()
    // This is because netlist changes may invalidate cached net driver pin data
    network_->sta::Network::clear();
    network_->setDefaultLibertyLibrary(default_lib);

    // Hierarchy flow change
    // go through all nets, not just the ones in the instance
    // Get the net set from the block
    // old code:
    // NetIterator* net_iter = network_->netIterator(network_->topInstance());
    // Note that in hierarchy mode, this will not present all the nets,
    // which is intent here. So get all flat nets from block
    //

    sortClkAndSignalLayers();

    odb::dbSet<odb::dbNet> nets = block_->getNets();
    for (auto db_net : nets) {
      sta::Net* cur_net = db_network_->dbToSta(db_net);
      estimateWireParasitic(cur_net, spef_writer);
    }
    parasitics_src_ = ParasiticsSrc::kPlacement;
    parasitics_invalid_.clear();
  }
}

void EstimateParasitics::estimateWireParasitic(const sta::Net* net,
                                               sta::SpefWriter* spef_writer)
{
  PinSet* drivers = network_->drivers(net);
  if (drivers && !drivers->empty()) {
    const Pin* drvr_pin = *drivers->begin();
    estimateWireParasitic(drvr_pin, net, spef_writer);
  }
}

void EstimateParasitics::estimateWireParasitic(const sta::Pin* drvr_pin,
                                               const sta::Net* net,
                                               sta::SpefWriter* spef_writer)
{
  if (!network_->isPower(net) && !network_->isGround(net)
      && !db_network_->staToDb(net)->isSpecial()) {
    if (isPadNet(net)) {
      // When an input port drives a pad instance with huge input
      // cap the elmore delay is gigantic. Annotate with zero
      // wire capacitance to prevent wireload model parasitics from being used.
      makePadParasitic(net, spef_writer);
    } else {
      estimateWireParasiticSteiner(drvr_pin, net, spef_writer);
    }
  }
}

void EstimateParasitics::makeWireParasitic(sta::Net* net,
                                           sta::Pin* drvr_pin,
                                           sta::Pin* load_pin,
                                           double wire_length,  // meters
                                           const Scene* corner)
{
  sta::Parasitics* parasitics = corner->parasitics(max_);
  sta::Parasitic* parasitic = parasitics->makeParasiticNetwork(net, false);
  sta::ParasiticNode* n1
      = parasitics->ensureParasiticNode(parasitic, drvr_pin, network_);
  sta::ParasiticNode* n2
      = parasitics->ensureParasiticNode(parasitic, load_pin, network_);
  double wire_cap = wire_length * wireSignalCapacitance(corner);
  double wire_res = wire_length * wireSignalResistance(corner);
  if (hasSpefCalibration()) {
    const NetRcScale scale = netRcScale(net);
    if (scale.valid) {
      wire_cap *= scale.cap_scale;
      wire_res *= scale.res_scale;
    }
  }

  // Reduce resistance if the net has NDR with increased width
  odb::dbTechNonDefaultRule* ndr
      = db_network_->staToDb(net)->getNonDefaultRule();
  if (ndr) {
    std::vector<odb::dbTechLayerRule*> layer_rules;
    ndr->getLayerRules(layer_rules);
    float ndr_ratio = (float) layer_rules.at(0)->getWidth()
                      / layer_rules.at(0)->getLayer()->getWidth();
    wire_res /= ndr_ratio;
  }

  parasitics->incrCap(n1, wire_cap / 2.0);
  parasitics->makeResistor(parasitic, 1, wire_res, n1, n2);
  parasitics->incrCap(n2, wire_cap / 2.0);
}

bool EstimateParasitics::isPadNet(const sta::Net* net) const
{
  const sta::Pin *pin1, *pin2;
  net2Pins(net, pin1, pin2);
  return pin1 && pin2
         && ((network_->isTopLevelPort(pin1) && isPadPin(pin2))
             || (network_->isTopLevelPort(pin2) && isPadPin(pin1)));
}

void EstimateParasitics::makePadParasitic(const sta::Net* net,
                                          sta::SpefWriter* spef_writer)
{
  const sta::Pin *pin1, *pin2;
  net2Pins(net, pin1, pin2);
  for (sta::Scene* corner : sta_->scenes()) {
    sta::Parasitics* parasitics = corner->parasitics(max_);
    sta::Parasitic* parasitic = parasitics->makeParasiticNetwork(net, false);
    sta::ParasiticNode* n1
        = parasitics->ensureParasiticNode(parasitic, pin1, network_);
    sta::ParasiticNode* n2
        = parasitics->ensureParasiticNode(parasitic, pin2, network_);

    // Use a small resistor to keep the connectivity intact.
    parasitics->makeResistor(parasitic, 1, .001, n1, n2);
    if (spef_writer) {
      spef_writer->writeNet(corner, net, parasitic, parasitics);
    }

    if (arc_delay_calc_->reduceSupported()) {
      arc_delay_calc_->reduceParasitic(
          parasitic, net, corner, sta::MinMaxAll::all());
      parasitics->deleteParasiticNetwork(net);
    }
  }
}

void EstimateParasitics::estimateWireParasiticSteiner(
    const sta::Pin* drvr_pin,
    const sta::Net* net,
    sta::SpefWriter* spef_writer)
{
  if (!isIdealClockPin(drvr_pin)) {
    SteinerTree* tree = makeSteinerTree(drvr_pin);
    if (tree) {
      double eff_cap_scale = 1.0;
      double eff_res_scale = 1.0;
      double eff_via_res_scale = 1.0;
      if (hasSpefCalibration()) {
        const NetRcScale scale = ensureNetRcScale(net);
        if (scale.valid) {
          eff_cap_scale = std::clamp<double>(scale.cap_scale, 0.85, 5.0);
          eff_res_scale = std::clamp<double>(scale.res_scale, 0.85, 5.0);
          eff_via_res_scale
              = std::clamp<double>(scale.unit_res_scale, 0.7, 2.5);
        }
      }
      debugPrint(logger_,
                 EST,
                 "estimate_parasitics",
                 1,
                 "estimate wire {}",
                 sdc_network_->pathName(net));
      for (sta::Scene* corner : sta_->scenes()) {
        if (sta_->isIdealClock(drvr_pin, corner->mode())) {
          continue;
        }
        std::set<const Pin*> connected_pins;
        Parasitics* parasitics = corner->parasitics(max_);
        Parasitic* parasitic = parasitics->makeParasiticNetwork(net, false);
        bool is_clk = global_router_->isNonLeafClock(db_network_->staToDb(net));
        double wire_cap = 0.0;
        double wire_res = 0.0;
        const int branch_count = tree->branchCount();
        int max_node_index = tree->getMaxIndex();
        size_t resistor_id = 1;
        for (int i = 0; i < branch_count; i++) {
          odb::Point pt1, pt2;
          SteinerPt steiner_pt1, steiner_pt2;
          int wire_length_dbu;
          tree->branch(i, pt1, steiner_pt1, pt2, steiner_pt2, wire_length_dbu);
          if (wire_length_dbu) {
            double dx = dbuToMeters(abs(pt1.x() - pt2.x()))
                        / dbuToMeters(wire_length_dbu);
            double dy = dbuToMeters(abs(pt1.y() - pt2.y()))
                        / dbuToMeters(wire_length_dbu);

            if (is_clk) {
              wire_cap = dx * wireClkHCapacitance(corner)
                         + dy * wireClkVCapacitance(corner);
              wire_res = dx * wireClkHResistance(corner)
                         + dy * wireClkVResistance(corner);
            } else {
              wire_cap = dx * wireSignalHCapacitance(corner)
                         + dy * wireSignalVCapacitance(corner);
              wire_res = dx * wireSignalHResistance(corner)
                         + dy * wireSignalVResistance(corner);
            }
          } else {
            wire_cap = is_clk ? wireClkCapacitance(corner)
                              : wireSignalCapacitance(corner);
            wire_res = is_clk ? wireClkResistance(corner)
                              : wireSignalResistance(corner);
          }
          sta::ParasiticNode* n1 = parasitics->ensureParasiticNode(
              parasitic, net, steiner_pt1, network_);
          sta::ParasiticNode* n2 = parasitics->ensureParasiticNode(
              parasitic, net, steiner_pt2, network_);
          if (wire_length_dbu == 0) {
            // Use a small resistor to keep the connectivity intact.
            parasitics->makeResistor(parasitic, resistor_id++, 1.0e-3, n1, n2);
          } else {
            double length = dbuToMeters(wire_length_dbu);
            double cap = length * wire_cap * eff_cap_scale;
            double res = length * wire_res * eff_res_scale;

            // Reduce resistance if the net has NDR with increased width
            odb::dbTechNonDefaultRule* ndr
                = db_network_->staToDb(net)->getNonDefaultRule();
            if (ndr) {
              std::vector<odb::dbTechLayerRule*> layer_rules;
              ndr->getLayerRules(layer_rules);
              float ratio = (float) layer_rules.at(0)->getWidth()
                            / layer_rules.at(0)->getLayer()->getWidth();
              res /= ratio;
            }

            // Make pi model for the wire.
            debugPrint(logger_,
                       EST,
                       "estimate_parasitics",
                       2,
                       " pi {} l={} c2={} rpi={} c1={} {}",
                       parasitics->name(n1),
                       units_->distanceUnit()->asString(length),
                       units_->capacitanceUnit()->asString(cap / 2.0),
                       units_->resistanceUnit()->asString(res),
                       units_->capacitanceUnit()->asString(cap / 2.0),
                       parasitics->name(n2));
            parasitics->incrCap(n1, cap / 2.0);
            parasitics->makeResistor(parasitic, resistor_id++, res, n1, n2);
            parasitics->incrCap(n2, cap / 2.0);
          }
          parasiticNodeConnectPins(parasitics,
                                   parasitic,
                                   n1,
                                   tree,
                                   steiner_pt1,
                                   resistor_id,
                                   corner,
                                   connected_pins,
                                   net,
                                   max_node_index,
                                   is_clk,
                                   eff_via_res_scale);
          parasiticNodeConnectPins(parasitics,
                                   parasitic,
                                   n2,
                                   tree,
                                   steiner_pt2,
                                   resistor_id,
                                   corner,
                                   connected_pins,
                                   net,
                                   max_node_index,
                                   is_clk,
                                   eff_via_res_scale);
        }
        if (spef_writer) {
          spef_writer->writeNet(corner, net, parasitic, parasitics);
        }

        if (arc_delay_calc_->reduceSupported()) {
          arc_delay_calc_->reduceParasitic(
              parasitic, net, corner, sta::MinMaxAll::all());
        }
      }
      delete tree;
    }
  }
}

odb::dbTechLayer* EstimateParasitics::getPinLayer(const sta::Pin* pin)
{
  odb::dbITerm* iterm;
  odb::dbBTerm* bterm;
  odb::dbModITerm* moditerm;
  db_network_->staToDb(pin, iterm, bterm, moditerm);

  odb::dbTechLayer* pin_layer = nullptr;
  odb::dbShape pin_shape;
  if (iterm) {
    int min_layer_idx = std::numeric_limits<int>::max();
    for (const auto& [layer, rect] : iterm->getGeometries()) {
      if (layer->getType() == odb::dbTechLayerType::ROUTING
          && layer->getRoutingLevel() < min_layer_idx) {
        min_layer_idx = layer->getRoutingLevel();
        pin_layer = layer;
      }
    }
  } else if (bterm && bterm->getFirstPin(pin_shape)) {
    pin_layer = pin_shape.getTechLayer();
  } else {
    logger_->error(EST,
                   164,
                   "sta::Pin {} has no placed iterm or bterm.",
                   network_->pathName(pin));
  }

  return pin_layer;
}

double EstimateParasitics::computeAverageCutResistance(sta::Scene* scene)
{
  if (layer_res_.empty()) {
    return 0.0;
  }

  double total_resistance = 0.0;
  int count = 0;

  int min_layer = block_->getMinRoutingLayer();
  int max_layer = block_->getMaxRoutingLayer();

  if (max_layer < 0) {
    // Fall back to roughly the middle of the frontside routing stack.
    // Halving the total routing-layer count picks the wrong stack when
    // backside layers (BPR, BM*, BRDL) push the midpoint below the
    // frontside boundary; counting only frontside levels keeps the
    // heuristic on the side that hosts the signal routes whose cut
    // resistance this function averages.
    odb::dbTech* tech = db_->getTech();
    const int total_levels = tech->getRoutingLayerCount();
    odb::dbTechLayer* first_front = tech->firstFrontsideRoutingLayer();
    if (first_front != nullptr) {
      const int first_level = first_front->getRoutingLevel();
      const int frontside_count = total_levels - first_level + 1;
      max_layer = first_level - 1 + frontside_count / 2;
    } else {
      max_layer = total_levels / 2;
    }
  }

  odb::dbTechLayer* min_tech_layer
      = db_->getTech()->findRoutingLayer(min_layer);
  odb::dbTechLayer* max_tech_layer
      = db_->getTech()->findRoutingLayer(max_layer);

  for (int layer_idx = min_tech_layer->getNumber();
       layer_idx <= max_tech_layer->getNumber();
       layer_idx++) {
    odb::dbTechLayer* layer = db_->getTech()->findLayer(layer_idx);
    if (layer && layer->getType() == odb::dbTechLayerType::CUT) {
      const float resistance = layer_res_[layer_idx][scene->index()];
      total_resistance += resistance;
      count++;
    }
  }

  return count > 0 ? total_resistance / count : 0.0;
}

void EstimateParasitics::parasiticNodeConnectPins(
    sta::Parasitics* parasitics,
    sta::Parasitic* parasitic,
    sta::ParasiticNode* node,
    SteinerTree* tree,
    SteinerPt pt,
    size_t& resistor_id,
    sta::Scene* corner,
    std::set<const Pin*>& connected_pins,
    const sta::Net* net,
    int& max_node_index,
    const bool is_clk,
    const double res_scale)
{
  const sta::PinSeq* pins = tree->pins(pt);
  if (pins) {
    odb::dbTechLayer* tree_layer;
    if (is_clk) {
      tree_layer = clk_layers_.empty() ? nullptr : clk_layers_[0];
    } else {
      tree_layer = signal_layers_.empty() ? nullptr : signal_layers_[0];
    }

    for (const sta::Pin* pin : *pins) {
      sta::ParasiticNode* pin_node
          = parasitics->ensureParasiticNode(parasitic, pin, network_);
      if (connected_pins.find(pin) == connected_pins.end()) {
        if (tree_layer != nullptr && !layer_res_.empty()) {
          odb::dbTechLayer* pin_layer = getPinLayer(pin);
          insertViaResistances(pin_layer,
                               tree_layer,
                               parasitics,
                               parasitic,
                               pin_node,
                               node,
                               resistor_id,
                               corner,
                               net,
                               max_node_index,
                               res_scale);
        } else {
          double cut_res
              = std::max(computeAverageCutResistance(corner) * res_scale,
                         1.0e-3);
          parasitics->makeResistor(
              parasitic, resistor_id++, cut_res, node, pin_node);
        }
        connected_pins.insert(pin);
      }
    }
  }
}

void EstimateParasitics::insertViaResistances(odb::dbTechLayer* pin_layer,
                                              odb::dbTechLayer* tree_layer,
                                              sta::Parasitics* parasitics,
                                              sta::Parasitic* parasitic,
                                              sta::ParasiticNode* pin_node,
                                              sta::ParasiticNode* node,
                                              size_t& resistor_id,
                                              sta::Scene* corner,
                                              const sta::Net* net,
                                              int& max_node_index,
                                              const double res_scale)
{
  sta::ParasiticNode* prev_node = nullptr;

  const int pin_layer_idx = pin_layer->getNumber();
  const int tree_layer_idx = tree_layer->getNumber();
  if (std::abs(pin_layer_idx - tree_layer->getNumber()) == 2) {
    // Directly connect pin node and tree node if they are one cut layer apart
    const int cut_layer_idx = pin_layer_idx < tree_layer_idx
                                  ? pin_layer_idx + 1
                                  : pin_layer_idx - 1;
    const double cut_res = std::max(
        layer_res_[cut_layer_idx][corner->index()] * res_scale, 1.0e-3);
    parasitics->makeResistor(parasitic, resistor_id++, cut_res, pin_node, node);
  } else if (pin_layer_idx == tree_layer_idx) {
    // Add a small resistor between the pin node and tree node to keep
    // connectivity
    parasitics->makeResistor(parasitic, resistor_id++, 1.0e-3, pin_node, node);
  } else {
    const auto [start_idx, end_idx]
        = std::minmax(pin_layer_idx, tree_layer_idx);
    const bool pin_is_below = (pin_layer_idx < tree_layer_idx);

    for (int layer_idx = start_idx; layer_idx < end_idx; layer_idx++) {
      odb::dbTechLayer* cut_layer = db_->getTech()->findLayer(layer_idx);
      if (cut_layer->getType() != odb::dbTechLayerType::CUT) {
        continue;
      }
      const double cut_res = std::max(
          layer_res_[layer_idx][corner->index()] * res_scale, 1.0e-3);

      // Resolve from/to endpoints first, so we only allocate a new mid_node
      // when this iteration actually needs one. On the terminal iteration the
      // resistor connects directly to the pin or tree anchor; pre-allocating
      // a mid_node here would create a floating ParasiticNode (singular row
      // in the conductance matrix for Prima/CCS).
      sta::ParasiticNode* from_node = prev_node;
      sta::ParasiticNode* to_node = nullptr;
      bool need_new_mid = true;
      if (pin_is_below) {
        if (layer_idx - 1 == pin_layer_idx) {
          from_node = pin_node;
        }
        if (layer_idx + 1 == tree_layer_idx) {
          to_node = node;
          need_new_mid = false;
        }
      } else {
        if (layer_idx - 1 == tree_layer_idx) {
          from_node = node;
        }
        if (layer_idx + 1 == pin_layer_idx) {
          to_node = pin_node;
          need_new_mid = false;
        }
      }

      sta::ParasiticNode* mid_node = nullptr;
      if (need_new_mid) {
        mid_node = parasitics->ensureParasiticNode(
            parasitic, net, ++max_node_index, network_);
        to_node = mid_node;
      }

      parasitics->makeResistor(
          parasitic, resistor_id++, cut_res, from_node, to_node);

      // On the terminal iteration mid_node is nullptr and prev_node is unused
      // by the next iteration (there is none); on every other iteration we
      // chain through the freshly allocated mid_node.
      prev_node = mid_node;
    }
  }
}

void EstimateParasitics::net2Pins(const sta::Net* net,
                                  const sta::Pin*& pin1,
                                  const sta::Pin*& pin2) const
{
  pin1 = nullptr;
  pin2 = nullptr;

  NetConnectedPinIterator* pin_iter = network_->connectedPinIterator(net);
  if (pin_iter->hasNext()) {
    pin1 = pin_iter->next();
  }
  if (pin_iter->hasNext()) {
    pin2 = pin_iter->next();
  }
  delete pin_iter;
}

bool EstimateParasitics::isPadPin(const sta::Pin* pin) const
{
  sta::Instance* inst = network_->instance(pin);
  return inst && !network_->isTopInstance(inst) && isPad(inst);
}

bool EstimateParasitics::isPad(const sta::Instance* inst) const
{
  dbInst* db_inst;
  dbModInst* mod_inst;
  db_network_->staToDb(inst, db_inst, mod_inst);
  if (mod_inst) {
    return false;
  }
  const auto type = db_inst->getMaster()->getType().getValue();
  // Use switch so if new types are added we get a compiler warning.
  switch (type) {
    case dbMasterType::CORE:
    case dbMasterType::CORE_ANTENNACELL:
    case dbMasterType::CORE_FEEDTHRU:
    case dbMasterType::CORE_TIEHIGH:
    case dbMasterType::CORE_TIELOW:
    case dbMasterType::CORE_WELLTAP:
    case dbMasterType::ENDCAP:
    case dbMasterType::ENDCAP_PRE:
    case dbMasterType::ENDCAP_POST:
    case dbMasterType::CORE_SPACER:
    case dbMasterType::BLOCK:
    case dbMasterType::BLOCK_BLACKBOX:
    case dbMasterType::BLOCK_SOFT:
    case dbMasterType::ENDCAP_TOPLEFT:
    case dbMasterType::ENDCAP_TOPRIGHT:
    case dbMasterType::ENDCAP_BOTTOMLEFT:
    case dbMasterType::ENDCAP_BOTTOMRIGHT:
    case dbMasterType::ENDCAP_LEF58_BOTTOMEDGE:
    case dbMasterType::ENDCAP_LEF58_TOPEDGE:
    case dbMasterType::ENDCAP_LEF58_RIGHTEDGE:
    case dbMasterType::ENDCAP_LEF58_LEFTEDGE:
    case dbMasterType::ENDCAP_LEF58_RIGHTBOTTOMEDGE:
    case dbMasterType::ENDCAP_LEF58_LEFTBOTTOMEDGE:
    case dbMasterType::ENDCAP_LEF58_RIGHTTOPEDGE:
    case dbMasterType::ENDCAP_LEF58_LEFTTOPEDGE:
    case dbMasterType::ENDCAP_LEF58_RIGHTBOTTOMCORNER:
    case dbMasterType::ENDCAP_LEF58_LEFTBOTTOMCORNER:
    case dbMasterType::ENDCAP_LEF58_RIGHTTOPCORNER:
    case dbMasterType::ENDCAP_LEF58_LEFTTOPCORNER:
    case dbMasterType::COVER:
    case dbMasterType::RING:
      return false;
    case dbMasterType::COVER_BUMP:
    case dbMasterType::PAD:
    case dbMasterType::PAD_AREAIO:
    case dbMasterType::PAD_INPUT:
    case dbMasterType::PAD_OUTPUT:
    case dbMasterType::PAD_INOUT:
    case dbMasterType::PAD_POWER:
    case dbMasterType::PAD_SPACER:
      return true;
  }
  // gcc warniing
  return false;
}

bool EstimateParasitics::isIdealClockPin(const sta::Pin* pin) const
{
  // An ideal clock pin carries fixed arrivals that do not depend on
  // parasitics, so its parasitics never need re-estimation.
  bool is_clock = false;
  for (sta::Mode* mode : sta_->modes()) {
    // In multi-mode designs, a pin may be an ideal clock only in a subset of
    // modes. Ignore modes where the pin is not a clock at all.
    // e.g., scan clock pin may not be defined as clock in function mode.
    if (!sta_->isClock(pin, mode)) {
      continue;
    }
    is_clock = true;
    if (!sta_->isIdealClock(pin, mode)) {
      return false;
    }
  }
  return is_clock;
}

bool EstimateParasitics::isIdealClockNet(const sta::Net* net) const
{
  odb::dbNet* db_net = db_network_->staToDb(net);
  if (db_net == nullptr) {
    return false;
  }

  PinSet* drivers = network_->drivers(net);
  if (drivers == nullptr || drivers->empty()) {
    return false;
  }

  const Pin* drvr_pin = *drivers->begin();
  return isIdealClockPin(drvr_pin);
}

void EstimateParasitics::parasiticsInvalid(const sta::Net* net)
{
  odb::dbNet* db_net = db_network_->findFlatDbNet(net);
  if (haveEstimatedParasitics() && db_net) {
    debugPrint(logger_,
               EST,
               "estimate_parasitics",
               2,
               "parasitics invalid {}",
               network_->pathName(net));
    parasitics_invalid_.insert(db_network_->dbToSta(db_net));
  }
}

void EstimateParasitics::parasiticsInvalid(const odb::dbNet* net)
{
  parasiticsInvalid(db_network_->dbToSta(net));
}

void EstimateParasitics::setParasiticsSrc(ParasiticsSrc src)
{
  if (incremental_parasitics_enabled_) {
    logger_->error(EST,
                   108,
                   "cannot change parasitics source while incremental "
                   "parasitics enabled");
  }

  parasitics_src_ = src;
}

void EstimateParasitics::eraseParasitics(const sta::Net* net)
{
  parasitics_invalid_.erase(net);
  if (spef_net_baselines_.find(net) == spef_net_baselines_.end()) {
    net_rc_scales_.erase(net);
  }
}

static void connectedPins(const sta::Net* net,
                          sta::Network* network,
                          sta::dbNetwork* db_network,
                          // Return value.
                          std::vector<PinLoc>& pins);

static void connectedPins(const sta::Net* net,
                          sta::Network* network,
                          sta::dbNetwork* db_network,
                          // Return value.
                          std::vector<PinLoc>& pins)
{
  NetConnectedPinIterator* pin_iter = network->connectedPinIterator(net);
  while (pin_iter->hasNext()) {
    const sta::Pin* pin = pin_iter->next();
    odb::dbITerm* iterm;
    odb::dbBTerm* bterm;
    odb::dbModITerm* moditerm;
    db_network->staToDb(pin, iterm, bterm, moditerm);
    //
    // only accumuate the flat pins (in hierarchical mode we may
    // hit moditerms/modbterms).
    //
    if (iterm || bterm) {
      odb::Point loc = db_network->location(pin);
      pins.push_back({pin, loc});
    }
  }
  delete pin_iter;
}

SteinerTree* EstimateParasitics::makeSteinerTree(
    odb::Point drvr_location,
    const std::vector<odb::Point>& sink_locations)
{
  SteinerTree* tree = new SteinerTree(drvr_location, logger_);
  std::vector<PinLoc>& pinlocs = tree->pinlocs();
  for (auto loc : sink_locations) {
    pinlocs.push_back(PinLoc{nullptr, loc});
  }
  // Sort pins by location
  sort(pinlocs.begin(),
       pinlocs.end(),
       [=](const PinLoc& pin1, const PinLoc& pin2) {
         return pin1.loc.getX() < pin2.loc.getX()
                || (pin1.loc.getX() == pin2.loc.getX()
                    && pin1.loc.getY() < pin2.loc.getY());
       });
  int pin_count = pinlocs.size();
  if (pin_count >= 1) {
    // Two separate vectors of coordinates needed by flute.
    std::vector<int> x, y;
    int drvr_idx = pinlocs.size();
    pinlocs.push_back(PinLoc{nullptr, drvr_location});
    for (int i = 0; i < pin_count + 1; i++) {
      const PinLoc& pinloc = pinlocs[i];
      x.push_back(pinloc.loc.x());
      y.push_back(pinloc.loc.y());
    }
    stt::Tree ftree = stt_builder_->makeSteinerTree(x, y, drvr_idx);
    tree->setTree(ftree);
    tree->populateSides();
    return tree;
  }
  delete tree;
  return nullptr;
}

// Returns nullptr if net has less than 2 pins or any pin is not placed.
SteinerTree* EstimateParasitics::makeSteinerTree(const sta::Pin* drvr_pin)
{
  sta::Network* sdc_network = network_->sdcNetwork();

  /*
    Handle hierarchy. Make sure all traversal on dbNets.
   */
  odb::dbNet* db_net = db_network_->findFlatDbNet(drvr_pin);
  sta::Net* net = db_network_->dbToSta(db_net);

  debugPrint(
      logger_, EST, "steiner", 1, "sta::Net {}", sdc_network->pathName(net));
  SteinerTree* tree = new SteinerTree(drvr_pin, db_network_, logger_);
  std::vector<PinLoc>& pinlocs = tree->pinlocs();
  // Find all the connected pins
  connectedPins(net, network_, db_network_, pinlocs);
  // Sort pins by location because connectedPins order is not deterministic.
  sort(pinlocs.begin(),
       pinlocs.end(),
       [=](const PinLoc& pin1, const PinLoc& pin2) {
         return pin1.loc.getX() < pin2.loc.getX()
                || (pin1.loc.getX() == pin2.loc.getX()
                    && pin1.loc.getY() < pin2.loc.getY());
       });
  int pin_count = pinlocs.size();
  bool is_placed = true;
  if (pin_count >= 2) {
    std::vector<int> x;  // Two separate vectors of coordinates needed by flute.
    std::vector<int> y;
    int drvr_idx = 0;  // The "driver_pin" or the root of the Steiner tree.
    for (int i = 0; i < pin_count; i++) {
      const PinLoc& pinloc = pinlocs[i];
      if (pinloc.pin == drvr_pin) {
        drvr_idx = i;  // drvr_index is needed by flute.
      }
      x.push_back(pinloc.loc.x());
      y.push_back(pinloc.loc.y());
      debugPrint(logger_,
                 EST,
                 "steiner",
                 3,
                 " {} ({} {})",
                 sdc_network->pathName(pinloc.pin),
                 pinloc.loc.x(),
                 pinloc.loc.y());
      // Track that all our pins are placed.
      is_placed &= db_network_->isPlaced(pinloc.pin);

      // Flute may reorder the input points, so it takes some unravelling
      // to find the mapping back to the original pins. The complication is
      // that multiple pins can occupy the same location.
      tree->locAddPin(pinloc.loc, pinloc.pin);
    }
    if (is_placed) {
      stt::Tree ftree = stt_builder_->makeSteinerTree(db_net, x, y, drvr_idx);

      tree->setTree(ftree);
      tree->createSteinerPtToPinMap();
      return tree;
    }
  }
  delete tree;
  return nullptr;
}

double EstimateParasitics::dbuToMeters(int dist) const
{
  return dist / (dbu_ * 1e+6);
}

void EstimateParasitics::highlightSteiner(const sta::Pin* drvr)
{
  if (steiner_renderer_) {
    est::SteinerTree* tree = nullptr;
    if (drvr) {
      tree = makeSteinerTree(drvr);
    }
    steiner_renderer_->highlight(tree);
  }
}

////////////////////////////////////////////////////////////////

IncrementalParasiticsGuard::IncrementalParasiticsGuard(
    est::EstimateParasitics* estimate_parasitics)
    : estimate_parasitics_(estimate_parasitics), need_unregister_(false)
{
  // check to allow reentrancy
  if (!estimate_parasitics_->isIncrementalParasiticsEnabled()) {
    if (!estimate_parasitics_->getBlock()) {
      estimate_parasitics_->initBlock();
    }

    const bool has_pending_invalid_parasitics
        = estimate_parasitics_->hasParasiticsInvalid();

    switch (estimate_parasitics_->getParasiticsSrc()) {
      case ParasiticsSrc::kPlacement:
        break;
      case ParasiticsSrc::kGlobalRouting:
      case ParasiticsSrc::kDetailedRouting:
        // TODO: add IncrementalDRoute
        estimate_parasitics_->setIncrementalGRT(
            new grt::IncrementalGRoute(estimate_parasitics_->getGlobalRouter(),
                                       estimate_parasitics_->getBlock()));
        // Don't print verbose messages for incremental routing
        estimate_parasitics_->getGlobalRouter()->setVerbose(false);
        break;
      case ParasiticsSrc::kNone:
        break;
    }

    estimate_parasitics_->setIncrementalParasiticsEnabled(true);
    if (has_pending_invalid_parasitics) {
      estimate_parasitics_->getLogger()->warn(
          EST, 104, "inconsistent parasitics state");
      estimate_parasitics_->updateParasitics();
    }
    estimate_parasitics_->setDbCbkOwner(estimate_parasitics_->getBlock());
    need_unregister_ = true;
  }
}

void IncrementalParasiticsGuard::update()
{
  estimate_parasitics_->updateParasitics();
}

IncrementalParasiticsGuard::~IncrementalParasiticsGuard()
{
  if (need_unregister_) {
    try {
      if (!estimate_parasitics_->hasSpefNets()) {
        estimate_parasitics_->removeDbCbkOwner();
      }
      estimate_parasitics_->updateParasitics();
    } catch (const std::exception& e) {
      // Exceptions must not escape destructors (implicitly noexcept).
      // Log and continue with cleanup.
      estimate_parasitics_->getLogger()->warn(
          EST, 165, "Exception during parasitic update: {}.", e.what());
    } catch (...) {
      estimate_parasitics_->getLogger()->warn(
          EST, 166, "Unknown exception during parasitic update.");
    }

    switch (estimate_parasitics_->getParasiticsSrc()) {
      case ParasiticsSrc::kPlacement:
        break;
      case ParasiticsSrc::kGlobalRouting:
      case ParasiticsSrc::kDetailedRouting:
        // TODO: add IncrementalDRoute
        delete estimate_parasitics_->getIncrementalGRT();
        estimate_parasitics_->setIncrementalGRT(nullptr);
        break;
      case ParasiticsSrc::kNone:
        break;
    }

    estimate_parasitics_->setIncrementalParasiticsEnabled(false);
  }
}

}  // namespace est
