// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "odb/db.h"
#include "sta/Liberty.hh"
#include "sta/NetworkClass.hh"
#include "tst/IntegratedFixture.h"

namespace rsz {

class TestResizer : public tst::IntegratedFixture
{
 public:
  TestResizer()
      : tst::IntegratedFixture(tst::IntegratedFixture::Technology::kNangate45,
                               "_main/src/rsz/test/")
  {
  }

 protected:
  static odb::dbITerm* findITerm(odb::dbInst* inst, const char* pin_name)
  {
    odb::dbITerm* iterm = inst->findITerm(pin_name);
    EXPECT_NE(iterm, nullptr);
    return iterm;
  }

  static std::string modNetName(odb::dbITerm* iterm)
  {
    odb::dbModNet* modnet = iterm->getModNet();
    return modnet ? modnet->getName() : "None";
  }

  sta::LibertyPort* findLibertyPort(sta::Instance* inst, const char* port_name)
  {
    std::unique_ptr<sta::InstancePinIterator> pin_iter{
        sta_->network()->pinIterator(inst)};
    while (pin_iter->hasNext()) {
      sta::Pin* pin = pin_iter->next();
      sta::LibertyPort* port = sta_->network()->libertyPort(pin);
      if (port != nullptr && port->name() == port_name) {
        return port;
      }
    }
    return nullptr;
  }
};

TEST_F(TestResizer, SwapPinsFeedthroughModNet)
{
  const testing::TestInfo* test_info
      = testing::UnitTest::GetInstance()->current_test_info();
  const std::string test_name
      = std::string(test_info->test_suite_name()) + "_" + test_info->name();

  readVerilogAndSetup(test_name + "_pre.v");

  odb::dbNet* net = block_->findNet("src_net");
  ASSERT_NE(net, nullptr);

  odb::dbInst* gate = block_->findInst("target");
  odb::dbInst* drv = block_->findInst("drv_ff");
  odb::dbInst* probe = block_->findInst("probe");
  ASSERT_NE(gate, nullptr);
  ASSERT_NE(drv, nullptr);
  ASSERT_NE(probe, nullptr);

  // Seed state matching the cva6 hierarchy:
  // one gate input already carries the feed-through output-side modnet,
  // while the rest of the flat src_net still uses the input-side name.
  EXPECT_EQ(modNetName(findITerm(gate, "A1")), "tap_out");
  EXPECT_EQ(modNetName(findITerm(drv, "Q")), "src_net");
  EXPECT_EQ(modNetName(findITerm(probe, "A")), "src_net");

  sta::Instance* sta_gate = db_network_->dbToSta(gate);
  ASSERT_NE(sta_gate, nullptr);
  sta::LibertyPort* port_a1 = findLibertyPort(sta_gate, "A1");
  sta::LibertyPort* port_a2 = findLibertyPort(sta_gate, "A2");
  ASSERT_NE(port_a1, nullptr);
  ASSERT_NE(port_a2, nullptr);

  // This is the exact reconnect path used by repair_timing's swap-pins move.
  ASSERT_TRUE(resizer_.swapPins(sta_gate, port_a1, port_a2));

  // The correct post-swap state should keep the output-side modnet local to
  // the swapped gate input. The rest of the flat src_net should preserve
  // its input-side modnet name.
  EXPECT_EQ(modNetName(findITerm(gate, "A2")), "tap_out");
  EXPECT_EQ(modNetName(findITerm(drv, "Q")), "src_net");
  EXPECT_EQ(modNetName(findITerm(probe, "A")), "src_net");

  writeAndCompareVerilogOutputFile(test_name, test_name + "_post.v");
}

TEST_F(TestResizer, ExternalPadOnlyNetCannotBeBuffered)
{
  readVerilogAndSetup("TestResizer_SwapPinsFeedthroughModNet_pre.v");

  odb::dbMaster* pad_master = odb::dbMaster::create(lib_, "TEST_PAD");
  pad_master->setWidth(100);
  pad_master->setHeight(100);
  pad_master->setType(odb::dbMasterType::PAD);
  odb::dbMTerm::create(
      pad_master, "PAD", odb::dbIoType::INOUT, odb::dbSigType::SIGNAL);
  odb::dbMTerm::create(
      pad_master, "C", odb::dbIoType::OUTPUT, odb::dbSigType::SIGNAL);
  pad_master->setFrozen();
  pad_master->staSetCell(sta_->network()->findLibertyCell("BUF_X1"));
  odb::dbInst* pad_inst = odb::dbInst::create(block_, pad_master, "pad0");

  odb::dbNet* external_net
      = odb::dbNet::create(block_, "external_pad_test_net");
  odb::dbBTerm* external_port
      = odb::dbBTerm::create(external_net, "external_pad_test_port");
  external_port->setIoType(odb::dbIoType::INPUT);
  pad_inst->findITerm("PAD")->connect(external_net);

  odb::dbNet* core_net = odb::dbNet::create(block_, "core_side_test_net");
  odb::dbBTerm* core_port
      = odb::dbBTerm::create(core_net, "core_side_test_port");
  core_port->setIoType(odb::dbIoType::INPUT);
  pad_inst->findITerm("C")->connect(core_net);

  sta_->postReadDef(block_);

  sta::Pin* external_pin = db_network_->dbToSta(external_port);
  ASSERT_NE(external_pin, nullptr);
  EXPECT_FALSE(resizer_.okToBufferNet(external_pin));

  sta::Pin* core_pin = db_network_->dbToSta(pad_inst->findITerm("C"));
  ASSERT_NE(core_pin, nullptr);
  EXPECT_TRUE(resizer_.okToBufferNet(core_pin));
}

}  // namespace rsz
