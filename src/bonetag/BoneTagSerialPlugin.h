/*
 * Copyright 2021 CNRS-UM LIRMM
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include "BoneTagSerial.h"
#include <mutex>
#include <thread>

namespace mc_plugin
{

struct BoneTagSerialPlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~BoneTagSerialPlugin() override;

protected:
  io::BoneTagSerial serial_;
  std::thread thread_;
  std::mutex dataMutex_;
  io::BoneTagSerial::Data data_;
  io::BoneTagSerial::Data lastData_;

  bool hasReceivedData_ = false;
  bool lastDataIsNew_ = false;

  bool running_ = true;
  bool verbose_ = true;
  unsigned rate_ = 0;
};

} // namespace mc_plugin
