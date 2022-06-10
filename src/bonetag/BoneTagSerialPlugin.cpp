#include "BoneTagSerialPlugin.h"

#include <mc_control/GlobalPluginMacros.h>
#include <optional>

namespace mc_plugin
{

constexpr auto DEFAULT_RATE = 200; // [Hz]

BoneTagSerialPlugin::~BoneTagSerialPlugin()
{
  mc_rtc::log::info("[BoneTagSerialPlugin] Stopping communication thread");
  running_ = false;
  thread_.join();
  mc_rtc::log::info("[BoneTagSerialPlugin] Communication thread stopped");
}

void BoneTagSerialPlugin::init(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & config)
{
  data_.fill(0);
  lastData_ = data_;

  config("verbose", verbose_);
  double rate = config("rate", DEFAULT_RATE);
  rate_ = (1. / rate) / gc.controller().timeStep;

  auto descriptor = config("descriptor", std::string{"/dev/ttyACM0"});
  thread_ = std::thread([this, descriptor]() {
    mc_rtc::log::info("[BoneTagSerialPlugin] Starting serial communication thread with {}", descriptor);
    serial_.open(descriptor);
    mc_rtc::log::info("[BoneTagSerialPlugin] Communication with device {} opened", descriptor);

    unsigned iter_ = 0;
    while(running_)
    {
      if(iter_ == 0 || iter_ % rate_ == 0)
      {
        try
        {
          auto data = serial_.read();
          std::lock_guard<std::mutex> lock(dataMutex_);
          data_ = data;
          hasReceivedData_ = true;
        }
        catch(std::runtime_error & e)
        {
          mc_rtc::log::error("[BoneTagSerialPlugin] Failed to read data");
          hasReceivedData_ = false;
        }
        iter_ = 0;
      }
      ++iter_;
    }
    serial_.close();
  });

  gc.controller().datastore().make<bool>("BoneTagSerialPlugin", true);
  gc.controller().datastore().make_call("BoneTagSerialPlugin::GetLastData",
                                        [this]() -> const io::BoneTagSerial::Data & { return lastData_; });
  gc.controller().datastore().make_call("BoneTagSerialPlugin::GetNewData",
                                        [this]() -> std::optional<io::BoneTagSerial::Data> {
                                          if(lastDataIsNew_)
                                          {
                                            lastDataIsNew_ = false;
                                            return lastData_;
                                          }
                                          else
                                          {
                                            return std::nullopt;
                                          }
                                        });
  gc.controller().datastore().make_call("BoneTagSerialPlugin::Stop", [this]() -> void { running_ = false; });

  gc.controller().gui()->addElement({"BoneTagSerialPlugin"},
                                    mc_rtc::gui::ArrayLabel("Data", {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9"},
                                                            [this]() { return lastData_; }),

                                    mc_rtc::gui::Button("Stop", [this]() {
                                      running_ = false;
                                      thread_.join();
                                    }));
}

void BoneTagSerialPlugin::reset(mc_control::MCGlobalController & controller) {}

void BoneTagSerialPlugin::before(mc_control::MCGlobalController & gc)
{
  if(hasReceivedData_)
  {
    std::lock_guard<std::mutex> lock(dataMutex_);
    lastData_ = data_;
    hasReceivedData_ = false;
    lastDataIsNew_ = true;
  }
}

void BoneTagSerialPlugin::after(mc_control::MCGlobalController & controller) {}

mc_control::GlobalPlugin::GlobalPluginConfiguration BoneTagSerialPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = false;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("BoneTagSerialPlugin", mc_plugin::BoneTagSerialPlugin)
