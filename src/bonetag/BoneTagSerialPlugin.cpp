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

void BoneTagSerialPlugin::connect()
{
  serial_.close();
  try
  {
    serial_.open(descriptor_);
    mc_rtc::log::success("[BoneTagSerialPlugin] Communication with device {} opened", descriptor_);
  }
  catch(std::runtime_error & e)
  {
    mc_rtc::log::warning(e.what());
  }
}

void BoneTagSerialPlugin::init(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & config)
{
  data_.fill(0);
  lastData_ = data_;

  config("verbose", verbose_);
  double rate = config("rate", DEFAULT_RATE);
  rate_ = (1. / rate) / gc.controller().timeStep;

  descriptor_ = config("descriptor", std::string{"/dev/ttyUSB0"});
  thread_ = std::thread(
      [this]()
      {
        mc_rtc::log::info("[BoneTagSerialPlugin] Starting serial communication thread with {}", descriptor_);
        connect();

        unsigned iter_ = 0;
        while(running_)
        {
          if(iter_ == 0 || iter_ % rate_ == 0)
          {
            if(connect_requested_)
            {
              connect();
              connect_requested_ = false;
            }
            try
            {
              auto data = serial_.read();
              std::lock_guard<std::mutex> lock(dataMutex_);
              data_ = data;
              hasReceivedData_ = true;
            }
            catch(std::runtime_error & e)
            {
              /* mc_rtc::log::error("[BoneTagSerialPlugin] Failed to read data"); */
              hasReceivedData_ = false;
            }
            iter_ = 0;
          }
          ++iter_;
        }
        serial_.close();
      });

  gc.controller().datastore().make<bool>("BoneTagSerialPlugin", true);
  gc.controller().datastore().make_call("BoneTagSerialPlugin::Connected", [this]() { return serial_.connected(); });
  gc.controller().datastore().make_call("BoneTagSerialPlugin::GetLastData",
                                        [this]() -> const io::BoneTagSerial::Data & { return lastData_; });
  gc.controller().datastore().make_call("BoneTagSerialPlugin::GetNewData",
                                        [this]() -> std::optional<io::BoneTagSerial::Data>
                                        {
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

  gc.controller().gui()->addElement(
      {"BoneTagSerialPlugin"}, mc_rtc::gui::Label("Connected", [this]() { return serial_.connected(); }),
      mc_rtc::gui::Button("Connect", [this]() { connect_requested_ = true; }),
      mc_rtc::gui::ArrayLabel("Data", {"0", "1", "2", "3"}, [this]() { return lastData_; }),

      mc_rtc::gui::Button("Stop",
                          [this]()
                          {
                            running_ = false;
                            thread_.join();
                          }));

  using Color = mc_rtc::gui::Color;
  using Style = mc_rtc::gui::plot::Style;
  const std::vector<std::pair<Color, Style>> sensorColors = {
      {Color::Red, Style::Solid},     {Color::Blue, Style::Dashed},          {Color::Green, Style::Solid},
      {Color::Black, Style::Dashed},  {Color::Gray, Style::Solid},           {Color::Cyan, Style::Dashed},
      {Color::Magenta, Style::Solid}, {Color(0.96, 0.74, 0), Style::Dashed}, {Color::Blue, Style::Solid},
      {Color::Green, Style::Dashed}};

  auto make_sensor_plot = [this, sensorColors](unsigned index)
  {
    return mc_rtc::gui::plot::Y(
        fmt::format("Sensor {}", index), [this, index]() { return lastData_[index]; }, sensorColors[index].first,
        sensorColors[index].second);
  };

  gc.controller().gui()->addPlot("BoneTag Measurements", mc_rtc::gui::plot::X("N", [this]() { return t_; }),
                                 make_sensor_plot(0), make_sensor_plot(1), make_sensor_plot(2), make_sensor_plot(3));
  gc.controller().logger().addLogEntry("BoneTag_Sensors", this,
                                       [this]() -> std::array<double, 4>
                                       {
                                         std::array<double, 4> data{0};
                                         for(int i = 0; i < lastData_.size(); ++i)
                                         {
                                           data[i] = lastData_[i];
                                         }
                                         return data;
                                       });
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

void BoneTagSerialPlugin::after(mc_control::MCGlobalController & controller)
{
  t_ += controller.timestep();
}

mc_control::GlobalPlugin::GlobalPluginConfiguration BoneTagSerialPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = false;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("BoneTagSerialPlugin", mc_plugin::BoneTagSerialPlugin)
