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
  serial_.close_serial_port();
  try
  {
    serial_.open_serial_port(serial_port_name, serial_port_baud_rate);
    mc_rtc::log::success("[BoneTagSerialPlugin] Communication with device {} opened", serial_port_name);
  }
  catch(std::runtime_error & e)
  {
    mc_rtc::log::warning(e.what());
  }
}

void BoneTagSerialPlugin::connectAndStartReading()
{
  connect();
  try
  {
    mc_rtc::log::success("[BoneTagSerialPlugin] Starting serial port reading ...");
    serial_.read_serial_port();
    mc_rtc::log::success("[BoneTagSerialPlugin] Serial port reading started !");
  }
  catch(std::runtime_error & e)
  {
    mc_rtc::log::error("[BoneTagSerialPlugin] Failed to read data");
    hasReceivedData_ = false;
  }
  connect_requested_ = false;
}
void BoneTagSerialPlugin::init(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & config)
{
  auto & ctl = gc.controller();
  data_.fill(0);
  lastData_ = data_;

  config("verbose", verbose_);
  double rate = config("rate", DEFAULT_RATE);
  rate_ = (1. / rate) / ctl.timeStep;

  serial_port_name = config("serial_port_name", std::string{"/dev/ttyUSB0"});
  serial_port_baud_rate = config("serial_port_baud_rate", 9600);

  thread_ = std::thread(
      [this]()
      {
        mc_rtc::log::info("[BoneTagSerialPlugin] Starting serial communication thread with {}", serial_port_name);
        connectAndStartReading();

        unsigned iter_ = 0;
        while(running_)
        {
          if(iter_ == 0 || iter_ % rate_ == 0)
          {
            if(connect_requested_)
            {
              connectAndStartReading();
            }
            iter_ = 0;
          }
          ++iter_;
        }
        serial_.close_serial_port();
      });

  gc.controller().datastore().make_call("BoneTagSerialPlugin::RequestNewFrame",
                                        [this]() { return serial_.requestNewFrame(); });
  gc.controller().datastore().make_call("BoneTagSerialPlugin::GotNewFrame",
                                        [this]() { return serial_.gotFullFrame(); });
  gc.controller().datastore().make_call("BoneTagSerialPlugin::GetLastFrame",
                                        [this]() { return serial_.getLastFrame(); });

  gc.controller().datastore().make<bool>("BoneTagSerialPlugin", true);
  gc.controller().datastore().make_call("BoneTagSerialPlugin::Connected", [this]() { return serial_.connected(); });
  gc.controller().datastore().make_call("BoneTagSerialPlugin::GetLastData",
                                        [this]() -> const io::BoneTagSerial::Data &
                                        {
                                          std::lock_guard<std::mutex> lockDataMutex(dataMutex_);
                                          return lastData_;
                                        });
  gc.controller().datastore().make_call("BoneTagSerialPlugin::GetNewData",
                                        [this]() -> std::optional<io::BoneTagSerial::Data>
                                        {
                                          // std::lock_guard<std::mutex> lockDataMutex(dataMutex_);
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
  std::vector<std::string> label;
  {
    std::lock_guard<std::mutex> lockDataMutex(dataMutex_);

    label.resize(lastData_.size());
    for(int i = 0; i < lastData_.size(); ++i)
    {
      label[i] = std::to_string(i);
    }
  }

  gc.controller().gui()->addElement(
      {"BoneTagSerialPlugin"}, mc_rtc::gui::Label("Connected", [this]() { return serial_.connected(); }),
      mc_rtc::gui::Button("Connect", [this]() { connect_requested_ = true; }),
      mc_rtc::gui::ArrayLabel("Data", label,
                              [this]()
                              {
                                std::lock_guard<std::mutex> lockDataMutex(dataMutex_);
                                return lastData_;
                              }),
      mc_rtc::gui::NumberInput(
          "Alpha Filter", [this]() { return serial_.alphaFilter(); }, [this](double a) { serial_.alphaFilter(a); }),

      mc_rtc::gui::Button("Stop",
                          [this]()
                          {
                            running_ = false;
                            thread_.join();
                          }));
}

void BoneTagSerialPlugin::reset(mc_control::MCGlobalController & controller) {}

void BoneTagSerialPlugin::before(mc_control::MCGlobalController & gc)
{
  std::lock_guard<std::mutex> lockReceivedDataMutex(serial_.received_data_mutex);

  // all_data.insert(all_data.end(),serial_.received_data.begin(),serial_.received_data.end());
  hasReceivedData_ = !serial_.received_data.empty();

  if(hasReceivedData_)
  {
    // mc_rtc::log::info("[BoneTagSerialPlugin] Received data {}", serial_.received_data.back()[0]);
    // data_ = serial_.received_data.back();
    for(auto data_el : serial_.received_data)
    {
      {
        std::lock_guard<std::mutex> lockDataMutex(dataMutex_);
        lastData_ = data_el;
        lastDataIsNew_ = true;
      }

      if(!plotDisplayed)
      {
        plotDisplayed = true;
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
              fmt::format("Sensor {}", index + 1), [this, index]() { return lastData_[index]; },
              sensorColors[index].first, sensorColors[index].second);
        };

        gc.controller().gui()->addPlot("BoneTag Measurements", mc_rtc::gui::plot::X("N", [this]() { return t_; }),
                                       make_sensor_plot(0), make_sensor_plot(1), make_sensor_plot(2),
                                       make_sensor_plot(3), make_sensor_plot(4), make_sensor_plot(5),
                                       make_sensor_plot(6), make_sensor_plot(7));

        std::vector<double> data;
        data.resize(lastData_.size());
        gc.controller().logger().addLogEntry("BoneTag_Sensors", this,
                                             [this, data]() mutable -> std::vector<double>
                                             {
                                               for(int i = 0; i < lastData_.size(); ++i)
                                               {
                                                 data[i] = lastData_[i];
                                               }
                                               return data;
                                             });
      }
    }
    serial_.received_data.clear();
    hasReceivedData_ = false;
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
