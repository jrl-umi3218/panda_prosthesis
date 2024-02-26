#include "PandaBraceController.h"
#include <mc_panda/devices/Robot.h>
#include <mc_rtc/io_utils.h>
#include "config.h"

namespace panda_prosthetics
{

PandaBraceController::PandaBraceController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config), config_(config)
{
  auto & gui = *this->gui();
  gui.addElement(
      {"PandaBrace", "Frames"}, mc_rtc::gui::Transform("Femur", [this]() { return robot().frame("Femur").position(); }),
      mc_rtc::gui::Transform("Tibia", [this]() { return robot("brace_bottom_setup").frame("Tibia").position(); }));

  gui.addElement(
      {"PandaBrace", "Forces"},
      mc_rtc::gui::Label("Details",
                         [this]() { return std::string{"Forces without gravity expressed in the reference frame"}; }));
  gui.addElement({"PandaBrace", "Forces", robot().name()},
                 mc_rtc::gui::Force(
                     "FemurWrench", [this]() { return robot().frame("Femur").wrench(); },
                     [this]() { return robot().frame("Femur").position(); }));
  gui.addElement({"PandaBrace", "Forces", "brace_bottom_setup"},
                 mc_rtc::gui::Force(
                     "TibiaWrench", [this]() { return robot("brace_bottom_setup").frame("Tibia").wrench(); },
                     [this]() { return robot("brace_bottom_setup").frame("Tibia").position(); }));
  logger().addLogEntry("controlRobot_Frame_Femur", [this]() { return robot().frame("Femur").position(); });
  logger().addLogEntry("realRobot_Frame_Femur", [this]() { return realRobot().frame("Femur").position(); });

  datastore().make<std::string>("ControllerName", "brace");

  if(config.has("robots") && config("robots").has(robot().name()))
  {
    auto robotConfig = config("robots")(robot().name());
    if(robotConfig.has("CollisionBehavior"))
    {
      auto colC = robotConfig("CollisionBehavior");
      mc_rtc::log::warning("[{}] Changing robot CollisionBeaviour to:\n{}", this->name_, colC.dump(true, true));
      auto & robot_device = robot().device<mc_panda::Robot>("Robot");
      robot_device.setCollisionBehavior(colC("lower_torque_thresholds").operator std::array<double, 7>(),
                                        colC("upper_torque_thresholds").operator std::array<double, 7>(),
                                        colC("lower_force_thresholds").operator std::array<double, 6>(),
                                        colC("upper_force_thresholds").operator std::array<double, 6>());
    }
  }

  // XXX should be done by the plugin
  // Log the raw force shoes values
  // The filtered values are set to the the robot's sensor
  logger().addLogEntry("brace_bottom_setup_Bottom_raw",
                       [this]()
                       {
                         if(datastore().has("ForceShoePlugin::RFForce"))
                         {
                           return datastore().get<sva::ForceVecd>("ForceShoePlugin::RFForce");
                         }
                         return sva::ForceVecd::Zero();
                       });
  logger().addLogEntry("brace_bottom_setup_Top_raw",
                       [this]()
                       {
                         if(datastore().has("ForceShoePlugin::LBForce"))
                         {
                           return datastore().get<sva::ForceVecd>("ForceShoePlugin::LBForce");
                         }
                         return sva::ForceVecd::Zero();
                       });

  for(auto & robot : robots())
  {
    for(auto & fs : robot.forceSensors())
    {
      robot.makeFrame(fs.name(), robot.frame(fs.parent()), fs.X_p_s());
    }
  }
}

bool PandaBraceController::run()
{
  // Set actual sensor values from the FoceShoePlugin
  // XXX should be done in the plugin
  // XXX robot and sensor names hardcoded
  auto setForceShoeSensorValue =
      [this](const std::string & datastoreName, const std::string & robotName, const std::string & sensorName)
  {
    auto & data = *robot(robotName).data();
    auto & fs = data.forceSensors[data.forceSensorsIndex.at(sensorName)];
    if(this->datastore().has(datastoreName))
    {
      auto & wrench = this->datastore().get<sva::ForceVecd>(datastoreName);
      fs.wrench(wrench);
    }
    else
    {
      fs.wrench(sva::ForceVecd::Zero());
    }
  };
  setForceShoeSensorValue("ForceShoePlugin::LBfiltered", "brace_bottom_setup", "BraceTopForceSensor");
  setForceShoeSensorValue("ForceShoePlugin::RFfiltered", "brace_bottom_setup", "BraceBottomForceSensor");

  //  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::Joints);
  static size_t iter_ = 0;
  if(++iter_ == 5000)
  {
    mc_rtc::log::warning("RESET CONTROL TO REAL");
    robot().mbc().q = realRobot().mbc().q;
    robot().forwardKinematics();
    getPostureTask(robot().name())->reset();
  }
  return mc_control::fsm::Controller::run();
}

void PandaBraceController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("PandaBrace", panda_prosthetics::PandaBraceController)

} // namespace panda_prosthetics
