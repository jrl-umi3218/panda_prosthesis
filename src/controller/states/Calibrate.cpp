#include "Calibrate.h"

#include <mc_control/fsm/Controller.h>

// XXX it would make more sense to store the pose corresponding to the
// calibrated joint without the calibration tool,
// e.g the one after PrepareJoint
void save(const std::string & etc_file, const mc_rbdyn::Robot & robot)
{
  mc_rtc::Configuration initial(etc_file);
  initial.add(robot.name());
  initial(robot.name()).add("pose", robot.posW());
  initial(robot.name()).add("joints", robot.mbc().q);
  initial.save(etc_file);
  mc_rtc::log::success("Calibration saved to {}", etc_file);
}

void Calibrate::start(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->addElement(this, {}, mc_rtc::gui::Button("Change Tool", [this]() { output("ChangeTool"); }));
  ctl.gui()->addElement(this, {}, mc_rtc::gui::Button("Skip tool changing", [this, &ctl]() { output("ChangeTool"); }));

  if(!ctl.datastore().has("Femur"))
  {
    ctl.datastore().make<sva::PTransformd>("Femur", sva::PTransformd::Identity());
  }
  if(!ctl.datastore().has("Tibia"))
  {
    ctl.datastore().make<sva::PTransformd>("Tibia", sva::PTransformd::Identity());
  }

  // Computes calibration online
  run(ctl);
}

bool Calibrate::run(mc_control::fsm::Controller & ctl)
{
  // 1. Compute relative pose between the panda_tibia/panda_femur floating base
  // taking panda_tibia as the (arbitrary) reference
  auto & panda_femur = ctl.realRobot("panda_femur");
  auto & panda_tibia = ctl.realRobot("panda_tibia");
  auto X_0_pt = panda_tibia.posW();
  auto X_0_pf = panda_femur.posW();
  auto X_pt_tibia = panda_tibia.frame("TibiaCalibration").position() * X_0_pt.inv();
  auto X_pf_femur = panda_femur.frame("FemurCalibration").position() * X_0_pf.inv();
  auto X_pt_pf = X_pf_femur.inv() * X_pt_tibia;
  X_0_pf = X_pt_pf * X_0_pt;

  // 2. Set the panda_femur floating base
  ctl.realRobot("panda_femur").posW(X_0_pf);
  ctl.robot("panda_femur").posW(X_0_pf);

  // X_0_tibiaCalib = X_0_femurCalib
  // Axis of rotation: X_0_tibia
  ctl.datastore().assign("Tibia", ctl.robot("panda_tibia").frame("Tibia").position());
  ctl.datastore().assign("Femur", ctl.robot("panda_tibia").frame("Tibia").position());

  return output().size() != 0;
}

void Calibrate::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("Calibrate", Calibrate)
