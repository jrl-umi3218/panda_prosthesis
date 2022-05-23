#include "Calibrate.h"

#include <mc_control/fsm/Controller.h>

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
  if(!ctl.config().has("ETC_DIR") && ctl.config()("ETC_DIR").empty())
  {
    mc_rtc::log::error_and_throw("[{}] No \"ETC_DIR\"  entry specified", name());
  }
  auto etc_file_panda_femur = static_cast<std::string>(ctl.config()("ETC_DIR")) + "/initial_panda_femur.yaml";
  auto etc_file_panda_tibia = static_cast<std::string>(ctl.config()("ETC_DIR")) + "/initial_panda_tibia.yaml";
  auto save_calibration = [&ctl, etc_file_panda_tibia, etc_file_panda_femur]() {
    save(etc_file_panda_femur, ctl.robot("panda_femur"));
    save(etc_file_panda_tibia, ctl.robot("panda_tibia"));
  };
  ctl.gui()->addElement(this, {}, mc_rtc::gui::Button("Save calibration", save_calibration));
  ctl.gui()->addElement(this, {}, mc_rtc::gui::Button("Save calibration and change tool", [this, save_calibration]() {
                          save_calibration();
                          output("ChangeTool");
                        }));
  ctl.gui()->addElement(this, {},
                        mc_rtc::gui::Button("Change Tool without saving", [this, &ctl]() { output("ChangeTool"); }));

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

  ctl.datastore().assign("Tibia", ctl.robot("panda_tibia").frame("Tibia").position());
  ctl.datastore().assign("Femur", ctl.robot("panda_femur").frame("Femur").position());

  return output().size() != 0;
}

void Calibrate::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("Calibrate", Calibrate)
