#include "Calibrate.h"

#include <mc_control/fsm/Controller.h>

void Calibrate::start(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->addElement(this, {}, mc_rtc::gui::Button("Save calibration", [this, &ctl]() {}),
                        mc_rtc::gui::Button("Change Tool", [this, &ctl]() { output("ChangeTool"); }));

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
