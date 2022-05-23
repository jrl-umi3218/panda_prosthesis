#include "Calibrate.h"

#include <mc_control/fsm/Controller.h>

void Calibrate::configure(const mc_rtc::Configuration & config) {}

void Calibrate::start(mc_control::fsm::Controller & ctl)
{
  // Computes calibration online
  run(ctl);

  ctl.gui()->addElement({}, mc_rtc::gui::Button("Save calibration", [this, &ctl]() {}),
                        mc_rtc::gui::Button("Next", [this, &ctl]() { output("OK"); }));
}

bool Calibrate::run(mc_control::fsm::Controller & ctl)
{
  // 1. Compute relative pose between the panda_tibia/panda_femur floating base
  // taking panda_tibia as the (arbitrary) reference
  auto & panda_femur = ctl.realRobot("panda_femur");
  auto & panda_tibia = ctl.realRobot("panda_tibia");
  auto X_0_pt = panda_tibia.posW();
  auto X_0_pf = panda_femur.posW();
  auto X_pt_tibia = panda_tibia.frame("TibiaHead").position() * X_0_pt.inv();
  auto X_pf_femur = panda_femur.frame("FemurHead").position() * X_0_pf.inv();
  auto X_pt_pf = X_pf_femur.inv() * X_pt_tibia;
  X_0_pf = X_pt_pf * X_0_pt;

  // 2. Set the panda_femur floating base
  ctl.realRobot("panda_femur").posW(X_0_pf);

  return output().size() != 0;
}

void Calibrate::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("Calibrate", Calibrate)
