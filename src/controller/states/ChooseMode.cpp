#include "ChooseMode.h"

#include <mc_control/fsm/Controller.h>

void ChooseMode::configure(const mc_rtc::Configuration & config) {}

void ChooseMode::start(mc_control::fsm::Controller & ctl)
{
  ctl.datastore().make<std::string>("Mode");
  ctl.gui()->addElement(this, {},
                        mc_rtc::gui::Button("Simulation",
                                            [&ctl, this]() {
                                              ctl.datastore().assign("Mode", std::string{"simulation"});
                                              output("Simulation");
                                            }),
                        mc_rtc::gui::Button("Real", [&ctl, this]() {
                          ctl.datastore().assign("Mode", std::string{"real"});
                          output("Real");
                        }));
}

bool ChooseMode::run(mc_control::fsm::Controller & ctl)
{
  return output().size() != 0;
}

void ChooseMode::teardown(mc_control::fsm::Controller & ctl)
{
  // FIXME (mc_rtc) only deletes one element instead of all
  // ctl.gui()->removeElements({}, this);
  ctl.gui()->removeElement({}, "Simulation");
  ctl.gui()->removeElement({}, "Real");
}

EXPORT_SINGLE_STATE("ChooseMode", ChooseMode)
