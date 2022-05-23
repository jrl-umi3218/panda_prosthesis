#include "Choice.h"

#include <mc_control/fsm/Controller.h>

void Choice::start(mc_control::fsm::Controller & ctl)
{
  // Choices as pairs of name -> transition output
  auto choices = config_("choices", std::vector<std::pair<std::string, std::string>>{});
  auto category = config_("category", std::vector<std::string>{});

  if(choices.empty())
  {
    mc_rtc::log::error_and_throw("[{}] No choice provided", name());
  }

  for(const auto & choice : choices)
  {
    const auto & name = choice.first;
    const auto & transition = choice.second;
    ctl.gui()->addElement(this, category, mc_rtc::gui::Button(choice.first, [this, transition]() {
                            mc_rtc::log::info("[{}] Selected transition {}", this->name(), transition);
                            output(transition);
                          }));
  }
}

bool Choice::run(mc_control::fsm::Controller & ctl)
{
  return output().size() != 0;
}

void Choice::teardown(mc_control::fsm::Controller & ctl)
{
  auto category = config_("category", std::vector<std::string>{});
  ctl.gui()->removeElements(category, this);
}

EXPORT_SINGLE_STATE("Choice", Choice)
