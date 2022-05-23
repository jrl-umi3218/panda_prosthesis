#include "TransformDatastore.h"

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/MetaTaskLoader.h>

void TransformDatastore::start(mc_control::fsm::Controller & ctl)
{
  datastorePose_ = static_cast<std::string>(config_("datastoreTarget"));
  if(!ctl.datastore().has(datastorePose_))
  {
    mc_rtc::log::error_and_throw("[{}] No target \"{}\" on the datastore", name(), datastorePose_);
  }

  config_("continuous", continuous_);

  task_ = mc_tasks::MetaTaskLoader::load<mc_tasks::TransformTask>(ctl.solver(), config_("TransformTask"));
  task_->target(ctl.datastore().get<sva::PTransformd>(datastorePose_));
  ctl.solver().addTask(task_);

  if(config_.has("completion"))
  {
    criteria_config_.load(config_("completion"));
  }
  criteria_.configure(*task_, ctl.timeStep, criteria_config_);

  output("OK");
  run(ctl);
}

bool TransformDatastore::run(mc_control::fsm::Controller & ctl)
{
  if(continuous_)
  {
    task_->target(ctl.datastore().get<sva::PTransformd>(datastorePose_));
  }
  return criteria_.completed(*task_);
}

void TransformDatastore::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.solver().removeTask(task_);
}

EXPORT_SINGLE_STATE("TransformDatastore", TransformDatastore)
