#include "TransformDatastore.h"

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/MetaTaskLoader.h>

void save(const std::string & etc_file, const mc_rbdyn::Robot & robot)
{
  mc_rtc::Configuration initial(etc_file);
  initial.add(robot.name());
  initial(robot.name()).add("pose", robot.posW());
  initial(robot.name()).add("joints", robot.mbc().q);
  initial.save(etc_file);
  mc_rtc::log::success("Calibration saved to {}", etc_file);
}

void TransformDatastore::start(mc_control::fsm::Controller & ctl)
{
  datastorePose_ = static_cast<std::string>(config_("datastoreTarget"));
  if(!ctl.datastore().has(datastorePose_))
  {
    mc_rtc::log::error_and_throw("[{}] No target \"{}\" on the datastore", name(), datastorePose_);
  }

  config_("continuous", continuous_);
  robot_ = static_cast<std::string>(config_("TransformTask")("robot"));
  config_("save", save_);

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
