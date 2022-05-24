#include "ManipulateKnee.h"

#include <mc_control/fsm/Controller.h>
#include <mc_filter/utils/clamp.h>
#include <mc_tasks/MetaTaskLoader.h>

void ManipulateKnee::start(mc_control::fsm::Controller & ctl)
{
  config_("min", min_);
  config_("max", max_);
  auto make_slider = [this](const std::string & name, size_t axis) {
    return mc_rtc::gui::NumberSlider(
        name, [this, axis]() { return rotation_[axis]; }, [this, axis](double angle) { rotation_[axis] = angle; },
        min_[axis], max_[axis]);
  };
  ctl.gui()->addElement(
      this, {},
      mc_rtc::gui::ArrayInput(
          "Rotation Angle [deg]", {"Roll", "Pitch", "Yaw"}, [this]() -> const Eigen::Vector3d { return rotation_; },
          [this](const Eigen::Vector3d & rotation) { rotation_ = mc_filter::utils::clamp(rotation, min_, max_); }),
      make_slider("Roll [deg]", 0), make_slider("Pitch [deg]", 1), make_slider("Yaw [deg]", 2),
      mc_rtc::gui::NumberSlider(
          "Percent Femur", [this]() { return percentFemur; }, [this](double percent) { percentFemur = percent; }, 0.,
          1.),
      mc_rtc::gui::ArrayInput(
          "Min Angle [deg]", {"Roll", "Pitch", "Yaw"}, [this]() -> const Eigen::Vector3d { return min_; },
          [this](const Eigen::Vector3d & rotation) { min_ = rotation; }),
      mc_rtc::gui::ArrayInput(
          "Max Angle [deg]", {"Roll", "Pitch", "Yaw"}, [this]() -> const Eigen::Vector3d { return max_; },
          [this](const Eigen::Vector3d & rotation) { max_ = rotation; }),
      mc_rtc::gui::Button("Finished", [this]() { output("Finished"); }));

  tibia_task_ = mc_tasks::MetaTaskLoader::load<mc_tasks::TransformTask>(ctl.solver(), config_("TibiaTask"));
  tibia_task_->reset();
  femur_task_ = mc_tasks::MetaTaskLoader::load<mc_tasks::TransformTask>(ctl.solver(), config_("FemurTask"));
  femur_task_->reset();
  ctl.solver().addTask(tibia_task_);
  ctl.solver().addTask(femur_task_);

  run(ctl);
}

bool ManipulateKnee::run(mc_control::fsm::Controller & ctl)
{
  Eigen::Vector3d femurRotationRad = percentFemur * mc_rtc::constants::PI / (180.) * rotation_;
  Eigen::Vector3d tibiaRotationRad = -(1 - percentFemur) * mc_rtc::constants::PI / (180.) * rotation_;

  const auto & X_0_tibiaFrame = ctl.datastore().get<sva::PTransformd>("Tibia");
  auto X_0_tibiaTarget = sva::PTransformd(mc_rbdyn::rpyToMat(tibiaRotationRad)) * X_0_tibiaFrame;
  const auto & X_0_femurFrame = X_0_tibiaFrame;
  auto X_0_femurTarget = sva::PTransformd(mc_rbdyn::rpyToMat(femurRotationRad)) * X_0_femurFrame;
  tibia_task_->target(X_0_tibiaTarget);
  femur_task_->target(X_0_femurTarget);

  return output().size() != 0;
}

void ManipulateKnee::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.solver().removeTask(tibia_task_);
  ctl.solver().removeTask(femur_task_);
  ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("ManipulateKnee", ManipulateKnee)
