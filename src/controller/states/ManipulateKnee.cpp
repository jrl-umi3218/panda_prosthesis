#include "ManipulateKnee.h"

#include <mc_control/fsm/Controller.h>
#include <mc_filter/utils/clamp.h>
#include <mc_tasks/MetaTaskLoader.h>

void ManipulateKnee::start(mc_control::fsm::Controller & ctl)
{
  config_("min", minRotation_);
  config_("max", maxRotation_);
  config_("minFemurTranslation", minFemurTranslation_);
  config_("minTibiaTranslation", minTibiaTranslation_);
  config_("maxFemurTranslation", maxFemurTranslation_);
  config_("maxTibiaTranslation", maxTibiaTranslation_);
  auto make_slider = [this](const std::string & name, size_t axis) {
    return mc_rtc::gui::NumberSlider(
        name, [this, axis]() { return rotation_[axis]; }, [this, axis](double angle) { rotation_[axis] = angle; },
        minRotation_[axis], maxRotation_[axis]);
  };
  ctl.gui()->addElement(
      this, {},
      mc_rtc::gui::ArrayInput(
          "Femur Translation [mm]", {"x", "y", "z"}, [this]() -> const Eigen::Vector3d & { return femurTranslation_; },
          [this](const Eigen::Vector3d & translation) {
            femurTranslation_ = mc_filter::utils::clamp(translation, minFemurTranslation_, maxFemurTranslation_);
          }),
      mc_rtc::gui::ArrayInput(
          "Tibia Translation [mm]", {"x", "y", "z"}, [this]() -> const Eigen::Vector3d & { return tibiaTranslation_; },
          [this](const Eigen::Vector3d & translation) {
            tibiaTranslation_ = mc_filter::utils::clamp(translation, minTibiaTranslation_, maxTibiaTranslation_);
          }),
      mc_rtc::gui::ArrayInput(
          "Rotation Angle [deg]", {"Roll", "Pitch", "Yaw"}, [this]() -> const Eigen::Vector3d { return rotation_; },
          [this](const Eigen::Vector3d & rotation) {
            rotation_ = mc_filter::utils::clamp(rotation, minRotation_, maxRotation_);
          }),
      make_slider("Roll [deg]", 0), make_slider("Pitch [deg]", 1), make_slider("Yaw [deg]", 2),
      mc_rtc::gui::NumberSlider(
          "Percent Femur", [this]() { return percentFemur; }, [this](double percent) { percentFemur = percent; }, 0.,
          1.),
      mc_rtc::gui::ArrayInput(
          "Min Angle [deg]", {"Roll", "Pitch", "Yaw"}, [this]() -> const Eigen::Vector3d { return minRotation_; },
          [this](const Eigen::Vector3d & rotation) { minRotation_ = rotation; }),
      mc_rtc::gui::ArrayInput(
          "Max Angle [deg]", {"Roll", "Pitch", "Yaw"}, [this]() -> const Eigen::Vector3d { return maxRotation_; },
          [this](const Eigen::Vector3d & rotation) { maxRotation_ = rotation; }),
      mc_rtc::gui::Button("Finished", [this]() { output("Finished"); }));

  ctl.gui()->addElement(
      this, {"ManipulateKnee", "Angle"},
      mc_rtc::gui::ArrayLabel("Tibia Position [m]", {"tx", "ty", "tz"},
                              [this]() -> const Eigen::Vector3d & { return tibia_angle_.translation(); }),
      mc_rtc::gui::ArrayLabel("Tibia Rotation [deg]", {"r", "p", "y"},
                              [this]() -> Eigen::Vector3d {
                                return 180 / mc_rtc::constants::PI * mc_rbdyn::rpyFromMat(tibia_angle_.rotation());
                              }),
      mc_rtc::gui::ArrayLabel("Femur Position [m]", {"tx", "ty", "tz"},
                              [this]() -> const Eigen::Vector3d & { return femur_angle_.translation(); }),
      mc_rtc::gui::ArrayLabel("Femur Rotation [deg]", {"r", "p", "y"}, [this]() -> Eigen::Vector3d {
        return 180 / mc_rtc::constants::PI * mc_rbdyn::rpyFromMat(femur_angle_.rotation());
      }));

  ctl.gui()->addElement(
      this, {"ManipulateKnee", "Error"},
      mc_rtc::gui::ArrayLabel("Tibia Error Position [m]", {"tx", "ty", "tz"},
                              [this]() -> const Eigen::Vector3d & { return tibia_error_.translation(); }),
      mc_rtc::gui::ArrayLabel("Tibia Error Rotation [deg]", {"r", "p", "y"},
                              [this]() -> Eigen::Vector3d {
                                return 180 / mc_rtc::constants::PI * mc_rbdyn::rpyFromMat(tibia_error_.rotation());
                              }),
      mc_rtc::gui::ArrayLabel("Femur Error Position [m]", {"tx", "ty", "tz"},
                              [this]() -> const Eigen::Vector3d & { return femur_error_.translation(); }),
      mc_rtc::gui::ArrayLabel("Femur Error Rotation [deg]", {"r", "p", "y"}, [this]() -> Eigen::Vector3d {
        return 180 / mc_rtc::constants::PI * mc_rbdyn::rpyFromMat(femur_error_.rotation());
      }));

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
  // Rotation axis for the knee joint
  // For now we rotate around the Tibia frame obtained during calibration
  const auto & X_0_tibiaFrame = ctl.datastore().get<sva::PTransformd>("Tibia");
  Eigen::Vector3d femurRotation = percentFemur * rotation_;
  Eigen::Vector3d tibiaRotation = -(1 - percentFemur) * rotation_;

  auto handle_motion = [](mc_tasks::TransformTask & task, const sva::PTransformd X_0_axisFrame,
                          Eigen::Vector3d translation, Eigen::Vector3d rotation) {
    translation *= 0.001; // translation in [m]
    rotation *= mc_rtc::constants::PI / 180.; // rotation in [rad]

    auto X_0_target = sva::PTransformd(mc_rbdyn::rpyToMat(rotation), translation) * X_0_axisFrame;
    task.target(X_0_target);

    auto X_0_actual = task.frame().position();
    // Error between current state and target
    auto error = X_0_actual * X_0_target.inv();
    // Angle between current state and axis
    auto angleActual = X_0_actual * X_0_target.inv();
    return std::tuple<sva::PTransformd, sva::PTransformd>{error, angleActual};
  };

  std::tie(femur_error_, femur_angle_) = handle_motion(*femur_task_, X_0_tibiaFrame, femurTranslation_, femurRotation);
  std::tie(tibia_error_, tibia_angle_) = handle_motion(*tibia_task_, X_0_tibiaFrame, tibiaTranslation_, tibiaRotation);

  return output().size() != 0;
}

void ManipulateKnee::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.solver().removeTask(tibia_task_);
  ctl.solver().removeTask(femur_task_);
  ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("ManipulateKnee", ManipulateKnee)
