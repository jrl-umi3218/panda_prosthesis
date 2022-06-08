#include "ManipulateKnee.h"

#include <mc_control/fsm/Controller.h>
#include <mc_filter/utils/clamp.h>
#include <mc_tasks/MetaTaskLoader.h>

void ManipulateKnee::start(mc_control::fsm::Controller & ctl)
{
  if(config_.has("femur"))
  {
    const auto & c = config_("femur");
    c("minTranslation", minFemurTranslation_);
    c("maxTranslation", maxFemurTranslation_);
    c("minRotation", minFemurRotation_);
    c("maxRotation", maxFemurRotation_);
  }

  if(config_.has("tibia"))
  {
    const auto & c = config_("femur");
    c("minTranslation", minTibiaTranslation_);
    c("maxTranslation", maxTibiaTranslation_);
    c("minRotation", minTibiaRotation_);
    c("maxRotation", maxTibiaRotation_);
  }

  auto make_input = [this](mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category, const std::string & name,
                           const std::string & title, std::vector<std::string> axes, Eigen::Vector3d & rotation,
                           const Eigen::Vector3d & minRotation, const Eigen::Vector3d & maxRotation) {
    // clang-format off
    gui.addElement(
      this, category,
      mc_rtc::gui::ArrayInput(
          name + " " + title, axes,
          [this, &rotation]() -> const Eigen::Vector3d { return rotation; },
          [this, &rotation, &minRotation, &maxRotation](const Eigen::Vector3d & r) {
            rotation = mc_filter::utils::clamp(r, minRotation, maxRotation);
          }),
      mc_rtc::gui::NumberSlider(
          name + " " + axes[0],
          [this, &rotation]() { return rotation[0]; },
          [this, &rotation](double angle) { rotation[0] = angle; },
          minRotation[0], maxRotation[0]),
      mc_rtc::gui::NumberSlider(
          name + " " + axes[1],
          [this, &rotation]() { return rotation[1]; },
          [this, &rotation](double angle) { rotation[1] = angle; },
          minRotation[1], maxRotation[1]),
      mc_rtc::gui::NumberSlider(
          name + " " + axes[2],
          [this, &rotation]() { return rotation[2]; },
          [this, &rotation](double angle) { rotation[2] = angle; },
          minRotation[2], maxRotation[2])
      );
    // clang-format on
  };

  auto & gui = *ctl.gui();
  // clang-format off
  make_input(gui, {"ManipulateKnee", "Tibia"}, "Tibia", "Translation", {"x [mm]", "y[mm]", "z[mm]"}, tibiaTranslation_, minTibiaTranslation_, maxTibiaTranslation_);
  make_input(gui, {"ManipulateKnee", "Tibia"}, "Tibia", "Rotation", {"Roll [deg]", "Pitch [deg]", "Yaw [deg]"}, tibiaRotation_,  minTibiaRotation_, maxTibiaRotation_);
  make_input(gui, {"ManipulateKnee", "Femur"}, "Femur", "Translation", {"x [mm]", "y[mm]", "z[mm]"}, femurTranslation_, minFemurTranslation_, maxFemurTranslation_);
  make_input(gui, {"ManipulateKnee", "Femur"}, "Femur", "Rotation", {"Roll [deg]", "Pitch [deg]", "Yaw [deg]"}, femurRotation_,  minFemurRotation_, maxFemurRotation_);
  // clang-format on

  gui.addElement(this, {}, mc_rtc::gui::Button("Finished", [this]() { output("Finished"); }));

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

  std::tie(femur_error_, femur_angle_) = handle_motion(*femur_task_, X_0_tibiaFrame, femurTranslation_, femurRotation_);
  std::tie(tibia_error_, tibia_angle_) = handle_motion(*tibia_task_, X_0_tibiaFrame, tibiaTranslation_, tibiaRotation_);

  return output().size() != 0;
}

void ManipulateKnee::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.solver().removeTask(tibia_task_);
  ctl.solver().removeTask(femur_task_);
  ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("ManipulateKnee", ManipulateKnee)
