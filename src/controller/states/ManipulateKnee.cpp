#include "ManipulateKnee.h"

#include <mc_control/fsm/Controller.h>
#include <mc_filter/utils/clamp.h>
#include <mc_tasks/MetaTaskLoader.h>
#include "../3rd-party/csv.h"

void ReadCSV::clear()
{
  femurTranslationVector.clear();
  tibiaTranslationVector.clear();
  femurRotationVector.clear();
  tibiaRotationVector.clear();
}

void ReadCSV::load(const std::string & path)
{
  clear();
  io::CSVReader<12> in(path);
  in.read_header(io::ignore_extra_column, "femur_tangage", "femur_roulis", "femur_lacet", "tibia_tangage",
                 "tibia_roulis", "tibia_lacet", "femur_x", "femur_y", "femur_z", "tibia_x", "tibia_y", "tibia_z");
  // std::string vendor; int size; double speed;
  Eigen::Vector3d femurRotation, tibiaRotation, femurTranslation, tibiaTranslation;
  while(in.read_row(femurRotation[0], femurRotation[1], femurRotation[2], tibiaRotation[0], tibiaRotation[1],
                    tibiaRotation[2], femurTranslation[0], femurTranslation[1], femurTranslation[2],
                    tibiaTranslation[0], tibiaTranslation[1], tibiaTranslation[2]))
  {
    femurTranslationVector.push_back(femurTranslation);
    tibiaTranslationVector.push_back(tibiaTranslation);
    femurRotationVector.push_back(femurRotation);
    tibiaRotationVector.push_back(tibiaRotation);
  }
}

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

  setRate(config_("rate", 0.2), ctl.timeStep);

  if(config_.has("thresholds"))
  {
    auto c = config_("thresholds");
    if(c.has("translation"))
    {
      translationTreshold_ = c("translation");
    }
    if(c.has("rotation"))
    {
      rotationTreshold_ = c("rotation");
    }
  }

  if(config_.has("file"))
  {
    file_.load(config_("file"));
    play_ = true;
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

  ctl.gui()->addElement(this, {"ManipulateKnee", "Trajectory"},
                        mc_rtc::gui::Checkbox(
                            "Play", [this]() { return play_; }, [this]() { play_ = !play_; }),
                        mc_rtc::gui::NumberInput(
                            "Rate [s]", [this, &ctl]() { return getRate(ctl.timeStep); },
                            [this, &ctl](double rate) { setRate(rate, ctl.timeStep); }),
                        mc_rtc::gui::NumberInput(
                            "Translation Threshold [mm]", [this]() { return translationTreshold_; },
                            [this](double treshold) { translationTreshold_ = treshold; }),
                        mc_rtc::gui::NumberInput(
                            "Rotation Threshold [deg]", [this]() { return rotationTreshold_; },
                            [this](double treshold) { rotationTreshold_ = treshold; }));

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
  if(file_.tibiaRotationVector.empty())
  {
    play_ = false;
  }

  if(play_)
  {
    if(next_)
    {
      tibiaRotation_ = file_.tibiaRotationVector.front();
      tibiaTranslation_ = file_.tibiaTranslationVector.front();
      femurRotation_ = file_.femurRotationVector.front();
      femurTranslation_ = file_.femurTranslationVector.front();
      file_.tibiaRotationVector.pop_front();
      file_.tibiaTranslationVector.pop_front();
      file_.femurRotationVector.pop_front();
      file_.femurTranslationVector.pop_front();
      mc_rtc::log::info("Next pose");
      next_ = false;
    }
    if(iter_ == 0 || iter_ % iterRate_ == 0)
    {
      iter_ = 0;
      auto tibia_error = tibia_task_->eval();
      auto femur_error = femur_task_->eval();
      if(tibia_error.head<3>().norm() <= mc_rtc::constants::toRad(rotationTreshold_)
         && tibia_error.tail<3>().norm() <= translationTreshold_ / 1000.
         && femur_error.head<3>().norm() <= mc_rtc::constants::toRad(rotationTreshold_)
         && femur_error.tail<3>().norm() <= translationTreshold_ / 1000.)
      {
        measure();
        // Only go to next when all measurements are finished.
        next_ = true;
      }
    }
  }

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

  ++iter_;
  return output().size() != 0;
}

void ManipulateKnee::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.solver().removeTask(tibia_task_);
  ctl.solver().removeTask(femur_task_);
  ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("ManipulateKnee", ManipulateKnee)
