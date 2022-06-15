#include "ManipulateKnee.h"

#include <mc_control/fsm/Controller.h>
#include <mc_filter/utils/clamp.h>
#include <mc_rtc/io_utils.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <boost/filesystem.hpp>
#include "../3rd-party/csv.h"

namespace fs = boost::filesystem;

double truncate(double value, double precision = 2)
{
  return (floor((value * pow(10, precision) + 0.5)) / pow(10, precision));
}

template<typename VectorT>
VectorT truncate(const VectorT & value, double precision = 2)
{
  VectorT r;
  for(unsigned i = 0; i < value.size(); ++i)
  {
    r[i] = truncate(value[i], precision);
  }
  return r;
}

/**
 * \brief   Return the filenames of all files that have the specified extension
 *          in the specified directory and all subdirectories.
 */
std::vector<std::string> get_all(fs::path const & root, std::string const & ext)
{
  std::vector<std::string> paths;

  if(fs::exists(root) && fs::is_directory(root))
  {
    for(auto const & entry : fs::recursive_directory_iterator(root))
    {
      if(fs::is_regular_file(entry) && entry.path().extension() == ext)
        paths.emplace_back(entry.path().filename().c_str());
    }
  }

  return paths;
}

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

std::string Result::to_csv() const
{
  return mc_rtc::io::to_string(
             std::vector<double>{
                 femurRotation.x(),
                 femurRotation.y(),
                 femurRotation.z(),
                 tibiaRotation.x(),
                 tibiaRotation.y(),
                 tibiaRotation.z(),
                 femurTranslation.x(),
                 femurTranslation.y(),
                 femurTranslation.z(),
                 tibiaTranslation.x(),
                 tibiaTranslation.y(),
                 tibiaTranslation.z(),
             },
             ",")
         + "," + mc_rtc::io::to_string(sensorData);
}

void ResultHandler::write_csv(const std::string & path)
{
  if(results_.empty()) return;
  std::ofstream csv;
  csv.open(path);
  if(!csv)
  {
    mc_rtc::log::error("Failed to write results to CSV file {}", path);
  }

  csv << "femur_tangage,femur_roulis,femur_lacet,tibia_tangage,tibia_roulis,tibia_lacet,femur_x,femur_y,femur_z,tibia_"
         "x,tibia_y,tibia_z,sensor_0,sensor_1,sensor_2,sensor_3,sensor_4,sensor_5,sensor_6,sensor_7,sensor_8,sensor_9"
      << std::endl;

  for(const auto & result : results_)
  {
    csv << result.to_csv() << std::endl;
  }

  csv.close();
  mc_rtc::log::success("Results written to {}", path);
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
    const auto & c = config_("tibia");
    c("minTranslation", minTibiaTranslation_);
    c("maxTranslation", maxTibiaTranslation_);
    c("minRotation", minTibiaRotation_);
    c("maxRotation", maxTibiaRotation_);
  }

  setRate(config_("rate", 0.2), ctl.timeStep);
  config_("samples", desiredSamples_);

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

  ctl.config()("trajectory_dir", trajectory_dir_);
  ctl.config()("results_dir", results_dir_);

  auto make_input = [this](mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category, const std::string & name,
                           const std::string & title, std::vector<std::string> axes, Eigen::Vector3d & rotation,
                           const Eigen::Vector3d & minRotation, const Eigen::Vector3d & maxRotation,
                           const Eigen::Vector3d & actual) {
    // clang-format off
    gui.addElement(
      this, category,
      mc_rtc::gui::ArrayInput(
          name + " " + title, axes,
          [this, &rotation]() -> Eigen::Vector3d { return rotation; },
          [this, &rotation, &minRotation, &maxRotation](const Eigen::Vector3d & r) {
            rotation = mc_filter::utils::clamp(r, minRotation, maxRotation);
          }),
      mc_rtc::gui::ArrayLabel(
          name + " " + title + " Actual", axes,
          [this, &actual]() -> const Eigen::Vector3d { return truncate(actual, 1); }),
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
  make_input(gui, {"ManipulateKnee", "Tibia"}, "Tibia", "Translation", {"x [mm]", "y[mm]", "z[mm]"}, tibiaTranslation_, minTibiaTranslation_, maxTibiaTranslation_, tibiaTranslationActual_);
  make_input(gui, {"ManipulateKnee", "Tibia"}, "Tibia", "Rotation", {"Roll [deg]", "Pitch [deg]", "Yaw [deg]"}, tibiaRotation_,  minTibiaRotation_, maxTibiaRotation_, tibiaRotationActual_);
  make_input(gui, {"ManipulateKnee", "Femur"}, "Femur", "Translation", {"x [mm]", "y[mm]", "z[mm]"}, femurTranslation_, minFemurTranslation_, maxFemurTranslation_, femurTranslationActual_);
  make_input(gui, {"ManipulateKnee", "Femur"}, "Femur", "Rotation", {"Roll [deg]", "Pitch [deg]", "Yaw [deg]"}, femurRotation_,  minFemurRotation_, maxFemurRotation_, femurRotationActual_);
  // clang-format on

  ctl.gui()->addElement(this, {"ManipulateKnee"}, mc_rtc::gui::Button("Reset to Zero", [this]() {
                          resetToZero();
                          stop();
                        }));

  ctl.gui()->addElement(this, {"ManipulateKnee", "Offsets"},
                        mc_rtc::gui::ArrayInput(
                            "Tibia Offset Translation", {"x [mm]", "y [mm]", "z [mm]"},
                            [this]() -> Eigen::Vector3d { return tibiaOffset_.translation() * 1000; },
                            [this, &ctl](const Eigen::Vector3d & t) {
                              tibiaOffset_.translation() = t / 1000;
                              updateAxes(ctl);
                            }),
                        mc_rtc::gui::ArrayInput(
                            "Tibia Offset Rotation", {"r [deg]", "p [deg]", "y [deg]"},
                            [this]() -> Eigen::Vector3d {
                              return mc_rbdyn::rpyFromMat(tibiaOffset_.rotation()) * 180 / mc_rtc::constants::PI;
                            },
                            [this, &ctl](const Eigen::Vector3d & r) {
                              tibiaOffset_.rotation() = mc_rbdyn::rpyToMat(r * mc_rtc::constants::PI / 180);
                              updateAxes(ctl);
                            }),
                        mc_rtc::gui::ArrayInput(
                            "Femur Offset Translation", {"x", "y", "z"},
                            [this]() -> Eigen::Vector3d { return femurOffset_.translation() * 1000; },
                            [this, &ctl](const Eigen::Vector3d & t) {
                              femurOffset_.translation() = t / 1000;
                              updateAxes(ctl);
                            }),
                        mc_rtc::gui::ArrayInput(
                            "Femur Offset Rotation", {"r [deg]", "p [deg]", "y [deg]"},
                            [this]() -> Eigen::Vector3d {
                              return mc_rbdyn::rpyFromMat(femurOffset_.rotation()) * 180 / mc_rtc::constants::PI;
                            },
                            [this, &ctl](const Eigen::Vector3d & r) {
                              femurOffset_.rotation() = mc_rbdyn::rpyToMat(r * mc_rtc::constants::PI / 180);
                              updateAxes(ctl);
                            }));

  // Load scenes from disk
  auto scene_files = get_all(trajectory_dir_, ".csv");
  mc_rtc::log::info("[{}] Looking for trajectory files in \"{}\"", name(), trajectory_dir_);
  if(scene_files.size())
  {
    mc_rtc::log::info("Found trajectory files: {}", mc_rtc::io::to_string(scene_files));
  }
  else
  {
    mc_rtc::log::warning("No trajectory file found in \"{}\" (expected extension .csv)", trajectory_dir_);
  }

  ctl.gui()->addElement(this, {"ManipulateKnee", "Trajectory"}, mc_rtc::gui::ElementsStacking::Horizontal,
                        mc_rtc::gui::ComboInput(
                            "Trajectory", scene_files, [&ctl, this]() { return trajectory_file_; },
                            [this](const std::string & name) {
                              stop();
                              trajectory_file_ = name;
                              file_.load(trajectory_dir_ + "/" + name);
                            }),
                        mc_rtc::gui::Button("Play",
                                            [this]() {
                                              stop();
                                              file_.load(trajectory_dir_ + "/" + trajectory_file_);
                                              play();
                                            }),
                        mc_rtc::gui::Checkbox(
                            "Pause", [this]() { return !play_; },
                            [this]() {
                              if(play_)
                              {
                                stop();
                              }
                              else
                              {
                                play();
                              }
                            })

  );
  ctl.gui()->addElement(
      this, {"ManipulateKnee", "Trajectory"}, mc_rtc::gui::Button("Force Next", [this]() { forceNext(); }),
      mc_rtc::gui::Label("Waypoints Remaining", [this]() { return std::to_string(file_.tibiaRotationVector.size()); }),
      mc_rtc::gui::NumberInput(
          "Rate [s]", [this, &ctl]() { return getRate(ctl.timeStep); },
          [this, &ctl](double rate) { setRate(rate, ctl.timeStep); }),
      mc_rtc::gui::NumberInput(
          "Translation Threshold [mm]", [this]() { return translationTreshold_; },
          [this](double treshold) { translationTreshold_ = treshold; }),
      mc_rtc::gui::NumberInput(
          "Rotation Threshold [deg]", [this]() { return rotationTreshold_; },
          [this](double treshold) { rotationTreshold_ = treshold; }));

  tibia_task_ = mc_tasks::MetaTaskLoader::load<mc_tasks::TransformTask>(ctl.solver(), config_("TibiaTask"));
  tibia_task_->reset();
  femur_task_ = mc_tasks::MetaTaskLoader::load<mc_tasks::TransformTask>(ctl.solver(), config_("FemurTask"));
  femur_task_->reset();
  ctl.solver().addTask(tibia_task_);
  ctl.solver().addTask(femur_task_);

  if(ctl.config().has("offsets"))
  {
    ctl.config()("offsets")("tibia", tibiaOffset_);
  }
  if(ctl.config().has("offsets"))
  {
    ctl.config()("offsets")("femur", femurOffset_);
  }
  updateAxes(ctl);

  run(ctl);
}

void ManipulateKnee::resetToZero()
{
  tibiaRotation_.setZero();
  tibiaTranslation_.setZero();
  femurRotation_.setZero();
  femurTranslation_.setZero();
  femurRotationActual_.setZero();
  femurTranslationActual_.setZero();
  tibiaRotationActual_.setZero();
  tibiaTranslationActual_.setZero();
}

void ManipulateKnee::updateAxes(mc_control::fsm::Controller & ctl)
{
  const auto & X_0_tibiaFrame = ctl.datastore().get<sva::PTransformd>("Tibia");
  const auto & X_0_femurFrame = ctl.datastore().get<sva::PTransformd>("Femur");
  X_0_femurAxis = femurOffset_ * X_0_femurFrame;
  X_0_tibiaAxis = tibiaOffset_ * X_0_tibiaFrame;
}

bool ManipulateKnee::measure(mc_control::fsm::Controller & ctl)
{
  if(!ctl.datastore().has("BoneTagSerialPlugin"))
  {
    return true;
  }
  else if(!ctl.datastore().call<bool>("BoneTagSerialPlugin::Connected"))
  {
    return true;
  }

  auto sensorData = ctl.datastore().call<std::optional<io::BoneTagSerial::Data>>("BoneTagSerialPlugin::GetNewData");
  if(sensorData)
  {
    // mc_rtc::log::info("Got new data");
    Result result;
    result.femurRotation = femurRotationActual_;
    result.femurTranslation = femurTranslationActual_;
    result.tibiaRotation = tibiaRotationActual_;
    result.tibiaTranslation = tibiaTranslationActual_;
    result.sensorData = *sensorData;
    results_.addResult(result);
    ++measuredSamples_;
  }
  return measuredSamples_ == desiredSamples_;
}

bool ManipulateKnee::run(mc_control::fsm::Controller & ctl)
{
  if(file_.tibiaRotationVector.empty())
  {
    if(play_)
    {
      saveResults();
    }
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
      iter_ = 0;
      measuredSamples_ = 0;
      next_ = false;
    }

    auto tibia_error = tibia_task_->eval();
    auto femur_error = femur_task_->eval();
    bool hasConverged = (tibia_error.head<3>().norm() <= mc_rtc::constants::toRad(rotationTreshold_)
                         && tibia_error.tail<3>().norm() <= translationTreshold_ / 1000.
                         && femur_error.head<3>().norm() <= mc_rtc::constants::toRad(rotationTreshold_)
                         && femur_error.tail<3>().norm() <= translationTreshold_ / 1000.);

    // Only go to next when all measurements are finished.
    next_ = hasConverged && iter_ >= iterRate_ && measure(ctl);
  }

  auto handle_motion = [](mc_tasks::TransformTask & task, const sva::PTransformd X_0_axisFrame,
                          Eigen::Vector3d translation, Eigen::Vector3d rotation, Eigen::Vector3d & translationActual,
                          Eigen::Vector3d & rotationActual) {
    translation *= 0.001; // translation in [m]
    rotation *= mc_rtc::constants::PI / 180.; // rotation in [rad]

    auto X_0_target = sva::PTransformd(mc_rbdyn::rpyToMat(rotation), translation) * X_0_axisFrame;
    task.target(X_0_target);

    auto X_0_actual = task.frame().position();
    auto X_axisFrame_actual = X_0_actual * X_0_axisFrame.inv();

    translationActual = X_axisFrame_actual.translation() * 1000;
    // XXX should we store as RPY?
    rotationActual = mc_rbdyn::rpyFromMat(X_axisFrame_actual.rotation()) * 180 / mc_rtc::constants::PI;
  };

  handle_motion(*femur_task_, X_0_femurAxis, femurTranslation_, femurRotation_, femurTranslationActual_,
                femurRotationActual_);
  handle_motion(*tibia_task_, X_0_tibiaAxis, tibiaTranslation_, tibiaRotation_, tibiaTranslationActual_,
                tibiaRotationActual_);

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
