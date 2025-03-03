#include "ManipulateKnee.h"

#include <mc_control/fsm/Controller.h>
#include <mc_filter/utils/clamp.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/io_utils.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <boost/filesystem.hpp>
#include <3rd-party/csv.h>

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
                           const Eigen::Vector3d & actual)
  {
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
  make_input(gui, {"ManipulateKnee", "Tibia", "Translation [mm]"}, "Tibia", "Translation [mm]", {"Left (x)", "Forward (y)", "Down (z)"}, tibiaTranslation_, minTibiaTranslation_, maxTibiaTranslation_, tibiaTranslationActual_);
  make_input(gui, {"ManipulateKnee", "Tibia", "Rotation [deg]"}, "Tibia", "Rotation [deg]", {"Tangage", "Roulis", "Lacet"}, tibiaRotation_,  minTibiaRotation_, maxTibiaRotation_, tibiaRotationActual_);
  make_input(gui, {"ManipulateKnee", "Femur", "Translation [mm]"}, "Femur", "Translation [mm]", {"Left (x)", "Forward (y)", "Down (z)"}, femurTranslation_, minFemurTranslation_, maxFemurTranslation_, femurTranslationActual_);
  make_input(gui, {"ManipulateKnee", "Femur", "Rotation [deg]"}, "Femur", "Rotation [deg]", {"Tangage", "Roulis", "Lacet"}, femurRotation_,  minFemurRotation_, maxFemurRotation_, femurRotationActual_);
  // clang-format on

  ctl.gui()->addElement(this, {"ManipulateKnee"},
                        mc_rtc::gui::Button("Reset to Zero",
                                            [this]()
                                            {
                                              resetToZero();
                                              stop();
                                            }));

  ctl.gui()->addElement(
      this, {"ManipulateKnee", "Offsets", "Tibia"},
      mc_rtc::gui::ArrayInput(
          "Tibia Offset",
          {"Tangage [deg]", "Roulis [deg]", "Lacet [deg]", "Left (x) [mm]", "Forward (y) [mm]", "Down (z) [mm]"},
          [this]() -> Eigen::Vector6d
          {
            Eigen::Vector6d res;
            Eigen::Vector3d translation = tibiaOffset_.translation() * 1000;
            Eigen::Vector3d rotation = mc_rbdyn::rpyFromMat(tibiaOffset_.rotation()) * 180 / mc_rtc::constants::PI;
            res[0] = rotation.x();
            res[1] = rotation.y();
            res[2] = rotation.z();
            res[3] = translation.x();
            res[4] = translation.y();
            res[5] = translation.z();
            return res;
          },
          [this, &ctl](const Eigen::Vector6d & offset)
          {
            Eigen::Vector3d r;
            r.x() = offset[0];
            r.y() = offset[1];
            r.z() = offset[2];
            Eigen::Vector3d trans;
            trans.x() = offset[3];
            trans.y() = offset[4];
            trans.z() = offset[5];
            updateTibiaOffset({mc_rbdyn::rpyToMat(r * mc_rtc::constants::PI / 180), trans / 1000});
          }),
      mc_rtc::gui::ArrayInput(
          "Femur Offset",
          {"Tangage [deg]", "Roulis [deg]", "Lacet [deg]", "Left (x) [mm]", "Forward (y) [mm]", "Down (z) [mm]"},
          [this]() -> Eigen::Vector6d
          {
            Eigen::Vector6d res;
            Eigen::Vector3d translation = femurOffset_.translation() * 1000;
            Eigen::Vector3d rotation = mc_rbdyn::rpyFromMat(femurOffset_.rotation()) * 180 / mc_rtc::constants::PI;
            res[0] = rotation.x();
            res[1] = rotation.y();
            res[2] = rotation.z();
            res[3] = translation.x();
            res[4] = translation.y();
            res[5] = translation.z();
            return res;
          },
          [this, &ctl](const Eigen::Vector6d & offset)
          {
            Eigen::Vector3d trans;
            trans.x() = offset[3];
            trans.y() = offset[4];
            trans.z() = offset[5];
            Eigen::Vector3d r;
            r.x() = offset[0];
            r.y() = offset[1];
            r.z() = offset[2];
            updateFemurOffset({mc_rbdyn::rpyToMat(r * mc_rtc::constants::PI / 180), trans / 1000});
          }),
      mc_rtc::gui::NumberInput(
          "Interpolation duration [s]", [this]() { return offsetDuration_; },
          [this](double duration) { offsetDuration_ = duration; }));
  ctl.gui()->addElement(this, {"ManipulateKnee", "Offsets"},
                        mc_rtc::gui::Button("Reset Offsets",
                                            [this, &ctl]()
                                            {
                                              updateTibiaOffset(tibiaOffsetInitial_);
                                              updateFemurOffset(femurOffsetInitial_);
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
                            [this](const std::string & name)
                            {
                              stop();
                              trajectory_file_ = name;
                              file_.load(trajectory_dir_ + "/" + name);
                            }),
                        mc_rtc::gui::Button("Play",
                                            [this]()
                                            {
                                              stop();
                                              file_.load(trajectory_dir_ + "/" + trajectory_file_);
                                              play();
                                            }));

  ctl.gui()->addElement(
      this, {"ManipulateKnee", "Trajectory"},
      mc_rtc::gui::Label("Waypoints Remaining", [this]() { return std::to_string(file_.tibiaRotationVector.size()); }),
      mc_rtc::gui::Label("Offsets",
                         [this]()
                         {
                           if(repeatWithOffset_)
                           {
                             return std::to_string(trajOffsets_.current()) + " / "
                                    + std::to_string(trajOffsets_.size());
                           }
                           else
                           {
                             return std::string{"none"};
                           }
                         }));

  ctl.gui()->addElement(this, {"ManipulateKnee", "Trajectory"}, mc_rtc::gui::ElementsStacking::Horizontal,
                        mc_rtc::gui::Checkbox(
                            "Continuous", [this]() { return continuous_; }, [this]() { continuous_ = !continuous_; }),
                        mc_rtc::gui::Checkbox(
                            "Repeat with offsets", [this]() { return repeatWithOffset_; },
                            [this]() { repeatWithOffset_ = !repeatWithOffset_; }),
                        mc_rtc::gui::Checkbox(
                            "Pause", [this]() { return !play_; },
                            [this]()
                            {
                              if(file_.tibiaRotationVector.empty()) return;
                              if(play_)
                              {
                                stop();
                              }
                              else
                              {
                                play();
                              }
                            }),
                        mc_rtc::gui::Button("Force Next", [this]() { forceNext(); }));

  ctl.gui()->addElement(this, {"ManipulateKnee", "Trajectory"}, mc_rtc::gui::ElementsStacking::Horizontal,
                        mc_rtc::gui::Checkbox(
                            "Measure", [this]() { return measure_; }, [this]() { measure_ = !measure_; }),
                        mc_rtc::gui::NumberInput(
                            "Samples", [this]() { return desiredSamples_; },
                            [this](double samples) { desiredSamples_ = std::max(1, static_cast<int>(samples)); }),
                        mc_rtc::gui::NumberInput(
                            "Rate [s]", [this, &ctl]() { return getRate(ctl.timeStep); },
                            [this, &ctl](double rate) { setRate(rate, ctl.timeStep); }));

  ctl.gui()->addElement(this, {"ManipulateKnee", "Trajectory", "Thresholds"},
                        mc_rtc::gui::NumberInput(
                            "Translation [mm]", [this]() { return translationTreshold_; },
                            [this](double treshold) { translationTreshold_ = treshold; }),
                        mc_rtc::gui::NumberInput(
                            "Rotation [deg]", [this]() { return rotationTreshold_; },
                            [this](double treshold) { rotationTreshold_ = treshold; }));

  ctl.gui()->addElement(this, {"ManipulateKnee", "Error"},
                        mc_rtc::gui::ArrayLabel("Tibia Error",
                                                {"Tangage [deg]", "Roulis [deg]", "Lacet [deg]", "Left (x) [mm]",
                                                 "Forward (y) [mm]", "Down (z) [mm]"},
                                                [this]()
                                                {
                                                  Eigen::Vector6d res;
                                                  res.head<3>() = tibiaError_.angular() * 180 / mc_rtc::constants::PI;
                                                  res.tail<3>() = tibiaError_.linear() * 1000;
                                                  return truncate(res);
                                                }),
                        mc_rtc::gui::ArrayLabel("Tibia Error (norm)", {"Rotation [deg]", "Translation [mm]"},
                                                [this]()
                                                {
                                                  Eigen::Vector2d res;
                                                  res[0] = tibiaError_.angular().norm() * 180 / mc_rtc::constants::PI;
                                                  res[1] = tibiaError_.linear().norm() * 1000;
                                                  return truncate(res);
                                                }),
                        mc_rtc::gui::ArrayLabel("Femur Error",
                                                {"Tangage [deg]", "Roulis [deg]", "Lacet [deg]", "Left (x) [mm]",
                                                 "Forward (y) [mm]", "Down (z) [mm]"},
                                                [this]()
                                                {
                                                  Eigen::Vector6d res;
                                                  res.head<3>() = femurError_.angular() * 180 / mc_rtc::constants::PI;
                                                  res.tail<3>() = femurError_.linear() * 1000;
                                                  return truncate(res);
                                                }),
                        mc_rtc::gui::ArrayLabel("Femur Error (norm)", {"Rotation [deg]", "Translation [mm]"},
                                                [this]()
                                                {
                                                  Eigen::Vector2d res;
                                                  res[0] = femurError_.angular().norm() * 180 / mc_rtc::constants::PI;
                                                  res[1] = femurError_.linear().norm() * 1000;
                                                  return truncate(res);
                                                }));

  tibia_task_ = mc_tasks::MetaTaskLoader::load<mc_tasks::TransformTask>(ctl.solver(), config_("TibiaTask"));
  tibia_task_->reset();
  femur_task_ = mc_tasks::MetaTaskLoader::load<mc_tasks::TransformTask>(ctl.solver(), config_("FemurTask"));
  femur_task_->reset();
  ctl.solver().addTask(tibia_task_);
  ctl.solver().addTask(femur_task_);

  X_0_tibiaAxisInitial = ctl.robot("panda_tibia").frame("Tibia").position();
  X_0_femurAxisInitial = X_0_tibiaAxisInitial;

  if(ctl.config().has("offsets"))
  {
    ctl.config()("offsets")("tibia", tibiaOffsetInitial_);
  }
  if(ctl.config().has("offsets"))
  {
    ctl.config()("offsets")("femur", femurOffsetInitial_);
  }
  updateTibiaOffset(tibiaOffsetInitial_);
  updateFemurOffset(femurOffsetInitial_);

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
  femurOffset_ = femurOffsetInterpolator_.compute(femurOffsetInterpolationTime_);
  tibiaOffset_ = tibiaOffsetInterpolator_.compute(tibiaOffsetInterpolationTime_);
  X_0_femurAxis = femurOffset_ * X_0_femurAxisInitial;
  X_0_tibiaAxis = tibiaOffset_ * X_0_tibiaAxisInitial;
  tibiaOffsetInterpolationTime_ += ctl.timeStep;
  femurOffsetInterpolationTime_ += ctl.timeStep;
}

bool ManipulateKnee::measure(mc_control::fsm::Controller & ctl)
{
  if(!measure_)
  {
    return true;
  }
  else if(!ctl.datastore().has("BoneTagSerialPlugin") || !ctl.datastore().call<bool>("BoneTagSerialPlugin::Connected"))
  {
    mc_rtc::log::error("[{}] Requested measement of BoneTag sensors but the sensor is unavailable", name());
    return false;
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
  updateAxes(ctl);
  if(file_.tibiaRotationVector.empty())
  {
    if(play_)
    {
      if(repeatWithOffset_ && !trajOffsets_.done())
      {
        trajOffsets_.next();
        mc_rtc::log::info("Repeating trajectory with offset {} / {}", trajOffsets_.current(), trajOffsets_.size());
        // Reload trajectory
        file_.load(trajectory_dir_ + "/" + trajectory_file_);
        // Replay trajectory with the new offset
        play();
      }
      else
      {
        stop();
      }
    }
  }

  if(play_)
  {
    if(next_)
    {
      tibiaRotation_ = file_.tibiaRotationVector.front();
      tibiaTranslation_ = file_.tibiaTranslationVector.front() + trajOffsets_.tibiaOffsetTranslation();
      femurRotation_ = file_.femurRotationVector.front();
      femurTranslation_ = file_.femurTranslationVector.front() + trajOffsets_.femurOffsetTranslation();
      file_.tibiaRotationVector.pop_front();
      file_.tibiaTranslationVector.pop_front();
      file_.femurRotationVector.pop_front();
      file_.femurTranslationVector.pop_front();
      iter_ = 0;
      measuredSamples_ = 0;
      next_ = false;
    }
    else
    {
      bool hasConverged = false;
      if(!continuous_)
      {
        const auto & tibia_error = tibiaError_;
        const auto & femur_error = femurError_;
        hasConverged = (tibia_error.angular().norm() <= mc_rtc::constants::toRad(rotationTreshold_)
                        && tibia_error.linear().norm() <= translationTreshold_ / 1000.
                        && femur_error.angular().norm() <= mc_rtc::constants::toRad(rotationTreshold_)
                        && femur_error.linear().norm() <= translationTreshold_ / 1000.);
      }
      else
      { // Ignore convergence criteria when running continous trajectories
        hasConverged = true;
      }
      next_ = hasConverged && iter_ >= iterRate_ && measure(ctl);
    }
  }

  auto handle_motion = [](mc_rbdyn::Robot & realRobot, sva::MotionVecd & error, mc_tasks::TransformTask & task,
                          const sva::PTransformd X_0_axisFrame, Eigen::Vector3d translation, Eigen::Vector3d rotation,
                          Eigen::Vector3d & translationActual, Eigen::Vector3d & rotationActual)
  {
    translation *= 0.001; // translation in [m]
    rotation *= mc_rtc::constants::PI / 180.; // rotation in [rad]

    auto X_0_target = sva::PTransformd(mc_rbdyn::rpyToMat(rotation), translation) * X_0_axisFrame;
    task.target(X_0_target);

    auto X_0_actual = realRobot.frame(task.frame().name()).position(); // task.frame().position();
    auto X_axisFrame_actual = X_0_actual * X_0_axisFrame.inv();

    translationActual = X_axisFrame_actual.translation() * 1000;
    // XXX should we store as RPY?
    rotationActual = mc_rbdyn::rpyFromMat(X_axisFrame_actual.rotation()) * 180 / mc_rtc::constants::PI;

    sva::PTransformd X_axis_target(mc_rbdyn::rpyToMat(rotation), translation);
    error = sva::transformError(X_axisFrame_actual, X_axis_target);
  };

  handle_motion(ctl.realRobot("panda_femur"), femurError_, *femur_task_, X_0_femurAxis, femurTranslation_,
                femurRotation_, femurTranslationActual_, femurRotationActual_);
  handle_motion(ctl.realRobot("panda_tibia"), tibiaError_, *tibia_task_, X_0_tibiaAxis, tibiaTranslation_,
                tibiaRotation_, tibiaTranslationActual_, tibiaRotationActual_);

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
