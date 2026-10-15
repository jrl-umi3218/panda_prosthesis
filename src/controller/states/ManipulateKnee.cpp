#include "ManipulateKnee.h"

#include <mc_control/fsm/Controller.h>
#include <mc_filter/utils/clamp.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/io_utils.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_trajectory/LinearInterpolation.h>
#include <boost/filesystem.hpp>
#include <3rd-party/csv.h>
#include <filesystem>
#include <iomanip>
#include <utils.h>

namespace fs = std::filesystem;

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

void ReadCSV::generateFromConfiguration(const mc_rtc::Configuration & config)
{
  int waypoints = config("Waypoints");

  Eigen::Vector3d minTibiaRot = config("MinTibiaRotation");
  Eigen::Vector3d maxTibiaRot = config("MaxTibiaRotation");
  Eigen::Vector3d minFemurRot = config("MinFemurRotation");
  Eigen::Vector3d maxFemurRot = config("MaxFemurRotation");
  Eigen::Vector3d minTibiaTrans = config("MinTibiaTranslation");
  Eigen::Vector3d maxTibiaTrans = config("MaxTibiaTranslation");
  Eigen::Vector3d minFemurTrans = config("minFemurTranslation");
  Eigen::Vector3d maxFemurTrans = config("maxFemurTranslation");

  mc_trajectory::LinearInterpolation<Eigen::Vector3d> interp;

  femurTranslationVector.clear();
  tibiaTranslationVector.clear();
  femurRotationVector.clear();
  tibiaRotationVector.clear();

  // Split waypoints between the two segments
  int n1 = waypoints / 2; // zero to min
  int n2 = waypoints - n1; // min to max

  // Interpolate from zero to min
  for(int i = 0; i < n1; ++i)
  {
    double t = n1 == 1 ? 0.0 : static_cast<double>(i) / (n1 - 1);
    femurRotationVector.push_back(interp(Eigen::Vector3d::Zero(), minFemurRot, t));
    tibiaRotationVector.push_back(interp(Eigen::Vector3d::Zero(), minTibiaRot, t));
    femurTranslationVector.push_back(interp(Eigen::Vector3d::Zero(), minFemurTrans, t));
    tibiaTranslationVector.push_back(interp(Eigen::Vector3d::Zero(), minTibiaTrans, t));
  }

  // Interpolate from min to max
  for(int i = 0; i < n2; ++i)
  {
    double t = n2 == 1 ? 0.0 : static_cast<double>(i) / (n2 - 1);
    femurRotationVector.push_back(interp(minFemurRot, maxFemurRot, t));
    tibiaRotationVector.push_back(interp(minTibiaRot, maxTibiaRot, t));
    femurTranslationVector.push_back(interp(minFemurTrans, maxFemurTrans, t));
    tibiaTranslationVector.push_back(interp(minTibiaTrans, maxTibiaTrans, t));
  }
}

void write_csv_bonetag(std::vector<BoneTagResult> results, const std::string & path)
{
  if(results.empty()) return;
  std::ofstream csv;
  csv.open(path);
  if(!csv)
  {
    mc_rtc::log::error("Failed to write results to CSV file {}", path);
  }

  csv << "iteration,femur_tangage,femur_roulis,femur_lacet,"
         "tibia_tangage,tibia_roulis,tibia_lacet,"
         "femur_x,femur_y,femur_z,"
         "tibia_x,tibia_y,tibia_z";

  // write sensor csv header
  for(size_t i = 0; i < results.front().sensorData.size(); ++i)
  {
    csv << ",sensor_" << i;
  }
  csv << std::endl;

  for(const auto & result : results)
  {
    csv << result.controllerIter << ","
        << mc_rtc::io::to_string(
               std::vector<double>{
                   result.femurRotation.x(),
                   result.femurRotation.y(),
                   result.femurRotation.z(),
                   result.tibiaRotation.x(),
                   result.tibiaRotation.y(),
                   result.tibiaRotation.z(),
                   result.femurTranslation.x(),
                   result.femurTranslation.y(),
                   result.femurTranslation.z(),
                   result.tibiaTranslation.x(),
                   result.tibiaTranslation.y(),
                   result.tibiaTranslation.z(),
               },
               ",")
        << "," << mc_rtc::io::to_string(result.sensorData, ",") << std::endl;
  }

  csv.close();
  mc_rtc::log::success("Results written to {}", path);
}

void write_csv_prototmr(const std::vector<ProtoTMRResult> & results, const std::string & path)
{
  if(results.empty()) return;
  std::ofstream csv;
  csv.open(path);
  if(!csv)
  {
    mc_rtc::log::error("Failed to write results to CSV file {}", path);
  }

  csv << "iteration,start_measurement_time[s],end_measurement_time[s],femur_tangage,femur_roulis,femur_lacet,"
         "tibia_tangage,tibia_roulis,tibia_lacet,"
         "femur_x,femur_y,femur_z,"
         "tibia_x,tibia_y,tibia_z,sensor_id";

  size_t dataSize = results.front().sensorData.data.front().size(); // number of measurement per sensor

  for(size_t i = 0; i < dataSize; ++i)
  {
    csv << ",sensor_value_" << i;
  }
  csv << std::endl;

  // write sensor data. N lines per sensor
  for(const auto & result : results)
  {
    const auto & sensorData = result.sensorData; // full sensor frame
    int sensorId = 0;
    for(const auto & perSensorData : sensorData.data)
    {
      csv << result.controllerIter << "," << sensorData.start_time_ms.count() / 1000.0 << ","
          << sensorData.end_time_ms.count() / 1000.0 << ","
          << mc_rtc::io::to_string(
                 std::vector<double>{
                     result.femurRotation.x(),
                     result.femurRotation.y(),
                     result.femurRotation.z(),
                     result.tibiaRotation.x(),
                     result.tibiaRotation.y(),
                     result.tibiaRotation.z(),
                     result.femurTranslation.x(),
                     result.femurTranslation.y(),
                     result.femurTranslation.z(),
                     result.tibiaTranslation.x(),
                     result.tibiaTranslation.y(),
                     result.tibiaTranslation.z(),
                 },
                 ",")
          << "," << sensorId++ << "," << mc_rtc::io::to_string(perSensorData, ",") << std::endl;
    }
  }

  csv.close();
  mc_rtc::log::success("Results written to {}", path);
}

void ManipulateKnee::triggerSaveResults(bool force)
{
  std::lock_guard<std::mutex> lock(saveResultsMutex_);
  if(sensorType_ == "ProtoTMRPlugin")
  {
    auto n = resultsProtoTMR_.results().size();
    if(n != 0 && (force || n % resultSaveAfterN == 0))
    {
      resultsProtoTMRCopy_ = resultsProtoTMR_.results();
      mc_rtc::log::info("[{}] Triggering results save for ProtoTMRPlugin, {} results to save", name(), n);
      saveResultsCv_.notify_one();
    }
  }
  else
  {
    auto n = resultsBoneTag_.results().size();
    if(n != 0 && (force || n % resultSaveAfterN == 0))
    {
      resultsBoneTagCopy_ = resultsBoneTag_.results();
      mc_rtc::log::info("[{}] Triggering results save for BoneTagSerialPlugin, {} results to save", name(), n);
      saveResultsCv_.notify_one();
    }
  }
}

void ManipulateKnee::saveResultsThread()
{
  auto makeResultPath = [](const std::string & resultPath, size_t resultSize)
  {
    fs::path origPath(resultPath);
    fs::path newPath = origPath.parent_path() / (origPath.stem().string() + "_" + std::to_string(resultSize) + ".csv");
    return newPath.string();
  };
  std::unique_lock<std::mutex> lock(saveResultsMutex_);
  while(saveResultsThreadRunning_)
  {
    saveResultsCv_.wait(lock);
    mc_rtc::log::info("[{}] Got request to save data, saving...", name());

    if(sensorType_ == "ProtoTMRPlugin")
    {
      write_csv_prototmr(resultsProtoTMRCopy_, makeResultPath(resultPath_, resultsProtoTMRCopy_.size()));
    }
    else if(sensorType_ == "BoneTagSerialPlugin")
    {
      write_csv_bonetag(resultsBoneTagCopy_, makeResultPath(resultPath_, resultsBoneTagCopy_.size()));
    }
    else
    {
      mc_rtc::log::warning("[{}] Unknown sensor type '{}', cannot save results", name(), sensorType_);
      return;
    }
    mc_rtc::log::info("[{}] Saving finished", name());
  }
}

void ManipulateKnee::start(mc_control::fsm::Controller & ctl)
{
  saveResultsThreadRunning_ = true;
  saveResultsThread_ = std::thread(&ManipulateKnee::saveResultsThread, this);

  config_("allow_missing_sensor", allowMissingSensor_);

  // Configure default sensor name
  // It will be the first available plugin whose sensor is connected matching the list of potentital candidates
  auto pluginNames = ctl.config()("Plugins", std::vector<std::string>{});
  sensorType_ = std::string{"None"};
  for(const auto & sensorType : std::vector<std::string>{"BoneTagSerialPlugin", "ProtoTMRPlugin"})
  {
    auto it = std::find(pluginNames.begin(), pluginNames.end(), sensorType);
    if(it == pluginNames.end())
    {
      mc_rtc::log::info("[{}] Sensor type {} not found in Plugins list, it won't be available for measurement", name(),
                        sensorType);
    }
    else
    {
      mc_rtc::log::success("[{}] Sensor type {} found in Plugins list, checking if connected...", name(), sensorType);
      if(ctl.datastore().has(sensorType + "::Connected") && ctl.datastore().call<bool>(sensorType + "::Connected"))
      {
        mc_rtc::log::success("[{}] Sensor {} is connected (or non required)", name(), sensorType_);
        sensorType_ = sensorType;
        break;
      }
      else
      {
        mc_rtc::log::warning("[{}] Sensor {} is not connected, it won't be available for measurement", name(),
                             sensorType);
      }
    }
  }

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

  iterRate_ = iterRateFromSeconds(config_("rate", 0.2), ctl.timeStep);
  config_("samples", desiredSamples_);

  if(auto convergenceC = config_.find("convergence"))
  {
    auto & c = convergence_;
    (*convergenceC)("tibiaAngularError", c.tibiaAngularError);
    (*convergenceC)("tibiaLinearError", c.tibiaLinearError);
    (*convergenceC)("femurAngularError", c.femurAngularError);
    (*convergenceC)("femurLinearError", c.femurLinearError);
    (*convergenceC)("femurVelocityError", c.femurVelocityError);
    (*convergenceC)("tibiaVelocityError", c.tibiaVelocityError);

    if(auto thresholds = convergenceC->find("thresholds"))
    {
      auto t = *thresholds;
      t("translation", c.translationTreshold_);
      t("rotation", c.rotationTreshold_);
      t("velocity", c.velocityThreshold_);
    }
  }

  ctl.config()("trajectory_dir", trajectory_dir_);
  results_dir_ = get_or_create_dir("results");

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
                        mc_rtc::gui::Button("Stop", [this]() { stop(); }),
                        mc_rtc::gui::Button("Play",
                                            [this]()
                                            {
                                              stop();
                                              if(trajectory_file_ != "custom.csv")
                                              {
                                                file_.load(trajectory_dir_ + "/" + trajectory_file_);
                                              }
                                              next_ = true;
                                              iter_ = 0;
                                              hasConverged_ = false;
                                              gotMeasurement_ = false;
                                              newFrameRequested_ = false;
                                              play();
                                            }));
  ctl.gui()->addElement(this, {"ManipulateKnee", "Trajectory", "Generate"},
                        mc_rtc::gui::Form(
                            "Generate trajectory",
                            [this](const mc_rtc::Configuration & data)
                            {
                              mc_rtc::log::info("Generating trajectory\n{}", data.dump(true, true));
                              trajectory_file_ = "custom.csv";
                              file_.generateFromConfiguration(data);
                            },
                            mc_rtc::gui::FormIntegerInput("Waypoints", false, 50),
                            mc_rtc::gui::FormArrayInput("MinTibiaRotation", false, minTibiaRotation_),
                            mc_rtc::gui::FormArrayInput("MaxTibiaRotation", false, maxTibiaRotation_),
                            mc_rtc::gui::FormArrayInput("MinFemurRotation", false, minFemurRotation_),
                            mc_rtc::gui::FormArrayInput("MaxFemurRotation", false, maxFemurRotation_),
                            mc_rtc::gui::FormArrayInput("MinTibiaTranslation", false, minTibiaTranslation_),
                            mc_rtc::gui::FormArrayInput("MaxTibiaTranslation", false, maxTibiaTranslation_),
                            mc_rtc::gui::FormArrayInput("minFemurTranslation", false, minFemurTranslation_),
                            mc_rtc::gui::FormArrayInput("maxFemurTranslation", false, maxFemurTranslation_)));

  ctl.gui()->addElement(this, {"ManipulateKnee", "Manual Logging"}, mc_rtc::gui::ElementsStacking::Horizontal,
                        mc_rtc::gui::Checkbox(
                            "Manual Logging", [this]() { return manualLogging_; }, [this]() {}),
                        mc_rtc::gui::Button("Start Logging",
                                            [this]()
                                            {
                                              mc_rtc::log::info("Start manual logging");
                                              manualLogging_ = true;
                                            }),
                        mc_rtc::gui::Button("Stop and Save",
                                            [this]()
                                            {
                                              mc_rtc::log::info("Saving manual logging results");
                                              manualLogging_ = false;
                                              triggerSaveResults(true);
                                            }),
                        mc_rtc::gui::Button("Clear Results",
                                            [this]()
                                            {
                                              resultsProtoTMR_.clear();
                                              resultsBoneTag_.clear();
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
                            "Rate [s]", [this, &ctl]() { return iterRateToSeconds(iterRate_, ctl.timeStep); },
                            [this, &ctl](double rate) { iterRate_ = iterRateFromSeconds(rate, ctl.timeStep); }));

  ctl.gui()->addElement(this, {"ManipulateKnee", "Trajectory", "Thresholds"},
                        mc_rtc::gui::NumberInput(
                            "Translation [mm]", [this]() { return convergence_.translationTreshold_; },
                            [this](double treshold) { convergence_.translationTreshold_ = treshold; }),
                        mc_rtc::gui::NumberInput(
                            "Rotation [deg]", [this]() { return convergence_.rotationTreshold_; },
                            [this](double treshold) { convergence_.rotationTreshold_ = treshold; }));

  auto addErrorLabels = [this, &ctl](const auto & poseError, const auto & velocityError, const std::string & prefix,
                                     const std::vector<std::string> & path)
  {
    ctl.gui()->addElement(
        this, path,
        mc_rtc::gui::ArrayLabel(
            prefix + " Error",
            {"Tangage [deg]", "Roulis [deg]", "Lacet [deg]", "Left (x) [mm]", "Forward (y) [mm]", "Down (z) [mm]"},
            [this, &poseError]()
            {
              Eigen::Vector6d res;
              res.head<3>() = poseError.angular() * 180 / mc_rtc::constants::PI;
              res.tail<3>() = poseError.linear() * 1000;
              return truncate(res);
            }),
        mc_rtc::gui::ArrayLabel(prefix + " Error (norm)", {"Rotation [deg]", "Translation [mm]", "Speed"},
                                [this, &poseError, &velocityError]()
                                {
                                  Eigen::Vector3d res;
                                  res[0] = poseError.angular().norm() * 180 / mc_rtc::constants::PI;
                                  res[1] = poseError.linear().norm() * 1000;
                                  res[2] = velocityError.linear().norm() * 1000; // Speed as norm of linear velocity
                                  return truncate(res);
                                }),
        mc_rtc::gui::ArrayLabel(prefix + " Velocity Error",
                                {"Angular X [deg/s]", "Angular Y [deg/s]", "Angular Z [deg/s]", "Linear X [mm/s]",
                                 "Linear Y [mm/s]", "Linear Z [mm/s]"},
                                [this, &velocityError]()
                                {
                                  Eigen::Vector6d res;
                                  res.head<3>() = velocityError.angular() * 180 / mc_rtc::constants::PI;
                                  res.tail<3>() = velocityError.linear() * 1000;
                                  return truncate(res);
                                }),
        mc_rtc::gui::ArrayLabel(prefix + " Velocity Error (angular/linear norm)",
                                {"Angular Speed [deg/s]", "Linear Speed [mm/s]"},
                                [this, &velocityError]()
                                {
                                  Eigen::Vector2d res;
                                  res[0] = velocityError.angular().norm() * 180 / mc_rtc::constants::PI;
                                  res[1] = velocityError.linear().norm() * 1000;
                                  return truncate(res);
                                }),
        mc_rtc::gui::Label(prefix + " Velocity Error (norm)",
                           [this, &velocityError]() { return velocityError.vector().norm(); }));
  };

  addErrorLabels(convergence_.femurError_, convergence_.femurVelocityError_, "Femur", {"ManipulateKnee", "Error"});
  addErrorLabels(convergence_.tibiaError_, convergence_.tibiaVelocityError_, "Tibia", {"ManipulateKnee", "Error"});

  tibia_task_ = mc_tasks::MetaTaskLoader::load<mc_tasks::TransformTask>(ctl.solver(), config_("TibiaTask"));
  tibia_task_->reset();
  femur_task_ = mc_tasks::MetaTaskLoader::load<mc_tasks::TransformTask>(ctl.solver(), config_("FemurTask"));
  femur_task_->reset();
  ctl.solver().addTask(tibia_task_);
  ctl.solver().addTask(femur_task_);

  X_0_tibiaAxisInitial = ctl.robot("panda_tibia").frame("Tibia").position();
  X_0_femurAxisInitial = ctl.robot("panda_femur").frame("Femur").position();
  // X_0_femurAxisInitial = X_0_tibiaAxisInitial;

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

  output("OK");
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
  else if(!ctl.datastore().has(sensorType_ + "::Connected") || !ctl.datastore().call<bool>(sensorType_ + "::Connected"))
  {
    mc_rtc::log::error("[{}] Requested measement of {} sensors but the sensor is unavailable", name(), sensorType_);
    return false;
  }

  if(!newFrameRequested_)
  {
    ctl.datastore().call(sensorType_ + "::RequestNewFrame");
    mc_rtc::log::info("[{}] Requested new sensor data frame at iter: {}, time: {}s", name(), controllerIter_,
                      iterRateToSeconds(controllerIter_, ctl.timeStep));
    newFrameRequested_ = true;
  }

  // Check until we got a new frame
  if(!ctl.datastore().call<bool>(sensorType_ + "::GotNewFrame"))
  {
    // mc_rtc::log::info("[{}] Waiting for new sensor data frame...", name());
    return false;
  }

  if(sensorType_ == "BoneTagSerialPlugin")
  {
    measure_bonetag(ctl);
  }
  else if(sensorType_ == "ProtoTMRPlugin")
  {
    measure_prototmr(ctl);
  }

  ++measuredSamples_;

  newFrameRequested_ = false;

  if(measuredSamples_ == desiredSamples_)
  {
    triggerSaveResults();
    mc_rtc::log::success("Measurement done! Got {} samples as requested", measuredSamples_);
    return true;
  }
  return false;
}

void ManipulateKnee::measure_bonetag(mc_control::fsm::Controller & ctl)
{
  // We got a new frame

  auto sensorData = ctl.datastore().call<io::BoneTagSerial::Data>(sensorType_ + "::GetLastFrame");
  mc_rtc::log::success("[{}] Got new sensor data frame: {}", name(), mc_rtc::io::to_string(sensorData));

  BoneTagResult result;
  result.controllerIter = controllerIter_;
  result.femurRotation = femurRotationActual_;
  result.femurTranslation = femurTranslationActual_;
  result.tibiaRotation = tibiaRotationActual_;
  result.tibiaTranslation = tibiaTranslationActual_;
  result.sensorData = std::move(sensorData);
  resultsBoneTag_.addResult(result);
}

void ManipulateKnee::measure_prototmr(mc_control::fsm::Controller & ctl)
{
  // We got a new frame
  auto sensorData = ctl.datastore().call<io::Serial::TimedRawData>(sensorType_ + "::GetLastFrame");
  mc_rtc::log::success("[{}] Got new sensor data frame at iter: {}, time: {}s", name(), controllerIter_,
                       iterRateToSeconds(controllerIter_, ctl.timeStep));
  // for(unsigned i = 0; i < sensorData.data.size(); ++i)
  // {
  //   mc_rtc::log::info("Sensor[{}]: {}", i, mc_rtc::io::to_string(sensorData.data[i]));
  // }

  ProtoTMRResult result;
  result.controllerIter = controllerIter_;
  result.femurRotation = femurRotationActual_;
  result.femurTranslation = femurTranslationActual_;
  result.tibiaRotation = tibiaRotationActual_;
  result.tibiaTranslation = tibiaTranslationActual_;
  result.sensorData = std::move(sensorData);
  resultsProtoTMR_.addResult(result);
  mc_rtc::log::info("Got new data between t={}[s] and t={}[s]", result.sensorData.start_time_ms.count() / 1000,
                    result.sensorData.end_time_ms.count() / 1000);
}

bool ManipulateKnee::run(mc_control::fsm::Controller & ctl)
{
  // Increase gains slowly
  if(controllerIter_++ <= 1000)
  {
    auto interpGains = [this](std::string && taskName, mc_tasks::TransformTask & task)
    {
      const auto & taskC = config_(taskName);
      auto tinterp = controllerIter_ / 1000.;
      auto s = stiffnessInterp_(taskC("initial_stiffness"), taskC("stiffness"), tinterp);
      task.stiffness(s);

      if(taskC.has("initial_damping") && taskC.has("damping"))
      {
        auto d = stiffnessInterp_(taskC("initial_damping"), taskC("damping"), tinterp);
        task.damping(d);
      }

      auto w = dimWeightInterp_(taskC("initial_dimWeight"), taskC("dimWeight"), tinterp);
      task.dimWeight(w);
    };
    interpGains("FemurTask", *femur_task_);
    interpGains("TibiaTask", *tibia_task_);
    return true;
  }
  else if(controllerIter_++ == 1001)
  {
    updateTibiaOffset(tibiaOffsetInitial_);
    updateFemurOffset(femurOffsetInitial_);
    updateAxes(ctl);
    return true;
  }

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
      // Wait until at least half of iterRate_
      // This is done in part to ensure that convergence criteria based on speed are not triggered
      // when the robot is still starting the motion
      if(iter_ >= iterRate_ / 2.)
      {
        if(!continuous_ && !hasConverged_)
        {
          const auto & c = convergence_;
          auto optionalErrorCheck = [](bool errorFlag, double errorValue, double threshold)
          { return !errorFlag || errorValue <= threshold; };

          hasConverged_ =
              optionalErrorCheck(c.tibiaAngularError, c.tibiaError_.angular().norm(),
                                 mc_rtc::constants::toRad(c.rotationTreshold_))
              && optionalErrorCheck(c.tibiaLinearError, c.tibiaError_.linear().norm(), c.translationTreshold_ / 1000.)
              && optionalErrorCheck(c.femurAngularError, c.femurError_.angular().norm(),
                                    mc_rtc::constants::toRad(c.rotationTreshold_))
              && optionalErrorCheck(c.femurLinearError, c.femurError_.linear().norm(), c.translationTreshold_ / 1000.)
              && optionalErrorCheck(c.femurVelocityError, c.femurVelocityError_.linear().norm(), c.velocityThreshold_)
              && optionalErrorCheck(c.tibiaVelocityError, c.tibiaVelocityError_.linear().norm(), c.velocityThreshold_);
          if(hasConverged_)
          {
            std::ostringstream msg;
            msg << "Convergence criteria met at iteration " << iter_ << ": ";

            if(c.tibiaAngularError)
              msg << "[Tibia angular: " << std::fixed << std::setprecision(2)
                  << c.tibiaError_.angular().norm() * 180 / mc_rtc::constants::PI << " deg] ";
            if(c.tibiaLinearError)
              msg << "[Tibia linear: " << std::fixed << std::setprecision(2) << c.tibiaError_.linear().norm() * 1000
                  << " mm] ";
            if(c.femurAngularError)
              msg << "[Femur angular: " << std::fixed << std::setprecision(2)
                  << c.femurError_.angular().norm() * 180 / mc_rtc::constants::PI << " deg] ";
            if(c.femurLinearError)
              msg << "[Femur linear: " << std::fixed << std::setprecision(2) << c.femurError_.linear().norm() * 1000
                  << " mm] ";
            if(c.femurVelocityError)
              msg << "[Femur velocity: " << std::fixed << std::setprecision(2) << c.femurVelocityError_.linear().norm()
                  << " m/s] ";
            if(c.tibiaVelocityError)
              msg << "[Tibia velocity: " << std::fixed << std::setprecision(2) << c.tibiaVelocityError_.linear().norm()
                  << " m/s] ";

            mc_rtc::log::success(msg.str());
          }
        }
        else
        { // Ignore convergence criteria when running continous trajectories
          hasConverged_ = true;
        }

        if(hasConverged_ || iter_ >= iterRate_)
        { // We have converged to a waypoint
          if(sensorType_ != "BoneTagSerialPlugin" && measure_ && !gotMeasurement_)
          { // Start measurement
            // mc_rtc::log::success("Stopping motion and requesting measurement at iteration {}...", iter_);
            femur_task_->reset();
            tibia_task_->reset();
            // But have not yet measured a sensor frame, request a new full frame
            // gotMeasurement_ will be set to true once we got the new frame, then we will wait until iterRate_ has been
            // reached before going to the next waypoint
            gotMeasurement_ = measure(ctl);
            if(gotMeasurement_)
            {
              mc_rtc::log::success("Got measurement for waypoint at iteration {}, waiting until iter({})>=iterRate({})",
                                   iter_, iter_, iterRate_);
            }
          }

          if(
              /*Proceed if not measuring, or if measuring and a measurement has been received.*/
              (sensorType_ == "BoneTagSerialPlugin" || !measure_ || gotMeasurement_)
              /*Proceed if not in continuous mode, or if in continuous mode and enough iterations have passed.*/
              && (!continuous_ || continuous_ && iter_ >= iterRate_))
          {
            auto remaining = file_.tibiaRotationVector.size();
            mc_rtc::log::info("Waypoint handled successfully, remaining: {}", remaining);
            gotMeasurement_ = false;
            hasConverged_ = false;
            next_ = true;
          }
        }
      }
    }
  }

  auto handle_motion = [](mc_rbdyn::Robot & realRobot, sva::MotionVecd & posError, sva::MotionVecd & velError,
                          mc_tasks::TransformTask & task, const sva::PTransformd X_0_axisFrame,
                          Eigen::Vector3d translation, Eigen::Vector3d rotation, Eigen::Vector3d & translationActual,
                          Eigen::Vector3d & rotationActual)
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
    posError = sva::transformError(X_axisFrame_actual, X_axis_target);
    velError = sva::MotionVecd(task.speed());
  };

  handle_motion(ctl.robot("panda_femur"), convergence_.femurError_, convergence_.femurVelocityError_, *femur_task_,
                X_0_femurAxis, femurTranslation_, femurRotation_, femurTranslationActual_, femurRotationActual_);
  handle_motion(ctl.robot("panda_tibia"), convergence_.tibiaError_, convergence_.tibiaVelocityError_, *tibia_task_,
                X_0_tibiaAxis, tibiaTranslation_, tibiaRotation_, tibiaTranslationActual_, tibiaRotationActual_);

  // Record new BoneTag data throughout motion and convergence waits.
  // GetNewData consumes the latest available frame, at most once per control cycle.
  if(sensorType_ == "BoneTagSerialPlugin")
  {
    if(measure_ && (play_ || manualLogging_))
    {
      auto data = ctl.datastore().call<std::optional<io::BoneTagSerial::Data>>("BoneTagSerialPlugin::GetNewData");
      if(data)
      {
        BoneTagResult result;
        result.controllerIter = controllerIter_;
        result.femurRotation = femurRotationActual_;
        result.femurTranslation = femurTranslationActual_;
        result.tibiaRotation = tibiaRotationActual_;
        result.tibiaTranslation = tibiaTranslationActual_;
        result.sensorData = *data;
        resultsBoneTag_.addResult(result);
        triggerSaveResults();
      }
    }
  }
  else if(manualLogging_)
  {
    measure(ctl);
  }

  ++controllerIter_;
  ++iter_;
  return true;
}

void ManipulateKnee::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.solver().removeTask(tibia_task_);
  ctl.solver().removeTask(femur_task_);
  ctl.gui()->removeElements(this);
  saveResultsThreadRunning_ = false;
  saveResultsThread_.join();
}

EXPORT_SINGLE_STATE("ManipulateKnee", ManipulateKnee)
