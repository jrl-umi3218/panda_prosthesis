#include "Initial.h"

#include <mc_control/fsm/Controller.h>

namespace
{

static sva::PTransformd LoadInitialPose(mc_control::fsm::Controller & ctl, const std::string & robot)
{
  std::string etc_dir = ctl.config()("ETC_DIR");
  mc_rtc::Configuration initial(etc_dir + "/initial_pose.yaml");
  return initial("initial_pose_" + robot, ctl.robot(robot).posW());
}

static void SaveInitialPose(mc_control::fsm::Controller & ctl, const std::string & robot)
{
  std::string etc_dir = ctl.config()("ETC_DIR");
  std::string initial_pose_yaml = etc_dir + "/initial_pose.yaml";
  mc_rtc::Configuration initial(initial_pose_yaml);
  initial.add("initial_pose_" + robot, ctl.robot(robot).posW());
  initial.save(initial_pose_yaml);
}

static void SetInitialPose(mc_control::fsm::Controller & ctl, const std::string & robot, const sva::PTransformd & pose)
{
  ctl.robot(robot).posW(pose);
  SaveInitialPose(ctl, robot);
}

} // namespace

void Initial::configure(const mc_rtc::Configuration & config) {}

void Initial::start(mc_control::fsm::Controller & ctl)
{
  initial_pose_panda_femur_ = LoadInitialPose(ctl, "panda_femur");
  SetInitialPose(ctl, "panda_femur", initial_pose_panda_femur_);
  ctl.gui()->addElement({}, mc_rtc::gui::Button("Done", [this]() { output("OK"); }),
                        mc_rtc::gui::Transform(
                            "Initial pose (femur)",
                            [this]() -> const sva::PTransformd & { return initial_pose_panda_femur_; },
                            [this](const sva::PTransformd & p) {
                              initial_pose_panda_femur_ = p;
                              pose_changed_ = true;
                            }));
  initial_pose_panda_tibia_ = LoadInitialPose(ctl, "panda_tibia");
  SetInitialPose(ctl, "panda_tibia", initial_pose_panda_tibia_);
  ctl.gui()->addElement({}, mc_rtc::gui::Transform(
                                "Initial pose (tibia)",
                                [this]() -> const sva::PTransformd & { return initial_pose_panda_tibia_; },
                                [this](const sva::PTransformd & p) {
                                  initial_pose_panda_tibia_ = p;
                                  pose_changed_ = true;
                                }));
}

bool Initial::run(mc_control::fsm::Controller & ctl)
{
  if(pose_changed_)
  {
    pose_changed_ = false;
    SetInitialPose(ctl, "panda_femur", initial_pose_panda_femur_);
    SetInitialPose(ctl, "panda_tibia", initial_pose_panda_tibia_);
  }
  return output().size() != 0;
}

void Initial::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeElement({}, "Done");
  ctl.gui()->removeElement({}, "Initial pose (femur)");
  ctl.gui()->removeElement({}, "Initial pose (tibia)");
}

EXPORT_SINGLE_STATE("Initial", Initial)
