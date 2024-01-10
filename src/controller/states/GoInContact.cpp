#include "GoInContact.h"
#include <mc_control/fsm/Controller.h>
#include <mc_tasks/TransformTask.h>
#include <cmath> // for copysign

void GoInContact::start(mc_control::fsm::Controller & ctl)

{
  config_("useForce", useForce_);
  config_("heightAboveLink6", heightAboveLink6_);
  config_("velocity", vel_);

  ctl.gui()->addElement(
      this, {"GoInContact"}, mc_rtc::gui::Input("Use Force", useForce_),
      mc_rtc::gui::Input("heightAboveLink6", heightAboveLink6_), mc_rtc::gui::Input("velocity [m/s]", vel_),
      mc_rtc::gui::Button(
          "Go to Contact",
          [this, &ctl]()
          {
            clicked = true;
            auto robotName = static_cast<std::string>(config_("robot"));
            auto frameName = static_cast<std::string>(config_("frame"));
            auto X_0_femur = ctl.robot().frame(frameName).position();
            
            auto X_0_link6 = ctl.robot("brace_bottom_setup").frame("Link6").position();
            auto X_0_final = sva::PTransformd(Eigen::Vector3d{0, 0, heightAboveLink6_}) * X_0_link6;
            auto velSgn = std::copysign(1.0, X_0_final.translation().z() - X_0_femur.translation().z());
            
            auto refVelLink6 = sva::MotionVecd({0, 0, 0}, {0, 0, velSgn * vel_});
            auto refVelW = X_0_link6.inv() * refVelLink6;
            mc_rtc::log::info("refVelW: {}", refVelW.vector().transpose());

            // for real
            // auto refVelW = sva::MotionVecd({0, 0, 0}, {0, 0, -0.01});

            velB_ = X_0_femur * refVelW;
            stiff_ = {{100, 100, 100}, {1, 1, 1}};
            damp_ = {{20, 20, 20}, {300, 300, 300}};
            auto targetRobotName = static_cast<std::string>(config_("target_robot"));
            auto targetFrameName = static_cast<std::string>(config_("target_frame"));
            auto force_sensor_output = ctl.robot(targetRobotName).frame(targetFrameName).wrench();
            target_force_z = force_sensor_output.force().z();

            if(!ctl.hasRobot(targetRobotName))
            {
              mc_rtc::log::error_and_throw("[{}] No robot named {}", name(), targetRobotName);
            }
            if(!ctl.robot(targetRobotName).hasFrame(targetFrameName))
            {
              mc_rtc::log::error_and_throw("[{}] No frame named {} in robot {}", name(), targetFrameName,
                                           targetRobotName);
            }

            if(!ctl.hasRobot(robotName))
            {
              mc_rtc::log::error_and_throw("[{}] No robot named {}", name(), robotName);
            }
            if(!ctl.robot(robotName).hasFrame(frameName))
            {
              mc_rtc::log::error_and_throw("[{}] No frame named {} in robot {}", name(), frameName, robotName);
            }
            transfoTask_ = std::make_shared<mc_tasks::TransformTask>(ctl.robot(robotName).frame(frameName), 2.0, 500.0);

            transfoTask_->refVelB(
                velB_); // on veut une vitesse de 0.01 m/s dans le rep�re fixe, refVelb prend les coordonn�es dans le
                        // rep�re de la frame control�e (f�mur) donc on fait un changement de rep�re. Ici, on n�gligera
            // l'angle de varus valgus suppos� faible et on consid�rera donc les 2 axes x confondus. L'angle entre les 2
            // frames est �gal � 18.5 degr�s et est constant au cours du mouvement qui est seulement une translation
            // selon Z_rep�re_fixe

            transfoTask_->setGains(stiff_, damp_);
            ctl.solver().addTask(transfoTask_);
            output("OK");
          }));
}

bool GoInContact::run(mc_control::fsm::Controller & ctl)
{
  auto targetRobotName = static_cast<std::string>(config_("target_robot"));
  auto targetFrameName = static_cast<std::string>(config_("target_frame"));
  auto force_sensor_output = ctl.robot(targetRobotName).frame(targetFrameName).wrench();

  if(clicked)
  {

    target_force_z = force_sensor_output.force().z();
    if(useForce_ && target_force_z <= -2)
    {
      return true;
    }
    if(!useForce_)
    {
      auto X_0_link6 = ctl.robot("brace_bottom_setup").frame("Link6").position();
      auto X_0_final = sva::PTransformd(Eigen::Vector3d{0, 0, heightAboveLink6_}) * X_0_link6;
      auto frameName = static_cast<std::string>(config_("frame"));
      auto X_0_Link2 = ctl.robot().frame(frameName).position();
      auto X_Link6_Link2 = X_0_Link2 * X_0_link6.inv();

      if(fabs(X_Link6_Link2.translation().z() - heightAboveLink6_) <= 1e-4)
      {
        transfoTask_->refVelB(sva::MotionVecd::Zero());
        return true;
      }
      else
      {
        return false;
      }
    }
  }
  return false;
}

void GoInContact::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.getPostureTask(config_("robot"))->reset();
  // XXX set high stiffness so that the robot stays here while selecting the trajectory
  // consider a better way
  ctl.getPostureTask(config_("robot"))->stiffness(100);
  ctl.solver().removeTask(transfoTask_);
  ctl.gui()->removeElements(this);
}

EXPORT_SINGLE_STATE("GoInContact", GoInContact)
