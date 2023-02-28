#include "panda_brace_module.h"
#include <RBDyn/parsers/urdf.h>
#include <boost/filesystem.hpp>
#include "config.h"
#include "config_panda_brace.h"
namespace bfs = boost::filesystem;

namespace mc_robots
{
PandaBraceRobotModule::PandaBraceRobotModule() : mc_robots::PandaRobotModule(false, false, false)
{
  // Merge with brace_top_setup urdf here
  auto merge_urdf = [this](const std::string & urdf_file, const std::string & merge_root,
                           const std::string & merge_with_link, const sva::PTransformd & X_root_link) {
    auto brace_urdf = rbd::parsers::from_urdf_file(urdf_file);
    const auto & brace_mb = brace_urdf.mb;
    const auto & bodies = brace_mb.bodies();
    const auto & pred = brace_mb.predecessors();
    const auto & succ = brace_mb.successors();
    const auto & trans = brace_mb.transforms();
    bool add = false;
    mc_rtc::log::info("Print joints predecessors and successors");
    for(int ji = 0; ji < brace_mb.joints().size(); ++ji)
    {
      const auto & j = brace_mb.joint(ji);
      auto predBi = pred[ji];
      auto succBi = succ[ji];
      auto predBName = std::string{};
      if(predBi != -1)
      {
        predBName = bodies[predBi].name();
      }
      else
      {
        predBName = "Root";
      }
      auto succBName = bodies[succBi].name();
      mc_rtc::log::info("pred {}, joint {}, succ {}", predBName, j.name(), succBName);
      mc_rtc::log::info("the static translation between the body {} and the joint {} is {}", j.name(), predBName,
                        trans[ji].translation().transpose());

      if(predBName == merge_with_link)
      {
        add = true;
      }

      if(add)
      {
        static bool first_add = true;
        auto addBody =
            [this, &bodies, &brace_urdf](const std::string & name, int bodyIdx)
            {
              mc_rtc::log::info("[PandaBrace] Add body {}", name);
              mbg.addBody(bodies[bodyIdx]);
              auto convex = bfs::path(panda_prosthesis::brace_top_setup_DIR) / "convex" / (name + "-ch.txt");
              if(!bfs::exists(convex))
              {
                mc_rtc::log::error_and_throw<std::runtime_error>("Invalid brace_top_setup, no convex found {}", convex.string());
              }
              _visual[name] = brace_urdf.visual[name];
              _collision[name] = brace_urdf.collision[name];
              _convexHull[name] = {name, convex.string()};
            };

        if(first_add)
        {
          addBody(predBName, predBi);
          first_add = false;
        }

        mc_rtc::log::info("[PandaBrace] Add joint {}", j.name());
        mbg.addJoint(j);

        mc_rtc::log::info("[PandaBrace] Add body {}", succBName);
        addBody(succBName, succBi);

        mc_rtc::log::info("[PandaBrace] Link body {} to joint {} with transform {}", predBName, j.name(),
                          trans[ji].translation().transpose());
        mbg.linkBodies(predBName, trans[ji], succBName, sva::PTransformd::Identity(), j.name());
      }
    }

    auto merge_fixed_joint = merge_root + "_" + merge_with_link + "_joint";
    mc_rtc::log::info("Add fixed joint {} between {} and {}", merge_fixed_joint, merge_root, merge_with_link);
    mbg.addJoint({rbd::Joint::Type::Fixed, true, merge_fixed_joint});
    mbg.linkBodies(merge_root, X_root_link, merge_with_link, sva::PTransformd::Identity(), merge_fixed_joint);

    mb = mbg.makeMultiBody(mb.body(0).name(), true);
    mbc = rbd::MultiBodyConfig(mb);
    mbc.zero(mb);
  };

  auto prosthesis = std::string{"panda_brace_femur"};
  auto transforms = bfs::path(panda_prosthesis::transforms_DIR);
  auto transform = transforms / (prosthesis + ".yml");
  if(!bfs::exists(transform))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Invalid prosthesis {}, no transform found {}", prosthesis,
                                                     transform.string());
  }
  auto transformC = mc_rtc::Configuration(transform.string());

  merge_urdf(panda_prosthesis::brace_top_setup_DIR + std::string{"/urdf/brace_top_setup.urdf"}, "panda_link8",
             "base_link", transformC("panda_link8_to_base_link"));

  _forceSensors.emplace_back("BraceTopForceSensor", "Link1", sva::PTransformd::Identity());

  // Save new URDF
  auto urdf_path = bfs::temp_directory_path() / ("panda_brace_femur.urdf");
  {
    rbd::parsers::Limits limits;
    limits.lower = _bounds[0];
    limits.upper = _bounds[1];
    limits.velocity = _bounds[3];
    limits.torque = _bounds[5];
    std::ofstream ofs(urdf_path.string());
    ofs << rbd::parsers::to_urdf({mb, mbc, mbg, limits, _visual, _collision, "panda_brace_femur"});
  }
  this->urdf_path = urdf_path.string();
  this->name = "panda_brace_femur";
  mc_rtc::log::info("Wrote URDF to {}", urdf_path.string());
}

} // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"PandaBrace::Femur"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("PandaBrace")
    if(n == "PandaBrace::Femur")
    {
      return new mc_robots::PandaBraceRobotModule();
    }
    else
    {
      mc_rtc::log::error("Panda module cannot create an object of type {}", n);
      return nullptr;
    }
  }
}
