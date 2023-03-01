#include "panda_brace_module.h"
#include <RBDyn/CoM.h>
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
  auto merge_urdf = [this](const rbd::parsers::ParserResult & brace_urdf, const std::string & merge_root,
                           const std::string & merge_with_link, const sva::PTransformd & X_root_link,
                           const mc_rtc::Configuration & extraConf) {
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
        auto addBody = [this, &bodies, &brace_urdf](const std::string & name, int bodyIdx) {
          mc_rtc::log::info("[PandaBrace] Add body {}", name);
          mbg.addBody(bodies[bodyIdx]);
          auto convex = bfs::path(panda_prosthesis::brace_top_setup_DIR) / "convex" / (name + "-ch.txt");
          if(!bfs::exists(convex))
          {
            mc_rtc::log::error_and_throw<std::runtime_error>("Invalid brace_top_setup, no convex found {}",
                                                             convex.string());
          }
          _visual[name] = brace_urdf.visual.at(name);
          _collision[name] = brace_urdf.collision.at(name);
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

  auto fix_inertia = [this](const rbd::MultiBody & brace_mb, const mc_rtc::Configuration & extraConf) {
    mc_rtc::log::info("Fix inertia");
    const auto & bodies = brace_mb.bodies();
    const auto & pred = brace_mb.predecessors();
    const auto & succ = brace_mb.successors();
    const auto & trans = brace_mb.transforms();

    double totalMass = 0;
    std::vector<rbd::Body> newBodies;
    for(const auto & body : brace_mb.bodies())
    {
      auto newBody = body;
      const auto & name = body.name();
      // Scale mass and inertia
      if(extraConf.has(name))
      {
        if(extraConf(name).has("mass"))
        {
          double newMass = extraConf(name)("mass");
          double oldMass = body.inertia().mass();
          double ratio = newMass / oldMass;
          // Scale mass and inertia
          newBody =
              rbd::Body{newMass, body.inertia().momentum() / oldMass, ratio * body.inertia().inertia(), body.name()};
          mc_rtc::log::info("[PandaBrace] Modifying inertia for body {}:\n- Old mass: {}, New mass: {}\n- Old com: "
                            "{}\n- New com: {}\n- Old inertia:\n{}\n- New inertia:\n{}",
                            name, oldMass, newMass, body.inertia().momentum().transpose() / oldMass,
                            newBody.inertia().momentum().transpose() / newMass, body.inertia().inertia(),
                            newBody.inertia().inertia());
        }
      }
      newBodies.push_back(newBody);
      totalMass += newBody.inertia().mass();
    }
    return std::make_tuple(rbd::MultiBody(newBodies, brace_mb.joints(), brace_mb.predecessors(), brace_mb.successors(),
                                          brace_mb.parents(), brace_mb.transforms()),
                           totalMass);
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

  auto urdf_file = panda_prosthesis::brace_top_setup_DIR + std::string{"/urdf/brace_top_setup.urdf"};
  auto brace_urdf = rbd::parsers::from_urdf_file(urdf_file);
  auto [newMb, totalMass] = fix_inertia(brace_urdf.mb, transformC);
  brace_urdf.mb = newMb;

  auto femurFrame = FrameDescription("Femur", "Link2", sva::PTransformd(Eigen::Vector3d{0, 0, -0.05}));
  _frames.push_back(femurFrame);

  merge_urdf(brace_urdf, "panda_link8", "base_link", transformC("panda_link8_to_base_link"), transformC);

  auto generate_panda_mechanical_data = [&brace_urdf, totalMass](const Eigen::Matrix3d & inertia) {
    // For Panda end effector configuration
    auto & mb = brace_urdf.mb;
    auto & mbc = brace_urdf.mbc;
    auto braceCoM = rbd::computeCoM(brace_urdf.mb, brace_urdf.mbc);
    mc_rtc::log::info("Brace CoM: {}", braceCoM.transpose());
    mc_rtc::log::info("Total Mass: {}", totalMass);
    mc_rtc::Configuration conf;
    conf.add("mass", totalMass);
    conf.add("centerOfMass", braceCoM);
    conf.add("inertia", inertia);
    Eigen::MatrixXd transformation = Eigen::MatrixXd::Identity(4,4);
    Eigen::VectorXd transfoVec = Eigen::VectorXd::Zero(16);
    for(int i = 0; i < transformation.rows(); i++)
    {
    	for(int j = 0; j < transformation.cols(); j++)
    	{
	  transfoVec(4*i + j) = transformation(i, j);
    	}
    }
    conf.add("transformation", transfoVec);

    // collision model?
    conf.add("collisionModel");
    Eigen::Matrix3d pointA = Eigen::Matrix3d::Zero();
    pointA(0,1) = 0.005;
    Eigen::Matrix3d pointB = Eigen::Matrix3d::Zero();
    pointB(0,1) = -0.005;
    Eigen::Vector3d radius{0.005, 0, 0};
    conf("collisionModel").add("pointA", pointA);
    conf("collisionModel").add("pointB", pointB);
    conf("collisionModel").add("radius", radius);
    return conf;
  };

  // XXX autocompute inertia instead
  auto mechanical_data_conf = generate_panda_mechanical_data(transformC("brace_inertia"));
  auto mechanical_data_path = bfs::temp_directory_path() / "brace_mechanical_data.json";
  mechanical_data_conf.save(mechanical_data_path.string());
  mc_rtc::log::info("Saved mechanical data file for the brace attachement to {}", mechanical_data_path.string());
  mc_rtc::log::info("Mechanical data is:\n {}", mechanical_data_conf.dump(true));

  _forceSensors.emplace_back("BraceTopForceSensor", "Link1", sva::PTransformd::Identity());

  const double i = 0.04;
  const double s = 0.02;
  const double d = 0.;
  _minimalSelfCollisions.emplace_back("panda_link0*", "Link2", i, s, d);
  _minimalSelfCollisions.emplace_back("panda_link1*", "Link2", i, s, d);
  _minimalSelfCollisions.emplace_back("panda_link2*", "Link2", i, s, d);
  _minimalSelfCollisions.emplace_back("panda_link3*", "Link2", i, s, d);
  _minimalSelfCollisions.emplace_back("panda_link4*", "Link2", i, s, d);

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
