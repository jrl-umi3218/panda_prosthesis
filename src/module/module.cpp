#include "module.h"

#include "config.h"

#include <RBDyn/parsers/urdf.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_robots
{

PandaProsthesisRobotModule::PandaProsthesisRobotModule(const std::string & prosthesis)
: mc_robots::PandaRobotModule(false, false, false)
{
  auto convexes = bfs::path(panda_prosthesis::convex_DIR);
  auto meshes = bfs::path(panda_prosthesis::meshes_DIR);
  auto inertias = bfs::path(panda_prosthesis::inertia_DIR);
  auto transforms = bfs::path(panda_prosthesis::transforms_DIR);

  auto addBody = [&](const std::string name) {
    auto mesh = meshes / (name + ".stl");
    auto convex = convexes / (name + "-ch.txt");
    auto inertia = inertias / (name + ".yml");
    if(!bfs::exists(mesh))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Invalid prosthesis {}, no mesh found {}", prosthesis,
                                                       mesh.string());
    }
    if(!bfs::exists(convex))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Invalid prosthesis {}, no convex found {}", prosthesis,
                                                       convex.string());
    }
    if(!bfs::exists(inertia))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Invalid prosthesis {}, no inertia found {}", prosthesis,
                                                       inertia.string());
    }
    auto inertiaC = mc_rtc::Configuration(inertia.string());
    double mass = inertiaC("mass");
    Eigen::Vector3d com = inertiaC("com");
    Eigen::Vector6d inertiaV = inertiaC("inertia");
    Eigen::Matrix3d inertiaM;
    // clang-format off
    inertiaM << inertiaV(0), inertiaV(1), inertiaV(2),
                        0.0, inertiaV(3), inertiaV(4),
                        0.0,         0.0, inertiaV(5);
    // clang-format on
    inertiaM = inertiaM.selfadjointView<Eigen::Upper>();
    mbg.addBody({mass, com, inertiaM, name});

    rbd::parsers::Geometry::Mesh geom_mesh;
    geom_mesh.scale = 0.001;
    geom_mesh.filename = mesh.string();
    rbd::parsers::Geometry geom;
    geom.data = geom_mesh;
    geom.type = rbd::parsers::Geometry::MESH;
    _visual[name] = {{name, sva::PTransformd::Identity(), geom, {}}};
    _collision[name] = {{name, sva::PTransformd::Identity(), geom, {}}};
    _convexHull[name] = {name, convex.string()};
    _collisionTransforms[name] = sva::PTransformd::Identity();
  };

  addBody(prosthesis);
  addBody("support_" + prosthesis);

  auto transform = transforms / (prosthesis + ".yml");
  if(!bfs::exists(transform))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Invalid prosthesis {}, no transform found {}", prosthesis,
                                                     transform.string());
  }
  auto transformC = mc_rtc::Configuration(transform.string());

  mbg.addJoint({rbd::Joint::Type::Fixed, true, "ee_to_support"});
  sva::PTransformd ee_to_support = transformC("ee_to_support");
  mbg.linkBodies("panda_link8", ee_to_support, "support_" + prosthesis, sva::PTransformd::Identity(), "ee_to_support");

  mbg.addJoint({rbd::Joint::Type::Fixed, true, "support_to_prosthesis"});
  sva::PTransformd support_to_prosthesis = transformC("support_to_prosthesis");
  mbg.linkBodies("support_" + prosthesis, support_to_prosthesis, prosthesis, sva::PTransformd::Identity(),
                 "support_to_prosthesis");

  mb = mbg.makeMultiBody(mb.body(0).name(), true);
  mbc = rbd::MultiBodyConfig(mb);
  mbc.zero(mb);

  auto urdf_path = bfs::temp_directory_path() / ("panda_" + prosthesis + ".urdf");
  {
    rbd::parsers::Limits limits;
    limits.lower = _bounds[0];
    limits.upper = _bounds[1];
    limits.velocity = _bounds[3];
    limits.torque = _bounds[5];
    std::ofstream ofs(urdf_path.string());
    ofs << rbd::parsers::to_urdf({mb, mbc, mbg, limits, _visual, _collision, "panda_" + prosthesis});
  }
  this->urdf_path = urdf_path.string();
  this->name = "panda_" + prosthesis;
  mc_rtc::log::info("Wrote URDF to {}", urdf_path.string());
}

PandaBraceRobotModule::PandaBraceRobotModule() : mc_robots::PandaRobotModule(false, false, false)
{

  // Merge with brace_top_setup urdf here
  auto merge_urdf = [this](const std::string & urdf_file, const std::string & merge_root,
                           const std::string & merge_with_link) {
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
        if(first_add)
        {
          mc_rtc::log::info("[PandaBrace] Add body {}", predBName);
          mbg.addBody(bodies[predBi]);
          _visual[predBName] = brace_urdf.visual[predBName];
          _collision[predBName] = brace_urdf.collision[predBName];
          first_add = false;
        }

        mc_rtc::log::info("[PandaBrace] Add joint {}", j.name());
        mbg.addJoint(j);

        mc_rtc::log::info("[PandaBrace] Add body {}", succBName);
        mbg.addBody(bodies[succBi]);
        _visual[succBName] = brace_urdf.visual[succBName];
        _collision[succBName] = brace_urdf.collision[succBName];

        mc_rtc::log::info("[PandaBrace] Link body {} to joint {} with transform {}", predBName, j.name(),
                          trans[ji].translation().transpose());
        mbg.linkBodies(predBName, trans[ji], succBName, sva::PTransformd::Identity(), j.name());
      }
    }
    auto merge_fixed_joint = merge_root + "_" + merge_with_link + "_joint";
    mc_rtc::log::info("Add fixed joint {} between {} and {}", merge_fixed_joint, merge_root, merge_with_link);
    mbg.addJoint({rbd::Joint::Type::Fixed, true, merge_fixed_joint});
    mbg.linkBodies(merge_root, sva::PTransformd::Identity(), merge_with_link, sva::PTransformd::Identity(),
                   merge_fixed_joint);

    mb = mbg.makeMultiBody(mb.body(0).name(), true);
    mbc = rbd::MultiBodyConfig(mb);
    mbc.zero(mb);
  };
  merge_urdf(panda_prosthesis::brace_top_setup_DIR + std::string{"/urdf/brace_top_setup.urdf"}, "panda_link8",
             "base_link");

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
