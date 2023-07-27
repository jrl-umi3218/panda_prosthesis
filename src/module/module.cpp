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
    // geom_mesh.scaleV = Eigen::Vector3d{0.001, 0.001, 0.001};
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

} // namespace mc_robots
