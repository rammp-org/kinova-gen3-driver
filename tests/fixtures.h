// Shared test fixtures built from the level 2F-85 model at URDF_PATH.
#pragma once

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

namespace kinova::test_fixtures {

inline std::string read_file(const std::string& path) {
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

inline void replace_once(std::string& s, const std::string& from, const std::string& to) {
  const auto pos = s.find(from);
  ASSERT_NE(pos, std::string::npos) << "fixture text not found: " << from;
  s.replace(pos, from.size(), to);
}

constexpr double kTilt = M_PI / 6.0;  // 30 deg, new_adapter_plate_v3

// The fixture with its root joint (world -> gen3_base_link) pitched +30 deg about Y:
// the flatbot / gen_1_5 mount. Root "world" plays the role of arm_mount.
inline std::string tilted_fixture() {
  std::string xml = read_file(URDF_PATH);
  replace_once(xml,
               "<joint name=\"gen3_base_joint\" type=\"fixed\">\n"
               "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>",
               "<joint name=\"gen3_base_joint\" type=\"fixed\">\n"
               "    <origin rpy=\"0 0.52359877559829887 0\" xyz=\"0 0 0\"/>");
  return xml;
}

inline Eigen::Matrix3d Ry(double a) {
  return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitY()).toRotationMatrix();
}

}  // namespace kinova::test_fixtures
