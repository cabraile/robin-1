#ifndef ROBIN_CORE_CPP_CORE_H
#define ROBIN_CORE_CPP_CORE_H

#include <eigen3/Eigen/Geometry>

namespace robin_core
{

using Vector3    = Eigen::Vector3d;
using Pose3      = Eigen::Isometry3d;
using Quaternion = Eigen::Quaterniond;

void dummy();

} // namespace robin_core

#endif