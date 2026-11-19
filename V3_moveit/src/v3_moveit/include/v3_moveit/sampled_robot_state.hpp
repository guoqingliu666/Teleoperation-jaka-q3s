#pragma once
#include <moveit/robot_state/robot_state.hpp>
#include <array>
#include <cmath>

namespace v3_moveit {
// A prospective curve sample must carry its own q/v/a. Retaining measured
// velocity from a different timestamp can falsely reject geometry near a limit.
// Position bounds are exact; v/a use the existing native interval tolerance.
inline bool set_sampled_robot_state(moveit::core::RobotState& state,
    const moveit::core::JointModelGroup* group,const std::array<double,6>& q,
    const std::array<double,6>& v,const std::array<double,6>& a,
    const std::array<double,6>& vmax,const std::array<double,6>& amax) {
  if(!group||group->getVariableCount()!=6)return false;
  for(size_t i=0;i<6;++i)
    if(!std::isfinite(q[i])||!std::isfinite(v[i])||!std::isfinite(a[i])||
       std::abs(v[i])>vmax[i]+1e-8||std::abs(a[i])>amax[i]+1e-8)return false;
  state.setJointGroupPositions(group,q.data());
  state.setJointGroupVelocities(group,v.data());
  state.setJointGroupAccelerations(group,a.data());state.update();
  for(const auto* joint:group->getActiveJointModels())
    if(!state.satisfiesPositionBounds(joint))return false;
  return true;
}
}
