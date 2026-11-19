#pragma once
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace v3_moveit {
using Joints = std::array<double, 6>;
// 这层只验证输入，不计算 IK，也不尝试多个速度候选。
struct Guard {
  Joints lower{}, upper{}, previous{}, previous_velocity{}, previous_acceleration{};
  double max_velocity{0.1745329252}, max_acceleration{0.6981317008}, max_jerk{2.7925268032};
  double tracking_error{0.0872664626};
  bool initialized{false};
  double previous_dt{0.}, previous_previous_dt{0.};
  // Reset represents a stationary start; the very first target must also obey jerk.
  void reset(const Joints& q) { previous=q; previous_velocity.fill(0); previous_acceleration.fill(0); previous_dt=0.; previous_previous_dt=0.; initialized=true; }
  void check(const Joints& q, const Joints& measured, double dt) {
    if (!initialized || !std::isfinite(dt) || dt < .004 || dt > .012)
      throw std::runtime_error("invalid control interval");
    // Missing history is a stationary extension, using the first actual cycle length.
    const double h1=previous_dt>0.?previous_dt:dt;
    const double h2=previous_previous_dt>0.?previous_previous_dt:h1;
    Joints velocity{}, acceleration{};
    for (size_t i=0; i<6; ++i) {
      if (!std::isfinite(q[i]) || !std::isfinite(measured[i])) throw std::runtime_error("nonfinite joint");
      if (q[i]<lower[i] || q[i]>upper[i]) throw std::runtime_error("joint limit");
      if (std::abs(q[i]-measured[i])>tracking_error) throw std::runtime_error("following error");
      velocity[i]=(q[i]-previous[i])/dt;
      // Factorial-scaled divided differences for unequal sample intervals.
      acceleration[i]=2.*(velocity[i]-previous_velocity[i])/(dt+h1);
      if (std::abs(velocity[i])>max_velocity*1.02+1e-7) throw std::runtime_error("joint target velocity exceeded");
      // 插值点数值误差容差写明；不能靠裁剪或静默重规划掩盖违规目标。
      if (std::abs(acceleration[i])>max_acceleration*1.10+1e-5) throw std::runtime_error("joint target acceleration exceeded");
      const double jerk=3.*(acceleration[i]-previous_acceleration[i])/(dt+h1+h2);
      // Third differences amplify position rounding; tolerance is in rad/s^3.
      if (std::abs(jerk)>max_jerk+1e-3) throw std::runtime_error("joint target jerk exceeded");
    }
    // Commit positions and timing together only after every joint passes.
    previous=q; previous_velocity=velocity; previous_acceleration=acceleration;
    previous_previous_dt=h1; previous_dt=dt;
  }
};
}
