#pragma once
#include <ruckig/ruckig.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace v3_moveit {
struct ServoIntervalPoint {
  std::int64_t time_ns;
  std::array<double,6> position,velocity,acceleration;
};

// Preserve native Ruckig p/v/a at every jerk boundary within one command tick.
// A JTC quintic between constant-jerk endpoints reproduces the original cubic;
// endpoint-only sampling across a jerk switch does not have this property.
// Time is relative to begin. Nanosecond quantization is required by ROS messages:
// evaluate at the represented instant, never relabel a state from another time.
inline std::vector<ServoIntervalPoint> sample_servo_interval(
    const ruckig::Trajectory<6>& trajectory,double begin,double end) {
  constexpr std::int64_t tick_ns=8000000;
  if(!std::isfinite(begin)||!std::isfinite(end)||begin<0.||end<=begin||
     std::abs((end-begin)-.008)>1e-11||!std::isfinite(trajectory.get_duration())||
     trajectory.get_duration()<0.)
    throw std::runtime_error("invalid native Servo interval");
  const auto profiles=trajectory.get_profiles();
  if(profiles.size()!=1||trajectory.degrees_of_freedom!=6)
    throw std::runtime_error("unsupported native Servo trajectory sections");
  std::vector<std::int64_t> times;times.reserve(64);times.push_back(0);times.push_back(tick_ns);
  auto add=[&](double time) {
    if(!std::isfinite(time)||time<0.)throw std::runtime_error("invalid native Servo phase time");
    if(time>begin&&time<end) {
      const auto ns=static_cast<std::int64_t>(std::llround((time-begin)*1e9));
      if(ns>0&&ns<tick_ns)times.push_back(ns);
    }
  };
  for(const auto& profile:profiles.front()) {
    if(!std::isfinite(profile.brake.duration)||profile.brake.duration<0.||
       !std::isfinite(profile.accel.duration)||profile.accel.duration!=0.)
      throw std::runtime_error("unsupported native Servo post-acceleration profile");
    // Ruckig 0.9.2 at_time() handles a two-part pre-brake, but its post-accel
    // evaluation is disabled. Fail closed if a future version supplies one.
    if(profile.brake.duration>0.) {
      if(!std::isfinite(profile.brake.t[0])||!std::isfinite(profile.brake.t[1])||
         profile.brake.t[0]<0.||profile.brake.t[1]<0.||
         std::abs(profile.brake.t[0]+profile.brake.t[1]-profile.brake.duration)>1e-11)
        throw std::runtime_error("invalid native Servo pre-brake profile");
      add(profile.brake.t[0]);add(profile.brake.duration);
    }
    double previous=0.;
    for(double time:profile.t_sum) {
      if(!std::isfinite(time)||time<previous)
        throw std::runtime_error("unordered native Servo phases");
      add(profile.brake.duration+time);previous=time;
    }
  }
  add(trajectory.get_duration());
  std::sort(times.begin(),times.end());times.erase(std::unique(times.begin(),times.end()),times.end());
  if(times.size()>64)throw std::runtime_error("native Servo interval knot budget exceeded");
  std::vector<ServoIntervalPoint> result;result.reserve(times.size());
  for(const auto ns:times) {
    ServoIntervalPoint point;point.time_ns=ns;
    // Keep the final output exactly equal to OutputParameter::new_*.
    const double time=ns==tick_ns?end:begin+static_cast<double>(ns)*1e-9;
    trajectory.at_time(time,point.position,point.velocity,point.acceleration);
    for(std::size_t joint=0;joint<6;++joint)
      if(!std::isfinite(point.position[joint])||!std::isfinite(point.velocity[joint])||
         !std::isfinite(point.acceleration[joint]))
        throw std::runtime_error("nonfinite native Servo interval state");
    result.push_back(point);
  }
  // Position extrema inside a constant-jerk segment do not require extra knots:
  // the polynomial already retains them; collision/position checks stay separate.
  return result;
}
} // namespace v3_moveit
