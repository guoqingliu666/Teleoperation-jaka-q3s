#pragma once
#include <ruckig/ruckig.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace v3_moveit {
using PathVector = std::vector<double>;
struct PathLimits { PathVector velocity, acceleration, jerk; };
struct PathSample { double time; PathVector position, velocity, acceleration; std::size_t segment; };
struct PathRetimingOptions {
  double sample_period{.008}, path_tolerance{1e-10}, max_duration{120.}, max_compute_seconds{2.};
  std::size_t max_input_points{8192}, max_vertices{512}, max_samples{20000};
};
struct RetimedPath { std::vector<PathVector> vertices; std::vector<PathSample> samples; };

inline double path_distance(const PathVector& a,const PathVector& b) {
  double result=0.;for(std::size_t i=0;i<a.size();++i)result=std::max(result,std::abs(a[i]-b[i]));return result;
}
inline bool on_line_segment(const PathVector& q,const PathVector& a,const PathVector& b,double tolerance) {
  std::size_t pivot=0;
  for(std::size_t i=1;i<a.size();++i)if(std::abs(b[i]-a[i])>std::abs(b[pivot]-a[pivot]))pivot=i;
  const double span=b[pivot]-a[pivot];
  if(std::abs(span)<=tolerance)return path_distance(q,a)<=tolerance;
  const double phase=(q[pivot]-a[pivot])/span;
  if(phase < -tolerance/std::abs(span) || phase > 1.+tolerance/std::abs(span))return false;
  for(std::size_t i=0;i<a.size();++i)if(std::abs(q[i]-(a[i]+phase*(b[i]-a[i])))>tolerance)return false;
  return true;
}
inline std::vector<PathVector> reduce_path(const std::vector<PathVector>& path,double tolerance) {
  std::vector<PathVector> result;
  for(const auto& q:path) {
    if(!result.empty() && path_distance(result.back(),q)<=tolerance)continue;
    while(result.size()>=2 && on_line_segment(result.back(),result[result.size()-2],q,tolerance))result.pop_back();
    result.push_back(q);
  }
  return result;
}

// Keeps the geometric polyline. Every retained corner is a stationary stop;
// this intentionally trades traversal speed for exact path preservation.
inline RetimedPath retime_path(const std::vector<PathVector>& path,const PathLimits& limits,
                              const PathRetimingOptions& options={}) {
  if(path.empty() || path.size()>options.max_input_points || path.front().empty())
    throw std::runtime_error("invalid or excessive geometric path");
  const auto dofs=path.front().size();
  if(!(std::isfinite(options.sample_period) && options.sample_period>0. &&
       std::isfinite(options.path_tolerance) && options.path_tolerance>0. &&
       std::isfinite(options.max_duration) && options.max_duration>0. &&
       std::isfinite(options.max_compute_seconds) && options.max_compute_seconds>0.))
    throw std::runtime_error("invalid retiming options");
  for(const auto* values:{&limits.velocity,&limits.acceleration,&limits.jerk}) {
    if(values->size()!=dofs)throw std::runtime_error("limit dimensions");
    for(double value:*values)if(!std::isfinite(value) || value<=0.)throw std::runtime_error("invalid retiming limit");
  }
  for(const auto& q:path) {
    if(q.size()!=dofs)throw std::runtime_error("path dimensions");
    for(double value:q)if(!std::isfinite(value))throw std::runtime_error("nonfinite path");
  }
  const auto begin=std::chrono::steady_clock::now();
  auto budget=[&] {
    if(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()>options.max_compute_seconds)
      throw std::runtime_error("path retiming computation budget exceeded");
  };
  RetimedPath result;result.vertices=reduce_path(path,options.path_tolerance);
  if(result.vertices.size()>options.max_vertices)throw std::runtime_error("too many path corners");
  // Local collinearity tolerances must not accumulate into a larger shortcut.
  std::size_t retained_segment=0;
  for(const auto& original:path) {
    if(retained_segment+1==result.vertices.size()) {
      if(path_distance(original,result.vertices.back())>options.path_tolerance)
        throw std::runtime_error("path reduction changed original geometry");
    }else {
      if(!on_line_segment(original,result.vertices[retained_segment],result.vertices[retained_segment+1],options.path_tolerance))
        throw std::runtime_error("path reduction changed original geometry");
      if(path_distance(original,result.vertices[retained_segment+1])<=options.path_tolerance)++retained_segment;
    }
  }
  const PathVector zero(dofs,0.);
  result.samples.push_back({0.,result.vertices.front(),zero,zero,0});
  if(options.max_samples<1)throw std::runtime_error("path sample budget exceeded");
  std::int64_t offset_ns=0;
  for(std::size_t segment=0;segment+1<result.vertices.size();++segment) {
    budget();
    ruckig::InputParameter<ruckig::DynamicDOFs> input(dofs);
    input.current_position=result.vertices[segment];input.target_position=result.vertices[segment+1];
    input.current_velocity=zero;input.current_acceleration=zero;
    input.target_velocity=zero;input.target_acceleration=zero;
    input.max_velocity=limits.velocity;input.max_acceleration=limits.acceleration;input.max_jerk=limits.jerk;
    input.synchronization=ruckig::Synchronization::Phase;
    input.duration_discretization=ruckig::DurationDiscretization::Discrete;
    ruckig::Ruckig<ruckig::DynamicDOFs> generator(dofs,options.sample_period);
    ruckig::Trajectory<ruckig::DynamicDOFs> trajectory(dofs);
    const auto status=generator.calculate(input,trajectory);
    if(status!=ruckig::Result::Working && status!=ruckig::Result::Finished)
      throw std::runtime_error("Ruckig path retiming failed");
    const double duration=trajectory.get_duration();
    if(!std::isfinite(duration) || duration<=0. || offset_ns*1e-9+duration>options.max_duration)
      throw std::runtime_error("retimed path duration exceeded");
    const auto duration_ns=static_cast<std::int64_t>(std::llround(duration*1e9));
    if(duration_ns<=0)throw std::runtime_error("unrepresentable segment duration");
    std::vector<std::int64_t> times{0,duration_ns};
    const auto grid_count=static_cast<std::size_t>(std::ceil(duration/options.sample_period));
    if(grid_count>options.max_samples || result.samples.size()+grid_count>options.max_samples)
      throw std::runtime_error("path sample budget exceeded");
    for(std::size_t k=1;k<grid_count;++k)times.push_back(std::llround(k*options.sample_period*1e9));
    for(const auto& section:trajectory.get_profiles())for(const auto& profile:section) {
      // Rest-to-rest inputs need no pre-brake. Reject unexpected profiles.
      if(profile.brake.duration>1e-12 || profile.accel.duration>1e-12)
        throw std::runtime_error("unexpected Ruckig braking profile");
      for(double switch_time:profile.t_sum)
        if(switch_time>0. && switch_time<duration)times.push_back(std::llround(switch_time*1e9));
    }
    std::sort(times.begin(),times.end());times.erase(std::unique(times.begin(),times.end()),times.end());
    if(result.samples.size()+times.size()-1>options.max_samples)throw std::runtime_error("path sample budget exceeded");
    auto sample=[&](double time) {
      PathSample point{offset_ns*1e-9+time,PathVector(dofs),PathVector(dofs),PathVector(dofs),segment};
      trajectory.at_time(time,point.position,point.velocity,point.acceleration);
      if(!on_line_segment(point.position,input.current_position,input.target_position,options.path_tolerance))
        throw std::runtime_error("Ruckig phase fallback left geometric segment");
      std::size_t pivot=0;
      for(std::size_t i=1;i<dofs;++i)
        if(std::abs(input.target_position[i]-input.current_position[i])>std::abs(input.target_position[pivot]-input.current_position[pivot]))pivot=i;
      const double direction=input.target_position[pivot]-input.current_position[pivot];
      for(std::size_t i=0;i<dofs;++i) {
        const double ratio=(input.target_position[i]-input.current_position[i])/direction;
        if(std::abs(point.velocity[i]-ratio*point.velocity[pivot])>1e-9 ||
           std::abs(point.acceleration[i]-ratio*point.acceleration[pivot])>1e-9)
          throw std::runtime_error("Ruckig phase derivatives left geometric segment");
        if(!std::isfinite(point.position[i]) || !std::isfinite(point.velocity[i]) || !std::isfinite(point.acceleration[i]) ||
           std::abs(point.velocity[i])>limits.velocity[i]+1e-9 || std::abs(point.acceleration[i])>limits.acceleration[i]+1e-9)
          throw std::runtime_error("invalid Ruckig sampled state");
      }
      return point;
    };
    for(std::size_t i=1;i<times.size();++i) {
      budget();
      // Midpoints also detect non-phase fallback between profile switches.
      sample((times[i-1]+times[i])*.5e-9);
      auto point=sample(times[i]*1e-9);
      point.time=(offset_ns+times[i])*1e-9;
      if(i+1==times.size()) {
        if(path_distance(point.position,input.target_position)>options.path_tolerance ||
           path_distance(point.velocity,zero)>1e-9 || path_distance(point.acceleration,zero)>1e-9)
          throw std::runtime_error("Ruckig did not reach stationary corner");
        point.position=input.target_position;point.velocity=zero;point.acceleration=zero;
      }
      result.samples.push_back(std::move(point));
    }
    offset_ns+=duration_ns;
  }
  return result;
}
}  // namespace v3_moveit
