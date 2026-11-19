#pragma once
#include "v3_moveit/guard.hpp"
#include <stdexcept>
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>
#include <ruckig/ruckig.hpp>

namespace v3_moveit {
// Kept independent of ROS so the exact production profile can be fault-tested.
class JointStopProfile {
 public:
  using Joints=std::array<double,6>;
  static constexpr double validated_max_duration=1.5;
  static bool limits_match_contract(const Joints& velocity,const Joints& acceleration,
                                    const Joints& jerk,double max_duration) {
    const Guard limits;
    if(!std::isfinite(max_duration)||std::abs(max_duration-validated_max_duration)>1e-9)return false;
    for(size_t i=0;i<6;++i)if(!std::isfinite(velocity[i])||!std::isfinite(acceleration[i])||!std::isfinite(jerk[i])||
        std::abs(velocity[i]-limits.max_velocity)>1e-9||
        std::abs(acceleration[i]-limits.max_acceleration)>1e-9||
        std::abs(jerk[i]-limits.max_jerk)>1e-9)return false;
    return true;
  }
  bool start(const std::vector<double>& q,const std::vector<double>& v,const std::vector<double>& a,
             const Joints& vmax,const Joints& amax,const Joints& jmax,double max_duration) {
    ready_=false;elapsed_=0.;error_.clear();
    if(q.size()!=6||v.size()!=6||a.size()!=6)return reject("missing commanded position/velocity/acceleration");
    ruckig::InputParameter<6> in;
    in.control_interface=ruckig::ControlInterface::Velocity;
    in.synchronization=ruckig::Synchronization::None;
    in.target_position.fill(0.);in.target_velocity.fill(0.);in.target_acceleration.fill(0.);
    for(size_t i=0;i<6;++i){
      if(!std::isfinite(q[i])||!std::isfinite(v[i])||!std::isfinite(a[i])||
         !std::isfinite(vmax[i])||!std::isfinite(amax[i])||!std::isfinite(jmax[i])||
         vmax[i]<=0||amax[i]<=0||jmax[i]<=0)return reject("nonfinite state or invalid brake limits");
      if(std::abs(v[i])>vmax[i]+1e-8||std::abs(a[i])>amax[i]+1e-8)
        return reject("commanded state exceeds braking limits");
      // Velocity-mode Ruckig does not enforce velocity limits. A positive initial
      // acceleration must first be removed at finite jerk, and likewise below zero.
      const double upper_peak=v[i]+std::pow(std::max(a[i],0.),2)/(2*jmax[i]);
      const double lower_peak=v[i]-std::pow(std::max(-a[i],0.),2)/(2*jmax[i]);
      if(upper_peak>vmax[i]+1e-8||lower_peak< -vmax[i]-1e-8)
        return reject("initial acceleration cannot be removed within velocity limits");
      in.current_position[i]=q[i];in.current_velocity[i]=v[i];in.current_acceleration[i]=a[i];
      in.max_velocity[i]=vmax[i];in.max_acceleration[i]=amax[i];in.max_jerk[i]=jmax[i];
      start_q_[i]=q[i];
    }
    const auto result=solver_.calculate(in,trajectory_);
    if(result<0)return reject("Ruckig rejected brake state: "+std::to_string(result));
    duration_=trajectory_.get_duration();
    if(!std::isfinite(duration_)||duration_<0||!std::isfinite(max_duration)||max_duration<=0||duration_>max_duration)
      return reject("brake duration exceeds explicit budget");
    trajectory_.at_time(duration_,terminal_q_,velocity_,acceleration_);
    // Ruckig Velocity mode leaves the unused target_position in Profile::pf.
    // Its extrema helper still includes pf; use the actual terminal position in
    // a profile COPY so this diagnostic never invents a visit to position zero.
    if(trajectory_.get_profiles().size()!=1)return reject("unexpected multisection stop profile");
    for(size_t i=0;i<6;++i){
      if(std::abs(velocity_[i])>1e-8||std::abs(acceleration_[i])>1e-8)
        return reject("Ruckig terminal velocity/acceleration is not zero");
      displacement_[i]=terminal_q_[i]-start_q_[i];
      auto profile=trajectory_.get_profiles().front()[i];
      profile.pf=terminal_q_[i];
      const auto extrema=profile.get_position_extrema();
      lower_[i]=extrema.min;upper_[i]=extrema.max;
      excursion_[i]=std::max(std::abs(lower_[i]-start_q_[i]),std::abs(upper_[i]-start_q_[i]));
      minimum_time_[i]=extrema.t_min;maximum_time_[i]=extrema.t_max;
      if(!std::isfinite(excursion_[i]))return reject("nonfinite brake position envelope");
    }
    position_=start_q_;std::copy(v.begin(),v.end(),velocity_.begin());std::copy(a.begin(),a.end(),acceleration_.begin());
    ready_=true;return true;
  }
  bool step(double dt){
    if(!ready_||!std::isfinite(dt)||dt<.004||dt>.012)return reject("invalid brake control interval");
    elapsed_+=dt;
    // at_time beyond the terminal time returns the zero-v/a terminal state.
    // The caller must write this last sample before acknowledging cancellation.
    trajectory_.at_time(elapsed_,position_,velocity_,acceleration_);
    for(size_t i=0;i<6;++i)if(!std::isfinite(position_[i])||!std::isfinite(velocity_[i])||!std::isfinite(acceleration_[i]))
      return reject("nonfinite Ruckig brake sample");
    return true;
  }
  bool finished()const{return ready_&&elapsed_>=duration_;}
  double duration()const{return duration_;}
  double elapsed()const{return elapsed_;}
  const Joints& position()const{return position_;}
  const Joints& velocity()const{return velocity_;}
  const Joints& acceleration()const{return acceleration_;}
  const Joints& displacement()const{return displacement_;}
  const Joints& excursion()const{return excursion_;}
  const Joints& lower_position()const{return lower_;}
  const Joints& upper_position()const{return upper_;}
  // Only the non-real-time planning validator constructs this sample schedule.
  std::vector<double> validation_times()const {
    if(!ready_)throw std::runtime_error("stop profile not ready");
    std::vector<double> times{0.,duration_};
    for(size_t k=1;k*.008<duration_;++k)times.push_back(k*.008);
    for(size_t i=0;i<6;++i){
      times.push_back(minimum_time_[i]);times.push_back(maximum_time_[i]);
      const auto& profile=trajectory_.get_profiles().front()[i];
      if(profile.brake.duration>0.){
        times.push_back(profile.brake.t[0]);times.push_back(profile.brake.duration);
      }
      for(double t:profile.t_sum)times.push_back(profile.brake.duration+t);
    }
    for(double time:times)if(!std::isfinite(time)||time<0.||time>duration_+1e-9)
      throw std::runtime_error("invalid stop validation time");
    for(auto& time:times)time=std::min(time,duration_);
    std::sort(times.begin(),times.end());times.erase(std::unique(times.begin(),times.end()),times.end());
    return times;
  }
  bool sample_at(double time,Joints& q,Joints& v,Joints& a)const {
    if(!ready_||!std::isfinite(time)||time<0.||time>duration_+1e-9)return false;
    trajectory_.at_time(std::min(time,duration_),q,v,a);
    for(size_t i=0;i<6;++i)if(!std::isfinite(q[i])||!std::isfinite(v[i])||!std::isfinite(a[i]))return false;
    return true;
  }
  const std::string& error()const{return error_;}
 private:
  bool reject(const std::string& reason){error_=reason;ready_=false;return false;}
  ruckig::Ruckig<6> solver_{.008};ruckig::Trajectory<6> trajectory_;
  Joints start_q_{},terminal_q_{},position_{},velocity_{},acceleration_{},displacement_{},excursion_{},lower_{},upper_{},minimum_time_{},maximum_time_{};
  double duration_{0},elapsed_{0};bool ready_{false};std::string error_;
};
}

