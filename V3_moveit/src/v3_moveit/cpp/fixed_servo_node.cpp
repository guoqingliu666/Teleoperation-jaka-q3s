/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/


/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Andrew Zelenak
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Transport adapter for MoveIt Servo 2.12.4 and JTC 4.40.1.
// Retains MoveIt IK, collision/singularity scaling and joint bounds. The native
// Ruckig target rule below follows MoveIt's BSD-licensed ruckig_filter.cpp;
// errors are rejected instead of forwarding an unfiltered target.
//
// A controller_state header denotes its reference tick; output.positions is
// the actual position command at header + 8 ms in SmoothTrajectoryController.
// Controller stamps, never callback arrival times, define this target grid.
// The 32 ms queued horizon is explicit command latency, not a physical stop
// guarantee. Physical deadlines, tracking and final command Guard are separate.
#include "v3_moveit/joint_stop_profile.hpp"
#include "v3_moveit/servo_interval.hpp"
#include "v3_moveit/sampled_robot_state.hpp"
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/servo_status.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <csignal>
#include <deque>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace v3_moveit {
class FixedServoNode {
 public:
  explicit FixedServoNode(const rclcpp::Node::SharedPtr& node):node_(node) {
    velocity_tracking_=node_->get_parameter_or<bool>("velocity_tracking",false);
    const auto mode=node_->get_parameter_or<std::string>("runtime_mode","");
    if(mode!="fake"&&mode!="hardware")throw std::runtime_error("runtime_mode must be fake or hardware");
    listener_=std::make_shared<servo::ParamListener>(node_,"moveit_servo");
    params_=listener_->get_params();
    if(std::abs(params_.publish_period-.008)>1e-12||std::abs(params_.max_expected_latency-.032)>1e-12||
       params_.use_smoothing||!params_.check_collisions||params_.move_group_name!="arm"||
       params_.command_out_type!="trajectory_msgs/JointTrajectory"||
       !params_.publish_joint_positions||!params_.publish_joint_velocities||!params_.publish_joint_accelerations)
      throw std::runtime_error("fixed Servo requires arm, 8 ms / 32 ms, collision checks, full q/v/a and internal smoothing disabled");
    scene_=moveit_servo::createPlanningSceneMonitor(node_,params_);
    servo_=std::make_unique<moveit_servo::Servo>(node_,listener_,scene_);
    servo_->setCommandType(moveit_servo::CommandType::POSE);
    group_=scene_->getRobotModel()->getJointModelGroup(params_.move_group_name);
    if(!group_||group_->getVariableCount()!=6)throw std::runtime_error("fixed Servo requires six joints");
    names_=group_->getVariableNames();
    for(size_t i=0;i<6;++i){
      if(names_[i]!="joint_"+std::to_string(i+1))throw std::runtime_error("unexpected Servo joint order");
      const auto& b=scene_->getRobotModel()->getVariableBounds(names_[i]);
      if(!b.position_bounded_||!b.velocity_bounded_||!b.acceleration_bounded_||!b.jerk_bounded_)
        throw std::runtime_error("Servo model requires bounded position, velocity, acceleration and jerk");
      vmax_[i]=std::min(-b.min_velocity_,b.max_velocity_);
      amax_[i]=std::min(-b.min_acceleration_,b.max_acceleration_);
      jmax_[i]=std::min(-b.min_jerk_,b.max_jerk_);
      generated_guard_.lower[i]=b.min_position_;generated_guard_.upper[i]=b.max_position_;
    }
    if(!JointStopProfile::limits_match_contract(vmax_,amax_,jmax_,JointStopProfile::validated_max_duration))
      throw std::runtime_error("Servo model dynamic limits differ from fixed Guard / stop contract");
    input_.max_velocity=vmax_;input_.max_acceleration=amax_;input_.max_jerk=jmax_;
    input_.synchronization=ruckig::Synchronization::Phase;
    if(velocity_tracking_){
      input_.control_interface=ruckig::ControlInterface::Velocity;
      input_.synchronization=ruckig::Synchronization::Time;
    }
    input_.target_acceleration.fill(0.);
    robot_state_=scene_->getStateMonitor()->getCurrentState();
    trajectory_pub_=node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(params_.command_out_topic,1);
    status_pub_=node_->create_publisher<moveit_msgs::msg::ServoStatus>(params_.status_topic,1);
    fault_pub_=node_->create_publisher<std_msgs::msg::String>("/v3/controller_fault",rclcpp::QoS(1).reliable().transient_local());
    state_sub_=node_->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      "/arm_controller/controller_state",rclcpp::QoS(16).reliable(),[this](const StateMessage::ConstSharedPtr msg){on_state(*msg);});
    pose_sub_=node_->create_subscription<geometry_msgs::msg::PoseStamped>(params_.pose_command_in_topic,1,
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg){if(!failed_&&!paused_){pose_=*msg;pose_receipt_=Clock::now();}});
    switch_service_=node_->create_service<moveit_msgs::srv::ServoCommandType>("~/switch_command_type",
      [this](const std::shared_ptr<moveit_msgs::srv::ServoCommandType::Request> request,
             std::shared_ptr<moveit_msgs::srv::ServoCommandType::Response> response){
        response->success=!failed_&&request->command_type==static_cast<int8_t>(moveit_servo::CommandType::POSE);
      });
    pause_service_=node_->create_service<std_srvs::srv::SetBool>("~/pause_servo",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response){pause(request->data,*response);});
    watchdog_=node_->create_wall_timer(std::chrono::milliseconds(8),[this]{
      if(initialized_&&!paused_&&!failed_&&seconds(Clock::now()-state_receipt_)>=.024)
        fail("controller command feedback expired; queued motion must be stopped by the hardware lease");
      publish_status();
    });
    RCLCPP_INFO(node_->get_logger(),"Fixed Servo target grid: 8 ms, queued horizon 32 ms; waiting for stationary command feedback");
  }
  ~FixedServoNode(){
    // The caller keeps the ROS context valid until all upstream worker threads
    // have joined. Servo::~Servo stops and joins CollisionMonitor; the scene
    // destructor then stops its monitor/publishing threads.
    watchdog_.reset();state_sub_.reset();pose_sub_.reset();
    servo_.reset();scene_.reset();
  }
 private:
  using Clock=std::chrono::steady_clock;
  using StateMessage=control_msgs::msg::JointTrajectoryControllerState;
  using Point=ServoIntervalPoint;
  static constexpr int64_t tick_ns=8000000,horizon_ns=32000000;
  static double seconds(Clock::duration dt){return std::chrono::duration<double>(dt).count();}
  static int64_t stamp_ns(const builtin_interfaces::msg::Time& stamp){return int64_t(stamp.sec)*1000000000+stamp.nanosec;}
  static bool stationary(const Point& p){
    for(size_t i=0;i<6;++i)if(std::abs(p.velocity[i])>1e-8||std::abs(p.acceleration[i])>1e-8)return false;
    return true;
  }
  static std::vector<double> vector(const Joints& a){return {a.begin(),a.end()};}
  static double max_difference(const Joints& a,const Joints& b){
    double d=0.;for(size_t i=0;i<6;++i)d=std::max(d,std::abs(a[i]-b[i]));return d;
  }
  bool command_ready()const{
    if(!have_state_||!quiet_since_||seconds(Clock::now()-state_receipt_)>=.024||
       seconds(Clock::now()-*quiet_since_)<.08)return false;
    const double age=(node_->now().nanoseconds()-state_stamp_)*1e-9;
    return age>=-.024&&age<=.024;
  }
  bool future_stationary()const{
    if(!initialized_)return true;
    if(!stationary(tail_)||stopping_)return false;
    for(const auto& p:window_)if(p.time_ns>=state_stamp_+tick_ns &&
        (!stationary(p)||max_difference(p.position,command_q_)>1e-10))return false;
    return true;
  }
  void parse_state(const StateMessage& msg){
    if(msg.joint_names.size()!=6||msg.output.positions.size()!=6||msg.reference.positions.size()!=6||
       msg.reference.velocities.size()!=6||msg.reference.accelerations.size()!=6||
       msg.feedback.positions.size()!=6||msg.feedback.velocities.size()!=6)
      throw std::runtime_error("incomplete controller command/reference/measured state");
    std::array<size_t,6> indices{};
    for(size_t i=0;i<6;++i){
      const auto first=std::find(msg.joint_names.begin(),msg.joint_names.end(),names_[i]);
      if(first==msg.joint_names.end()||std::count(msg.joint_names.begin(),msg.joint_names.end(),names_[i])!=1)
        throw std::runtime_error("ambiguous controller joint names");
      indices[i]=std::distance(msg.joint_names.begin(),first);
    }
    const auto now=Clock::now();const int64_t stamp=stamp_ns(msg.header.stamp);
    const double age=(node_->now().nanoseconds()-stamp)*1e-9;
    if(age<-.024||age>.024||stamp<=state_stamp_)
      throw std::runtime_error("stale/replayed controller target timestamp");
    if(initialized_&&!paused_&&stamp-state_stamp_!=tick_ns)
      throw std::runtime_error("controller target clock skipped a tick; targets are never relabeled or caught up");
    const bool continuous=have_state_&&seconds(now-state_receipt_)<.024;
    bool quiet=true;Joints q{};
    for(size_t i=0;i<6;++i){
      const auto n=indices[i];q[i]=msg.output.positions[n];
      const double reference=msg.reference.positions[n],v=msg.reference.velocities[n],a=msg.reference.accelerations[n];
      const double measured=msg.feedback.positions[n],measured_v=msg.feedback.velocities[n];
      if(!std::isfinite(q[i])||!std::isfinite(reference)||!std::isfinite(v)||!std::isfinite(a)||
         !std::isfinite(measured)||!std::isfinite(measured_v)||std::abs(q[i]-measured)>generated_guard_.tracking_error)
        throw std::runtime_error("invalid controller state or following error");
      quiet=quiet&&std::abs(q[i]-reference)<=1e-10&&std::abs(v)<=1e-8&&std::abs(a)<=1e-8&&std::abs(measured_v)<.002;
    }
    if(quiet){
      if(!continuous||!quiet_since_||max_difference(q,quiet_q_)>1e-10){quiet_since_=now;quiet_q_=q;}
    }else quiet_since_.reset();
    command_q_=q;state_stamp_=stamp;state_receipt_=now;have_state_=true;
  }
  void on_state(const StateMessage& msg){
    if(failed_)return;
    try{
      parse_state(msg);
      if(paused_){publish_status();return;}
      if(!initialized_){if(command_ready())initialize();publish_status();return;}
      const auto begin=Clock::now();
      const auto expected=std::find_if(window_.begin(),window_.end(),[this](const Point& p){return p.time_ns==state_stamp_+tick_ns;});
      if(expected==window_.end()||max_difference(expected->position,command_q_)>1e-10)
        throw std::runtime_error("controller output differs from immutable Servo target sequence");
      auto interval=next_interval();
      const auto& candidate=interval.back();
      Guard next_guard=generated_guard_;next_guard.check(candidate.position,candidate.position,.008);
      validate_interval_and_stop(interval,begin);
      if(seconds(Clock::now()-begin)>.006)
        throw std::runtime_error("Servo generation/scene validation exceeded 6 ms; no target submitted");
      append_interval(interval);
      generated_guard_=next_guard;
      publish_window();publish_status();
    }catch(const std::exception& error){fail(error.what());}
  }
  void initialize(){
    if(!command_ready())throw std::runtime_error("stationary commanded anchor is not fresh for 80 ms");
    tail_=Point{};tail_.position=command_q_;tail_.time_ns=state_stamp_+tick_ns;
    window_.clear();
    // An exact stationary bridge on the controller's grid, not a measured-q
    // reset and not a zero-header replay of old moving samples.
    for(int i=-1;i<=4;++i){auto p=tail_;p.time_ns+=i*tick_ns;window_.push_back(p);}
    tail_=window_.back();generated_guard_.reset(command_q_);
    reset_ruckig(tail_);stopping_=false;initialized_=true;
    publish_window();
  }
  void reset_ruckig(const Point& state){
    ruckig_.reset();output_=ruckig::OutputParameter<6>();
    input_.current_position=state.position;input_.current_velocity=state.velocity;input_.current_acceleration=state.acceleration;
    input_.target_position=state.position;input_.target_velocity.fill(0.);input_.target_acceleration.fill(0.);
  }
  void set_robot_state(){
    robot_state_->setJointGroupPositions(group_,tail_.position.data());
    robot_state_->setJointGroupVelocities(group_,tail_.velocity.data());
    robot_state_->setJointGroupAccelerations(group_,tail_.acceleration.data());robot_state_->update();
  }
  bool pose_fresh()const{
    if(!pose_||seconds(Clock::now()-pose_receipt_)>=params_.incoming_command_timeout)return false;
    const double age=(node_->now().nanoseconds()-stamp_ns(pose_->header.stamp))*1e-9;
    return age>=-.04&&age<params_.incoming_command_timeout;
  }
  std::vector<Point> next_interval(){
    set_robot_state();
    const bool follow=pose_fresh()&&!stopping_;
    moveit_servo::KinematicState raw;
    if(follow){
      servo_->setCommandType(moveit_servo::CommandType::POSE);
      auto command=moveit_servo::poseFromPoseStamped(*pose_);
      if(!moveit_servo::isValidCommand(command))throw std::runtime_error("invalid Pose command");
      if(velocity_tracking_){
        // A pose error servo with 240 ms response time. MoveIt still applies
        // its Cartesian/joint speed limits, IK, singularity and collision scale.
        // Feeding an 8 ms waypoint as a stopping destination to position-mode
        // Ruckig repeatedly accelerates/brakes. Smooth the requested velocity
        // instead; all emitted intervals and their stops are checked below.
        const auto tip=moveit_servo::getIKSolverTipFrame(robot_state_,params_.move_group_name);
        if(!tip||!robot_state_->knowsFrameTransform(command.frame_id))
          throw std::runtime_error("tracking requires a robot-model pose frame and IK tip");
        const Eigen::Isometry3d current=robot_state_->getGlobalLinkTransform(command.frame_id).inverse()*
            robot_state_->getGlobalLinkTransform(*tip);
        constexpr double fraction=.008/.24;
        command.pose.translation()=current.translation()+fraction*(command.pose.translation()-current.translation());
        command.pose.linear()=Eigen::Quaterniond(current.linear()).slerp(fraction,Eigen::Quaterniond(command.pose.linear())).toRotationMatrix();
      }
      raw=servo_->getNextJointState(robot_state_,command);
    }else{
      // Query the same upstream collision/singularity/limit handling while
      // holding or stopping. This target is not sent to the native smoother.
      servo_->setCommandType(moveit_servo::CommandType::JOINT_JOG);
      raw=servo_->getNextJointState(robot_state_,moveit_servo::JointJogCommand{names_,std::vector<double>(6,0.)});
      servo_->setCommandType(moveit_servo::CommandType::POSE);
    }
    status_=servo_->getStatus();
    if(status_==moveit_servo::StatusCode::INVALID)throw std::runtime_error("MoveIt Servo rejected the input");
    const bool hard=status_==moveit_servo::StatusCode::HALT_FOR_COLLISION||
                    status_==moveit_servo::StatusCode::HALT_FOR_SINGULARITY||status_==moveit_servo::StatusCode::JOINT_BOUND;
    if(hard){
      if(!future_stationary())throw std::runtime_error("MoveIt hard halt with queued motion; hardware stop required");
      return hold_interval();
    }
    if(follow){
      if(raw.positions.size()!=6||!raw.positions.allFinite()||raw.velocities.size()!=6||!raw.velocities.allFinite())throw std::runtime_error("invalid MoveIt IK target");
      input_.current_position=tail_.position;input_.current_velocity=tail_.velocity;input_.current_acceleration=tail_.acceleration;
      for(size_t i=0;i<6;++i){
        input_.target_position[i]=raw.positions[i];
        // Same target assumption as upstream MoveIt RuckigFilterPlugin.
        input_.target_velocity[i]=velocity_tracking_?raw.velocities[i]:input_.current_velocity[i]+input_.current_acceleration[i]*.008;
        input_.target_acceleration[i]=0.;
      }
      const auto result=ruckig_.update(input_,output_);
      if(result!=ruckig::Result::Working&&result!=ruckig::Result::Finished)
        throw std::runtime_error("native Ruckig rejected Servo target: "+std::to_string(static_cast<int>(result)));
      auto interval=sample_servo_interval(output_.trajectory,output_.time-.008,output_.time);
      if(interval.empty()||max_difference(interval.front().position,tail_.position)>1e-10||
         max_difference(interval.front().velocity,tail_.velocity)>1e-8||
         max_difference(interval.front().acceleration,tail_.acceleration)>1e-8)
        throw std::runtime_error("native Ruckig interval does not begin at the last commanded q/v/a");
      return interval;
    }
    if(!stopping_&&stationary(tail_))return hold_interval();
    if(!stopping_){
      if(!stop_.start(vector(tail_.position),vector(tail_.velocity),vector(tail_.acceleration),vmax_,amax_,jmax_,1.5))
        throw std::runtime_error("Servo stop unavailable: "+stop_.error());
      stop_knots_=stop_.validation_times();stopping_=true;
      RCLCPP_WARN(node_->get_logger(),"Servo input stale: native stop %.6f s after explicit 32 ms queued horizon",stop_.duration());
    }
    const double start=stop_.elapsed();
    if(!stop_.step(.008))throw std::runtime_error("Servo stop failed: "+stop_.error());
    std::vector<int64_t> ticks{0,tick_ns};
    for(double t:stop_knots_)if(t>start&&t<start+.008)ticks.push_back(std::llround((t-start)*1e9));
    std::sort(ticks.begin(),ticks.end());ticks.erase(std::unique(ticks.begin(),ticks.end()),ticks.end());
    std::vector<Point> points;
    for(auto ns:ticks){
      Point p;p.time_ns=ns;
      if(!stop_.sample_at(std::min(start+ns*1e-9,stop_.duration()),p.position,p.velocity,p.acceleration))
        throw std::runtime_error("invalid native stop knot");
      points.push_back(p);
    }
    // The terminal sample is part of this interval and is emitted before hold.
    if(stop_.finished()){stopping_=false;reset_ruckig(points.back());}
    return points;
  }
  std::vector<Point> hold_interval()const{
    auto first=tail_,last=tail_;first.time_ns=0;last.time_ns=tick_ns;return {first,last};
  }
  void validate_interval_and_stop(const std::vector<Point>& interval,const Clock::time_point& begin){
    if(interval.size()<2||interval.front().time_ns!=0||interval.back().time_ns!=tick_ns)
      throw std::runtime_error("incomplete Servo interval");
    const auto& candidate=interval.back();
    for(const auto& p:interval)for(size_t i=0;i<6;++i)
      if(!std::isfinite(p.position[i])||!std::isfinite(p.velocity[i])||!std::isfinite(p.acceleration[i])||
         std::abs(p.velocity[i])>vmax_[i]+1e-8||std::abs(p.acceleration[i])>amax_[i]+1e-8)
        throw std::runtime_error("native Servo interval exceeded limits");
    // Collision at an already stationary command is reported to the supervisor
    // without inventing a move out of collision. A moving window fails above.
    if(stationary(candidate)&&future_stationary()&&max_difference(candidate.position,tail_.position)<=1e-10)return;
    JointStopProfile stop;
    if(!stop.start(vector(candidate.position),vector(candidate.velocity),vector(candidate.acceleration),vmax_,amax_,jmax_,1.5))
      throw std::runtime_error("next Servo target has no validated stop: "+stop.error());
    for(size_t i=0;i<6;++i)if(stop.lower_position()[i]<generated_guard_.lower[i]||stop.upper_position()[i]>generated_guard_.upper[i])
      throw std::runtime_error("Servo stop envelope exceeds joint bounds");
    auto budget=[&]{if(seconds(Clock::now()-begin)>.006)throw std::runtime_error("Servo stop-envelope scene check exceeded 6 ms");};
    budget();
    // RO lock prevents a scene update midway through this discrete check. The
    // current measured scene (including attached bodies) is retained. This is
    // not continuous collision detection or a physical stopping proof.
    planning_scene_monitor::LockedPlanningSceneRO scene(scene_);
    auto probe=scene->getCurrentState();size_t checks=0;
    auto check=[&](const Joints& q,const Joints& v,const Joints& a){
      budget();if(++checks>512)throw std::runtime_error("Servo stop scene check exceeded 512 states");
      if(!set_sampled_robot_state(probe,group_,q,v,a,vmax_,amax_))
        throw std::runtime_error("Servo interval/stop sample violates joint position/dynamic bounds");
      if(scene->isStateColliding(probe,params_.move_group_name))
        throw std::runtime_error("Servo interval/stop envelope intersects scene");
      budget();
    };
    // Recheck the already queued horizon against this same scene snapshot.
    // Supervisor additionally revokes the lease on a moving scene-epoch change.
    for(const auto& p:window_)if(p.time_ns>=state_stamp_+tick_ns)check(p.position,p.velocity,p.acceleration);
    for(const auto& p:interval)check(p.position,p.velocity,p.acceleration);
    for(double time:stop.validation_times()){
      Joints q,v,a;if(!stop.sample_at(time,q,v,a))throw std::runtime_error("invalid Servo stop envelope sample");check(q,v,a);
    }
  }
  void append_interval(const std::vector<Point>& interval){
    const auto start=tail_.time_ns;
    for(size_t i=1;i<interval.size();++i){auto p=interval[i];p.time_ns+=start;
      if(p.time_ns<=window_.back().time_ns)throw std::runtime_error("non-increasing Servo knots");window_.push_back(p);}
    tail_=window_.back();
    if(tail_.time_ns!=state_stamp_+tick_ns+horizon_ns)throw std::runtime_error("Servo horizon drifted from controller grid");
    while(window_.size()>2&&window_[1].time_ns<state_stamp_+tick_ns-horizon_ns)window_.pop_front();
    if(window_.size()>512)throw std::runtime_error("Servo rolling window exceeded 512 knots");
  }
  void publish_window(){
    if(failed_||paused_||window_.empty())return;
    trajectory_msgs::msg::JointTrajectory msg;msg.joint_names=names_;
    msg.header.stamp=rclcpp::Time(window_.front().time_ns,node_->get_clock()->get_clock_type());
    for(const auto& p:window_){trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions=vector(p.position);point.velocities=vector(p.velocity);point.accelerations=vector(p.acceleration);
      point.time_from_start=rclcpp::Duration::from_nanoseconds(p.time_ns-window_.front().time_ns);msg.points.push_back(std::move(point));}
    trajectory_pub_->publish(msg);
  }
  void pause(bool requested,std_srvs::srv::SetBool::Response& response){
    response.success=false;
    if(failed_){response.message="Servo failure is latched";return;}
    if(requested==paused_){response.success=true;response.message="Requested pause state already active";return;}
    if(!command_ready()||(requested&&!future_stationary())){
      response.message="Fresh commanded and measured rest for 80 ms and an empty moving horizon are required";return;
    }
    pose_.reset();
    if(requested){paused_=true;initialized_=false;window_.clear();}
    else{paused_=false;try{initialize();}catch(const std::exception& error){fail(error.what());response.message=error.what();return;}}
    // Collision monitoring remains active through pause; no blind resume gap.
    response.success=true;response.message=requested?"Servo paused at exact commanded rest":"Servo resumed from exact commanded rest";
  }
  void publish_status(){
    moveit_msgs::msg::ServoStatus msg;
    msg.code=static_cast<int8_t>(failed_?moveit_servo::StatusCode::INVALID:status_);
    msg.message=failed_?fault_reason_:moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);status_pub_->publish(msg);
  }
  void fail(const std::string& reason){
    if(failed_)return;failed_=true;fault_reason_=reason;pose_.reset();
    std_msgs::msg::String msg;msg.data="fixed Servo: "+reason;fault_pub_->publish(msg);publish_status();
    RCLCPP_ERROR(node_->get_logger(),"%s; latched fault requires hardware lease revocation",msg.data.c_str());
  }
  rclcpp::Node::SharedPtr node_;std::shared_ptr<servo::ParamListener> listener_;servo::Params params_;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_;std::unique_ptr<moveit_servo::Servo> servo_;
  moveit::core::RobotStatePtr robot_state_;const moveit::core::JointModelGroup* group_{};
  std::vector<std::string> names_;Guard generated_guard_;Joints vmax_{},amax_{},jmax_{},command_q_{},quiet_q_{};
  ruckig::Ruckig<6> ruckig_{.008};ruckig::InputParameter<6> input_;ruckig::OutputParameter<6> output_;
  JointStopProfile stop_;std::vector<double> stop_knots_;Point tail_{};std::deque<Point> window_;
  bool velocity_tracking_{false};
  bool failed_{false},paused_{false},initialized_{false},have_state_{false},stopping_{false};
  int64_t state_stamp_{0};Clock::time_point state_receipt_{},pose_receipt_{};std::optional<Clock::time_point> quiet_since_;
  std::optional<geometry_msgs::msg::PoseStamped> pose_;moveit_servo::StatusCode status_{moveit_servo::StatusCode::NO_WARNING};
  std::string fault_reason_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<moveit_msgs::msg::ServoStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_pub_;
  rclcpp::Subscription<StateMessage>::SharedPtr state_sub_;rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Service<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;rclcpp::TimerBase::SharedPtr watchdog_;
};
} // namespace v3_moveit

namespace {
volatile std::sig_atomic_t fixed_servo_shutdown_requested=0;
extern "C" void request_fixed_servo_shutdown(int) noexcept {
  // Async-signal-safe: no ROS, locks, logging or object destruction here.
  fixed_servo_shutdown_requested=1;
}
}

int main(int argc,char** argv){
  int result=0;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<v3_moveit::FixedServoNode> adapter;
  try{
    // Upstream CollisionMonitor calls WallRate::sleep from its worker thread.
    // Its context must stay valid until Servo::~Servo has stopped/joined it.
    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT,request_fixed_servo_shutdown);
    std::signal(SIGTERM,request_fixed_servo_shutdown);
    node=std::make_shared<rclcpp::Node>("servo_node",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    adapter=std::make_unique<v3_moveit::FixedServoNode>(node);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while(!fixed_servo_shutdown_requested&&rclcpp::ok())
      executor.spin_once(std::chrono::milliseconds(1));
    executor.remove_node(node);
  }catch(const std::exception& error){
    RCLCPP_FATAL(rclcpp::get_logger("fixed_servo_node"),"%s",error.what());result=1;
  }
  // Both the normal signal path and exception path destroy the adapter before
  // invalidating the context. This cleanup is not a physical controlled stop.
  adapter.reset();node.reset();rclcpp::shutdown();return result;
}
