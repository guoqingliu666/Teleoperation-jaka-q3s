// Normal trajectories use upstream JTC; cancellation shares this exact pure
// Ruckig implementation with planning-time discrete stopping-envelope checks.
#include "v3_moveit/joint_stop_profile.hpp"

#ifndef V3_SMOOTH_PROFILE_ONLY
#include <atomic>
#include <memory>
#include <joint_trajectory_controller/joint_trajectory_controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_action/create_server.hpp>
#include <std_msgs/msg/string.hpp>

namespace v3_moveit {
class SmoothTrajectoryController : public joint_trajectory_controller::JointTrajectoryController {
 public:
  using Base=joint_trajectory_controller::JointTrajectoryController;
  using CB=controller_interface::CallbackReturn;
  using Result=controller_interface::return_type;
  using GoalHandle=rclcpp_action::ServerGoalHandle<FollowJTrajAction>;
  CB on_init()override{
    auto result=Base::on_init();if(result!=CB::SUCCESS)return result;
    auto_declare<std::vector<double>>("smooth_stop.max_velocity",std::vector<double>(6,.17453292519943295));
    auto_declare<std::vector<double>>("smooth_stop.max_acceleration",std::vector<double>(6,.6981317007977318));
    auto_declare<std::vector<double>>("smooth_stop.max_jerk",std::vector<double>(6,2.792526803190927));
    auto_declare<double>("smooth_stop.max_duration",1.5);
    return CB::SUCCESS;
  }
  CB on_configure(const rclcpp_lifecycle::State& state)override{
    auto result=Base::on_configure(state);if(result!=CB::SUCCESS)return result;
    if(get_update_rate()!=125||params_.joints.size()!=6||params_.command_interfaces!=std::vector<std::string>{"position"}||
       !params_.interpolate_from_desired_state||!has_velocity_state_interface_||use_closed_loop_pid_adapter_||
       params_.speed_scaling.initial_scaling_factor!=1.||!params_.speed_scaling.state_interface.empty()||
       !params_.speed_scaling.command_interface.empty()){
      RCLCPP_ERROR(get_node()->get_logger(),"Smooth stop requires 125 Hz, six position-command joints, commanded-state interpolation and speed scaling 1");
      return CB::ERROR;
    }
    if(!load_limits("smooth_stop.max_velocity",vmax_)||!load_limits("smooth_stop.max_acceleration",amax_)||
       !load_limits("smooth_stop.max_jerk",jmax_))return CB::ERROR;
    max_duration_=get_node()->get_parameter("smooth_stop.max_duration").as_double();
    if(!JointStopProfile::limits_match_contract(vmax_,amax_,jmax_,max_duration_)){
      RCLCPP_ERROR(get_node()->get_logger(),"Smooth stop limits must match the Guard/planning cancellation contract, duration 1.5 s");
      return CB::ERROR;
    }
    fault_publisher_=get_node()->create_publisher<std_msgs::msg::String>("/v3/controller_fault",rclcpp::QoS(1).reliable().transient_local());
    // Base callbacks are not virtual in JTC 4.40.1. Rebind only the input gates
    // and cancel callback, retaining upstream validation and goal acceptance.
    action_server_.reset();
    action_server_=rclcpp_action::create_server<FollowJTrajAction>(
      get_node()->get_node_base_interface(),get_node()->get_node_clock_interface(),
      get_node()->get_node_logging_interface(),get_node()->get_node_waitables_interface(),
      get_node()->get_name()+std::string("/follow_joint_trajectory"),
      [this](const rclcpp_action::GoalUUID& id,std::shared_ptr<const FollowJTrajAction::Goal> goal){
        release_completed_stop();
        if(stop_gate_.load())return rclcpp_action::GoalResponse::REJECT;
        return Base::goal_received_callback(id,goal);
      },
      [this](std::shared_ptr<GoalHandle> goal){return request_cancel(goal);},
      [this](std::shared_ptr<GoalHandle> goal){
        if(stop_gate_.load()){
          auto result=std::make_shared<FollowJTrajAction::Result>();
          result->error_code=FollowJTrajAction::Result::INVALID_GOAL;
          result->error_string="Goal arrived during a smooth stop";
          if(!goal->is_executing())goal->execute();goal->abort(result);return;
        }
        Base::goal_accepted_callback(goal);
      });
    joint_command_subscriber_.reset();
    joint_command_subscriber_=get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory",rclcpp::SystemDefaultsQoS(),
      [this](std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg){
        release_completed_stop();
        if(!stop_gate_.load())Base::topic_callback(msg);
      });
    return CB::SUCCESS;
  }
  CB on_activate(const rclcpp_lifecycle::State& state)override{
    stopping_=false;failed_=false;stop_gate_=false;cancel_requested_=false;completion_pending_=false;stopping_goal_.reset();
    command_tick_initialized_=false;
    completion_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    const auto result=Base::on_activate(state);
    if(result!=CB::SUCCESS)return result;
    // Upstream 4.40.1 rejects command-as-state when velocity feedback exists
    // without a velocity command interface. It then seeds HOLD from a newer
    // encoder sample, discontinuous with JakaSystem's activation HOLD. Retain
    // the existing position command; measured state stays measured state.
    JointTrajectoryPoint seed;
    seed.positions.resize(6);seed.velocities.assign(6,0.);seed.accelerations.assign(6,0.);
    size_t unset=0;
    if(joint_command_interface_[0].size()!=6)return CB::ERROR;
    for(size_t i=0;i<6;++i){
      const auto value=joint_command_interface_[0][i].get().get_optional();
      if(!value){RCLCPP_ERROR(get_node()->get_logger(),"Activation command unavailable");return CB::ERROR;}
      seed.positions[i]=*value;
      if(std::isnan(*value))++unset;
      else if(!std::isfinite(*value)){RCLCPP_ERROR(get_node()->get_logger(),"Nonfinite activation command");return CB::ERROR;}
    }
    // GenericSystem can begin with every command unset. A real JakaSystem
    // always exports its finite, validated activation HOLD command.
    if(unset==6)seed.positions=state_current_.positions;
    else if(unset){RCLCPP_ERROR(get_node()->get_logger(),"Partially unset activation command");return CB::ERROR;}
    if(state_current_.positions.size()!=6||state_current_.velocities.size()!=6)return CB::ERROR;
    for(size_t i=0;i<6;++i){
      if(!std::isfinite(seed.positions[i])||!std::isfinite(state_current_.positions[i])||
         !std::isfinite(state_current_.velocities[i])||std::abs(state_current_.velocities[i])>.005||
         std::abs(seed.positions[i]-state_current_.positions[i])>.001){
        RCLCPP_ERROR(get_node()->get_logger(),"Activation requires stationary feedback near the existing HOLD command");
        return CB::ERROR;
      }
    }
    last_commanded_state_=seed;command_next_=seed;state_desired_=seed;
    auto hold=std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    hold->joint_names=params_.joints;hold->points.push_back(seed);
    add_new_trajectory_msg(hold);rt_is_holding_=true;
    return CB::SUCCESS;
  }
  CB on_deactivate(const rclcpp_lifecycle::State& state)override{
    stop_gate_=true;stopping_=false;cancel_requested_=false;completion_pending_=false;
    // Hard deactivation is never delayed to finish a smooth cancellation.
    return Base::on_deactivate(state);
  }
  Result update(const rclcpp::Time& time,const rclcpp::Duration& period)override{
    if(failed_)return Result::ERROR;
    const double actual_dt=period.seconds();
    if(!std::isfinite(actual_dt)||actual_dt<.004||actual_dt>.012)
      return fail("actual controller cycle outside 4..12 ms; command ticks are never skipped or replayed");
    // JAKA EDG step_num=1 requests an 8 ms target interval. Use that same
    // interval for every normal/cancel target and first-sample transition.
    // Actual scheduler/send deadlines are separate checks; this target clock
    // does not establish the cabinet's physical interpolation or network delay.
    if(!command_tick_initialized_){command_tick_=time;command_tick_initialized_=true;}
    else command_tick_+=rclcpp::Duration(0,8000000);
    const auto target_period=rclcpp::Duration(0,8000000);
    if(scaling_factor_.load()!=1.)return fail("speed scaling changed; commanded v/a no longer represent physical-time derivatives");
    if(cancel_requested_.exchange(false)){
      const auto requested=*cancel_target_.readFromRT();
      const auto active=*rt_active_goal_.readFromRT();
      if(requested&&active==requested){
        stopping_goal_=requested;
        if(!profile_.start(last_commanded_state_.positions,last_commanded_state_.velocities,
            last_commanded_state_.accelerations,vmax_,amax_,jmax_,max_duration_))return fail(profile_.error());
        stopping_=true;
        double max_distance=0.;for(double d:profile_.excursion())max_distance=std::max(max_distance,std::abs(d));
        RCLCPP_WARN(get_node()->get_logger(),
          "Ruckig cancellation: duration=%.6f s, max joint excursion=%.6f rad; additional stopping distance requires scene/physical validation",
          profile_.duration(),max_distance);
      }else stop_gate_=false; // The target goal already completed or was replaced.
    }
    if(!stopping_){
      const auto result=Base::update(command_tick_,target_period);
      if(result!=Result::OK)return fail("upstream JTC update failed");
      // Upstream samples command_next at tick + one period but stores tick.
      // Use the sample timestamp for the next trajectory's point-before bridge.
      // Equality confirms upstream committed the sampled command, including holds.
      if(has_active_trajectory()&&last_commanded_state_.positions.size()==6&&
         last_commanded_time_==command_tick_&&last_commanded_state_==command_next_)
        last_commanded_time_=command_tick_+target_period;
      return result;
    }
    if(!profile_.step(.008))return fail(profile_.error());
    read_state_from_state_interfaces(state_current_);
    for(size_t i=0;i<6;++i){
      if(!std::isfinite(state_current_.positions[i])||!std::isfinite(state_current_.velocities[i]))
        return fail("nonfinite measured state during cancellation");
      if(std::abs(profile_.velocity()[i])>vmax_[i]+1e-8||std::abs(profile_.acceleration()[i])>amax_[i]+1e-8)
        return fail("Ruckig cancellation exceeded configured bounds");
    }
    // Same controller update that owns normal JTC writes: no asynchronous state
    // snapshot and no intermediate base constant-deceleration cancel trajectory.
    for(size_t i=0;i<num_cmd_joints_;++i){
      if(!joint_command_interface_[0][i].get().set_value(profile_.position()[map_cmd_to_joints_[i]]))
        return fail("position command interface rejected cancellation sample");
    }
    last_commanded_state_.positions.assign(profile_.position().begin(),profile_.position().end());
    last_commanded_state_.velocities.assign(profile_.velocity().begin(),profile_.velocity().end());
    last_commanded_state_.accelerations.assign(profile_.acceleration().begin(),profile_.acceleration().end());
    last_commanded_state_.time_from_start=rclcpp::Duration::from_seconds(profile_.elapsed());
    last_commanded_time_=command_tick_+target_period;command_next_=last_commanded_state_;state_desired_=last_commanded_state_;
    for(size_t i=0;i<6;++i)compute_error_for_joint(state_error_,i,state_current_,state_desired_);
    // This feedback header represents the target sequence clock.
    publish_state(command_tick_,state_desired_,state_current_,state_error_);
    if(profile_.finished()){
      // The terminal zero-v/a sample above is written before setting CANCELED.
      hold_position_msg_ptr_->points[0]=last_commanded_state_;
      hold_position_msg_ptr_->points[0].time_from_start=rclcpp::Duration(0,0);
      hold_position_msg_ptr_->header.stamp=builtin_interfaces::msg::Time();
      current_trajectory_->update(hold_position_msg_ptr_);
      new_trajectory_msg_.reset();new_trajectory_msg_.initRT(hold_position_msg_ptr_);
      rt_is_holding_=true;rt_has_pending_goal_=false;
      auto result=std::make_shared<FollowJTrajAction::Result>();
      result->error_code=FollowJTrajAction::Result::SUCCESSFUL;
      result->error_string="Ruckig cancellation terminal command emitted; physical rest is checked separately";
      stopping_goal_->setCanceled(result);
      completion_goal_.writeFromNonRT(stopping_goal_);
      completion_pending_=true;
      rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
      stopping_goal_.reset();stopping_=false;
      // Keep the gate closed until a non-RT callback has delivered the old
      // result. Base goal acceptance replaces its result timer.
    }
    return Result::OK;
  }
 private:
  void release_completed_stop(){
    if(!completion_pending_.load())return;
    const auto completed=*completion_goal_.readFromNonRT();
    if(!completed)return;
    completed->runNonRealtime();
    if(!completed->gh_->is_active()){
      completion_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
      completion_pending_=false;stop_gate_=false;
    }
  }
  bool load_limits(const std::string& name,JointStopProfile::Joints& values){
    const auto source=get_node()->get_parameter(name).as_double_array();
    if(source.size()!=6){RCLCPP_ERROR(get_node()->get_logger(),"%s requires six values",name.c_str());return false;}
    for(size_t i=0;i<6;++i){if(!std::isfinite(source[i])||source[i]<=0)return false;values[i]=source[i];}
    return true;
  }
  rclcpp_action::CancelResponse request_cancel(const std::shared_ptr<GoalHandle>& goal){
    const auto active=*rt_active_goal_.readFromNonRT();
    // A goal already completed in update may still await its non-RT result.
    // Accepting its late cancel would mark it CANCELING and suppress success.
    if(!active||active->gh_!=goal)return rclcpp_action::CancelResponse::REJECT;
    if(!stop_gate_.exchange(true)){
      cancel_target_.writeFromNonRT(active);
      cancel_requested_=true;
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  Result fail(const std::string& reason){
    failed_=true;stop_gate_=true;
    RCLCPP_ERROR(get_node()->get_logger(),"Smooth cancellation failed: %s; supervisor must revoke motion lease",reason.c_str());
    std_msgs::msg::String msg;msg.data=reason;fault_publisher_->publish(msg);
    return Result::ERROR;
  }
  JointStopProfile profile_;JointStopProfile::Joints vmax_{},amax_{},jmax_{};double max_duration_{1.5};
  std::atomic<bool> stop_gate_{false},cancel_requested_{false},completion_pending_{false};bool stopping_{false},failed_{false};
  realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr> cancel_target_,completion_goal_;
  RealtimeGoalHandlePtr stopping_goal_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_publisher_;
  rclcpp::Time command_tick_{0,0,RCL_ROS_TIME};bool command_tick_initialized_{false};
};
}
PLUGINLIB_EXPORT_CLASS(v3_moveit::SmoothTrajectoryController,controller_interface::ControllerInterface)
#endif
