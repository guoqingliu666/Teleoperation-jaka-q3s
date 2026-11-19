#include "v3_moveit/sampled_robot_state.hpp"
#include "v3_moveit/path_retime.hpp"
#include "v3_moveit/joint_stop_profile.hpp"
#include <moveit/kinematic_constraints/kinematic_constraint.hpp>
#include <moveit/planning_interface/planning_response_adapter.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <set>

namespace v3_moveit {
class PathRuckigRetiming : public planning_interface::PlanningResponseAdapter {
public:
  std::string getDescription() const override {return "Path-preserving sampled Ruckig retiming";}
  void adapt(const planning_scene::PlanningSceneConstPtr& scene,
             const planning_interface::MotionPlanRequest& request,
             planning_interface::MotionPlanResponse& response) const override {
    if(!response || !response.trajectory)return;
    try {
      // Covers retiming and the sampled stopping-envelope checks together.
      const auto validation_begin=std::chrono::steady_clock::now();
      auto budget=[&] {
        if(std::chrono::duration<double>(std::chrono::steady_clock::now()-validation_begin).count()>2.)
          throw std::runtime_error("retiming/stop validation exceeded 2 s budget");
      };
      if(!scene)throw std::runtime_error("missing planning scene for cancellation validation");
      const auto& old=*response.trajectory;const auto* group=old.getGroup();
      if(!group || old.getWayPointCount()==0)throw std::runtime_error("missing geometric path/group");
      const auto& names=group->getVariableNames();
      const auto& start=request.start_state.joint_state;
      if(start.name.size()!=start.velocity.size())throw std::runtime_error("explicit stationary start velocities required");
      std::set<std::string> seen;
      for(std::size_t i=0;i<start.name.size();++i) {
        if(!seen.insert(start.name[i]).second || !std::isfinite(start.velocity[i]) || std::abs(start.velocity[i])>1e-9)
          throw std::runtime_error("nonstationary or ambiguous planning start");
      }
      for(const auto& name:names)if(!seen.count(name))throw std::runtime_error("missing planning joint start velocity");
      auto scale=[](double value) {if(!std::isfinite(value) || value<=0. || value>1.)throw std::runtime_error("invalid planning scaling");return value;};
      const double velocity_scale=scale(request.max_velocity_scaling_factor);
      const double acceleration_scale=scale(request.max_acceleration_scaling_factor);
      PathLimits limits;
      auto bound=[](bool bounded,double lo,double hi) {
        if(!bounded || !std::isfinite(lo) || !std::isfinite(hi) || lo>=0. || hi<=0.)throw std::runtime_error("missing finite positive model dynamic bounds");
        return std::min(-lo,hi);
      };
      for(const auto& name:names) {
        const auto& b=old.getRobotModel()->getVariableBounds(name);
        limits.velocity.push_back(bound(b.velocity_bounded_,b.min_velocity_,b.max_velocity_)*velocity_scale);
        limits.acceleration.push_back(bound(b.acceleration_bounded_,b.min_acceleration_,b.max_acceleration_)*acceleration_scale);
        limits.jerk.push_back(bound(b.jerk_bounded_,b.min_jerk_,b.max_jerk_));
      }
      std::vector<PathVector> path;
      for(std::size_t i=0;i<old.getWayPointCount();++i) {
        PathVector q;old.getWayPoint(i).copyJointGroupPositions(group,q);path.push_back(std::move(q));
      }
      const auto retimed=retime_path(path,limits);
      budget();
      if(names.size()!=6)throw std::runtime_error("cancellation validation requires six joints");
      for(size_t i=0;i<6;++i)if(names[i]!="joint_"+std::to_string(i+1))
        throw std::runtime_error("cancellation joint order differs from controller");
      kinematic_constraints::KinematicConstraintSet constraints(old.getRobotModel());
      if(!constraints.add(request.path_constraints,scene->getTransforms()))
        throw std::runtime_error("could not configure cancellation path constraints");
      const Guard guard;
      JointStopProfile::Joints stop_v,stop_a,stop_j;
      stop_v.fill(guard.max_velocity);stop_a.fill(guard.max_acceleration);stop_j.fill(guard.max_jerk);
      auto probe=old.getFirstWayPoint();
      std::size_t collision_checks=0,stop_starts=0;
      std::int64_t next_tick=0;
      const auto final_tick=static_cast<std::int64_t>(std::llround(retimed.samples.back().time*1e9));
      for(const auto& point:retimed.samples) {
        const auto stamp=static_cast<std::int64_t>(std::llround(point.time*1e9));
        if(stamp%8000000!=0 && stamp!=final_tick)continue;
        if(stamp!=next_tick)throw std::runtime_error("retimed path omitted a cancellation command tick");
        next_tick+=8000000;budget();
        JointStopProfile stop;
        if(!stop.start(point.position,point.velocity,point.acceleration,stop_v,stop_a,stop_j,
                       JointStopProfile::validated_max_duration))
          throw std::runtime_error("cancellation profile unavailable: "+stop.error());
        ++stop_starts;
        for(size_t i=0;i<6;++i) {
          const auto& bound=old.getRobotModel()->getVariableBounds(names[i]);
          if(bound.position_bounded_ &&
             (!std::isfinite(bound.min_position_)||!std::isfinite(bound.max_position_)||
              stop.lower_position()[i]<bound.min_position_||stop.upper_position()[i]>bound.max_position_))
            throw std::runtime_error("cancellation position envelope exceeds joint bounds");
        }
        // This is a DISCRETE model check, not continuous collision detection or
        // a claim about physical stopping distance. At the fixed Guard v limit,
        // 8 ms spans at most 0.001397 rad per joint (< 0.002 rad).
        // The schedule additionally includes exact terminal, profile switches
        // and position extrema; the actual controller uses the same stop class.
        for(double time:stop.validation_times()) {
          budget();
          if(collision_checks>=30000)throw std::runtime_error("cancellation validation exceeded 30000 state checks");
          JointStopProfile::Joints q,v,a;
          if(!stop.sample_at(time,q,v,a))throw std::runtime_error("invalid cancellation validation sample");
          const bool sample_valid=set_sampled_robot_state(probe,group,q,v,a,stop_v,stop_a);
          ++collision_checks;
          if(!sample_valid||!scene->isStateValid(probe,constraints,group->getName()))
            throw std::runtime_error("cancellation state violates bounds/collision/path constraints at path time "+
                                     std::to_string(point.time)+", stop time "+std::to_string(time));
          budget();
        }
      }
      if(next_tick!=final_tick+8000000)throw std::runtime_error("retimed endpoint is not on the command grid");
      auto output=std::make_shared<robot_trajectory::RobotTrajectory>(old.getRobotModel(),group->getName());
      auto state=old.getFirstWayPoint();double previous_time=0.;
      for(const auto& point:retimed.samples) {
        state.setJointGroupPositions(group,point.position);
        state.setJointGroupVelocities(group,point.velocity);
        state.setJointGroupAccelerations(group,point.acceleration);
        output->addSuffixWayPoint(state,point.time-previous_time);previous_time=point.time;
      }
      budget();
      response.trajectory=std::move(output);
      RCLCPP_INFO(rclcpp::get_logger("v3_path_retiming"),
                  "Discrete cancellation validation: %zu command starts / %zu scene states (not continuous or physical clearance)",
                  stop_starts,collision_checks);
      RCLCPP_INFO(rclcpp::get_logger("v3_path_retiming"),"Retimed %zu geometric points to %zu stationary vertices / %zu curve samples",
                  path.size(),retimed.vertices.size(),retimed.samples.size());
    }catch(const std::exception& error) {
      RCLCPP_ERROR(rclcpp::get_logger("v3_path_retiming"),"Rejected path retiming: %s",error.what());
      response.error_code=moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      response.trajectory.reset();
    }
  }
};
}  // namespace v3_moveit
PLUGINLIB_EXPORT_CLASS(v3_moveit::PathRuckigRetiming,planning_interface::PlanningResponseAdapter)
