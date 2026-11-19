// Pure native regression: creates no ROS node, executor, publisher, or hardware.
#include "v3_moveit/servo_interval.hpp"
#include "v3_moveit/guard.hpp"
#include <joint_trajectory_controller/trajectory.hpp>
#include <iostream>
#include <iomanip>
#include <limits>
#include <random>
#include <string>

using V=std::array<double,6>;
using Point=trajectory_msgs::msg::JointTrajectoryPoint;
using Trajectory=ruckig::Trajectory<6>;
constexpr double tick=.008;
constexpr int64_t tick_ns=8000000;
std::size_t interval_count=0,interpolations=0,guard_checks=0,max_knots=0,brake_cases=0;
double max_p_error=0.,max_v_error=0.,max_a_error=0.;
int64_t worst_a_segment_ns=0;double worst_a_begin=0.;
std::size_t aligned_grid_samples=0;double grid_p_error=0.,grid_v_error=0.,grid_a_error=0.;
void check_grid(const Point& actual,const V&p,const V&v,const V&a) {
  for(size_t j=0;j<6;++j){grid_p_error=std::max(grid_p_error,std::abs(actual.positions[j]-p[j]));grid_v_error=std::max(grid_v_error,std::abs(actual.velocities[j]-v[j]));grid_a_error=std::max(grid_a_error,std::abs(actual.accelerations[j]-a[j]));}
  ++aligned_grid_samples;
}
void require(bool value,const std::string& why) {if(!value)throw std::runtime_error(why);}
Point point(const v3_moveit::ServoIntervalPoint& in) {
  Point p;p.positions.assign(in.position.begin(),in.position.end());
  p.velocities.assign(in.velocity.begin(),in.velocity.end());p.accelerations.assign(in.acceleration.begin(),in.acceleration.end());
  return p;
}
ruckig::InputParameter<6> input(double span,bool offset=true) {
  ruckig::InputParameter<6> in;
  for(size_t j=0;j<6;++j) {
    in.current_position[j]=offset?(-.6+.53*j):0.;in.target_position[j]=in.current_position[j]+span*(j%2?-1.:1.)*(1.+j*.13);
    in.current_velocity[j]=in.current_acceleration[j]=in.target_velocity[j]=in.target_acceleration[j]=0.;
    in.max_velocity[j]=.1745329252;in.max_acceleration[j]=.6981317008;in.max_jerk[j]=2.7925268032;
  }
  return in;
}
void check_interval(const Trajectory& trajectory,double begin,double end) {
  const auto knots=v3_moveit::sample_servo_interval(trajectory,begin,end);
  ++interval_count;max_knots=std::max(max_knots,knots.size());
  require(knots.front().time_ns==0&&knots.back().time_ns==tick_ns,"missing interval endpoints");
  joint_trajectory_controller::Trajectory jtc;
  for(size_t k=1;k<knots.size();++k) {
    const auto &a=knots[k-1],&b=knots[k];require(b.time_ns>a.time_ns,"nonmonotonic knots");
    const auto pa=point(a),pb=point(b);
    for(int sample=0;sample<=8;++sample) {
      const int64_t ns=a.time_ns+(b.time_ns-a.time_ns)*sample/8;
      Point actual;jtc.interpolate_between_points(rclcpp::Time(a.time_ns,RCL_ROS_TIME),pa,rclcpp::Time(b.time_ns,RCL_ROS_TIME),pb,rclcpp::Time(ns,RCL_ROS_TIME),actual);
      V p,v,acc;trajectory.at_time(ns==tick_ns?end:begin+ns*1e-9,p,v,acc);
      for(size_t j=0;j<6;++j) {
        require(std::isfinite(actual.positions[j])&&std::isfinite(actual.velocities[j])&&std::isfinite(actual.accelerations[j]),"nonfinite native JTC output");
        max_p_error=std::max(max_p_error,std::abs(p[j]-actual.positions[j]));
        max_v_error=std::max(max_v_error,std::abs(v[j]-actual.velocities[j]));
        if(std::abs(acc[j]-actual.accelerations[j])>max_a_error) {max_a_error=std::abs(acc[j]-actual.accelerations[j]);worst_a_segment_ns=b.time_ns-a.time_ns;worst_a_begin=begin;}
      }
      ++interpolations;
    }
  }
}
void check_rest_trajectory(const ruckig::InputParameter<6>& in) {
  ruckig::Ruckig<6> generator(tick);Trajectory tr;
  auto rc=generator.calculate(in,tr);require(rc==ruckig::Result::Working||rc==ruckig::Result::Finished,"Ruckig calculation failed");
  auto msg=std::make_shared<trajectory_msgs::msg::JointTrajectory>();msg->joint_names={"a","b","c","d","e","f"};
  const int64_t last=static_cast<int64_t>(std::ceil(tr.get_duration()/tick))+2;
  for(int64_t k=0;k<last;++k) {
    check_interval(tr,k*tick,(k+1)*tick);
    auto knots=v3_moveit::sample_servo_interval(tr,k*tick,(k+1)*tick);
    for(size_t i=(k?1:0);i<knots.size();++i) {
      auto p=point(knots[i]);p.time_from_start=rclcpp::Duration::from_nanoseconds(k*tick_ns+knots[i].time_ns);msg->points.push_back(p);
    }
  }
  // Exercise four sample-grid phases. A stationary prelude makes Guard's reset
  // history physically continuous for each phase, including the first step.
  for(int64_t phase:{0LL,1000000LL,4000000LL,7999999LL}) {
    auto shifted=std::make_shared<trajectory_msgs::msg::JointTrajectory>(*msg);
    for(auto& p:shifted->points)p.time_from_start=rclcpp::Duration(p.time_from_start)+rclcpp::Duration::from_nanoseconds(32000000+phase);
    shifted->header.stamp=rclcpp::Time(1000000000LL,RCL_ROS_TIME);
    joint_trajectory_controller::Trajectory jtc(shifted);
    Point start=msg->points.front();jtc.set_point_before_trajectory_msg(rclcpp::Time(1000000000LL,RCL_ROS_TIME),start);
    v3_moveit::Guard guard;guard.lower.fill(-10);guard.upper.fill(10);guard.reset(in.current_position);
    V previous=in.current_position;
    for(int64_t ns=tick_ns;ns<(last+7)*tick_ns;ns+=tick_ns) {
      Point out;joint_trajectory_controller::TrajectoryPointConstIter a,b;
      require(jtc.sample(rclcpp::Time(1000000000LL+ns,RCL_ROS_TIME),joint_trajectory_controller::interpolation_methods::InterpolationMethod::VARIABLE_DEGREE_SPLINE,out,a,b),"native JTC sample failed");
      if(phase==0) {
        V p,v,a;if(ns<32000000){p=in.current_position;v.fill(0);a.fill(0);}else tr.at_time((ns-32000000)*1e-9,p,v,a);
        check_grid(out,p,v,a);
      }
      V q;std::copy(out.positions.begin(),out.positions.end(),q.begin());guard.check(q,previous,tick);previous=q;++guard_checks;
    }
  }
}
// Native velocity interface coverage for the fake-only continuous follower.
void check_velocity_tracking() {
  auto in=input(0.,false);in.control_interface=ruckig::ControlInterface::Velocity;
  in.synchronization=ruckig::Synchronization::Time;
  ruckig::Ruckig<6> generator(tick);ruckig::OutputParameter<6> output;
  v3_moveit::Guard guard;guard.lower.fill(-10);guard.upper.fill(10);guard.reset(in.current_position);
  V previous=in.current_position;
  for(int k=0;k<1500;++k){
    for(size_t j=0;j<6;++j)in.target_velocity[j]=k<1000?.08*std::sin(k*.008*2.+j*.5):0.;
    const auto rc=generator.update(in,output);
    require(rc==ruckig::Result::Working||rc==ruckig::Result::Finished,"velocity Ruckig failed");
    const auto knots=v3_moveit::sample_servo_interval(output.trajectory,output.time-tick,output.time);
    for(size_t j=0;j<6;++j){
      require(std::abs(knots.front().position[j]-in.current_position[j])<1e-10,"velocity anchor position");
      require(std::abs(knots.front().velocity[j]-in.current_velocity[j])<1e-8,"velocity anchor velocity");
      require(std::abs(knots.front().acceleration[j]-in.current_acceleration[j])<1e-8,"velocity anchor acceleration");
    }
    check_interval(output.trajectory,output.time-tick,output.time);
    guard.check(output.new_position,previous,tick);previous=output.new_position;++guard_checks;
    output.pass_to_input(in);
  }
  for(size_t j=0;j<6;++j)require(std::abs(output.new_velocity[j])<1e-10&&std::abs(output.new_acceleration[j])<1e-10,"velocity release did not settle");
}
int main() {
 try {
  check_velocity_tracking();
  for(double span:{1e-14,1e-12,1e-9,1e-7,1e-5,.001,.01,.08}) {
    auto in=input(span);check_rest_trajectory(in);
    in.synchronization=ruckig::Synchronization::None;check_rest_trajectory(in);
  }
  // Online retargeting exercises new_calculation/time resets and a moving C2 anchor.
  ruckig::Ruckig<6> generator(tick);ruckig::OutputParameter<6> out;auto in=input(.03);
  auto online_msg=std::make_shared<trajectory_msgs::msg::JointTrajectory>();online_msg->joint_names={"a","b","c","d","e","f"};
  const auto initial=in.current_position;
  for(int n=0;n<300;++n) {
    if(n%37==0)for(size_t j=0;j<6;++j)in.target_position[j]=(-.6+.53*j)+.035*std::sin(n*.013+j);
    const auto rc=generator.update(in,out);require(rc==ruckig::Result::Working||rc==ruckig::Result::Finished,"Ruckig online update failed");
    check_interval(out.trajectory,out.time-tick,out.time);
    const auto knots=v3_moveit::sample_servo_interval(out.trajectory,out.time-tick,out.time);
    for(size_t j=0;j<6;++j)require(knots.back().position[j]==out.new_position[j]&&knots.back().velocity[j]==out.new_velocity[j]&&knots.back().acceleration[j]==out.new_acceleration[j],"changed Ruckig final state");
    for(size_t k=(n?1:0);k<knots.size();++k) {
      auto p=point(knots[k]);p.time_from_start=rclcpp::Duration::from_nanoseconds(n*tick_ns+knots[k].time_ns);online_msg->points.push_back(p);
    }
    for(size_t j=0;j<6;++j)require(std::abs(knots.front().position[j]-in.current_position[j])<1e-12&&std::abs(knots.front().velocity[j]-in.current_velocity[j])<1e-12&&std::abs(knots.front().acceleration[j]-in.current_acceleration[j])<1e-12,"online C2 anchor changed");
    out.pass_to_input(in);
  }
  for(int64_t phase:{0LL,1000000LL,4000000LL,7999999LL}) {
    auto msg=std::make_shared<trajectory_msgs::msg::JointTrajectory>(*online_msg);
    for(auto& p:msg->points)p.time_from_start=rclcpp::Duration(p.time_from_start)+rclcpp::Duration::from_nanoseconds(32000000+phase);
    msg->header.stamp=rclcpp::Time(1000000000LL,RCL_ROS_TIME);
    joint_trajectory_controller::Trajectory jtc(msg);jtc.set_point_before_trajectory_msg(rclcpp::Time(1000000000LL,RCL_ROS_TIME),online_msg->points.front());
    v3_moveit::Guard guard;guard.lower.fill(-10);guard.upper.fill(10);guard.reset(initial);V previous=initial;
    for(int64_t ns=tick_ns;ns<300*tick_ns+32000000+phase;ns+=tick_ns) {
      Point sample;joint_trajectory_controller::TrajectoryPointConstIter a,b;
      require(jtc.sample(rclcpp::Time(1000000000LL+ns,RCL_ROS_TIME),joint_trajectory_controller::interpolation_methods::InterpolationMethod::VARIABLE_DEGREE_SPLINE,sample,a,b),"online native JTC sample failed");
      if(phase==0) {
        V p,v,a;if(ns<32000000){p=initial;v.fill(0);a.fill(0);}else {
          const auto at=ns-32000000;auto it=std::find_if(online_msg->points.begin(),online_msg->points.end(),[&](const auto& point){return rclcpp::Duration(point.time_from_start).nanoseconds()==at;});
          require(it!=online_msg->points.end(),"missing online grid endpoint");
          std::copy(it->positions.begin(),it->positions.end(),p.begin());std::copy(it->velocities.begin(),it->velocities.end(),v.begin());std::copy(it->accelerations.begin(),it->accelerations.end(),a.begin());
        }
        check_grid(sample,p,v,a);
      }
      V q;std::copy(sample.positions.begin(),sample.positions.end(),q.begin());guard.check(q,previous,tick);previous=q;++guard_checks;
    }
  }
  // Pre-braking is represented, not silently discarded. These deliberately
  // out-of-bound starts are not passed to Guard as valid hardware starts.
  for(double velocity:{.18,-.18}) {
    auto in=input(.03);in.current_velocity.fill(velocity);in.current_acceleration.fill(velocity>0?.75:-.75);
    Trajectory tr;ruckig::Ruckig<6> gen(tick);auto rc=gen.calculate(in,tr);require(rc==ruckig::Result::Working||rc==ruckig::Result::Finished,"pre-brake generation failed");
    const auto profiles=tr.get_profiles();bool brake=false;for(const auto& p:profiles.front())brake|=p.brake.duration>0.;require(brake,"pre-brake fixture lacks brake");
    for(int n=0;n<20;++n)check_interval(tr,n*tick,(n+1)*tick);
    ++brake_cases;
  }
  // Invalid API intervals must fail closed.
  Trajectory tr;ruckig::Ruckig<6> gen(tick);auto in2=input(.01);gen.calculate(in2,tr);
  for(const auto range:std::vector<std::array<double,2>>{{-.008,0},{0,.004},{0,.012},{0,std::numeric_limits<double>::quiet_NaN()}}) {
    bool rejected=false;try{v3_moveit::sample_servo_interval(tr,range[0],range[1]);}catch(const std::runtime_error&){rejected=true;}require(rejected,"invalid interval accepted");
  }
  require(max_p_error<1e-12,"JTC curve position error exceeded 1e-12 rad");
  require(grid_p_error<1e-12,"aligned JTC position error exceeded 1e-12 rad");
  require(grid_v_error<1e-12,"aligned JTC velocity error exceeded 1e-12 rad/s");
  require(grid_a_error<1e-10,"aligned JTC acceleration error exceeded 1e-10 rad/s2");
  // Keep the adversarial counterexample visible: 386ns subsegments at absolute
  // q ~ 2rad suffer JTC's coefficient cancellation. This is not a continuous-
  // curve accuracy acceptance. Production must sample the anchored 8ms grid;
  // unexpected phase changes/bridges still require their own acceptance checks.
  require(max_a_error>1e-3,"known off-grid JTC cancellation fixture was not exercised");
  std::cout<<std::setprecision(17)<<"{\"passed\":true,\"fixed_grid_contract_passed\":true,\"continuous_curve_precision\":{\"passed\":false,\"known_limitation\":true},\"pure_native_no_ros_init\":true,\"intervals\":"<<interval_count<<",\"native_jtc_interpolations\":"<<interpolations<<",\"final_guard_checks\":"<<guard_checks<<",\"max_knots_per_interval\":"<<max_knots<<",\"prebrake_cases\":"<<brake_cases<<",\"max_position_error\":"<<max_p_error<<",\"max_velocity_error\":"<<max_v_error<<",\"max_acceleration_error\":"<<max_a_error<<",\"aligned_grid_samples\":"<<aligned_grid_samples<<",\"aligned_grid_position_error\":"<<grid_p_error<<",\"aligned_grid_velocity_error\":"<<grid_v_error<<",\"aligned_grid_acceleration_error\":"<<grid_a_error<<",\"continuous_curve_accuracy_claim\":false,\"off_grid_acceleration_error_threshold_1e_minus_3_exceeded\":true,\"worst_off_grid_segment_ns\":"<<worst_a_segment_ns<<"}\n";
 }catch(const std::exception&e){std::cerr<<std::setprecision(17)<<e.what()<<"; max errors p="<<max_p_error<<" v="<<max_v_error<<" a="<<max_a_error<<" worst segment ns="<<worst_a_segment_ns<<" interval begin="<<worst_a_begin<<"\n";return 1;}
}
