#include "v3_moveit/path_retime.hpp"
#include "v3_moveit/guard.hpp"
#include <joint_trajectory_controller/trajectory.hpp>
#include <iostream>
#include <limits>

using namespace v3_moveit;
namespace {
int assertions=0;
void require(bool value,const char* reason) {++assertions;if(!value)throw std::runtime_error(reason);}
PathLimits limits() {return {PathVector(6,.1745329252),PathVector(6,.6981317008),PathVector(6,2.7925268032)};}
PathVector q(double a,double b=0.) {return {a,b,0.,0.,0.,0.};}
Joints joints(const PathVector& values) {Joints result{};std::copy(values.begin(),values.end(),result.begin());return result;}
bool rejects(const std::vector<PathVector>& path,const PathLimits& bound,const PathRetimingOptions& options={}) {
  try {retime_path(path,bound,options);return false;}catch(const std::runtime_error&){return true;}
}
void verify_jtc(const RetimedPath& path,const std::vector<double>& periods) {
  auto message=std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  for(int i=1;i<=6;++i)message->joint_names.push_back("joint_"+std::to_string(i));
  for(const auto& sample:path.samples) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions=sample.position;point.velocities=sample.velocity;point.accelerations=sample.acceleration;
    point.time_from_start=rclcpp::Duration::from_seconds(sample.time);message->points.push_back(point);
  }
  joint_trajectory_controller::Trajectory trajectory(message);
  const auto start=rclcpp::Time(100,0,RCL_ROS_TIME);
  trajectory.set_point_before_trajectory_msg(start,message->points.front());
  trajectory_msgs::msg::JointTrajectoryPoint output;
  joint_trajectory_controller::TrajectoryPointConstIter left,right;
  const auto interpolation=joint_trajectory_controller::interpolation_methods::InterpolationMethod::VARIABLE_DEGREE_SPLINE;
  require(trajectory.sample(start,interpolation,output,left,right),"initial JTC sample");
  Guard guard;guard.lower.fill(-3.);guard.upper.fill(3.);guard.reset(joints(output.positions));
  double time=0.;std::size_t step=0;
  while(time<path.samples.back().time+.1) {
    const double dt=periods[step++%periods.size()];time+=dt;
    require(trajectory.sample(start+rclcpp::Duration::from_seconds(time),interpolation,output,left,right),"JTC sample failed");
    const auto target=joints(output.positions);guard.check(target,target,dt);
    std::size_t segment=path.vertices.size()>1?path.vertices.size()-2:0;
    for(const auto& point:path.samples)if(point.time>=time) {segment=point.segment;break;}
    if(path.vertices.size()>1)require(on_line_segment(output.positions,path.vertices[segment],path.vertices[segment+1],1e-9),"JTC departed geometric segment");
  }
  require(path_distance(output.positions,path.vertices.back())<1e-12,"JTC terminal position");
  require(path_distance(output.velocities,PathVector(6))<1e-12,"JTC terminal velocity");
  require(path_distance(output.accelerations,PathVector(6))<1e-12,"JTC terminal acceleration");
}
void check(const std::vector<PathVector>& geometry,std::size_t vertices) {
  const auto path=retime_path(geometry,limits());
  require(path.vertices.size()==vertices,"wrong vertex reduction");
  require(path.samples.front().time==0.,"missing initial point");
  require(path.samples.front().velocity==PathVector(6) && path.samples.front().acceleration==PathVector(6),"nonstationary start");
  require(path.samples.back().velocity==PathVector(6) && path.samples.back().acceleration==PathVector(6),"nonstationary end");
  for(std::size_t i=1;i<path.samples.size();++i)require(path.samples[i].time>path.samples[i-1].time,"duplicate sample time");
  for(std::size_t v=1;v+1<path.vertices.size();++v) {
    bool stationary=false;
    for(const auto& point:path.samples)if(point.position==path.vertices[v])stationary|=point.velocity==PathVector(6) && point.acceleration==PathVector(6);
    require(stationary,"corner was not a stationary waypoint");
  }
  verify_jtc(path,{.008});verify_jtc(path,{.004,.008,.012});verify_jtc(path,{.012,.004,.008});
}
}
int main() {
  try {
    check({q(0.),q(.01),q(.02),q(.02),q(.04)},2);
    check({q(0.),q(-.03,.012)},2);
    check({q(.8,-.4),q(.82,-.4),q(.82,-.38)},3);
    check({q(0.),q(.02),q(0.)},3);
    check({q(.3),q(.3)},1);
    // Captured six-axis OMPL path that previously started at nonzero v/a.
    check({
      {-0.5610825730281083,1.5856222980266734,-1.3430758018114668,1.4126046666970458,2.0086184585364566,0.5435565864742854},
      {-0.5584742580674482,1.5834939340588403,-1.3395851433074781,1.411741515040514,2.0052958802606837,0.5434060768390281},
      {-0.5539120907460355,1.5797712423254535,-1.3334796811998348,1.4102317885158249,1.9994844049361202,0.5431428225350482},
      {-0.5534557376005566,1.5793988618399315,-1.3328689524296846,1.410080770721032,1.998903083634775,0.5431164892330849}},2);
    auto slow=limits();for(auto& value:slow.velocity)value*=.2;
    require(retime_path({q(0.),q(.2)},slow).samples.back().time>retime_path({q(0.),q(.2)},limits()).samples.back().time,"velocity scaling ineffective");
    auto bad=limits();bad.jerk[0]=0.;require(rejects({q(0.),q(.1)},bad),"accepted absent jerk limit");
    bad=limits();bad.velocity[0]=std::numeric_limits<double>::infinity();require(rejects({q(0.),q(.1)},bad),"accepted nonfinite limit");
    auto point=q(0.);point[1]=std::numeric_limits<double>::quiet_NaN();require(rejects({point,q(.1)},limits()),"accepted nonfinite path");
    PathRetimingOptions options;options.max_duration=.01;require(rejects({q(0.),q(.1)},limits(),options),"ignored duration budget");
    options=PathRetimingOptions{};options.max_samples=2;require(rejects({q(0.),q(.1)},limits(),options),"ignored sample budget");
    options=PathRetimingOptions{};options.max_vertices=2;require(rejects({q(0.),q(.02),q(.02,.02)},limits(),options),"ignored vertex budget");
    std::cout<<"PASS: "<<assertions<<" path retiming assertions with installed JTC interpolation and native Guard; no ROS node or SDK\n";
  }catch(const std::exception& error) {std::cerr<<"FAIL: "<<error.what()<<'\n';return 1;}
}
