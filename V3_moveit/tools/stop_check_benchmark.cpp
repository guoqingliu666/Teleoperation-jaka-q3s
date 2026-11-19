// Offline timing diagnostic. No ROS initialization, SDK, network or robot commands.
#include "v3_moveit/joint_stop_profile.hpp"
#include "v3_moveit/sampled_robot_state.hpp"
#include <moveit/planning_scene/planning_scene.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <algorithm>
std::string read(const std::string&p){std::ifstream f(p);if(!f)throw std::runtime_error(p);return {std::istreambuf_iterator<char>(f),{}};}
double cpu(){timespec t;clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t);return t.tv_sec+t.tv_nsec*1e-9;}
int main(int argc,char**argv){try{
 if(argc!=3)throw std::runtime_error("usage ROOT q_and_floor.txt");std::string root=argv[1];std::ifstream data(argv[2]);
 auto u=urdf::parseURDF(read(root+"/src/v3_moveit/description/s5.urdf"));auto sr=std::make_shared<srdf::Model>();if(!sr->initString(*u,read(root+"/src/v3_moveit/description/s5.srdf")))throw std::runtime_error("SRDF");
 auto model=std::make_shared<moveit::core::RobotModel>(u,sr);planning_scene::PlanningScene scene(model);const auto*g=model->getJointModelGroup("arm");
 v3_moveit::Joints q,vm,am,jm;for(auto&x:q)data>>x;v3_moveit::Guard guard;vm.fill(guard.max_velocity);am.fill(guard.max_acceleration);jm.fill(guard.max_jerk);
 moveit_msgs::msg::CollisionObject co;co.id="cabinet_floor_proxy";co.header.frame_id="world";co.operation=co.ADD;shape_msgs::msg::SolidPrimitive shape;shape.type=shape.BOX;shape.dimensions.resize(3);for(auto&x:shape.dimensions)data>>x;geometry_msgs::msg::Pose pose;data>>pose.position.x>>pose.position.y>>pose.position.z;pose.orientation.w=1.;co.primitives.push_back(shape);co.primitive_poses.push_back(pose);if(!data||!scene.processCollisionObjectMsg(co))throw std::runtime_error("input/scene");
 std::vector<double> elapsed,compute;size_t checks=0,collisions=0;double worst_cpu=0.;
 for(int run=0;run<1000;++run){auto start=std::chrono::steady_clock::now();double c0=cpu();
  v3_moveit::Joints v{},a{};for(size_t i=0;i<6;++i)v[i]=vm[i]*(run%2?-1.:1.)*(.3+.7*(run%7)/6.);
  v3_moveit::JointStopProfile stop;if(!stop.start({q.begin(),q.end()},{v.begin(),v.end()},{a.begin(),a.end()},vm,am,jm,1.5))throw std::runtime_error(stop.error());
  auto state=scene.getCurrentState();
  for(double t:stop.validation_times()){v3_moveit::Joints x,y,z;stop.sample_at(t,x,y,z);if(!v3_moveit::set_sampled_robot_state(state,g,x,y,z,vm,am))throw std::runtime_error("bounds");collisions+=scene.isStateColliding(state,"arm");++checks;}
  double c=1000*(cpu()-c0),w=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();if(elapsed.empty()||w>*std::max_element(elapsed.begin(),elapsed.end()))worst_cpu=c;elapsed.push_back(w);compute.push_back(c);
 }
 std::sort(elapsed.begin(),elapsed.end());std::sort(compute.begin(),compute.end());std::cout<<std::setprecision(10)<<"{\"offline_only\":true,\"sdk_calls\":0,\"iterations\":1000,\"scene_checks\":"<<checks<<",\"collision_samples\":"<<collisions<<",\"wall_ms_p50\":"<<elapsed[500]<<",\"wall_ms_p99\":"<<elapsed[990]<<",\"wall_ms_max\":"<<elapsed.back()<<",\"cpu_ms_p50\":"<<compute[500]<<",\"cpu_ms_p99\":"<<compute[990]<<",\"cpu_ms_at_worst_wall\":"<<worst_cpu<<",\"over_6ms\":"<<std::count_if(elapsed.begin(),elapsed.end(),[](double t){return t>6.;})<<",\"physical_acceptance\":false}\n";
}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
