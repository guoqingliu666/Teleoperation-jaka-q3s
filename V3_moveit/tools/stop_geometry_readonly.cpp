// Offline diagnostic of a recorded simulated q/v/a and projected stop.
// No ROS initialization, node, SDK, network or commanded motion.
#include "v3_moveit/joint_stop_profile.hpp"
#include <moveit/planning_scene/planning_scene.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
std::string read(const std::string&p){std::ifstream f(p);if(!f)throw std::runtime_error(p);return {std::istreambuf_iterator<char>(f),{}};}
int main(int argc,char**argv){try{
 if(argc!=3)throw std::runtime_error("usage ROOT recorded_q_v_a.txt");std::string root=argv[1];std::ifstream input(argv[2]);
 auto u=urdf::parseURDF(read(root+"/src/v3_moveit/description/s5.urdf"));auto sr=std::make_shared<srdf::Model>();if(!sr->initString(*u,read(root+"/src/v3_moveit/description/s5.srdf")))throw std::runtime_error("SRDF");
 auto model=std::make_shared<moveit::core::RobotModel>(u,sr);planning_scene::PlanningScene scene(model);auto state=scene.getCurrentState();const auto*g=model->getJointModelGroup("arm");
 v3_moveit::Joints p,v,a,vm,am,jm;for(auto& x:p)input>>x;for(auto&x:v)input>>x;for(auto&x:a)input>>x;
 moveit_msgs::msg::CollisionObject co;co.id="cabinet_floor_proxy";co.header.frame_id="world";co.operation=co.ADD;shape_msgs::msg::SolidPrimitive shape;shape.type=shape.BOX;shape.dimensions.resize(3);for(auto&x:shape.dimensions)input>>x;geometry_msgs::msg::Pose pose;input>>pose.position.x>>pose.position.y>>pose.position.z;pose.orientation.w=1.;co.primitives.push_back(shape);co.primitive_poses.push_back(pose);if(!input||!scene.processCollisionObjectMsg(co))throw std::runtime_error("input/scene");
 vm.fill(.1745329252);am.fill(.6981317008);jm.fill(2.7925268032);
 std::cout<<std::setprecision(17)<<"{\"offline_only\":true,\"sdk_calls\":0,\"physical_acceptance\":false,\"projection\":\"constant acceleration extrapolation, not an executed or exact next command\",\"samples\":[";
 for(int k=0;k<=10;++k){double dt=k*.008;auto q=p,w=v,b=a;for(size_t i=0;i<6;++i){q[i]+=v[i]*dt+.5*a[i]*dt*dt;w[i]=std::clamp(v[i]+a[i]*dt,-vm[i],vm[i]);}
 v3_moveit::JointStopProfile stop;if(!stop.start({q.begin(),q.end()},{w.begin(),w.end()},{b.begin(),b.end()},vm,am,jm,1.5))throw std::runtime_error(stop.error());
 if(k)std::cout<<",";std::cout<<"{\"projection_seconds\":"<<dt<<",\"stop_seconds\":"<<stop.duration()<<",\"collisions\":[";bool first=true;
 for(double t:stop.validation_times()){v3_moveit::Joints x,y,z;stop.sample_at(t,x,y,z);state.setJointGroupPositions(g,x.data());state.update();collision_detection::CollisionRequest request;request.group_name="arm";request.contacts=true;request.max_contacts=10;collision_detection::CollisionResult result;scene.checkCollision(request,result,state);
 if(result.collision){for(const auto& pair:result.contacts){if(!first)std::cout<<",";first=false;std::cout<<"{\"stop_at\":"<<t<<",\"pair\":["<<std::quoted(pair.first.first)<<","<<std::quoted(pair.first.second)<<"]}";}break;}}
 std::cout<<"]}";}
 std::cout<<"]}\n";
}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
