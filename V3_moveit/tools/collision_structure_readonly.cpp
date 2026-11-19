// Offline geometry sensitivity diagnostic. No ROS init/node, SDK or motion.
#include <moveit/planning_scene/planning_scene.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>
std::string read(const std::string& p){std::ifstream f(p);if(!f)throw std::runtime_error("read "+p);return {std::istreambuf_iterator<char>(f),{}};}
int main(int argc,char**argv){try{
 if(argc!=8)throw std::runtime_error("usage: ROOT q1 q2 q3 q4 q5 q6");
 const std::string root=argv[1];auto u=urdf::parseURDF(read(root+"/src/v3_moveit/description/s5.urdf"));auto s=std::make_shared<srdf::Model>();
 if(!u||!s->initString(*u,read(root+"/src/v3_moveit/description/s5.srdf")))throw std::runtime_error("model parse");
 auto model=std::make_shared<moveit::core::RobotModel>(u,s);planning_scene::PlanningScene scene(model);
 const auto*g=model->getJointModelGroup("arm");std::vector<double> q;for(int i=2;i<8;++i)q.push_back(std::stod(argv[i]));
 collision_detection::DistanceRequest req;req.type=collision_detection::DistanceRequestType::SINGLE;req.group_name="arm";req.enableGroup(model);req.acm=&scene.getAllowedCollisionMatrix();req.enable_signed_distance=true;
 auto state=scene.getCurrentState();
 std::cout<<std::setprecision(17)<<"{\"offline\":true,\"sdk_access\":false,\"scene_mutation\":false,\"samples\":[";bool first=true;
 for(int axis=-1;axis<6;++axis)for(double delta:std::vector<double>{-.5,-.2,-.1,-.02,0.,.02,.1,.2,.5}){
  if(axis==-1&&delta!=0.)continue;if(axis>=0&&delta==0.)continue;
  auto test=q;if(axis>=0)test[axis]+=delta;state.setJointGroupPositions(g,test);state.update();if(!state.satisfiesBounds(g))continue;
  collision_detection::DistanceResult result;scene.getCollisionEnvUnpadded()->distanceSelf(req,result,state);
  if(!first)std::cout<<",";first=false;std::cout<<"{\"axis\":"<<axis+1<<",\"delta\":"<<delta<<",\"collision\":"<<(result.collision?"true":"false")<<",\"distances\":[";bool fp=true;
  for(const auto& [key,values]:result.distances){if(!((key.first=="base_link"&&key.second=="Link_02")||(key.second=="base_link"&&key.first=="Link_02")||(key.first=="Link_04"&&key.second=="Link_06")||(key.first=="Link_03"&&key.second=="Link_05")))continue;
   for(const auto& d:values){if(!fp)std::cout<<",";fp=false;std::cout<<"{\"pair\":["<<std::quoted(d.link_names[0])<<","<<std::quoted(d.link_names[1])<<"],\"distance_m\":"<<d.distance<<"}";}}
  std::cout<<"]}";
 }
 std::cout<<"]}\n";return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
