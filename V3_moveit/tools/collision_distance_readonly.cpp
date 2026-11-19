// Read-only diagnostic: requests a planning-scene snapshot; never publishes motion,
// modifies the remote scene/ACM, or loads/calls the robot SDK.
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <chrono>

std::string read(const std::string& path) {
  std::ifstream f(path); if (!f) throw std::runtime_error("cannot read " + path);
  return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}
void number(std::ostream& s,double x) { if(std::isfinite(x)&&std::abs(x)<1e100)s<<x;else s<<"null"; }
void pair_json(std::ostream& s,const collision_detection::DistanceResultsData& d) {
  s<<"{\"pair\":["<<std::quoted(d.link_names[0])<<","<<std::quoted(d.link_names[1])<<"],\"distance_m\":";
  number(s,d.distance);s<<",\"nearest_points\":[";
  for(int i=0;i<2;++i){if(i)s<<",";s<<"[";for(int j=0;j<3;++j){if(j)s<<",";number(s,d.nearest_points[i][j]);}s<<"]";}s<<"]}";
}
void result_json(std::ostream& s,const collision_detection::DistanceResult& d) {
  s<<"{\"collision\":"<<(d.collision?"true":"false")<<",\"minimum\":";pair_json(s,d.minimum_distance);
  std::vector<collision_detection::DistanceResultsData> near;
  for(const auto& item:d.distances)for(const auto& point:item.second)near.push_back(point);
  std::sort(near.begin(),near.end(),[](const auto&a,const auto&b){return a.distance<b.distance;});
  s<<",\"nearest_pairs\":[";for(size_t i=0;i<std::min<size_t>(near.size(),20);++i){if(i)s<<",";pair_json(s,near[i]);}s<<"]}";
}
int main(int argc,char**argv) {
  if(argc<2){std::cerr<<"usage: collision_distance_readonly PROJECT_ROOT [OUTPUT_JSON]\n";return 2;}
  try {
    const std::string root=argv[1],outpath=argc>2?argv[2]:"";
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("v3_collision_distance_readonly");
    auto client=node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    if(!client->wait_for_service(std::chrono::seconds(5)))throw std::runtime_error("get_planning_scene unavailable");
    auto req=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    req->components.components=511; // settings, current state/attached, world, octomap, transforms, ACM, padding
    auto future=client->async_send_request(req);
    if(rclcpp::spin_until_future_complete(node,future,std::chrono::seconds(5))!=rclcpp::FutureReturnCode::SUCCESS)
      throw std::runtime_error("get_planning_scene timed out");
    const auto response=future.get();const auto receipt=node->now();
    auto urdf=urdf::parseURDF(read(root+"/src/v3_moveit/description/s5.urdf"));
    auto srdf=std::make_shared<srdf::Model>();
    if(!urdf||!srdf->initString(*urdf,read(root+"/src/v3_moveit/description/s5.srdf")))throw std::runtime_error("model parse failed");
    auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
    planning_scene::PlanningScene scene(model);
    if(!scene.setPlanningSceneMsg(response->scene))throw std::runtime_error("scene snapshot rejected");
    if(!model->hasJointModelGroup("arm"))throw std::runtime_error("arm group absent");
    auto state=scene.getCurrentState();state.update();
    collision_detection::DistanceRequest dreq;dreq.enable_nearest_points=true;dreq.enable_signed_distance=true;
    dreq.type=collision_detection::DistanceRequestType::SINGLE;dreq.group_name="arm";dreq.enableGroup(model);
    dreq.acm=&scene.getAllowedCollisionMatrix();
    collision_detection::DistanceResult self,world,padded_self;
    scene.getCollisionEnvUnpadded()->distanceSelf(dreq,self,state);
    scene.getCollisionEnv()->distanceRobot(dreq,world,state);
    scene.getCollisionEnv()->distanceSelf(dreq,padded_self,state);
    std::ostringstream report;report<<std::setprecision(17)<<"{\"read_only\":true,\"sdk_access\":false,\"scene_service\":\"/get_planning_scene\",\"scene_name\":"<<std::quoted(response->scene.name)
      <<",\"collision_detector\":"<<std::quoted(scene.getCollisionDetectorName())<<",\"receipt_ros_s\":"<<receipt.seconds()
      <<",\"joint_stamp_ros_s\":"<<rclcpp::Time(response->scene.robot_state.joint_state.header.stamp).seconds()
      <<",\"scene_is_diff\":"<<(response->scene.is_diff?"true":"false")<<",\"group\":\"arm\",\"joint_names\":[";
    const auto* group=model->getJointModelGroup("arm");const auto names=group->getVariableNames();
    std::vector<double> q;state.copyJointGroupPositions(group,q);
    for(size_t i=0;i<names.size();++i){if(i)report<<",";report<<std::quoted(names[i]);}report<<"],\"q\":[";
    for(size_t i=0;i<q.size();++i){if(i)report<<",";number(report,q[i]);}report<<"],\"world_ids\":[";
    const auto ids=scene.getWorld()->getObjectIds();for(size_t i=0;i<ids.size();++i){if(i)report<<",";report<<std::quoted(ids[i]);}
    report<<"],\"attached_ids\":[";std::vector<const moveit::core::AttachedBody*> attached;state.getAttachedBodies(attached);
    for(size_t i=0;i<attached.size();++i){if(i)report<<",";report<<std::quoted(attached[i]->getName());}
    report<<"],\"self_unpadded\":";result_json(report,self);report<<",\"world_padded\":";result_json(report,world);
    report<<",\"self_padded_comparison\":";result_json(report,padded_self);report<<"}\n";
    if(!outpath.empty()){std::ofstream f(outpath);if(!f)throw std::runtime_error("output unavailable");f<<report.str();}
    std::cout<<report.str();rclcpp::shutdown();return 0;
  } catch(const std::exception&e){std::cerr<<"read-only diagnostic failed: "<<e.what()<<"\n";if(rclcpp::ok())rclcpp::shutdown();return 1;}
}
