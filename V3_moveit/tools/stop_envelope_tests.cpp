// Native model/collision regression only: no ROS node, executor, SDK or hardware.
#include "../src/v3_moveit/cpp/path_ruckig_retiming.cpp"
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <iostream>
#include <sstream>

using namespace v3_moveit;
namespace {
int checks=0;
void require(bool ok,const char* message){++checks;if(!ok)throw std::runtime_error(message);}
struct Fixture {
  moveit::core::RobotModelPtr model;
  planning_scene::PlanningScenePtr scene;
  PathVector first=PathVector(6,0.),last=PathVector(6,0.);
  Fixture(){
    std::ostringstream xml;xml<<"<robot name='stop_test'><link name='base'/>";
    for(int i=1;i<=6;++i){
      xml<<"<link name='l"<<i<<"'>";
      if(i==6)xml<<"<collision><geometry><sphere radius='0.0004'/></geometry></collision>";
      xml<<"</link><joint name='joint_"<<i<<"' type='prismatic'><parent link='"<<(i==1?"base":"l"+std::to_string(i-1))
         <<"'/><child link='l"<<i<<"'/><axis xyz='"<<(i==1?"1 0 0":i==2?"0 1 0":"0 0 1")
         <<"'/><limit lower='-3' upper='3' velocity='0.1745329252' effort='1'/></joint>";
    }
    xml<<"</robot>";auto urdf=urdf::parseURDF(xml.str());auto srdf=std::make_shared<srdf::Model>();
    require(urdf!=nullptr&&srdf->initString(*urdf,"<robot name='stop_test'><group name='arm'><chain base_link='base' tip_link='l6'/></group></robot>"),"test model parse");
    model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
    const Guard limits;
    for(int i=1;i<=6;++i){auto name="joint_"+std::to_string(i);auto bound=model->getVariableBounds(name);
      bound.acceleration_bounded_=true;bound.min_acceleration_=-limits.max_acceleration;bound.max_acceleration_=limits.max_acceleration;
      bound.jerk_bounded_=true;bound.min_jerk_=-limits.max_jerk;bound.max_jerk_=limits.max_jerk;
      model->getJointModel(name)->setVariableBounds(name,bound);
    }
    scene=std::make_shared<planning_scene::PlanningScene>(model);first[1]=.2;last=first;last[0]=.2;last[1]=.3;
  }
  planning_interface::MotionPlanRequest request(){
    planning_interface::MotionPlanRequest r;r.group_name="arm";r.max_velocity_scaling_factor=1.;r.max_acceleration_scaling_factor=1.;
    r.start_state.joint_state.name=model->getJointModelGroup("arm")->getVariableNames();
    r.start_state.joint_state.position=first;r.start_state.joint_state.velocity=PathVector(6);return r;
  }
  planning_interface::MotionPlanResponse response(){
    planning_interface::MotionPlanResponse r;r.error_code=moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    r.trajectory=std::make_shared<robot_trajectory::RobotTrajectory>(model,"arm");
    auto state=scene->getCurrentState();state.setJointGroupPositions("arm",first);state.update();r.trajectory->addSuffixWayPoint(state,0.);
    state.setJointGroupPositions("arm",last);state.update();r.trajectory->addSuffixWayPoint(state,0.);return r;
  }
};
}
int main(){try{
  const Guard g;JointStopProfile::Joints vm,am,jm;vm.fill(g.max_velocity);am.fill(g.max_acceleration);jm.fill(g.max_jerk);
  require(JointStopProfile::limits_match_contract(vm,am,jm,1.5),"correct contract rejected");
  auto wrong=jm;wrong[1]+=.001;require(!JointStopProfile::limits_match_contract(vm,am,wrong,1.5),"different jerk accepted");
  require(!JointStopProfile::limits_match_contract(vm,am,jm,1.6),"different stop duration accepted");
  JointStopProfile stationary;PathVector q={.5,-.7,1.2,-1.3,2.,-.2},zero(6);
  require(stationary.start(q,zero,zero,vm,am,jm,1.5),"stationary profile failed");
  for(size_t i=0;i<6;++i)require(std::abs(stationary.lower_position()[i]-q[i])<1e-12&&std::abs(stationary.upper_position()[i]-q[i])<1e-12,"velocity-mode extrema invented zero target");
  JointStopProfile moving;auto v=zero;v[0]=.1;v[1]=-.04;
  require(moving.start(q,v,zero,vm,am,jm,1.5),"moving profile failed");
  const auto times=moving.validation_times();require(times.front()==0.&&times.back()==moving.duration(),"missing exact envelope endpoints");
  JointStopProfile::Joints p,dv,da;
  for(size_t k=0;k<times.size();++k){
    require(k==0||times[k]-times[k-1]<=.008+1e-12,"stop validation gap over 8 ms");
    require(moving.sample_at(times[k],p,dv,da),"stop validation sample failed");
    for(size_t i=0;i<6;++i)require(p[i]>=moving.lower_position()[i]-1e-12&&p[i]<=moving.upper_position()[i]+1e-12,"sample outside computed extrema");
  }
  require(moving.sample_at(.008,p,dv,da)&&moving.step(.008),"runtime/reference sample failed");
  require(p==moving.position()&&dv==moving.velocity()&&da==moving.acceleration(),"validation curve differs from production step");
  Fixture f;PathRuckigRetiming adapter;auto req=f.request();auto res=f.response();
  {
    auto probe=f.scene->getCurrentState();const auto* group=f.model->getJointModelGroup("arm");
    std::array<double,6> q{},v{},a{};v[0]=g.max_velocity*1.001;
    probe.setJointGroupVelocities(group,v.data());probe.setJointGroupPositions(group,q.data());probe.update();
    require(!probe.satisfiesBounds(group),"fixture must reproduce stale measured-velocity rejection");
    v[0]=g.max_velocity*.9;a[0]=g.max_acceleration*.5;
    require(set_sampled_robot_state(probe,group,q,v,a,vm,am),"valid prospective q/v/a rejected by stale feedback");
    require(probe.getJointVelocities("joint_1")[0]==v[0]&&probe.getJointAccelerations("joint_1")[0]==a[0],"prospective dynamics not applied");
    v[0]=g.max_velocity*1.001;require(!set_sampled_robot_state(probe,group,q,v,a,vm,am),"real sample overspeed accepted");
    v[0]=0.;a[0]=g.max_acceleration*1.001;require(!set_sampled_robot_state(probe,group,q,v,a,vm,am),"real sample acceleration violation accepted");
    a[0]=0.;q[0]=3.001;require(!set_sampled_robot_state(probe,group,q,v,a,vm,am),"real sample position violation accepted");
  }
  const auto started=std::chrono::steady_clock::now();adapter.adapt(f.scene,req,res);
  require(bool(res)&&res.trajectory!=nullptr,"empty scene rejected valid stop envelopes");
  std::cout<<"empty_scene_validation_s="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()<<'\n';
  // Locate a stop state outside the checked straight geometric path. The
  // obstacle collides only with the cancellation curve, not the planned path.
  PathLimits limits{PathVector(6,g.max_velocity),PathVector(6,g.max_acceleration),PathVector(6,g.max_jerk)};
  const auto path=retime_path({f.first,f.last},limits);double maximum_departure=0.;JointStopProfile::Joints obstacle{};
  for(const auto& point:path.samples){if(std::llround(point.time*1e9)%8000000!=0)continue;
    JointStopProfile stop;require(stop.start(point.position,point.velocity,point.acceleration,vm,am,jm,1.5),"test stop start failed");
    for(double t:stop.validation_times()){stop.sample_at(t,p,dv,da);const double departure=std::abs(p[1]-.2-.5*p[0])/std::sqrt(1.25);
      if(departure>maximum_departure){maximum_departure=departure;obstacle=p;}}
  }
  require(maximum_departure>.002,"fixture did not leave original geometric segment");
  moveit_msgs::msg::CollisionObject object;object.id="cancel_only_obstacle";object.header.frame_id="base";
  shape_msgs::msg::SolidPrimitive shape;shape.type=shape.SPHERE;shape.dimensions={.0004};object.primitives.push_back(shape);
  geometry_msgs::msg::Pose pose;pose.orientation.w=1.;pose.position.x=obstacle[0];pose.position.y=obstacle[1];object.primitive_poses.push_back(pose);
  require(f.scene->processCollisionObjectMsg(object),"collision fixture rejected");
  auto state=f.scene->getCurrentState();for(const auto& point:path.samples){state.setJointGroupPositions("arm",point.position);state.update();require(!f.scene->isStateColliding(state,"arm"),"original path already collides");}
  res=f.response();adapter.adapt(f.scene,req,res);require(!bool(res)&&!res.trajectory,"cancel-only obstacle was accepted");
  f.scene->getWorldNonConst()->removeObject(object.id);
  req=f.request();moveit_msgs::msg::JointConstraint constraint;constraint.joint_name="joint_1";constraint.position=0.;constraint.tolerance_above=.01;constraint.tolerance_below=3.;constraint.weight=1.;req.path_constraints.joint_constraints.push_back(constraint);
  res=f.response();adapter.adapt(f.scene,req,res);require(!bool(res)&&!res.trajectory,"path constraint violation accepted");
  req=f.request();req.path_constraints.joint_constraints.push_back(constraint);req.path_constraints.joint_constraints.front().joint_name="not_a_joint";
  res=f.response();adapter.adapt(f.scene,req,res);require(!bool(res)&&!res.trajectory,"invalid path constraint silently ignored");
  req=f.request();res=f.response();adapter.adapt({},req,res);require(!bool(res)&&!res.trajectory,"missing scene accepted");
  // A long path needs >30000 state checks; either hard limit must fail closed.
  f.last[0]=2.8;f.last[1]=.2;req=f.request();res=f.response();adapter.adapt(f.scene,req,res);require(!bool(res)&&!res.trajectory,"validation budget did not fail closed");
  std::cout<<"PASS: "<<checks<<" stop-envelope assertions, real native MoveIt collision checks; no ROS node or SDK\n";
}catch(const std::exception& error){std::cerr<<"FAIL: "<<error.what()<<'\n';return 1;}}
