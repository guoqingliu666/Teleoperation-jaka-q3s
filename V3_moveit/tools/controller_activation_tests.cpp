// Real upstream controller + loaned in-memory interfaces; no robot SDK.
#include "../src/v3_moveit/cpp/smooth_trajectory_controller.cpp"
#include "v3_moveit/guard.hpp"
#include <fstream>
#include <iostream>
#include <limits>
using CB=controller_interface::CallbackReturn;
void require(bool ok,const char* msg){if(!ok)throw std::runtime_error(msg);}
struct Fixture {
  v3_moveit::SmoothTrajectoryController controller;
  v3_moveit::Joints initial{-.4,1.58,-2.27,3.33,2.23,-.96}, command=initial, position=initial, velocity{};
  std::vector<hardware_interface::CommandInterface> ci;
  std::vector<hardware_interface::StateInterface> si;
  Fixture(const std::string& urdf,double noise,double speed,bool uninitialized=false,bool partial=false){
    for(size_t i=0;i<6;++i){position[i]+=noise*(i%2?1:-1);velocity[i]=speed;}
    if(uninitialized)command.fill(std::numeric_limits<double>::quiet_NaN());
    if(partial)command[1]=std::numeric_limits<double>::quiet_NaN();
    std::vector<std::string> names;for(int i=1;i<=6;++i)names.push_back("joint_"+std::to_string(i));
    rclcpp::NodeOptions opts;opts.parameter_overrides({
      rclcpp::Parameter("joints",names),
      rclcpp::Parameter("command_interfaces",std::vector<std::string>{"position"}),
      rclcpp::Parameter("state_interfaces",std::vector<std::string>{"position","velocity"}),
      rclcpp::Parameter("interpolate_from_desired_state",true),
      rclcpp::Parameter("allow_nonzero_velocity_at_trajectory_end",true)});
    require(controller.init("activation_probe",urdf,125,"",opts)==controller_interface::return_type::OK,"controller init");
    require(controller.configure().label()=="inactive","controller configure");
    ci.reserve(6);si.reserve(12);
    for(size_t i=0;i<6;++i){ci.emplace_back(names[i],"position",&command[i]);si.emplace_back(names[i],"position",&position[i]);si.emplace_back(names[i],"velocity",&velocity[i]);}
    std::vector<hardware_interface::LoanedCommandInterface> lc;
    std::vector<hardware_interface::LoanedStateInterface> ls;
    for(auto& i:ci)lc.emplace_back(i);for(auto& i:si)ls.emplace_back(i);
    controller.assign_interfaces(std::move(lc),std::move(ls));
  }
  ~Fixture(){controller.release_interfaces();}
  CB activate(){return controller.on_activate(rclcpp_lifecycle::State());}
  void check_hold(const v3_moveit::Joints& expected){
    v3_moveit::Guard guard;guard.lower.fill(-7);guard.upper.fill(7);guard.reset(expected);
    for(int tick=0;tick<64;++tick){
      require(controller.update(rclcpp::Time(1000000000LL+tick*8000000LL),rclcpp::Duration(0,8000000))==controller_interface::return_type::OK,"update failed");
      guard.check(command,position,.008);
      for(size_t i=0;i<6;++i){require(std::abs(command[i]-expected[i])<1e-13,"activation changed an existing HOLD command");position[i]+=(tick%2?1:-1)*1.745329252e-5;}
    }
  }
};
int main(int argc,char**argv){try{
 require(argc==2,"usage URDF");std::ifstream file(argv[1]);require(bool(file),"URDF missing");std::string urdf{std::istreambuf_iterator<char>(file),{}};
 rclcpp::init(argc,argv);
 for(double noise:{0.,1.745329252e-5,-3.490658504e-5,6.981317008e-5}){
  Fixture f(urdf,noise,.0002);require(f.activate()==CB::SUCCESS,"stationary activation rejected");f.check_hold(f.initial);
 }
 {Fixture f(urdf,0.,0.,true);require(f.activate()==CB::SUCCESS,"empty fake command bootstrap rejected");f.check_hold(f.initial);}
 {Fixture f(urdf,.002,0.);require(f.activate()==CB::ERROR,"distant command accepted");}
 {Fixture f(urdf,0.,.006);require(f.activate()==CB::ERROR,"moving activation accepted");}
 {Fixture f(urdf,0.,0.,false,true);require(f.activate()==CB::ERROR,"partly invalid command accepted");}
 rclcpp::shutdown();std::cout<<"PASS: 8 activation cases; 320 strict Guard writes; no SDK calls\n";return 0;
}catch(const std::exception&e){std::cerr<<"FAIL: "<<e.what()<<"\n";if(rclcpp::ok())rclcpp::shutdown();return 1;}}
