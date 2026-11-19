// Explicit cabinet power/enable actions. No Servo, EDG, trajectory or joint targets.
#include "v3_moveit/jaka_sdk.hpp"
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fcntl.h>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <thread>
#include <unistd.h>
static volatile sig_atomic_t canceled=0;
static int last_sdk_code=0;
static void stop(int){canceled=1;}
static void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
static void checked(int rc,const char* op){if(rc){last_sdk_code=rc;throw std::runtime_error(std::string(op)+" code="+std::to_string(rc));}}
static void poll_delay(){std::this_thread::sleep_for(std::chrono::milliseconds(100));}
struct Snapshot {RobotStatus_simple status{};MotionStatus motion{};BOOL servo=0,drag=0;ProgramState program{};JointValue q{};};
static Snapshot read_state(JKHD& h){
  Snapshot s;checked(get_robot_status_simple(&h,&s.status),"读取电源/使能状态");checked(get_motion_status(&h,&s.motion),"读取运动状态");
  checked(is_in_servomove(&h,&s.servo),"读取 Servo 状态");checked(is_in_drag_mode(&h,&s.drag),"读取拖动状态");checked(get_program_state(&h,&s.program),"读取程序状态");checked(get_actual_joint_position(&h,&s.q),"读取关节反馈");return s;
}
static void emit_state(const Snapshot& s){
  std::cout<<"{\"event\":\"status\",\"powered_on\":"<<s.status.powered_on<<",\"enabled\":"<<s.status.enabled<<",\"errcode\":"<<s.status.errcode<<",\"estop\":"<<s.motion.isInEstop<<",\"collision\":"<<s.motion.isInCollision<<",\"servo\":"<<s.servo<<",\"drag\":"<<s.drag<<",\"program\":"<<int(s.program)<<",\"queue\":"<<s.motion.queue<<",\"active_queue\":"<<s.motion.active_queue<<",\"inpos\":"<<s.motion.inpos<<"}"<<std::endl;
}
static void guard(const Snapshot& s,const std::array<double,6>& lo,const std::array<double,6>& hi){
  require(!canceled,"操作已取消");require(!s.status.errcode,"机器人有报警，请先在控制柜处理");
  require(!s.motion.isInEstop&&!s.motion.isInCollision&&!s.motion.isOnLimit,"急停、碰撞或限位状态未解除");
  require(!s.servo&&!s.drag&&s.program==PROGRAM_IDLE&&!s.motion.queue&&!s.motion.active_queue&&!s.motion.paused,"机器人正被 Servo、程序、拖动或运动队列占用");
  require(s.status.enabled!=1||s.motion.inpos==TRUE,"机器人运动尚未完成");
  for(int i=0;i<6;i++)require(std::isfinite(s.q.jVal[i])&&s.q.jVal[i]>=lo[i]&&s.q.jVal[i]<=hi[i],"关节反馈无效或超出模型范围");
}
static int emit_tooling(JKHD& h,bool list){
  int id=-1;CartesianPose tcp{};PayLoad load{};
  checked(get_tool_id(&h,&id),"读取当前工具");checked(get_tool_data(&h,id,&tcp),"读取当前 TCP");checked(get_payload(&h,&load),"读取负载与质心");
  double t[]={tcp.tran.x,tcp.tran.y,tcp.tran.z,tcp.rpy.rx,tcp.rpy.ry,tcp.rpy.rz};
  for(double v:t)require(std::isfinite(v),"TCP 读数无效");
  require(std::isfinite(load.mass)&&load.mass>=0&&std::isfinite(load.centroid.x)&&std::isfinite(load.centroid.y)&&std::isfinite(load.centroid.z),"负载或质心读数无效");
  std::cout<<"{\"event\":\"tooling\",\"tool_id\":"<<id<<",\"tcp_mm_rad\":[";
  for(int i=0;i<6;i++){if(i)std::cout<<',';std::cout<<t[i];}
  std::cout<<"],\"payload_kg\":"<<load.mass<<",\"cog_mm\":["<<load.centroid.x<<','<<load.centroid.y<<','<<load.centroid.z<<"],\"tools\":[";
  bool comma=false;
  if(list)for(int tool=0;tool<=10;tool++){
    CartesianPose p{};if(get_tool_data(&h,tool,&p))continue;
    double values[]={p.tran.x,p.tran.y,p.tran.z,p.rpy.rx,p.rpy.ry,p.rpy.rz};bool valid=true;for(double v:values)valid&=std::isfinite(v);if(!valid)continue;
    if(comma)std::cout<<',';
    comma=true;
    std::cout<<"{\"tool_id\":"<<tool<<",\"tcp_mm_rad\":[";for(int i=0;i<6;i++){if(i)std::cout<<',';std::cout<<values[i];}std::cout<<"]}";
  }
  std::cout<<"]}"<<std::endl;
  return id;
}
int main(int argc,char** argv){
  if(argc!=39||std::string(argv[1])!="--hardware")return 2;
  signal(SIGTERM,stop);signal(SIGINT,stop);std::cout<<std::setprecision(17)<<std::unitbuf;
  JKHD h{};bool connected=false;int lock=-1,result=1;std::string action=argv[3];
  try{
    require(action=="poll"||action=="status"||action=="check"||action=="power_on"||action=="enable"||action=="select_tool"||action=="refresh_payload","未知机器人操作");
    int requested_tool=std::stoi(argv[32]);require(requested_tool>=0&&requested_tool<=10,"工具号无效");
    std::array<double,6> expected_tool{};for(int i=0;i<6;i++){expected_tool[i]=std::stod(argv[33+i]);require(std::isfinite(expected_tool[i]),"TCP 配置无效");}
    double mass=std::stod(argv[4]);std::array<double,3> cog{};JointValue ref{};std::array<double,6> expected{},lo{},hi{};
    require(std::isfinite(mass)&&mass>=0,"负载配置无效");
    for(int i=0;i<3;i++){cog[i]=std::stod(argv[5+i]);require(std::isfinite(cog[i]),"重心配置无效");}
    for(int i=0;i<6;i++){ref.jVal[i]=std::stod(argv[8+i]);expected[i]=std::stod(argv[14+i]);lo[i]=std::stod(argv[20+i]);hi[i]=std::stod(argv[26+i]);require(std::isfinite(ref.jVal[i])&&std::isfinite(expected[i])&&std::isfinite(lo[i])&&std::isfinite(hi[i])&&lo[i]<hi[i],"模型配置无效");}
    lock=open(".cache/operator/control.lock",O_CREAT|O_RDWR|O_NOFOLLOW,0600);require(lock>=0&&flock(lock,LOCK_EX|LOCK_NB)==0,"DEVICE_BUSY: 已有 V3 控制会话，请先停止遥操作");
    checked(create_handler(argv[2],&h),"连接 JAKA 控制柜");connected=true;
    Snapshot first=read_state(h);emit_state(first);
    if(action!="poll")emit_tooling(h,true);
    if(action=="select_tool"||action=="refresh_payload"){
      guard(first,lo,hi);poll_delay();Snapshot fresh=read_state(h);guard(fresh,lo,hi);
      for(int i=0;i<6;i++)require(std::abs(fresh.q.jVal[i]-first.q.jVal[i])<.001,"机器人尚未静止");
      int current=-1;checked(get_tool_id(&h,&current),"读取当前工具");
      if(action=="select_tool"){
        CartesianPose selected{};checked(get_tool_data(&h,requested_tool,&selected),"读取所选 TCP 定义");
        double v[]={selected.tran.x,selected.tran.y,selected.tran.z,selected.rpy.rx,selected.rpy.ry,selected.rpy.rz};for(double n:v)require(std::isfinite(n),"所选 TCP 定义无效");
        require(!canceled,"操作已取消");if(current!=requested_tool)checked(set_tool_id(&h,requested_tool),"切换 JAKA 工具坐标系");
        checked(get_tool_id(&h,&current),"回读工具号");require(current==requested_tool,"TOOL_MISMATCH: 工具切换未确认，请重新读取状态");
      }else require(current==requested_tool,"TOOL_MISMATCH: 当前工具与 V3 不一致，请先选择并应用工具坐标系");
      require(emit_tooling(h,false)==requested_tool,"TOOL_MISMATCH: 回读期间工具被改变，请重新操作");
    }else if(action!="status"&&action!="poll"){
      guard(first,lo,hi);
      int tool=0,level=0;CartesianPose tcp{},fk{};PayLoad payload{};
      checked(get_tool_id(&h,&tool),"读取工具号");require(tool==requested_tool,"TOOL_MISMATCH: 当前工具与 V3 所选工具不一致");
      checked(get_tool_data(&h,tool,&tcp),"读取 TCP");double t[]={tcp.tran.x,tcp.tran.y,tcp.tran.z,tcp.rpy.rx,tcp.rpy.ry,tcp.rpy.rz};
      for(int i=0;i<6;i++)require(std::isfinite(t[i])&&std::abs(t[i]-expected_tool[i])<=.001,"TCP_MISMATCH: TCP 定义已变化，请重新应用工具坐标系");
      checked(get_payload(&h,&payload),"读取负载");require(std::isfinite(payload.mass)&&std::abs(payload.mass-mass)<=1e-6,"PAYLOAD_MISMATCH: 实际负载与 V3 不一致");
      double c[]={payload.centroid.x,payload.centroid.y,payload.centroid.z};for(int i=0;i<3;i++)require(std::isfinite(c[i])&&std::abs(c[i]-cog[i])<=.001,"PAYLOAD_CENTROID_MISMATCH: 实际负载质心与 V3 不一致");
      checked(get_collision_level(&h,&level),"读取碰撞保护");require(level>=1&&level<=5,"控制柜碰撞保护未开启");
      checked(kine_forward(&h,&ref,&fk),"核对机器人型号和 FK");double f[]={fk.tran.x,fk.tran.y,fk.tran.z,fk.rpy.rx,fk.rpy.ry,fk.rpy.rz};
      for(int i=0;i<6;i++){double e=f[i]-expected[i];if(i>=3)e=std::atan2(std::sin(e),std::cos(e));require(std::isfinite(e)&&std::abs(e)<=(i<3?.1:.001),"控制柜机器人型号 / FK 与工程不符");}
      poll_delay();Snapshot fresh=read_state(h);guard(fresh,lo,hi);
      for(int i=0;i<6;i++)require(std::abs(fresh.q.jVal[i]-first.q.jVal[i])<.001,"机器人尚未静止");
      bool done=action=="power_on"?fresh.status.powered_on==1:fresh.status.enabled==1;
      if(action=="check")done=true;
      if(action=="enable")require(fresh.status.powered_on==1,"请先点击打开本体电源");
      if(!done){
        require(!canceled,"操作已取消");checked(action=="power_on"?power_on(&h):enable_robot(&h),action=="power_on"?"打开本体电源":"机器人使能");
        std::cout<<"{\"event\":\"command\",\"action\":\""<<action<<"\"}"<<std::endl;
        auto until=std::chrono::steady_clock::now()+std::chrono::seconds(30);
        while(!done&&std::chrono::steady_clock::now()<until){poll_delay();fresh=read_state(h);emit_state(fresh);guard(fresh,lo,hi);done=action=="power_on"?fresh.status.powered_on==1:fresh.status.enabled==1;}
        require(done,"指令已发出但状态未确认，请读取状态后再操作");
      }
      emit_state(fresh);
    }
    result=0;
  }catch(const std::exception& e){
    bool busy=std::string(e.what()).find("DEVICE_BUSY:")!=std::string::npos||last_sdk_code==-10||last_sdk_code==-58;
    std::cout<<"{\"event\":\"error\",\"detail\":\""<<e.what()<<"\",\"sdk_code\":"<<last_sdk_code<<",\"kind\":\""<<(busy?"occupied":"error")<<"\"}"<<std::endl;
  }
  int disconnect=connected?destory_handler(&h):0;if(lock>=0)close(lock);
  std::cout<<"{\"event\":\"result\",\"ok\":"<<(!result&&!disconnect?"true":"false")<<",\"disconnect_rc\":"<<disconnect<<",\"motion_commands\":0,\"servo_enable_calls\":0}"<<std::endl;
  return result||disconnect?1:0;
}
