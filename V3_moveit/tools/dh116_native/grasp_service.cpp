// Single DH116 owner. SDK setters run on this thread; SOEM owns cyclic exchange.
#include "EthercatMaster.h"
#include "grasp_policy.hpp"
#include "grasp_sdk_motion.hpp"
#include "grasp_manual.hpp"
#include "LHandProLib.hpp"
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fcntl.h>
#include <grp.h>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <pwd.h>
#include <sstream>
#include <stdexcept>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "grasp_workspace.hpp"
using namespace lhplib;
static volatile sig_atomic_t running=1;
static void stop_signal(int){running=0;}
static double now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
static void check(int rc){require(rc==0,"SDK command failed");}
static std::string hex(const uint8_t* b,int n){std::ostringstream s;for(int i=0;i<n;i++)s<<std::hex<<std::setw(2)<<std::setfill('0')<<int(b[i]);return s.str();}
template<class A> static void array_json(const A& a,std::ostream& out=std::cout){out<<'[';bool first=true;for(auto v:a){if(!first)out<<',';first=false;out<<v;}out<<']';}
static void publish_frame(const std::ostringstream& out,bool workspace){
  const std::string frame=out.str()+"\n";
  if(!workspace){std::cout<<frame<<std::flush;return;}
  // Each UI sample is a complete pipe write. Keep capacity for mode/fault acknowledgements.
  int bytes=0,capacity=fcntl(STDOUT_FILENO,F_GETPIPE_SZ);
  if(capacity>0 && ioctl(STDOUT_FILENO,FIONREAD,&bytes)==0 && capacity-bytes<int(frame.size())+1024)return;
  if(frame.size()<=4096){auto ignored=write(STDOUT_FILENO,frame.data(),frame.size());(void)ignored;}
}
using Getter=int(LHandProLib::*)(int,float*,int*);
static std::vector<float> forces(LHandProLib& sdk,int id,Getter getter){int n=0;check((sdk.*getter)(id,nullptr,&n));require(n>0&&n<=16,"Bad force count");std::vector<float> v(n);check((sdk.*getter)(id,v.data(),&n));require(n==int(v.size()),"Changed force count");for(float x:v)require(std::isfinite(x)&&x>=0,"Invalid force sample");return v;}
static std::vector<float> optional_sensor(LHandProLib& sdk,int id,Getter getter){int n=0;if((sdk.*getter)(id,nullptr,&n)||n<=0||n>128)return {};std::vector<float> v(n);if((sdk.*getter)(id,v.data(),&n)||n!=int(v.size()))return {};for(float x:v)if(!std::isfinite(x)||x<0)return {};return v;}
// Independent register read, also used after the master has closed to prove INIT.
static bool verify_init(ecx_contextt& c){
  // This second socket has also seen the cyclic broadcast traffic. Discard it
  // before issuing an independent register request; never interpret a queued PDO as ALSTAT.
  for(int attempt=0;attempt<10;attempt++){
    unsigned char discarded[2048];while(recv(c.port.sockhandle,discarded,sizeof(discarded),MSG_DONTWAIT)>0){}
    uint16_t state=0;int w=ecx_BRD(&c.port,0,ECT_REG_ALSTAT,2,&state,20000);
    if(w==1&&etohs(state)==EC_STATE_INIT)return true;usleep(20000);
  }return false;
}
int main(int argc,char** argv){
  if((argc!=19&&argc!=31) || std::string(argv[1])!="--hardware") {std::cerr<<"Usage: --hardware INTERFACE MODE VELOCITY CURRENT FORCE OPENx6 CLOSEDx6 [manual: VELOCITYx6 CURRENTx6]\n";return 2;}
  // A stalled GUI/log reader must not block the watchdog on a full stdout pipe.
  fcntl(STDOUT_FILENO,F_SETFL,fcntl(STDOUT_FILENO,F_GETFL)|O_NONBLOCK);
  std::cout<<std::fixed<<std::setprecision(9);
  signal(SIGINT,stop_signal);signal(SIGTERM,stop_signal);signal(SIGPIPE,SIG_IGN);
  const auto parent=getppid();const std::string mode=argv[3];
  const bool workspace=mode=="workspace";
  const bool thumb_probe=mode=="thumb_probe"||mode=="thumb_probe_combined"||mode=="thumb_open_combined"||mode=="thumb_probe_after_enable";
  ecx_contextt verifier{};bool verifier_open=false;
  EthercatMaster master;LHandProLib sdk;
  std::function<bool(const unsigned char*,unsigned)> send;
  std::function<bool(unsigned,unsigned char,int,unsigned,unsigned*)> sdo;
  bool configured=false,initialized=false,motor_touched=false,passed=false,profile_passed=false,opened=false,disabled=false;
  bool homing_active=false,homing_completed=false;int result=1,lockfd=-1;
  unsigned workspace_moves=0,workspace_enables=0;
  double joint_at=0,sensor_at=0,joint_published=0,tactile_published=0;uint64_t seq=0;std::array<int,6> pos{},status{},torque{},velocity{},current{},reached{};
  std::array<std::vector<float>,5> nf,tf,dir,proximity,pressure,sensor_x,sensor_y;
  auto feedback=[&](bool preparing_disabled=false){
    std::array<uint8_t,192> raw{};uint64_t next=0;double stamp=0;
    if(!master.getVerifiedInputs(1,raw.data(),raw.size(),next,stamp)||next<=seq||now()-stamp>=.08)return;
    seq=next;check(sdk.set_tpdo_data_decode(raw.data(),raw.size()));
    if(raw[0]==0&&raw[1]==27){
      bool expected_stop=false;
      for(int i=0;i<6;i++){
        int s=raw[164+i];
        if(preparing_disabled&&(s>>5)==6)expected_stop=true;
        else if(s==0 || ((s>>5)>1 && !(homing_active && (s>>5)==7))){std::cout<<"{\"event\":\"rejected_status\",\"axis\":"<<i+1<<",\"status\":"<<s<<",\"raw_hex\":\""<<hex(raw.data(),192)<<"\"}"<<std::endl;throw std::runtime_error("Axis alarm or invalid status");}
      }
      // stop_motors() revokes its stop command asynchronously. A disabled
      // emergency-stop sample is expected here but cannot qualify readiness.
      if(expected_stop){std::cout<<"{\"event\":\"preparation_stop_pending\",\"sequence\":"<<seq<<"}"<<std::endl;return;}
      for(int i=0;i<6;i++){
        int rawpos=int16_t(uint16_t(raw[2+6*i])|uint16_t(raw[3+6*i])<<8);
        check(sdk.get_now_position(i+1,&pos[i]));require(pos[i]==std::clamp(rawpos,0,10000),"SDK/raw position mismatch");
        status[i]=raw[164+i];
        check(sdk.get_now_current(i+1,&current[i]));check(sdk.get_now_position_velocity(i+1,&velocity[i]));
        check(sdk.get_torque_reached(i+1,&torque[i]));check(sdk.get_position_reached(i+1,&reached[i]));
      }
      joint_at=stamp;
      if(workspace && stamp-joint_published<.02)return;
      joint_published=stamp;std::ostringstream out;out<<std::fixed<<std::setprecision(9);
      out<<"{\"event\":\"sample\",\"valid\":true,\"wkc\":1,\"decode_result\":0,\"sequence\":"<<seq<<",\"received_monotonic\":"<<stamp<<",\"raw_hex\":\""<<hex(raw.data(),192)<<"\",\"positions\":";array_json(pos,out);
      out<<",\"raw_positions\":[";for(int i=0;i<6;i++){if(i)out<<',';out<<int16_t(uint16_t(raw[2+6*i])|uint16_t(raw[3+6*i])<<8);}out<<"],\"axis_status\":";array_json(status,out);
      out<<",\"currents\":";array_json(current,out);out<<",\"velocities\":";array_json(velocity,out);out<<",\"torque_reached\":";array_json(torque,out);out<<",\"position_reached\":";array_json(reached,out);out<<"}";
      publish_frame(out,workspace);
    }else if(raw[0]==0x40&&raw[1]==6){
      for(int i=0;i<5;i++){int id=1+2*i;nf[i]=forces(sdk,id,&LHandProLib::get_finger_normal_force_ex);tf[i]=forces(sdk,id,&LHandProLib::get_finger_tangential_force_ex);dir[i]=forces(sdk,id,&LHandProLib::get_finger_force_direction_ex);proximity[i]=optional_sensor(sdk,id,&LHandProLib::get_finger_proximity_ex);pressure[i]=optional_sensor(sdk,id,&LHandProLib::get_finger_pressure);require(nf[i].size()==tf[i].size()&&nf[i].size()==dir[i].size(),"Force layout mismatch");}
      sensor_at=stamp;
      if(workspace && stamp-tactile_published<.02)return;
      tactile_published=stamp;std::ostringstream out;out<<std::fixed<<std::setprecision(9);
      out<<"{\"event\":\"tactile\",\"sequence\":"<<seq<<",\"received_monotonic\":"<<stamp<<",\"raw_hex\":\""<<hex(raw.data(),192)<<"\",\"fingers\":[";
      for(int i=0;i<5;i++){if(i)out<<',';out<<"{\"normal_n\":";array_json(nf[i],out);out<<",\"tangent_n\":";array_json(tf[i],out);out<<",\"direction_deg\":";array_json(dir[i],out);out<<",\"proximity\":";array_json(proximity[i],out);out<<",\"pressure_unit\":\"normalized\",\"pressure\":";array_json(pressure[i],out);out<<",\"pressure_xy\":[";for(size_t j=0;j<sensor_x[i].size();j++){if(j)out<<',';out<<'['<<sensor_x[i][j]<<','<<sensor_y[i][j]<<']';}out<<"]}";}out<<"]}";
      publish_frame(out,workspace);
    }
  };
  try{
    int speed=std::stoi(argv[4]),cap=std::stoi(argv[5]);double limit=std::stod(argv[6]);std::array<int,6> open{},closed{};
    require(workspace||mode=="monitor"||mode=="home"||mode=="control"||mode=="manual"||mode=="acceptance"||mode=="acceptance_profile"||mode=="open"||thumb_probe,"Bad mode");
    require(argc==(mode=="manual"?31:19),"Wrong mode parameter count");
    require(speed>=100&&speed<=2000&&cap>=1&&cap<=900&&std::isfinite(limit)&&limit>=.1&&limit<=20,"Uncommissioned speed/current or invalid force limit");
    for(int i=0;i<6;i++){open[i]=std::stoi(argv[7+i]);closed[i]=std::stoi(argv[13+i]);require(open[i]>=0&&closed[i]>=open[i]&&closed[i]<=10000,"Invalid endpoints");}
    std::array<int,6> axis_speed{},axis_current{};axis_speed.fill(speed);axis_current.fill(cap);
    if(mode=="manual")for(int i=0;i<6;i++){axis_speed[i]=std::stoi(argv[19+i]);axis_current[i]=std::stoi(argv[25+i]);require(axis_speed[i]>=100&&axis_speed[i]<=speed&&axis_current[i]>=1&&axis_current[i]<=cap,"Invalid per-axis speed/current");}
    lockfd=::open(".cache/operator/hand.lock",O_CREAT|O_RDWR|O_NOFOLLOW,0600);require(lockfd>=0&&flock(lockfd,LOCK_EX|LOCK_NB)==0,"DH116 owner lock unavailable");
    if(geteuid()==0&&getenv("PKEXEC_UID"))fchown(lockfd,std::stoi(getenv("PKEXEC_UID")),-1);
    verifier_open=ecx_init(&verifier,argv[2]);require(verifier_open,"Cannot open register verification socket");
    require(verify_init(verifier),"Expected one slave in INIT; refusing takeover");
    auto names=master.scanNetworkInterfaces();auto found=std::find(names.begin(),names.end(),argv[2]);require(found!=names.end(),"Interface not found");
    configured=true;require(master.init(found-names.begin())&&master.getSlaveCount()==1,"EtherCAT init/identity failed");
    require(master.verifyDh116Identity(1),"DH116 SII vendor/product/revision mismatch");
    require(master.getSlaveInputSize(1)==192&&master.getSlaveOutputSize(1)==64,"Wrong PDO map");require(master.start(),"Cannot enter OP");master.run();
    // Retain only already-open raw sockets; the Qt owner can always signal this process.
    if(geteuid()==0&&getenv("PKEXEC_UID")){
      auto uid=uid_t(std::stoul(getenv("PKEXEC_UID")));auto* account=getpwuid(uid);
      require(account&&setgroups(0,nullptr)==0&&setgid(account->pw_gid)==0&&setuid(uid)==0,"Privilege drop failed");
    }
    int model=-1,side=-1,missing=0;bool bad=false;
    send=[&](const unsigned char* d,unsigned n){bool ok=master.setSlaveOutputs(1,d,n);if(!workspace)std::cout<<"{\"event\":\"rpdo_queued\",\"at\":"<<now()<<",\"accepted\":"<<(ok?"true":"false")<<",\"raw_hex\":\""<<hex(d,n)<<"\"}"<<std::endl;return ok;};sdk.set_send_rpdo_callback_ex(&send);
    sdo=[&](unsigned idx,unsigned char sub,int write,unsigned value,unsigned* out){bool ok=write?master.sdoWrite(idx,sub,value,1):master.sdoRead(idx,sub,out,1);if(ok&&!write&&idx==0x200e){if(sub==0x1d)model=*out;if(sub==0x1e)side=*out;}if(!ok){if(!write&&idx==0x2030&&(sub==5||sub==6)&&master.getLastError().find("06020000")!=std::string::npos)missing|=sub==5?1:2;else bad=true;}return ok;};sdk.set_ecat_sdo_callback_ex(&sdo);
    int rc=sdk.initial(LCN_ECAT);initialized=true;require(model==0&&side==0&&!bad&&(rc==0||(rc==LER_COMM_RECV&&missing==3)),"DH116 right-hand identity/SDK init failed");
    // Current format 0 still provides NF/TF/DIR in ECAT 0x40; pressure channels stay unitless.
    int order[]={1,3,5,7,9,11};check(sdk.set_sensor_order(order,6));check(sdk.set_sensor_data_format(0));check(sdk.set_sensor_enable(1));
    // Read static SDK channel coordinates once; display-only failure never invents a layout.
    for(int i=0;i<5;i++){
      int n=0,id=1+2*i;if(sdk.get_finger_sensor_pos(id,nullptr,nullptr,&n)||n<=0||n>128)continue;
      sensor_x[i].resize(n);sensor_y[i].resize(n);
      bool valid=sdk.get_finger_sensor_pos(id,sensor_x[i].data(),sensor_y[i].data(),&n)==0&&n==int(sensor_x[i].size());
      for(int j=0;valid&&j<n;j++)valid=std::isfinite(sensor_x[i][j])&&std::isfinite(sensor_y[i][j])&&sensor_x[i][j]>=0&&sensor_x[i][j]<=1&&sensor_y[i][j]>=0&&sensor_y[i][j]<=1;
      if(!valid){sensor_x[i].clear();sensor_y[i].clear();}
    }
    std::cout<<std::fixed<<std::setprecision(9);
    for(double begin=now();running&&now()-begin<3&&(joint_at==0||sensor_at==0);){feedback();usleep(5000);}
    require(running&&joint_at>0&&sensor_at>0,"Missing joint/tactile feedback");
    for(int i=1;i<=6;i++){int en=-1;check(sdk.get_enable(i,&en));require(en==0,"Device already enabled; refusing takeover");}
    auto prepare_disabled=[&](){
      // SDK stop_motors deliberately blocks for 100 ms before revoking its stop bit.
      // A workspace may have been monitoring for a while before the first enable.
      for(int s:status)require(!(s&1),"Device already enabled; refusing takeover");
      // Reacquire both streams after the disabled stop preparation.
      motor_touched=true;check(sdk.stop_motors(0));double stopped_at=now();
      while(running&&now()-stopped_at<1&&(joint_at<=stopped_at||sensor_at<=stopped_at)){feedback(true);usleep(5000);}
      require(joint_at>stopped_at&&sensor_at>stopped_at&&grasp::fresh(joint_at,now())&&grasp::fresh(sensor_at,now()),"No fresh feedback after disabled preparation");
    };
    if(mode!="monitor" && !workspace)prepare_disabled();
    std::cout<<"{\"event\":\"ready\",\"mode\":\""<<mode<<"\"}"<<std::endl;
    auto healthy=[&](){require(running&&getppid()==parent&&std::cout.good(),"Stop or owner disappeared");require(master.isConnected()&&now()-joint_at<.08&&now()-sensor_at<.08,"Joint/tactile feedback lost");};
    auto force_ok=[&](){grasp::force_limit(nf,limit);};
    auto enable=[&](){healthy();force_ok();for(int s:status)require(s&16,"Homing required; automatic homing is disabled");
      if(workspace&&!motor_touched){prepare_disabled();healthy();force_ok();}
      motor_touched=true;for(int i=0;i<6;i++){check(sdk.set_max_current(i+1,axis_current[i]));check(sdk.set_position_velocity(i+1,axis_speed[i]));}check(sdk.apply_motion_params());check(sdk.sync_position_to_target(0));check(sdk.set_enable(thumb_probe?2:0,1));
      double begin=now();bool enabled=false;while(now()-begin<1){feedback();healthy();enabled=true;for(int i=1;i<=6;i++){int en=0;check(sdk.get_enable(i,&en));if(en!=(thumb_probe&&i!=2?0:1))enabled=false;}if(enabled)break;usleep(5000);}require(enabled,"Enable feedback not confirmed");
      if(!thumb_probe||mode=="thumb_probe_after_enable"){if(mode=="manual"||workspace)grasp::resend_axis_motion_params(sdk,axis_speed,axis_current);else grasp::resend_motion_params(sdk,speed,cap);feedback();healthy();force_ok();}
    };
    auto move=[&](const std::array<int,6>& target){healthy();force_ok();for(int i=0;i<6;i++){check(sdk.set_target_position(i+1,target[i]));check(sdk.set_position_velocity(i+1,speed));check(sdk.set_max_current(i+1,cap));}check(sdk.move_motors());};
    auto home_cycle=[&](){
      if(workspace&&!motor_touched)prepare_disabled();
      homing_completed=false;
      motor_touched=true;
      // The firmware uses its installed homing parameters. No positioning target is sent.
      healthy();force_ok();check(sdk.sync_position_to_target(0));healthy();check(sdk.set_enable(0,1));
      const double enable_at=now();
      do {
        feedback();healthy();force_ok();
        if (joint_at-enable_at>=1. && std::all_of(status.begin(),status.end(),[](int s){return s&1;})) break;
        require(now()-enable_at<1.5,"Enable feedback not confirmed");
        usleep(5000);
      } while(true);
      grasp::HomingCycle cycle(status);
      healthy();force_ok();homing_active=true;
      const double commanded_at=now();
      check(sdk.home_motors(0));
      std::cout<<"{\"event\":\"homing\"}"<<std::endl;
      double last_joint=commanded_at;
      while (now()-commanded_at<12) {
        feedback();healthy();force_ok();
        if (joint_at>last_joint) {
          last_joint=joint_at;
          if (cycle.completed(status)) {homing_completed=true;break;}
        }
        usleep(5000);
      }
      require(homing_completed,"Six-axis homing feedback not confirmed");
      homing_active=false;
    };
    if(workspace){
      auto disable=[&](){
        check(sdk.stop_motors(0));check(sdk.set_enable(0,0));double since=now();
        do {
          require(running&&getppid()==parent,"Stop or owner disappeared");feedback(true);
          if(joint_at>since&&sensor_at>since&&std::all_of(status.begin(),status.end(),[](int s){return !(s&1);}))break;
          require(now()-since<1,"Disable feedback not confirmed");usleep(3000);
        }while(true);
      };
      auto send_pose=[&](const auto& target,const auto& velocity,const auto& currents){
        healthy();force_ok();
        for(int i=0;i<6;++i){
          check(sdk.set_target_position(i+1,target[i]));check(sdk.set_position_velocity(i+1,velocity[i]));
          check(sdk.set_max_current(i+1,currents[i]));
        }
        ++workspace_moves;check(sdk.move_motors());
      };
      auto configure=[&](const grasp::Action& a){axis_speed=a.velocity;axis_current=a.manual_current;limit=a.hard_force;};
      grasp::HandWorkspace hand{pos,status,torque,nf,[&](){++workspace_enables;enable();},disable,
                                [&](){++workspace_enables;home_cycle();disable();},send_pose,configure,now};
      fcntl(STDIN_FILENO,F_SETFL,O_NONBLOCK);std::string pending;
      while(running&&getppid()==parent){
        feedback();healthy();char buf[4096];auto n=read(STDIN_FILENO,buf,sizeof(buf));
        require(n!=0,"Control pipe closed");if(n>0)pending.append(buf,n);require(pending.size()<16384,"Hand input queue overflow");
        size_t end;
        while((end=pending.find('\n'))!=std::string::npos){auto line=pending.substr(0,end);pending.erase(0,end+1);hand.line(line);}
        hand.tick();usleep(3000);
      }
    }else if(mode=="home"){home_cycle();
    }else if(thumb_probe){
      // Three degrees of thumb flexion, then return to zero. Other axes stay disabled.
      enable();auto target=pos;
      const auto values=mode=="thumb_open_combined"?std::vector<int>{0}:std::vector<int>{1000,0};
      for(int value:values){
        target[1]=value;
        std::cout<<"{\"event\":\"probe_step\",\"target\":"<<value<<"}"<<std::endl;
        if(mode=="thumb_probe_combined"||mode=="thumb_open_combined"){
          // Vendor ECAT worksheet row 34: 0x0B, six bytes per axis, speed / 10.
          healthy();force_ok();std::array<unsigned char,64> packet{};packet[0]=0x0b;packet[1]=6;
          for(int i=0;i<6;i++){int values[]={target[i],speed/10,cap};for(int j=0;j<3;j++){packet[2+6*i+2*j]=values[j]&255;packet[3+6*i+2*j]=(values[j]>>8)&255;}}
          require(send(packet.data(),packet.size()),"Combined diagnostic frame rejected");
        }else move(target);
        for(double begin=now();now()-begin<3;){feedback();healthy();force_ok();require(!torque[1],"Thumb torque reached during probe");usleep(5000);}
        std::cout<<"{\"event\":\"probe_result\",\"target\":"<<value<<",\"position\":"<<pos[1]<<",\"status\":"<<status[1]<<",\"position_reached\":"<<reached[1]<<"}"<<std::endl;
        require(std::abs(pos[1]-value)<=200&&reached[1]==1,"Thumb probe target not reached");
      }
    }else if(mode=="acceptance"||mode=="acceptance_profile"||mode=="open"){
      enable();std::array<int,6> target{};
      // Confirm open first; thumb's two axes close together, fingers remain closed cumulatively.
      // Recovery runs only the same bounded all-zero opening step, never closure.
      for(int step=-1;step<(mode=="open"?0:6);step++){
        target=mode=="acceptance_profile"?grasp::acceptance_target(step,open,closed):grasp::acceptance_target(step);
        std::cout<<"{\"event\":\"step\",\"step\":"<<step<<",\"targets\":";array_json(target);std::cout<<"}"<<std::endl;move(target);
        bool arrived=false;double settled=0;
        for(double begin=now();now()-begin<8;){feedback();healthy();force_ok();bool close=true;for(int i=0;i<6;i++){if(std::abs(pos[i]-target[i])>200)close=false;if(torque[i]&&std::abs(pos[i]-target[i])>200)throw std::runtime_error("Torque reached before position target");}if(close){if(!settled)settled=now();if(now()-settled>.3){arrived=true;break;}}else settled=0;usleep(5000);}
        require(arrived,"Acceptance target not reached within 8 seconds");
      }
      opened=true;passed=mode=="acceptance";profile_passed=mode=="acceptance_profile";
    }else if(mode=="manual"){
      // Explicit operator enable; sync_position_to_target prevents a startup jump.
      enable();auto last_target=pos;grasp::ManualInput input;input.received=now();
      fcntl(STDIN_FILENO,F_SETFL,O_NONBLOCK);std::string pending;
      std::cout<<"{\"event\":\"armed\",\"mode\":\"manual\"}"<<std::endl;
      while(running&&getppid()==parent&&std::cout.good()){
        feedback();healthy();force_ok();char buf[2048];auto n=read(STDIN_FILENO,buf,sizeof(buf));
        require(n!=0,"Manual control pipe closed");if(n>0)pending.append(buf,n);require(pending.size()<8192,"Manual input queue overflow");
        size_t end;while((end=pending.find('\n'))!=std::string::npos){
          auto line=pending.substr(0,end);pending.erase(0,end+1);
          if(input.consume(line,now(),open,closed,speed,cap)){
            require(input.velocity<=axis_speed[input.axis-1]&&input.current<=axis_current[input.axis-1],"Manual command exceeds configured axis limits");
            healthy();force_ok();check(sdk.set_target_position(input.axis,input.target));check(sdk.set_position_velocity(input.axis,input.velocity));check(sdk.set_max_current(input.axis,input.current));check(sdk.move_motors(input.axis));last_target[input.axis-1]=input.target;
            std::cout<<"{\"event\":\"manual_target\",\"axis\":"<<input.axis<<",\"target\":"<<input.target<<",\"velocity\":"<<input.velocity<<",\"current\":"<<input.current<<"}"<<std::endl;
          }
        }
        require(grasp::fresh(input.received,now()),"Manual heartbeat stale; stop latched");
        for(int i=0;i<6;i++)require(!(torque[i]&&std::abs(pos[i]-last_target[i])>200),"Torque reached before manual position target");
        usleep(5000);
      }
    }else{
      fcntl(STDIN_FILENO,F_SETFL,O_NONBLOCK);std::string pending;bool armed=false;grasp::Input input;double last_move=0;std::array<int,6> last_target{};double begin=now();
      while(running&&getppid()==parent&&std::cout.good()){
        feedback();healthy();if(mode=="control"){
          char buf[2048];auto n=read(STDIN_FILENO,buf,sizeof(buf));if(n==0)throw std::runtime_error("Control pipe closed");if(n>0)pending.append(buf,n);require(pending.size()<8192,"Control queue overflow");
          size_t end;while((end=pending.find('\n'))!=std::string::npos){std::string line=pending.substr(0,end);pending.erase(0,end+1);
            if(!input.consume(line,now(),armed))continue;
            if(!armed){enable();armed=true;last_target=pos;std::cout<<"{\"event\":\"armed\"}"<<std::endl;}
            force_ok();auto target=grasp::interpolate(open,closed,input.trigger);
            if(target!=last_target&&now()-last_move>=.03){move(target);last_target=target;last_move=now();}
          }
          if(armed){require(grasp::fresh(input.received,now()),"Trigger input stale; stop latched");force_ok();for(int i=0;i<6;i++)require(!(torque[i]&&std::abs(pos[i]-last_target[i])>200),"Torque reached before position target");}else require(now()-begin<120,"No released trigger within 120 seconds");
        }usleep(5000);
      }
    }
    result=0;
  }catch(const std::exception& e){std::cout<<"{\"event\":\"error\",\"detail\":\""<<e.what()<<"\"}"<<std::endl;}
  int stop_rc=0,disable_rc=0;
  if(initialized&&motor_touched){stop_rc=sdk.stop_motors(0);disable_rc=sdk.set_enable(0,0);double start=now();uint64_t before=seq;while(now()-start<.4){try{feedback();}catch(...){}usleep(5000);}disabled=seq>before&&now()-joint_at<.08;for(int i=1;i<=6;i++){int en=-1;if(sdk.get_enable(i,&en)||en!=0)disabled=false;}}
  if(initialized){sdk.set_sensor_enable(0);sdk.close();}if(configured)master.stop();
  bool init=configured&&verifier_open&&verify_init(verifier);
  if(verifier_open)ecx_close(&verifier);
  bool cleanup=init&&(!motor_touched||(disabled&&stop_rc==0&&disable_rc==0));
  const bool homed_after_disable=std::all_of(status.begin(),status.end(),[](int s){return (s&0x11)==0x10;});
  const bool home_ok=homing_completed && homed_after_disable && running && result==0 && cleanup;
  if(mode=="home" && !home_ok)result=1;
  std::cout<<"{\"event\":\"closed\",\"init_confirmed\":"<<(init?"true":"false")<<",\"disabled\":"<<(disabled?"true":"false")<<",\"stop_rc\":"<<stop_rc<<",\"disable_rc\":"<<disable_rc<<",\"homing_passed\":"<<(home_ok?"true":"false")<<",\"open_passed\":"<<(opened&&cleanup?"true":"false")<<",\"profile_motion_passed\":"<<(profile_passed&&cleanup?"true":"false")<<",\"motion_passed\":"<<(passed&&cleanup?"true":"false")<<"}"<<std::endl;
  if(workspace)std::cout<<"{\"event\":\"workspace_summary\",\"motor_touched\":"<<(motor_touched?"true":"false")<<",\"motion_commands\":"<<workspace_moves<<",\"enable_requests\":"<<workspace_enables<<"}"<<std::endl;
  if(lockfd>=0)close(lockfd);return cleanup?result:1;
}
