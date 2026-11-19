// Feedback-only connection: SAFE_OP + physical input reads, never OP or RPDO.
#include <soem/soem.h>
#include <LHandProLib/LHandProLib.hpp>
#include <array>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <pwd.h>
#include <grp.h>
#include <algorithm>
#include <sys/file.h>
#include <fcntl.h>

static ecx_contextt ctx{};
static std::array<unsigned char,4096> iomap{};
static volatile sig_atomic_t running=1;
static void stop(int){running=0;}
static double now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static std::string hex(const unsigned char* p,size_t n){std::ostringstream o;for(size_t i=0;i<n;i++)o<<std::hex<<std::setfill('0')<<std::setw(2)<<unsigned(p[i]);return o.str();}
static int16_t i16(const unsigned char* p){return int16_t(uint16_t(p[0]) | uint16_t(p[1])<<8);}
static unsigned blocked_sends=0;
static bool reject_send(const unsigned char*,unsigned){++blocked_sends;return false;}

int main(int argc,char**argv){
  if(argc!=3){std::cerr<<"Usage: v3_dh116_feedback INTERFACE SECONDS (0=until disconnect, max 60)\n";return 2;}
  const int duration=std::atoi(argv[2]);if(duration<0||duration>60)return 2;
  const pid_t owner=getppid();signal(SIGPIPE,SIG_IGN);
  signal(SIGINT,stop);signal(SIGTERM,stop);
  bool opened=false,configured=false;int result=1,lockfd=-1;
  try{
    if(!ecx_init(&ctx,argv[1]))throw std::runtime_error("Cannot open EtherCAT socket");opened=true;
    if(geteuid()==0&&getenv("PKEXEC_UID")){
      auto uid=uid_t(std::stoul(getenv("PKEXEC_UID")));auto* pw=getpwuid(uid);
      if(!pw||setgroups(0,nullptr)||setgid(pw->pw_gid)||setuid(uid))throw std::runtime_error("Privilege drop failed");
    }
    lockfd=open(".cache/operator/hand.lock",O_CREAT|O_RDWR|O_NOFOLLOW,0600);
    if(lockfd<0)throw std::runtime_error("Cannot open DH116 lock: check workspace cwd and permissions");
    if(flock(lockfd,LOCK_EX|LOCK_NB))throw std::runtime_error("Another DH116 connection owns the lock");
    uint16_t state=0;
    if(ecx_BRD(&ctx.port,0,ECT_REG_ALSTAT,2,&state,20000)!=1)throw std::runtime_error("Expected exactly one responding slave");
    if((etohs(state)&0xf)!=EC_STATE_INIT)throw std::runtime_error("Slave is not INIT; refusing takeover of another connection");
    configured=true;
    if(ecx_config_init(&ctx)!=1)throw std::runtime_error("Expected exactly one configured slave");
    auto& slave=ctx.slavelist[1];
    std::cout<<"{\"event\":\"identity\",\"name\":\""<<slave.name<<"\",\"vendor\":"<<slave.eep_man<<",\"product\":"<<slave.eep_id<<",\"revision\":"<<slave.eep_rev<<"}"<<std::endl;
    if(slave.eep_id!=0x00660000||slave.eep_rev!=1)throw std::runtime_error("Slave identity differs from manufacturer DH116 example");
    ecx_config_map_group(&ctx,iomap.data(),0);
    if(slave.Ibytes!=192||slave.Obytes!=64)throw std::runtime_error("Unexpected DH116 PDO map size");
    slave.state=EC_STATE_SAFE_OP;ecx_writestate(&ctx,1);
    if(ecx_statecheck(&ctx,1,EC_STATE_SAFE_OP,2000000)!=EC_STATE_SAFE_OP)throw std::runtime_error("SAFE_OP not reached");
    auto input_addr=etohs(slave.SM[3].StartAddr);
    if(input_addr!=0x1400||etohs(slave.SM[3].SMlength)!=192)throw std::runtime_error("Unexpected input SyncManager");
    lhplib::LHandProLib sdk;
    // Select the SDK's EtherCAT decoder but forbid every outgoing SDK message.
    // initial() returns COMM_SEND because its frame-schedule request is denied.
    sdk.set_send_rpdo_callback(reject_send);
    int init_result=sdk.initial(lhplib::LCN_ECAT);
    std::cout<<"{\"event\":\"decoder\",\"sdk_init_result\":"<<init_result<<",\"send_policy\":\"deny_all\"}"<<std::endl;
    unsigned samples=0,good=0;
    for(double start=now();running&&getppid()==owner&&std::cout.good()&&(!duration||now()-start<duration);std::this_thread::sleep_for(std::chrono::milliseconds(20))){
      std::array<unsigned char,192> input;input.fill(0xcc);
      int wkc=ecx_FPRD(&ctx.port,slave.configadr,input_addr,input.size(),input.data(),20000);
      bool valid=wkc==1&&((input[0]==0&&input[1]==27)||(input[0]==3&&input[1]==12)||(input[0]==6&&input[1]==6));
      int decode=valid?sdk.set_tpdo_data_decode(input.data(),input.size()):-1;
      std::array<int,6> positions{};std::array<int,6> codes{};
      for(int i=0;i<6;i++){
        positions[i]=-987654;codes[i]=valid?sdk.get_now_position(i+1,&positions[i]):-1;
        valid=valid&&decode==0&&codes[i]==0&&positions[i]==std::clamp(int(i16(input.data()+2+6*i)),0,10000);
      }
      ++samples;if(valid)++good;
      std::cout<<"{\"event\":\"sample\",\"sequence\":"<<samples<<",\"received_monotonic\":"<<std::fixed<<std::setprecision(9)<<now()<<",\"wkc\":"<<wkc<<",\"valid\":"<<(valid?"true":"false")<<",\"decode_result\":"<<decode<<",\"raw_hex\":\""<<hex(input.data(),input.size())<<"\",\"positions\":[";
      for(int i=0;i<6;i++){if(i)std::cout<<',';std::cout<<positions[i];}
      std::cout<<"],\"raw_positions\":[";
      for(int i=0;i<6;i++){if(i)std::cout<<',';std::cout<<i16(input.data()+2+6*i);}
      std::cout<<"],\"axis_status\":[";
      int capacity=input[0]==0?27:input[0]==3?12:6;
      for(int i=0;i<6;i++){if(i)std::cout<<',';std::cout<<unsigned(input[2+capacity*6+i]);}
      std::cout<<"]}"<<std::endl;
    }
    result=good>=20?0:1;
    sdk.close();
    std::cout<<"{\"event\":\"summary\",\"samples\":"<<samples<<",\"verified_samples\":"<<good<<",\"sdk_sends_blocked\":"<<blocked_sends<<",\"motor_commands\":0,\"pdo_writes\":0}"<<std::endl;
  }catch(const std::exception&e){std::cout<<"{\"event\":\"error\",\"detail\":\""<<e.what()<<"\"}"<<std::endl;}
  bool clean=!configured;
  if(configured){ctx.slavelist[0].state=EC_STATE_INIT;ecx_writestate(&ctx,0);clean=ecx_statecheck(&ctx,0,EC_STATE_INIT,500000)==EC_STATE_INIT;}
  if(opened)ecx_close(&ctx);
  if(lockfd>=0)close(lockfd);
  std::cout<<"{\"event\":\"closed\",\"init_confirmed\":"<<(configured&&clean?"true":"false")<<"}"<<std::endl;
  return clean?result:1;
}
