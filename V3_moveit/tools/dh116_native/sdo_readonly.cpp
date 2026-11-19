// Read manufacturer object names/values in PRE_OP. No RPDO or motor commands.
#include <soem/soem.h>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <sys/file.h>
#include <unistd.h>
#include <chrono>
static ecx_contextt ctx{};
static volatile sig_atomic_t running=1;
static void stop(int){running=0;}
static void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main(int argc,char** argv){
  if((argc!=3&&argc!=4)||std::string(argv[1])!="--hardware"||(argc==4&&std::string(argv[3])!="--thumb-params"))return 2;
  signal(SIGTERM,stop);signal(SIGINT,stop);
  int lockfd=-1,result=1;bool opened=false,configured=false,clean=false;
  try{
    lockfd=open(".cache/operator/hand.lock",O_RDWR|O_NOFOLLOW);
    require(lockfd>=0&&flock(lockfd,LOCK_EX|LOCK_NB)==0,"Another hand owner or no lock file");
    opened=ecx_init(&ctx,argv[2]);require(opened,"Cannot open raw socket");
    uint16_t state=0;require(ecx_BRD(&ctx.port,0,ECT_REG_ALSTAT,2,&state,20000)==1&&etohs(state)==EC_STATE_INIT,"Expected one slave in INIT");
    configured=true;require(ecx_config_init(&ctx)==1,"Expected one slave");
    require(ctx.slavelist[1].eep_id==0x00660000&&ctx.slavelist[1].eep_rev==1,"Wrong identity");
    require(ecx_statecheck(&ctx,1,EC_STATE_PRE_OP,500000)==EC_STATE_PRE_OP,"PRE_OP not confirmed");
    if(argc==4){
      // Exact axis-2 addresses from the official v1.4.5 params_sdo_address.csv.
      const uint16_t indices[]={0x2059,0x2059,0x2059,0x2048,0x2057,0x2057,0x204c,0x2062,0x9103};
      const uint8_t subs[]={1,3,4,0x18,1,2,0x12,0x0c,2};
      for(int i=0;i<9&&running;i++){
        uint32_t value=0;int size=sizeof(value);int wkc=ecx_SDOread(&ctx,1,indices[i],subs[i],FALSE,&size,&value,EC_TIMEOUTRXM);
        std::cout<<"TARGETED "<<std::hex<<indices[i]<<':'<<int(subs[i])<<std::dec<<" wkc="<<wkc<<" bytes="<<size<<" value="<<value<<std::endl;
        while(ctx.ecaterror)std::cout<<ecx_elist2string(&ctx)<<std::endl;
      }
    }else{
    auto od=std::make_unique<ec_ODlistt>();auto oe=std::make_unique<ec_OElistt>();
    require(ecx_readODlist(&ctx,1,od.get())>0,"Device does not provide object dictionary");
    auto start=std::chrono::steady_clock::now();
    for(int i=0;i<od->Entries&&running;i++){
      require(std::chrono::steady_clock::now()-start<std::chrono::seconds(50),"Dictionary read time bound");
      auto index=od->Index[i];
      if(!((index>=0x2000&&index<0x3000)||(index>=0x9100&&index<=0x9106)))continue;
      require(ecx_readODdescription(&ctx,i,od.get())>0,"Object description failed");
      std::cout<<"OBJECT "<<std::hex<<index<<std::dec<<' '<<od->Name[i]<<" maxsub="<<int(od->MaxSub[i])<<std::endl;
      memset(oe.get(),0,sizeof(*oe));require(ecx_readOE(&ctx,i,od.get(),oe.get())>0,"Entry description failed");
      for(int sub=0;sub<=od->MaxSub[i]&&running;sub++){
        if(!oe->BitLength[sub])continue;
        std::cout<<"ENTRY "<<std::hex<<index<<':'<<sub<<std::dec<<" bits="<<oe->BitLength[sub]<<" access="<<oe->ObjAccess[sub]<<' '<<oe->Name[sub];
        if((oe->ObjAccess[sub]&7)&&oe->BitLength[sub]<=32){
          uint32_t value=0;int size=sizeof(value);int wkc=ecx_SDOread(&ctx,1,index,sub,FALSE,&size,&value,EC_TIMEOUTRXM);
          std::cout<<" read_wkc="<<wkc<<" bytes="<<size<<" value="<<value;
        }
        std::cout<<std::endl;
      }
    }
    }
    require(running,"Stopped");result=0;
  }catch(const std::exception& e){std::cout<<"ERROR "<<e.what()<<std::endl;}
  if(configured){ctx.slavelist[0].state=EC_STATE_INIT;ecx_writestate(&ctx,0);clean=ecx_statecheck(&ctx,0,EC_STATE_INIT,500000)==EC_STATE_INIT;}
  if(opened)ecx_close(&ctx);if(lockfd>=0)close(lockfd);
  std::cout<<"CLOSED init_confirmed="<<clean<<" motor_commands=0 pdo_writes=0"<<std::endl;
  return clean?result:1;
}
