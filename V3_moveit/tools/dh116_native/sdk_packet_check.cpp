// Offline vendor-SDK packet experiment: no sockets or hardware.
#include "LHandProLib.hpp"
#include "grasp_sdk_motion.hpp"
#include <array>
#include <cassert>
#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>
#include <vector>
int main(){
  lhplib::LHandProLib sdk;const char* phase="initial";std::vector<std::vector<unsigned char>> frames;
  std::function<bool(const unsigned char*,unsigned)> send=[&](const unsigned char* p,unsigned n){
    frames.emplace_back(p,p+n);std::cout<<phase<<" rpdo=";
    for(unsigned i=0;i<n;i++)std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(p[i]);
    std::cout<<std::dec<<std::endl;return true;
  };
  sdk.set_send_rpdo_callback_ex(&send);assert(sdk.initial(lhplib::LCN_ECAT)==0);
  assert(sdk.set_hand_type(lhplib::LAC_DOF_6)==0);
  std::array<unsigned char,192> raw{};raw[1]=27;raw[8]=223;
  for(int i=0;i<6;i++)raw[164+i]=6;
  assert(sdk.set_tpdo_data_decode(raw.data(),raw.size())==0);
  phase="sync_unhomed";assert(sdk.sync_position_to_target(0)==0);
  phase="enable_for_home";assert(sdk.set_enable(0,1)==0);
  for(int i=0;i<6;i++)raw[164+i]=7;
  assert(sdk.set_tpdo_data_decode(raw.data(),raw.size())==0);
  phase="home";const auto before_home=frames.size();
  assert(sdk.home_motors(0)==0 && frames.size()==before_home+2);
  for(int step=0;step<2;step++){
    const auto& frame=frames[before_home+step];
    assert(frame.size()==64 && frame[0]==1 && frame[1]==6);
    for(int axis=0;axis<6;axis++)assert(frame[2+axis*2]==0x25 && frame[3+axis*2]==(step?4:0));
  }
  phase="disable_after_home";assert(sdk.set_enable(0,0)==0);
  for(int i=0;i<6;i++)raw[164+i]=22;
  assert(sdk.set_tpdo_data_decode(raw.data(),raw.size())==0);
  phase="stop";assert(sdk.stop_motors(0)==0);
  phase="params";assert(sdk.set_position_velocity(0,2000)==0);assert(sdk.set_max_current(0,200)==0);assert(sdk.apply_motion_params()==0);
  phase="sync";assert(sdk.sync_position_to_target(0)==0);
  phase="enable";assert(sdk.set_enable(0,1)==0);
  for(int i=0;i<6;i++)raw[164+i]=23;
  assert(sdk.set_tpdo_data_decode(raw.data(),raw.size())==0);
  phase="refresh_after_enable";auto before=frames.size();
  grasp::resend_motion_params(sdk,2000,200);
  assert(frames.size()==before+2);
  for(int j=0;j<2;j++){
    const auto& frame=frames[before+j];assert(frame.size()==64&&frame[0]==3+j&&frame[1]==6);
    for(int i=0;i<6;i++)assert(frame[2+2*i]==200&&frame[3+2*i]==0);
  }
  phase="open";for(int i=1;i<=6;i++){assert(sdk.set_target_position(i,0)==0);assert(sdk.set_position_velocity(i,2000)==0);assert(sdk.set_max_current(i,200)==0);}
  assert(sdk.move_motors()==0);
  int target=-1,pos=-1,reached=-1;assert(sdk.get_target_position(2,&target)==0);assert(sdk.get_now_position(2,&pos)==0);assert(sdk.get_position_reached(2,&reached)==0);
  assert(target==0&&pos==223&&reached==1);
  std::cout<<"cached_target="<<target<<" supplied_position="<<pos<<" supplied_position_reached="<<reached<<" hardware_connections=0"<<std::endl;
  phase="single_axis_1000";assert(sdk.set_target_position(2,1000)==0);assert(sdk.move_motors(2)==0);
  phase="authorized_ceiling";grasp::resend_motion_params(sdk,2000,900);
  assert(frames.back()[0]==4&&frames.back()[2]==0x84&&frames.back()[3]==3);
  before=frames.size();bool rejected=false;
  try{grasp::resend_motion_params(sdk,2000,901);}catch(const std::runtime_error&){rejected=true;}
  assert(rejected&&frames.size()==before);
  phase="manual_axis_3";assert(sdk.set_target_position(3,700)==0);assert(sdk.set_position_velocity(3,1500)==0);assert(sdk.set_max_current(3,600)==0);assert(sdk.move_motors(3)==0);
  const auto& manual=frames.back();assert(manual[0]==2&&manual[1]==6);
  const int expected[]={0,1000,700,0,0,0};for(int i=0;i<6;i++)assert((manual[2+2*i]|int(manual[3+2*i])<<8)==expected[i]);
  phase="per_axis_enable_refresh";std::array<int,6> speeds{1000,1100,1200,1300,1400,1500},currents{200,300,400,500,600,700};before=frames.size();grasp::resend_axis_motion_params(sdk,speeds,currents);assert(frames.size()==before+2);
  for(int i=0;i<6;i++){assert((frames[before][2+2*i]|int(frames[before][3+2*i])<<8)==speeds[i]/10);assert((frames[before+1][2+2*i]|int(frames[before+1][3+2*i])<<8)==currents[i]);}
  before=frames.size();currents[2]=901;rejected=false;try{grasp::resend_axis_motion_params(sdk,speeds,currents);}catch(const std::runtime_error&){rejected=true;}assert(rejected&&frames.size()==before);
}
