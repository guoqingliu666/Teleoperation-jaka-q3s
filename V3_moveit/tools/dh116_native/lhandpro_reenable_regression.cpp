// Offline SDK regression: outgoing frames go only to a local vector. No sockets.
#include "LHandProLib.hpp"
#include <cassert>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

int main(int argc,char**argv){
  if(argc!=3)return 2;
  bool fixed=std::string(argv[1])=="patched";
  assert(fixed||std::string(argv[1])=="original");
  std::ifstream f(argv[2],std::ios::binary);
  std::vector<unsigned char> raw((std::istreambuf_iterator<char>(f)),{});
  assert(raw.size()==192 && raw[0]==0 && raw[1]==27);
  std::vector<std::vector<unsigned char>> frames;
  bool send_ok=true;
  std::function<bool(const unsigned char*,unsigned)> send=[&](const unsigned char*p,unsigned n){
    frames.emplace_back(p,p+n);return send_ok;
  };
  lhplib::LHandProLib sdk;
  sdk.set_send_rpdo_callback_ex(&send);
  assert(sdk.initial(lhplib::LCN_ECAT)==0);
  assert(sdk.set_hand_type(lhplib::LAC_DOF_6)==0);
  assert(sdk.set_tpdo_data_decode(raw.data(),raw.size())==0);
  assert(sdk.sync_position_to_target(0)==0);
  assert(sdk.set_position_velocity(0,1000)==0);
  assert(sdk.set_max_current(0,600)==0);
  auto verify=[&](std::vector<int> types,int target){
    assert(frames.size()==types.size());
    for(size_t k=0;k<frames.size();k++){
      const auto& p=frames[k];assert(p.size()==64 && p[0]==types[k] && p[1]==6);
      if(p[0]==3||p[0]==4){
        for(int j=0;j<6;j++)assert((p[2+2*j]|p[3+2*j]<<8)==(p[0]==3?100:600));
      }
      if(p[0]==2){
        assert((p[6]|p[7]<<8)==target);
        for(int j=0;j<6;j++)if(j!=2)assert((p[2+2*j]|p[3+2*j]<<8)==(raw[2+6*j]|raw[3+6*j]<<8));
      }
    }
  };
  assert(sdk.set_enable(0,1)==0);frames.clear();
  assert(sdk.set_target_position(3,2947)==0);assert(sdk.move_motors(3)==0);
  verify({3,4,2},2947);frames.clear();
  assert(sdk.set_target_position(3,3347)==0);assert(sdk.move_motors(3)==0);
  verify({2},3347);
  // Repeated writes use unchanged settings, matching the live failure.
  for(int id:{0,3}){
    assert(sdk.set_enable(id,0)==0);assert(sdk.set_enable(id,1)==0);
    assert(sdk.set_position_velocity(0,1000)==0);assert(sdk.set_max_current(0,600)==0);
    frames.clear();assert(sdk.set_target_position(3,3147)==0);assert(sdk.move_motors(3)==0);
    verify(fixed?std::vector<int>{3,4,2}:std::vector<int>{2},3147);
    frames.clear();assert(sdk.set_target_position(3,3347)==0);assert(sdk.move_motors(3)==0);
    verify({2},3347);
  }
  frames.clear();assert(sdk.set_enable(-1,1)==1);assert(frames.empty());
  assert(sdk.set_enable(7,1)==1);assert(frames.empty());
  send_ok=false;assert(sdk.set_enable(0,1)!=0);
  assert(frames.size()==1 && frames[0][0]==1);
  send_ok=true;sdk.close();
  std::cout<<"PASS "<<argv[1]<<": first move, unchanged settings, all/single-axis re-enable, target preservation, invalid IDs, send failure; no hardware calls\n";
}
