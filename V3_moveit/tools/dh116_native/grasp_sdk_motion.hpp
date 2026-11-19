#pragma once
#include "LHandProLib.hpp"
#include <array>
#include <stdexcept>

namespace grasp {
inline void resend_axis_motion_params(lhplib::LHandProLib& sdk,const std::array<int,6>& speed,const std::array<int,6>& current){
  for(int i=0;i<6;i++)if(speed[i]<100||speed[i]>20000||current[i]<1||current[i]>900)throw std::runtime_error("Uncommissioned axis motion parameters");
  auto check=[](int rc){if(rc)throw std::runtime_error("SDK axis parameter refresh failed");};
  for(int i=0;i<6;i++){
    check(sdk.set_position_velocity(i+1,speed[i]==100?101:speed[i]-1));check(sdk.set_position_velocity(i+1,speed[i]));
    check(sdk.set_max_current(i+1,current[i]==1?2:current[i]-1));check(sdk.set_max_current(i+1,current[i]));
  }
  check(sdk.apply_motion_params());
}
// Resending settings after confirmed enable restored motion in the live
// 2026-11-14 comparison; firmware internals are not known. The SDK only sends
// dirty cached parameters. These
// setters dirty the cache without sending the intermediate values; apply sends
// precisely the requested bounded speed/current, after enable is confirmed.
inline void resend_motion_params(lhplib::LHandProLib& sdk,int speed,int current){
  // User explicitly authorized a 900 per-mille ceiling on 2026-11-14.
  if(speed<100||speed>2000||current<1||current>900)
    throw std::runtime_error("Uncommissioned motion parameters");
  auto check=[](int rc){if(rc)throw std::runtime_error("SDK motion parameter refresh failed");};
  check(sdk.set_position_velocity(0,speed==100?101:speed-1));
  check(sdk.set_position_velocity(0,speed));
  check(sdk.set_max_current(0,current==1?2:current-1));
  check(sdk.set_max_current(0,current));
  check(sdk.apply_motion_params());
}
}
