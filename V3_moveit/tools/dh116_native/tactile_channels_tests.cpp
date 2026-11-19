// Real vendor SDK with synthetic PDO bytes. No sockets, no device connection.
#include "LHandProLib.hpp"
#include <array>
#include <cassert>
#include <cmath>
#include <functional>
#include <iostream>
#include <vector>
int main(){
  lhplib::LHandProLib sdk;std::function<bool(const unsigned char*,unsigned)> sink=[](auto*,auto){return true;};sdk.set_send_rpdo_callback_ex(&sink);
  assert(sdk.initial(lhplib::LCN_ECAT)==0);assert(sdk.set_hand_type(lhplib::LAC_DOF_6)==0);int order[]={1,3,5,7,9,11};assert(sdk.set_sensor_order(order,6)==0);assert(sdk.set_sensor_data_format(0)==0);
  std::array<unsigned char,192> raw{};raw[0]=64;raw[1]=6;for(int i=2;i<192;i++)raw[i]=i;assert(sdk.set_tpdo_data_decode(raw.data(),192)==0);
  for(int finger=0;finger<5;finger++){
    int id=1+finger*2,n=0,count=0;assert(sdk.get_finger_sensor_pos(id,nullptr,nullptr,&n)==0);assert(n==(finger==0||finger==4?5:9));std::vector<float>x(n),y(n),pressure(n);
    assert(sdk.get_finger_sensor_pos(id,x.data(),y.data(),&n)==0);assert(sdk.get_finger_pressure(id,nullptr,&count)==0&&count==n);assert(sdk.get_finger_pressure(id,pressure.data(),&count)==0);
    for(int j=0;j<n;j++){assert(x[j]>=0&&x[j]<=1&&y[j]>=0&&y[j]<=1);assert(std::abs(pressure[j]-raw[2+30*finger+j]/255.f)<1e-6);}
    assert(std::abs(x[0]-.5f)<1e-6);assert(std::abs(y[0]-(n==5?.18f:.08f))<1e-6);
  }
  std::cout<<"PASS five SDK layouts (5/9/9/9/5 channels), normalized pressure and raw byte correspondence; no hardware\n";
}
