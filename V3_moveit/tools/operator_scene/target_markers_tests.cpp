#include "target_markers.hpp"
#include <cassert>
#include <iostream>
int main() {
  using namespace target_display;
  QJsonObject data{{"frame","base_link"},{"source","/v3/hand_target"},{"session_started",10.},
    {"updated",10.1},{"active",true},{"target",QJsonObject{{"received_monotonic",10.1},
    {"position_m",QJsonArray{.2,-.1,.8}},{"quaternion_xyzw",QJsonArray{0,0,std::sqrt(.5),std::sqrt(.5)}}}}};
  data["trail"]=QJsonArray{
    QJsonObject{{"received_monotonic",10.},{"segment",1},{"position_m",QJsonArray{.1,-.1,.8}}},
    QJsonObject{{"received_monotonic",10.1},{"segment",1},{"position_m",QJsonArray{.2,-.1,.8}}}};
  auto rendered=make(data,10.1);int arrows=0,lines=0;double alpha=0;
  for(const auto& m:rendered.markers) {
    if(m.action==Marker::ADD&&m.type==Marker::ARROW){
      arrows++;assert(m.header.frame_id=="base_link");
      if(m.id==0){assert(std::abs(m.points[1].x-.2)<1e-9);assert(std::abs(m.points[1].y-.02)<1e-9);}
    }
    if(m.type==Marker::LINE_LIST){lines++;alpha=m.color.a;assert(m.points.size()==2);}
  }
  assert(arrows==3&&lines==1&&alpha>.9);
  auto old=make(data,14.8);assert(old.markers.size()==2);assert(old.markers[1].color.a<.02);
  assert(make(data,15.1).markers.size()==1);assert(make(data,10.1,11.).markers.size()==1);
  data["active"]=false;assert(make(data,10.1).markers.size()==2);
  auto trail=data["trail"].toArray();auto second=trail[1].toObject();second["segment"]=2;trail[1]=second;data["trail"]=trail;
  assert(make(data,10.1).markers.size()==1);
  std::cout<<"PASS: target axes, quaternion rotation, base frame, age colour/alpha, expiry and clutch breaks\n";
}
