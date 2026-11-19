#pragma once
#include <QJsonObject>
#include <QJsonArray>
#include <array>
#include <algorithm>
#include <cmath>
#include <visualization_msgs/msg/marker_array.hpp>

namespace target_display {
using Marker=visualization_msgs::msg::Marker;
using Point=geometry_msgs::msg::Point;
inline bool vector(const QJsonValue& value,int n) {
  const auto a=value.toArray();
  if(a.size()!=n)return false;
  for(const auto& v:a)if(!v.isDouble()||!std::isfinite(v.toDouble()))return false;
  return true;
}
inline Point point(const QJsonArray& p) {Point v;v.x=p[0].toDouble();v.y=p[1].toDouble();v.z=p[2].toDouble();return v;}
inline Marker marker(int id,int type) {
  Marker m;m.header.frame_id="base_link";m.ns="hand_target_display";m.id=id;m.type=type;m.action=Marker::ADD;
  m.pose.orientation.w=1.;m.color.a=1.;return m;
}
inline visualization_msgs::msg::MarkerArray make(const QJsonObject& data,double now,double session_floor=0) {
  visualization_msgs::msg::MarkerArray out;
  Marker erase;erase.action=Marker::DELETEALL;out.markers.push_back(erase);
  if(data["frame"].toString()!="base_link"||data["source"].toString()!="/v3/hand_target"||
     !data["session_started"].isDouble()||data["session_started"].toDouble()<session_floor)return out;
  // Fixed age buckets use marker-wide alpha; RViz versions differ in their
  // support for per-vertex alpha. Every sample expires at five seconds.
  std::array<Marker,20> trail;
  for(int i=0;i<20;i++) {
    auto m=marker(20+i,Marker::LINE_LIST);m.scale.x=.004;
    double age=(i+.5)/20.,young=1.-age;
    m.color.r=.15+.40*age;m.color.g=.88-.58*age;m.color.b=.75+.20*age;
    m.color.a=std::pow(young,1.5);trail[i]=m;
  }
  auto samples=data["trail"].toArray();
  for(int i=1;i<samples.size()&&i<750;i++) {
    auto a=samples[i-1].toObject(),b=samples[i].toObject();
    double ta=a["received_monotonic"].toDouble(-1),tb=b["received_monotonic"].toDouble(-1);
    if(!vector(a["position_m"],3)||!vector(b["position_m"],3)||!a["segment"].isDouble()||
       a["segment"]!=b["segment"]||now-ta>=5.||now-tb>=5.||now-ta<0||now-tb<0||tb<=ta||tb-ta>.15)continue;
    auto p=point(a["position_m"].toArray()),q=point(b["position_m"].toArray());
    if(std::hypot(std::hypot(p.x-q.x,p.y-q.y),p.z-q.z)<1e-6)continue;
    int bucket=std::clamp(int((now-(ta+tb)*.5)*4),0,19);
    trail[bucket].points.push_back(p);trail[bucket].points.push_back(q);
  }
  for(auto& m:trail)if(!m.points.empty())out.markers.push_back(m);
  auto target=data["target"].toObject();auto q=target["quaternion_xyzw"].toArray();
  double updated=data["updated"].toDouble(-1),received=target["received_monotonic"].toDouble(-1);
  if(!data["active"].toBool()||now-updated<0||now-updated>=.15||now-received<0||now-received>=.15||
     !vector(target["position_m"],3)||!vector(target["quaternion_xyzw"],4))return out;
  double x=q[0].toDouble(),y=q[1].toDouble(),z=q[2].toDouble(),w=q[3].toDouble();
  double norm=x*x+y*y+z*z+w*w;if(std::abs(norm-1.)>=.02)return out;
  double inv=1/std::sqrt(norm);x*=inv;y*=inv;z*=inv;w*=inv;
  const double r[3][3]={{1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)},
                        {2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)},
                        {2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)}};
  Point origin=point(target["position_m"].toArray());
  const char* names[]={"X","Y","Z"};const double colors[3][3]={{1.,.20,.22},{.22,1.,.40},{.25,.55,1.}};
  for(int i=0;i<3;i++) {
    auto arrow=marker(i,Marker::ARROW);arrow.scale.x=.004;arrow.scale.y=.008;arrow.scale.z=.018;
    arrow.color.r=colors[i][0];arrow.color.g=colors[i][1];arrow.color.b=colors[i][2];
    Point tip=origin;tip.x+=.12*r[0][i];tip.y+=.12*r[1][i];tip.z+=.12*r[2][i];arrow.points={origin,tip};out.markers.push_back(arrow);
    auto label=marker(4+i,Marker::TEXT_VIEW_FACING);label.pose.position=tip;label.pose.position.z+=.012;label.scale.z=.022;
    label.color=arrow.color;label.text=names[i];out.markers.push_back(label);
  }
  auto center=marker(3,Marker::SPHERE);center.pose.position=origin;center.scale.x=center.scale.y=center.scale.z=.014;
  center.color.r=.8;center.color.g=1.;center.color.b=1.;out.markers.push_back(center);
  auto label=marker(7,Marker::TEXT_VIEW_FACING);label.pose.position=origin;label.pose.position.z+=.035;label.scale.z=.018;
  label.color.r=.8;label.color.g=1.;label.color.b=1.;label.text="TARGET";out.markers.push_back(label);
  return out;
}
}
