#pragma once
#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
namespace grasp {
inline void demand(bool yes,const char* text){if(!yes)throw std::runtime_error(text);}
inline bool fresh(double stamp,double now){return std::isfinite(stamp)&&std::isfinite(now)&&stamp>0&&now>=stamp&&now-stamp<.08;}
// A previous homed bit alone cannot acknowledge a newly requested homing cycle.
struct HomingCycle {
  std::array<bool,6> started{};
  explicit HomingCycle(const std::array<int,6>& initial) {
    for (size_t i=0;i<started.size();++i) started[i]=!(initial[i]&0x10);
  }
  bool completed(const std::array<int,6>& status) {
    bool all_done=true;
    for (size_t i=0;i<started.size();++i) {
      int state=status[i]>>5;
      demand((status[i]&1) && (state==0 || state==1 || state==7), "Axis disabled or faulted during homing");
      bool homed=status[i]&0x10;
      if (!homed || state==7) started[i]=true;
      if (!started[i] || !homed || state!=0) all_done=false;
    }
    return all_done;
  }
};
inline void force_limit(const std::array<std::vector<float>,5>& forces,double limit){
  demand(std::isfinite(limit)&&limit>=.1&&limit<=20,"Invalid force limit");
  for(const auto& finger:forces){double sum=0;demand(!finger.empty(),"Missing force sensor");for(double f:finger){demand(std::isfinite(f)&&f>=0&&f<=655.35,"Invalid force value");sum+=f;}demand(sum<limit,"Normal force limit reached; stop latched");}
}
inline std::array<int,6> interpolate(const std::array<int,6>& open,const std::array<int,6>& closed,double trigger){
  demand(std::isfinite(trigger)&&trigger>=0&&trigger<=1,"Invalid trigger");std::array<int,6> target{};
  for(int i=0;i<6;i++){demand(open[i]>=0&&open[i]<=closed[i]&&closed[i]<=10000,"Invalid endpoints");target[i]=std::lround(open[i]+trigger*(closed[i]-open[i]));}return target;
}
inline std::array<int,6> acceptance_target(int step,const std::array<int,6>& open,const std::array<int,6>& closed){
  demand(step>=-1&&step<=5,"Invalid acceptance step");auto target=interpolate(open,closed,0);
  if(step>=0&&step<5){for(int i=0;i<step+2;i++)target[i]=closed[i];}
  return target;
}
inline std::array<int,6> acceptance_target(int step){return acceptance_target(step,{},{{10000,10000,10000,10000,10000,10000}});}
struct Input {
  long sequence=-1;double source=0,received=0,trigger=0;bool released=false;
  bool consume(const std::string& text,double now,bool armed){
    std::istringstream line(text);char op;long id;double stamp,t;int valid;std::string extra;
    demand(bool(line>>op>>id>>stamp>>t>>valid)&&!(line>>extra)&&op=='T'&&id>sequence&&stamp>source&&fresh(stamp,now)&&std::isfinite(t)&&t>=0&&t<=1&&(valid==0||valid==1),"Invalid/replayed/delayed control");
    sequence=id;source=stamp;trigger=t;
    if(!valid){released=false;demand(!armed,"Input invalid; stop latched");return false;}
    if(!fresh(received,now))released=false;
    received=stamp;if(t<.05)released=true;return released;
  }
};
}
