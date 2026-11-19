#pragma once
#include "grasp_policy.hpp"
#include <algorithm>

namespace grasp {
struct FingerTrack { int start_ms{}, end_ms{}; double force_n{}; };
struct Action {
  int duration{}, closing{}, opening{}, current{};
  double hard_force{};
  std::array<int,6> low{}, closed{}, velocity{}, manual_current{};
  std::array<FingerTrack,5> tracks{};
  static Action read(std::istream& in) {
    Action a;
    demand(bool(in>>a.duration>>a.closing>>a.opening>>a.current>>a.hard_force),"Incomplete action");
    demand(a.duration>=100 && a.duration<=30000 && a.opening>=100 && a.opening<=20000 &&
           a.closing>=100 && a.closing<=20000 && a.current>=1 && a.current<=900 &&
           std::isfinite(a.hard_force) && a.hard_force>=.1 && a.hard_force<=20,"Invalid action limits");
    for(auto group:{&a.low,&a.closed,&a.velocity,&a.manual_current})
      for(int& v:*group)demand(bool(in>>v),"Incomplete axis settings");
    for(size_t i=0;i<6;++i)
      demand(a.low[i]>=0 && a.low[i]<=a.closed[i] && a.closed[i]<=10000 && a.velocity[i]>=100 &&
             a.velocity[i]<=20000 && a.manual_current[i]>=1 && a.manual_current[i]<=900,"Invalid axis settings");
    for(auto& t:a.tracks) {
      demand(bool(in>>t.start_ms>>t.end_ms>>t.force_n),"Incomplete finger track");
      demand(t.start_ms>=0 && t.start_ms<t.end_ms && t.end_ms<=a.duration &&
             std::isfinite(t.force_n) && t.force_n>=.1 && t.force_n<=a.hard_force,"Invalid finger track");
    }
    std::string extra;demand(!(in>>extra),"Unexpected action parameters");return a;
  }
  bool can_open() const {return std::all_of(low.begin(),low.end(),[](int n){return n==0;});}
  std::array<int,6> target(double trigger) const {
    demand(std::isfinite(trigger) && trigger>=0 && trigger<=1 && can_open(),"Invalid trigger or nonzero open limit");
    std::array<int,6> out{};
    for(int i=0;i<6;++i){
      const auto& t=tracks[std::max(0,i-1)];
      double phase=std::clamp((trigger*duration-t.start_ms)/(t.end_ms-t.start_ms),0.,1.);
      out[i]=std::lround(closed[i]*phase);
    }
    return out;
  }
};
struct ContactStops {
  std::array<int,6> hold{{-1,-1,-1,-1,-1,-1}};
  std::array<int,6> apply(const Action& a,const std::array<int,6>& requested,const std::array<int,6>& actual,
                          const std::array<std::vector<float>,5>& forces){
    auto result=requested;
    for(int i=0;i<6;++i){
      int finger=std::max(0,i-1);
      double sum=0;demand(!forces[finger].empty(),"Missing force sensor");
      for(double f:forces[finger]){demand(std::isfinite(f)&&f>=0,"Invalid force");sum+=f;}
      if(hold[i]>=0 && requested[i]<=hold[i]) hold[i]=-1; // Reaching the held position releases contact, even at zero.
      if(hold[i]<0 && sum>=a.tracks[finger].force_n && requested[i]>actual[i]) hold[i]=actual[i];
      if(hold[i]>=0) result[i]=std::min(requested[i],hold[i]);
    }
    return result;
  }
};
}
