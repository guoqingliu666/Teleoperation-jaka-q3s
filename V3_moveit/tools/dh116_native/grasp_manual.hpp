#pragma once
#include "grasp_policy.hpp"
namespace grasp {
struct ManualInput {
  long sequence=-1;double source=0,received=0;int axis=0,target=0,velocity=0,current=0;
  bool consume(const std::string& text,double now,const std::array<int,6>& low,const std::array<int,6>& high,int speed_cap,int current_cap){
    std::istringstream line(text);char op;long id;double stamp;std::string extra;
    demand(bool(line>>op>>id>>stamp)&&id>sequence&&stamp>source&&fresh(stamp,now),"Invalid/replayed/stale manual input");
    demand(op=='H'||op=='M',"Invalid manual command");
    if(op=='M'){
      demand(bool(line>>axis>>target>>velocity>>current),"Incomplete manual target");
      demand(axis>=1&&axis<=6&&low[axis-1]<=target&&target<=high[axis-1]&&target>=0&&target<=10000,"Manual target outside configured limits");
      demand(velocity>=100&&velocity<=speed_cap&&velocity<=2000&&current>=1&&current<=current_cap&&current<=900,"Manual speed/current outside authorized limits");
    }
    demand(!(line>>extra),"Trailing manual input");sequence=id;source=stamp;received=stamp;return op=='M';
  }
};
}
