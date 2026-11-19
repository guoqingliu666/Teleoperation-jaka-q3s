#pragma once
#include "workspace_action.hpp"
#include <cerrno>
#include <functional>
#include <unistd.h>

namespace grasp {
// User-selected workspace. EtherCAT and both input streams stay owned across
// manual/trigger/feedback transitions. Only the SDK owner can change targets.
struct HandWorkspace {
  using Axes=std::array<int,6>;
  const Axes& pos; const Axes& status; const Axes& torque;
  const std::array<std::vector<float>,5>& forces;
  std::function<void()> enable, disable, home;
  std::function<void(const Axes&,const Axes&,const Axes&)> move;
  std::function<void(const Action&)> configure;
  std::function<double()> clock;
  Action action{};
  ContactStops contacts{};
  Axes wanted{}, commanded{}, currents{}, speeds{};
  bool configured=false, armed=false, pending=false, trigger_live=false, manual_lease=false, auto_trigger=false;
  std::string source="disabled";
  double heartbeat=0,last_write=0;
  double phase=0,phase_at=0,published_phase=-1,phase_report_at=0;
  long serial=-1;
  Input trigger{};
  void announce(const char* reason) const {
    std::cout<<"{\"event\":\"workspace\",\"source\":\""<<source<<"\",\"armed\":"<<(armed?"true":"false")
             <<",\"reason\":\""<<reason<<"\"}"<<std::endl;
  }
  void park(const char* reason,const std::string& next) {
    if(armed)disable();
    armed=false;pending=false;manual_lease=false;trigger_live=false;auto_trigger=false;source=next;trigger=Input{};
    phase=0;phase_at=clock();published_phase=-1;wanted=commanded=pos;contacts=ContactStops{};announce(reason);
  }
  bool homed() const {return std::all_of(status.begin(),status.end(),[](int s){return (s&16)!=0;});}
  bool force_safe() const {
    for(const auto& finger:forces){
      double sum=0;for(double f:finger)sum+=f;
      if(finger.empty()||!std::isfinite(sum)||sum>=action.hard_force)return false;
    }return true;
  }
  bool arm(){
    if(armed)return true;
    if(!homed()){announce("homing_required");return false;}
    if(!force_safe()){announce("force_limit");return false;}
    enable();armed=true;wanted=commanded=pos;contacts=ContactStops{};speeds=action.velocity;
    currents=action.manual_current;heartbeat=clock();announce("enabled");return true;
  }
  void line(const std::string& text){
    std::istringstream in(text);char op{};in>>op;
    if(op=='P'){auto next=Action::read(in);park("profile","disabled");configure(next);action=next;configured=true;return;}
    demand(configured,"Workspace not configured");
    if(op=='S'){
      std::string next,extra;demand(bool(in>>next)&&!(in>>extra),"Invalid control source");
      demand(next=="manual"||next=="trigger"||next=="disabled"||next=="home"||next=="open","Unknown control source");
      if(next=="open"){
        if(!action.can_open()){announce("nonzero_open_limit");return;}
        if(source!="manual")park("switch","manual");
        if(!arm())return;
        wanted.fill(0);speeds=action.velocity;currents=action.manual_current;pending=true;manual_lease=true;heartbeat=clock();
        announce("opening");return;
      }
      park("switch",next);
      if(next=="home"){home();source="manual";announce("homed");arm();}
      else if(next=="manual")arm();
      else if(next=="trigger"&&!action.can_open())park("nonzero_open_limit","disabled");
      return;
    }
    if(op=='T'){
      if(source!="trigger" && source!="manual")return;
      double now=clock();
      try{
        bool valid=trigger.consume(text,now,trigger_live);
        if(!valid)return;
      }catch(const std::runtime_error&){
        if(trigger_live)park("trigger_lost",auto_trigger?"manual":"trigger");else trigger=Input{};
        return;
      }
      if(source=="manual"){
        // Idle trigger packets must not undo a mouse target. A fresh release followed
        // by a deliberate squeeze transfers control without cycling the EtherCAT bus.
        if(trigger.trigger<.05 || !action.can_open())return;
        if(!armed){if(arm())trigger=Input{};return;}
        source="trigger";auto_trigger=true;manual_lease=false;announce("trigger_follow");
      }
      if(!armed){
        if(arm())trigger=Input{}; // Enabling can take longer than a packet lease. Await a new release.
        return;
      }
      if(!trigger_live){phase=0;phase_at=clock();}
      trigger_live=true;speeds.fill(20000);currents.fill(action.current);pending=true;
      return;
    }
    if(op=='X'){
      if(source=="trigger" && (trigger_live||trigger.released))park("trigger_lost",auto_trigger?"manual":"trigger");
      else trigger=Input{};
      return;
    }
    demand(op=='H'||op=='M',"Unknown workspace command");
    if(source!="manual")return;
    long id{};double stamp{};demand(bool(in>>id>>stamp),"Incomplete manual command");
    if(id<=serial || !fresh(stamp,clock())){
      if(manual_lease)park("manual_paused","manual");
      return;
    }
    serial=id;heartbeat=stamp;
    if(op=='H'){std::string extra;demand(!(in>>extra),"Invalid heartbeat");return;}
    int axis,target,velocity,current;std::string extra;
    demand(bool(in>>axis>>target>>velocity>>current)&&!(in>>extra)&&axis>=1&&axis<=6,"Invalid manual target");
    int i=axis-1;
    demand(target>=action.low[i]&&target<=action.closed[i]&&velocity>=100&&velocity<=20000&&
           current>=1&&current<=action.manual_current[i],"Manual target exceeds profile");
    if(!arm())return;
    if(!fresh(stamp,clock()))return;
    wanted[i]=target;speeds[i]=std::min(velocity,action.velocity[i]);currents[i]=current;
    pending=true;manual_lease=true;heartbeat=clock();
  }
  void tick(){
    if(!armed)return;
    const auto now=clock();
    if(std::any_of(status.begin(),status.end(),[](int s){return !(s&1);})){park("axis_disabled","disabled");return;}
    if(!force_safe()){park("force_limit","disabled");return;}
    if(trigger_live&&!fresh(trigger.received,now)){park("trigger_lost",auto_trigger?"manual":"trigger");return;}
    if(manual_lease&&!fresh(heartbeat,now)){park("manual_paused","manual");return;}
    if(trigger_live){
      // The lever sets the destination on the clip. A fast squeeze still observes
      // each finger's start time; a complete release immediately requests opening.
      double step=std::max(0.,now-phase_at)*1000/action.duration;phase_at=now;
      phase=trigger.trigger==0 ? 0 : phase+std::clamp(trigger.trigger-phase,-step,step);
      wanted=action.target(phase);
      if(phase!=published_phase && now-phase_report_at>=.066){
        std::cout<<"{\"event\":\"action_progress\",\"phase\":"<<phase<<"}"<<std::endl;
        phase_report_at=now;published_phase=phase;
      }
    }
    auto previous=contacts.hold;
    auto target=contacts.apply(action,wanted,pos,forces);
    for(int finger=0;finger<5;++finger){
      const int i=finger+1;
      bool held=contacts.hold[i]>=0 || (finger==0&&contacts.hold[0]>=0);
      bool was_held=previous[i]>=0 || (finger==0&&previous[0]>=0);
      if(held!=was_held)
        std::cout<<"{\"event\":\"finger_contact\",\"finger\":"<<finger<<",\"held\":"<<(held?"true":"false")
                 <<",\"limit_n\":"<<action.tracks[finger].force_n<<"}"<<std::endl;
    }
    bool contact_changed=false;
    for(int i=0;i<6;++i)if(contacts.hold[i]>=0&&target[i]!=commanded[i])contact_changed=true;
    if(target!=commanded && (contact_changed||now-last_write>=.02)){
      Axes velocity{};
      for(int i=0;i<6;++i)velocity[i]=std::min(speeds[i],target[i]>=pos[i]?action.closing:action.opening);
      move(target,velocity,currents);commanded=target;last_write=now;
    }
    pending=target!=commanded;
    if(auto_trigger && trigger.trigger==0 && !pending){
      source="manual";auto_trigger=false;trigger_live=false;manual_lease=true;heartbeat=now;
      speeds=action.velocity;currents=action.manual_current;announce("released");
    }
    bool arrived=!pending;
    for(int i=0;i<6;++i){
      if(torque[i]&&std::abs(pos[i]-commanded[i])>200){park("torque_limit","disabled");return;}
      if(std::abs(pos[i]-commanded[i])>20 || !(status[i]&2))arrived=false;
    }
    if(arrived)manual_lease=false;
  }
};
}
