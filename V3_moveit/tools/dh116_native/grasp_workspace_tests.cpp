// Drive the production workspace state machine with a clock and a simulated hand.
#ifdef NDEBUG
#undef NDEBUG  // Keep these checks active in release builds.
#endif
#include <array>
#include <cassert>
#include <iostream>
#include <sstream>
#include "grasp_workspace.hpp"
int main(){
  using grasp::HandWorkspace;using Axes=HandWorkspace::Axes;
  Axes pos{},status{},torque{},last{},speed{};status.fill(0x13);
  std::array<std::vector<float>,5> forces;for(auto& f:forces)f={0.f};
  double t=10.;int enables=0,parks=0,moves=0,homes=0;
  HandWorkspace w{pos,status,torque,forces,[&]{++enables;},[&]{++parks;},[&]{++homes;},
    [&](const Axes& goal,const Axes& v,const Axes&){last=goal;speed=v;++moves;},
    [](const grasp::Action&){},[&]{return t;}};
  std::string profile="P 1000 14000 6000 400 4 0 0 0 0 0 0 8000 6000 10000 10000 10000 10000 "
                      "14000 14000 14000 14000 14000 14000 400 400 400 400 400 400 "
                      "0 1000 2 250 750 1.5 500 1000 2 0 500 2 0 1000 2";
  w.line(profile);w.line("S manual");assert(w.armed&&moves==0&&enables==1);
  t=50.;w.tick();assert(w.armed&&parks==0); // Long manual idle keeps the same owner and touch stream.
  w.line("M 1 50 3 2000 14000 400");w.tick();assert(last[2]==2000&&speed[2]==14000);
  t+=.081;w.tick();assert(!w.armed&&w.source=="manual"&&parks==1);
  w.line("H 2 50.081");assert(!w.armed);
  t=51.;w.line("M 3 51 3 1000 14000 400");w.tick();assert(w.armed&&last[2]==1000);
  pos=last;t+=.03;
  std::ostringstream opening;auto previous=std::cout.rdbuf(opening.rdbuf());
  w.line("S open");std::cout.rdbuf(previous);w.tick();
  assert(opening.str().find("\"reason\":\"opening\"")!=std::string::npos);
  assert(w.armed&&w.source=="manual"&&last==Axes{}&&speed[2]==6000&&homes==0);
  pos.fill(0);
  t=52.;w.line("S trigger");assert(!w.armed);
  w.line("T 1 52 1 1");assert(!w.armed);t+=.01;w.line("T 2 52.01 0 1");assert(w.armed&&!w.trigger_live);
  t+=.01;w.line("T 3 52.02 1 1");assert(!w.trigger_live);
  t=52.03;w.line("T 4 52.03 0 1");w.tick();assert(w.trigger_live);
  long seq=5;
  auto frame=[&](double value){
    t+=.02;std::ostringstream line;line.precision(12);line<<"T "<<seq++<<" "<<t<<" "<<value<<" 1";
    w.line(line.str());w.tick();
  };
  for(int i=0;i<10;++i)frame(.5);
  assert(last[2]==0 && last[0]>0); // Index starts at 250 ms, even on a quick squeeze.
  for(int i=0;i<18;++i)frame(.5);
  assert((last==Axes{4000,3000,5000,0,10000,5000}));
  pos=last;forces[1]={1.6f};
  for(int i=0;i<28;++i)frame(1);
  assert((last==Axes{8000,6000,5000,10000,10000,10000}));
  forces[1]={0};frame(1);assert(last[2]==5000);
  frame(0);frame(0);assert((last==Axes{})&&speed[2]==6000);
  pos.fill(0);forces[1]={1.6f};
  for(int i=0;i<25;++i)frame(1);
  assert(w.contacts.hold[2]==0&&last[2]==0); // Contact while fully open.
  forces[1]={0};frame(0);assert(w.contacts.hold[2]<0);
  for(int i=0;i<25;++i)frame(1);
  assert(last[2]>0); // The next squeeze works without reconnecting.
  t+=.09;w.tick();assert(!w.armed&&w.source=="trigger");
  w.line("S manual");forces[3]={4.1f};w.tick();assert(!w.armed&&w.source=="disabled");
  forces[3]={0};w.line("S home");assert(homes==1&&w.armed);
  w.line("S disabled");assert(!w.armed&&w.configured);
  w.line("S manual");int before=moves;
  frame(0);frame(0);assert(moves==before && w.source=="manual"); // A released idle lever leaves mouse control alone.
  for(int i=0;i<55;++i)frame(1);
  assert(w.source=="trigger" && last[2]==10000);
  frame(0);frame(0);assert(w.source=="manual" && (last==Axes{}));
  w.line(profile);status[2]=0x6;w.line("S manual");assert(!w.armed);
  std::cout<<"PASS persistent manual, fresh re-arm, signed speed selection, five tracks, contact hold, guarded reopen, global stop, homing; simulated SDK only\n";
}
