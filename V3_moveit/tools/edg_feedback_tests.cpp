// SDK-free deterministic tests of the installed EDG decoded-cache semantics.
#include "v3_moveit/edg_feedback.hpp"
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
using namespace v3_moveit;
static size_t assertions=0;
static void expect(bool value,const char* message){++assertions;if(!value)throw std::runtime_error(message);}
struct FakeSdk {
  std::array<unsigned long,3> decoded{1000,0,0};
  std::array<double,6> state{};
  bool new_frame=true;
  int get_calls=0,details_calls=0;
  void get_stat(std::array<double,6>& out) {
    ++get_calls;
    if(new_frame){decoded[1]+=8000000;decoded[2]=(decoded[2]+1)%256;state.fill(double(decoded[1]));}
    out=state;
  }
  std::array<unsigned long,3> details(){++details_calls;return decoded;}
};
static std::array<unsigned long,3> stamp(unsigned long ns,unsigned long count=0){return {1000,ns,count};}
int main(){
 try {
  EdgFreshness f;f.reset(10.);FakeSdk sdk;
  std::array<double,6> state{};
  for(int i=0;i<64;++i){
    const auto before=sdk.details();sdk.get_stat(state);const auto after=sdk.details();
    expect(before!=after,"stub must update its cache in get_stat, not in details");
    expect(state[0]==double(after[1]),"state and after must describe one decoded frame");
    expect(f.observe(after,10.+i*.008,10.+i*.008+.0001)==EdgAdmission::NewSample,
      "continuous fresh get_stat updates must be admitted immediately");
    expect(f.accepted_at()==10.+i*.008,"receipt must use the call start");
  }
  expect(sdk.get_calls==64&&sdk.details_calls==128,"unexpected fake SDK calls");
  sdk.new_frame=false;const double prior=f.accepted_at();
  sdk.get_stat(state);auto duplicate=sdk.details();
  expect(f.observe(duplicate,prior+.008,prior+.0081)==EdgAdmission::Duplicate,"duplicate must be identified");
  expect(f.accepted_at()==prior,"duplicate must not renew freshness");
  auto changed_count=duplicate;changed_count[2]=(changed_count[2]+1)%256;
  expect(f.observe(changed_count,prior+.016,prior+.0161)==EdgAdmission::Duplicate,"counter alone cannot renew source time");
  expect(f.accepted_at()==prior,"counter-only change renewed freshness");
  auto backward=duplicate;backward[1]-=1;
  expect(f.observe(backward,prior+.018,prior+.0181)==EdgAdmission::RegressedStamp,"backward source stamp must fail");
  expect(f.accepted_at()==prior,"regression renewed freshness");
  expect(f.observe(duplicate,prior+.025,prior+.0251)==EdgAdmission::Expired,"duplicate expiry must remain 24ms");
  sdk.new_frame=true;sdk.get_stat(state);auto late_new=sdk.details();
  expect(f.observe(late_new,prior+.026,prior+.0261)==EdgAdmission::Expired,"new frame must not erase previous expiry");
  expect(f.accepted_at()==prior,"expired new frame renewed freshness");
  f.reset(20.);
  expect(f.observe(stamp(1),20.,20.025)==EdgAdmission::SlowCall,"slow SDK call must be rejected");
  expect(!f.have_sample(),"slow call accepted a sample");
  expect(f.observe(stamp(1),21.01,21.0101)==EdgAdmission::Expired,"startup must retain 1s budget");
  f.reset(30.);
  expect(f.observe({0,0,0},30.,30.0001)==EdgAdmission::Duplicate,"empty startup cache must wait");
  expect(!f.have_sample()&&f.accepted_at()==0.,"empty cache must not establish freshness");
  expect(f.observe({0,0,0},30.998,30.999)==EdgAdmission::Duplicate,"startup wait must retain original budget");
  expect(!f.have_sample()&&f.accepted_at()==0.,"repeated empty cache renewed freshness");
  expect(f.observe({0,0,0},31.001,31.0011)==EdgAdmission::Expired,"empty cache must expire after 1s");
  f.reset(30.);
  expect(f.observe({0,1,0},30.,30.0001)==EdgAdmission::InvalidStamp,"partially zero timestamp admitted");
  expect(f.observe({0,0,1},30.,30.0001)==EdgAdmission::InvalidStamp,"nonzero counter is not empty cache");
  expect(f.observe({1000,1000000000,0},30.,30.0001)==EdgAdmission::InvalidStamp,"invalid nsec admitted");
  expect(f.observe(stamp(1,256),30.,30.0001)==EdgAdmission::InvalidStamp,"invalid SDK byte counter admitted");
  expect(f.observe(stamp(1),30.,29.)==EdgAdmission::InvalidClock,"host regression admitted");
  expect(f.observe(stamp(1),30.,std::numeric_limits<double>::quiet_NaN())==EdgAdmission::InvalidClock,"nonfinite host time admitted");
  expect(f.observe(stamp(100,255),30.,30.0001)==EdgAdmission::NewSample,"initial valid stamp refused");
  const double valid_anchor=f.accepted_at();
  expect(f.observe({0,0,0},30.001,30.0011)==EdgAdmission::InvalidStamp,"zero after valid sample must fail");
  expect(f.have_sample()&&f.accepted_at()==valid_anchor,"post-valid zero altered freshness");
  expect(f.observe(stamp(200,0),30.008,30.0081)==EdgAdmission::NewSample,"valid byte counter wrap refused");
  expect(f.observe({1001,0,1},30.016,30.0161)==EdgAdmission::NewSample,"valid second rollover refused");
  EdgAuditRing ring;ring.reset();
  for(size_t i=0;i<1025;++i){EdgAuditSample s;s.begin=double(i);s.admission=i%2?EdgAdmission::Duplicate:EdgAdmission::NewSample;ring.push(s);}
  expect(ring.count()==1025&&ring.new_count()==513&&ring.duplicate_count()==512,"audit counters incorrect");
  expect(ring.last()&&ring.last()->sequence==1024,"audit last sample incorrect");
  std::ostringstream out;ring.write_jsonl(out);auto text=out.str();
  expect(text.find("\"sequence\":512,")==std::string::npos,"oldest audit overflow retained");
  expect(text.find("\"sequence\":513,")!=std::string::npos,"ring chronological start missing");
  expect(text.find("\"sequence\":1024,")!=std::string::npos,"ring latest sample missing");
  expect(text.find("\"retained\":512")!=std::string::npos,"ring capacity not bounded");
  ring.reset();expect(ring.count()==0&&!ring.last(),"ring reset failed");
  std::cout<<"{\"passed\":true,\"assertions\":"<<assertions<<",\"fake_cache_updates\":64,\"audit_samples_exercised\":1025,\"robot_connections\":0,\"sdk_calls\":0,\"ros_started\":false,\"physical_acceptance\":false}\n";
  return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
