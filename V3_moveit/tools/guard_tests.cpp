#include "v3_moveit/guard.hpp"
#include "jakaAPI.h"
#include <iostream>
#include <limits>
using namespace v3_moveit;
static_assert(sizeof(JointValue)==48,"SDK joint layout");
static_assert(sizeof(EDGState)==432,"SDK feedback layout");

static Guard ready(const Joints& q=Joints{}) {
  Guard g;g.lower.fill(-3);g.upper.fill(3);g.reset(q);return g;
}
static void require(bool condition,const char* message) {
  if(!condition)throw std::runtime_error(message);
}
static std::string rejection(Guard& g,const Joints& q,const Joints& actual,double dt) {
  try{g.check(q,actual,dt);return "";}catch(const std::runtime_error& e){return e.what();}
}
static bool rejected(Guard g,const Joints& q,const Joints& actual,double dt) {
  return !rejection(g,q,actual,dt).empty();
}
static bool same_history(const Guard& a,const Guard& b) {
  return a.previous==b.previous&&a.previous_velocity==b.previous_velocity&&
    a.previous_acceleration==b.previous_acceleration&&a.previous_dt==b.previous_dt&&
    a.previous_previous_dt==b.previous_previous_dt&&a.initialized==b.initialized;
}

int main(){
 try{
  Joints zero{};auto g=ready();
  for(int i=0;i<100;++i)g.check(zero,zero,.008);
  Joints bad{};bad[0]=std::numeric_limits<double>::quiet_NaN();require(rejected(g,bad,zero,.008),"nonfinite joint accepted");
  bad=zero;bad[1]=3.1;require(rejected(g,bad,zero,.008),"joint limit accepted");
  bad=zero;bad[2]=.002;require(rejected(g,bad,zero,.008),"velocity limit accepted");
  require(rejected(g,zero,zero,.1),"invalid control interval accepted");
  Joints actual{};actual[3]=.1;require(rejected(g,zero,actual,.008),"following error accepted");
  g.reset(zero);Joints q{};
  for(int i=1;i<=100;++i){double t=i*.008;q[0]=.5*.01*t*t;g.check(q,q,.008);}

  // A rest-to-cubic trajectory has constant jerk 2.7, below the unchanged limit.
  // Varying 4/8/12 ms intervals must not manufacture a jerk spike.
  const Joints origin{.4,.1,-.3,.2,1.,-.9};
  const std::array<double,9> intervals{.004,.008,.012,.012,.004,.008,.008,.012,.004};
  g=ready(origin);double t=0.;
  for(int cycle=0;cycle<3;++cycle)for(double dt:intervals){
    t+=dt;
    for(size_t j=0;j<6;++j)q[j]=origin[j]+(j%2?-1.:1.)*2.7*t*t*t/6.;
    g.check(q,q,dt);
  }
  require(g.previous_dt==intervals.back(),"actual interval not retained");

  // A genuinely excessive cubic jerk must fail, without changing any history.
  g=ready();t=0.;bool excessive_jerk_rejected=false;
  for(double dt:intervals){
    t+=dt;q=zero;q[0]=4.*t*t*t/6.;const auto before=g;
    const auto reason=rejection(g,q,q,dt);
    if(!reason.empty()){
      require(reason=="joint target jerk exceeded","excessive cubic failed for a different limit");
      require(same_history(g,before),"rejected jerk changed history");
      excessive_jerk_rejected=true;break;
    }
  }
  require(excessive_jerk_rejected,"excessive nonuniform jerk accepted");

  // Reset retains the actual initial position, not the first commanded target.
  // Each legal positive-jerk first step passes; a clearly excessive first jump fails.
  for(double dt:{.004,.008,.012}){
    g=ready(origin);q=origin;q[0]+=2.7*dt*dt*dt/6.;g.check(q,q,dt);
    require(g.previous_dt==dt&&g.previous_previous_dt==dt,"first-step virtual timing is inconsistent");
    g.reset(origin);require(g.previous_dt==0.&&g.previous_previous_dt==0.,"reset retained timing history");
    q=origin;q[0]+=g.max_jerk*dt*dt*dt*1.2;const auto before=g;
    require(rejection(g,q,q,dt)=="joint target jerk exceeded","first-step jerk jump accepted");
    require(same_history(g,before),"first-step rejection changed history");
  }

  // Freezing a moving position command creates a discontinuous stop, even at low speed.
  g=ready();t=0.;
  for(int i=0;i<5;++i){t+=.008;q=zero;q[0]=t*t*t/6.;g.check(q,q,.008);}
  const auto moving=g;
  require(rejection(g,q,q,.008)=="joint target jerk exceeded","instantaneous freeze accepted");
  require(same_history(g,moving),"freeze rejection changed history");

  // A genuinely stationary hold remains valid with unequal intervals after reset.
  g.reset(origin);
  for(int i=0;i<10;++i)for(double dt:intervals)g.check(origin,origin,dt);
  for(double dt:{.003999,.012001}){
    const auto before=g;
    require(rejection(g,origin,origin,dt)=="invalid control interval","actual period boundary ignored");
    require(same_history(g,before),"rejected interval changed history");
  }
  const auto before_nan=g;bad=origin;bad[5]=std::numeric_limits<double>::quiet_NaN();
  require(rejection(g,bad,origin,.008)=="nonfinite joint","late-joint NaN accepted");
  require(same_history(g,before_nan),"late-joint rejection partially committed history");
  std::cout<<"PASS: native guards, unequal-period cubic jerk, first-step/reset/freeze rejection, unchanged rejected history and SDK struct layout; no SDK functions called\n";
  return 0;
 }catch(const std::exception& error){std::cerr<<"FAIL: "<<error.what()<<'\n';return 1;}
}
