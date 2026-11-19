#include "grasp_policy.hpp"
#include <cassert>
#include <iostream>
#include <limits>
template<class F> void rejected(F action){bool caught=false;try{action();}catch(const std::runtime_error&){caught=true;}assert(caught);}
int main(){
 using namespace grasp;
 Input in;assert(!in.consume("T 1 10 .5 1",10.01,false));assert(in.consume("T 2 10.02 0 1",10.03,false));assert(in.consume("T 3 10.04 .5 1",10.05,true));
 rejected([&]{in.consume("T 3 10.05 .8 1",10.06,true);});rejected([&]{in.consume("T 4 10.05 .8 1 extra",10.06,true);});rejected([&]{in.consume("T 4 10.05 .8 1",10.2,true);});rejected([&]{in.consume("T 4 10.07 .8 0",10.08,true);});
 Input gap;assert(gap.consume("T 1 10 0 1",10,false));assert(!gap.consume("T 2 10.2 .7 1",10.2,false));
 assert(!fresh(10,9));assert(!fresh(10,10.081));assert(!fresh(0,.01));assert(fresh(10,10.079));
 std::array<std::vector<float>,5> forces;for(auto& f:forces)f={.5,.5};force_limit(forces,3);forces[1]={1.6,1.6};rejected([&]{force_limit(forces,3);});forces[1]={};rejected([&]{force_limit(forces,3);});forces[1]={std::numeric_limits<float>::quiet_NaN()};rejected([&]{force_limit(forces,3);});
 std::array<int,6> open{},closed{10000,8000,5000,0,1000,2000};auto target=interpolate(open,closed,.5);assert((target==std::array<int,6>{5000,4000,2500,0,500,1000}));rejected([&]{interpolate(open,closed,-.1);});
 assert((acceptance_target(0)==std::array<int,6>{10000,10000,0,0,0,0}));assert((acceptance_target(4)==std::array<int,6>{10000,10000,10000,10000,10000,10000}));assert(acceptance_target(-1)==open);assert(acceptance_target(5)==open);
 std::array<int,6> pose{10000,10000,9500,10000,10000,10000};
 assert((acceptance_target(1,open,pose)==std::array<int,6>{10000,10000,9500,0,0,0}));
 assert(acceptance_target(4,open,pose)==pose);assert(acceptance_target(5,open,pose)==open);
 pose[2]=10001;rejected([&]{acceptance_target(1,open,pose);});
 std::array<int,6> axes;axes.fill(7);HomingCycle first(axes);
 axes.fill(23);axes[3]=7;assert(!first.completed(axes));
 axes[3]=23;assert(first.completed(axes));
 HomingCycle repeat(axes);assert(!repeat.completed(axes));
 axes.fill((7<<5)|1);assert(!repeat.completed(axes));
 axes.fill(23);assert(repeat.completed(axes));
 axes[1]=6;rejected([&]{repeat.completed(axes);});
 axes[1]=(2<<5)|1;rejected([&]{repeat.completed(axes);});
 std::cout<<"PASS: native input lease/replay/release, force sum/missing/NaN, full sequential fist and simultaneous open; no SDK or hardware\n";
}
