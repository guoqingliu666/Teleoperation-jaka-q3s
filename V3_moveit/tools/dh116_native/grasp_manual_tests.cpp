#include "grasp_manual.hpp"
#include <cassert>
#include <iostream>
template<class F>void rejected(F action){bool caught=false;try{action();}catch(const std::runtime_error&){caught=true;}assert(caught);}
int main(){
 std::array<int,6> low{},high{10000,10000,9000,5500,10000,10000};grasp::ManualInput input;
 assert(!input.consume("H 1 10",10.01,low,high,2000,900));
 assert(input.consume("M 2 10.02 3 9000 1500 600",10.03,low,high,2000,900));assert(input.axis==3&&input.target==9000);
 assert(grasp::fresh(input.received,10.09));assert(!grasp::fresh(input.received,10.11));
 for(const auto* bad:{"M 3 10.04 4 5501 1500 600","M 3 10.04 0 0 1500 600","M 3 10.04 7 0 1500 600","M 3 10.04 2 -1 1500 600","M 3 10.04 2 0 2001 600","M 3 10.04 2 0 1500 901","M 3 10.04 2 0 99 600","M 3 10.04 2 0 1500 0","M 3 10.04 2 0 1500 600 extra","H 3 10.04 extra","M 2 10.04 2 0 1500 600","H 3 10.01","H 3 nan","H 3 11","Z 3 10.04"})rejected([&]{input.consume(bad,10.05,low,high,2000,900);});
 rejected([&]{input.consume("H 3 10.03",10.12,low,high,2000,900);});
 rejected([&]{input.consume("M 3 10.04 2 100 1500 601",10.05,low,high,2000,600);});
 rejected([&]{input.consume("M 3 10.04 2 100 1600 600",10.05,low,high,1500,900);});
 std::cout<<"PASS: manual input sequence/time/lease, per-axis endpoints, speed/current caps; no hardware\n";
}
