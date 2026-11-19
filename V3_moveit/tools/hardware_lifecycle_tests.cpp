
// Pure native fake-SDK tests. The runner extracts CURRENT driver method bodies
// into hardware_lifecycle_methods.inc; this file contains no copied control logic.
// No vendor library, ROS runtime, socket, or hardware connection is used.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include "v3_moveit/jaka_sdk.hpp"
#include "v3_moveit/guard.hpp"
#include "v3_moveit/edg_feedback.hpp"
namespace rclcpp { static const char* get_logger(const char* name) { return name; } }
template<class... Args> static void mock_log(Args&&...) {}
#define RCLCPP_ERROR(...) mock_log(__VA_ARGS__)
#define RCLCPP_INFO(...) mock_log(__VA_ARGS__)
namespace rclcpp_lifecycle { struct State {}; }
using Clock=std::chrono::steady_clock;
static double seconds(){return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
enum class CB { SUCCESS, ERROR };
using v3_moveit::Joints;
using v3_moveit::Guard;
using namespace v3_moveit;
class Harness {
public:
  // These declarations and bodies come from the source snapshot being tested.
#include "hardware_lifecycle_members.inc"
  Harness() { handle_=1; logged_=true; edg_=true; mass_=1.; client_="127.0.0.1"; }
#include "hardware_lifecycle_methods.inc"
};
namespace mock {
Harness* owner=nullptr;
MotionStatus motion{};RobotStatus_simple simple{};
ProgramState program=PROGRAM_IDLE;BOOL drag=FALSE,servo=FALSE;
CartesianPose tcp{},fk{};PayLoad payload{};
int abort_rc=0,disable_rc=0,read_rc=0,enable_rc=0,edg_rc=0,destroy_rc=0;
int tool_id=1;
int edg_reads=0,edg_fail_after=0,edg_read_delay_ms=0;
unsigned long cache_frame=0;
bool edg_duplicate=false,edg_nonfinite=false;
std::atomic<bool> worker_left{false};
bool require_worker_joined=false;
bool entry_was_pending=false,block_motion=false,motion_entered=false,release_motion=false;
std::mutex lock;std::condition_variable changed;
std::vector<std::string> calls;
void record(const std::string& name){std::lock_guard<std::mutex> g(lock);calls.push_back(name);}
void reset(){
  motion={};simple={};simple.powered_on=1;simple.enabled=1;
  program=PROGRAM_IDLE;drag=FALSE;servo=FALSE;tcp={};tcp.tran.z=200;fk={};payload={};payload.mass=1;
  abort_rc=disable_rc=read_rc=enable_rc=edg_rc=destroy_rc=0;
  tool_id=1;
  entry_was_pending=block_motion=motion_entered=release_motion=false;calls.clear();owner=nullptr;
  edg_reads=edg_fail_after=edg_read_delay_ms=0;cache_frame=0;
  edg_duplicate=edg_nonfinite=false;worker_left=false;require_worker_joined=false;
}
}
extern "C" {
errno_t get_motion_status(const JKHD*,MotionStatus* value){
  mock::record("motion");
  if(mock::block_motion){std::unique_lock<std::mutex> g(mock::lock);mock::motion_entered=true;mock::changed.notify_all();mock::changed.wait(g,[]{return mock::release_motion;});}
  *value=mock::motion;return 0;
}
errno_t get_robot_status_simple(const JKHD*,RobotStatus_simple* value){mock::record("simple");*value=mock::simple;return 0;}
errno_t is_in_servomove(const JKHD*,BOOL* value){mock::record("read_servo");*value=mock::servo;return mock::read_rc;}
errno_t is_in_pos(const JKHD*,BOOL* value){*value=TRUE;return 0;}
errno_t get_program_state(const JKHD*,ProgramState* value){*value=mock::program;return 0;}
errno_t is_in_drag_mode(const JKHD*,BOOL* value){*value=mock::drag;return 0;}
errno_t get_tool_id(const JKHD*,int* value){*value=mock::tool_id;return 0;}
errno_t get_tool_data(const JKHD*,int,CartesianPose* value){*value=mock::tcp;return 0;}
errno_t get_payload(const JKHD*,PayLoad* value){*value=mock::payload;return 0;}
errno_t get_collision_level(const JKHD*,int* value){*value=1;return 0;}
errno_t kine_forward(const JKHD*,const JointValue*,CartesianPose* value){*value=mock::fk;return 0;}
errno_t motion_abort(const JKHD*){mock::record("abort");return mock::abort_rc;}
errno_t servo_move_enable(const JKHD*,BOOL enable){
  mock::record(enable?"enable":"disable");mock::servo=enable;
  if(enable){mock::entry_was_pending=mock::owner&&mock::owner->servo_pending_;return mock::enable_rc;}
  return mock::disable_rc;
}
errno_t edg_init_extend(const JKHD*,BOOL enable,const char*,int,int){
  if(enable)throw std::runtime_error("test must not open an EDG session");
  if(mock::require_worker_joined&&!mock::worker_left)throw std::runtime_error("EDG closed before worker joined");
  mock::record("edg_off");return mock::edg_rc;
}
errno_t edg_get_stat(const JKHD*,EDGState* state,unsigned char){
  mock::record("get_stat");++mock::edg_reads;
  if(mock::edg_read_delay_ms)std::this_thread::sleep_for(std::chrono::milliseconds(mock::edg_read_delay_ms));
  if(mock::edg_fail_after&&mock::edg_reads>mock::edg_fail_after)return -7;
  if(!mock::edg_duplicate||!mock::cache_frame)++mock::cache_frame;
  *state={};state->jointVal.jVal[0]=double(mock::cache_frame)*.001;
  if(mock::edg_nonfinite)state->jointVal.jVal[0]=std::numeric_limits<double>::quiet_NaN();
  return 0;
}
errno_t edg_stat_details(const JKHD*,unsigned long* stamp){
  mock::record("details");stamp[0]=101;stamp[1]=mock::cache_frame*8000000UL;stamp[2]=mock::cache_frame%256;return 0;
}
errno_t destory_handler(const JKHD*){mock::record("destroy");return mock::destroy_rc;}
}
static int cases=0;
static void expect(bool value,const char* message){if(!value)throw std::runtime_error(message);}
static void check(const char* name,const std::function<void()>& test){mock::reset();test();++cases;std::cout<<"PASS "<<name<<'\n';}
static bool rejects(const std::function<void()>& f){try{f();return false;}catch(const std::runtime_error&){return true;}}
static CB activate_with_feedback(Harness& h,double velocity=0.,double drift_rate=0.){
  std::atomic<bool> running{true};double started=seconds();
  std::thread feeder([&]{while(running){
    double now=seconds();
    {std::lock_guard<std::mutex> g(h.data_);h.measured_[0]=drift_rate*(now-started);h.velocity_[0]=velocity;h.feedback_at_=now;}
    h.safety_at_=now;std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }});
  CB result=h.on_activate({});running=false;feeder.join();return result;
}
int main(){
 try{
 check("ready preflight",[]{Harness h;h.preflight();});
 check("queued, active and paused motion rejected",[]{Harness h;
   mock::motion.queue=1;expect(rejects([&]{h.preflight();}),"queued accepted");mock::motion.queue=0;
   mock::motion.active_queue=1;expect(rejects([&]{h.preflight();}),"active queue accepted");mock::motion.active_queue=0;
   mock::motion.paused=TRUE;expect(rejects([&]{h.preflight();}),"paused accepted");});
 check("running and paused programs rejected",[]{Harness h;mock::program=PROGRAM_RUNNING;expect(rejects([&]{h.preflight();}),"running program accepted");mock::program=PROGRAM_PAUSED;expect(rejects([&]{h.preflight();}),"paused program accepted");});
 check("drag rejected",[]{Harness h;mock::drag=TRUE;expect(rejects([&]{h.preflight();}),"drag accepted");});
 check("nonfinite TCP rejected",[]{Harness h;mock::tcp.tran.x=std::numeric_limits<double>::quiet_NaN();expect(rejects([&]{h.preflight();}),"NaN TCP accepted");});
 check("nonfinite payload rejected",[]{Harness h;mock::payload.mass=std::numeric_limits<double>::quiet_NaN();expect(rejects([&]{h.preflight();}),"NaN mass accepted");});
 check("selected TCP3 accepted with matching definition",[]{Harness h;h.tool_id_=mock::tool_id=3;h.tool_tcp_[0]=mock::tcp.tran.x=12.;h.preflight();});
 check("changed active tool rejected",[]{Harness h;mock::tool_id=3;expect(rejects([&]{h.preflight();}),"Changed tool accepted");});
 check("changed TCP definition rejected",[]{Harness h;mock::tcp.tran.x=12.;expect(rejects([&]{h.preflight();}),"Changed TCP accepted");});
 check("nonfinite centroid rejected",[]{Harness h;mock::payload.centroid.x=std::numeric_limits<double>::quiet_NaN();expect(rejects([&]{h.preflight();}),"NaN cog accepted");});
 check("nonfinite FK rejected",[]{Harness h;mock::fk.rpy.rx=std::numeric_limits<double>::quiet_NaN();expect(rejects([&]{h.preflight();}),"NaN FK accepted");});
 check("unattempted servo is not stopped",[]{Harness h;expect(h.stop_servo(),"unattempted not ok");expect(mock::calls.empty(),"unowned stop called");});
 check("successful stop precedes session teardown",[]{Harness h;h.servo_pending_=true;expect(h.shutdown(),"shutdown failed");expect(!h.servo_pending_&&!h.logged_&&!h.edg_,"ownership retained after success");expect(mock::calls==std::vector<std::string>{"abort","disable","read_servo","edg_off","destroy"},"wrong stop ordering");});
 check("three failed stop attempts retain owner; later retry succeeds",[]{Harness h;h.servo_pending_=true;mock::read_rc=-1;expect(!h.shutdown(),"failed stop passed");expect(h.servo_pending_&&h.logged_&&h.edg_,"failed stop discarded owner");expect(mock::calls.size()==9,"retry count not three or premature teardown");mock::read_rc=0;expect(h.shutdown(),"later stop retry failed");expect(!h.servo_pending_&&!h.logged_,"later retry retained owner");});
 check("abort error is retained despite successful disable/readback",[]{Harness h;h.servo_pending_=true;mock::abort_rc=-1;expect(!h.shutdown(),"abort error accepted");expect(h.servo_pending_&&h.logged_,"abort error discarded pending stop");expect(mock::calls.size()==9,"wrong attempt count");});
 check("enable error still triggers stop transaction",[]{Harness h;mock::owner=&h;mock::enable_rc=-1;expect(activate_with_feedback(h)==CB::ERROR,"enable failure accepted");expect(mock::entry_was_pending,"pending set too late");expect(mock::calls==std::vector<std::string>{"enable","abort","disable","read_servo","edg_off","destroy"},"ambiguous enable skipped cleanup");});
 check("stop waits for in-flight RPC, skips next poll RPC, then stops",[]{
   Harness h;h.servo_pending_=true;h.running_=true;mock::block_motion=true;
   h.poller_=std::thread([&]{h.poll();});
   {std::unique_lock<std::mutex> g(mock::lock);expect(mock::changed.wait_for(g,std::chrono::seconds(1),[]{return mock::motion_entered;}),"poll did not enter");}
   bool result=false;std::thread stopping([&]{result=h.shutdown();});
   while(!h.stop_requested_)std::this_thread::yield();
   std::this_thread::sleep_for(std::chrono::milliseconds(20));
   bool still_only_motion;
   {std::lock_guard<std::mutex> g(mock::lock);still_only_motion=mock::calls==std::vector<std::string>{"motion"};mock::release_motion=true;}
   mock::changed.notify_all();stopping.join();
   expect(still_only_motion,"stop overlapped blocked control RPC");expect(result,"serialized stop failed");
   expect(mock::calls==std::vector<std::string>{"motion","abort","disable","read_servo","edg_off","destroy"},"poll entered another RPC after stop request");
 });
 check("failed EDG teardown retains handle for retry",[]{Harness h;h.servo_pending_=true;mock::edg_rc=-1;expect(!h.shutdown(),"EDG error accepted");expect(h.logged_&&h.edg_&&!h.servo_pending_,"EDG failure discarded owner");mock::edg_rc=0;expect(h.shutdown(),"EDG retry failed");});
 check("all lifecycle callbacks propagate unconfirmed stop",[]{Harness h;h.servo_pending_=true;mock::read_rc=-1;expect(h.on_deactivate({})==CB::ERROR,"deactivate returned success");expect(h.on_cleanup({})==CB::ERROR,"cleanup returned success");expect(h.on_error({})==CB::ERROR,"error returned success");expect(h.on_shutdown({})==CB::ERROR,"shutdown returned success");expect(h.servo_pending_&&h.logged_,"lifecycle discarded owner");});

 check("activation waits for continuous 0.25 seconds of new quiet samples",[]{Harness h;double start=seconds();expect(activate_with_feedback(h)==CB::SUCCESS,"quiet activation refused");expect(seconds()-start>=.25,"quiet interval bypassed");expect(h.active_&&h.servo_pending_,"activation state missing");expect(h.shutdown(),"quiet stop failed");});
 check("nonstationary speed blocks activation",[]{Harness h;expect(activate_with_feedback(h,.006)==CB::ERROR,"moving activation accepted");expect(std::find(mock::calls.begin(),mock::calls.end(),"enable")==mock::calls.end(),"servo enabled while moving");});
 check("joint drift blocks activation even with zero reported velocity",[]{Harness h;expect(activate_with_feedback(h,0.,.02)==CB::ERROR,"drifting activation accepted");expect(std::find(mock::calls.begin(),mock::calls.end(),"enable")==mock::calls.end(),"servo enabled while drifting");});
 check("single fresh sample cannot satisfy stationary warmup",[]{Harness h;h.feedback_at_=seconds();h.safety_at_=seconds();expect(h.on_activate({})==CB::ERROR,"single sample activation accepted");expect(std::find(mock::calls.begin(),mock::calls.end(),"enable")==mock::calls.end(),"servo enabled without continuous samples");});
 check("normal on_shutdown stops before closing and repeated callbacks are idempotent",[]{
   Harness h;h.servo_pending_=true;h.active_=true;h.running_=true;
   expect(h.on_shutdown({})==CB::SUCCESS,"normal shutdown failed");
   expect(h.stop_requested_&&!h.active_&&!h.running_,"shutdown did not latch inactive state");
   const auto once=mock::calls;
   expect(once==std::vector<std::string>{"abort","disable","read_servo","edg_off","destroy"},"normal shutdown order");
   expect(h.on_cleanup({})==CB::SUCCESS&&h.on_error({})==CB::SUCCESS&&
          h.on_deactivate({})==CB::SUCCESS&&h.on_shutdown({})==CB::SUCCESS,"repeat cleanup failed");
   expect(mock::calls==once,"repeat cleanup called SDK again");
 });
 check("destroy failure retains only the handle and retries without another stop",[]{
   Harness h;h.servo_pending_=true;mock::destroy_rc=-1;
   expect(h.on_shutdown({})==CB::ERROR,"destroy failure accepted");
   expect(h.logged_&&!h.edg_&&!h.servo_pending_,"destroy failure lost retry state");
   const auto before=mock::calls.size();mock::destroy_rc=0;
   expect(h.on_cleanup({})==CB::SUCCESS,"destroy retry failed");
   expect(mock::calls.size()==before+1&&mock::calls.back()=="destroy","retry reopened prior stop stages");
 });
 check("EDG off failure skips destroy and retries only remaining teardown",[]{
   Harness h;h.servo_pending_=true;mock::edg_rc=-1;
   expect(h.on_shutdown({})==CB::ERROR,"EDG off failure accepted");
   expect(mock::calls==std::vector<std::string>{"abort","disable","read_servo","edg_off"},"destroy ran before EDG off confirmed");
   mock::edg_rc=0;expect(h.on_shutdown({})==CB::SUCCESS,"EDG off retry failed");
   expect(mock::calls==std::vector<std::string>{"abort","disable","read_servo","edg_off","edg_off","destroy"},"wrong EDG retry sequence");
 });
 check("worker exits before EDG session teardown",[]{
   Harness h;h.servo_pending_=true;h.running_=true;mock::require_worker_joined=true;
   h.worker_=std::thread([&]{while(h.running_)std::this_thread::yield();mock::worker_left=true;});
   expect(h.on_shutdown({})==CB::SUCCESS,"worker join shutdown failed");
   expect(!h.worker_.joinable()&&mock::worker_left,"worker not joined");
 });
 check("worker accepts every get_stat cache advance with one post-read stamp",[]{
   Harness h;h.running_=true;mock::edg_fail_after=4;h.worker();
   expect(h.fault_&&h.edg_audit_.new_count()==4,"normal SDK cache advances were discarded");
   expect(h.edg_audit_.count()==5&&h.edg_audit_.last()->admission==EdgAdmission::SdkError,"SDK error did not latch");
   expect(h.measured_[0]==.004&&h.feedback_at_>0,"accepted feedback not committed");
   std::vector<std::string> expected;
   for(int i=0;i<4;++i){expected.push_back("get_stat");expected.push_back("details");}
   expected.push_back("get_stat");
   expect(mock::calls==expected,"worker used a pre-get details query or queried after SDK error");
   expect(h.on_error({})==CB::SUCCESS,"worker error cleanup failed");
 });
 check("worker duplicate source cannot refresh host freshness",[]{
   Harness h;h.running_=true;mock::edg_duplicate=true;h.worker();
   expect(h.fault_&&h.edg_audit_.new_count()==1&&h.edg_audit_.duplicate_count()>0,"duplicate source accepted");
   expect(h.edg_audit_.last()->admission==EdgAdmission::Expired,"duplicate timeout not enforced");
   expect(h.on_error({})==CB::SUCCESS,"duplicate failure cleanup failed");
 });
 check("worker slow SDK return cannot refresh host freshness",[]{
   Harness h;h.running_=true;mock::edg_read_delay_ms=30;h.worker();
   expect(h.fault_&&h.edg_audit_.new_count()==0&&h.feedback_at_==0,"slow return refreshed feedback");
   expect(h.edg_audit_.last()->admission==EdgAdmission::SlowCall,"slow SDK return not classified");
   expect(h.on_error({})==CB::SUCCESS,"slow failure cleanup failed");
 });
 check("worker nonfinite sample cannot reach measured state",[]{
   Harness h;h.running_=true;mock::edg_nonfinite=true;h.worker();
   expect(h.fault_&&h.edg_audit_.new_count()==0&&h.feedback_at_==0,"nonfinite sample accepted");
   expect(h.edg_audit_.last()->admission==EdgAdmission::InvalidSample,"nonfinite sample not classified");
   expect(h.measured_[0]==0.,"nonfinite value entered measured state");
   expect(h.on_error({})==CB::SUCCESS,"nonfinite failure cleanup failed");
 });
 std::cout<<"TOTAL "<<cases<<" cases; SDK stub only, no robot/ROS session\n";
 return 0;
 }catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
