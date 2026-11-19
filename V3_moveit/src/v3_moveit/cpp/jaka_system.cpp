// SDK 单所有者；控制器插值与 Servo/OMPL 计算均不在这个线程里执行。
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "v3_moveit/jaka_sdk.hpp"
#include "v3_moveit/guard.hpp"
#include "v3_moveit/edg_feedback.hpp"

namespace v3_moveit {
using Clock=std::chrono::steady_clock;
static double seconds() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }
using CB=hardware_interface::CallbackReturn;
class JakaSystem : public hardware_interface::SystemInterface {
 public:
  CB on_init(const hardware_interface::HardwareInfo& info) override {
    if (SystemInterface::on_init(info)!=CB::SUCCESS || info.joints.size()!=6) return CB::ERROR;
    try {
      auto p=info.hardware_parameters;
      if (p.at("allow_hardware")!="true") throw std::runtime_error("hardware explicitly disabled");
      ip_=p.at("robot_ip"); client_=p.at("edg_client_ip"); port_=std::stoi(p.at("edg_port"));
      if(const char* path=std::getenv("V3_EDG_AUDIT_PATH")){
        edg_audit_path_=path;
        if(edg_audit_path_.empty()||edg_audit_path_.front()!='/')
          throw std::runtime_error("V3_EDG_AUDIT_PATH must be an absolute project-drive path");
      }
      mass_=std::stod(p.at("payload_kg"));
      tool_id_=std::stoi(p.at("tool_id"));
      if(tool_id_<0||tool_id_>10)throw std::runtime_error("invalid configured tool id");
      for(size_t i=0;i<6;++i){tool_tcp_[i]=std::stod(p.at("tool_tcp_"+std::to_string(i)));if(!std::isfinite(tool_tcp_[i]))throw std::runtime_error("invalid configured TCP");}
      if(!std::isfinite(mass_)||mass_<0)throw std::runtime_error("invalid configured payload");
      for (size_t i=0;i<3;++i) {
        cog_[i]=std::stod(p.at("cog_"+std::to_string(i)));
        if(!std::isfinite(cog_[i]))throw std::runtime_error("invalid configured centroid");
      }
      for(size_t i=0;i<6;++i) {
        if(info.joints[i].name!="joint_"+std::to_string(i+1)) throw std::runtime_error("joint order mismatch");
        guard_.lower[i]=std::stod(p.at("lower_"+std::to_string(i)));
        guard_.upper[i]=std::stod(p.at("upper_"+std::to_string(i)));
        fk_q_[i]=std::stod(p.at("fk_q_"+std::to_string(i)));fk_tcp_[i]=std::stod(p.at("fk_tcp_"+std::to_string(i)));
        if(!std::isfinite(guard_.lower[i])||!std::isfinite(guard_.upper[i])||guard_.lower[i]>=guard_.upper[i]||
           !std::isfinite(fk_q_[i])||!std::isfinite(fk_tcp_[i]))throw std::runtime_error("invalid configured model");
      }
      node_=rclcpp::Node::make_shared("jaka_hardware_watchdog");
      lease_sub_=node_->create_subscription<std_msgs::msg::Bool>("/v3/motion_lease",rclcpp::QoS(1),
        [this](std_msgs::msg::Bool::ConstSharedPtr m){ lease_ok_=m->data; lease_at_=seconds(); });
      executor_.add_node(node_);
      executor_thread_=std::thread([this]{executor_.spin();});
      return CB::SUCCESS;
    } catch(const std::exception& e){RCLCPP_ERROR(rclcpp::get_logger("v3_jaka"),"%s",e.what());return CB::ERROR;}
  }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> out;
    for(size_t i=0;i<6;++i){out.emplace_back(info_.joints[i].name,"position",&state_q_[i]);out.emplace_back(info_.joints[i].name,"velocity",&state_v_[i]);} return out;
  }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> out;
    for(size_t i=0;i<6;++i)out.emplace_back(info_.joints[i].name,"position",&command_[i]);return out;
  }
  static void checked(int c,const char* name){if(c)throw std::runtime_error(std::string(name)+" code="+std::to_string(c));}
  void preflight() {
    MotionStatus m{};RobotStatus_simple s{};BOOL b=FALSE;int tool=0,level=0;CartesianPose tcp{};PayLoad load{};
    checked(get_motion_status(&handle_,&m),"motion status");checked(get_robot_status_simple(&handle_,&s),"robot status");
    if(m.isInEstop||m.isInCollision||m.isOnLimit||s.errcode||s.powered_on!=1||s.enabled!=1)throw std::runtime_error("robot not ready");
    checked(is_in_servomove(&handle_,&b),"servo state");if(b!=FALSE)throw std::runtime_error("another servo owner exists");
    checked(is_in_pos(&handle_,&b),"motion completion");if(b!=TRUE)throw std::runtime_error("robot motion not complete");
    if(m.queue!=0||m.active_queue!=0||m.paused!=FALSE)throw std::runtime_error("motion queue or pause state prevents takeover");
    ProgramState program=PROGRAM_RUNNING;
    checked(get_program_state(&handle_,&program),"program state");
    if(program!=PROGRAM_IDLE)throw std::runtime_error("controller program is not idle");
    b=TRUE;checked(is_in_drag_mode(&handle_,&b),"drag state");
    if(b!=FALSE)throw std::runtime_error("drag mode prevents takeover");
    checked(get_tool_id(&handle_,&tool),"tool id");if(tool!=tool_id_)throw std::runtime_error("TOOL_MISMATCH: active cabinet tool differs from V3 selection");
    checked(get_tool_data(&handle_,tool_id_,&tcp),"selected TCP");
    const double* t=reinterpret_cast<const double*>(&tcp);
    for(size_t i=0;i<6;++i)if(!std::isfinite(t[i])||std::abs(t[i]-tool_tcp_[i])>.001)throw std::runtime_error("TCP_MISMATCH: cabinet TCP definition changed");
    checked(get_payload(&handle_,&load),"payload");
    auto check_payload=[](double cabinet,double configured,double tolerance,const std::string& field,const char* unit,const char* error){
      if(std::isfinite(cabinet)&&std::abs(cabinet-configured)<=tolerance)return;
      std::ostringstream detail;
      detail<<std::setprecision(12)<<error<<": 控制柜"<<field<<' '<<cabinet<<' '<<unit<<"，V3 配置 "<<configured<<' '<<unit<<"。";
      throw std::runtime_error(detail.str());
    };
    check_payload(load.mass,mass_,1e-6,"负载","kg","PAYLOAD_MISMATCH");
    const double* c=reinterpret_cast<const double*>(&load.centroid);
    for(size_t i=0;i<3;++i)check_payload(c[i],cog_[i],.001,std::string("质心 ")+"XYZ"[i],"mm","PAYLOAD_CENTROID_MISMATCH");
    checked(get_collision_level(&handle_,&level),"collision level");if(level<1||level>5)throw std::runtime_error("cabinet collision protection disabled");
    JointValue sample{};std::copy(fk_q_.begin(),fk_q_.end(),sample.jVal);CartesianPose actual{};
    checked(kine_forward(&handle_,&sample,&actual),"official FK model check");
    const double* pose=reinterpret_cast<const double*>(&actual);
    for(size_t i=0;i<6;++i){if(!std::isfinite(pose[i]))throw std::runtime_error("nonfinite official FK");double error=pose[i]-fk_tcp_[i];if(i>=3)error=std::atan2(std::sin(error),std::cos(error));
      if(std::abs(error)>(i<3?.1:.001))throw std::runtime_error("installed model does not match current controller FK/TCP");}
  }
  CB on_configure(const rclcpp_lifecycle::State&) override {
    try {
      if(logged_||servo_pending_)throw std::runtime_error("previous SDK session has not completed shutdown");
      edg_audit_.reset();edg_audit_flushed_=false;
      stop_requested_=false;fault_=false;ever_lease_=false;lease_ok_=false;lease_at_=0;safety_at_=0;feedback_at_=0;previous_send_=0;
      checked(create_handler(ip_.c_str(),&handle_),"create handler");logged_=true;
      preflight(); JointValue q{};checked(get_actual_joint_position(&handle_,&q),"initial joints");
      for(size_t i=0;i<6;++i)if(!std::isfinite(q.jVal[i])||q.jVal[i]<guard_.lower[i]||q.jVal[i]>guard_.upper[i])
        throw std::runtime_error("initial joints invalid or outside configured limits");
      std::copy(q.jVal,q.jVal+6,state_q_.begin());measured_=state_q_;command_=state_q_;latest_command_=state_q_;guard_.reset(state_q_);
      checked(set_network_exception_handle(&handle_,100,static_cast<ProcessType>(2)),"network watchdog");
      edg_=true; // An unsuccessful reply does not prove the EDG request was rejected.
      checked(edg_init_extend(&handle_,TRUE,client_.c_str(),port_,0),"EDG start");
      running_=true; fault_=false; worker_=std::thread([this]{worker();});poller_=std::thread([this]{poll();});
      return CB::SUCCESS;
    }catch(const std::exception& e){fail(e.what());shutdown();return CB::ERROR;}
  }
  CB on_activate(const rclcpp_lifecycle::State&) override {
    try {
      // These are takeover thresholds, not a proof of physical stopping time or distance.
      // Require new, fresh EDG samples for 0.25 s within a 2 s warmup budget.
      double until=seconds()+2.,quiet_since=0.,last_sample=0.;bool quiet_ready=false;Joints quiet_reference{};
      while(seconds()<until&&!fault_&&!stop_requested_){
        Joints q{},v{};double sample_at;
        {std::lock_guard<std::mutex> lock(data_);q=measured_;v=velocity_;sample_at=feedback_at_;}
        double now=seconds();
        if(sample_at<=0||now-sample_at>=.024||now-safety_at_>=.15)quiet_since=0.;
        else if(sample_at>last_sample){
          bool slow=true,drift=false;
          for(size_t i=0;i<6;++i){
            if(!std::isfinite(q[i])||!std::isfinite(v[i])||std::abs(v[i])>.005)slow=false;
            if(quiet_since>0&&std::abs(q[i]-quiet_reference[i])>.001)drift=true;
          }
          if(!slow)quiet_since=0.;
          else {
            if(quiet_since==0.||drift){quiet_since=sample_at;quiet_reference=q;}
            if(sample_at-quiet_since>=.25){quiet_ready=true;break;}
          }
        }
        last_sample=sample_at;
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
      }
      if(!quiet_ready||fault_||stop_requested_)throw std::runtime_error("activation requires 0.25 s of fresh stationary feedback");
      {
        std::lock_guard<std::mutex> lock(rpc_);
        if(stop_requested_||fault_||seconds()-safety_at_>=.15)throw std::runtime_error("stop requested or safety stale before servo entry");
        {
          std::lock_guard<std::mutex> data_lock(data_);
          if(seconds()-feedback_at_>=.024)throw std::runtime_error("feedback stale before servo entry");
          for(size_t i=0;i<6;++i)if(!std::isfinite(measured_[i])||!std::isfinite(velocity_[i])||
              std::abs(velocity_[i])>.005||std::abs(measured_[i]-quiet_reference[i])>.001)
            throw std::runtime_error("robot no longer stationary before servo entry");
          state_q_=measured_;command_=measured_;initial_=measured_;guard_.reset(measured_);
        }
        // A failed reply does not prove the cabinet rejected the enable request.
        servo_pending_=true;
        checked(servo_move_enable(&handle_,TRUE),"enter servo");
        BOOL enabled=FALSE;checked(is_in_servomove(&handle_,&enabled),"verify servo");
        if(enabled!=TRUE||stop_requested_||fault_)throw std::runtime_error("servo entry not confirmed or stop requested");
      }
      previous_send_=0.;active_=true;return CB::SUCCESS;
    }catch(const std::exception& e){fail(e.what());shutdown();return CB::ERROR;}
  }
  CB on_deactivate(const rclcpp_lifecycle::State&) override {return shutdown()?CB::SUCCESS:CB::ERROR;}
  CB on_cleanup(const rclcpp_lifecycle::State&) override {return shutdown()?CB::SUCCESS:CB::ERROR;}
  CB on_error(const rclcpp_lifecycle::State&) override {return shutdown()?CB::SUCCESS:CB::ERROR;}
  CB on_shutdown(const rclcpp_lifecycle::State&) override {return shutdown()?CB::SUCCESS:CB::ERROR;}
  hardware_interface::return_type read(const rclcpp::Time&,const rclcpp::Duration&) override {
    if(fault_)return hardware_interface::return_type::ERROR;
    std::lock_guard<std::mutex> lock(data_);state_q_=measured_;state_v_=velocity_;
    if(active_ && feedback_at_>0 && seconds()-feedback_at_>.024){fail("EDG feedback stale");return hardware_interface::return_type::ERROR;}
    return hardware_interface::return_type::OK;
  }
  hardware_interface::return_type write(const rclcpp::Time&,const rclcpp::Duration& period) override {
    if(fault_)return hardware_interface::return_type::ERROR;
    if(!active_)return hardware_interface::return_type::OK;
    try {
      double now=seconds();bool lease=lease_ok_&&now-lease_at_<.08;
      if(lease)ever_lease_=true;
      if(ever_lease_&&!lease)throw std::runtime_error("supervisor lease lost");
      if(now-safety_at_>.15)throw std::runtime_error("safety status expired");
      Joints actual;double fresh;
      {std::lock_guard<std::mutex> lock(data_);actual=measured_;fresh=feedback_at_;}
      if(now-fresh>.024)throw std::runtime_error("feedback expired before send");
      if(previous_send_>0&&now-previous_send_>.012)throw std::runtime_error("controller missed send deadline");
      // Validate delivery timing separately from the target sequence. The installed
      // controller advances one 8 ms target per update, matching EDG step_num=1.
      // ros2_control measures controller and hardware periods independently.
      const double delivery_dt=period.seconds();
      if(!std::isfinite(delivery_dt)||delivery_dt<.004||delivery_dt>.012)
        throw std::runtime_error("invalid hardware delivery interval");
      Joints target=lease?command_:initial_;
      guard_.check(target,actual,.008);
      JointValue cmd{};std::copy(target.begin(),target.end(),cmd.jVal);
      checked(edg_servo_j_extend(&handle_,&cmd,ABS,1,0),"EDG joint target");previous_send_=now;
      if(seconds()-now>.012)throw std::runtime_error("SDK send overrun");
      return hardware_interface::return_type::OK;
    }catch(const std::exception& e){fail(e.what());return hardware_interface::return_type::ERROR;}
  }
  ~JakaSystem() override {
    if(!shutdown())RCLCPP_ERROR(rclcpp::get_logger("v3_jaka"),"Hardware shutdown unconfirmed; no software stop-time guarantee");
    executor_.cancel();if(executor_thread_.joinable())executor_thread_.join();
  }
 private:
  void fail(const std::string& reason){
    stop_requested_=true;active_=false;
    if(!fault_.exchange(true))RCLCPP_ERROR(rclcpp::get_logger("v3_jaka"),"Latched stop: %s",reason.c_str());
  }
  bool stop_servo() {
    if(!servo_pending_)return true;
    // The current RPC cannot be cancelled by this SDK. This lock preserves one
    // control owner; three attempts bound retry count, NOT elapsed stop time.
    const double wait_start=seconds();
    std::lock_guard<std::mutex> lock(rpc_);
    RCLCPP_INFO(rclcpp::get_logger("v3_jaka"),"Stop RPC ownership wait_ms=%.3f",(seconds()-wait_start)*1000.);
    for(int attempt=1;attempt<=3;++attempt){
      const double abort_start=seconds();int a=motion_abort(&handle_);const double abort_end=seconds();
      int b=servo_move_enable(&handle_,FALSE);const double disable_end=seconds();BOOL state=TRUE;
      int c=is_in_servomove(&handle_,&state);const double query_end=seconds();
      RCLCPP_INFO(rclcpp::get_logger("v3_jaka"),
        "Stop RPC attempt=%d abort_ms=%.3f disable_ms=%.3f query_ms=%.3f rc=%d/%d/%d servo=%d",
        attempt,(abort_end-abort_start)*1000.,(disable_end-abort_end)*1000.,(query_end-disable_end)*1000.,a,b,c,state);
      if(!a&&!b&&!c&&state==FALSE){
        servo_pending_=false;
        RCLCPP_INFO(rclcpp::get_logger("v3_jaka"),"Servo exit confirmed; physical stop distance is a separate measurement");
        return true;
      }
      RCLCPP_ERROR(rclcpp::get_logger("v3_jaka"),"STOP NOT CONFIRMED attempt=%d/3: abort=%d disable=%d read=%d state=%d",attempt,a,b,c,state);
    }
    fail("servo exit unconfirmed; retain SDK ownership for a later stop retry");
    return false;
  }
  void poll() {
    try {
      while(running_&&!fault_){
        MotionStatus m{};RobotStatus_simple s{};
        double start=seconds();
        {
          std::lock_guard<std::mutex> lock(rpc_);
          if(stop_requested_||!running_||fault_)break;
          checked(get_motion_status(&handle_,&m),"safety motion");
          // Do not enter a second RPC after a stop request arrived during the first.
          if(stop_requested_||!running_||fault_)break;
          checked(get_robot_status_simple(&handle_,&s),"safety power");
        }
        if(m.isInEstop||m.isInCollision||m.isOnLimit||s.errcode||s.powered_on!=1||s.enabled!=1)throw std::runtime_error("raw cabinet fault");
        safety_at_=start; // 查询开始时间，不用返回时间掩盖 SDK 长等待。
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
      }
    }catch(const std::exception& e){fail(e.what());}
  }
  void worker() {
    EdgFreshness freshness;freshness.reset(seconds());
    try {
      while(running_&&!fault_){
        EDGState state{};EdgAuditSample sample{};
        sample.previous_accepted_at=freshness.accepted_at();sample.begin=seconds();
        // This is the ONLY decoded-cache reader. The installed SDK's get_stat
        // calls recv(), then copies state. details() reads that same decoded
        // cache; UDP reception only queues data and edg_servo_j only writes the
        // separate command buffer. Requiring pre-get == post-get wrongly drops
        // each normal cache advance. See docs/edg_cache_contract.md and SDK hash.
        sample.state_rc=edg_get_stat(&handle_,&state,0);sample.state_end=seconds();
        sample.details_rc=sample.state_rc?std::numeric_limits<int>::min():
          edg_stat_details(&handle_,sample.stamp.data());sample.end=seconds();
        std::copy(state.jointVal.jVal,state.jointVal.jVal+6,sample.position.begin());
        std::copy(state.jointVel.jVel,state.jointVel.jVel+6,sample.velocity.begin());
        bool finite=true;
        for(size_t i=0;i<6;++i)if(!std::isfinite(sample.position[i])||!std::isfinite(sample.velocity[i]))finite=false;
        if(sample.state_rc||sample.details_rc)sample.admission=EdgAdmission::SdkError;
        else if(!finite)sample.admission=EdgAdmission::InvalidSample;
        else sample.admission=freshness.observe(sample.stamp,sample.begin,sample.end);
        edg_audit_.push(sample); // Fixed-size memory only; no logging/allocation/I/O.
        checked(sample.state_rc,"EDG state");checked(sample.details_rc,"EDG stamp after state");
        if(sample.admission==EdgAdmission::NewSample){
          std::lock_guard<std::mutex> lock(data_);
          measured_=sample.position;velocity_=sample.velocity;feedback_at_=freshness.accepted_at();
        }else if(sample.admission!=EdgAdmission::Duplicate){
          throw std::runtime_error(std::string("EDG admission: ")+edg_admission_name(sample.admission));
        }
        if(active_&&seconds()-safety_at_>.15)throw std::runtime_error("safety status expired");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }catch(const std::exception& e){fail(e.what());}
    // 停止由 controller_manager 生命周期串行处理；不在反馈线程并发发送运动 RPC。
  }
  void flush_edg_audit() {
    if(edg_audit_flushed_.exchange(true)||!edg_audit_.count())return;
    const auto* last=edg_audit_.last();
    RCLCPP_INFO(rclcpp::get_logger("v3_jaka"),
      "EDG audit after worker join: total=%lu new=%lu duplicates=%lu last=%s stamp=%lu/%lu/%lu get_ms=%.3f details_ms=%.3f prior_age_ms=%.3f",
      static_cast<unsigned long>(edg_audit_.count()),static_cast<unsigned long>(edg_audit_.new_count()),
      static_cast<unsigned long>(edg_audit_.duplicate_count()),edg_admission_name(last->admission),
      last->stamp[0],last->stamp[1],last->stamp[2],(last->state_end-last->begin)*1000.,
      (last->end-last->state_end)*1000.,(last->end-last->previous_accepted_at)*1000.);
    if(edg_audit_path_.empty())return;
    try {
      std::ofstream out;out.exceptions(std::ios::failbit|std::ios::badbit);
      out.open(edg_audit_path_);edg_audit_.write_jsonl(out);out.close();
      RCLCPP_INFO(rclcpp::get_logger("v3_jaka"),"Stopped EDG audit written: %s",edg_audit_path_.c_str());
    }catch(const std::exception& error){
      RCLCPP_ERROR(rclcpp::get_logger("v3_jaka"),"Stopped EDG audit write failed: %s",error.what());
    }
  }
  bool shutdown(){
    stop_requested_=true;active_=false;running_=false;
    // Request the serialized stop before joining workers. In-flight SDK calls
    // still have to return; neither this ordering nor the watchdog proves a bound.
    bool stopped=stop_servo();
    if(worker_.joinable())worker_.join();
    if(poller_.joinable())poller_.join();
    flush_edg_audit();
    if(!stopped)return false; // Keep the handle and pending state for a later retry.
    if(edg_){
      int code=edg_init_extend(&handle_,FALSE,client_.c_str(),port_,0);
      if(code){fail("EDG shutdown code="+std::to_string(code));return false;}
      edg_=false;
    }
    if(logged_){
      int code=destory_handler(&handle_);
      if(code){fail("destroy handler code="+std::to_string(code));return false;}
      logged_=false;
    }
    return true;
  }
  // Pending starts before enable is sent and ends only after a confirmed stop.
  JKHD handle_{};bool logged_{false},edg_{false},servo_pending_{false},ever_lease_{false};std::string ip_,client_;int port_{30003};double mass_{};std::array<double,3> cog_{};
  int tool_id_{1};std::array<double,6> tool_tcp_{{0,0,200,0,0,0}};
  Joints state_q_{},state_v_{},command_{},measured_{},velocity_{},latest_command_{},initial_{},fk_q_{},fk_tcp_{};Guard guard_;
  EdgAuditRing edg_audit_;std::atomic<bool> edg_audit_flushed_{false};std::string edg_audit_path_;
  std::mutex data_,rpc_;double feedback_at_{0},previous_send_{0};
  std::atomic<bool> active_{false},running_{false},fault_{false},lease_ok_{false},stop_requested_{false};
  std::atomic<double> lease_at_{0},safety_at_{0};std::thread worker_,poller_,executor_thread_;
  rclcpp::Node::SharedPtr node_;rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lease_sub_;
};
}
PLUGINLIB_EXPORT_CLASS(v3_moveit::JakaSystem,hardware_interface::SystemInterface)
