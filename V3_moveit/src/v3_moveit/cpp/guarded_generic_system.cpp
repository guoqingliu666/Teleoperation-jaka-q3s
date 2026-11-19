// Test-only sink for final ros2_control position commands. No SDK is linked.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <vector>
#include <mock_components/generic_system.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "v3_moveit/guard.hpp"

namespace v3_moveit {
namespace {
std::string json_string(const std::string& text) {
  std::string result="\"";
  const char hex[]="0123456789abcdef";
  for (unsigned char c:text) {
    if(c=='"'||c=='\\'){result+='\\';result+=static_cast<char>(c);}
    else if(c<0x20){result+="\\u00";result+=hex[c>>4];result+=hex[c&15];}
    else result+=static_cast<char>(c);
  }
  return result+'"';
}
void array_json(std::ostream& out,const Joints& values) {
  out<<'[';
  for(size_t i=0;i<values.size();++i){if(i)out<<',';if(std::isfinite(values[i]))out<<values[i];else out<<"null";}
  out<<']';
}
// The record store is allocated and value-initialized only in on_configure.
// No formatting, file I/O, or growing container is used in the write callback.
enum class AuditResult { Accepted, StartupNoCommand, Rejected };
constexpr size_t kAuditCapacity=30000;  // 240 seconds at the nominal 125 Hz.
constexpr size_t kReasonCapacity=192;
using AuditReason=std::array<char,kReasonCapacity>;
void set_reason(AuditReason& output,const char* text) noexcept {
  size_t i=0;
  for(;i+1<output.size()&&text[i]!='\0';++i)output[i]=text[i];
  output[i]='\0';
}
struct AuditRecord {
  uint64_t index{0};
  int64_t time_ns{0},period_ns{0},host_ns{0};
  double period_s{0.};
  Joints command{},state{};
  Guard before{};
  AuditResult result{AuditResult::Accepted};
  bool first_rejection{false};
  AuditReason reason{};
};
const char* result_name(AuditResult result) noexcept {
  switch(result){
    case AuditResult::Accepted:return "accepted";
    case AuditResult::StartupNoCommand:return "startup_no_command";
    default:return "rejected";
  }
}
}  // namespace

class GuardedGenericSystem final : public mock_components::GenericSystem {
 public:
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params) override {
    if(GenericSystem::on_init(params)!=hardware_interface::CallbackReturn::SUCCESS)
      return hardware_interface::CallbackReturn::ERROR;
    try {
      const auto& info=get_hardware_info();
      const auto& p=info.hardware_parameters;
      if(p.at("allow_hardware")!="false")throw std::runtime_error("GuardedGenericSystem is fake-only; allow_hardware=false required");
      if(info.joints.size()!=6)throw std::runtime_error("six ordered joints required");
      audit_path_=p.at("audit_path");
      if(!std::filesystem::path(audit_path_).is_absolute())throw std::runtime_error("audit_path must be absolute");
      auto strict=p.find("strict");
      if(strict!=p.end()){
        if(strict->second!="true"&&strict->second!="false")throw std::runtime_error("strict must be true or false");
        strict_=strict->second=="true";
      }
      for(size_t i=0;i<6;++i){
        if(info.joints[i].name!="joint_"+std::to_string(i+1))throw std::runtime_error("joint order mismatch");
        names_[i]=info.joints[i].name+"/position";
        guard_.lower[i]=std::stod(p.at("lower_"+std::to_string(i)));
        guard_.upper[i]=std::stod(p.at("upper_"+std::to_string(i)));
      }
      return hardware_interface::CallbackReturn::SUCCESS;
    }catch(const std::exception& error){
      RCLCPP_ERROR(get_logger(),"Test plugin initialization: %s",error.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous) override {
    if(GenericSystem::on_configure(previous)!=hardware_interface::CallbackReturn::SUCCESS)
      return hardware_interface::CallbackReturn::ERROR;
    try {
      Joints initial{};
      for(size_t i=0;i<6;++i){
        commands_[i]=get_command_interface_handle(names_[i]);
        states_[i]=get_state_interface_handle(names_[i]);
        if(!get_state(states_[i],initial[i],false)||!std::isfinite(initial[i]))
          throw std::runtime_error("initial position state is unavailable or nonfinite");
      }
      // Lifecycle configuration is outside the controller write loop. Finish any
      // previous session before resetting its bounded storage and counters.
      if(session_open_&&!flush_session())throw std::runtime_error("cannot finalize previous test audit");
      if(audit_.is_open())audit_.close();
      records_.assign(kAuditCapacity,AuditRecord{});  // Allocate and touch pages here.
      // Preserve the initial state as the stationary baseline, including for the first real target.
      guard_.reset(initial);
      seen_command_=false;latched_=false;overflow_=false;recorded_=0;
      writes_=0;rejections_=0;startup_writes_=0;unrecorded_writes_=0;first_reason_.fill('\0');
      audit_.open(audit_path_,std::ios::app);
      if(!audit_)throw std::runtime_error("cannot open test audit_path");
      audit_<<std::setprecision(17)<<"{\"event\":\"session_start\",\"strict\":"<<(strict_?"true":"false")
            <<",\"mode\":"<<json_string(strict_?"strict":"audit_only")<<",\"initial_state\":";
      array_json(audit_,initial);
      audit_<<",\"guard_reference\":\"last_accepted_command\",\"sdk_calls\":0"
            <<",\"audit_storage\":\"bounded_preallocated_memory\",\"audit_capacity\":"<<kAuditCapacity
            <<",\"audit_record_bytes\":"<<sizeof(AuditRecord)<<"}\n";
      audit_.flush();
      session_open_=static_cast<bool>(audit_);
      return audit_?hardware_interface::CallbackReturn::SUCCESS:hardware_interface::CallbackReturn::ERROR;
    }catch(const std::exception& error){
      RCLCPP_ERROR(get_logger(),"Test plugin configuration: %s",error.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  hardware_interface::return_type write(const rclcpp::Time& time,const rclcpp::Duration& period) override {
    // A full store has already failed closed. Count unexpected later invocations
    // without allocating more memory or pretending their samples were retained.
    if(overflow_){++writes_;++rejections_;++unrecorded_writes_;return hardware_interface::return_type::ERROR;}
    const bool full=recorded_==records_.size();
    AuditRecord& record=full?overflow_record_:records_[recorded_++];
    record=AuditRecord{};
    record.index=writes_++;
    record.time_ns=time.nanoseconds();record.period_ns=period.nanoseconds();record.period_s=period.seconds();
    record.host_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    record.command.fill(std::numeric_limits<double>::quiet_NaN());record.state=record.command;
    record.before=guard_;
    bool readable=true;
    for(size_t i=0;i<6;++i){
      const bool command_ok=get_command(commands_[i],record.command[i],false);
      const bool state_ok=get_state(states_[i],record.state[i],false);
      readable=readable&&command_ok&&state_ok;
    }
    const bool state_finite=std::all_of(record.state.begin(),record.state.end(),[](double x){return std::isfinite(x);});
    const bool startup=readable&&state_finite&&!seen_command_&&
        std::all_of(record.command.begin(),record.command.end(),[](double x){return std::isnan(x);});
    auto reject=[&record](const char* reason){record.result=AuditResult::Rejected;set_reason(record.reason,reason);};
    if(full){overflow_=true;reject("test audit capacity exhausted; no further commands admitted");}
    else if(strict_&&latched_)reject("previous guard rejection latched");
    else if(!readable)reject("command or state interface read failed");
    else if(startup){record.result=AuditResult::StartupNoCommand;++startup_writes_;}
    else {
      seen_command_=true;
      try{
        const double delivery_dt=record.period_s;
        if(!std::isfinite(delivery_dt)||delivery_dt<.004||delivery_dt>.012)
          throw std::runtime_error("invalid hardware delivery interval");
        // Match the production EDG step_num=1 target clock; retain the raw
        // hardware interval in the audit and enforce it independently above.
        guard_.check(record.command,record.state,.008);
      }
      catch(const std::exception& error){reject(error.what());}
    }
    const bool rejected=record.result==AuditResult::Rejected;
    record.first_rejection=rejected&&first_reason_[0]=='\0';
    if(rejected){
      ++rejections_;latched_=true;
      if(record.first_rejection)first_reason_=record.reason;
    }
    // Defer even first-rejection logging to teardown: logging can block on disk.
    // Guard exceptions remain the unchanged validation failure path; the audit
    // itself stores fixed-size data and performs no heap allocation in write().
    if(full||(strict_&&rejected))return hardware_interface::return_type::ERROR;
    // Audit-only preserves raw commands and GenericSystem feedback after a rejection.
    // Guard history remains exactly as implemented by Guard: the last accepted command.
    return GenericSystem::write(time,period);
  }

  ~GuardedGenericSystem() override {
    try {
      if(session_open_&&!flush_session())RCLCPP_ERROR(get_logger(),"Test audit finalization failed");
    }catch(const std::exception& error){
      RCLCPP_ERROR(get_logger(),"Test audit finalization failed: %s",error.what());
    }
  }

 private:
  void serialize_record(const AuditRecord& record) {
    audit_<<"{\"event\":\"write\",\"index\":"<<record.index<<",\"time_ns\":"<<record.time_ns
          <<",\"command_period_s\":0.008,\"period_ns\":"<<record.period_ns<<",\"period_s\":"<<record.period_s
          <<",\"host_monotonic_ns\":"<<record.host_ns<<",\"strict\":"<<(strict_?"true":"false")
          <<",\"result\":"<<json_string(result_name(record.result))
          <<",\"reason\":"<<json_string(record.reason.data())<<",\"first_rejection\":"<<(record.first_rejection?"true":"false")
          <<",\"command\":";array_json(audit_,record.command);
    audit_<<",\"state\":";array_json(audit_,record.state);
    audit_<<",\"guard_previous\":";array_json(audit_,record.before.previous);
    audit_<<",\"guard_previous_velocity\":";array_json(audit_,record.before.previous_velocity);
    audit_<<",\"guard_previous_acceleration\":";array_json(audit_,record.before.previous_acceleration);
    audit_<<",\"guard_previous_dt\":"<<record.before.previous_dt
          <<",\"guard_previous_previous_dt\":"<<record.before.previous_previous_dt<<"}\n";
  }

  bool flush_session() {
    for(size_t i=0;i<recorded_;++i)serialize_record(records_[i]);
    if(overflow_)serialize_record(overflow_record_);
    audit_<<"{\"event\":\"session_end\",\"writes\":"<<writes_<<",\"rejections\":"<<rejections_
          <<",\"startup_writes\":"<<startup_writes_<<",\"first_rejection\":"<<json_string(first_reason_.data())
          <<",\"audit_overflow\":"<<(overflow_?"true":"false")
          <<",\"recorded_writes\":"<<(recorded_+(overflow_?1:0))
          <<",\"unrecorded_write_attempts\":"<<unrecorded_writes_<<"}\n";
    audit_.flush();session_open_=false;
    if(first_reason_[0]!='\0')RCLCPP_ERROR(get_logger(),"Final write Guard rejected: %s (%s)",first_reason_.data(),strict_?"strict":"audit_only");
    return static_cast<bool>(audit_);
  }

  Guard guard_;
  std::array<std::string,6> names_;
  std::array<hardware_interface::CommandInterface::SharedPtr,6> commands_;
  std::array<hardware_interface::StateInterface::SharedPtr,6> states_;
  std::string audit_path_;
  AuditReason first_reason_{};
  std::ofstream audit_;
  std::vector<AuditRecord> records_;
  AuditRecord overflow_record_{};
  bool strict_{true},seen_command_{false},latched_{false},session_open_{false},overflow_{false};
  size_t recorded_{0};
  uint64_t writes_{0},rejections_{0},startup_writes_{0},unrecorded_writes_{0};
};
}  // namespace v3_moveit
PLUGINLIB_EXPORT_CLASS(v3_moveit::GuardedGenericSystem,hardware_interface::SystemInterface)
