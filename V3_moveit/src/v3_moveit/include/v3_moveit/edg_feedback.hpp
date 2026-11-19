#pragma once
// Single-reader admission for the installed JAKA 1.7.2 decoded EDG cache.
// get_stat() drains/decodes receive data; details() then describes that same
// cache. A pre-get stamp is intentionally NOT compared with the post-get stamp.
// SDK identity and static call-path evidence: docs/edg_cache_contract.md.
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ostream>

namespace v3_moveit {
enum class EdgAdmission { NewSample, Duplicate, InvalidClock, InvalidStamp,
                          RegressedStamp, Expired, SlowCall, InvalidSample, SdkError };
inline const char* edg_admission_name(EdgAdmission value) {
  switch(value) {
    case EdgAdmission::NewSample:return "new";
    case EdgAdmission::Duplicate:return "duplicate";
    case EdgAdmission::InvalidClock:return "invalid_host_clock";
    case EdgAdmission::InvalidStamp:return "invalid_source_stamp";
    case EdgAdmission::RegressedStamp:return "source_regressed";
    case EdgAdmission::Expired:return "feedback_expired";
    case EdgAdmission::SlowCall:return "SDK_sample_call_exceeded_24ms";
    case EdgAdmission::InvalidSample:return "nonfinite_feedback";
    case EdgAdmission::SdkError:return "SDK_error";
  }
  return "unknown";
}

class EdgFreshness {
 public:
  static constexpr double deadline_s=.024;
  void reset(double started) {started_=started;accepted_at_=0.;have_sample_=false;stamp_={};}
  EdgAdmission observe(const std::array<unsigned long,3>& stamp,double begin,double end) {
    if(!std::isfinite(begin)||!std::isfinite(end)||begin<started_||end<begin)
      return EdgAdmission::InvalidClock;
    if(end-begin>deadline_s)return EdgAdmission::SlowCall;
    // A new arrival cannot erase a delivery lapse already observed by this
    // thread. Anchor at the SDK call BEGIN, never its potentially late return.
    if((have_sample_&&end-accepted_at_>deadline_s)||(!have_sample_&&end-started_>1.))
      return EdgAdmission::Expired;
    // An empty startup cache is not a sample. Keep the original one-second
    // first-frame budget without establishing/renewing a feedback anchor.
    // After any accepted sample, zero is a malformed regression, not a reset.
    if(!have_sample_&&stamp[0]==0&&stamp[1]==0&&stamp[2]==0)
      return EdgAdmission::Duplicate;
    if(stamp[0]==0||stamp[1]>=1000000000UL||stamp[2]>255UL)
      return EdgAdmission::InvalidStamp;
    if(have_sample_) {
      if(stamp[0]<stamp_[0]||(stamp[0]==stamp_[0]&&stamp[1]<stamp_[1]))
        return EdgAdmission::RegressedStamp;
      if(stamp[0]==stamp_[0]&&stamp[1]==stamp_[1])return EdgAdmission::Duplicate;
    }
    stamp_=stamp;accepted_at_=begin;have_sample_=true;
    return EdgAdmission::NewSample;
  }
  double accepted_at() const {return accepted_at_;}
  bool have_sample() const {return have_sample_;}
 private:
  std::array<unsigned long,3> stamp_{};
  double started_{0.},accepted_at_{0.};bool have_sample_{false};
};

struct EdgAuditSample {
  uint64_t sequence{0};
  double begin{0.},state_end{0.},end{0.},previous_accepted_at{0.};
  int state_rc{0},details_rc{0};
  std::array<unsigned long,3> stamp{};
  std::array<double,6> position{},velocity{};
  EdgAdmission admission{EdgAdmission::SdkError};
};
class EdgAuditRing {
 public:
  static constexpr size_t capacity=512;
  void reset() {count_=0;new_=0;duplicate_=0;other_=0;}
  void push(EdgAuditSample sample) {
    sample.sequence=count_;
    samples_[count_%capacity]=sample;++count_;
    if(sample.admission==EdgAdmission::NewSample)++new_;
    else if(sample.admission==EdgAdmission::Duplicate)++duplicate_;
    else ++other_;
  }
  uint64_t count() const {return count_;}
  uint64_t new_count() const {return new_;}
  uint64_t duplicate_count() const {return duplicate_;}
  const EdgAuditSample* last() const {return count_?&samples_[(count_-1)%capacity]:nullptr;}
  // Caller must first join the sole writer. This performs I/O only at teardown.
  void write_jsonl(std::ostream& out) const {
    out.precision(17);
    out<<"{\"kind\":\"edg_audit_summary\",\"schema_version\":1,\"samples_total\":"<<count_
       <<",\"new_samples\":"<<new_<<",\"duplicates\":"<<duplicate_<<",\"other\":"<<other_
       <<",\"capacity\":"<<capacity<<",\"retained\":"<<(count_<capacity?count_:capacity)
       <<",\"freshness_limit_s\":0.024,\"receipt_anchor\":\"SDK_call_begin\",\"physical_acceptance\":false}\n";
    for(uint64_t i=count_>capacity?count_-capacity:0;i<count_;++i) {
      const auto& s=samples_[i%capacity];
      out<<"{\"kind\":\"edg_sample\",\"sequence\":"<<s.sequence
         <<",\"host_begin_s\":"<<s.begin<<",\"host_get_return_s\":"<<s.state_end
         <<",\"host_details_return_s\":"<<s.end<<",\"previous_accepted_host_s\":"<<s.previous_accepted_at
         <<",\"get_rc\":"<<s.state_rc<<",\"details_rc\":"<<s.details_rc
         <<",\"stamp\":["<<s.stamp[0]<<','<<s.stamp[1]<<','<<s.stamp[2]
         <<"],\"admission\":\""<<edg_admission_name(s.admission)<<"\",\"q\":[";
      for(size_t j=0;j<6;++j){if(j)out<<',';if(std::isfinite(s.position[j]))out<<s.position[j];else out<<"null";}
      out<<"],\"velocity\":[";
      for(size_t j=0;j<6;++j){if(j)out<<',';if(std::isfinite(s.velocity[j]))out<<s.velocity[j];else out<<"null";}
      out<<"]}\n";
    }
  }
 private:
  std::array<EdgAuditSample,capacity> samples_{};
  uint64_t count_{0},new_{0},duplicate_{0},other_{0};
};
}
