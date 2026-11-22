#pragma once
#include "el/map_paths.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace el {
// All stations are distances from this run's starting position in driving order.
class LapPlan {
  struct Section {
    float entry, obstacle, speed, exit;
  };
  std::vector<Section> sections_;
  Section pending_{};
  std::vector<float> scout_lateral_;
  bool use_line_=false, recording_=false, ready_=false, scout_stopped_=false;
  float cap_=0;
  unsigned planned_entries_=0, brake_steps_=0;

 public:
  void observe(int index, size_t size, float lateral) {
    if (ready_ || scout_stopped_) return;
    if (scout_lateral_.empty())
      scout_lateral_.assign(size,std::numeric_limits<float>::quiet_NaN());
    scout_lateral_[index]=lateral;
  }
  const std::vector<float>* avoidance_line() const {return use_line_ ? &scout_lateral_ : nullptr;}
  void invalidate_scout() {if (!ready_) scout_stopped_=true;}
  bool ready() const {return ready_;}
  unsigned sections() const {return sections_.size();}
  unsigned entries() const {return planned_entries_;}
  unsigned brake_steps() const {return brake_steps_;}
  float cap() const {return cap_;}

  bool update(float pos, float length, float first, float speed, float cruise,
              const RouterSettings& settings) {
    // Finish any scout traversal crossing the start line before promoting it.
    if (pos>.5f && speed<.05f) invalidate_scout();
    if (pos>=length && !recording_ && !scout_stopped_) ready_=true;
    cap_=ready_ ? cruise : std::min(cruise,settings.scout_speed_cap);
    use_line_=false;
    if (!ready_) return false;

    bool enter=false;
    for (const auto& section:sections_) {
      const float span=section.exit-section.entry;
      float ahead=std::fmod(section.entry-pos,length);
      if (ahead < -span) ahead+=length;
      if (ahead > length-span) ahead-=length;

      // Recheck occupancy before switching policies or preferring the scout line.
      bool confirmed=false;
      if (std::isfinite(first)) {
        const float offset=std::remainder(pos+first-section.obstacle,length);
        confirmed=offset>=-1.f && offset<=section.exit-section.obstacle+1.f;
      }
      const float lead=.35f+settings.plan_lead_time*section.speed;
      const float remaining=std::max(0.f,ahead-lead-.20f*speed);
      const float approach=settings.plan_speed_fraction*section.speed;
      const float limit=std::sqrt(approach*approach+2*1.8f*remaining);
      cap_=std::min(cap_,limit);
      if (confirmed && ahead<=lead+2.f) use_line_=true;
      if (confirmed && ahead>=-.5f && ahead<=lead) enter=true;
    }
    if (cap_<cruise) ++brake_steps_;
    return enter;
  }
  void enter(float pos, float first, float speed, float scout_cap, bool scheduled) {
    if (scheduled) ++planned_entries_;
    if (!ready_ && !scout_stopped_ && std::isfinite(first)) {
      pending_={pos,pos+first,std::clamp(speed,.5f,std::max(.5f,scout_cap)),pos};
      recording_=true;
    }
  }
  void returned(float pos) {
    if (!recording_) return;
    pending_.exit=pos;
    sections_.push_back(pending_);
    recording_=false;
  }
};
}
