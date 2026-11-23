#pragma once
#include "el/map_paths.hpp"
#include "el/core.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace el {

// Apply v_next² <= v² + 2*a*ds in both directions on a closed route.
// Start at the tightest target so braking and acceleration also cross the seam.
inline void plan_cyclic_speeds(std::vector<float>& speed2,const std::vector<float>& ds,
                               float acceleration,float braking) {
  if(speed2.empty())return;
  const size_t count=speed2.size();
  size_t lowest=std::min_element(speed2.begin(),speed2.end())-speed2.begin();
  size_t i=lowest;
  for(size_t k=0;k<count;++k){
    const size_t prev=(i+count-1)%count;
    speed2[prev]=std::min(speed2[prev],speed2[i]+2*braking*ds[prev]);
    i=prev;
  }
  for(size_t k=0;k<count;++k){
    const size_t next=(i+1)%count;
    speed2[next]=std::min(speed2[next],speed2[i]+2*acceleration*ds[i]);
    i=next;
  }
}

// All stations are distances from this run's starting position in driving order.
class LapPlan {
  friend struct OfflineSnapshot;
  struct Section {
    float entry, obstacle, speed, exit;
  };
  std::vector<Section> sections_;
  Section pending_{};
  std::vector<float> race_speed2_;
  float race_cruise_=0;
  std::vector<float> scout_lateral_;
  bool use_line_=false, recording_=false, ready_=false, scout_stopped_=false;
  float cap_=0, legacy_avoid_cap_=0;
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
  float legacy_avoid_cap()const{return legacy_avoid_cap_;}
  float cap() const {return cap_;}

  bool update(float pos, float length, float first, float speed, float cruise,
              const RouterSettings& settings) {
    // Finish any scout traversal crossing the start line before promoting it.
    if (pos>.5f && speed<.05f) invalidate_scout();
    if (pos>=length && !recording_ && !scout_stopped_) ready_=true;
    cap_=ready_ ? cruise : std::min(cruise,settings.scout_speed_cap);
    legacy_avoid_cap_=std::min(cap_,3.f);
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
      if(settings.sim_section_speed_profile){
        float obstacle_distance=ahead+section.obstacle-section.entry;
        if(obstacle_distance<-.65f)obstacle_distance+=length;
        obstacle_distance=std::min(obstacle_distance,first);
        const float space=std::max(0.f,obstacle_distance-.65f-.25f*speed);
        const float slow=settings.sim_avoid_min;
        cap_=std::min(cap_,sqrtf(slow*slow+3.6f*space));
      }
      if(!settings.sim_section_speed_profile || settings.sim_race_only){
        const float remaining=std::max(0.f,ahead-lead-.20f*speed);
        const float approach=settings.plan_speed_fraction*section.speed;
        const float legacy_limit=std::sqrt(approach*approach+2*1.8f*remaining);
        legacy_avoid_cap_=std::min(legacy_avoid_cap_,legacy_limit);
        if(!settings.sim_section_speed_profile)cap_=std::min(cap_,legacy_limit);
      }
      if (confirmed && ahead<=lead+2.f) use_line_=true;
      if (confirmed && ahead>=-.5f && ahead<=lead) enter=true;
    }
    if (cap_<cruise) ++brake_steps_;
    return enter;
  }

  // The first clean scout supplies obstacle sections and tracking offsets.
  // Station zero in sections is the scout start, which can be anywhere on the map.
  float race_speed(const std::vector<Waypoint>& pts,size_t index,float position,
                   const State& car,float cruise,float avoid_entry) {
    if(!ready_)return 0;
    const size_t n=pts.size();
    const float length=pts.back().s+hypotf(pts.back().x-pts.front().x,pts.back().y-pts.front().y);
    if(race_speed2_.size()!=n || race_cruise_!=cruise){
      race_speed2_.assign(n,cruise*cruise);
      std::vector<float> ds(n);
      const float origin=pts[index].s-position;
      for(size_t i=0;i<n;++i){
        const auto& a=pts[i];
        const auto& b=pts[(i+1)%n];
        ds[i]=hypotf(b.x-a.x,b.y-a.y);
        const float curvature=ds[i]>.001f ? fabsf(wrap(b.yaw-a.yaw))/ds[i] : 0.f;
        // Reduce lateral acceleration where the scout was displaced from the line.
        float deviation=.30f;
        if(i<scout_lateral_.size() && std::isfinite(scout_lateral_[i]))
          deviation=fabsf(scout_lateral_[i]);
        const float lateral_accel=5.2f/(1.f+std::min(.6f,deviation));
        race_speed2_[i]=std::min(race_speed2_[i],lateral_accel/std::max(.01f,curvature));
        for(const auto& section:sections_){
          float from_entry=fmodf(a.s-origin-section.entry,length);
          if(from_entry<0)from_entry+=length;
          if(from_entry<=section.exit-section.entry)
            race_speed2_[i]=std::min(race_speed2_[i],avoid_entry*avoid_entry);
        }
      }
      plan_cyclic_speeds(race_speed2_,ds,3.f,2.6f);
      race_cruise_=cruise;
    }
    // Include the entire distance travelled during response latency. A low
    // target between the current station and the projected nose must still bind.
    const auto& here=pts[index];
    float offset=(car.x-here.x)*cosf(here.yaw)+(car.y-here.y)*sinf(here.yaw);
    size_t i=index;
    auto segment=[&](size_t a){
      const auto& b=pts[(a+1)%n];
      return hypotf(b.x-pts[a].x,b.y-pts[a].y);
    };
    if(offset<0){
      i=(index+n-1)%n;
      if(segment(i)<.001f)i=(i+n-1)%n;
      offset+=segment(i);
    }
    offset=std::clamp(offset,0.f,segment(i));
    float travel=.22f*std::max(0.f,car.v);
    float limit=cruise*cruise;
    for(size_t k=0;k<n;++k){
      const size_t next=(i+1)%n;
      const float ds=segment(i);
      if(ds>.001f){
        const float end=std::min(ds,offset+travel);
        const float delta=(race_speed2_[next]-race_speed2_[i])/ds;
        limit=std::min(limit,race_speed2_[i]+std::min(offset*delta,end*delta));
        if(travel<=ds-offset)break;
        travel-=ds-offset;
      }
      offset=0;
      i=next;
    }
    return sqrtf(std::max(0.f,limit));
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
