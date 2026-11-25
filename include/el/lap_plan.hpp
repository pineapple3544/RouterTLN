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

// Build a closed reference from clearance measured during this run's scout.
// Each mask describes permissible vehicle centers in one track cross-section.
class ScoutRoute {
  friend struct OfflineSnapshot;
  static constexpr int columns=25, center=columns/2;
  static constexpr float spacing=.10f;
  std::vector<uint32_t> free_,traversed_;
  std::vector<float> offsets_, speeds2_;
  int last_=-1;
  bool attempted_=false;

 public:
  bool available() const {return !offsets_.empty();}
  const char* status() const {
    if(available())return "planned";
    if(!attempted_)return "scouting";
    return std::find(free_.begin(),free_.end(),0u)!=free_.end()?"blocked_or_unseen_row":"disconnected";
  }
  const std::vector<float>& line() const {return offsets_;}
  float speed(size_t i) const {return std::sqrt(speeds2_.at(i));}

  void observe(const std::vector<Waypoint>& pts,int index,const State& car,const Memory& memory){
    if(index==last_ || attempted_)return;
    const int n=static_cast<int>(pts.size());
    if(free_.empty()){free_.resize(n);traversed_.resize(n);}
    int steps=last_<0?1:(index-last_+n)%n;
    // A localization jump supplies no measurements for its skipped stations.
    if(steps>6)steps=1;
    for(int back=steps-1;back>=0;--back){
      const int i=(index-back+n)%n;
      const auto& p=pts[i];
      uint32_t mask=0,driven=0;
      const float offset=-(car.x-p.x)*std::sin(p.yaw)+(car.y-p.y)*std::cos(p.yaw);
      for(int j=0;j<columns;++j){
        const float d=(j-center)*spacing;
        State pose{p.x-std::sin(p.yaw)*d,p.y+std::cos(p.yaw)*d,p.yaw};
        if(footprint(memory,pose,.09f)<=.5f)mask|=uint32_t{1}<<j;
        // A turning car can fit where a rectangle aligned to the reference
        // cannot. Test only bins adjoining the observed scout trajectory.
        if(std::abs(d-offset)<=spacing){
          pose.yaw=car.yaw;
          if(footprint(memory,pose,.04f)<=.5f)driven|=uint32_t{1}<<j;
        }
      }
      free_[i]=mask;traversed_[i]=driven;
    }
    last_=index;
  }

  void finish(const std::vector<Waypoint>& pts,const std::vector<float>& scout,float cap,float floor){
    if(attempted_)return;
    attempted_=true;
    const size_t n=pts.size();
    if(n<5 || free_.size()!=n || scout.size()!=n)return;
    // Close the loop at a clear stretch on the centerline. No map index is fixed.
    auto mask=free_;
    size_t seam=0;
    int anchor=-1;
    float best_room=-1.f;
    for(size_t i=0;i<n;++i){
      if(!(mask[i]|traversed_[i]))return;
      for(int j=0;j<columns;++j){
        if(!(free_[i]&(1u<<j)))continue;
        int room=0;
        while(j-room>=0 && j+room<columns && (free_[i]&(1u<<(j-room))) &&
              (free_[i]&(1u<<(j+room))))++room;
        float merit=room-.1f*std::abs(j-center);
        if(merit>best_room){best_room=merit;seam=i;anchor=j;}
      }
    }
    using Costs=std::array<float,columns>;
    std::vector<std::array<int8_t,columns>> parent(n);
    auto close_loop=[&](int start,int reach){
      Costs cost,next;
      cost.fill(INFINITY);cost[start]=0;
      for(size_t k=1;k<=n;++k){
        const size_t i=(seam+k)%n,prev=(i+n-1)%n;
        next.fill(INFINITY);
        const float prior=std::isfinite(scout[i])?scout[i]:0.f;
        for(int j=0;j<columns;++j){
          if(!(mask[i]&(1u<<j)) || (k==n && j!=start))continue;
          const float d=(j-center)*spacing;
          const bool narrow=j==0 || j==columns-1 ||
            !(mask[i]&(1u<<(j-1))) || !(mask[i]&(1u<<(j+1)));
          const float penalty=.20f*d*d+.65f*(d-prior)*(d-prior)+(narrow?1.f:0.f);
          for(int from=std::max(0,j-reach);from<=std::min(columns-1,j+reach);++from){
            if(std::abs(j-from)>1 && !((mask[i]&mask[prev])&(1u<<((j+from)/2))))continue;
            const float v=cost[from]+penalty+1.5f*(j-from)*(j-from);
            if(v<next[j]){next[j]=v;parent[i][j]=static_cast<int8_t>(from);}
          }
        }
        cost=next;
        if(!std::isfinite(*std::min_element(cost.begin(),cost.end())))return false;
      }
      if(!std::isfinite(cost[start]))return false;
      offsets_.resize(n);int j=start;
      for(size_t k=n;k>0;--k){
        const size_t i=(seam+k)%n;
        offsets_[i]=(j-center)*spacing;j=parent[i][j];
      }
      return true;
    };
    if(anchor<0)return;
    if(!close_loop(anchor,1)){
      for(size_t i=0;i<n;++i)mask[i]|=traversed_[i];
      // A wide cross-section can still lie just before a sharp lane change.
      // Retry anchored at the traversed side before giving up on a closed line.
      const int driven=std::isfinite(scout[seam])?
        std::clamp(center+int(std::lround(scout[seam]/spacing)),0,columns-1):anchor;
      // Propagate a bit for each possible start through a single lap; then
      // run the cost search only for a start that can return to itself.
      std::array<uint32_t,columns> origins{},next{};
      for(int j=0;j<columns;++j)origins[j]=mask[seam]&(1u<<j);
      for(size_t k=1;k<=n;++k){
        const size_t i=(seam+k)%n,prev=(i+n-1)%n;
        next.fill(0);
        for(int j=0;j<columns;++j){
          if(!(mask[i]&(1u<<j)))continue;
          for(int from=std::max(0,j-2);from<=std::min(columns-1,j+2);++from){
            if(std::abs(j-from)>1 && !((mask[i]&mask[prev])&(1u<<((j+from)/2))))continue;
            next[j]|=origins[from];
          }
        }
        origins=next;
      }
      int feasible=-1;
      for(int j=0;j<columns;++j)
        if((origins[j]&(1u<<j)) && (feasible<0 || std::abs(j-driven)<std::abs(feasible-driven)))feasible=j;
      if(feasible<0 || !close_loop(feasible,2))return;
    }
    // Remove grid steps without crossing a blocked lateral bin. The local
    // corridor still checks the continuous vehicle motion against live LiDAR.
    std::vector<float> smoothed(n),lo(n),hi(n);
    for(size_t i=0;i<n;++i){
      const int col=center+static_cast<int>(std::lround(offsets_[i]/spacing));
      int left=col,right=col;
      while(left>0 && (mask[i]&(1u<<(left-1))))--left;
      while(right+1<columns && (mask[i]&(1u<<(right+1))))++right;
      lo[i]=(left-center)*spacing;hi[i]=(right-center)*spacing;
    }
    for(int pass=0;pass<6;++pass){
      for(size_t i=0;i<n;++i){
        const float mean=.25f*(offsets_[(i+n-1)%n]+2*offsets_[i]+offsets_[(i+1)%n]);
        smoothed[i]=std::clamp(mean,lo[i],hi[i]);
      }
      offsets_.swap(smoothed);
    }
    auto xy=[&](size_t i){
      const auto& p=pts[i];float d=offsets_[i];
      return std::pair<float,float>{p.x-std::sin(p.yaw)*d,p.y+std::cos(p.yaw)*d};
    };
    std::vector<float> ds(n);
    speeds2_.resize(n);
    for(size_t i=0;i<n;++i){
      const auto [x,y]=xy(i);
      const auto [xn,yn]=xy((i+1)%n);
      const auto [ax,ay]=xy((i+n-2)%n);
      const auto [bx,by]=xy((i+2)%n);
      ds[i]=std::hypot(xn-x,yn-y);
      const float denom=std::hypot(x-ax,y-ay)*std::hypot(bx-x,by-y)*std::hypot(bx-ax,by-ay);
      const float curvature=2*std::abs((x-ax)*(by-y)-(y-ay)*(bx-x))/std::max(1e-6f,denom);
      const int column=center+int(std::lround(offsets_[i]/spacing));
      const float upper=(free_[i]&(1u<<column))?cap:floor;
      speeds2_[i]=std::min(upper*upper,4.5f/std::max(.01f,curvature));
    }
    plan_cyclic_speeds(speeds2_,ds,1.8f,1.8f);
  }
};

// All stations are distances from this run's starting position in driving order.
class LapPlan {
  friend struct OfflineSnapshot;
  struct Section {
    float entry, obstacle, speed, exit;
  };
  std::vector<Section> sections_;
  ScoutRoute route_;
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
  void observe_route(const std::vector<Waypoint>& pts,int i,const State& car,const Memory& mem){
    if(!ready_ && !scout_stopped_)route_.observe(pts,i,car,mem);
  }
  void finish_route(const std::vector<Waypoint>& pts,float cruise,float floor){
    if(ready_)route_.finish(pts,scout_lateral_,cruise,floor);
  }
  bool has_route() const {return route_.available();}
  const char* route_status() const {return route_.status();}
  float route_speed(size_t i) const {return route_.speed(i);}
  const std::vector<float>& scout_line() const {return scout_lateral_;}
  const std::vector<float>* avoidance_line() const {
    if(route_.available())return &route_.line();
    return use_line_ ? &scout_lateral_ : nullptr;
  }
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
