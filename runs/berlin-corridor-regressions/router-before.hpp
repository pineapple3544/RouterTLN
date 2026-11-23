#pragma once
#include "el/dynamics.hpp"
#include "el/map_paths.hpp"
#include "el/lap_plan.hpp"
#include "el/policy.hpp"
#include "el/avoid_corridor.hpp"
#include <algorithm>
#include <limits>

namespace el {
struct SectionDecision {
  Mode mode;
  Control command;
  const char* reason;
  bool blocked;
};
class SectionRouter {
  friend struct OfflineSnapshot;
 public:
  const LapPlan& lap_plan()const{return lap_plan_;}
  bool avoiding() const {return avoiding_;}
  int side() const {return side_;}
  void reset() { *this = SectionRouter{}; }
  void stop(const char* reason) { if(!stop_reason_)stop_reason_=reason; }

  SectionDecision step(const Track& track, const Memory& memory, const State& car,
                       const std::deque<Scan>& scans, Policy& race, Policy& avoid,
                       float cap, float dt, double now, const RouterSettings& settings={}, ControlTiming* timing=nullptr, CorridorTrace* trace=nullptr) {
    auto stop_with = [&](const char* reason) {
      stop(reason);
      return SectionDecision{Mode::Emergency, {car.steer, 0}, reason, avoiding_};
    };
    if (stop_reason_) return stop_with(stop_reason_);
    // Keep the Berlin reference cruise at 3 m/s during local stress trials;
    // only Avoid on flying laps may use the wider validated band.
    if (settings.sim_avoid_max>0) cap=std::min(cap,3.f);
    Observation input;
    auto decision=route_observation(track,memory,car,scans,avoid.uses_corridor(),now,input,settings,cap,timing,trace);
    if(timing)timing->mark(ControlTiming::corridor);
    if(decision.mode==Mode::Emergency)return stop_with(decision.reason);
    Control cmd=(avoiding_?avoid:race).infer(input,timing);
    if (!std::isfinite(cmd.steer)||!std::isfinite(cmd.speed)) return stop_with("invalid_action");
    if(!avoiding_)cmd.speed*=settings.race_speed_scale;
    if(settings.lap_planning)cap=std::min(cap,lap_plan_.cap());
    cmd={clamp(cmd.steer,-.4f,.4f),clamp(cmd.speed,0,cap)};
    if(avoiding_ && avoid.uses_corridor())cmd.speed=corridor_.speed_limit(std::min(cmd.speed,settings.avoid_speed_cap),car.v);
    // Stress the chosen command band after the usual soft speed limits.
    // Keep the scout, zero commands and all footprint/braking checks intact.
    if (avoiding_ && lap_plan_.ready() && settings.sim_avoid_max>0 && cmd.speed>0)
      cmd.speed=std::clamp(cmd.speed,settings.sim_avoid_min,settings.sim_avoid_max);
    if (settings.lap_planning && cmd.speed<=0) lap_plan_.invalidate_scout();
    float slip=0;
    if(have_pose_ && hypotf(car.x-last_pose_.x,car.y-last_pose_.y)>.015f){
      const float mid_yaw=last_pose_.yaw+.5f*wrap(car.yaw-last_pose_.yaw);
      slip=wrap(atan2f(car.y-last_pose_.y,car.x-last_pose_.x)-mid_yaw);
    }
    const float pending=have_pose_?last_command_.steer:car.steer;
    DynamicState dyn{{car.x,car.y,car.steer,car.v,car.yaw,car.yaw_rate,slip},{pending,pending}};
    const int ticks=std::max(1,int(std::ceil(dt/.01f-1e-5f)));
    // The first physics interval can still execute the preceding command.
    // physics_step then consumes the two queued steering inputs and applies
    // the steering actuator's rate limit before the selected angle is reached.
    const Control committed=have_pose_?last_command_:Control{car.steer,car.v};
    dyn=physics_step(dyn,committed,Vehicle{});
    if(trace)trace->policy_steer=cmd.steer;
    if(avoiding_ && avoid.uses_corridor() && settings.lap_planning)
      cmd.steer=corridor_.preview_steer(dyn,cmd,memory,dt,ticks);
    if(trace)trace->candidate_steer=cmd.steer;
    // Predict the command interval with the measured drive response. Keep the
    // slower braking envelope as an independent geometric clearance check.
    State prediction=predict_command_tick(car,committed,.01f);
    for(int i=0;i<ticks;++i){
      prediction=predict_command_tick(prediction,cmd,dt/ticks);
      if(footprint(memory,prediction)>.5f || gym_front_blocked(memory,prediction)) return stop_with("command_footprint");
    }
    Control brake{prediction.steer,0};
    for(int i=0;prediction.v>=.045f && i<300;++i){
      prediction=advance(prediction,brake,.01f);
      if(footprint(memory,prediction)>.5f || gym_front_blocked(memory,prediction)){
        if(trace){trace->brake_event=true;trace->brake_contact=prediction;}
        return stop_with("braking_footprint");
      }
    }
    if(prediction.v>=.045f) return stop_with("braking_horizon");

    last_pose_=car;have_pose_=true;
    // Include one additional physics tick before braking for sampled actuation.
    for(int i=0;i<ticks+1;++i){
      dyn=physics_step(dyn,cmd,Vehicle{});
      if(!std::isfinite(dyn.z[0]) || !std::isfinite(dyn.z[1]) || !std::isfinite(dyn.z[4]) ||
         footprint(memory,basic(dyn))>.5f || gym_front_blocked(memory,basic(dyn))) return stop_with("dynamic_command_footprint");
    }
    brake={dyn.z[2],0};
    for(int i=0;dyn.z[3]>=.045f && i<300;++i){
      dyn=physics_step(dyn,brake,Vehicle{});
      if(!std::isfinite(dyn.z[0]) || !std::isfinite(dyn.z[1]) || !std::isfinite(dyn.z[4]) ||
         footprint(memory,basic(dyn))>.5f || gym_front_blocked(memory,basic(dyn))) return stop_with("dynamic_braking_footprint");
    }
    if(!std::isfinite(dyn.z[3]) || dyn.z[3]>=.045f) return stop_with("braking_horizon");
    last_command_=cmd;
    decision.command=cmd;
    return decision;
  }
  bool merge_blocked()const{return merge_blocked_;}
  // Shared sensor-only routing and policy features for online inference and
  // offline expert labels at learner-visited states. Does not publish commands.
  SectionDecision route_observation(const Track& track,const Memory& memory,const State& car,
                                    const std::deque<Scan>& scans,bool guided,double now,Observation& input,
                                    const RouterSettings& settings={}, float cruise=3.f, ControlTiming* timing=nullptr, CorridorTrace* trace=nullptr){
    const bool plan_laps=settings.lap_planning;
    int n = track.closest(car);
    const auto& points = track.points;
    const auto& here = points[n];
    float length = points.back().s + hypotf(points.back().x-points.front().x,
                                             points.back().y-points.front().y);
    if (last_station_ >= 0) position_ += std::remainder(here.s-last_station_, length);
    last_station_ = here.s;
    float lateral = -(car.x-here.x)*sinf(here.yaw)+(car.y-here.y)*cosf(here.yaw);
    if(plan_laps)lap_plan_.observe(n,points.size(),lateral);
    float entry = 2.0f + .45f*car.v + car.v*car.v/10;
    float merge_distance = entry + .7f + fabsf(lateral);
    float horizon = std::min(length*.48f, entry+merge_distance+5);
    float first = INFINITY, end = -INFINITY;
    struct ObstacleLayer{
      const Memory& mem;
      float at(float x,float y)const{return mem.obstacle_at(x,y);}
    } obstacles{memory};
    bool merge_clear = true;
    for (size_t i=0; i<points.size(); ++i) {
      auto& w = points[(n+i)%points.size()];
      float ahead = w.s-here.s;
      if (ahead < -.001f) ahead += length;
      if (ahead > horizon) break;
      State sample{w.x,w.y,w.yaw};
      if (ahead <= 1.5f) {
        float t = clamp(ahead/1.5f,0,1);
        float d = lateral*(1-t*t*(3-2*t));
        State merge{w.x-d*sinf(w.yaw),w.y+d*cosf(w.yaw),
                    wrap(w.yaw + atanf(-lateral*6*t*(1-t)/1.5f))};
        if (footprint(memory,merge)>.5f) merge_clear=false;
      }
      // Nearby obstacle faces can intersect the car's approach or merge even
      // when the mathematical reference line itself just clears the box.
      if (footprint(memory,sample)<=.5f && footprint(obstacles,sample,.19f)<=.5f) continue;
      if (std::isfinite(first) && ahead-end>merge_distance) break;
      first=std::min(first,ahead);
      end=ahead;
    }
    merge_blocked_=!merge_clear;
    const char* reason = avoiding_ ? "hold" : "race";
    bool hazard = first<=entry;
    bool scheduled=plan_laps && lap_plan_.update(position_,length,first,car.v,cruise,settings);
    if (scheduled || hazard || (avoiding_ && position_+first<=end_+merge_distance)) {
      end_=std::max(end_,position_+end);
      if (!avoiding_) {
        reason=scheduled?"planned_start":"start";
        if(plan_laps)lap_plan_.enter(position_,first,car.v,settings.scout_speed_cap,scheduled);
      }
      avoiding_=true;
    }

    if (avoiding_) {
      bool ready=position_>end_+.35f && !hazard && merge_clear &&
                 fabsf(lateral)<.30f && fabsf(wrap(car.yaw-here.yaw))<.32f;
      if (!ready) clear_since_=-1;
      else if (clear_since_<0) clear_since_=now;
      else if (now-clear_since_>=.20) {
        if(plan_laps)lap_plan_.returned(position_);
        avoiding_=false; side_=0; clear_since_=-1; end_=-INFINITY; reason="return";
      }
    }
    // Schema 3 demonstrations used side = 0. Let Avoid produce the lateral action.
    if(timing)timing->mark(ControlTiming::route);
    input=observe(scans,car,memory,track,avoiding_?1:0,0);
    if(timing)timing->mark(ControlTiming::observation);
    if(avoiding_){
      const bool returning=position_>end_+.35f && !hazard && merge_clear;
      if(returning && !plan_laps)corridor_.follow_reference(track,n);
      else if(!corridor_.plan(track,n,car,memory,plan_laps && !returning?lap_plan_.avoidance_line():nullptr,trace,plan_laps))
        return {Mode::Emergency,{car.steer,0},"no_corridor",avoiding_};
      if(hazard)side_=corridor_.side_at(first+.35f);
      if(guided)corridor_.condition(input,car,settings.lookahead_scale,returning);
    }
    return {avoiding_?Mode::Avoid:Mode::Race,{},reason,hazard};
  }
 private:
  State last_pose_{};
  Control last_command_{};
  bool have_pose_=false;
  LapPlan lap_plan_;
  bool merge_blocked_=false;
  AvoidCorridor corridor_;
  float last_station_=-1,position_=0,end_=-INFINITY;
  double clear_since_=-1;
  int side_=0;
  bool avoiding_=false;
  const char* stop_reason_=nullptr;
};
}
