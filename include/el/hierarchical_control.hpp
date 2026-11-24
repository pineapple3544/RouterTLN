#pragma once
#include "el/core.hpp"
#include <algorithm>
#include <cmath>

namespace el {
// Driving phase is independent of the hard-selected expert. Preserve Mode's
// numeric values for the legacy evaluator, ROS topics and physical judge.
enum class DrivingPhase : int {Race=0, Prepare=1, Avoid=2, Merge=3};

struct DrivingPlan {
  bool active=false;
  Mode expert=Mode::Race;
  DrivingPhase phase=DrivingPhase::Race;
  int route_index=0;
  float obstacle_distance=-1;
  float curve_cap=-1, sight_cap=-1, approach_cap=-1, corridor_cap=-1;
  float target_speed=0, safe_speed=0, reference_steer=0;
  // Same five ego-relative reference points as schema 3/4, normalized by 10 m.
  std::array<float,10> path_context{};
};

inline DrivingPhase driving_phase(bool avoiding,bool merging,float distance,float prepare_distance) {
  if(avoiding)return merging?DrivingPhase::Merge:DrivingPhase::Avoid;
  return std::isfinite(distance) && distance<=prepare_distance
    ?DrivingPhase::Prepare:DrivingPhase::Race;
}

// Provisional approach envelope until a feasible local corridor is available.
// It is a ceiling, never a speed floor or a requirement to accelerate.
inline float approach_speed_limit(float cruise,float distance,float speed,float entry_speed) {
  if(!std::isfinite(distance))return cruise;
  const float space=std::max(0.f,distance-.65f-.22f*std::max(0.f,speed));
  return std::min(cruise,std::sqrt(entry_speed*entry_speed+2.f*2.6f*space));
}

// Keep legacy entry distance as a minimum. Earlier selection is allowed only
// for a currently observed conflict, never for memory alone.
inline float handoff_entry_distance(float legacy_distance,float speed,float entry_speed) {
  const float v=std::max(0.f,speed);
  const float braking=std::max(0.f,v*v-entry_speed*entry_speed)/(2.f*2.6f);
  return std::max(legacy_distance,braking+.22f*v+1.65f);
}

inline bool handoff_dynamics_ready(const State& car,float curvature,float race_cap) {
  return std::abs(car.steer-std::atan(.3302f*curvature))<=.20f &&
         std::abs(car.yaw_rate-car.v*curvature)<=1.5f &&
         car.v<=race_cap+.20f;
}

// Retain the trained checkpoint's observation schema. Future plan-aware
// checkpoints can consume the remaining fields explicitly after retraining.
inline void apply_plan_path(Observation& input,const DrivingPlan& plan) {
  for(size_t i=0;i<plan.path_context.size();++i)input.context[5+i]=plan.path_context[i];
  if(plan.expert==Mode::Avoid)input.context[4]=plan.reference_steer/.4f;
}

inline Control constrain_policy(Control raw,const DrivingPlan& plan,float scale,
                                float previous_speed,float actual_speed,float dt,bool have_previous) {
  float cap=std::max(0.f,plan.safe_speed);
  if(have_previous)
    cap=std::min(cap,std::max(previous_speed,actual_speed)+4.f*dt);
  return {clamp(raw.steer,-.4f,.4f),clamp(raw.speed*scale,0.f,cap)};
}

struct V2Selection {
  Control command{};
  float before_guard_steer=0;
  const char* refusal=nullptr;
  bool preview_used=false, fallback_used=false;
};

// A normal proposal is never replaced merely because another candidate scores
// better. First validate it; search only if its local tracking preview or the
// independent stopping envelope fails (or in the always-search ablation).
// The guard is applied to every command eligible for execution.
template<class Preview,class Guard>
V2Selection select_v2_control(Control proposal,bool avoiding,bool policy_first,
                              bool tracking_ok,float current_steer,float actual_speed,
                              Preview preview,Guard guard) {
  V2Selection result;
  result.command=proposal;
  auto search=[&](){
    result.command.steer=preview(result.command);
    result.preview_used=true;
  };
  if(avoiding && (!policy_first || !tracking_ok))search();
  result.before_guard_steer=result.command.steer;
  result.refusal=guard(result.command);
  if(result.refusal && avoiding && !result.preview_used){
    search();
    result.before_guard_steer=result.command.steer;
    result.refusal=guard(result.command);
  }
  if(result.refusal && avoiding && result.command.speed>0){
    const Control requested=result.command;
    const float slower=std::min(requested.speed,
      std::max(.6f,.65f*std::min(requested.speed,actual_speed)));
    for(float speed:{requested.speed,slower}){
      for(float steer:{requested.steer,current_steer,0.f,-.2f,.2f,-.4f,.4f}){
        Control candidate{clamp(steer,-.4f,.4f),speed};
        if(!guard(candidate)){
          result.command=candidate;
          result.refusal=nullptr;
          result.fallback_used=true;
          return result;
        }
      }
      if(slower==requested.speed)break;
    }
  }
  return result;
}
}
