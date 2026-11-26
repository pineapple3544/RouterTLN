#pragma once
#include "el/dynamics.hpp"
#include "el/race_speed.hpp"
#include "el/map_paths.hpp"
#include "el/lap_plan.hpp"
#include "el/policy.hpp"
#include "el/avoid_corridor.hpp"
#include <algorithm>
#include <chrono>
#include <limits>

namespace el {
struct SpeedLimits {
  float policy=-1, lap=-1, corridor=-1, before_guard=-1;
  float curve=-1, sight=-1, steering=-1, planned=-1;
  bool preview_brake=false;
};
struct SectionDecision {
  Mode mode;
  Control command;
  const char* reason;
  bool blocked;
  SpeedLimits speed_limits{};
};
class SectionRouter {
  friend struct OfflineSnapshot;
 public:
  const LapPlan& lap_plan()const{return lap_plan_;}
  bool avoiding() const {return avoiding_;}
  int side() const {return side_;}
  void reset() { *this = SectionRouter{}; }
  void stop(const char* reason) { if(!stop_reason_)stop_reason_=reason; }

  template<class RacePolicy>
  SectionDecision step(const Track& track, const Memory& memory, const State& car,
                       const std::deque<Scan>& scans, RacePolicy& race, Policy& avoid,
                       float cap, float dt, double now, const RouterSettings& settings={}, ControlTiming* timing=nullptr, CorridorTrace* trace=nullptr) {
    SpeedLimits limits;
    const auto step_start=std::chrono::steady_clock::now();
    auto elapsed_ms=[&]{return std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-step_start).count();};
    auto stop_with = [&](const char* reason) {
      stop(reason);
      return SectionDecision{Mode::Emergency, stop_command(car,settings), reason, avoiding_,limits};
    };
    if (stop_reason_) return stop_with(stop_reason_);
    // The original stress mode changes only the flying-lap Avoid band.
    const float requested_cap=cap;
    const bool adaptive=settings.sim_section_speed_profile && lap_plan_.ready();
    if (settings.sim_avoid_max>0 && !adaptive) cap=std::min(cap,3.f);
    Observation input;
    int route_index=0;
    auto decision=route_observation(track,memory,car,scans,avoid.uses_corridor(),now,input,settings,cap,timing,trace,&route_index);
    if(timing)timing->mark(ControlTiming::corridor);
    if(decision.mode==Mode::Emergency)return stop_with(decision.reason);
    Control cmd=avoiding_?avoid.infer(input,timing):race.infer(input,timing);
    if (!std::isfinite(cmd.steer)||!std::isfinite(cmd.speed)) return stop_with("invalid_action");
    limits.policy=cmd.speed;
    const bool adaptive_avoid=adaptive && !settings.sim_race_only;
    if(!avoiding_){
      cmd.speed*=settings.race_speed_scale;
      if(adaptive){
        if(settings.race_lap_speed){
          limits.planned=lap_plan_.race_speed(track.points,route_index,position_,car,requested_cap,settings.sim_avoid_min);
          cap=std::min(cap,limits.planned);
        }else{
          cap=curve_speed_cap(track,route_index,car.v,cap,
            settings.race_exit_speed?7.0f:4.5f,settings.race_exit_speed?3.0f:1.8f);
          limits.curve=cap;
        }
        float policy_cap=requested_cap;
        if(settings.race_sight_speed || settings.race_lap_speed){
          limits.sight=scans.empty()?0.f:race_sight_speed_limit(scans.back(),6.f);
          cap=std::min(cap,limits.sight);
        }
        if(settings.race_exit_speed || settings.race_lap_speed){
          if(settings.race_exit_speed)cap=std::min(cap,narrow_bend_speed_cap(track,route_index,memory,car,requested_cap));
          // Actuation delay means the wheel may still be turning harder than
          // the network requests on exit. Budget lateral grip for both angles.
          const float steer=std::max(fabsf(cmd.steer),fabsf(car.steer));
          limits.steering=sqrtf(8.0f*.3302f/std::max(.001f,tanf(steer)));
          const auto& ref=track.points[route_index];
          const float yaw_error=fabsf(wrap(car.yaw-ref.yaw));
          const float correction=fabsf(cmd.steer-car.steer);
          const float stability=clamp(1.f-2.5f*(yaw_error+correction),0.f,1.f);
          limits.steering=std::min(limits.steering,3.3f+stability*(requested_cap-3.3f));
          cap=std::min(cap,limits.steering);
          // Convert the planned limit into the Race policy's nominal 3 m/s scale.
          if(cmd.speed>0)
            cmd.speed=std::max(cmd.speed,cap*3.f/requested_cap);
        }
        if(settings.straight_boost && requested_cap>3.5f){
          // Use the extra speed only with an open stopping path after the scout.
          const bool straight=std::max(fabsf(cmd.steer),fabsf(car.steer))<.09f &&
            std::min(cap,lap_plan_.cap())>=requested_cap-.01f;
          if(!straight || scans.empty() || !clear_for_boost(scans.back(),requested_cap)){
            policy_cap=3.5f;
            cap=std::min(cap,policy_cap);
          }
        }
        cmd.speed*=policy_cap/3.f;
      }
    }
    if(settings.lap_planning){
      limits.lap=avoiding_ && settings.sim_race_only ? lap_plan_.legacy_avoid_cap() : lap_plan_.cap();
      cap=std::min(cap,limits.lap);
    }
    cmd={clamp(cmd.steer,-.4f,.4f),clamp(cmd.speed,0,cap)};
    if(avoiding_ && avoid.uses_corridor())
      limits.corridor=corridor_.speed_limit(settings.avoid_speed_cap,car.v);
    if(adaptive_avoid && avoiding_ && avoid.uses_corridor() && cmd.speed>0){
      // The router schedules positive Avoid speeds from the planned approach and
      // path curvature. An even lower approach cap or a stop takes precedence.
      const float upper=std::min({cap,settings.sim_avoid_max,settings.avoid_speed_cap});
      const float floor=std::min(settings.sim_avoid_min,upper);
      const float desired=std::clamp(cmd.speed*(requested_cap/3.f),floor,upper);
      cmd.speed=std::min(desired,limits.corridor);
    }else if(avoiding_ && avoid.uses_corridor())
      cmd.speed=std::min(cmd.speed,limits.corridor);
    // Stress the chosen command band after the usual soft speed limits.
    // Keep the scout, zero commands and all footprint/braking checks intact.
    if (!adaptive_avoid && avoiding_ && lap_plan_.ready() && settings.sim_avoid_max>0 && cmd.speed>0)
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
    if((settings.race_exit_speed || settings.race_lap_speed) && adaptive && !avoiding_ && have_pose_){
      constexpr float exit_acceleration_mps2=4.f;
      cmd.speed=std::min(cmd.speed,std::max(last_command_.speed,car.v)+exit_acceleration_mps2*dt);
    }
    limits.before_guard=cmd.speed;
    if(settings.sim_race_only && adaptive && !avoiding_ && settings.race_brake_preview_s>0
       && cmd.speed>0 && std::max(cmd.speed,car.v)>3.f){
      // Allow for several control updates at the current steering angle before braking.
      // The normal one-cycle refusal still validates the command that will be issued.
      const int ahead_ticks=std::max(ticks,int(std::ceil(settings.race_brake_preview_s/.01f)));
      if(command_refusal(memory,car,committed,cmd,dyn,.01f*ahead_ticks,ahead_ticks,nullptr)){
        const float slower=std::min(cmd.speed,std::max(.8f,car.v-.8f));
        limits.preview_brake=slower<cmd.speed;
        cmd.speed=slower;
      }
    }
    if((settings.race_exit_speed || settings.race_lap_speed) && adaptive && !avoiding_ && cmd.speed>3.f){
      // The existing 180 ms guard runs first. Check the higher exit speed at 240 ms.
      constexpr int count=24;
      if(command_refusal(memory,car,committed,cmd,dyn,.01f*count,count,nullptr)){
        const float slower=std::max(1.f,car.v-1.0f);
        cmd.speed=std::min(cmd.speed,slower);
        limits.preview_brake=true;
      }
    }
    if(trace)trace->policy_steer=cmd.steer;
    if(avoiding_ && avoid.uses_corridor() && settings.lap_planning)
      cmd.steer=corridor_.preview_steer(dyn,cmd,memory,dt,ticks);
    // A scout can unwind full lock while the next obstacle is still in the
    // stopping arc. Compare the queued turn and the requested turn before the
    // immediate guard loses all feasible actions. Re-evaluate on each scan.
    if(avoiding_ && avoid.uses_corridor() && settings.lap_planning && !lap_plan_.ready()
       && decision.blocked && std::abs(committed.steer)>=.29f
       && (committed.steer-cmd.steer)*committed.steer>.018f){
      const float nominal=scout_stopping_clearance(memory,dyn,cmd);
      if(nominal<.12f){
        Control keep_turn{committed.steer,cmd.speed};
        const float clearance=scout_stopping_clearance(memory,dyn,keep_turn);
        if(clearance>nominal){
          cmd=keep_turn;
          decision.reason="hold_clear_turn";
        }
      }
    }
    if(trace)trace->candidate_steer=cmd.steer;
    const char* refusal=command_refusal(memory,car,committed,cmd,dyn,dt,ticks,trace);
    const bool avoid_fallback=avoiding_ && avoid.uses_corridor() && settings.lap_planning;
    const bool race_fallback=settings.race_fallback && !avoiding_;
    if(refusal && (avoid_fallback || race_fallback) && cmd.speed>0){
      // Try a rolling slowdown only if the requested action failed the stop
      // envelope. Use the same guards for every replacement, including the
      // steering already queued. The planned-lap speed floor still applies.
      const float floor=avoid_fallback && lap_plan_.ready() && !adaptive_avoid?settings.sim_avoid_min:0.f;
      const float slower=std::min(cmd.speed,std::max({.6f,floor,.65f*std::min(cmd.speed,car.v)}));
      bool out_of_time=false;
      for(float speed:{cmd.speed,slower}){
        for(float steer:{cmd.steer,car.steer,0.f,-.2f,.2f,-.4f,.4f}){
          // Optional compute budget (0 = unlimited, the evaluated behaviour).
          if(settings.fallback_budget_ms>0 && elapsed_ms()>settings.fallback_budget_ms){out_of_time=true;break;}
          Control candidate{steer,speed};
          if(!command_refusal(memory,car,committed,candidate,dyn,dt,ticks,nullptr)){
            cmd=candidate;refusal=nullptr;decision.reason="safe_alternative";break;
          }
        }
        if(!refusal || out_of_time || slower==cmd.speed)break;
      }
      if(refusal && out_of_time){
        // Out of time: one guard-checked brake command instead of a latched stop. The next
        // cycle plans again from the new state. If braking also fails, the stop latches.
        const Control brake{committed.steer,0};
        if(!command_refusal(memory,car,committed,brake,dyn,dt,ticks,nullptr)){
          cmd=brake;refusal=nullptr;decision.reason="budget_brake";
        }
      }
    }
    if(refusal)return stop_with(refusal);
    if(trace)trace->candidate_steer=cmd.steer;
    last_pose_=car;have_pose_=true;
    last_command_=cmd;
    decision.speed_limits=limits;
    decision.command=cmd;
    return decision;
  }
  bool merge_blocked()const{return merge_blocked_;}
  // The command a stop publishes. The estimated steering grows as v -> 0 (yaw rate / v), so a
  // stop that feeds it back can steer away from the arc the guard checked; optionally hold
  // the last published steering instead.
  Control stop_command(const State& car,const RouterSettings& settings)const{
    return {settings.stop_hold_steering && have_pose_ ? last_command_.steer : car.steer, 0};
  }
  // Shared sensor-only routing and policy features for online inference and
  // offline expert labels at learner-visited states. Does not publish commands.
  SectionDecision route_observation(const Track& track,const Memory& memory,const State& car,
                                    const std::deque<Scan>& scans,bool guided,double now,Observation& input,
                                    const RouterSettings& settings={}, float cruise=3.f, ControlTiming* timing=nullptr, CorridorTrace* trace=nullptr,int* route_index=nullptr){
    const bool plan_laps=settings.lap_planning;
    int n = track.closest(car);
    if(route_index)*route_index=n;
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
  static float narrow_bend_speed_cap(const Track& track,int index,const Memory& memory,const State& car,float cruise){
    constexpr float lateral_probe_m=.60f;
    constexpr float bend_speed_mps=3.9f;
    constexpr float bend_curvature=.22f;
    constexpr float deceleration=2.4f;
    const auto& points=track.points;
    float allowed=cruise,ahead=0;
    for(size_t i=0;i+3<points.size() && ahead<8.f;i+=3){
      const auto& a=points[(index+i)%points.size()];
      const auto& b=points[(index+i+3)%points.size()];
      const float ds=std::max(.01f,hypotf(b.x-a.x,b.y-a.y));
      const float k=wrap(b.yaw-a.yaw)/ds;
      if(fabsf(k)>bend_curvature){
        const float nx=-sinf(a.yaw),ny=cosf(a.yaw);
        for(float offset:{-lateral_probe_m,lateral_probe_m}){
          State edge{a.x+offset*nx,a.y+offset*ny,a.yaw};
          if(footprint(memory,edge,.03f)>.5f){
            allowed=std::min(allowed,sqrtf(bend_speed_mps*bend_speed_mps+2*deceleration*std::max(0.f,ahead-.48f*car.v)));
            break;
          }
        }
      }
      ahead+=ds;
    }
    return allowed;
  }
  static bool clear_for_boost(const Scan& scan,float speed){
    const float distance=.35f+.25f*speed+speed*speed/(2.f*1.8f);
    int rays=0;
    for(int b=0;b<beams;++b){
      const float angle=scan.explicit_angles?scan.angles[b]:
        scan.angle_min+(scan.angle_max-scan.angle_min)*b/(beams-1);
      if(fabsf(angle)>.12f)continue;
      if(scan.valid[b]<.5f || !(scan.ranges[b]>distance))return false;
      ++rays;
    }
    return rays>=3;
  }
  static float curve_speed_cap(const Track& track,size_t start,float speed,float cruise,float lateral_acceleration,float braking){
    // Read geometry ahead over the stopping distance, independent of map station.
    const auto& points=track.points;
    float allowed=cruise,distance=0;
    for(size_t k=0;k<points.size();++k){
      const auto& from=points[(start+k)%points.size()];
      const auto& to=points[(start+k+1)%points.size()];
      const float ds=hypotf(to.x-from.x,to.y-from.y);
      if(ds<.001f)continue;
      const float curvature=fabsf(wrap(to.yaw-from.yaw))/ds;
      const float remaining=std::max(0.f,distance-.25f*speed);
      allowed=std::min(allowed,sqrtf(lateral_acceleration/std::max(.01f,curvature)+2.f*braking*remaining));
      if(2.f*braking*remaining>=allowed*allowed)break;
      distance+=ds;
    }
    return allowed;
  }

  // Clearance over one short steering manoeuvre plus a full stop. Starting
  // from the delayed dynamic state includes steering slew in the comparison.
  static float scout_stopping_clearance(const Memory& memory,DynamicState dyn,Control cmd){
    float clearance=.16f;
    auto measure=[&](const State& pose){
      while(clearance>.001f && (footprint(memory,pose,clearance)>.5f ||
                                gym_front_blocked(memory,pose,clearance)))clearance-=.04f;
    };
    for(int i=1;i<=25;++i){
      dyn=physics_step(dyn,cmd,Vehicle{});
      if(i%3==0 || i==25)measure(basic(dyn));
    }
    State car=basic(dyn);
    const Control brake{car.steer,0};
    for(int i=0;car.v>=.045f && i<300;++i){
      car=advance(car,brake,.01f);
      measure(car);
    }
    return clearance;
  }
  static const char* command_refusal(const Memory& memory,const State& car,const Control& committed,
                                     Control cmd,DynamicState dyn,float dt,int ticks,CorridorTrace* trace){
    // Predict the command interval with the measured drive response. Keep the
    // slower braking envelope as an independent geometric clearance check.
    State prediction=predict_command_tick(car,committed,.01f);
    for(int i=0;i<ticks;++i){
      prediction=predict_command_tick(prediction,cmd,dt/ticks);
      if(footprint(memory,prediction)>.5f || gym_front_blocked(memory,prediction)) return "command_footprint";
    }
    Control brake{prediction.steer,0};
    for(int i=0;prediction.v>=.045f && i<300;++i){
      prediction=advance(prediction,brake,.01f);
      if(footprint(memory,prediction)>.5f || gym_front_blocked(memory,prediction)){
        if(trace){trace->brake_event=true;trace->brake_contact=prediction;}
        return "braking_footprint";
      }
    }
    if(prediction.v>=.045f) return "braking_horizon";

    // Include one additional physics tick before braking for sampled actuation.
    for(int i=0;i<ticks+1;++i){
      dyn=physics_step(dyn,cmd,Vehicle{});
      if(!std::isfinite(dyn.z[0]) || !std::isfinite(dyn.z[1]) || !std::isfinite(dyn.z[4]) ||
         footprint(memory,basic(dyn))>.5f || gym_front_blocked(memory,basic(dyn))) return "dynamic_command_footprint";
    }
    brake={dyn.z[2],0};
    for(int i=0;dyn.z[3]>=.045f && i<300;++i){
      dyn=physics_step(dyn,brake,Vehicle{});
      if(!std::isfinite(dyn.z[0]) || !std::isfinite(dyn.z[1]) || !std::isfinite(dyn.z[4]) ||
         footprint(memory,basic(dyn))>.5f || gym_front_blocked(memory,basic(dyn))) return "dynamic_braking_footprint";
    }
    if(!std::isfinite(dyn.z[3]) || dyn.z[3]>=.045f) return "braking_horizon";
    return nullptr;
  }
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
