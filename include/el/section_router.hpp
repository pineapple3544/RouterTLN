#pragma once
#include "el/dynamics.hpp"
#include "el/race_speed.hpp"
#include "el/map_paths.hpp"
#include "el/lap_plan.hpp"
#include "el/policy.hpp"
#include "el/avoid_corridor.hpp"
#include <algorithm>
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
  float avoid_policy_steer=NAN;
  bool avoid_policy_verified=false;
  bool cluster_evaluated=false,cluster_changed=false,isolated_exit=false,exit_evaluated=false,exit_smoothed=false;
  float entry_blend=0.f;
};
class SectionRouter {
  friend struct OfflineSnapshot;
 public:
  const LapPlan& lap_plan()const{return lap_plan_;}
  bool avoiding() const {return avoiding_;}
  float distance_past_section()const{return position_-end_;}
  int side() const {return side_;}
  void reset() { *this = SectionRouter{}; }
  void stop(const char* reason) { if(!stop_reason_)stop_reason_=reason; }
  // Independent safety wrapper for the experimental planner. No TLN or lap plan
  // is evaluated here; keep the same measured actuator and braking envelopes.
  SectionDecision guard_external(const Memory& memory,const State& car,Control command,
                                 float cap,float dt,bool plan_valid,const char* reason){
    auto emergency=[&](const char* why){stop(why);return SectionDecision{Mode::Emergency,{car.steer,0},why,true};};
    if(stop_reason_)return emergency(stop_reason_);
    if(!std::isfinite(command.steer)||!std::isfinite(command.speed))return emergency("invalid_action");
    command.steer=clamp(command.steer,-.4f,.4f);command.speed=clamp(command.speed,0.f,cap);
    if(!plan_valid)command.speed=0;
    const Control committed=have_pose_?last_command_:Control{car.steer,car.v};
    DynamicState dyn=physics_step(delayed_state(car),committed,Vehicle{});
    const int ticks=std::max(1,int(std::ceil(dt/.01f-1e-5f)));
    const char* refusal=command_refusal(memory,car,committed,command,dyn,dt,ticks,nullptr);
    if(refusal){
      bool found=false;
      // Bounded alternatives only; never use a faster command than the planner.
      for(float speed:{.6f*std::min(command.speed,car.v),0.f}){
        for(float steer:{command.steer,car.steer,last_command_.steer}){
          Control trial{clamp(steer,-.4f,.4f),speed};
          if(!command_refusal(memory,car,committed,trial,dyn,dt,ticks,nullptr)){
            command=trial;reason="scenario_guard_brake";found=true;break;
          }
        }
        if(found)break;
      }
      if(!found)return emergency(refusal);
    }
    last_pose_=car;last_command_=command;have_pose_=true;
    SectionDecision d{Mode::Avoid,command,reason,!plan_valid};
    d.speed_limits.before_guard=command.speed;return d;
  }

  template<class RacePolicy>
  SectionDecision step(const Track& track, const Memory& memory, const State& car,
                       const std::deque<Scan>& scans, RacePolicy& race, Policy& avoid,
                       float cap, float dt, double now, const RouterSettings& settings={}, ControlTiming* timing=nullptr, CorridorTrace* trace=nullptr) {
    SpeedLimits limits;
    auto stop_with = [&](const char* reason) {
      stop(reason);
      return SectionDecision{Mode::Emergency, {car.steer, 0}, reason, avoiding_,limits};
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
    if(settings.whole_lap_path)lap_plan_.finish_route(track.points,requested_cap,settings.sim_avoid_min);
    if(decision.mode==Mode::Emergency)return stop_with(decision.reason);
    Control cmd=avoiding_?avoid.infer(input,timing):race.infer(input,timing);
    if (!std::isfinite(cmd.steer)||!std::isfinite(cmd.speed)) return stop_with("invalid_action");
    if(settings.smooth_isolated_exit && adaptive && decision.isolated_exit && lap_plan_.has_route()){
      const float threshold=settings.sim_avoid_min+.01f;
      const float other=std::min({cmd.speed,lap_plan_.cap(),lap_plan_.route_speed(route_index)});
      if(other>threshold && corridor_.speed_limit(settings.avoid_speed_cap,car.v)<=threshold){
        decision.exit_evaluated=true;
        if(corridor_.smooth_exit(track,route_index,memory,car.v,other)){
          State aim=car;
          if(have_pose_)for(int tick=0;tick<6;++tick)aim=predict_command_tick(aim,last_command_,.01f);
          corridor_.condition(input,car,settings.lookahead_scale,true,&aim);
          cmd=avoid.infer(input,timing);
          if(!std::isfinite(cmd.steer)||!std::isfinite(cmd.speed))return stop_with("invalid_action");
          decision.exit_smoothed=true;
        }
      }
    }
    limits.policy=cmd.speed;
    const bool adaptive_avoid=adaptive && !settings.sim_race_only;
    float available_race_cap=0.f;
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
          const bool aligned=std::max(fabsf(cmd.steer),fabsf(car.steer))<.09f;
          const float envelope=std::min(cap,lap_plan_.cap());
          available_race_cap=envelope;
          if(settings.avoid_policy_first){
            // A partly open straight can support an intermediate speed. Apply
            // the same stopping-distance inequality to that speed directly.
            const float visible=aligned && !scans.empty()?straight_boost_speed_limit(scans.back()):0.f;
            policy_cap=std::max(3.5f,std::min({requested_cap,envelope,visible}));
          }else if(!aligned || envelope<requested_cap-.01f || scans.empty() ||
                   !clear_for_boost(scans.back(),requested_cap)){
            policy_cap=3.5f;
          }
          cap=std::min(cap,policy_cap);
        }
        cmd.speed*=policy_cap/3.f;
      }
    }
    if(settings.lap_planning){
      limits.lap=avoiding_ && settings.sim_race_only && !settings.shared_speed_band ? lap_plan_.legacy_avoid_cap() : lap_plan_.cap();
      cap=std::min(cap,limits.lap);
    }
    if(settings.whole_lap_path && lap_plan_.has_route()){
      limits.planned=lap_plan_.route_speed(route_index);
      cap=std::min(cap,limits.planned);
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
    // This is the nominal band for both experts. A zero action stays zero;
    // steering, command latency, full-stop and sensor guards run afterwards.
    if(adaptive && settings.shared_speed_band && cmd.speed>0)
      cmd.speed=std::clamp(std::min(cmd.speed,cap),settings.sim_avoid_min,requested_cap);
    if (settings.lap_planning && cmd.speed<=0) lap_plan_.invalidate_scout();
    DynamicState dyn=delayed_state(car);
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
    if(settings.race_verified_pace && settings.straight_boost && settings.sim_race_only &&
       adaptive && !avoiding_ && !scans.empty() && cmd.speed>0 && have_pose_ && limits.curve>0 &&
       lap_plan_.cap()>=requested_cap-.01f && fabsf(cmd.steer)<=fabsf(car.steer)){
      // Use the same race policy in a gentle bend. Limit lateral acceleration
      // using both the measured wheel and the requested angle, then verify
      // the delayed arc and the following complete stop before accelerating.
      const float turn=std::max(fabsf(car.steer),fabsf(cmd.steer));
      const float corner=sqrtf(4.5f*.3302f/std::max(.001f,tanf(turn)));
      const float sight=race_sight_speed_limit(scans.back(),6.f);
      const float proposed=std::min({requested_cap,available_race_cap,corner,sight,
                                     limits.policy*settings.race_speed_scale*requested_cap/3.f});
      if(proposed>3.5f && proposed>cmd.speed){
        Control trial{cmd.steer,proposed};
        const int horizon=std::max(ticks,int(std::ceil(.25f/.01f)));
        if(!command_refusal(memory,car,committed,trial,dyn,.01f*horizon,horizon,nullptr))
          cmd=trial;
      }
    }
    if(settings.race_verified_pace && adaptive && !avoiding_ && have_pose_ && cmd.speed>0){
      // A steering or range error can erase the last admissible brake arc
      // before the next measurement. Correct a narrowing arc while a full
      // stop with reserve is still possible; keep the hard stop check below.
      const int horizon=std::max(ticks,int(std::ceil(settings.race_brake_preview_s/.01f)));
      constexpr float reserve=.06f;
      if(command_refusal(memory,car,committed,cmd,dyn,.01f*horizon,horizon,nullptr,reserve)){
        const auto& here=track.points[route_index];
        const float lat=-(car.x-here.x)*sinf(here.yaw)+(car.y-here.y)*cosf(here.yaw);
        const float toward=wrap(here.yaw-car.yaw)-atan2f(lat,1.f);
        const float step=copysignf(3.2f*dt,toward);
        bool found=false;
        for(float target:{cmd.speed,std::min(cmd.speed,std::max(.8f,car.v-.8f))}){
          for(float delta:{0.f,step,-step}){
            Control trial{clamp(cmd.steer+delta,-.4f,.4f),target};
            if(!command_refusal(memory,car,committed,trial,dyn,.01f*horizon,horizon,nullptr,reserve)){
              cmd=trial;found=true;break;
            }
          }
          if(found)break;
        }
      }
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
    if(avoiding_ && avoid.uses_corridor() && settings.lap_planning){
      decision.avoid_policy_steer=cmd.steer;
      // A passing rollout can skip the discrete search only when the original
      // TLN command also passes the standard one-cycle and braking envelopes.
      float neural=NAN;
      if(settings.avoid_policy_first && !command_refusal(memory,car,committed,cmd,dyn,dt,ticks,nullptr))
        neural=cmd.steer;
      if(settings.avoid_corridor_steer_seed)
        cmd.steer=input.context[4]*AvoidCorridor::steering_scale_rad;
      if(trace)trace->preview_seed_steer=cmd.steer;
      // Release only the previous section's approach limit. The current lap
      // plan still accounts for the next obstacle, including clustered boxes.
      if(settings.avoid_exit_pace && adaptive && std::isfinite(neural) &&
         std::isfinite(end_) && distance_past_section()>.35f && !decision.blocked &&
         !merge_blocked_ && !scans.empty() && fabsf(neural)<=fabsf(car.steer)){
        const float sight=race_sight_speed_limit(scans.back(),1.8f);
        const float exit_speed=std::min({limits.policy,limits.corridor,lap_plan_.cap(),
                                        settings.sim_avoid_max,requested_cap,sight,
                                        lap_plan_.has_route()?limits.planned:requested_cap,
                                        committed.speed+1.8f*dt});
        if(exit_speed>cmd.speed){
          const Control trial{neural,exit_speed};
          constexpr int preview_ticks=25;
          if(predicted_merge_stays_ready(track,memory,basic(dyn),trial,true) &&
             !command_refusal(memory,car,committed,trial,dyn,dt,ticks,nullptr) &&
             !command_refusal(memory,car,committed,trial,dyn,.25f,preview_ticks,nullptr)){
            const Control seed{cmd.steer,exit_speed};
            corridor_.preview_steer(dyn,seed,memory,dt,ticks,neural,&decision.avoid_policy_verified);
            if(decision.avoid_policy_verified){
              cmd=trial;
              limits.before_guard=exit_speed;
              decision.reason="avoid_exit_pace";
            }
          }
        }
      }
      if(!decision.avoid_policy_verified)
        cmd.steer=corridor_.preview_steer(dyn,cmd,memory,dt,ticks,neural,&decision.avoid_policy_verified,settings.race_verified_pace && adaptive);
    }
    if(avoiding_ && settings.whole_lap_path && lap_plan_.has_route()){
      const float steer=corridor_.measured_pursuit(track,route_index,basic(dyn),lap_plan_.scout_line(),memory,.55f);
      const Control turn{steer,cmd.speed};
      if(std::isfinite(steer) && fabsf(steer-cmd.steer)>.015f && !command_refusal(memory,car,committed,turn,dyn,.15f,15,nullptr) &&
         !command_refusal(memory,car,committed,turn,dyn,dt,ticks,nullptr)){
        cmd=turn;
        decision.avoid_policy_verified=decision.avoid_policy_verified && cmd.steer==decision.avoid_policy_steer;
      }
    }
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
    // A command that can stop now can still use up the steering reserve before
    // the next scan. Test continued actuation followed by a stop, including the
    // queued wheel commands, while the car can still choose a different arc.
    if(settings.avoid_policy_first && settings.lap_planning && !adaptive && !avoiding_ && cmd.speed>0 && have_pose_){
      constexpr int preview_ticks=14;
      constexpr float reserve_m=.05f;
      auto clears_preview=[&](Control trial){
        return !command_refusal(memory,car,committed,trial,dyn,.01f*preview_ticks,preview_ticks,nullptr,reserve_m);
      };
      if(!clears_preview(cmd)){
        const float wheel_step=3.2f*dt;
        const float slower=std::min(cmd.speed,.8f*car.v);
        bool found=false;
        for(float speed:{cmd.speed,slower}){
          for(float delta:{0.f,wheel_step,-wheel_step,2*wheel_step,-2*wheel_step}){
            Control trial{clamp(cmd.steer+delta,-.4f,.4f),speed};
            if(trial.steer==cmd.steer && trial.speed==cmd.speed)continue;
            if(clears_preview(trial) && !command_refusal(memory,car,committed,trial,dyn,dt,ticks,nullptr)){
              cmd=trial;decision.reason="clear_braking_arc";found=true;break;
            }
          }
          if(found || slower==cmd.speed)break;
        }
      }
    }
    if(trace)trace->candidate_steer=cmd.steer;
    // A scout hold can still replace the verified command before publication.
    decision.avoid_policy_verified=decision.avoid_policy_verified && cmd.steer==decision.avoid_policy_steer;
    const char* refusal=decision.avoid_policy_verified?nullptr:command_refusal(memory,car,committed,cmd,dyn,dt,ticks,trace);
    if(refusal && !avoiding_ && adaptive && settings.lap_planning && cmd.speed>0){
      // Keep the Race steering if a rolling deceleration clears the same
      // immediate and full-stop envelopes on this scan.
      Control slower{cmd.steer,std::min(cmd.speed,.8f*car.v)};
      if(slower.speed>0 && slower.speed<cmd.speed &&
         !command_refusal(memory,car,committed,slower,dyn,dt,ticks,nullptr)){
        cmd=slower;refusal=nullptr;decision.reason="race_brake_alternative";
      }
    }
    if(refusal && avoiding_ && avoid.uses_corridor() && settings.lap_planning && cmd.speed>0){
      // Try a rolling slowdown only if the requested action failed the stop
      // envelope. Use the same guards for every replacement, including the
      // steering already queued. The planned-lap speed floor still applies.
      const float floor=lap_plan_.ready() && !adaptive_avoid?settings.sim_avoid_min:0.f;
      const float slower=std::min(cmd.speed,std::max({.6f,floor,.65f*std::min(cmd.speed,car.v)}));
      for(float speed:{cmd.speed,slower}){
        for(float steer:{cmd.steer,car.steer,0.f,-.2f,.2f,-.4f,.4f}){
          Control candidate{steer,speed};
          if(!command_refusal(memory,car,committed,candidate,dyn,dt,ticks,nullptr)){
            cmd=candidate;refusal=nullptr;decision.reason="safe_alternative";break;
          }
        }
        if(!refusal || slower==cmd.speed)break;
      }
    }
    if(refusal)return stop_with(refusal);
    if(trace)trace->candidate_steer=cmd.steer;
    last_pose_=car;have_pose_=true;
    previous_verified_=decision.avoid_policy_verified;
    last_command_=cmd;
    decision.speed_limits=limits;
    decision.command=cmd;
    return decision;
  }
  bool merge_blocked()const{return merge_blocked_;}
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
    if(settings.whole_lap_path)lap_plan_.observe_route(points,n,car,memory);
    if(plan_laps)lap_plan_.observe(n,points.size(),lateral);
    float entry = 2.0f + .45f*car.v + car.v*car.v/10;
    float merge_distance = entry + .7f + fabsf(lateral);
    float horizon = std::min(length*.48f, entry+merge_distance+5);
    float first = INFINITY, end = -INFINITY;
    struct ObstacleLayer{
      const Memory& mem;
      float at(float x,float y)const{return mem.obstacle_at(x,y);}
    } obstacles{memory};
    const bool compare_guides=(settings.cluster_entry || settings.smooth_isolated_exit) && lap_plan_.has_route();
    // A half-second steering preview and the normal 0.20 s clear hold span
    // both obstacles only when their observed free gap is short enough.
    const float pair_gap=settings.entry_candidates==EntryCandidates::coupled
      ?std::min(merge_distance,.70f*std::max(0.f,car.v)):merge_distance;
    float last_obstacle=-INFINITY,second_obstacle=INFINITY;
    bool cluster_evaluated=false,cluster_changed=false,isolated_exit=false;
    float entry_blend=0.f;
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
      if(compare_guides && footprint(obstacles,sample,.19f)>.5f){
        if(std::isfinite(last_obstacle) && ahead-last_obstacle>.45f && ahead-last_obstacle<=pair_gap)
          second_obstacle=std::min(second_obstacle,ahead);
        last_obstacle=ahead;
      }
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

    if(!avoiding_)cluster_station_=-1;
    else if(compare_guides && std::isfinite(second_obstacle))
      cluster_station_=std::max(cluster_station_,position_+second_obstacle);
    if (avoiding_) {
      bool ready=position_>end_+.35f && !hazard && merge_clear &&
                 fabsf(lateral)<.30f && fabsf(wrap(car.yaw-here.yaw))<.32f;
      if (!ready) clear_since_=-1;
      else if (clear_since_<0) clear_since_=now;
      else if (now-clear_since_>=.20 ||
               (settings.avoid_policy_first && lap_plan_.ready() && now-clear_since_>=.05 &&
                predicted_merge_stays_ready(track,memory,car,last_command_))) {
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
      else if(!corridor_.plan(track,n,car,memory,plan_laps && (!returning || lap_plan_.has_route())?lap_plan_.avoidance_line():nullptr,trace,plan_laps))
        return {Mode::Emergency,{car.steer,0},"no_corridor",avoiding_};
      isolated_exit=returning && cluster_station_<0 && first>std::max(5.f,entry);
      const float to_second=cluster_station_-position_;
      if(settings.cluster_entry && lap_plan_.has_route() && have_pose_ && to_second>.5f && to_second<5.8f){
        DynamicState dyn=delayed_state(car);
        dyn=physics_step(dyn,last_command_,Vehicle{});
        const float speed=std::clamp(std::max(car.v,last_command_.speed),settings.sim_avoid_min,cruise);
        cluster_evaluated=true;
        cluster_changed=corridor_.choose_cluster_entry(track,n,memory,*lap_plan_.avoidance_line(),dyn,speed,to_second,
                                                     settings.entry_candidates,previous_verified_,&entry_blend);
      }
      if(settings.avoid_policy_first && settings.whole_lap_path && lap_plan_.has_route())
        corridor_.follow_scout_bend(track,n,car,memory,lap_plan_.scout_line(),*lap_plan_.avoidance_line());
      if(hazard)side_=corridor_.side_at(first+.35f);
      if(guided){
        State aim=car;
        if(returning && settings.avoid_policy_first && lap_plan_.ready() && have_pose_){
          // Compensate a short actuation delay only for the return pursuit angle.
          constexpr int preview_ticks=6;
          for(int tick=0;tick<preview_ticks;++tick)aim=predict_command_tick(aim,last_command_,.01f);
        }
        corridor_.condition(input,car,settings.lookahead_scale,returning,&aim);
      }
    }
    SectionDecision decision{avoiding_?Mode::Avoid:Mode::Race,{},reason,hazard};
    decision.cluster_evaluated=cluster_evaluated;decision.cluster_changed=cluster_changed;
    decision.entry_blend=entry_blend;
    decision.isolated_exit=isolated_exit;
    return decision;
  }
 private:
  DynamicState delayed_state(const State& car)const{
    float slip=0;
    if(have_pose_ && hypotf(car.x-last_pose_.x,car.y-last_pose_.y)>.015f){
      const float mid_yaw=last_pose_.yaw+.5f*wrap(car.yaw-last_pose_.yaw);
      slip=wrap(atan2f(car.y-last_pose_.y,car.x-last_pose_.x)-mid_yaw);
    }
    const float pending=have_pose_?last_command_.steer:car.steer;
    return {{car.x,car.y,car.steer,car.v,car.yaw,car.yaw_rate,slip},{pending,pending}};
  }
  static bool predicted_merge_stays_ready(const Track& track,const Memory& memory,State car,Control committed,
                                          bool following_avoid_exit=false){
    // After several clear scans, require the pending command to stay inside
    // the same lateral/heading limits and clear of occupied cells for 200 ms.
    const auto& ref=track.points[track.closest(car)];
    const float initial_offset=-(car.x-ref.x)*sinf(ref.yaw)+(car.y-ref.y)*cosf(ref.yaw);
    // Avoid still commands the bypass path. It can converge from outside
    // the Race handoff envelope, provided the offset does not grow.
    const float lateral_limit=following_avoid_exit?std::max(.30f,fabsf(initial_offset)+1e-5f):.30f;
    const float centered_tolerance=following_avoid_exit?.10f:0.f;
    for(int tick=1;tick<=20;++tick){
      car=predict_command_tick(car,committed,.01f);
      if(tick%4)continue;
      const auto& w=track.points[track.closest(car)];
      const float lateral=-(car.x-w.x)*sinf(w.yaw)+(car.y-w.y)*cosf(w.yaw);
      // Release early only if the delayed trajectory also ends closer to the
      // racing line. The ordinary settled handoff handles parallel offsets.
      // For an Avoid speed increment, a car already centered may track
      // within the same 10 cm tolerance as its certified TLN rollout.
      if(tick==20 && fabsf(lateral)>std::max(fabsf(initial_offset),centered_tolerance))return false;
      if(!std::isfinite(lateral) || !std::isfinite(car.yaw) ||
         fabsf(lateral)>=lateral_limit || fabsf(wrap(car.yaw-w.yaw))>=.32f ||
         footprint(memory,car,.13f)>.5f || gym_front_blocked(memory,car,.13f))return false;
    }
    return true;
  }
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
                                     Control cmd,DynamicState dyn,float dt,int ticks,CorridorTrace* trace,float extra_margin=0.f){
    // Predict the command interval with the measured drive response. Keep the
    // slower braking envelope as an independent geometric clearance check.
    State prediction=predict_command_tick(car,committed,.01f);
    for(int i=0;i<ticks;++i){
      prediction=predict_command_tick(prediction,cmd,dt/ticks);
      if(footprint(memory,prediction,.04f+extra_margin)>.5f || gym_front_blocked(memory,prediction,extra_margin)) return "command_footprint";
    }
    Control brake{prediction.steer,0};
    for(int i=0;prediction.v>=.045f && i<300;++i){
      prediction=advance(prediction,brake,.01f);
      if(footprint(memory,prediction,.04f+extra_margin)>.5f || gym_front_blocked(memory,prediction,extra_margin)){
        if(trace){trace->brake_event=true;trace->brake_contact=prediction;}
        return "braking_footprint";
      }
    }
    if(prediction.v>=.045f) return "braking_horizon";

    // Include one additional physics tick before braking for sampled actuation.
    for(int i=0;i<ticks+1;++i){
      dyn=physics_step(dyn,cmd,Vehicle{});
      if(!std::isfinite(dyn.z[0]) || !std::isfinite(dyn.z[1]) || !std::isfinite(dyn.z[4]) ||
         footprint(memory,basic(dyn),.04f+extra_margin)>.5f || gym_front_blocked(memory,basic(dyn),extra_margin)) return "dynamic_command_footprint";
    }
    brake={dyn.z[2],0};
    for(int i=0;dyn.z[3]>=.045f && i<300;++i){
      dyn=physics_step(dyn,brake,Vehicle{});
      if(!std::isfinite(dyn.z[0]) || !std::isfinite(dyn.z[1]) || !std::isfinite(dyn.z[4]) ||
         footprint(memory,basic(dyn),.04f+extra_margin)>.5f || gym_front_blocked(memory,basic(dyn),extra_margin)) return "dynamic_braking_footprint";
    }
    if(!std::isfinite(dyn.z[3]) || dyn.z[3]>=.045f) return "braking_horizon";
    return nullptr;
  }
  State last_pose_{};
  Control last_command_{};
  bool have_pose_=false;
  LapPlan lap_plan_;
  bool merge_blocked_=false;
  float cluster_station_=-1;
  bool previous_verified_=false;
  AvoidCorridor corridor_;
  float last_station_=-1,position_=0,end_=-INFINITY;
  double clear_since_=-1;
  int side_=0;
  bool avoiding_=false;
  const char* stop_reason_=nullptr;
};
}
