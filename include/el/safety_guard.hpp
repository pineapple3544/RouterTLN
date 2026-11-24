#pragma once
#include "el/avoid_corridor.hpp"

namespace el {
// The v1 command and full braking envelope, moved without changing its math.
// Shared by legacy and hierarchical controllers.
class SafetyGuard {
 public:
  static const char* refusal(const Memory& memory,const State& car,const Control& committed,
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
};
}
