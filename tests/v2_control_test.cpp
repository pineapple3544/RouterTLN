#include "el/hierarchical_control.hpp"
#include <stdexcept>
#include <iostream>
using namespace el;
static void check(bool condition,const char* what){if(!condition)throw std::runtime_error(what);}
int main(){
  check(driving_phase(false,false,10,5)==DrivingPhase::Race,"race");
  check(driving_phase(false,false,4,5)==DrivingPhase::Prepare,"prepare");
  check(driving_phase(true,false,4,5)==DrivingPhase::Avoid,"avoid");
  check(driving_phase(true,true,INFINITY,5)==DrivingPhase::Merge,"merge");
  check(approach_speed_limit(4.25f,0,4,3)==3.f,"approach ceiling");
  check(approach_speed_limit(4.25f,INFINITY,4,3)==4.25f,"open cruise");
  State car{};car.v=3;check(handoff_dynamics_ready(car,0,3.5f),"handoff");
  car.steer=.35f;check(!handoff_dynamics_ready(car,0,3.5f),"handoff steering");
  DrivingPlan plan;plan.safe_speed=3.7f;plan.expert=Mode::Avoid;
  plan.reference_steer=.2f;plan.path_context[0]=.31f;
  Observation observation;apply_plan_path(observation,plan);
  check(observation.context[4]==.5f && observation.context[5]==.31f,"schema compatibility");
  check(constrain_policy({.2f,1.f},plan,1.f,3,3,.025f,true).speed==1.f,"no forced floor");
  check(constrain_policy({.5f,5.f},plan,1.f,3,3,.025f,true).speed<=3.10001f,"handoff acceleration");
  check(constrain_policy({.5f,5.f},plan,1.f,0,0,.025f,false).steer==.4f,"steer bound");
  int previews=0,checks=0;
  auto preview=[&](Control){++previews;return .1f;};
  auto accept=[&](Control)->const char*{++checks;return nullptr;};
  auto result=select_v2_control({.17f,3},true,true,true,0,3,preview,accept);
  check(!result.refusal && previews==0 && checks==1 && result.command.steer==.17f,"safe policy preserved");
  result=select_v2_control({.17f,3},true,false,true,0,3,preview,accept);
  check(result.preview_used && result.command.steer==.1f,"always-preview ablation");
  result=select_v2_control({.17f,3},true,true,false,0,3,preview,accept);
  check(result.preview_used,"tracking risk triggers preview");
  auto reject_original=[](Control c)->const char*{return c.steer==.17f?"unsafe":nullptr;};
  result=select_v2_control({.17f,3},true,true,true,0,3,preview,reject_original);
  check(!result.refusal && result.preview_used && result.command.steer==.1f,"guard triggers preview");
  auto slow_only=[](Control c)->const char*{return c.speed>2.f?"unsafe":nullptr;};
  result=select_v2_control({.17f,3},true,true,true,0,3,preview,slow_only);
  check(!result.refusal && result.fallback_used && result.command.speed<=2.f,"rolling fallback");
  auto deny=[](Control)->const char*{return "unsafe";};
  result=select_v2_control({.17f,3},true,true,true,0,3,preview,deny);
  check(result.refusal!=nullptr,"all candidates rejected");
  result=select_v2_control({.17f,3},false,true,true,0,3,preview,deny);
  check(result.refusal && !result.preview_used && !result.fallback_used,"race guard retained");
  std::cout<<"v2 control contracts passed\n";
}
