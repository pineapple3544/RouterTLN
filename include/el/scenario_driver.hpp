#pragma once
#include "el/scenario_plan.hpp"
#include <array>
#include <string>
#include <fstream>
namespace el::scenario {
constexpr int feature_count=64;
using Features=std::array<float,feature_count>;
class Residual {
 public:
  explicit Residual(const std::string& checkpoint="");
  Control infer(const Features&, bool* defer=nullptr)const;
  bool enabled()const{return enabled_;}
 private:
  bool enabled_=false, has_gate_=false;
  std::array<float,32> gate_{};
  std::array<float,1> gate_bias_{};
  std::array<float,48*feature_count> w1_{};std::array<float,48> b1_{};
  std::array<float,32*48> w2_{};std::array<float,32> b2_{};
  std::array<float,2*32> w3_{};std::array<float,2> b3_{};
};
struct Settings {
  float request_period=.10f,optional_budget_ms=55.f,plan_age=.55f;
  float lateral_accel=5.5f,accel=2.5f,brake=3.f;
  float nominal_size=default_obstacle_size,position_step=.20f;
  bool assume_passable=false,visibility_enabled=true;
  float visibility_cost_weight=.12f;
  std::string residual_checkpoint,record_path,no_path_snapshot,gpu_profile,gate_capture;
};
struct Action {
  Control base{},correction{},command{};
  bool valid_plan=false,race_clear=false;int hypotheses=0,chosen=-1;
  float race_speed_cap=0;
  float planner_ms=0,age_ms=-1,visible_m=0;
  uint64_t generation=0;
  int visibility_candidates=0,visibility_completed=0;
  float first_visible_lead=-1,response_slack=-1,reveal_eta=-1;
  const char* reason="scenario_wait";
};
class Driver {
 public:
  Driver(const Track&,const Memory&,Settings);~Driver();
  Driver(const Driver&)=delete;Driver& operator=(const Driver&)=delete;
  void invalidate();
  Action step(const Track&,const Memory&,const Scan&,float cap);
  // Only for data collection, after publication; not a scored timing run.
  void record(const Memory&,const Scan&,const Action&,const std::string& run);
 private:
  struct Impl;std::unique_ptr<Impl> impl_;
};
}
