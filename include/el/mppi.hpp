#pragma once
#include "el/core.hpp"
#include <memory>
namespace el {
struct MppiConfig {int samples=512,horizon=32;float dt=.05f,temperature=1.5f,steer_sigma=.16f,speed_sigma=.7f;uint64_t seed=2026;};
struct Plan {Control action;std::vector<Control> sequence;bool valid=false;float cost=0,compute_ms=0;
  int source=0; // teacher: 0 weighted update, 1 shifted warm, 2 nominal proposal, 3 guide, 4 sampled trajectory
  float effective_samples=0;};
class Mppi {
 public:
  explicit Mppi(MppiConfig={});~Mppi();
  Mppi(const Mppi&)=delete;Mppi& operator=(const Mppi&)=delete;
  Plan solve(const Problem&,Control initial);
  void reset();
 private:struct Impl;std::unique_ptr<Impl> impl_;
};
}
