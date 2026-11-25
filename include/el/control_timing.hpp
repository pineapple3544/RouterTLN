#pragma once
#include <array>
#include <chrono>
#include <ctime>
#include <ostream>

namespace el {
// Host elapsed time includes waits; thread CPU time excludes descheduling.
// No CUDA synchronization is added by the profiler.
class ControlTiming {
 public:
  enum Stage { scan, memory, route, observation, corridor, inputs, forward, readback, safety, packing, count };
  static constexpr std::array<const char*,count> names{
    "scan","memory","route","observation","corridor","inputs","forward","readback","safety","packing"
  };
  void mark(Stage stage) {
    auto current = sample();
    wall_[stage] += std::chrono::duration<double,std::milli>(current.wall-last_.wall).count();
    cpu_[stage] += (current.cpu_ns-last_.cpu_ns)*1e-6;
    last_ = current;
  }
  void restart_interval(){last_=sample();}
  double wall_ms(Stage stage)const{return wall_[stage];}
  double cpu_ms(Stage stage)const{return cpu_[stage];}
  static void header(std::ostream& out) {
    for (auto name:names) out << ',' << name << "_ms," << name << "_cpu_ms";
  }
  void write(std::ostream& out) const {
    for (unsigned i=0;i<count;++i) out << ',' << wall_[i] << ',' << cpu_[i];
  }
 private:
  struct Sample { std::chrono::steady_clock::time_point wall; long long cpu_ns; };
  static Sample sample() {
    timespec cpu{};
    clock_gettime(CLOCK_THREAD_CPUTIME_ID,&cpu);
    return {std::chrono::steady_clock::now(),cpu.tv_sec*1000000000LL+cpu.tv_nsec};
  }
  std::array<double,count> wall_{},cpu_{};
  Sample last_=sample();
};
}
