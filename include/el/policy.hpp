#pragma once
#include "el/control_timing.hpp"
#include "el/core.hpp"
#include <memory>
namespace el {
enum class InferenceDevice { cuda, cpu };
class Policy {
 public:explicit Policy(const std::string& checkpoint, InferenceDevice device=InferenceDevice::cuda);~Policy();
  bool uses_corridor()const;
  bool is_unified()const;
  Control infer(const Observation&, ControlTiming* timing=nullptr);
 private:struct Impl;std::unique_ptr<Impl> impl_;
};
}
