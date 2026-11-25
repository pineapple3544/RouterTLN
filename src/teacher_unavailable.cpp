#include "el/teacher.hpp"
#include <stdexcept>

namespace el {
struct Teacher::Impl {};
Teacher::Teacher(TeacherConfig) {
  throw std::runtime_error("GPU teacher requires a CUDA build (ROUTERTLN_INFERENCE_ONLY=OFF)");
}
Teacher::~Teacher() = default;
Plan Teacher::solve(const TeacherProblem&, const std::vector<Control>&) {
  throw std::logic_error("GPU teacher is unavailable in the inference image");
}
}
