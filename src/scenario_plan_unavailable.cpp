#include "el/scenario_plan.hpp"
#include <stdexcept>
namespace el::scenario {
struct Optimizer::Impl{};
Optimizer::Optimizer(){throw std::runtime_error("scenario planner requires CUDA build (ROUTERTLN_INFERENCE_ONLY=OFF)");}
Optimizer::~Optimizer()=default;
void Optimizer::upload(MapView){throw std::runtime_error("CUDA unavailable");}
std::vector<Path> Optimizer::solve(const Frame&,const std::vector<Hypothesis>&){throw std::runtime_error("CUDA unavailable");}
}
