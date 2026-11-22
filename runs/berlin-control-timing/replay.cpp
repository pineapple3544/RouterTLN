#include "el/policy.hpp"
#include "el/dataset.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
using namespace el;
int main(int argc,char** argv){
  if(argc<3)return 2;
  auto records=read_dataset(argv[2]);
  if(records.empty())return 2;
  Policy cuda(argv[1]),cpu(argv[1],InferenceDevice::cpu);
  double latency[2]{};float steer_error=0,speed_error=0;
  for(const auto& r:records){
    Control controls[2];int i=0;
    for(auto* policy:{&cuda,&cpu}){
      auto start=std::chrono::steady_clock::now();
      controls[i]=policy->infer(r.observation);
      latency[i]+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();++i;
    }
    for(auto u:controls)if(!std::isfinite(u.speed)||!std::isfinite(u.steer))return 1;
    steer_error=std::max(steer_error,std::abs(controls[1].steer-controls[0].steer));
    speed_error=std::max(speed_error,std::abs(controls[1].speed-controls[0].speed));
  }
  std::printf("%s samples=%zu max_steer_error_rad=%.9g max_speed_error_mps=%.9g cuda_mean_ms=%.5f cpu_mean_ms=%.5f\n",argv[1],records.size(),steer_error,speed_error,latency[0]/records.size(),latency[1]/records.size());
  return steer_error>1e-5f || speed_error>1e-4f;
}
