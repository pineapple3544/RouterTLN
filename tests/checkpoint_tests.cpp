#include "el/checkpoint.hpp"
#include <iostream>
#include <unistd.h>
using namespace el;
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main(){try{
  torch::set_num_threads(2);auto path="/tmp/el-best-checkpoint-"+std::to_string(getpid())+".pt";
  Tln model;BestCheckpoint best(path);
  auto fill=[&](float x){torch::NoGradGuard guard;for(auto& p:model->parameters())p.fill_(x);};
  fill(1);require(best.consider(model,.3,1),"first epoch must save");
  fill(2);require(best.consider(model,.1,2),"lower validation must replace");
  fill(3);require(!best.consider(model,.2,3),"worse final epoch must not replace");
  require(!best.consider(model,.1,4),"tie keeps first best");
  bool rejected=false;try{best.consider(model,NAN,5);}catch(...){rejected=true;}require(rejected,"NaN rejected");
  Tln restored;torch::load(restored,path);
  for(auto& p:restored->parameters())require(torch::allclose(p,torch::full_like(p,2)),"saved weights must come from best epoch");
  require(best.epoch()==2&&best.loss()==.1,"best epoch metadata");
  rejected=false;try{BestCheckpoint other(path);}catch(...){rejected=true;}require(rejected,"new run must not overwrite");
  // A schema-4 policy keeps steering within its residual range around the
  // corridor direction, while the legacy head and speed channel stay usable.
  {
    torch::NoGradGuard grad;
    Observation o;o.context[4]=-.65f;
    auto inputs=tensors(o,torch::kCPU);
    auto raw=restored->forward(inputs[0],inputs[1],inputs[2]);
    restored->guided=true;
    auto guided=restored->forward(inputs[0],inputs[1],inputs[2]);
    require(fabsf(guided[0][0].item<float>()-(-.65f+.20f*raw[0][0].item<float>()))<1e-5,"corridor steering residual");
    require(guided[0][1].item<float>()==raw[0][1].item<float>(),"speed head");
  }
  std::filesystem::remove(path);std::cout<<"PASS best weights, non-improvement, tie, NaN, reload, overwrite guard\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
