#include "el/scenario_plan.hpp"
#include <vector>
#include <cmath>
#include <stdexcept>
#include <iostream>
using namespace el::scenario;
int main(){
  std::vector<uint8_t> cells(300*100,Free);MapView map{cells.data(),300,100,.04f,-2,-2};
  Frame f;f.car.v=1.f;
  for(int i=0;i<points;++i)f.ref[i]={i*station_step,0,0,-.55f,.55f,-.8f,.8f};
  Hypothesis h;h.count=1;h.boxes[0]={3,0,.15f};
  Optimizer opt;opt.upload(map);auto result=opt.solve(f,{Hypothesis{},h});
  for(int j=0;j<2;++j){
    float best=invalid_cost;Path scratch;for(int k=0;k<candidates;++k)best=fminf(best,evaluate(f,map,j?h:Hypothesis{},k,scratch));
    if(result[j].candidate<0||fabsf(best-result[j].cost)>.002f)throw std::runtime_error("CPU/GPU score mismatch");
  }
  std::cout<<"scenario CUDA parity passed\n";
}
