#include "el/scenario_plan.hpp"
#include <iostream>
#include <vector>
#include <stdexcept>
using namespace el::scenario;
static void require(bool b,const char* msg){if(!b)throw std::runtime_error(msg);}
int main(){
  std::vector<uint8_t> cells(300*100,Free);
  MapView map{cells.data(),300,100,.04f,-2,-2};
  Frame frame{};frame.car.v=1.f;
  for(int i=0;i<points;++i)frame.ref[i]={i*station_step,0,0,-.55f,.55f,-.8f,.8f};
  Path p;require(evaluate(frame,map,{},0,p)<invalid_cost,"clear line");
  require(p.p[points-1].speed==0,"terminal stop profile");
  Hypothesis h{};h.count=1;h.boxes[0]={3,0,.15f};
  require(evaluate(frame,map,h,0,p)==invalid_cost,"nominal collides");
  bool bypass=false;for(int k=1;k<candidates&&!bypass;++k)bypass=evaluate(frame,map,h,k,p)<invalid_cost;
  require(bypass,"bypass exists");
  cells[50*300+125]=Occupied;PathPoint hit{};hit.x=3;hit.y=0;
  require(collision(map,{},hit,.04f),"actual obstacle cell");
  std::cout<<"scenario geometry tests passed\n";
}
