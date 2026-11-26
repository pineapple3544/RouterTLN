#include "el/scenario_plan.hpp"
#include "el/scenario_guide.hpp"
#include "el/scenario_motion_search.hpp"
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
  require(evaluate(frame,map,{},lattice_candidates,p)<invalid_cost,"forward rollout on clear line");
  require(p.p[points-1].speed==0,"rollout keeps terminal stop");
  for(int i=1;i<points;++i){
    const float ds=p.p[i].s-p.p[i-1].s;
    require(ds>0 && fabsf(atanf(.3302f*p.p[i].kappa))<=.39f,"rollout steering bound");
  }
  Hypothesis h{};h.count=1;h.boxes[0]={3,0,default_obstacle_size};
  require(evaluate(frame,map,h,0,p)==invalid_cost,"nominal collides");
  bool bypass=false;for(int k=1;k<candidates&&!bypass;++k)bypass=evaluate(frame,map,h,k,p)<invalid_cost;
  require(bypass,"bypass exists");
  PathPoint car{};
  h.boxes[0]={0,.30f,default_obstacle_size};
  require(collision(map,h,car,0),"30 cm square corner at 45 degrees");
  h.boxes[0].y=.33f;
  require(!collision(map,h,car,0),"outside circumscribed footprint");
  cells[50*300+125]=Occupied;PathPoint hit{};hit.x=3;hit.y=0;
  require(collision(map,{},hit,.04f),"actual obstacle cell");
  auto guided_frame=frame;
  require(build_observed_guide(guided_frame,map),"connected guide around observed obstacle");
  require(fabsf(guided_frame.guide[24])>.15f,"guide clears the observed obstacle");
  auto connected=search_motion(guided_frame,map);
  require(connected.candidate>=0&&connected.p[points-1].speed==0,"steering sequence has braking reserve");
  for(const auto& q:connected.p)require(!collision(map,{},q,0),"steering sequence clears occupied cells");
  cells[50*300+125]=Free;
  // A near-field trajectory can terminate at rest before a blocked far section.
  for(int y=0;y<100;++y)cells[y*300+162]=Occupied;
  auto blocked_frame=frame;
  require(!build_observed_guide(blocked_frame,map),"no guide through full-width wall");
  for(int k=0;k<candidates;++k)
    require(evaluate(frame,map,{},k,p)==invalid_cost,"full horizon must reject barrier");
  Frame short_frame=frame;short_frame.spacing=station_step*.5f;
  for(int i=0;i<points;++i)short_frame.ref[i].x=i*short_frame.spacing;
  require(evaluate(short_frame,map,{},0,p)<invalid_cost&&p.p[points-1].speed==0,
          "short horizon must preserve terminal braking");
  std::cout<<"scenario geometry tests passed\n";
}
