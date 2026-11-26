#include "el/scenario_visibility.hpp"
#include <iostream>
#include <stdexcept>
using namespace el::scenario;
static void require(bool b,const char* msg){if(!b)throw std::runtime_error(msg);}
int main(){
  std::vector<uint8_t> cells(200*200,Free);
  MapView map{cells.data(),200,200,.05f,-5,-5};
  SensorGeometry sensor;Obstacle o{2,0,.20f};
  require(predicted_visible(map,-2,0,0,o,sensor),"clear straight");
  require(!predicted_visible(map,-2,0,el::pi,o,sensor),"FOV");
  auto short_range=sensor;short_range.range=1;
  require(!predicted_visible(map,-2,0,0,o,short_range),"range");
  for(int y=80;y<120;++y)cells[y*200+100]=Occupied;
  require(!predicted_visible(map,-2,0,0,o,sensor),"wall occludes");
  require(predicted_visible(map,.5f,0,0,o,sensor),"visible after wall");
  require(!ray_clear(map,-2,0,1,0,4),"DDA obstacle");
  require(ray_clear(map,-2,2,1,0,4),"DDA clear");
  require(fabsf(forward_distance(9.5f,.5f,10)-1)<.001f,"lap wrap");
  // An L-shaped approach: the inside wall hides a target beyond the turn.
  std::vector<VisibilityRow> rows{
    {{-3,-1,0,-.4f,.4f,-.6f,.6f},0},{{-1,-1,0,-.4f,.4f,-.6f,.6f},2},
    {{0,-1,el::pi/2,-.4f,.4f,-.6f,.6f},3},{{0,1,el::pi/2,-.4f,.4f,-.6f,.6f},5},
    {{0,3,0,-.4f,.4f,-.6f,.6f},7},{{3,3,-el::pi/2,-.4f,.4f,-.6f,.6f},10},
    {{3,-1,el::pi,-.4f,.4f,-.6f,.6f},14}};
  std::fill(cells.begin(),cells.end(),Free);
  Obstacle corner{0,2,.30f};
  float open_lead=first_visible_lead(rows,20,map,corner,6,0,sensor,6);
  for(int y=92;y<200;++y)for(int x=0;x<92;++x)cells[y*200+x]=Occupied;
  float blind_lead=first_visible_lead(rows,20,map,corner,6,0,sensor,6);
  require(open_lead>blind_lead+.5f&&blind_lead>0,"corner delays detection");
  std::cout<<"corner lead open="<<open_lead<<" blocked="<<blind_lead<<" m\n";
  std::fill(cells.begin(),cells.end(),Unknown);
  Frame f{};f.station=0;f.car.v=3;f.cap=3.5f;
  VisibilityAtlas a;a.length=20;
  VisibilityEntry early;early.obstacle={4,0,.15f};early.station=4;early.lead={4,4,4};early.gap=1;
  VisibilityEntry late=early;late.obstacle={4,1,.15f};late.lead={.6f,.6f,.6f};
  a.entries={early,late};auto ranks=a.rank(f,map);
  require(ranks.size()==2&&ranks[0].hypothesis.boxes[0].y==1,"late sight prioritized");
  require(ranks[0].slack<ranks[1].slack,"response slack");
  set_visibility_watches(f,ranks,.12f);require(f.watch_count==1,"urgent visibility target only");
  // Once the actual map reports a hypothetical center free, do not schedule it.
  cells[120*200+180]=Free;ranks=a.rank(f,map);
  require(ranks.size()==1&&ranks[0].hypothesis.boxes[0].y==0,"observed free excluded");
  std::cout<<"scenario visibility regression passed\n";
}
