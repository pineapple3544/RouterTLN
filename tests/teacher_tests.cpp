#include "el/teacher.hpp"
#include <iostream>
#include <stdexcept>
using namespace el;
void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
int main(){try{
  std::vector<float> x,y;for(int i=-10;i<=10;++i){float v=i*.05f;x.push_back(v);y.push_back(-.5f*(v-.13f)*(v-.13f)/(.2f*.2f));}
  require(fabs(fit_svg_sigma(x,y,.1,.01,1)-.2)<1e-4,"SVG weighted Gaussian fit incorrect");
  require(fit_svg_sigma({1,1,1},{0,0,0},.17,.01,1)==.17f,"degenerate guide must retain covariance");
  for(auto kind:{TeacherKind::Smppi,TeacherKind::Svg}){TeacherConfig c;c.kind=kind;c.samples=128;c.horizon=12;Teacher teacher(c);TeacherProblem p;p.max_speed=2;
    for(int i=0;i<teacher_refs;++i)p.ref[i]={.1f*i,0,0,2,.1f*i};std::vector<Control> initial(c.horizon,{0,2});
    auto plan=teacher.solve(p,initial);require(plan.valid&&plan.action.speed>0,"free straight path must advance");
    require(plan.cost<=teacher_cost(p,initial)+.001f,"weighted update must not degrade a feasible incumbent");
    require(plan.effective_samples>=.99f&&plan.effective_samples<=c.samples+1,"effective sample count invalid");
    p.grid.cost.fill(1);teacher.reset();plan=teacher.solve(p,initial);require(!plan.valid&&plan.action.speed==0,"blocked footprint must reject teacher labels");
  }
  TeacherProblem world;world.stable_grid=true;world.collision.width=100;world.collision.height=100;world.collision.x0=-2.5;world.collision.y0=-2.5;
  for(int y=0;y<100;++y)world.collision.cells[y*100+60]=1;
  DynamicState physical;physical.z[0]=.4;float expected=teacher_footprint(world,physical);
  require(expected>.5,"world grid must detect footprint overlapping a wall");
  world.collision.ego_x=1;world.collision.ego_y=1;world.collision.c=0;world.collision.s=1;
  DynamicState rotated;rotated.z[0]=-1;rotated.z[1]=.6;rotated.z[4]=-pi*.5;
  require(teacher_footprint(world,rotated)==expected,"collision result must be invariant to ego-frame rotation and translation");
  world.collision.cells.fill(1);TeacherConfig cfg;cfg.samples=128;cfg.horizon=12;Teacher blocked_world(cfg);
  auto rejected=blocked_world.solve(world,std::vector<Control>(12,{0,2}));require(!rejected.valid,"CUDA world-grid path must reject occupied footprint");
  std::cout<<"Teacher Gaussian fit and CUDA free/blocked checks passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
