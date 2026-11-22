#pragma once
// Dynamics port attribution and BSD license: ../../THIRD_PARTY_DYNAMICS.txt.
#include "el/dynamics.hpp"
#include "el/mppi.hpp"
#include <memory>
namespace el {
constexpr int teacher_refs=256;
struct WorldCollisionGrid {
  static constexpr int capacity=600*600;std::array<uint8_t,capacity> cells{};
  int width=0,height=0;float resolution=.05f,x0=0,y0=0,ego_x=0,ego_y=0,c=1,s=0;
  EL_HD float at(float x,float y)const{float wx=ego_x+c*x-s*y,wy=ego_y+s*x+c*y;int ix=floorf((wx-x0)/resolution),iy=floorf((wy-y0)/resolution);
    return ix<0||iy<0||ix>=width||iy>=height?1.f:float(cells[iy*width+ix]);}
};
struct TeacherProblem {DynamicState state;Vehicle vehicle;Grid grid;Ref ref[teacher_refs]{};float max_speed=10;Control previous;WorldCollisionGrid collision;bool stable_grid=false;};
EL_HD inline float teacher_footprint(const TeacherProblem& p,const DynamicState& s,float margin=.04f){return p.stable_grid?footprint(p.collision,basic(s),margin):footprint(p.grid,basic(s),margin);}
enum class TeacherKind {Smppi,Svg};
struct TeacherConfig {
  TeacherKind kind=TeacherKind::Smppi;int samples=1024,horizon=40,iterations=2,guide_samples=128,guide_iterations=6;
  float temperature=1.f,steer_sigma=.10f,speed_sigma=.8f,guide_step=.005f;uint64_t seed=20261029;
};
// Eq.13-14 of SVG-MPPI: weighted quadratic fit to log density, with bounded fallback.
float fit_svg_sigma(const std::vector<float>& values,const std::vector<float>& log_density,float fallback,float low,float high);
bool teacher_valid(const TeacherProblem&,const std::vector<Control>&);
float teacher_cost(const TeacherProblem&,const std::vector<Control>&);
class Teacher {
 public:explicit Teacher(TeacherConfig);~Teacher();Plan solve(const TeacherProblem&,const std::vector<Control>& initial);void reset();
 private:struct Impl;std::unique_ptr<Impl> impl_;
};
}
