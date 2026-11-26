#pragma once
#include "el/core.hpp"
#include <memory>
#include <vector>
#include <cstdint>
namespace el::scenario {
constexpr int points=49, knots=4, span=12, bins=5, candidates=625, batch_size=4;
constexpr float station_step=.125f, invalid_cost=1.e20f;
enum Cell:uint8_t { Unknown=0, Free=1, Occupied=2 };
struct MapView {
  const uint8_t* data=nullptr; int width=0,height=0; float resolution=.04f,x0=0,y0=0;
  EL_HD uint8_t cell(float x,float y)const{
    const int ix=(int)floorf((x-x0)/resolution),iy=(int)floorf((y-y0)/resolution);
    return ix<0||iy<0||ix>=width||iy>=height?uint8_t(Occupied):data[iy*width+ix];
  }
  EL_HD float at(float x,float y)const{return cell(x,y)==Occupied?1.f:0.f;}
};
struct Row {float x=0,y=0,yaw=0,lo=0,hi=0,wall_lo=0,wall_hi=0;};
struct Frame {
  State car{}; Row ref[points]{}; float offset=0,slope=0,cap=3.5f;
  float lateral_accel=3.f,accel=2.5f,brake=3.f;
};
// Full side length; a circumscribed disc covers every square orientation.
struct Obstacle {float x=0,y=0,size=.15f;};
struct Hypothesis {int count=0;Obstacle boxes[2]{};};
struct PathPoint {float x=0,y=0,yaw=0,kappa=0,speed=0,s=0;};
struct Path {
  PathPoint p[points]{}; Hypothesis hypothesis{};float cost=invalid_cost;
  int candidate=-1;
  double created=0;uint64_t generation=0;
};
EL_HD inline float knot_offset(const Row& r,int choice){
  const float zero=clamp(0,r.lo,r.hi);
  if(choice==0)return zero;
  if(choice==1)return .5f*(zero+r.hi);
  if(choice==2)return r.hi;
  if(choice==3)return .5f*(zero+r.lo);
  return r.lo;
}
// Conservative grid-cell overlap test against an expanded oriented rectangle.
// Unknown cells are not asserted free: visibility is separately speed-limited at runtime.
EL_HD inline bool collision(MapView map,const Hypothesis& h,const PathPoint& p,float margin){
  const float c=cosf(p.yaw),q=sinf(p.yaw),hx=.29f+margin,hy=.1016f+margin;
  const float cell_pad=.707107f*map.resolution;
  const float wx=fabsf(c)*hx+fabsf(q)*hy+cell_pad,wy=fabsf(q)*hx+fabsf(c)*hy+cell_pad;
  int x0=(int)floorf((p.x-wx-map.x0)/map.resolution),x1=(int)floorf((p.x+wx-map.x0)/map.resolution);
  int y0=(int)floorf((p.y-wy-map.y0)/map.resolution),y1=(int)floorf((p.y+wy-map.y0)/map.resolution);
  if(x0<0||y0<0||x1>=map.width||y1>=map.height)return true;
  for(int y=y0;y<=y1;++y)for(int x=x0;x<=x1;++x){
    if(map.data[y*map.width+x]!=Occupied)continue;
    const float dx=map.x0+(x+.5f)*map.resolution-p.x,dy=map.y0+(y+.5f)*map.resolution-p.y;
    if(fabsf(c*dx+q*dy)<=hx+cell_pad&&fabsf(-q*dx+c*dy)<=hy+cell_pad)return true;
  }
  for(int i=0;i<h.count;++i){
    const auto o=h.boxes[i];const float dx=o.x-p.x,dy=o.y-p.y;
    const float lx=fmaxf(0.f,fabsf(c*dx+q*dy)-hx),ly=fmaxf(0.f,fabsf(-q*dx+c*dy)-hy);
    const float r=.707107f*o.size;
    if(lx*lx+ly*ly<=r*r)return true;
  }
  return false;
}
EL_HD inline float evaluate(const Frame& f,MapView map,const Hypothesis& h,int id,Path& out){
  out.candidate=-1;out.cost=invalid_cost;out.hypothesis=h;
  float d[knots+1],m[knots+1];d[0]=f.offset;m[0]=f.slope*station_step*span;
  int code=id;
  for(int k=1;k<=knots;++k){const auto& r=f.ref[k*span];if(r.lo>r.hi)return invalid_cost;d[k]=knot_offset(r,code%bins);code/=bins;}
  for(int k=1;k<knots;++k)m[k]=.5f*(d[k+1]-d[k-1]);m[knots]=0;
  for(int i=0;i<points;++i){
    int k=i/span;if(k>=knots)k=knots-1;
    float t=float(i-k*span)/span,t2=t*t,t3=t2*t;
    float off=(2*t3-3*t2+1)*d[k]+(t3-2*t2+t)*m[k]+(-2*t3+3*t2)*d[k+1]+(t3-t2)*m[k+1];
    auto r=f.ref[i];if(i>0&&(off<r.lo-.015f||off>r.hi+.015f))return invalid_cost;
    out.p[i].x=r.x-sinf(r.yaw)*off;out.p[i].y=r.y+cosf(r.yaw)*off;
  }
  out.p[0].x=f.car.x;out.p[0].y=f.car.y;
  float length=0,soft=0;
  for(int i=0;i<points;++i){
    auto& p=out.p[i];const auto& a=out.p[i>0?i-1:0];const auto& b=out.p[i+1<points?i+1:points-1];
    p.yaw=atan2f(b.y-a.y,b.x-a.x);
    if(i==0&&fabsf(wrap(p.yaw-f.car.yaw))>.24f)return invalid_cost;
    if(i){const float ds=hypotf(p.x-a.x,p.y-a.y);if(ds<.035f||ds>.38f)return invalid_cost;length+=ds;}
    p.s=length;
    // Inflate by more than half the nominal station step for between-sample risk.
    if(collision(map,h,p,.085f))return invalid_cost;
    if(collision(map,h,p,.14f))soft+=.018f;
  }
  for(int i=0;i<points;++i){
    auto& p=out.p[i];const auto& a=out.p[i>0?i-1:0];const auto& b=out.p[i+1<points?i+1:points-1];
    p.kappa=wrap(b.yaw-a.yaw)/fmaxf(.035f,b.s-a.s);
    if(fabsf(atanf(.3302f*p.kappa))>.39f)return invalid_cost;
    p.speed=fminf(f.cap,sqrtf(f.lateral_accel/fmaxf(.01f,fabsf(p.kappa))));
    if(i){
      const float dk=atanf(.3302f*p.kappa)-atanf(.3302f*a.kappa);
      soft+=.09f*dk*dk;
      const float ds=p.s-a.s;
      p.speed=fminf(p.speed,3.0f*ds/fmaxf(.001f,fabsf(dk)));
    }
  }
  // Terminal braking reserve; profile is a rolling-horizon reference, not a guarantee.
  out.p[points-1].speed=0;
  for(int i=points-2;i>=0;--i){
    auto& a=out.p[i];const auto& b=out.p[i+1];
    a.speed=fminf(a.speed,sqrtf(b.speed*b.speed+2*f.brake*(b.s-a.s)));
  }
  float v=fminf(out.p[0].speed,fmaxf(0.f,f.car.v)),cost=0;
  for(int i=1;i<points;++i){
    auto& p=out.p[i];const float ds=p.s-out.p[i-1].s;
    p.speed=fminf(p.speed,sqrtf(v*v+2*f.accel*ds));
    cost+=2*ds/fmaxf(.1f,v+p.speed);v=p.speed;
  }
  // A mild continuity preference at the horizon end, not a hard in-course bonus.
  cost+=soft+.06f*fabsf(d[knots]);
  out.cost=cost;out.candidate=id;return cost;
}
struct Request {
  Frame frame{};MapView map{};std::vector<uint8_t> cells;
  std::vector<Hypothesis> hypotheses;
  uint64_t generation=0,epoch=0;double timestamp=0;
  float optional_budget_ms=60;
  MapView view()const{auto v=map;v.data=cells.data();return v;}
};
struct Result {uint64_t generation=0,epoch=0,revision=0;double timestamp=0;std::vector<Path> paths;float compute_ms=0;};
class Optimizer {
 public:
  Optimizer();~Optimizer();Optimizer(const Optimizer&)=delete;Optimizer& operator=(const Optimizer&)=delete;
  void upload(MapView map);
  std::vector<Path> solve(const Frame&,const std::vector<Hypothesis>&);
 private:struct Impl;std::unique_ptr<Impl> impl_;
};
}
