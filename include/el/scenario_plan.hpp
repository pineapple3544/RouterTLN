#pragma once
#include "el/core.hpp"
#include <memory>
#include <array>
#include <vector>
#include <cstdint>
namespace el::scenario {
constexpr int points=49, knots=4, span=12, bins=5, lattice_candidates=625, candidates=3*lattice_candidates, batch_size=4;
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
// Full side length; a circumscribed disc covers every square orientation.
constexpr float default_obstacle_size=.30f;
struct Obstacle {float x=0,y=0,size=default_obstacle_size;};
struct SensorGeometry {
  float angle_min=-2.35f,angle_max=2.35f,range=18.f,offset=.27f;
  int rays=beams,min_hits=2;
};
struct VisibilityWatch {Obstacle obstacle{};float ahead=0,weight=0;};
constexpr int visibility_watches=2;
struct Frame {
  State car{}; Row ref[points]{}; float spacing=station_step,lattice_radius=1.e6f; bool recovery_sweep=false; float offset=0,slope=0,cap=3.5f;
  float lateral_accel=3.f,accel=2.5f,brake=3.f,station=0;
  float guide[points]{};bool have_guide=false;
  SensorGeometry sensor{};
  VisibilityWatch watch[visibility_watches]{};
  int watch_count=0;
};
// Grid DDA: an occupied cell before the target surface blocks the beam.
// Unknown is transparent ONLY for a hypothetical visibility prediction,
// never evidence of observed free space.
EL_HD inline bool ray_clear(MapView map,float x,float y,float dx,float dy,float end){
  if(!map.data||!(map.resolution>0)||!(end>0))return false;
  int ix=int(floorf((x-map.x0)/map.resolution)),iy=int(floorf((y-map.y0)/map.resolution));
  const int sx=dx>0?1:dx<0?-1:0,sy=dy>0?1:dy<0?-1:0;
  float tx=sx?((map.x0+(ix+(sx>0))*map.resolution)-x)/dx:INFINITY;
  float ty=sy?((map.y0+(iy+(sy>0))*map.resolution)-y)/dy:INFINITY;
  const float stepx=sx?map.resolution/fabsf(dx):INFINITY,stepy=sy?map.resolution/fabsf(dy):INFINITY;
  float distance=0;
  for(int k=0;k<map.width+map.height+2&&distance<end;++k){
    if(ix<0||iy<0||ix>=map.width||iy>=map.height||map.data[iy*map.width+ix]==Occupied)return false;
    // At a corner crossing check both adjacent cells conservatively.
    if(fabsf(tx-ty)<1.e-6f){
      if(ix+sx<0||ix+sx>=map.width||iy+sy<0||iy+sy>=map.height)return false;
      if(tx<end&&(map.data[iy*map.width+ix+sx]==Occupied||map.data[(iy+sy)*map.width+ix]==Occupied))return false;
      distance=tx;ix+=sx;iy+=sy;tx+=stepx;ty+=stepy;
    }else if(tx<ty){distance=tx;ix+=sx;tx+=stepx;}
    else{distance=ty;iy+=sy;ty+=stepy;}
  }
  return distance>=end;
}
// Predict detection using the inscribed disc (not the larger collision disc),
// actual discrete beam directions, sensor FOV/range and static/dynamic occlusion.
EL_HD inline bool predicted_visible(MapView map,float x,float y,float yaw,const Obstacle& o,const SensorGeometry& g){
  if(g.rays<2||g.rays>4096||g.min_hits<1||g.min_hits>g.rays||
     !(g.angle_max>g.angle_min)||!(g.range>0)||!(o.size>0))return false;
  const float sx=x+g.offset*cosf(yaw),sy=y+g.offset*sinf(yaw);
  const float dx=o.x-sx,dy=o.y-sy,r=.5f*o.size,dist=hypotf(dx,dy);
  if(dist<=r||dist-r>g.range)return false;
  const float center=wrap(atan2f(dy,dx)-yaw),half=asinf(fminf(1.f,r/dist));
  const float increment=(g.angle_max-g.angle_min)/(g.rays-1);
  int begin=int(ceilf((center-half-g.angle_min)/increment)),end=int(floorf((center+half-g.angle_min)/increment));
  if(begin<0)begin=0;if(end>=g.rays)end=g.rays-1;
  if(end-begin+1<g.min_hits)return false;
  int hits=0;
  for(int i=begin;i<=end;++i){
    const float a=g.angle_min+increment*i,b=wrap(a-center);
    const float normal=dist*sinf(b),disc=r*r-normal*normal;
    if(disc<0)continue;
    const float stop=dist*cosf(b)-sqrtf(fmaxf(0.f,disc));
    if(stop<=0||stop>g.range)continue;
    if(ray_clear(map,sx,sy,cosf(yaw+a),sinf(yaw+a),stop)&&++hits>=g.min_hits)return true;
  }
  return false;
}
EL_HD inline float response_distance(float speed,float brake,float size){
  return .45f+.707107f*size+.20f*speed+speed*speed/(2*fmaxf(.1f,brake));
}
struct Hypothesis {int count=0;Obstacle boxes[2]{};};
struct PathPoint {float x=0,y=0,yaw=0,kappa=0,speed=0,s=0;};
struct Path {
  PathPoint p[points]{}; Hypothesis hypothesis{};float cost=invalid_cost;
  bool short_horizon=false,refined_lattice=false,recovery_sweep=false;
  int candidate=-1;
  double created=0;uint64_t generation=0;
};
EL_HD inline float knot_offset(const Row& r,int choice,float radius){
  const float zero=clamp(0,r.lo,r.hi);
  if(choice==0)return zero;
  const float hi=fminf(r.hi,zero+radius),lo=fmaxf(r.lo,zero-radius);
  if(choice==1)return .5f*(zero+hi);
  if(choice==2)return hi;
  if(choice==3)return .5f*(zero+lo);
  return lo;
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
struct Rejection {const char* reason="valid";int row=-1;};
EL_HD inline float reject_candidate(Rejection* trace,const char* reason,int row){
  if(trace){trace->reason=reason;trace->row=row;}
  return invalid_cost;
}
// Forward-integrate a rate-limited bicycle toward the lateral guides. This
// family covers bends where a Cartesian spline through four knots violates
// curvature even though a driveable trajectory passes between those knots.
EL_HD inline bool rollout(const Frame& f,const float* d,Path& path,bool guided){
  float x=f.car.x,y=f.car.y,yaw=f.car.yaw,steer=f.car.steer;
  int station=0;
  for(int i=0;i<points;++i){
    auto& p=path.p[i];p.x=x;p.y=y;p.yaw=yaw;
    if(i==points-1)break;
    float best=1.e9f;
    int nearest=station;
    for(int j=station;j<points && j<=station+5;++j){
      const auto& r=f.ref[j];
      float ex=x-r.x,ey=y-r.y,dist=ex*ex+ey*ey;
      if(dist<best){best=dist;nearest=j;}
    }
    station=nearest;
    const int ahead=int(.72f/f.spacing+.5f);
    const int target=station+ahead<points?station+ahead:points-1;
    const int knot=target/span<knots?target/span:knots-1;
    float q=float(target-knot*span)/span;
    float offset=d[knot]+q*(d[knot+1]-d[knot]);
    if(guided)offset+=f.guide[target];
    const auto& r=f.ref[target];
    float extra=fmaxf(0,float(station+ahead-(points-1))*f.spacing);
    float dx=r.x-offset*sinf(r.yaw)+extra*cosf(r.yaw)-x;
    float dy=r.y+offset*cosf(r.yaw)+extra*sinf(r.yaw)-y;
    float desired=clamp(atan2f(.6604f*(-sinf(yaw)*dx+cosf(yaw)*dy),fmaxf(.02f,dx*dx+dy*dy)),-.385f,.385f);
    const float rate=3.f*f.spacing/fmaxf(1.5f,f.car.v);
    steer+=clamp(desired-steer,-rate,rate);
    float turn=f.spacing*tanf(steer)/.3302f;
    x+=f.spacing*cosf(yaw+.5f*turn);y+=f.spacing*sinf(yaw+.5f*turn);yaw=wrap(yaw+turn);
  }
  return station>=points-12 && fabsf(wrap(yaw-f.ref[station].yaw))<.85f;
}
EL_HD inline float evaluate(const Frame& f,MapView map,const Hypothesis& h,int id,Path& out,Rejection* trace=nullptr){
  out.candidate=-1;out.cost=invalid_cost;out.hypothesis=h;
  out.recovery_sweep=f.recovery_sweep;
  out.short_horizon=f.spacing<station_step;
  out.refined_lattice=f.lattice_radius<1.e6f;
  float d[knots+1],m[knots+1];d[0]=f.offset;m[0]=f.slope*f.spacing*span;
  const bool shooting=id>=lattice_candidates;
  const bool guided=id>=2*lattice_candidates;
  if(guided&&!f.have_guide)return reject_candidate(trace,"no_guide",0);
  int code=id%lattice_candidates;
  for(int k=1;k<=knots;++k){const auto& r=f.ref[k*span];if(r.lo>r.hi)return reject_candidate(trace,"width",k*span);const int choice=code%bins;
    d[k]=guided?(choice==0?0.f:(choice<=2?.13f*choice:-.13f*(choice-2))):knot_offset(r,choice,f.lattice_radius);code/=bins;}
  for(int k=1;k<knots;++k)m[k]=.5f*(d[k+1]-d[k-1]);m[knots]=0;
  if(shooting){
    if(guided)d[0]=0;
    if(!rollout(f,d,out,guided))return reject_candidate(trace,"progress",points-1);
  }else for(int i=0;i<points;++i){
    int k=i/span;if(k>=knots)k=knots-1;
    float t=float(i-k*span)/span,t2=t*t,t3=t2*t;
    float off=(2*t3-3*t2+1)*d[k]+(t3-2*t2+t)*m[k]+(-2*t3+3*t2)*d[k+1]+(t3-t2)*m[k+1];
    auto r=f.ref[i];if(i>0&&(off<r.lo-.015f||off>r.hi+.015f))return reject_candidate(trace,"offset",i);
    out.p[i].x=r.x-sinf(r.yaw)*off;out.p[i].y=r.y+cosf(r.yaw)*off;
  }
  out.p[0].x=f.car.x;out.p[0].y=f.car.y;
  float length=0,soft=0;
  int guide_station=0;
  for(int i=0;i<points;++i){
    auto& p=out.p[i];const auto& a=out.p[i>0?i-1:0];const auto& b=out.p[i+1<points?i+1:points-1];
    if(!shooting)p.yaw=atan2f(b.y-a.y,b.x-a.x);
    if(i==0&&fabsf(wrap(p.yaw-f.car.yaw))>.24f)return reject_candidate(trace,"heading",i);
    if(i){const float ds=hypotf(p.x-a.x,p.y-a.y);if(ds<.035f||ds>.38f)return reject_candidate(trace,"spacing",i);length+=ds;}
    p.s=length;
    if(f.have_guide){
      float nearest=1.e9f;int station=guide_station;
      for(int j=guide_station;j<points && j<=guide_station+4;++j){
        float dx=p.x-f.ref[j].x,dy=p.y-f.ref[j].y,ds=dx*dx+dy*dy;
        if(ds<nearest){nearest=ds;station=j;}
      }
      guide_station=station;
      const auto& r=f.ref[station];
      float offset=-(p.x-r.x)*sinf(r.yaw)+(p.y-r.y)*cosf(r.yaw);
      float error=offset-f.guide[station];soft+=.055f*error*error;
    }
    // Inflate by more than half the nominal station step for between-sample risk.
    const float reach=f.recovery_sweep?fminf(1.f,p.s/.70f):1.f;
    if(f.recovery_sweep && i && a.s<.70f){
      const float from_yaw=i==1?f.car.yaw:a.yaw;
      const float dyaw=wrap(p.yaw-from_yaw),ds=p.s-a.s;
      const float sweep_pad=(ds+.40f*fabsf(dyaw))/8;
      for(int sample=0;sample<=4;++sample){
        const float q=sample*.25f;
        PathPoint sweep; sweep.x=a.x+q*(p.x-a.x);sweep.y=a.y+q*(p.y-a.y);sweep.yaw=from_yaw+q*dyaw;
        if(collision(map,h,sweep,.04f+.045f*fminf(1.f,(a.s+q*ds)/.70f)+sweep_pad))
          return reject_candidate(trace,"sweep",i);
      }
    }else{
      PathPoint pose=p;
      if(f.recovery_sweep && i==0)pose.yaw=f.car.yaw;
      if(collision(map,h,pose,.04f+.045f*reach))return reject_candidate(trace,"footprint",i);
    }
    if(collision(map,h,p,.21f))soft+=.045f;
    if(collision(map,h,p,.14f))soft+=.085f;
  }
  for(int i=0;i<points;++i){
    auto& p=out.p[i];const auto& a=out.p[i>0?i-1:0];const auto& b=out.p[i+1<points?i+1:points-1];
    p.kappa=wrap(b.yaw-a.yaw)/fmaxf(.035f,b.s-a.s);
    if(fabsf(atanf(.3302f*p.kappa))>.39f)return reject_candidate(trace,"curvature",i);
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
    const float attainable=fminf(p.speed,sqrtf(v*v+2*f.accel*ds));
    cost+=2*ds/fmaxf(.1f,v+attainable);v=attainable;
  }
  // Optional soft visibility cost: encourage earlier sight of the most urgent
  // unresolved obstacle hypotheses along the candidate's actual pose sequence.
  // Never turn those hypotheses into occupied cells or override safety limits.
  for(int w=0;w<f.watch_count&&w<visibility_watches;++w){
    const auto& target=f.watch[w];
    int goal=int(target.ahead/f.spacing+.5f);
    if(goal<2||goal>=points)continue;
    float lead=0,detect_speed=f.cap;int consecutive=0;
    for(int i=0;i<goal;i+=2){
      const auto& p=out.p[i];
      if(predicted_visible(map,p.x,p.y,p.yaw,target.obstacle,f.sensor)){
        if(++consecutive>=2){lead=out.p[goal].s-p.s;detect_speed=fmaxf(.5f,p.speed);break;}
      }else consecutive=0;
    }
    const float deficit=fmaxf(0.f,response_distance(detect_speed,f.brake,target.obstacle.size)-lead);
    const float seconds=deficit/fmaxf(.5f,f.cap);
    soft+=target.weight*seconds*seconds;
  }
  // A mild continuity preference at the horizon end, not a hard in-course bonus.
  const auto& end=out.p[points-1];const auto& goal=f.ref[points-1];
  float along=(goal.x-end.x)*cosf(goal.yaw)+(goal.y-end.y)*sinf(goal.yaw);
  cost+=soft+.06f*fabsf(d[knots])+.35f*fmaxf(0,along);
  out.cost=cost;out.candidate=id;return cost;
}
struct Request {
  Frame frame{};MapView map{};std::vector<uint8_t> cells;
  std::vector<Hypothesis> hypotheses;
  bool measured_obstacle=false, defer_speculation=false, gate_features_valid=false;
  std::array<float,64> gate_features{};
  uint64_t generation=0,epoch=0;double timestamp=0;
  float optional_budget_ms=60;
  MapView view()const{auto v=map;v.data=cells.data();return v;}
};
struct Result {uint64_t generation=0,epoch=0,revision=0;double timestamp=0;std::vector<Path> paths;float compute_ms=0;
  int visibility_candidates=0,visibility_completed=0;
  float first_visible_lead=-1,response_slack=-1,reveal_eta=-1;
};
class Optimizer {
 public:
  Optimizer();~Optimizer();Optimizer(const Optimizer&)=delete;Optimizer& operator=(const Optimizer&)=delete;
  void upload(MapView map);
  std::vector<Path> solve(const Frame&,const std::vector<Hypothesis>&,float* kernel_ms=nullptr);
 private:struct Impl;std::unique_ptr<Impl> impl_;
};
}
