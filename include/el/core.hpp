#pragma once
#include <array>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>
#include <cmath>
#ifdef __CUDACC__
#define EL_HD __host__ __device__
#else
#define EL_HD
#endif
namespace el {
constexpr int beams=181, history=4, crop=32, channels=3, context_size=16;
constexpr int grid_n=96, ref_n=80;
constexpr float pi=3.14159265358979323846f;
EL_HD inline float clamp(float a,float lo,float hi){return fminf(hi,fmaxf(lo,a));}
EL_HD inline float wrap(float a){return atan2f(sinf(a),cosf(a));}
struct State {float x=0,y=0,yaw=0,v=0,steer=0,yaw_rate=0;};
struct Control {float steer=0,speed=0;};
struct Ref {float x=0,y=0,yaw=0,speed=0,s=0;};
struct Grid {
  float resolution=.15f,x0=-3.6f,y0=-7.2f;
  std::array<float,grid_n*grid_n> cost{};
  EL_HD float at(float x,float y) const {
    int ix=(int)floorf((x-x0)/resolution),iy=(int)floorf((y-y0)/resolution);
    return ix<0||iy<0||ix>=grid_n||iy>=grid_n?1.f:cost[iy*grid_n+ix];
  }
};
struct Problem {State state;Grid grid;std::array<Ref,ref_n> ref;float max_speed=3.f;};
// Low-speed kinematic predictor; simulator retains its independent dynamic model.
EL_HD inline State advance_with_acceleration(State s,Control u,float dt,float a){
  u.steer=clamp(u.steer,-.40f,.40f);u.speed=clamp(u.speed,0.f,10.f);
  s.steer+=clamp(u.steer-s.steer,-3.2f*dt,3.2f*dt);
  float midv=fmaxf(0.f,s.v+.5f*a*dt);
  float rate=midv/.3302f*tanf(s.steer);
  s.x+=midv*cosf(s.yaw+.5f*rate*dt)*dt;
  s.y+=midv*sinf(s.yaw+.5f*rate*dt)*dt;
  s.yaw=wrap(s.yaw+rate*dt);s.yaw_rate=rate;s.v=fmaxf(0.f,s.v+a*dt);
  return s;
}
EL_HD inline State advance(State s,Control u,float dt){
  return advance_with_acceleration(s,u,dt,clamp((clamp(u.speed,0.f,10.f)-s.v)/.15f,-6.f,4.f));
}
// Dense footprint samples with a conservative boundary margin (meters).
template<class GridType> EL_HD inline float footprint(const GridType& g,const State& s,float margin=.04f){
  float c=cosf(s.yaw),q=sinf(s.yaw),value=0;
  for(int i=0;i<7;++i)for(int j=0;j<5;++j){
    float half_length=.29f+margin,half_width=.1016f+margin;
    float x=-half_length+i*(2*half_length/6),y=-half_width+j*(2*half_width/4);
    value=fmaxf(value,g.at(s.x+c*x-q*y,s.y+q*x+c*y));
  }return value;
}
EL_HD inline int nearest(const Problem& p,const State& s){
  int best=0;float d=1e20f;
  for(int i=0;i<ref_n;++i){float dx=s.x-p.ref[i].x,dy=s.y-p.ref[i].y,ds=dx*dx+dy*dy;
    if(ds<d){d=ds;best=i;}}
  return best;
}
struct Observation {
  std::array<float,history*2*beams> scans{};
  std::array<float,channels*crop*crop> map{};
  std::array<float,context_size> context{};
};
struct Scan {std::array<float,beams> ranges{},valid{},angles{};bool explicit_angles=false;State pose;double stamp=0;float angle_min=-2.35f,angle_max=2.35f,range_max=10;};
struct Waypoint {float x,y,yaw,speed,s;};
class Track {
 public:
  explicit Track(const std::string& path);
  int closest(const State&)const;
  Problem problem(const State&,const Grid&,float)const;
  Control pursuit(const State&,float)const;
  std::vector<Waypoint> points;
};
class Memory {
  friend struct OfflineSnapshot;
 public:
  explicit Memory(const std::string& yaml);
  void update(const Scan&);
  Grid local(const State&,float resolution=.15f)const;
  void features(const State&,double,Observation&)const;
  void reset();
  size_t occupied()const;
  std::vector<int> obstacle_cells()const;
  float at(float x,float y)const;
  float obstacle_at(float x,float y)const;
  size_t associated_wall_returns()const{return associated_wall_returns_;}
  bool base_free_disc(float x,float y,float radius)const;
  bool wall_hides_disc(float sx,float sy,float x,float y,float radius)const;
  void copy_occupancy(std::vector<uint8_t>&,int& width,int& height,float& resolution,float& x0,float& y0)const;
 private:
  int width_=0,height_=0;float resolution_=0,x0_=0,y0_=0;
  // schema-3 policy observations retain the inverse sensor model used in training.
  // Planner occupancy independently associates known-wall returns with the base map.
  std::vector<float> base_,odds_,feature_odds_;
  std::vector<double> seen_;
  std::vector<int> current_hits_;
  std::vector<uint8_t> current_hit_mask_;
  void replace_current_hits(std::vector<int> hits);
  size_t associated_wall_returns_=0;
  int index(float,float)const;
  float occupancy(int)const;
  float base_hit_range(float sx,float sy,float dx,float dy,float limit)const;
};
Observation observe(const std::deque<Scan>&,const State&,const Memory&,const Track&,int mode,int side);
bool safe_stop(const Problem&,Control,float first_dt=.025f);
bool valid_plan(const Problem&,const std::vector<Control>&,float dt);
enum class Mode:int {Race=0,Avoid=1,Mppi=2,Emergency=3};
class Router {
 public:
  Mode update(bool hazard,bool candidate_safe,bool mppi_valid,double now);
  Mode mode()const{return mode_;}
  bool wants_avoid(bool hazard,double now);
  bool allow_candidate(bool safe,double now);
  void reset(){mode_=Mode::Race;avoid_=false;clear_since_=-1;safe_since_=-1;}
 private:Mode mode_=Mode::Race;bool avoid_=false;double clear_since_=-1,safe_since_=-1;
};
}
