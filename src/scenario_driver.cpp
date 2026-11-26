#include "el/scenario_driver.hpp"
#include "el/avoid_corridor.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <stdexcept>
#include <limits>
#include <nlohmann/json.hpp>
namespace el::scenario {
namespace {
using Clock=std::chrono::steady_clock;
double seconds(){return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
template<size_t N> void load_values(std::istream& f,std::array<float,N>& a){for(auto& v:a)if(!(f>>v)||!std::isfinite(v))throw std::runtime_error("invalid RCN weights");}
template<size_t O,size_t I> std::array<float,O> layer(const std::array<float,I>& x,const std::array<float,O*I>& w,const std::array<float,O>& b,bool relu){
  std::array<float,O> out{};for(size_t i=0;i<O;++i){float y=b[i];for(size_t j=0;j<I;++j)y+=w[i*I+j]*x[j];out[i]=relu?fmaxf(0.f,y):tanhf(y);}return out;
}
struct Width {float lo=0,hi=0;};
std::vector<Width> analyze(const Track& t,const Memory& m){
  std::vector<Width> out;out.reserve(t.points.size());
  for(const auto& p:t.points){
    Width w;for(float sign:{-1.f,1.f}){
      float bound=0;
      for(float d=.02f;d<=3.f;d+=.02f){float x=p.x-sign*d*sinf(p.yaw),y=p.y+sign*d*cosf(p.yaw);if(m.static_blocked(x,y))break;bound=d;}
      if(sign<0)w.lo=-bound;else w.hi=bound;
    }out.push_back(w);
  }return out;
}
Frame frame_for(const Track& t,const std::vector<Width>& widths,const State& car,float cap,const Settings& cfg){
  Frame f;f.car=car;f.cap=cap;f.lateral_accel=cfg.lateral_accel;f.accel=cfg.accel;f.brake=cfg.brake;
  int n=int(t.points.size()),near=t.closest(car),idx=near;float fraction=0,best=INFINITY;
  for(int di=-2;di<=2;++di){
    int j=(near+n+di)%n;auto a=t.points[j],b=t.points[(j+1)%n];
    float dx=b.x-a.x,dy=b.y-a.y,q=clamp(((car.x-a.x)*dx+(car.y-a.y)*dy)/fmaxf(1.e-8f,dx*dx+dy*dy),0,1);
    float dist=hypotf(car.x-a.x-q*dx,car.y-a.y-q*dy);
    if(dist<best){best=dist;idx=j;fraction=q;}
  }
  auto segment=[&](int j){auto a=t.points[j],b=t.points[(j+1)%n];return fmaxf(.001f,hypotf(b.x-a.x,b.y-a.y));};
  float remaining=fraction*segment(idx);
  for(int i=0;i<points;++i){
    while(remaining>segment(idx)){remaining-=segment(idx);idx=(idx+1)%n;}
    int j=(idx+1)%n;auto a=t.points[idx],b=t.points[j];float q=remaining/segment(idx);
    float yaw=wrap(a.yaw+q*wrap(b.yaw-a.yaw));
    float wl=widths[idx].lo*(1-q)+q*widths[j].lo,wh=widths[idx].hi*(1-q)+q*widths[j].hi;
    f.ref[i]={a.x+q*(b.x-a.x),a.y+q*(b.y-a.y),yaw,wl+.19f,wh-.19f,wl,wh};
    remaining+=station_step;
  }
  const auto& r=f.ref[0];f.offset=-sinf(r.yaw)*(car.x-r.x)+cosf(r.yaw)*(car.y-r.y);
  f.slope=clamp(tanf(clamp(wrap(car.yaw-r.yaw),-.8f,.8f)),-1.f,1.f);
  return f;
}
std::vector<Hypothesis> generate(const Frame& f,MapView map,const Settings& cfg){
  std::vector<Hypothesis> out(1); // Actual observations always receive the first batch.
  auto add=[&](float size,int stride,bool refine){
    const float rad=.707107f*size;
    for(int i=10;i<points-4;i+=stride){
      const auto& r=f.ref[i];float lo=r.wall_lo+rad,hi=r.wall_hi-rad;if(lo>hi)continue;
      float width=hi-lo;
      int columns=std::min(9,std::max(1,int(ceilf(width/cfg.position_step))+1));
      if(refine&&width<.5f)columns=std::max(columns,5);
      for(int k=0;k<columns;++k){
        float d=columns==1?.5f*(lo+hi):lo+width*k/(columns-1);
        // Only prune impossible-pass placements if the generator contract is verified.
        if(cfg.assume_passable&&fmaxf(d-rad-r.wall_lo,r.wall_hi-d-rad)<.2032f)continue;
        float x=r.x-sinf(r.yaw)*d,y=r.y+cosf(r.yaw)*d;
        if(map.cell(x,y)!=Unknown)continue;
        bool hit=false;for(int a=-1;a<=1;++a)for(int b=-1;b<=1;++b)
          if(map.cell(x+a*rad*.7f,y+b*rad*.7f)==Occupied)hit=true;
        if(hit)continue;
        Hypothesis h;h.count=1;h.boxes[0]={x,y,size};out.push_back(h);
        if(out.size()>=120)return;
      }
    }
  };
  add(cfg.nominal_size,6,false);
  // Primary size and imminent positions first; other sizes spend only optional budget.
  add(.20f,6,false);add(.30f,6,false);add(.10f,6,true);
  const size_t singles=out.size();
  for(size_t a=1;a<singles&&out.size()<144;++a){
    if(fabsf(out[a].boxes[0].size-cfg.nominal_size)>.001f)continue;
    for(size_t b=a+1;b<singles&&out.size()<144;++b){
      auto x=out[a].boxes[0],y=out[b].boxes[0];
      float dist=hypotf(x.x-y.x,x.y-y.y);
      if(fabsf(y.size-cfg.nominal_size)>.001f||dist<.65f||dist>2.5f)continue;
      Hypothesis h;h.count=2;h.boxes[0]=x;h.boxes[1]=y;out.push_back(h);break;
    }
  }
  return out;
}
int nearest_path(const Path& p,const State& car){
  int best=-1;float dist=INFINITY;
  for(int i=0;i<points;++i){
    float d=hypotf(car.x-p.p[i].x,car.y-p.p[i].y);
    if(d<dist&&fabsf(wrap(car.yaw-p.p[i].yaw))<.65f){dist=d;best=i;}
  }return dist<=.32f?best:-1;
}
bool path_valid(const Path& p,int n,const Memory& m,const State& car){
  if(p.candidate<0||n<0||n>=points-5||p.p[points-1].s-p.p[n].s<.7f)return false;
  if(footprint(m,car,.04f)>.5f)return false;
  for(int i=n;i<points;++i){
    State s{p.p[i].x,p.p[i].y,p.p[i].yaw};
    if(footprint(m,s,.065f)>.5f||gym_front_blocked(m,s,.02f))return false;
  }return true;
}
bool recently_free(const Memory& m,float x,float y,double stamp){
  if(m.scenario_cell(x,y,stamp)==Occupied)return false;
  // Sparse lidar rays can leave a one-cell sampling gap; a nearby ray is evidence,
  // not proof. Retain a conservative stopping envelope and the independent guard.
  if(m.scenario_cell(x,y,stamp)==Free)return true;
  for(float dx:{-.04f,0.f,.04f})for(float dy:{-.04f,0.f,.04f})
    if(m.scenario_cell(x+dx,y+dy,stamp)==Free)return true;
  return false;
}
float visible_distance(const Path& p,int n,const Memory& m,double stamp){
  float distance=0;
  for(int i=n+2;i<points;++i){
    const auto& q=p.p[i];bool clear=true;
    for(float d:{-.10f,0.f,.10f}){
      float x=q.x+.29f*cosf(q.yaw)-d*sinf(q.yaw),y=q.y+.29f*sinf(q.yaw)+d*cosf(q.yaw);
      if(!recently_free(m,x,y,stamp)){clear=false;break;}
    }
    if(!clear)break;
    distance=q.s-p.p[n].s;
  }return distance;
}
float braking_cap(float distance,float brake){
  constexpr float delay=.20f;
  float a=brake*delay;return fmaxf(0.f,sqrtf(a*a+2*brake*fmaxf(0.f,distance-.12f))-a);
}
struct Evidence {float x=0,y=0,width=0,length=0;bool valid=false;};
std::vector<std::array<float,2>> hits(const Memory& m,const Scan& scan){
  std::vector<std::array<float,2>> out;const auto& car=scan.pose;
  for(int b=0;b<beams;++b){
    if(scan.valid[b]<.5f||scan.ranges[b]>=scan.range_max-.02f)continue;
    float a=car.yaw+(scan.explicit_angles?scan.angles[b]:scan.angle_min+(scan.angle_max-scan.angle_min)*b/(beams-1));
    float x=car.x+.27f*cosf(car.yaw)+scan.ranges[b]*cosf(a),y=car.y+.27f*sinf(car.yaw)+scan.ranges[b]*sinf(a);
    if(m.obstacle_at(x,y)>.5f)out.push_back({x,y});
  }return out;
}
Features features(const Scan& scan,const Memory& m,const Path& p,int n,Control base,
                  const std::vector<std::array<float,2>>& observed){
  Features x{};const auto& car=scan.pose;
  for(int k=0;k<24;++k){
    float r=5;int count=0,begin=k*beams/24,end=(k+1)*beams/24;
    for(int i=begin;i<end;++i)if(scan.valid[i]>.5f){r=fminf(r,scan.ranges[i]);++count;}
    x[k]=clamp(r/5,0,1);x[k+24]=float(count)/std::max(1,end-begin);
  }
  const auto& q=p.p[n];float c=cosf(q.yaw),s=sinf(q.yaw);
  x[48]=car.v/3.5f;x[49]=car.steer/.4f;x[50]=car.yaw_rate/5;
  x[51]=base.steer/.4f;x[52]=base.speed/3.5f;
  x[53]=(-s*(car.x-q.x)+c*(car.y-q.y))/.5f;x[54]=wrap(car.yaw-q.yaw);x[55]=q.kappa/1.2f;
  Obstacle target{};target.x=q.x+1.5f*c;target.y=q.y+1.5f*s;target.size=0;
  if(p.hypothesis.count){
    target=p.hypothesis.boxes[0];for(int i=1;i<p.hypothesis.count;++i)
      if(hypotf(p.hypothesis.boxes[i].x-car.x,p.hypothesis.boxes[i].y-car.y)<hypotf(target.x-car.x,target.y-car.y))target=p.hypothesis.boxes[i];
  }else{
    float best=3.f;for(auto h:observed){
      float dx=h[0]-car.x,dy=h[1]-car.y,d=hypotf(dx,dy);
      if(d<best&&dx*cosf(car.yaw)+dy*sinf(car.yaw)>0){best=d;target.x=h[0];target.y=h[1];}
    }
  }
  x[56]=target.size/.3f;
  float lo=INFINITY,hi=-INFINITY,left=INFINITY,right=-INFINITY;int count=0;
  for(auto h:observed){
    float dx=h[0]-target.x,dy=h[1]-target.y;if(hypotf(dx,dy)>.45f)continue;
    float a=c*dx+s*dy,b=-s*dx+c*dy;lo=fminf(lo,a);hi=fmaxf(hi,a);left=fminf(left,b);right=fmaxf(right,b);++count;
  }
  if(count>=2){x[57]=.5f*(lo+hi);x[58]=(left+right);x[59]=(right-left)/.5f;x[60]=(hi-lo)/.5f;x[61]=1;}
  for(int k=0;k<2;++k){
    const auto& look=p.p[std::min(points-1,n+6)];float d=0,sign=k?1.f:-1.f;
    for(float a=.04f;a<=.8f;a+=.04f){if(m.at(look.x-sign*a*sinf(look.yaw),look.y+sign*a*cosf(look.yaw))>.5f)break;d=a;}
    x[62+k]=d/.8f;
  }
  for(auto& a:x)a=std::isfinite(a)?clamp(a,-4,4):0;
  return x;
}
}
Residual::Residual(const std::string& path){
  if(path.empty())return;
  std::ifstream f(path);std::string magic,extra;if(!(f>>magic)||magic!="RCN1")throw std::runtime_error("RCN1 checkpoint required");
  load_values(f,w1_);load_values(f,b1_);load_values(f,w2_);load_values(f,b2_);load_values(f,w3_);load_values(f,b3_);
  if(f>>extra)throw std::runtime_error("unexpected RCN checkpoint data");
  enabled_=true;
}
Control Residual::infer(const Features& x)const{
  if(!enabled_)return {};
  auto a=layer<48,feature_count>(x,w1_,b1_,true);auto b=layer<32,48>(a,w2_,b2_,true);
  auto out=layer<2,32>(b,w3_,b3_,false);return {.08f*out[0],.5f*out[1]};
}
struct Driver::Impl {
  Settings cfg;std::vector<Width> widths;Residual net;
  std::mutex mutex;std::condition_variable cv;std::unique_ptr<Request> pending;
  std::shared_ptr<const Result> published;std::atomic<bool> stopping{false},failed{false};
  std::atomic<uint64_t> epoch{1};std::thread worker;
  uint64_t generation=0,used_generation=0;size_t used_count=0;
  double last_request=-1;Path active{};uint64_t active_generation=0;double active_stamp=0;
  Features last_features{};int last_near=-1;std::ofstream log;int log_tick=0;
  Impl(const Track& t,const Memory& m,Settings c):cfg(std::move(c)),widths(analyze(t,m)),net(cfg.residual_checkpoint){
    if(!(cfg.request_period>=.05f&&cfg.request_period<=.5f&&cfg.optional_budget_ms>=0&&cfg.optional_budget_ms<=250&&
      cfg.plan_age>=cfg.request_period&&cfg.plan_age<=2&&cfg.lateral_accel>0&&cfg.accel>0&&cfg.brake>0&&
      cfg.nominal_size>=.08f&&cfg.nominal_size<=.4f&&cfg.position_step>=.08f&&cfg.position_step<=.5f))
      throw std::invalid_argument("invalid scenario settings");
    if(!cfg.record_path.empty()){log.open(cfg.record_path);if(!log)throw std::runtime_error("cannot open scenario training log");}
    worker=std::thread([this]{run();});
  }
  ~Impl(){stopping=true;cv.notify_all();if(worker.joinable())worker.join();}
  void run(){
    try{
      Optimizer opt;
      while(!stopping){
        std::unique_ptr<Request> req;
        {std::unique_lock<std::mutex> lock(mutex);cv.wait(lock,[&]{return stopping||pending!=nullptr;});if(stopping)return;req=std::move(pending);}
        auto begin=seconds();auto view=req->view();opt.upload(view);
        Result result;result.generation=req->generation;result.epoch=req->epoch;result.timestamp=req->timestamp;
        auto save=[&](const std::vector<Path>& paths){
          for(auto& p:paths)if(p.candidate>=0&&std::isfinite(p.cost))result.paths.push_back(p);
          result.compute_ms=float((seconds()-begin)*1000);
          auto ready=std::make_shared<const Result>(result);
          if(epoch.load()==req->epoch)std::atomic_store(&published,std::move(ready));
        };
        save(opt.solve(req->frame,{Hypothesis{}}));
        auto hypotheses=generate(req->frame,view,cfg);
        for(size_t i=1;i<hypotheses.size()&&!stopping&&epoch.load()==req->epoch;){
          if((seconds()-begin)*1000>=req->optional_budget_ms)break;
          {std::lock_guard<std::mutex> lock(mutex);if(pending)break;}
          size_t count=std::min<size_t>(batch_size,hypotheses.size()-i);
          std::vector<Hypothesis> batch(hypotheses.begin()+i,hypotheses.begin()+i+count);
          save(opt.solve(req->frame,batch));i+=count;
        }
      }
    }catch(const std::exception&){failed=true;}
  }
};
Driver::Driver(const Track& t,const Memory& m,Settings c):impl_(new Impl(t,m,std::move(c))){}
Driver::~Driver()=default;
void Driver::invalidate(){
  auto& a=*impl_;a.epoch.fetch_add(1);
  {std::lock_guard<std::mutex> lock(a.mutex);a.pending.reset();}
  std::atomic_store(&a.published,std::shared_ptr<const Result>{});
  a.active=Path{};a.active_stamp=0;a.last_request=-1;a.used_generation=0;a.used_count=0;a.last_near=-1;
}
Action Driver::step(const Track& t,const Memory& m,const Scan& scan,float cap){
  auto& a=*impl_;const auto& car=scan.pose;Action out;out.command=out.base={car.steer,0};a.last_near=-1;
  if(a.failed){out.reason="scenario_gpu_error";return out;}
  double now=seconds();auto observed=hits(m,scan);
  if(a.last_request<0||now-a.last_request>=a.cfg.request_period){
    auto req=std::make_unique<Request>();req->frame=frame_for(t,a.widths,car,cap,a.cfg);
    req->generation=++a.generation;req->epoch=a.epoch;req->timestamp=now;req->optional_budget_ms=a.cfg.optional_budget_ms;
    m.scenario_snapshot(req->cells,req->map.width,req->map.height,req->map.resolution,req->map.x0,req->map.y0,car,scan.stamp);
    {std::lock_guard<std::mutex> lock(a.mutex);a.pending=std::move(req);}
    a.cv.notify_one();a.last_request=now;
  }
  auto ready=std::atomic_load(&a.published);
  int near=nearest_path(a.active,car);
  bool active_ok=now-a.active_stamp<=a.cfg.plan_age&&path_valid(a.active,near,m,car);
  if(ready&&ready->epoch==a.epoch&&now-ready->timestamp<=a.cfg.plan_age){
    out.planner_ms=ready->compute_ms;out.hypotheses=int(ready->paths.size());
    bool fresh=ready->generation!=a.used_generation||ready->paths.size()!=a.used_count;
    if(fresh||!active_ok){
      // Rank by observed evidence, not simulator positions; try a bounded number.
      std::vector<std::pair<float,int>> rank;rank.reserve(ready->paths.size());
      for(int i=0;i<int(ready->paths.size());++i){
        const auto& p=ready->paths[i];float penalty=0;
        for(int j=0;j<p.hypothesis.count;++j){
          auto h=p.hypothesis.boxes[j];float best=INFINITY;
          for(auto xy:observed)best=fminf(best,hypotf(xy[0]-h.x,xy[1]-h.y));
          penalty+=best<.30f?-.25f:(m.scenario_cell(h.x,h.y,scan.stamp)==Free?.8f:.15f);
        }
        rank.emplace_back(p.cost+penalty,i);
      }
      std::sort(rank.begin(),rank.end());
      int tried=0;for(auto r:rank){
        if(tried++>=8)break;const auto& p=ready->paths[r.second];int n=nearest_path(p,car);
        if(!path_valid(p,n,m,car))continue;
        // Avoid swapping to a different tangent/offset mid-command.
        if(active_ok&&fabsf(wrap(p.p[n].yaw-a.active.p[near].yaw))>.20f)continue;
        a.active=p;a.active_stamp=ready->timestamp;a.active_generation=ready->generation;
        active_ok=true;near=n;break;
      }
      a.used_generation=ready->generation;a.used_count=ready->paths.size();
    }
  }
  if(!active_ok)return out;
  const auto& p=a.active;const auto& q=p.p[near];
  int aim=near;float look=clamp(.45f+.16f*car.v,.45f,1.0f);
  while(aim+1<points&&p.p[aim].s-q.s<look)++aim;
  const auto& target=p.p[aim];
  float ey=-sinf(q.yaw)*(car.x-q.x)+cosf(q.yaw)*(car.y-q.y),heading=wrap(car.yaw-q.yaw);
  // Curvature preview plus stabilizing feedback. Gains are experimental, not tuned.
  float steer=atanf(.3302f*target.kappa)-(.85f/(1+.20f*car.v))*ey-.70f*heading;
  float visibility=visible_distance(p,near,m,scan.stamp);
  float profile=std::min(p.p[near].speed,p.p[std::min(points-1,near+2)].speed);
  float safe=std::min({cap,profile,braking_cap(visibility,a.cfg.brake),
                       sqrtf(a.cfg.lateral_accel*.3302f/fmaxf(.01f,fabsf(tanf(car.steer))))});
  safe=fminf(safe,fmaxf(.3f,cap-3.f*fabsf(ey)-.8f*fabsf(heading)));
  out.base={clamp(steer,-.4f,.4f),fmaxf(0.f,safe)};
  a.last_features=features(scan,m,p,near,out.base,observed);a.last_near=near;
  out.correction=a.net.infer(a.last_features);
  out.command={clamp(out.base.steer+out.correction.steer,-.4f,.4f),
    clamp(out.base.speed+out.correction.speed,0.f,safe)};
  out.command.speed=fminf(out.command.speed,sqrtf(a.cfg.lateral_accel*.3302f/fmaxf(.01f,fabsf(tanf(out.command.steer)))));
  out.valid_plan=true;out.chosen=p.candidate;out.visible_m=visibility;out.age_ms=float((now-a.active_stamp)*1000);
  out.generation=a.active_generation;out.reason="scenario_track";return out;
}
void Driver::record(const Memory& memory,const Scan& scan,const Action& action,const std::string& run){
  auto& a=*impl_;if(!a.log.is_open()||!action.valid_plan||a.last_near<0||a.log_tick++%8)return;
  const auto& car=scan.pose;const auto& p=a.active;const int n=a.last_near;
  Grid grid=memory.local(car,.08f);nlohmann::json ref=nlohmann::json::array();
  float c=cosf(car.yaw),s=sinf(car.yaw);
  for(int i=0;i<ref_n;++i){
    const auto& q=p.p[std::min(points-1,n+i)];float dx=q.x-car.x,dy=q.y-car.y;
    ref.push_back({c*dx+s*dy,-s*dx+c*dy,wrap(q.yaw-car.yaw),q.speed,q.s-p.p[n].s});
  }
  a.log<<nlohmann::json{{"run",run},{"features",a.last_features},{"base",{action.base.steer,action.base.speed}},
    {"state",{car.v,car.steer,car.yaw_rate}},{"grid",grid.cost},{"geometry",{grid.resolution,grid.x0,grid.y0}},
    {"ref",ref},{"stamp",scan.stamp}}.dump()<<'\n';
}
}
