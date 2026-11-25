#include "el/core.hpp"
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <png.h>
#include <fstream>
#include <filesystem>
#include <stdexcept>
#include <algorithm>
namespace el {
Track::Track(const std::string& path){
  std::ifstream f(path);if(!f)throw std::runtime_error("cannot open waypoints: "+path);
  nlohmann::json j;f>>j;
  for(auto& w:j.at("global_traj_wpnts_iqp").at("wpnts"))
    points.push_back({w.at("x_m"),w.at("y_m"),w.at("psi_rad"),w.at("vx_mps"),w.at("s_m")});
  if(points.size()<20)throw std::runtime_error("too few waypoints");
}
int Track::closest(const State& s)const{
  int best=0;float d=1e20f;
  for(size_t i=0;i<points.size();++i){float dx=s.x-points[i].x,dy=s.y-points[i].y;
    if(dx*dx+dy*dy<d){best=i;d=dx*dx+dy*dy;}}
  return best;
}
Problem Track::problem(const State& s,const Grid& g,float vmax)const{
  Problem p;p.state=s;p.state.x=p.state.y=p.state.yaw=0;p.grid=g;p.max_speed=vmax;
  int start=closest(s);float distance=0,c=cosf(s.yaw),q=sinf(s.yaw);
  for(int i=0;i<ref_n;++i){auto w=points[(start+i*2)%points.size()];
    if(i){auto prev=points[(start+(i-1)*2)%points.size()];distance+=hypotf(w.x-prev.x,w.y-prev.y);}
    float x=w.x-s.x,y=w.y-s.y;
    p.ref[i]={c*x+q*y,-q*x+c*y,wrap(w.yaw-s.yaw),fminf(vmax,w.speed),distance};
  }return p;
}
Control Track::pursuit(const State& s,float vmax)const{
  int n=closest(s),target=n;float d=0,look=clamp(.7f+s.v*.22f,.7f,1.8f);
  while(d<look){int next=(target+1)%points.size();d+=hypotf(points[next].x-points[target].x,points[next].y-points[target].y);target=next;if(target==n)break;}
  float dx=points[target].x-s.x,dy=points[target].y-s.y;
  float lateral=-sinf(s.yaw)*dx+cosf(s.yaw)*dy;
  float steer=atan2f(2*.3302f*lateral,dx*dx+dy*dy);
  float speed=fminf(vmax,points[n].speed);
  speed=fminf(speed,sqrtf(3.0f*.3302f/fmaxf(.02f,fabsf(tanf(steer)))));
  return {clamp(steer,-.4f,.4f),speed};
}
Memory::Memory(const std::string& path){
  auto y=YAML::LoadFile(path);resolution_=y["resolution"].as<float>();
  x0_=y["origin"][0].as<float>();y0_=y["origin"][1].as<float>();
  if(fabs(y["origin"][2].as<float>())>1e-5)throw std::runtime_error("rotated map origin is unsupported");
  auto file=std::filesystem::path(path).parent_path()/y["image"].as<std::string>();
  png_image img{};img.version=PNG_IMAGE_VERSION;
  if(!png_image_begin_read_from_file(&img,file.c_str()))throw std::runtime_error("map PNG open failed");
  img.format=PNG_FORMAT_GRAY;std::vector<unsigned char> bytes(PNG_IMAGE_SIZE(img));
  if(!png_image_finish_read(&img,nullptr,bytes.data(),0,nullptr))throw std::runtime_error("map PNG decode failed");
  width_=img.width;height_=img.height;png_image_free(&img);
  if(resolution_<=0||width_<=0||height_<=0)throw std::runtime_error("invalid map geometry");
  base_.resize(width_*height_);odds_.assign(base_.size(),0);feature_odds_.assign(base_.size(),0);seen_.assign(base_.size(),-1);
  current_hit_mask_.assign(base_.size(),0);
  bool negate=y["negate"].as<int>()!=0;float free=y["free_thresh"].as<float>();
  for(int row=0;row<height_;++row)for(int col=0;col<width_;++col){
    float gray=bytes[(height_-1-row)*width_+col]/255.f;
    float occ=negate?gray:1-gray;
    base_[row*width_+col]=occ<free?0.f:1.f; // Unknown base-map cells remain blocked.
  }
}
float Memory::occupancy(int i)const{
  if(i<0)return 1.f;
  if(base_[i]>.5f)return 1.f;
  return odds_[i]>.5f?1.f:0.f;
}
float Memory::base_hit_range(float sx,float sy,float dx,float dy,float limit)const{
  // Exact cell-entry distances avoid associating with a nearby wall off this ray.
  int x=(int)floorf((sx-x0_)/resolution_),y=(int)floorf((sy-y0_)/resolution_);
  int step_x=dx>0?1:dx<0?-1:0,step_y=dy>0?1:dy<0?-1:0;
  float next_x=step_x?((x0_+(x+(step_x>0))*resolution_)-sx)/dx:INFINITY;
  float next_y=step_y?((y0_+(y+(step_y>0))*resolution_)-sy)/dy:INFINITY;
  float delta_x=step_x?resolution_/fabsf(dx):INFINITY,delta_y=step_y?resolution_/fabsf(dy):INFINITY,t=0;
  for(int n=0;n<width_+height_+2&&t<=limit;++n){
    if(x<0||y<0||x>=width_||y>=height_)return INFINITY;
    if(base_[y*width_+x]>.5f)return t;
    if(next_x<next_y){t=next_x;next_x+=delta_x;x+=step_x;}
    else if(next_y<next_x){t=next_y;next_y+=delta_y;y+=step_y;}
    else{t=next_x;next_x+=delta_x;next_y+=delta_y;x+=step_x;y+=step_y;}
  }
  return INFINITY;
}
std::vector<int> Memory::obstacle_cells()const{
  std::vector<int> cells;
  for(size_t i=0;i<odds_.size();++i)
    if(base_[i]<.5f && odds_[i]>.5f)cells.push_back(static_cast<int>(i));
  for(int i:current_hits_)if(base_[i]<.5f)cells.push_back(i);
  std::sort(cells.begin(),cells.end());
  cells.erase(std::unique(cells.begin(),cells.end()),cells.end());
  return cells;
}
float Memory::obstacle_at(float x,float y)const{
  int k=index(x,y);
  return k>=0 && base_[k]<.5f && (odds_[k]>.5f || current_hit_mask_[k]);
}
// Preserve the sparse hit list for snapshots and update only touched bits.
void Memory::replace_current_hits(std::vector<int> hits){
  for(int k:current_hits_)current_hit_mask_[k]=0;
  current_hits_=std::move(hits);
  for(int k:current_hits_)current_hit_mask_[k]=1;
}
void Memory::update(const Scan& scan){
  std::vector<int> hits,feature_hits,free_cells;
  float sx=scan.pose.x+.27f*cosf(scan.pose.yaw),sy=scan.pose.y+.27f*sinf(scan.pose.yaw);
  for(int b=0;b<beams;++b){
    if(scan.valid[b]<.5f)continue;
    float r=clamp(scan.ranges[b],0,scan.range_max),a=scan.pose.yaw+(scan.explicit_angles?scan.angles[b]:scan.angle_min+(scan.angle_max-scan.angle_min)*b/(beams-1));
    float c=cosf(a),q=sinf(a);bool hit=r<scan.range_max-.02f;
    for(float d=0;d<r-resolution_;d+=resolution_*.7f){int k=index(sx+c*d,sy+q*d);if(k>=0)free_cells.push_back(k);}
    if(hit){int k=index(sx+c*r,sy+q*r);if(k>=0){
      feature_hits.push_back(k);
      // The simulator's range sigma is 1 cm. Associate returns within 3 sigma
      // of the first mapped wall; do not grow a second wall into the free cell.
      // No free-space evidence is added at the ambiguous endpoint: previously
      // observed obstacles remain until a ray actually observes through them.
      constexpr float wall_range_tolerance=.03f;
      float expected=base_[k]<.5f?base_hit_range(sx,sy,c,q,r+wall_range_tolerance):INFINITY;
      if(std::isfinite(expected)&&fabsf(r-expected)<=wall_range_tolerance){++associated_wall_returns_;
        int mapped=index(sx+c*(expected+.001f),sy+q*(expected+.001f));if(mapped>=0&&base_[mapped]>.5f)hits.push_back(mapped);
      }
      else hits.push_back(k);
    }}
  }
  auto unique=[](std::vector<int>& a){std::sort(a.begin(),a.end());a.erase(std::unique(a.begin(),a.end()),a.end());};
  unique(hits);replace_current_hits(std::move(hits));unique(feature_hits);unique(free_cells);
  for(int k:free_cells)if(!current_hit_mask_[k])odds_[k]=fmaxf(-3.f,odds_[k]-.7f);
  for(int k:current_hits_)odds_[k]=fminf(4.f,odds_[k]+1.f);
  for(int k:free_cells)if(!std::binary_search(feature_hits.begin(),feature_hits.end(),k)){feature_odds_[k]=fmaxf(-3.f,feature_odds_[k]-.7f);seen_[k]=scan.stamp;}
  for(int k:feature_hits){feature_odds_[k]=fminf(4.f,feature_odds_[k]+1.f);seen_[k]=scan.stamp;}
}
Grid Memory::local(const State& s,float resolution)const{
  Grid g;g.resolution=resolution;g.x0=-grid_n*resolution*.25f;g.y0=-grid_n*resolution*.5f;float c=cosf(s.yaw),q=sinf(s.yaw);
  for(int y=0;y<grid_n;++y)for(int x=0;x<grid_n;++x){
    float lx=g.x0+(x+.5f)*g.resolution,ly=g.y0+(y+.5f)*g.resolution,v=0;
    // Max pooling prevents thin obstacles disappearing when resampling a finer map.
    for(float a=-g.resolution*.5f;a<=g.resolution*.5f+.001f;a+=resolution_*.8f)
      for(float b=-g.resolution*.5f;b<=g.resolution*.5f+.001f;b+=resolution_*.8f)
        v=fmaxf(v,occupancy(index(s.x+c*(lx+a)-q*(ly+b),s.y+q*(lx+a)+c*(ly+b))));
    g.cost[y*grid_n+x]=v;
  }return g;
}
void Memory::features(const State& s,double now,Observation& o)const{
  float c=cosf(s.yaw),q=sinf(s.yaw);
  for(int y=0;y<crop;++y)for(int x=0;x<crop;++x){
    float lx=-2.f+(x+.5f)*.35f,ly=-5.6f+(y+.5f)*.35f;
    int k=index(s.x+c*lx-q*ly,s.y+q*lx+c*ly),i=y*crop+x;
    o.map[i]=k<0||base_[k]>.5f||feature_odds_[k]>.5f?1.f:0.f;
    o.map[crop*crop+i]=k>=0&&seen_[k]>=0?1.f:0.f;
    o.map[2*crop*crop+i]=k>=0&&seen_[k]>=0?clamp((now-seen_[k])/60.,0,1):1.f;
  }
}
void Memory::reset(){replace_current_hits({});std::fill(odds_.begin(),odds_.end(),0);std::fill(feature_odds_.begin(),feature_odds_.end(),0);std::fill(seen_.begin(),seen_.end(),-1);associated_wall_returns_=0;}
size_t Memory::occupied()const{size_t n=0;for(size_t i=0;i<odds_.size();++i)if(base_[i]<.5f&&odds_[i]>.5f)++n;return n;}
void Memory::copy_occupancy(std::vector<uint8_t>& cells,int& width,int& height,float& resolution,float& x0,float& y0)const{
  width=width_;height=height_;resolution=resolution_;x0=x0_;y0=y0_;cells.resize(base_.size());
  for(size_t i=0;i<cells.size();++i)cells[i]=occupancy(i)>.5f;
}
bool Memory::base_free_disc(float x,float y,float radius)const{
  for(float a=-radius;a<=radius+resolution_*.2f;a+=resolution_*.5f)for(float b=-radius;b<=radius+resolution_*.2f;b+=resolution_*.5f){
    if(a*a+b*b>radius*radius)continue;int k=index(x+a,y+b);if(k<0||base_[k]>.5f)return false;}return true;
}
bool Memory::wall_hides_disc(float sx,float sy,float x,float y,float radius)const{
  float d=hypotf(x-sx,y-sy);if(d<=radius+.5f)return false;float center=atan2f(y-sy,x-sx),span=asinf(radius/d);
  int rays=std::max(2,int(ceilf(2*span*d/(resolution_*.4f))));
  for(int j=0;j<=rays;++j){float angle=center-span+2*span*j/rays;bool blocked=false;
    for(float r=.3f;r<d-radius-.25f;r+=resolution_*.4f){int k=index(sx+cosf(angle)*r,sy+sinf(angle)*r);if(k>=0&&base_[k]>.5f){blocked=true;break;}}
    if(!blocked)return false;}return true;
}
Observation observe(const std::deque<Scan>& scans,const State& s,const Memory& mem,const Track& track,int mode,int side){
  Observation o;
  // Reproject historical endpoints to current sensor frame; unobserved bins retain validity=0.
  for(int h=0;h<history;++h){
    auto offset=h*2*beams;std::fill_n(o.scans.begin()+offset,beams,1.f);
    if(h>=(int)scans.size())continue;
    const auto& old=scans[scans.size()-1-h];
    for(int b=0;b<beams;++b){if(old.valid[b]<.5f)continue;
      float a=old.pose.yaw+(old.explicit_angles?old.angles[b]:old.angle_min+(old.angle_max-old.angle_min)*b/(beams-1)),r=old.ranges[b];
      float wx=old.pose.x+.27f*cosf(old.pose.yaw)+r*cosf(a),wy=old.pose.y+.27f*sinf(old.pose.yaw)+r*sinf(a);
      float dx=wx-s.x-.27f*cosf(s.yaw),dy=wy-s.y-.27f*sinf(s.yaw);
      float angle=wrap(atan2f(dy,dx)-s.yaw);
      int bin=(int)lround((angle+2.35f)/4.7f*(beams-1));if(bin<0||bin>=beams)continue;
      float range=scans.empty()?10:scans.back().range_max;
      o.scans[offset+bin]=fminf(o.scans[offset+bin],clamp(hypotf(dx,dy)/range,0,1));o.scans[offset+beams+bin]=1;
    }
  }
  mem.features(s,scans.empty()?0:scans.back().stamp,o);
  o.context[0]=s.v/8;o.context[1]=s.yaw_rate/5;o.context[2]=s.steer/.4f;
  o.context[3]=mode==1?1.f:0.f;o.context[4]=side;
  int n=track.closest(s);float c=cosf(s.yaw),q=sinf(s.yaw);
  for(int j=0;j<5;++j){auto w=track.points[(n+(j+1)*10)%track.points.size()];float dx=w.x-s.x,dy=w.y-s.y;
    o.context[5+j*2]=(c*dx+q*dy)/10;o.context[6+j*2]=(-q*dx+c*dy)/10;}
  o.context[15]=std::min((int)scans.size(),history)/(float)history;
  return o;
}
bool safe_stop(const Problem& p,Control u,float first_dt){
  if(!std::isfinite(u.steer)||!std::isfinite(u.speed)||u.speed<0||u.speed>p.max_speed+.01||fabs(u.steer)>.401)return false;
  State s=p.state;
  for(int i=0;i<2;++i){s=advance(s,u,first_dt/2);if(footprint(p.grid,s)>.5f)return false;}
  int steps=(int)ceil(s.v/6/.01)+30;
  Control brake{s.steer,0};
  for(int i=0;i<steps;++i){s=advance(s,brake,.01f);if(footprint(p.grid,s)>.5f)return false;}
  return s.v<.05f;
}
bool valid_plan(const Problem& p,const std::vector<Control>& controls,float dt){
  State s=p.state;
  for(auto u:controls){if(!std::isfinite(u.steer)||!std::isfinite(u.speed)||fabs(u.steer)>.401||u.speed<0||u.speed>p.max_speed+.01)return false;
    for(int j=0;j<4;++j){s=advance(s,u,dt/4);if(footprint(p.grid,s)>.5f)return false;}}
  Problem end=p;end.state=s;return safe_stop(end,{s.steer,0},.01f);
}
bool Router::wants_avoid(bool hazard,double now){
  if(hazard){avoid_=true;clear_since_=-1;}
  else if(avoid_){if(clear_since_<0)clear_since_=now;if(now-clear_since_>.5){avoid_=false;clear_since_=-1;}}
  return avoid_;
}
Mode Router::update(bool hazard,bool safe,bool mppi,double now){
  bool avoid=wants_avoid(hazard,now);
  mode_=safe?(avoid?Mode::Avoid:Mode::Race):(mppi?Mode::Mppi:Mode::Emergency);return mode_;
}
bool Router::allow_candidate(bool safe,double now){
  if(!safe){safe_since_=-1;return false;}
  if(mode_!=Mode::Mppi&&mode_!=Mode::Emergency)return true;
  if(safe_since_<0)safe_since_=now;
  return now-safe_since_>=.3;
}
}
