#pragma once
#include "el/scenario_plan.hpp"
#include <algorithm>
#include <array>
#include <limits>
#include <stdexcept>
namespace el::scenario {
// Static atlas is indexed by cyclic station, independent of the actual run seed.
// All poses and obstacle coordinates are derived from the map and reference line.
struct VisibilityRow {Row row{};float station=0;};
struct VisibilityEntry {
  Obstacle obstacle{};float station=0,lateral=0;
  std::array<float,3> lead{}; // center, right, left; 0 = not confirmed in lookback
  float gap=0;
};
struct VisibilityRank {
  Hypothesis hypothesis{};
  float ahead=0,lead=0,slack=0,eta=0,priority=0;
  bool nominal=false;
};
inline float forward_distance(float from,float to,float length){
  if(!(length>0))return 0;
  float d=fmodf(to-from,length);return d<0?d+length:d;
}
inline float cyclic_station(float s,float length){return forward_distance(0,s,length);}
inline VisibilityRow interpolate_row(const std::vector<VisibilityRow>& rows,float length,float station){
  station=cyclic_station(station,length);
  auto next=std::upper_bound(rows.begin(),rows.end(),station,[](float s,const VisibilityRow& p){return s<p.station;});
  size_t b=next==rows.end()?0:size_t(next-rows.begin()),a=(b+rows.size()-1)%rows.size();
  const auto& p=rows[a];const auto& q=rows[b];
  float fraction=clamp(forward_distance(p.station,station,length)/fmaxf(.001f,forward_distance(p.station,q.station,length)),0,1);
  auto mix=[&](float x,float y){return x+(y-x)*fraction;};
  return {{mix(p.row.x,q.row.x),mix(p.row.y,q.row.y),wrap(p.row.yaw+fraction*wrap(q.row.yaw-p.row.yaw)),
    mix(p.row.lo,q.row.lo),mix(p.row.hi,q.row.hi),mix(p.row.wall_lo,q.row.wall_lo),mix(p.row.wall_hi,q.row.wall_hi)},station};
}
inline float lane_offset(const Row& r,int lane){
  const float center=clamp(0,r.lo,r.hi);
  if(lane==1)return .5f*(center+r.lo);
  if(lane==2)return .5f*(center+r.hi);
  return center;
}
inline bool possible_center(MapView map,const Obstacle& o){
  // Check the inscribed circle against cell centers. Keep uncertain wall-edge
  // orientations rather than claiming to know the original spawner's rules.
  float r=.5f*o.size;
  if(map.cell(o.x,o.y)==Occupied)return false;
  for(float x=-r;x<=r+.001f;x+=map.resolution*.5f)for(float y=-r;y<=r+.001f;y+=map.resolution*.5f)
    if(x*x+y*y<=r*r&&map.cell(o.x+x,o.y+y)==Occupied)return false;
  return true;
}
inline float first_visible_lead(const std::vector<VisibilityRow>& rows,float length,MapView map,
                                 const Obstacle& o,float station,int lane,const SensorGeometry& sensor,
                                 float lookback=8.f,float sample=.25f){
  int consecutive=0;
  for(float lead=fminf(lookback,length*.45f);lead>=.25f;lead-=sample){
    const auto r=interpolate_row(rows,length,station-lead).row;
    if(r.lo>r.hi){consecutive=0;continue;}
    float d=lane_offset(r,lane),x=r.x-d*sinf(r.yaw),y=r.y+d*cosf(r.yaw);
    if(predicted_visible(map,x,y,r.yaw,o,sensor)){
      // Use the later of two consecutive spatial samples as a conservative
      // detection proxy. This is not two actual sensor frames.
      if(++consecutive>=2)return lead;
    }else consecutive=0;
  }
  return 0;
}
struct VisibilityAtlas {
  float length=0,nominal_size=.15f;
  std::vector<VisibilityEntry> entries;
  void build(const std::vector<VisibilityRow>& rows,float loop_length,MapView map,float nominal,float lateral_step,
             const SensorGeometry& sensor,bool assume_passable){
    if(rows.size()<3||!(loop_length>0)||!(lateral_step>0)||!map.data||!(map.resolution>0))
      throw std::invalid_argument("invalid visibility atlas geometry");
    length=loop_length;nominal_size=nominal;entries.clear();
    const int stations=std::max(3,int(ceilf(length/.375f)));
    const std::array<float,4> sizes{nominal,.20f,.30f,.10f};
    for(int k=0;k<stations;++k){
      const float s=length*k/stations;const auto r=interpolate_row(rows,length,s).row;
      for(size_t z=0;z<sizes.size();++z){
        const float size=sizes[z],rad=.5f*size;
        bool duplicate=false;for(size_t a=0;a<z;++a)if(fabsf(sizes[a]-size)<.001f)duplicate=true;
        if(duplicate)continue;
        const float lo=r.wall_lo+rad,hi=r.wall_hi-rad;if(lo>hi)continue;
        int n=std::min(9,std::max(1,int(ceilf((hi-lo)/lateral_step))+1));
        if(hi-lo<.5f&&hi-lo>.05f)n=std::max(n,5);
        for(int i=0;i<n;++i){
          float d=n==1?.5f*(lo+hi):lo+(hi-lo)*i/(n-1);
          float gap=fmaxf(d-rad-r.wall_lo,r.wall_hi-d-rad);
          if(assume_passable&&gap<.2032f)continue;
          Obstacle o{r.x-d*sinf(r.yaw),r.y+d*cosf(r.yaw),size};
          if(!possible_center(map,o))continue;
          VisibilityEntry e;e.obstacle=o;e.station=s;e.lateral=d;e.gap=gap;
          for(int lane=0;lane<3;++lane)e.lead[lane]=first_visible_lead(rows,length,map,o,s,lane,sensor);
          entries.push_back(e);
        }
      }
    }
  }
  std::vector<VisibilityRank> rank(const Frame& f,MapView map)const{
    std::vector<VisibilityRank> out;
    if(!(length>0))return out;
    const float speed=fmaxf(.5f,fmaxf(f.car.v,.8f*f.cap));
    for(const auto& e:entries){
      const float ahead=forward_distance(f.station,e.station,length);
      if(ahead<.75f||ahead>(points-5)*station_step||ahead>length*.45f)continue;
      if(map.cell(e.obstacle.x,e.obstacle.y)!=Unknown)continue;
      if(!possible_center(map,e.obstacle))continue;
      const int lane=f.offset<-.12f?1:f.offset>.12f?2:0;
      // A route offset can hide a box which is visible from the racing line.
      float lead=fminf(e.lead[0],e.lead[lane]);
      lead=fminf(lead,ahead);
      VisibilityRank r;r.hypothesis.count=1;r.hypothesis.boxes[0]=e.obstacle;
      r.ahead=ahead;r.lead=lead;r.nominal=fabsf(e.obstacle.size-nominal_size)<.001f;
      const float stop=response_distance(speed,f.brake,e.obstacle.size);
      // Estimate a lateral shift time as a ranking heuristic, not a certified
      // last feasible avoidance point. Narrow openings receive extra urgency.
      const float shift=fmaxf(0.f,.1016f+.707107f*e.obstacle.size+.08f-fabsf(e.lateral));
      const float turn=.45f+.707107f*e.obstacle.size+speed*(.20f+2*sqrtf(shift/fmaxf(.1f,f.lateral_accel)));
      r.slack=(lead-fmaxf(stop,turn))/speed;
      r.eta=fmaxf(0.f,ahead-lead)/speed;
      r.priority=r.slack+.35f*r.eta+(r.nominal?0.f:.24f)+.10f*fminf(1.f,e.gap);
      out.push_back(r);
    }
    std::stable_sort(out.begin(),out.end(),[](const auto& a,const auto& b){return a.priority<b.priority;});
    if(out.size()>96)out.resize(96);
    return out;
  }
};
// Refine static predictions with the immutable observed occupancy snapshot.
// Unknown remains transparent only in this forecast. Rank more urgently if
// another actual obstacle hides the hypothetical target.
inline void refine_visibility(const Frame& f,MapView map,VisibilityRank& r){
  const auto o=r.hypothesis.boxes[0];
  int consecutive=0;float lead=0;
  int end=std::min(points-1,int(r.ahead/station_step));
  for(int i=0;i<end;i+=2){
    const auto& q=f.ref[i];if(q.lo>q.hi){consecutive=0;continue;}
    float d=clamp(f.offset,q.lo,q.hi);
    if(predicted_visible(map,q.x-d*sinf(q.yaw),q.y+d*cosf(q.yaw),q.yaw,o,f.sensor)){
      if(++consecutive>=2){lead=fmaxf(0.f,r.ahead-i*station_step);break;}
    }else consecutive=0;
  }
  float speed=fmaxf(.5f,fmaxf(f.car.v,.8f*f.cap));
  if(lead<r.lead){
    float loss=(r.lead-lead)/speed;
    r.slack-=loss;r.eta+=loss;r.priority-=.65f*loss;r.lead=lead;
  }
}
inline void set_visibility_watches(Frame& f,const std::vector<VisibilityRank>& ranked,float weight){
  f.watch_count=0;if(!(weight>0))return;
  for(const auto& r:ranked){
    if(!r.nominal||r.slack>=0||r.eta>1.8f)continue;
    bool duplicate=false;
    for(int i=0;i<f.watch_count;++i)
      if(hypotf(f.watch[i].obstacle.x-r.hypothesis.boxes[0].x,f.watch[i].obstacle.y-r.hypothesis.boxes[0].y)<.65f)duplicate=true;
    if(duplicate)continue;
    f.watch[f.watch_count++]={r.hypothesis.boxes[0],r.ahead,weight};
    if(f.watch_count==visibility_watches)break;
  }
}
}
