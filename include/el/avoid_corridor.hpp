#pragma once
#include "el/core.hpp"
#include "el/dynamics.hpp"
#include <algorithm>
#include <array>
#include <limits>

namespace el {
// Gym casts from a LiDAR 0.27 m ahead of the reference point. Its forward
// TTC envelope adds half the wheelbase and five milliseconds of beam closure.
// The normal vehicle footprint still covers the rear half and both sides.
inline bool gym_front_blocked(const Memory& memory,const State& pose,float margin=0){
  const float c=cosf(pose.yaw),s=sinf(pose.yaw);
  const float front=.27f+.3302f*.5f+.005f*std::max(0.f,pose.v)+margin;
  const float half_width=.1016f+margin;
  for(int i=0;i<5;++i){
    const float x=.29f+(front-.29f)*i/4;
    for(int j=-3;j<=3;++j){
      const float y=half_width*j/3;
      if(memory.at(pose.x+c*x-s*y,pose.y+s*x+c*y)>.5f)return true;
    }
  }
  return false;
}

// Lateral lattice in the IQP track frame. Only observed/memorized occupancy is used.
// Each edge spans two waypoints; the change of slope is penalized at each vertex.
struct CorridorTrace {
  int subdivisions=1;
  std::array<float,32> planned_offsets{};
  float gate_offset=0,gate_entry=0,gate_exit=0;
  std::array<std::array<uint32_t,4>,32> fine_free{},fine_reachable{};
  bool wide=false, brake_event=false;
  float policy_steer=0,candidate_steer=0;
  State brake_contact{};

  int waypoint=-1, origin=-1, first_empty=-1;
  std::array<uint32_t,32> free{},reachable{};
};
class AvoidCorridor {
  friend struct OfflineSnapshot;
 public:
  static constexpr int count=32, width=25, middle=width/2;
  bool plan(const Track& track,int start,const State& car,const Memory& memory,const std::vector<float>* replay_line=nullptr, CorridorTrace* trace=nullptr,bool prefer_reserve=false) {
    // Prefer a full corridor with reserve for steering and occupancy uncertainty.
    // Ramp the extra margin ahead of the car so a narrow entry can open onto
    // a wide exit. The ordinary clearance search remains available at a pinch.
    const Passage passage=prefer_reserve?preview_passage(track,start,car,memory):Passage{};
    // Do not switch sides just because the other side is representable on the
    // coarse grid. Evaluate the narrow side on the same fine lattice first.
    if(passage.valid){
      if(trace){*trace=CorridorTrace{};trace->subdivisions=4;}
      if(plan_lattice<4>(track,start,car,memory,replay_line,trace,false,passage))return true;
    }
    if(prefer_reserve){
      if(trace){*trace=CorridorTrace{};trace->wide=true;}
      if(plan_lattice<1>(track,start,car,memory,replay_line,trace,true,passage))return true;
    }
    if(trace)*trace=CorridorTrace{};
    if(plan_lattice<1>(track,start,car,memory,replay_line,trace,false,passage))return true;
    // Keep the 10 cm lateral reach per edge. A 2.5 cm fallback can take
    // different sub-column positions on successive rows through a narrow gap.
    if(!passage.valid){
      if(trace){*trace=CorridorTrace{};trace->subdivisions=4;}
      if(plan_lattice<4>(track,start,car,memory,replay_line,trace,false,passage))return true;
    }
    if(prefer_reserve){
      // On a bend, progress measured along the reference can be much smaller
      // than distance driven. Try up to 30 cm laterally per 20 cm reference
      // step when every ordinary path disconnects. This is a geometric guide;
      // the delayed steering and braking checks still validate each command.
      if(trace){*trace=CorridorTrace{};trace->subdivisions=2;}
      if(plan_lattice<2,3>(track,start,car,memory,replay_line,trace,false,passage))return true;
      if(plan_curved_entry(track,start,car,memory,trace))return true;
    }
    valid_=false;return false;
  }
 private:
  // Across an obstacle, check both the opening and its exit at every waypoint.
  // A wide opening that closes a few rows later is a poor steering target.
  struct Passage {float offset=0,entry=0,exit=0;bool valid=false;};
  Passage preview_passage(const Track& track,int start,const State& car,const Memory& memory)const{
    struct ObstacleLayer {
      const Memory& map;
      float at(float x,float y)const{return map.obstacle_at(x,y);}
    } obstacles{memory};
    const auto& wp=track.points;
    int first=-1,last=-1;
    for(int i=0;i<42;++i){
      const auto& p=wp[(start+i)%wp.size()];
      if(footprint(obstacles,{p.x,p.y,p.yaw},.10f)>.5f){
        if(first<0)first=i;
        last=i;
      }else if(last>=0 && i-last>=4)break;
    }
    if(first<0)return {};
    constexpr float spacing=.05f;
    constexpr int cols=49;
    std::array<bool,cols> open;open.fill(true);
    for(int i=first;i<=last+4;++i){
      const auto& p=wp[(start+i)%wp.size()];
      const float c=cosf(p.yaw),s=sinf(p.yaw);
      for(int j=0;j<cols;++j){
        if(!open[j])continue;
        float d=(j-cols/2)*spacing;
        open[j]=footprint(memory,{p.x-s*d,p.y+c*d,p.yaw},.065f)<=.5f;
      }
    }
    Passage gate;
    const auto& entry=wp[start];
    const float lateral=-(car.x-entry.x)*sinf(entry.yaw)+(car.y-entry.y)*cosf(entry.yaw);
    const size_t index=(start+first)%wp.size();
    const float prior=valid_ && previous_.size()==wp.size() && previous_[index].stamp==generation_
      ?previous_[index].d:lateral;
    float best=-std::numeric_limits<float>::infinity();
    for(int j=0;j<cols;){
      if(!open[j]){++j;continue;}
      const int begin=j;
      while(j<cols && open[j])++j;
      const int width=j-begin;
      const float center=(.5f*(begin+j-1)-cols/2)*spacing;
      // Prefer a reachable approach. One newly observed cell at the far
      // edge should not reverse an already committed steering manoeuvre.
      const float score=width*spacing-.50f*fabsf(center-prior)-.20f*fabsf(center-lateral);
      if(score>best){
        gate.offset=center;best=score;
      }
    }
    gate.valid=std::isfinite(best);
    gate.entry=.10f*first;
    gate.exit=.10f*(last+4);
    return gate;
  }
  // Connect the measured entry pose to the previous downstream guide. The
  // lattice uses a reference-aligned body at each row. In a tight bend its
  // body corners can strike a wall that clears a car tangent to the curve.
  bool plan_curved_entry(const Track& track,int start,const State& car,const Memory& memory,CorridorTrace* trace){
    if(!valid_ || previous_.size()!=track.points.size())return false;
    constexpr int samples=2*count-1;
    const auto& wp=track.points;
    std::array<State,samples> ref{},curve{},best_curve{};
    std::array<float,samples> nx{},ny{},arc{},prior{},offset{},best_offset{};
    const auto& first=wp[start];
    const float lateral=-(car.x-first.x)*sinf(first.yaw)+(car.y-first.y)*cosf(first.yaw);
    const float slope=tanf(clamp(wrap(car.yaw-first.yaw),-1.15f,1.15f));
    arc[0]=-(car.x-first.x)*cosf(first.yaw)-(car.y-first.y)*sinf(first.yaw);
    for(int i=0;i<samples;++i){
      const size_t k=(start+i)%wp.size();const auto& p=wp[k];
      ref[i]={p.x,p.y,p.yaw};nx[i]=-sinf(p.yaw);ny[i]=cosf(p.yaw);
      if(i)arc[i]=arc[i-1]+hypotf(p.x-ref[i-1].x,p.y-ref[i-1].y);
      prior[i]=previous_[k].stamp==generation_?previous_[k].d:(i?prior[i-1]:lateral);
    }
    float best=std::numeric_limits<float>::infinity();
    for(float length:{.8f,1.3f,1.6f,2.f,2.5f}){
      for(int col=-12;col<=12;++col){
        const float goal=col*.10f;
        for(int i=0;i<samples;++i){
          const float t=clamp(arc[i]/length,0,1),blend=t*t*(3-2*t);
          const float approach=(1-blend)*lateral+blend*goal+length*slope*t*(1-t)*(1-t);
          const float u=clamp((arc[i]-std::max(1.8f,length))/1.1f,0,1),merge=u*u*(3-2*u);
          offset[i]=approach+(prior[i]-approach)*merge;
          curve[i]={ref[i].x+nx[i]*offset[i],ref[i].y+ny[i]*offset[i]};
        }
        curve[0]=car;
        float peak=0,effort=0;
        bool clear=true;
        for(int i=1;i<samples;++i){
          auto& a=curve[i-1];auto& b=curve[i];const auto& next=curve[std::min(i+1,samples-1)];
          const float ds=hypotf(b.x-a.x,b.y-a.y);
          b.yaw=atan2f(next.y-a.y,next.x-a.x);
          const float turn=wrap(b.yaw-a.yaw);
          const State mid{(a.x+b.x)*.5f,(a.y+b.y)*.5f,wrap(a.yaw+.5f*turn)};
          if(footprint(memory,mid,.065f)>.5f || footprint(memory,b,.065f)>.5f){clear=false;break;}
          if(arc[i]<3.f){
            const float curvature=fabsf(turn)/std::max(.03f,ds);
            peak=std::max(peak,curvature);effort+=curvature*curvature*ds;
          }
        }
        const float score=peak+.15f*effort+.08f*fabsf(goal-prior[15]);
        if(clear && score<best){best=score;best_curve=curve;best_offset=offset;}
      }
    }
    if(!std::isfinite(best))return false;
    ++generation_;
    for(int i=0;i<samples;++i){
      previous_[(start+i)%wp.size()]={generation_,best_offset[i]};
      if(i%2==0){path_[i/2]=best_curve[i];offsets_[i/2]=best_offset[i];}
    }
    if(trace){*trace=CorridorTrace{};trace->waypoint=start;trace->subdivisions=0;trace->planned_offsets=offsets_;}
    return true;
  }
  template<int subdivisions,int reach=1>
  bool plan_lattice(const Track& track,int start,const State& car,const Memory& memory,
                    const std::vector<float>* replay_line,CorridorTrace* trace,bool wide,const Passage& passage){
    constexpr int cols=(width-1)*subdivisions+1,center=cols/2,slope_extent=reach*subdivisions,slopes=2*slope_extent+1;
    constexpr float step_m=.10f/subdivisions;
    auto coarse=[](int col){return (col+subdivisions/2)/subdivisions;};
    const auto& wp=track.points;
    auto ref=[&](int i)->const Waypoint&{return wp[(start+2*i)%wp.size()];};
    const auto& w=ref(0);
    float lateral=-(car.x-w.x)*sinf(w.yaw)+(car.y-w.y)*cosf(w.yaw);
    const float entry_heading=wrap(car.yaw-w.yaw);
    float slope=clamp(tanf(entry_heading),-.8f,.8f);
    float cost[cols][slopes],next[cols][slopes];
    signed char prev[count][cols][slopes]{};
    constexpr float inf=std::numeric_limits<float>::infinity();
    for(auto& row:cost)std::fill_n(row,slopes,inf);
    int origin=std::clamp(int(lroundf(lateral/step_m))+center,0,cols-1);
    cost[origin][std::clamp(int(lroundf(slope*subdivisions/.5f))+slope_extent,0,slopes-1)]=0;
    if(trace){
      trace->waypoint=start; trace->origin=coarse(origin);
      trace->reachable[0]=1u<<coarse(origin);
      trace->fine_reachable[0][origin/32]|=1u<<(origin%32);
    }
    float ahead=0;
    for(int i=1;i<count;++i){
      auto& p=ref(i);
      ahead+=hypotf(p.x-ref(i-1).x,p.y-ref(i-1).y);
      // The near-field body is still turning onto the corridor. Starting
      // every footprint at the reference yaw can close the first lattice row.
      constexpr float alignment_distance=2*.3302f;
      const float body_yaw=wrap(p.yaw+entry_heading*std::max(0.f,1-ahead/alignment_distance));
      float preferred=0;
      bool recorded=false;
      if(replay_line){
        int index=(start+2*i)%wp.size();
        for(int delta:{0,-1,1,-2,2}){
          float value=(*replay_line)[(index+delta+int(wp.size()))%wp.size()];
          if(std::isfinite(value)){preferred=value;recorded=true;break;}
        }
      }
      const float reserve=wide?(.16f-.065f)*clamp((ahead-.65f)/.65f,0,1):0.f;
      // Validate both waypoint parities. A route through every other cross
      // section can disappear as soon as the car advances a single waypoint.
      const auto& mid=wp[(start+2*i-1)%wp.size()];
      const float mid_ahead=ahead-hypotf(p.x-mid.x,p.y-mid.y);
      const float mid_yaw=wrap(mid.yaw+entry_heading*std::max(0.f,1-mid_ahead/alignment_distance));
      std::array<bool,2*cols-1> mid_free{};
      for(int k=0;k<2*cols-1;++k){
        const float d=(k-2*center)*(.5f*step_m);
        State pose{mid.x-sinf(mid.yaw)*d,mid.y+cosf(mid.yaw)*d,mid_yaw};
        mid_free[k]=footprint(memory,pose,.065f)<=.5f;
      }
      std::array<bool,cols> free{};
      std::array<float,cols> room{};
      for(int j=0;j<cols;++j){
        const float offset=(j-center)*step_m;
        State pose{p.x-sinf(p.yaw)*offset,p.y+cosf(p.yaw)*offset,body_yaw};
        if(footprint(memory,pose,.065f)>.5f)continue;
        if(reserve>0 && footprint(memory,pose,.065f+reserve)>.5f)continue;
        free[j]=true;
        if(i==1 && std::abs(j-origin)<=slope_extent){
          for(float t:{.25f,.5f,.75f}){
            State sample{car.x+t*(pose.x-car.x),car.y+t*(pose.y-car.y),
                         wrap(car.yaw+t*wrap(body_yaw-car.yaw))};
            if(footprint(memory,sample,.065f)>.5f){free[j]=false;break;}
          }
        }
      }
      int run=0;
      for(int j=0;j<cols;++j){run=free[j]?run+1:0;room[j]=run*step_m;}
      run=0;
      for(int j=cols-1;j>=0;--j){
        run=free[j]?run+1:0;room[j]=std::min(room[j],run*step_m);
      }
      for(auto& row:next)std::fill_n(row,slopes,inf);
      for(int j=0;j<cols;++j){
        if(!free[j])continue;
        float offset=(j-center)*step_m;
        State pose{p.x-sinf(p.yaw)*offset,p.y+cosf(p.yaw)*offset,body_yaw};
        if(trace){trace->free[i]|=1u<<coarse(j);trace->fine_free[i][j/32]|=1u<<(j%32);}
        float penalty=.32f*offset*offset;
        const float shortfall=std::max(0.f,1-room[j]/.35f);
        penalty+=12.f*shortfall*shortfall;
        if(passage.valid){
          const float lead=std::max(.8f,.80f*car.v);
          const float approach=clamp(1-(passage.entry-ahead)/lead,0,1);
          const float exit=clamp(1-(ahead-passage.exit)/.7f,0,1);
          const float error=offset-passage.offset;
          penalty+=48.f*approach*exit*error*error;
        }
        if(recorded)penalty+=3.f*(offset-preferred)*(offset-preferred);
        if(footprint(memory,pose,.16f)>.5f)penalty+=1.6f;
        if(footprint(memory,pose,.23f)>.5f)penalty+=.32f;
        if(valid_){
          size_t idx=(start+i*2)%wp.size();
          if(previous_[idx].stamp==generation_)penalty+=3.f*(offset-previous_[idx].d)*(offset-previous_[idx].d);
        }
        for(int delta=-slope_extent;delta<=slope_extent;++delta){
          int from=j-delta;
          if(from<0||from>=cols || !mid_free[from+j])continue;
          for(int old=-slope_extent;old<=slope_extent;++old){
            float value=cost[from][old+slope_extent]+penalty+(.15f*delta*delta+1.6f*(delta-old)*(delta-old))/(subdivisions*subdivisions);
            if(value<next[j][delta+slope_extent]){
              next[j][delta+slope_extent]=value;
              prev[i][j][delta+slope_extent]=old;
            }
          }
        }
      }
      uint32_t reachable=0;
      for(int j=0;j<cols;++j){
        std::copy_n(next[j],slopes,cost[j]);
        if(std::isfinite(*std::min_element(next[j],next[j]+slopes))){
          reachable|=1u<<coarse(j);
          if(trace)trace->fine_reachable[i][j/32]|=1u<<(j%32);
        }
      }
      if(trace){
        trace->reachable[i]=reachable;
        if(!reachable && trace->first_empty<0)trace->first_empty=i;
      }
      if(!reachable && (!trace || wide))return false;
    }
    float best=inf;int col=0,step=0;
    for(int j=0;j<cols;++j)for(int k=-slope_extent;k<=slope_extent;++k){
      if(cost[j][k+slope_extent]<best){best=cost[j][k+slope_extent];col=j;step=k;}
    }
    if(!std::isfinite(best))return false;
    if(previous_.size()!=wp.size())previous_.assign(wp.size(),{});
    ++generation_;
    for(int i=count-1;i>=0;--i){
      float d=(col-center)*step_m;
      auto& p=ref(i);
      path_[i]={p.x-sinf(p.yaw)*d,p.y+cosf(p.yaw)*d,p.yaw};
      offsets_[i]=d;
      size_t idx=(start+i*2)%wp.size();
      previous_[idx]={generation_,d};
      if(i<count-1)previous_[(idx+1)%wp.size()]={generation_,.5f*(d+offsets_[i+1])};
      int old=prev[i][col][step+slope_extent];col-=step;step=old;
    }
    if(trace){trace->planned_offsets=offsets_;trace->gate_offset=passage.offset;trace->gate_entry=passage.entry;trace->gate_exit=passage.exit;}
    valid_=true;return true;
  }
 public:
  void follow_reference(const Track& track,int start){
    for(int i=0;i<count;++i){
      const auto& w=track.points[(start+2*i)%track.points.size()];
      path_[i]={w.x,w.y,w.yaw};offsets_[i]=0;
    }
    valid_=false;
  }
  float speed_limit(float requested,float speed)const{
    // Preview curvature over a metre to smooth lateral lattice quantization.
    float limit=requested;
    for(int i=2;i<=14;++i){
      const auto& a=path_[i-2];const auto& b=path_[i];const auto& c=path_[i+2];
      float ux=b.x-a.x,uy=b.y-a.y,vx=c.x-b.x,vy=c.y-b.y;
      float denom=hypotf(ux,uy)*hypotf(vx,vy)*hypotf(ux+vx,uy+vy);
      float curvature=2*std::abs(ux*vy-uy*vx)/std::max(denom,.001f);
      float advance_distance=std::max(0.f,hypotf(a.x-path_[0].x,a.y-path_[0].y)-.20f*speed);
      limit=std::min(limit,sqrtf(2.2f/std::max(curvature,.01f)+6.f*advance_distance));
    }
    return limit;
  }
  // Project candidate steering onto the observed corridor using the same delayed
  // vehicle dynamics as the command guard. Near a bend the scout line can be
  // traversable yet a faster vehicle can cut across its inner boundary.
  float preview_steer(const DynamicState& initial,Control policy,const Memory& memory,float dt,int ticks) const {
    if(!valid_)return policy.steer;
    // On a sharp inside entry the near point may lie on the outside of the
    // bend while the open passage already demands steering into the turn.
    // Use the bearing one metre along the guide as a lower bound on that
    // turning demand. Delayed rollouts and the command guard still decide.
    const float bend=wrap(path_[7].yaw-path_[0].yaw);
    const float direction=bend<0?-1.f:1.f;
    if(fabsf(bend)>.60f && direction*(offsets_[7]-offsets_[0])>.15f && direction*offsets_[7]>.15f){
      const State car=basic(initial);
      for(int i=1;i<12;++i){
        const float x=path_[i].x-car.x,y=path_[i].y-car.y;
        const float squared=x*x+y*y;
        if(squared<.90f)continue;
        const float needed=clamp(atan2f(2*.3302f*(-sinf(car.yaw)*x+cosf(car.yaw)*y),squared),-.4f,.4f);
        if(direction*needed>0 && direction*needed>direction*policy.steer)policy.steer=needed;
        break;
      }
    }
    // A driving arc can be clear and still have an obstructed stopping arc
    // if steering is frozen at the point where a stop would be requested.
    auto braking_cost=[&](State car,float margin){
      const Control brake{car.steer,0};
      for(int tick=1;car.v>=.045f && tick<=300;++tick){
        car=advance(car,brake,.01f);
        if(footprint(memory,car,margin)>.5f || gym_front_blocked(memory,car,margin))return 1.f;
      }
      return 0.f;
    };
    // If the guide turns sharply before the passage, a half-second preview
    // at scout speed can end before the front bumper reaches the obstacle.
    const State measured=basic(initial);
    const float turning=fabsf(wrap(path_[7].yaw-measured.yaw));
    const int horizon=turning>.50f && std::abs(offsets_[7])>.12f
      ?std::clamp(int(std::ceil(1.70f/(.01f*std::max(1.f,measured.v)))),50,140):50;
    auto score=[&](float steer){
      DynamicState state=initial;
      float loss=2.f*(steer-policy.steer)*(steer-policy.steer);
      State next=basic(initial);
      const Control candidate{steer,policy.speed};
      for(int i=0;i<ticks;++i)next=predict_command_tick(next,candidate,dt/ticks);
      loss+=1000.f*braking_cost(next,.04f);
      int segment=0;
      float continuation=steer;
      for(int tick=1;tick<=horizon;++tick){
        state=physics_step(state,{tick<=15?steer:continuation,policy.speed},Vehicle{});
        if(tick%5)continue;
        const State car=basic(state);
        if(!std::isfinite(car.x)||!std::isfinite(car.y)||!std::isfinite(car.yaw))
          return std::numeric_limits<float>::infinity();
        if(tick==10 || tick==20 || tick==30)
          loss+=4.f*braking_cost(car,.08f);
        // Find the nearest point on this local polyline, then look ahead along
        // its arc. This also works when the corridor doubles back in a hairpin.
        float best=std::numeric_limits<float>::infinity(),fraction=0;
        int nearest=segment;
        for(int j=segment;j<std::min(segment+5,count-1);++j){
          const auto& a=path_[j];const auto& b=path_[j+1];
          const float dx=b.x-a.x,dy=b.y-a.y;
          const float t=clamp(((car.x-a.x)*dx+(car.y-a.y)*dy)/std::max(.0001f,dx*dx+dy*dy),0,1);
          const float ex=car.x-a.x-t*dx,ey=car.y-a.y-t*dy;
          const float distance=ex*ex+ey*ey;
          if(distance<best){best=distance;nearest=j;fraction=t;}
        }
        segment=nearest;
        loss+=best;
        if(footprint(memory,car,.065f)>.5f || gym_front_blocked(memory,car,.05f))loss+=12.f;
        if(footprint(memory,car,.13f)>.5f)loss+=.35f;
        float look=.65f;
        for(int j=nearest;j<count-1;++j){
          const auto& a=path_[j];const auto& b=path_[j+1];
          const float dx=b.x-a.x,dy=b.y-a.y,length=std::max(.001f,hypotf(dx,dy));
          const float from=j==nearest?fraction:0.f;
          if((1-from)*length>=look || j==count-2){
            const float t=std::min(1.f,from+look/length);
            const float x=a.x+t*dx-car.x,y=a.y+t*dy-car.y;
            continuation=clamp(atan2f(2*.3302f*(-sinf(car.yaw)*x+cosf(car.yaw)*y),x*x+y*y),-.4f,.4f);
            break;
          }
          look-=(1-from)*length;
        }
      }
      return loss;
    };
    float selected=policy.steer,best=score(selected);
    for(int k=-4;k<=4;++k){
      const float steer=.1f*k;
      const float loss=score(steer);
      if(loss<best){selected=steer;best=loss;}
    }
    return selected;
  }
  int side_at(float ahead)const{
    int i=std::clamp(int(std::lround(ahead/.20f)),1,count-1);
    return offsets_[i]>.05f?1:offsets_[i]<-.05f?-1:0;
  }
  void condition(Observation& o,const State& car,float lookahead_scale=1.f,bool returning=false)const{
    float x=0,y=0,look=lookahead_scale*(.65f+.12f*std::max(0.f,car.v));
    const float extra=std::max(0.f,car.v-1.8f);
    if(extra>0){
      // At higher speed the short Euclidean chord can select a point beside
      // the car during a merge. Interpolate at a forward distance that also
      // gives the delayed, rate-limited steering time to unwind.
      look+=extra*(returning?.65f:.35f);
      const float c=cosf(car.yaw),s=sinf(car.yaw);
      float px=path_[0].x-car.x,py=path_[0].y-car.y;
      for(int j=1;j<count;++j){
        x=path_[j].x-car.x;y=path_[j].y-car.y;
        const float prev=c*px+s*py, ahead=c*x+s*y;
        if(ahead>=look && ahead>prev){
          const float t=clamp((look-prev)/(ahead-prev),0,1);
          x=px+t*(x-px);y=py+t*(y-py);break;
        }
        px=x;py=y;
      }
    }else for(int j=1;j<count;++j){
      x=path_[j].x-car.x;y=path_[j].y-car.y;
      if(hypotf(x,y)>=look)break;
    }
    float lat=-sinf(car.yaw)*x+cosf(car.yaw)*y;
    o.context[4]=clamp(atan2f(2*.3302f*lat,x*x+y*y)/.4f,-1,1);
    for(int i=0;i<5;++i){
      const State& p=path_[(i+1)*5];
      float x=p.x-car.x,y=p.y-car.y;
      o.context[5+2*i]=(cosf(car.yaw)*x+sinf(car.yaw)*y)/10;
      o.context[6+2*i]=(-sinf(car.yaw)*x+cosf(car.yaw)*y)/10;
    }
  }
 private:
  struct Offset {uint64_t stamp=0;float d=0;};
  std::vector<Offset> previous_;
  uint64_t generation_=0;
  bool valid_=false;
  std::array<State,count> path_{};
  std::array<float,count> offsets_{};
};
}
