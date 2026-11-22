#pragma once
#include "el/core.hpp"
#include <algorithm>
#include <array>
#include <limits>

namespace el {
// Lateral lattice in the IQP track frame. Only observed/memorized occupancy is used.
// Each edge spans two waypoints; the change of slope is penalized at each vertex.
class AvoidCorridor {
 public:
  static constexpr int count=32, width=25, middle=width/2;
  bool plan(const Track& track,int start,const State& car,const Memory& memory,const std::vector<float>* replay_line=nullptr) {
    const auto& wp=track.points;
    auto ref=[&](int i)->const Waypoint&{return wp[(start+2*i)%wp.size()];};
    const auto& w=ref(0);
    float lateral=-(car.x-w.x)*sinf(w.yaw)+(car.y-w.y)*cosf(w.yaw);
    float slope=clamp(tanf(wrap(car.yaw-w.yaw)),-.8f,.8f);
    float cost[width][3],next[width][3];
    signed char prev[count][width][3]{};
    constexpr float inf=std::numeric_limits<float>::infinity();
    for(auto& row:cost)std::fill_n(row,3,inf);
    int origin=std::clamp(int(lroundf(lateral/.10f))+middle,0,width-1);
    cost[origin][std::clamp(int(lroundf(slope/.5f))+1,0,2)]=0;
    for(int i=1;i<count;++i){
      auto& p=ref(i);
      float preferred=0;
      bool recorded=false;
      if(replay_line){
        int index=(start+2*i)%wp.size();
        for(int delta:{0,-1,1,-2,2}){
          float value=(*replay_line)[(index+delta+int(wp.size()))%wp.size()];
          if(std::isfinite(value)){preferred=value;recorded=true;break;}
        }
      }
      for(auto& row:next)std::fill_n(row,3,inf);
      for(int j=0;j<width;++j){
        float offset=(j-middle)*.10f;
        State pose{p.x-sinf(p.yaw)*offset,p.y+cosf(p.yaw)*offset,p.yaw};
        if(footprint(memory,pose,.065f)>.5f)continue;
        float penalty=.32f*offset*offset;
        if(recorded)penalty+=12.f*(offset-preferred)*(offset-preferred);
        if(footprint(memory,pose,.16f)>.5f)penalty+=1.6f;
        if(footprint(memory,pose,.23f)>.5f)penalty+=.32f;
        if(valid_){
          size_t idx=(start+i*2)%wp.size();
          if(previous_[idx].stamp==generation_)penalty+=.18f*(offset-previous_[idx].d)*(offset-previous_[idx].d);
        }
        for(int delta=-1;delta<=1;++delta){
          int from=j-delta;
          if(from<0||from>=width)continue;
          for(int old=-1;old<=1;++old){
            float value=cost[from][old+1]+penalty+.15f*delta*delta+1.6f*(delta-old)*(delta-old);
            if(value<next[j][delta+1]){
              next[j][delta+1]=value;
              prev[i][j][delta+1]=old;
            }
          }
        }
      }
      for(int j=0;j<width;++j)std::copy_n(next[j],3,cost[j]);
    }
    float best=inf;int col=0,step=0;
    for(int j=0;j<width;++j)for(int k=-1;k<=1;++k){
      if(cost[j][k+1]<best){best=cost[j][k+1];col=j;step=k;}
    }
    if(!std::isfinite(best)){valid_=false;return false;}
    if(previous_.size()!=wp.size())previous_.assign(wp.size(),{});
    ++generation_;
    for(int i=count-1;i>=0;--i){
      float d=(col-middle)*.1f;
      auto& p=ref(i);
      path_[i]={p.x-sinf(p.yaw)*d,p.y+cosf(p.yaw)*d,p.yaw};
      offsets_[i]=d;
      size_t idx=(start+i*2)%wp.size();
      previous_[idx]={generation_,d};
      if(i<count-1)previous_[(idx+1)%wp.size()]={generation_,.5f*(d+offsets_[i+1])};
      int old=prev[i][col][step+1];col-=step;step=old;
    }
    valid_=true;return true;
  }
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
