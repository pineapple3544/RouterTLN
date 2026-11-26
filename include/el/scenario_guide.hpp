#pragma once
#include "el/scenario_plan.hpp"
#include <algorithm>
#include <array>

namespace el::scenario {
// Search for an open lane through the complete observed horizon. These
// offsets are targets for the CUDA rollout and never bypass its guards.
inline bool build_observed_guide(Frame& f,MapView map){
  constexpr int half=22,cols=2*half+1,directions=5,rows=(points+1)/2;
  constexpr float pitch=.10f;
  using Costs=std::array<std::array<float,directions>,cols>;
  using Parents=std::array<std::array<int,directions>,cols>;
  std::array<Parents,rows> parent{};
  Costs prev{},next{};
  for(auto& v:prev)v.fill(invalid_cost);
  int start=std::clamp(int(std::lround(f.offset/pitch))+half,0,cols-1);
  int heading=std::clamp(int(std::lround(f.slope*2*f.spacing/pitch)),-2,2);
  prev[start][heading+2]=0;
  for(int row=1;row<rows;++row){
    const auto& ref=f.ref[2*row];
    for(auto& v:next)v.fill(invalid_cost);
    for(int col=0;col<cols;++col){
      float d=(col-half)*pitch;
      if(d<ref.lo||d>ref.hi)continue;
      PathPoint pose;pose.x=ref.x-sinf(ref.yaw)*d;pose.y=ref.y+cosf(ref.yaw)*d;
      for(int dir=-2;dir<=2;++dir){
        int from=col-dir;if(from<0||from>=cols)continue;
        float best=invalid_cost;int best_dir=-1;
        for(int old=-2;old<=2;++old){
          float cost=prev[from][old+2]+.13f*(dir-old)*(dir-old);
          if(cost<best){best=cost;best_dir=old+2;}
        }
        if(best_dir<0)continue;
        pose.yaw=ref.yaw+atan2f(dir*pitch,2*f.spacing);
        if(collision(map,{},pose,.075f))continue;
        float cost=.03f*d*d+.055f*dir*dir;
        if(collision(map,{},pose,.22f))cost+=.20f;
        if(collision(map,{},pose,.14f))cost+=.65f;
        next[col][dir+2]=best+cost;parent[row][col][dir+2]=best_dir;
      }
    }
    prev=next;
  }
  float best=invalid_cost;int col=-1,dir=0;
  for(int j=0;j<cols;++j)for(int k=0;k<directions;++k){
    float d=(j-half)*pitch,cost=prev[j][k]+.28f*d*d+.12f*(k-2)*(k-2);
    if(cost<best){best=cost;col=j;dir=k;}
  }
  if(col<0)return false;
  for(int row=rows-1;row>0;--row){
    f.guide[2*row]=(col-half)*pitch;
    int old=parent[row][col][dir];col-=dir-2;dir=old;
  }
  f.guide[0]=f.offset;
  for(int i=1;i<points;i+=2)f.guide[i]=.5f*(f.guide[i-1]+f.guide[i+1]);
  f.have_guide=true;return true;
}
}
