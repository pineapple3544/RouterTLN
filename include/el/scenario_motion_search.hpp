#pragma once
#include "el/scenario_plan.hpp"
#include <algorithm>
#include <array>
#include <vector>

namespace el::scenario {
// Reconnect a failed full-horizon solve by searching steering sequences
// directly. Keep different positions and headings at the same progress so an
// initially slower side of a chicane survives until its exit becomes visible.
inline Path search_motion(const Frame& f, MapView map){
  constexpr int branches=7, beam=80, segments=(points-1)/3;
  struct Node {
    float x,y,yaw,steer,lateral,cost;
    int station,parent,action,key;
  };
  std::array<std::vector<Node>,segments+1> levels;
  const float ds=f.spacing;
  const float slew=3.f*ds/fmaxf(1.6f,f.car.v);
  auto move=[&](Node& n,int action){
    float target=(action-3)*(.39f/3);
    n.steer+=clamp(target-n.steer,-slew,slew);
    const float turn=ds*tanf(n.steer)/.3302f;
    n.x+=ds*cosf(n.yaw+turn*.5f);n.y+=ds*sinf(n.yaw+turn*.5f);n.yaw=wrap(n.yaw+turn);
  };
  levels[0].push_back({f.car.x,f.car.y,f.car.yaw,f.car.steer,f.offset,0,0,-1,3,0});
  std::vector<Node> choices;choices.reserve(beam*branches);
  for(int level=1;level<=segments;++level){
    choices.clear();
    for(size_t i=0;i<levels[level-1].size();++i){
      const Node& parent=levels[level-1][i];
      for(int action=0;action<branches;++action){
        Node n=parent;n.parent=i;n.action=action;
        bool clear=true;float risk=0;
        for(int sub=0;sub<3;++sub){
          move(n,action);
          PathPoint p{};p.x=n.x;p.y=n.y;p.yaw=n.yaw;
          if(collision(map,{},p,.065f)){clear=false;break;}
          if(collision(map,{},p,.17f))risk+=.07f;
          if(collision(map,{},p,.23f))risk+=.02f;
        }
        if(!clear)continue;
        float best=1.e10f;
        for(int j=std::max(0,parent.station-2);j<=std::min(points-1,parent.station+10);++j){
          const float dx=n.x-f.ref[j].x,dy=n.y-f.ref[j].y,d=dx*dx+dy*dy;
          if(d<best){best=d;n.station=j;}
        }
        const auto& r=f.ref[n.station];float heading=wrap(n.yaw-r.yaw);
        if(fabsf(heading)>1.5f||best>5.f)continue;
        n.lateral=-(n.x-r.x)*sinf(r.yaw)+(n.y-r.y)*cosf(r.yaw);
        float error=n.lateral-(f.have_guide?f.guide[n.station]:0.f);
        n.cost+=risk+.043f*error*error+.045f*heading*heading+.05f*n.steer*n.steer+
                .14f*(n.steer-parent.steer)*(n.steer-parent.steer)-.13f*(n.station-parent.station);
        const int lateral=int(std::lround(n.lateral/.14f))+32;
        const int angle=int(std::lround(heading/.16f))+16;
        n.key=((n.station/3)*64+lateral)*32+angle;
        choices.push_back(n);
      }
    }
    if(choices.empty())return {};
    std::sort(choices.begin(),choices.end(),[](const Node& a,const Node& b){return a.cost<b.cost;});
    auto& kept=levels[level];kept.reserve(beam);
    for(const auto& n:choices){
      if(std::any_of(kept.begin(),kept.end(),[&](const Node& k){return k.key==n.key;}))continue;
      kept.push_back(n);if(kept.size()==beam)break;
    }
  }
  int index=-1;float best=invalid_cost;
  for(size_t i=0;i<levels.back().size();++i){
    const auto& n=levels.back()[i];if(n.station<points/2)continue;
    float cost=n.cost+.15f*fabsf(n.lateral)+.35f*fabsf(wrap(n.yaw-f.ref[n.station].yaw));
    if(cost<best){best=cost;index=i;}
  }
  if(index<0)return {};
  std::array<int,segments> actions;
  for(int level=segments;level>0;--level){
    const auto& n=levels[level][index];actions[level-1]=n.action;index=n.parent;
  }
  Path path;Node n=levels[0].front();path.candidate=candidates;
  float time=0;
  for(int i=0;i<points;++i){
    if(i)move(n,actions[(i-1)/3]);
    auto& p=path.p[i];p.x=n.x;p.y=n.y;p.yaw=n.yaw;p.s=i*ds;p.kappa=tanf(n.steer)/.3302f;
    p.speed=fminf(f.cap,sqrtf(f.lateral_accel/fmaxf(.01f,fabsf(p.kappa))));
  }
  path.p[points-1].speed=0;
  for(int i=points-2;i>=0;--i){
    auto& p=path.p[i];float v=path.p[i+1].speed;
    p.speed=fminf(p.speed,sqrtf(v*v+2*f.brake*ds));time+=2*ds/fmaxf(.1f,p.speed+v);
  }
  path.cost=time;return path;
}
}
