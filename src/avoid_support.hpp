#pragma once
#include "sim_support.hpp"
// Lateral lattice using only observed occupancy; SVG still optimizes each action.
void local_route(TeacherProblem& p){
  bool blocked=false;for(int i=0;i<90;++i){auto r=p.ref[i];DynamicState s;s.z[0]=r.x;s.z[1]=r.y;s.z[4]=r.yaw;if(teacher_footprint(p,s,.08f)>.5f){blocked=true;break;}}
  if(!blocked)return;
  constexpr int max_count=80,width=25;int count=2;
  while(count<max_count){auto r=p.ref[count*2];if(r.x<p.grid.x0+.5||r.x>p.grid.x0+grid_n*p.grid.resolution-.5||fabsf(r.y)>6.5)break;++count;}
  float cost[width][width],next[width][width],penalty[max_count][width];int parent[max_count][width][width];
  float current_offset=sinf(p.ref[0].yaw)*p.ref[0].x-cosf(p.ref[0].yaw)*p.ref[0].y;
  float current_slope=clamp(tanf(wrap(-p.ref[0].yaw)),-.5,.5);
  for(int i=0;i<count;++i)for(int j=0;j<width;++j){auto r=p.ref[i*2];float d=(j-width/2)*.1f;
    DynamicState pose;pose.z[0]=r.x-sinf(r.yaw)*d;pose.z[1]=r.y+cosf(r.yaw)*d;pose.z[4]=r.yaw;
    penalty[i][j]=teacher_footprint(p,pose,.08f)>.5?1e6f:.08f*d*d;}
  for(int j=0;j<width;++j)for(int k=0;k<width;++k){float d=(j-width/2)*.1f,slope=(k-j)*.5f;
    cost[j][k]=abs(j-k)<=1?penalty[0][j]+penalty[1][k]+100*(d-current_offset)*(d-current_offset)+4*(slope-current_slope)*(slope-current_slope):1e20f;}
  for(int i=2;i<count;++i){for(auto& row:next)for(auto& v:row)v=1e20f;
    for(int k=0;k<width;++k)for(int j=std::max(0,k-1);j<=std::min(width-1,k+1);++j)
      for(int h=std::max(0,k-1);h<=std::min(width-1,k+1);++h){float curve=j-2*k+h;
        float v=cost[h][k]+penalty[i][j]+.12f*(j-k)*(j-k)+2.f*curve*curve;
        if(v<next[k][j]){next[k][j]=v;parent[i][k][j]=h;}}
    for(int j=0;j<width;++j)for(int k=0;k<width;++k)cost[j][k]=next[j][k];}
  int a=0,b=0;for(int j=0;j<width;++j)for(int k=0;k<width;++k)if(cost[j][k]<cost[a][b]){a=j;b=k;}
  float offsets[max_count];for(int i=count-1;i>=2;--i){offsets[i]=(b-width/2)*.1f;int h=parent[i][a][b];b=a;a=h;}
  offsets[0]=(a-width/2)*.1f;offsets[1]=(b-width/2)*.1f;
  int end=count*2-1;
  for(int i=0;i<end;++i){float d=i%2?(offsets[i/2]+offsets[i/2+1])*.5f:offsets[i/2];auto& r=p.ref[i];r.x-=sinf(r.yaw)*d;r.y+=cosf(r.yaw)*d;}
  for(int i=0;i<end;++i){int a=std::max(0,i-2),b=std::min(end-1,i+2);p.ref[i].yaw=atan2f(p.ref[b].y-p.ref[a].y,p.ref[b].x-p.ref[a].x);}
  for(int i=2;i<end-2;++i){float k=fabsf(wrap(p.ref[i+2].yaw-p.ref[i-2].yaw))/fmaxf(.05f,p.ref[i+2].s-p.ref[i-2].s);p.ref[i].speed=std::min(p.ref[i].speed,sqrtf(3.5f/fmaxf(k,.001f)));}
  for(int i=end-2;i>=0;--i)p.ref[i].speed=std::min(p.ref[i].speed,sqrtf(p.ref[i+1].speed*p.ref[i+1].speed+8*(p.ref[i+1].s-p.ref[i].s)));
}
// Independent oriented-box overlap check for dataset QA, never supplied to the teacher.
float separation(const DynamicState& s,const json& o){
  float cx=o[0].get<float>()-s.z[0],cy=o[1].get<float>()-s.z[1],a=s.z[4],b=o[2],best=-1e9;
  for(float axis:{a,a+pi*.5f,b,b+pi*.5f}){float center=fabsf(cx*cosf(axis)+cy*sinf(axis));
    float ra=.29f*fabsf(cosf(a-axis))+.1016f*fabsf(sinf(a-axis));
    float rb=o[3].get<float>()*.5f*fabsf(cosf(b-axis))+o[4].get<float>()*.5f*fabsf(sinf(b-axis));best=std::max(best,center-ra-rb);}
  return best;
}
