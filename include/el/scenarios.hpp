#pragma once
#include "el/core.hpp"
#include <string>
#include <stdexcept>
#include <algorithm>
namespace el {
inline void orient_track(Track& track,bool reverse){
  auto a=track.points.front(),b=track.points.back();if(hypotf(a.x-b.x,a.y-b.y)<.05f)track.points.pop_back();
  if(reverse){std::reverse(track.points.begin(),track.points.end());for(auto& w:track.points)w.yaw=wrap(w.yaw+pi);}
  float s=0;for(size_t i=0;i<track.points.size();++i){if(i)s+=hypotf(track.points[i].x-track.points[i-1].x,track.points[i].y-track.points[i-1].y);track.points[i].s=s;}
}
struct ObstacleShape {float min_length,max_length,min_width,max_width,max_rotation;};
inline ObstacleShape obstacle_shape(const std::string& profile){
  if(profile=="compact")return {.16f,.24f,.16f,.24f,.7f};
  if(profile=="standard")return {.22f,.32f,.22f,.32f,.7f};
  if(profile=="wide")return {.20f,.30f,.30f,.38f,.35f};
  if(profile=="long")return {.34f,.50f,.16f,.24f,.8f};
  if(profile=="rotated")return {.20f,.36f,.20f,.36f,1.4f};
  if(profile=="legacy")return {.16f,.34f,.16f,.34f,.7f};
  throw std::runtime_error("unknown obstacle profile");
}
inline float route_curvature(const Track& track,int index){int n=track.points.size();auto a=track.points[(index+n-5)%n],b=track.points[(index+5)%n];float length=0;
  for(int j=-5;j<5;++j){auto p=track.points[(index+n+j)%n],q=track.points[(index+n+j+1)%n];length+=hypotf(p.x-q.x,p.y-q.y);}
  return wrap(b.yaw-a.yaw)/fmaxf(length,.05f);
}
inline std::string road_region(float curvature){float k=fabsf(curvature);return k<.1f?"straight":k<.25f?"transition":"corner";}
}
