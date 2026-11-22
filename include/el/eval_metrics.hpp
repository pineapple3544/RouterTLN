#pragma once
#include "el/core.hpp"
#include <algorithm>
#include <stdexcept>
namespace el {
struct RouteProjection {double s=0,distance=1e20;};
inline double route_length(const Track& track){auto a=track.points.front(),b=track.points.back();return b.s+hypot(a.x-b.x,a.y-b.y);}
inline RouteProjection project_route(const Track& track,const State& state){
  RouteProjection best;
  for(size_t i=0;i<track.points.size();++i){auto a=track.points[i],b=track.points[(i+1)%track.points.size()];
    double dx=b.x-a.x,dy=b.y-a.y,length=hypot(dx,dy),u=std::clamp(((state.x-a.x)*dx+(state.y-a.y)*dy)/std::max(length*length,1e-12),0.,1.);
    double distance=hypot(state.x-a.x-u*dx,state.y-a.y-u*dy);if(distance<best.distance)best={a.s+u*length,distance};}
  return best;
}
inline double route_delta(double current,double previous,double length){double d=current-previous;if(d>length/2)d-=length;if(d< -length/2)d+=length;return d;}
inline double crossing_time(double target,double before,double after,double t0,double t1){
  if(!(after>before&&target>=before&&target<=after))throw std::runtime_error("invalid lap crossing");return t0+(target-before)/(after-before)*(t1-t0);
}
}
