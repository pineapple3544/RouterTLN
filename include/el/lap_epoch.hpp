#pragma once
#include <cmath>

namespace el {
// Same directed finish segment and signed arc accounting as race_manager.
// Inputs are map geometry and the scan localizer's estimate.
struct LapEpoch {
  double x=0, y=0, yaw=0, left=0, right=0, length=0;
  bool initialized=false, armed=false;
  double last_x=0, last_y=0, last_s=0, progress=0;

  bool update(double px,double py,double station){
    bool credited=false;
    if(initialized){
      const double dx=px-last_x, dy=py-last_y;
      if(std::hypot(dx,dy)>1.5){
        progress=0;
      }else{
        const double c=std::cos(yaw), s=std::sin(yaw);
        const double advance=c*dx+s*dy;
        const double before=c*(last_x-x)+s*(last_y-y);
        if(advance>0 && before<0 && before+advance>=0){
          const double fraction=-before/advance;
          const double lateral=-s*(last_x+fraction*dx-x)+c*(last_y+fraction*dy-y);
          if(lateral>=-right && lateral<left){
            credited=armed && progress>=length/2;
            armed=true;
            progress=0;
          }
        }
        progress+=std::remainder(station-last_s,length);
      }
    }
    initialized=true;
    last_x=px;last_y=py;last_s=station;
    return credited;
  }
  void reset(){initialized=armed=false;progress=0;}
};
}
