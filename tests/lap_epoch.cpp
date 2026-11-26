#include "el/lap_epoch.hpp"
#include <cstdlib>
int main(){
  el::LapEpoch lap{0,0,0,1,1,10};
  auto step=[&](double x,double y,double s,bool expected=false){
    if(lap.update(x,y,s)!=expected)std::abort();
  };
  step(-.1,0,9.9);step(.1,0,.1); // first crossing only arms
  step(-.1,0,9.9);step(.1,0,.1); // seam jitter earns no lap
  for(int s=1;s<=9;++s)step(.2,0,s);
  step(-.1,0,9.9);step(.1,0,.1,true);
  step(-.1,1.2,9.9);step(.1,1.2,.1); // crossed extension outside track
  for(int s=1;s<=8;++s)step(.2,1.2,s);
  step(-3,0,8);step(-1.5,0,9);step(-.1,0,9.9);step(.1,0,.1);
  lap.reset();step(-.1,0,9.9);step(.1,0,.1);
  // The official segment intersection credits arrival at the line only once.
  lap.reset();step(-.1,0,9.9);step(0,0,0);
  if(!lap.armed)std::abort();
  step(.1,0,.1);
  lap.reset();step(-.1,1,9.9);step(.1,1,.1);
  if(lap.armed)std::abort();
}
