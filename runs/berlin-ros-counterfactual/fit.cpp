// Recover actuator history from timestamped commands and 100 Hz measured poses.
#include "el/dynamics.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>
using namespace el;
using json=nlohmann::json;
struct Fit {DynamicState state{}; double loss=0; int command=-1,parent=-1;};
int main(int argc,char**argv){
  json data;std::ifstream(argv[1])>>data;
  auto& frames=data["physics"];auto& controls=data["controls"];
  std::vector<double> speed(frames.size(),-1);
  for(auto& c:controls)speed.at(c["odom"].get<int>())=c["v"];
  Fit start;start.state.z[0]=frames[0][1];start.state.z[1]=frames[0][2];start.state.z[4]=frames[0][3];
  std::vector<std::vector<Fit>> layers{{start}};
  for(size_t i=1;i<frames.size();++i){
    std::vector<Fit> candidates;
    auto& prev=layers.back();
    for(size_t j=0;j<prev.size();++j){
      int newest=prev[j].command;
      while(newest+1<int(controls.size()) && controls[newest+1]["scan"].get<int>()<int(i))++newest;
      for(int k=std::max(prev[j].command,newest-2);k<=newest;++k){
        if(k<newest && int(i)-controls[k+1]["scan"].get<int>()>5)continue;
        Control action{};if(k>=0)action={controls[k]["steer"],controls[k]["target"]};
        auto v=physics_step(prev[j].state,action,Vehicle{});
        double ex=v.z[0]-frames[i][1].get<double>(),ey=v.z[1]-frames[i][2].get<double>();
        double eh=wrap(v.z[4]-frames[i][3].get<double>()),ev=speed[i]>=0?v.z[3]-speed[i]:0;
        double cost=prev[j].loss+ex*ex+ey*ey+eh*eh*.2+ev*ev*.01;
        v.z[0]=frames[i][1];v.z[1]=frames[i][2];v.z[4]=frames[i][3];
        candidates.push_back({v,cost,k,int(j)});
      }
    }
    std::sort(candidates.begin(),candidates.end(),[](const Fit&a,const Fit&b){return a.loss<b.loss;});
    std::vector<Fit> next;
    for(auto& a:candidates){
      bool duplicate=false;for(auto& b:next)if(a.command==b.command && a.state.delay[0]==b.state.delay[0] && a.state.delay[1]==b.state.delay[1] &&
        fabs(a.state.z[2]-b.state.z[2])<1e-5 && fabs(a.state.z[3]-b.state.z[3])<1e-5 && fabs(a.state.z[5]-b.state.z[5])<1e-5 && fabs(a.state.z[6]-b.state.z[6])<1e-5){duplicate=true;break;}
      if(!duplicate)next.push_back(a);if(next.size()==24)break;
    }
    if(next.empty())return 2;layers.push_back(std::move(next));
  }
  std::vector<json> output(frames.size());int j=0;
  for(int i=frames.size()-1;i>=0;--i){
    auto& a=layers[i][j];output[i]={{"z",a.state.z},{"delay",a.state.delay},{"command",a.command},{"loss",a.loss}};
    j=a.parent;
  }
  std::ofstream(argv[2])<<json(output);
}
