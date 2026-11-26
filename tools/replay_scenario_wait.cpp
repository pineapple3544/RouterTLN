#include "el/scenario_plan.hpp"
#include "el/scenario_guide.hpp"
#include "el/scenario_motion_search.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <map>
using namespace el::scenario;
int main(int argc,char** argv){
  if(argc!=2){std::cerr<<"usage: replay_scenario_wait no_path.json\n";return 2;}
  nlohmann::json data;std::ifstream(argv[1])>>data;
  auto cells=data.at("cells").get<std::vector<uint8_t>>();
  MapView map{cells.data(),data.at("width"),data.at("height"),data.at("resolution"),
              data.at("origin").at(0),data.at("origin").at(1)};
  if(cells.size()!=size_t(map.width)*map.height)return 2;
  for(auto name:{"full","short"}){
    const auto& j=data.at(name);const auto& c=j.at("car");
    Frame f;f.car={c.at(0),c.at(1),c.at(2),c.at(3),c.at(4),c.at(5)};
    f.offset=j.at("offset");f.slope=j.at("slope");f.spacing=j.at("spacing");
    f.cap=j.at("cap");f.brake=j.at("brake");f.accel=j.at("accel");f.lateral_accel=j.at("lateral_accel");
    for(int i=0;i<points;++i){
      const auto& r=j.at("rows").at(i);f.ref[i]={r.at(0),r.at(1),r.at(2),r.at(3),r.at(4),r.at(5),r.at(6)};
    }
    build_observed_guide(f,map);
    const auto motion=search_motion(f,map);
    std::cout<<name<<" steering_sequence="<<(motion.candidate>=0)<<"\n";
    for(float radius:{1.e6f,.58f})for(bool swept:{false,true}){
      f.lattice_radius=radius;f.recovery_sweep=swept;
      std::map<std::string,std::map<int,int>> counts;
      for(int k=0;k<candidates;++k){
        Path path;Rejection rejection;
        evaluate(f,map,{},k,path,&rejection);
        ++counts[rejection.reason][rejection.row];
      }
      std::cout<<name<<" radius="<<radius<<" swept="<<swept<<" "<<nlohmann::json(counts).dump()<<"\n";
    }
  }
}
