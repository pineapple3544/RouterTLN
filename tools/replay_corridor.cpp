#include "el/avoid_corridor.hpp"
#include "el/scenarios.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

// Replay an observed occupancy map and entry pose without a simulator or TLN.
int main(int argc,char** argv){
  if(argc!=4){std::cerr<<"usage: el_replay_corridor MAP.yaml WAYPOINTS.json FRAME.json\n";return 2;}
  try {
    el::Memory memory(argv[1]);
    el::Track track(argv[2]);el::orient_track(track,false);
    std::ifstream input(argv[3]);
    auto frame=nlohmann::json::parse(input);
    auto xyz=frame.at("pose").get<std::array<float,3>>();
    el::State car{xyz[0],xyz[1],xyz[2],frame.at("speed").get<float>()};
    el::AvoidCorridor planner;el::CorridorTrace trace;
    bool feasible=planner.plan(track,frame.at("waypoint"),car,memory,nullptr,&trace,true);
    std::cout<<nlohmann::json{{"feasible",feasible},{"subdivisions",trace.subdivisions},{"first_empty",trace.first_empty},
      {"free",trace.free},{"reachable",trace.reachable}}.dump()<<'\n';
    return feasible?0:1;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
