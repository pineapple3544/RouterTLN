#include "el/mppi.hpp"
#include "el/scenario_driver.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>
int main(int argc,char** argv){
  using namespace el;
  if(argc!=3){std::cerr<<"usage: el_scenario_teacher records.jsonl labels.jsonl\n";return 2;}
  std::ifstream in(argv[1]);std::ofstream out(argv[2]);if(!in||!out)return 2;
  MppiConfig cfg;cfg.samples=2048;cfg.horizon=40;cfg.dt=.05f;
  Mppi teacher(cfg);std::string line;size_t accepted=0,rejected=0;
  try{
    while(std::getline(in,line)){
      auto j=nlohmann::json::parse(line);Problem p{};p.max_speed=3.5f;
      p.state.v=j.at("state")[0].get<float>();p.state.steer=j.at("state")[1].get<float>();p.state.yaw_rate=j.at("state")[2].get<float>();
      p.grid.resolution=j.at("geometry")[0].get<float>();p.grid.x0=j.at("geometry")[1].get<float>();p.grid.y0=j.at("geometry")[2].get<float>();
      p.grid.cost=j.at("grid").get<decltype(p.grid.cost)>();
      if(j.at("ref").size()!=ref_n||j.at("features").size()!=scenario::feature_count)throw std::runtime_error("bad record shape");
      for(int i=0;i<ref_n;++i){auto r=j.at("ref")[i].get<std::array<float,5>>();p.ref[i]={r[0],r[1],r[2],r[3],r[4]};}
      auto b=j.at("base").get<std::array<float,2>>();Control base{b[0],b[1]};
      teacher.reset();auto plan=teacher.solve(p,base);
      if(!plan.valid||!safe_stop(p,plan.action)){++rejected;continue;}
      float ds=plan.action.steer-base.steer,dv=plan.action.speed-base.speed;
      // Large disagreements imply a different manoeuvre, not a residual label.
      if(!std::isfinite(ds)||!std::isfinite(dv)||fabsf(ds)>.16f||fabsf(dv)>1.2f){++rejected;continue;}
      out<<nlohmann::json{{"run",j.at("run")},{"features",j.at("features")},
        {"target",{clamp(ds/.08f,-1,1),clamp(dv/.5f,-1,1)}}}.dump()<<'\n';++accepted;
    }
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
  std::cout<<"accepted "<<accepted<<" rejected "<<rejected<<'\n';return accepted?0:1;
}
