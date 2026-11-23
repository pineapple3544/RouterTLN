#include "el/dataset.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
using namespace el;
int main(int argc,char** argv){try{
  if(argc!=3)throw std::runtime_error("Usage: el_inspect --dataset EPISODE.elds | --metrics CONTROL.csv");
  std::string mode=argv[1];nlohmann::json result;
  if(mode=="--dataset"){
    auto data=read_dataset(argv[2]);size_t valid=0,race=0,avoid=0,collisions=0;uint64_t last=0;double stamp=-1;
    for(size_t i=0;i<data.size();++i){auto& r=data[i];if(i&&(r.step<=last||r.stamp<=stamp))throw std::runtime_error("nonmonotonic episode");last=r.step;stamp=r.stamp;
      for(auto x:r.observation.scans)if(!std::isfinite(x))throw std::runtime_error("nonfinite scan feature");
      for(auto x:r.observation.map)if(!std::isfinite(x))throw std::runtime_error("nonfinite map feature");
      for(auto x:r.observation.context)if(!std::isfinite(x))throw std::runtime_error("nonfinite state feature");
      valid+=r.valid!=0;race+=r.mode==0;avoid+=r.mode==1;collisions+=r.collision!=0;
    }
    if(data.empty())throw std::runtime_error("empty episode");
    result={{"records",data.size()},{"valid_labels",valid},{"race",race},{"avoid",avoid},{"collision_frames",collisions},{"training_executed",false}};
  }else if(mode=="--metrics"){
    std::ifstream f(argv[2]);if(!f)throw std::runtime_error("metrics open failed");std::string line;getline(f,line);std::vector<double> timings;bool all_contacts_scored=true;int collision=0,invalid=0,mppi=0;double progress=0,max_speed=0;
    while(getline(f,line)){if(line.empty())continue;std::stringstream s(line);std::string field;std::vector<double> row;
      while(row.size()<14 && getline(s,field,',')){
        if(row.size()==11 && field.empty()){all_contacts_scored=false;row.push_back(0);}
        else row.push_back(std::stod(field));
      }
      if(row.size()!=14)throw std::runtime_error("incomplete metric row");
      timings.push_back(row[8]);invalid+=row[7]==0;collision+=row[11]!=0;mppi+=row[5]==2;progress=row[13];max_speed=std::max(max_speed,row[4]);
    }
    if(timings.empty())throw std::runtime_error("no control iterations recorded");std::sort(timings.begin(),timings.end());
    result={{"iterations",timings.size()},{"collision_observed",all_contacts_scored},{"collision_frames",all_contacts_scored?nlohmann::json(collision):nlohmann::json(nullptr)},{"invalid_commands",invalid},{"mppi_steps",mppi},{"progress_m",progress},{"max_speed",max_speed},
      {"p50_ms",timings[timings.size()/2]},{"p95_ms",timings[(timings.size()-1)*95/100]},{"p99_ms",timings[(timings.size()-1)*99/100]},{"max_ms",timings.back()}};
  }else throw std::runtime_error("unknown inspection mode");
  std::cout<<result.dump(2)<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
