#include <nlohmann/json.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
using json=nlohmann::json;
json read(const std::string& path){std::ifstream f(path);if(!f)throw std::runtime_error("cannot open "+path);json j;f>>j;return j;}
std::vector<double> laps(const std::string& path){std::ifstream f(path);if(!f)throw std::runtime_error("missing CSV "+path);std::string line;getline(f,line);std::vector<double> result;double pt=0,pp=0,last=0;int lap=1;bool valid=true;
  while(getline(f,line)){std::stringstream s(line);std::string v;std::vector<double> r;while(getline(s,v,','))r.push_back(std::stod(v));if(r.size()<12)throw std::runtime_error("bad CSV row");
    double t=r[1],p=std::round(r[10]/41.9037232642963*420);valid=valid&&r[8]!=0&&r[9]==0;
    if(p>=lap*420&&p>pp){double crossing=pt+(t-pt)*(lap*420-pp)/(p-pp);if(valid)result.push_back(crossing-last);last=crossing;++lap;valid=true;}pt=t;pp=p;
  }return result;}
json stats(std::vector<double> v){if(v.empty())return nullptr;std::sort(v.begin(),v.end());double total=0;for(double x:v)total+=x;
  return {{"count",v.size()},{"mean",total/v.size()},{"median",v.size()%2?v[v.size()/2]:(v[v.size()/2-1]+v[v.size()/2])*.5},{"min",v.front()},{"max",v.back()}};}
int main(int argc,char** argv){try{if(argc!=4)throw std::runtime_error("Usage: el_compare_teachers BASELINE IMPROVED OUTPUT.json");auto a=read(std::string(argv[1])+"/summary.json"),b=read(std::string(argv[2])+"/summary.json");if(a.size()!=b.size())throw std::runtime_error("case count mismatch");json report;report["cases"]=json::array();
  for(std::string mode:{"race","avoid"}){std::vector<double> oldlaps,newlaps,oldtime,newtime;int total=0,oldsuccess=0,newsuccess=0,oldcollision=0,newcollision=0;
    for(size_t i=0;i<a.size();++i){auto x=a[i],y=b[i];if(x["scenario"]!=y["scenario"]||x["obstacles"]!=y["obstacles"])throw std::runtime_error("comparison scenarios differ");if(x["scenario"]["mode"]!=mode)continue;++total;
      std::string name=x["name"];auto l1=laps(std::string(argv[1])+"/"+name+".csv"),l2=laps(std::string(argv[2])+"/"+name+".csv");oldlaps.insert(oldlaps.end(),l1.begin(),l1.end());newlaps.insert(newlaps.end(),l2.begin(),l2.end());
      oldsuccess+=x["reason"]=="completed";newsuccess+=y["reason"]=="completed";oldcollision+=x["collisions"].get<int>();newcollision+=y["collisions"].get<int>();
      oldtime.push_back(x["mean_teacher_ms"]);newtime.push_back(y["mean_teacher_ms"]);
      report["cases"].push_back({{"name",name},{"baseline_reason",x["reason"]},{"improved_reason",y["reason"]},{"baseline_laps_s",l1},{"improved_laps_s",l2},{"baseline_progress_m",x["progress_m"]},{"improved_progress_m",y["progress_m"]},{"improved_plan_sources",y["plan_sources_weighted_warm_nominal_guide_sample"]}});
    }
    report[mode]={{"cases",total},{"baseline_completed",oldsuccess},{"improved_completed",newsuccess},{"baseline_collisions",oldcollision},{"improved_collisions",newcollision},{"baseline_lap_seconds",stats(oldlaps)},{"improved_lap_seconds",stats(newlaps)},{"baseline_case_mean_compute_ms",stats(oldtime)},{"improved_case_mean_compute_ms",stats(newtime)}};
  }
  report["conditions"]="Identical fixed obstacle coordinates, dimensions, seeds, starts, speed caps and simulation timestep. Incomplete laps excluded. Control compute time excludes simulator/feature preparation.";
  std::ofstream output(argv[3]);output<<report.dump(2);std::cout<<report.dump(2)<<std::endl;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
