#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
// Offline diagnostics only. Lap estimates use controller progress, not race_manager.
int main(int argc,char** argv){try{
  if(argc!=3)throw std::runtime_error("usage: planner_metrics WAYPOINTS METRICS_CSV");
  nlohmann::json j;std::ifstream input(argv[1]);input>>j;
  auto w=j.at("global_traj_wpnts_iqp").at("wpnts");
  const auto& a=w.front();const auto& b=w.back();
  double length=b["s_m"].get<double>()+std::hypot(a["x_m"].get<double>()-b["x_m"].get<double>(),a["y_m"].get<double>()-b["y_m"].get<double>());
  std::ifstream csv(argv[2]);std::string line;std::getline(csv,line);
  double start=-1,prev_stamp=0,prev_progress=0,progress=0,stamp=0,sum2=0,max_error=0,max_steer=0;
  std::vector<double> errors,crossings,steady_errors;
  while(std::getline(csv,line)){
    std::stringstream row(line);std::string field;std::vector<double> v;
    while(std::getline(row,field,','))v.push_back(std::stod(field));
    if(v.size()!=14)throw std::runtime_error("invalid CSV row");
    stamp=v[1];progress=v[13];if(start<0)start=prev_stamp=stamp;
    double threshold=(crossings.size()+1)*length;
    if(progress>=threshold&&prev_progress<threshold){
      double ratio=(threshold-prev_progress)/(progress-prev_progress);
      crossings.push_back(prev_stamp+ratio*(stamp-prev_stamp)-start);
    }
    double distance=1e9;
    for(size_t i=0;i<w.size();++i){const auto& p=w[i];const auto& q=w[(i+1)%w.size()];
      double x=p["x_m"],y=p["y_m"],dx=q["x_m"].get<double>()-x,dy=q["y_m"].get<double>()-y;
      double t=std::clamp(((v[2]-x)*dx+(v[3]-y)*dy)/std::max(1e-12,dx*dx+dy*dy),0.,1.);
      distance=std::min(distance,std::hypot(v[2]-x-t*dx,v[3]-y-t*dy));
    }
    errors.push_back(distance);sum2+=distance*distance;max_error=std::max(max_error,distance);max_steer=std::max(max_steer,std::abs(v[9]));
    if(progress>=length)steady_errors.push_back(distance);
    prev_stamp=stamp;prev_progress=progress;
  }
  if(errors.empty())throw std::runtime_error("empty CSV");
  std::sort(errors.begin(),errors.end());std::vector<double> laps;double previous=0;
  for(double t:crossings){laps.push_back(t-previous);previous=t;}
  nlohmann::json result={{"progress_lap_length_m",length},{"elapsed_stamp_s",stamp-start},{"estimated_laps",progress/length},
    {"estimated_lap_times_s",laps},{"distance_to_reference_rms_m",std::sqrt(sum2/errors.size())},
    {"distance_to_reference_p95_m",errors[size_t(.95*(errors.size()-1))]},{"distance_to_reference_max_m",max_error},{"max_command_steer_rad",max_steer}};
  if(!steady_errors.empty()){
    std::sort(steady_errors.begin(),steady_errors.end());double ss=0;for(double d:steady_errors)ss+=d*d;
    result["after_first_lap_distance_rms_m"]=std::sqrt(ss/steady_errors.size());
    result["after_first_lap_distance_max_m"]=steady_errors.back();
  }
  std::cout<<result.dump(2)<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
