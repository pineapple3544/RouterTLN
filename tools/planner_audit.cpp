#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>
// Read-only audit of existing planner outputs; exports isolated copies for A/B tests.
int main(int argc,char** argv){try{
  if(argc!=3)throw std::runtime_error("usage: planner_audit MAP_DIR OUTPUT_DIR");
  namespace fs=std::filesystem;fs::path source=argv[1],out=argv[2];
  std::ifstream f(source/"global_waypoints.json");nlohmann::json original;f>>original;
  nlohmann::json report;
  for(auto name:{"iqp","sp"}){
    std::string key=std::string("global_traj_wpnts_")+name;auto w=original.at(key).at("wpnts");
    double length=0,minspeed=1e9,maxspeed=0,kappa=0,left=1e9,right=1e9,ay=0;int steer_violations=0;
    for(size_t i=0;i<w.size();++i){auto a=w[i],b=w[(i+1)%w.size()];
      length+=std::hypot(a["x_m"].get<double>()-b["x_m"].get<double>(),a["y_m"].get<double>()-b["y_m"].get<double>());
      double v=a["vx_mps"],k=std::abs(a["kappa_radpm"].get<double>());
      minspeed=std::min(minspeed,v);maxspeed=std::max(maxspeed,v);kappa=std::max(kappa,k);ay=std::max(ay,v*v*k);
      left=std::min(left,a["d_left"].get<double>());right=std::min(right,a["d_right"].get<double>());
      steer_violations+=std::atan(.3302*k)>.4;
    }
    report[name]={{"points",w.size()},{"polygon_length_m",length},{"last_s_m",w.back()["s_m"]},{"min_speed",minspeed},{"max_speed",maxspeed},
      {"max_abs_curvature",kappa},{"required_kinematic_steer_rad",std::atan(.3302*kappa)},{"steer_limit_violations",steer_violations},
      {"max_nominal_lateral_accel",ay},{"max_lateral_accel_at_2_5",6.25*kappa},{"min_left_bound_distance",left},{"min_right_bound_distance",right}};
    fs::path dir=out/name;fs::create_directories(dir);
    auto variant=original;variant["global_traj_wpnts_iqp"]=original.at(key);
    std::ofstream route(dir/"global_waypoints.json");route<<variant;
    for(auto file:{"hyhall12_orig.yaml","hyhall12_orig.png"})fs::copy_file(source/file,dir/file,fs::copy_options::overwrite_existing);
  }
  std::ofstream summary(out/"geometry.json");summary<<report.dump(2);std::cout<<report.dump(2)<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
