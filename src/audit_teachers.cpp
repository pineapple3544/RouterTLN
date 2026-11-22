#include "el/dataset.hpp"
#include "el/scenarios.hpp"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
using namespace el;using json=nlohmann::json;
void require(bool ok,const std::string& text){if(!ok)throw std::runtime_error(text);}
void increment(json& counts,const std::string& key,int n=1){if(counts.is_null())counts=json::object();counts[key]=counts.value(key,0)+n;}
int main(int argc,char** argv){try{
  require(argc==2,"Usage: el_audit_teachers COLLECTION_DIRECTORY");std::string dir=argv[1];std::ifstream f(dir+"/summary.json");json episodes;f>>episodes;
  json report;report["accepted_episodes"]=0;report["quarantined_episodes"]=0;
  std::set<std::string> used,maps;std::map<std::string,std::string> attempt_split,layout_split,seed_split;
  for(std::string mode:{"race","avoid"}){
    int count=0,frames=0,passed=0,spawned=0,wall=0,rear=0,on_waypoint=0,jittered=0;
    double duration=0,lap=0,ms=0,peak=0,error=0;std::map<std::string,std::vector<std::string>> lists;
    std::set<std::string> location_bins;json coverage=json::object(),sources=json::object();
    for(auto ep:episodes){if(ep["scenario"]["mode"]!=mode)continue;
      auto split=ep["scenario"].value("split",std::string("train"));require(split=="train"||split=="validation"||split=="test","unknown split");
      auto group=ep.value("source_attempt",ep["name"].get<std::string>());auto found=attempt_split.find(group);
      require(found==attempt_split.end()||found->second==split,"source attempt split leakage");attempt_split[group]=split;
      auto seed=mode+":"+std::to_string(ep["seed"].get<int>());auto sf=seed_split.find(seed);require(sf==seed_split.end()||sf->second==split,"seed split leakage");seed_split[seed]=split;
      if(!sources.contains(group)){
        json source=ep;auto attempt=dir+"/attempts/"+group+".json";
        if(ep["scenario"].value("cycling_obstacles",false))require(std::filesystem::exists(attempt),"missing original attempt metadata");
        if(std::filesystem::exists(attempt)){std::ifstream af(attempt);af>>source;}
        sources[group]={{"reason",source["reason"]},{"status",source["status"]},{"split",split},{"records",source["records"]},{"obstacles_spawned",source.value("obstacles_spawned",0)},{"obstacles_passed",source.value("obstacles_passed",0)},{"target_encounters",source["scenario"].value("target_encounters",0)},{"collisions",source["collisions"]},{"invalid_plans",source["invalid_plans"]}};
      }
      if(ep["status"]!="accepted"){increment(report,"quarantined_episodes");increment(report[mode+"_quarantine_reasons"],ep["reason"].get<std::string>());continue;}
      require(!ep.value("metrics_only",false),"metrics-only probe cannot enter a manifest");
      auto path=ep["path"].get<std::string>();require(used.insert(path).second,"duplicate file");
      std::ifstream sidecar(path+".json");json stored;sidecar>>stored;
      stored.erase("teacher_config");auto expected=ep;expected.erase("path");require(stored==expected,"summary/sidecar metadata mismatch");
      require(ep["teacher"]==(mode=="race"?"SMPPI":"SVG-MPPI"),"wrong teacher identity");
      require(ep["collisions"]==0&&!ep.value("geometry_overlap",true)&&ep["invalid_plans"]==0,"invalid episode quality");
      require(std::isfinite(ep["max_model_error"].get<double>())&&ep["max_model_error"].get<double>()<=.003,"model mismatch exceeds collection gate");
      require(fabs(ep.value("lidar_range_m",0.f)-18.f)<.001,"LiDAR must be 18 m");
      auto records=read_dataset(path);require(!records.empty()&&records.size()==ep["records"].get<size_t>(),"record count mismatch");
      double stamp=-1;uint64_t step=0;
      for(auto& r:records){require(r.step==step++&&r.stamp>stamp,"record order");stamp=r.stamp;
        require(r.mode==(mode=="race"?0:1)&&r.valid==1&&r.collision==0,"invalid learning label");
        require(std::isfinite(r.stamp)&&std::isfinite(r.label.steer)&&std::isfinite(r.label.speed),"nonfinite label or timestamp");
        require(fabs(r.label.steer)<=.4001&&r.label.speed>=0&&r.label.speed<=10.001,"label bounds");
        require(r.label.steer==r.executed.steer&&r.label.speed==r.executed.speed,"label/action mismatch");
        for(float v:r.observation.scans)require(std::isfinite(v),"nonfinite LiDAR feature");
        for(float v:r.observation.map)require(std::isfinite(v),"nonfinite map feature");
        for(float v:r.observation.context)require(std::isfinite(v),"nonfinite context feature");}
      lists[split].push_back(path);
      std::string base=ep.value("map_dir",std::string("/home/misys/forza_ws/race_stack/stack_master/maps/hyhall12"));
      auto map=std::filesystem::path(base).filename().string();maps.insert(map);bool reverse=ep["scenario"].value("reverse_track",false);
      Track reference(base+"/global_waypoints.json");if(ep.contains("map_dir"))orient_track(reference,reverse);
      increment(coverage["maps"],map);increment(coverage["directions"],reverse?"reverse":"forward");
      increment(coverage["split_map_direction"],split+":"+map+":"+(reverse?"reverse":"forward"));
      increment(coverage["speed_caps"],std::to_string(ep["speed_cap_mps"].get<float>()));
      increment(coverage["split_records"],split,records.size());
      increment(coverage["omitted_cruise_prefix_records"],split,ep.value("omitted_cruise_prefix_records",0));
      if(mode=="avoid"){
        require(ep["scenario"].value("cycling_obstacles",false),"audit expects cycling avoidance data");
        passed+=ep["obstacles_passed"].get<int>();spawned+=ep["obstacles_spawned"].get<int>();
        require(ep["obstacles_passed"].get<int>()>=ep["scenario"].value("minimum_passes",1),"not enough encounters");
        std::set<int> spawn_ids,remove_ids;
        for(auto event:ep["obstacle_events"]){
          if(event["type"]=="remove"){
            require(remove_ids.insert(event["passed_id"].get<int>()).second,"duplicate removal");
            increment(coverage["passing_side"],event.value("passing_side",std::string("unrecorded")));continue;
          }
          require(event["type"]=="spawn","unknown obstacle event");require(spawn_ids.insert(event["id"].get<int>()).second,"duplicate spawn");
          float minimum=event.value("required_min_distance_m",8.f);
          require(event["visible_beams"]==0&&event["sensor_distance_m"].get<float>()>=minimum,"unsafe spawn");
          require(event["wall_occluded"].get<bool>()||event["rear_blind_straight"].get<bool>(),"spawn lacks visibility guard");
          auto key=map+":"+event["obstacle"].dump();auto lf=layout_split.find(key);
          require(lf==layout_split.end()||lf->second==split,"obstacle geometry split leakage");layout_split[key]=split;
          if(event["mode"]=="wall_occluded")++wall;else ++rear;
          int idx=event["index"];location_bins.insert(map+":"+(reverse?"r:":"f:")+std::to_string(idx/5));
          auto w=reference.points.at(idx);float dx=event["obstacle"][0].get<float>()-w.x,dy=event["obstacle"][1].get<float>()-w.y;
          float across=-sinf(w.yaw)*dx+cosf(w.yaw)*dy,along=cosf(w.yaw)*dx+sinf(w.yaw)*dy;
          require(fabs(across-event["lateral_offset_m"].get<float>())<1e-4&&fabs(along-event["longitudinal_offset_m"].get<float>())<1e-4,"placement offset mismatch");
          require(fabs(across)<=.4001f&&fabs(along)<=.2501f,"jitter outside bounds");
          if(event["placement"]=="on_waypoint")require(hypotf(dx,dy)<1e-4,"on-waypoint obstacle is offset");
          if(hypotf(dx,dy)<1e-4)++on_waypoint;else ++jittered;
          if(event.contains("profile")){
            auto profile=event["profile"].get<std::string>();auto shape=obstacle_shape(profile);
            float len=event["obstacle"][3],width=event["obstacle"][4],yaw=event["obstacle"][2];
            require(len>=shape.min_length-1e-5&&len<=shape.max_length+1e-5&&width>=shape.min_width-1e-5&&width<=shape.max_width+1e-5,"shape outside profile");
            require(fabsf(wrap(yaw-w.yaw))<=shape.max_rotation+1e-5,"rotation outside profile");
            require(event["road_region"]==road_region(route_curvature(reference,idx)),"road region mismatch");
            increment(coverage["profiles"],profile);increment(coverage["road_regions"],event["road_region"].get<std::string>());
            increment(coverage["scenario_strata"],map+":"+(reverse?"reverse:":"forward:")+profile+":"+event["road_region"].get<std::string>()+":"+std::to_string(ep["speed_cap_mps"].get<float>()));
            increment(coverage["nominal_route_blocked"],event.value("nominal_route_blocked",false)?"yes":"no");
          }
        }
        require(spawn_ids==remove_ids&&spawn_ids.size()==size_t(ep["obstacles_passed"].get<int>()),"incomplete encounter accepted");
        require(!ep["min_obstacle_separating_axis_gap_m"].is_null()&&ep["min_obstacle_separating_axis_gap_m"].get<float>()>0,"overlapping obstacle geometry");
      }
      ++count;frames+=records.size();duration+=records.size()*.05;lap+=ep["laps"].get<double>();ms+=ep["mean_teacher_ms"].get<double>()*records.size();
      peak=std::max(peak,ep["peak_actual_speed_mps"].get<double>());error=std::max(error,ep["max_model_error"].get<double>());
    }
    require(!lists["train"].empty()&&!lists["validation"].empty(),mode+" lacks accepted train or validation episodes");
    std::ofstream y(dir+"/"+mode+"-training.yaml");y<<"# Prepared only; no training. Test episodes must never enter training/model selection.\nexpert: "<<mode<<'\n';
    for(auto split:{"train","validation","test"}){y<<split<<"_episodes:\n";for(auto& p:lists[split])y<<"  - "<<p<<'\n';}
    y<<"epochs: 20\nbatch_size: 64\nlearning_rate: 0.0001\ncheckpoint: /workspace/models/"<<mode<<"-teachers-expanded-v5.pt\n";
    report[mode]={{"episodes",count},{"records",frames},{"train_episodes",lists["train"].size()},{"validation_episodes",lists["validation"].size()},{"test_episodes",lists["test"].size()},{"simulation_seconds",duration},{"laps",lap},{"peak_actual_speed_mps",peak},{"mean_teacher_ms",ms/frames},{"max_model_error",error},{"obstacles_spawned",spawned},{"obstacles_passed",passed},{"wall_occluded_spawns",wall},{"rear_blind_spawns",rear},{"on_waypoint_placements",on_waypoint},{"offset_placements",jittered},{"distinct_route_bins",location_bins.size()},{"coverage",coverage},{"source_attempts",sources.size()}};
    for(auto it=sources.begin();it!=sources.end();++it){auto s=it.value();increment(report[mode]["source_attempt_reasons"],s["reason"].get<std::string>());
      increment(report[mode],"attempt_collisions",s["collisions"].get<int>());increment(report[mode],"attempt_invalid_plans",s["invalid_plans"].get<int>());
      increment(report[mode],"attempt_obstacles_spawned",s["obstacles_spawned"].get<int>());increment(report[mode],"attempt_obstacles_passed",s["obstacles_passed"].get<int>());
      if(mode=="avoid"&&s["obstacles_passed"].get<int>()>=s["target_encounters"].get<int>())increment(report[mode],"attempt_target_encounters_met");}
    increment(report,"accepted_episodes",count);
  }
  report["maps"]=maps;report["training_executed"]=false;
  report["scope"]="Static single-obstacle cycling; held-out source attempts and seeds, not held-out maps. No optimality or 10 m/s claim.";
  std::ofstream out(dir+"/audit.json");out<<report.dump(2);std::cout<<report.dump(2)<<std::endl;return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}}
