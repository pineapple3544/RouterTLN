#include "sim_support.hpp"
#include "el/policy.hpp"
#include "el/scan_localizer.hpp"
#include "el/eval_metrics.hpp"
#include <numeric>
#include <iomanip>
using Clock=std::chrono::steady_clock;
double elapsed_ms(Clock::time_point start){return std::chrono::duration<double,std::milli>(Clock::now()-start).count();}
json command_diagnostic(const TeacherProblem& p,Control u){
  DynamicState s=p.state;double time=0;
  for(int phase=0;phase<2;++phase){Control command=phase?Control{s.z[2],0}:u;
    for(int j=0;j<(phase?220:5);++j){if(phase&&s.z[3]<=.05f)break;s=physics_step(s,command,p.vehicle);time+=.01;
      float cost=teacher_footprint(p,s,.04f);if(cost>.5f||fabs(s.z[6])>.5f)return {{"passes",false},{"phase",phase?"braking":"command"},{"time_s",time},{"footprint_cost",cost},{"slip_rad",s.z[6]}};}
  }
  return {{"passes",s.z[3]<.05f},{"final_speed",s.z[3]}};
}
json distribution(std::vector<double> values){
  if(values.empty())return {{"count",0},{"mean",nullptr},{"median",nullptr},{"min",nullptr},{"max",nullptr},{"p95",nullptr}};
  std::sort(values.begin(),values.end());size_t n=values.size();double median=n%2?values[n/2]:(values[n/2-1]+values[n/2])*.5;
  return {{"count",n},{"mean",std::accumulate(values.begin(),values.end(),0.)/n},{"median",median},{"min",values.front()},{"max",values.back()},{"p95",values[size_t(std::ceil(.95*n))-1]}};
}
json aggregate(const json& results,const std::string& controller){
  int count=0,completed=0,collisions=0,steps=0,fallback=0,rejected=0;std::vector<double> laps,completed_runs_laps,peaks;json reasons=json::object(),by_map=json::object();
  for(auto& r:results)if(r["controller"]==controller){++count;bool done=r["reason"]=="completed";completed+=done;collisions+=r["collision"].get<bool>();steps+=r["steps"].get<int>();fallback+=r["fallback_steps"].get<int>();rejected+=r["candidate_rejected_steps"].get<int>();
    std::string reason=r["reason"];reasons[reason]=reasons.value(reason,0)+1;
    for(double v:r["lap_times_s"]){laps.push_back(v);if(done)completed_runs_laps.push_back(v);}peaks.push_back(r["peak_actual_speed_mps"]);
    auto map=r["scenario"]["map_name"].get<std::string>();if(!by_map.contains(map))by_map[map]={{"runs",0},{"completed",0},{"collisions",0}};
    by_map[map]["runs"]=by_map[map]["runs"].get<int>()+1;by_map[map]["completed"]=by_map[map]["completed"].get<int>()+int(done);by_map[map]["collisions"]=by_map[map]["collisions"].get<int>()+int(r["collision"].get<bool>());
  }
  return {{"runs",count},{"completed_runs",completed},{"completion_rate",double(completed)/std::max(1,count)},{"collision_runs",collisions},{"reasons",reasons},{"steps",steps},{"fallback_steps",fallback},{"fallback_fraction",double(fallback)/std::max(1,steps)},{"candidate_rejected_steps",rejected},{"all_completed_lap_times_s",distribution(laps)},{"fully_completed_run_lap_times_s",distribution(completed_runs_laps)},{"peak_actual_speeds_mps",distribution(peaks)},{"maps",by_map}};
}
int main(int argc,char** argv){try{
  if(argc!=2)throw std::runtime_error("Usage: el_eval_race CONFIG.json");std::ifstream f(argv[1]);json cfg;f>>cfg;
  std::string output=cfg.at("output"),checkpoint=cfg.at("checkpoint");if(std::filesystem::exists(output+"/summary.json"))throw std::runtime_error("evaluation already exists");
  std::filesystem::create_directories(output);std::ofstream(output+"/config.json")<<cfg.dump(2);
  auto controllers=cfg.at("controllers").get<std::vector<std::string>>();for(auto c:controllers)if(c!="tln_only"&&c!="tln_smppi_guarded"&&c!="smppi")throw std::runtime_error("unknown controller");
  Py_Initialize();Policy policy(checkpoint);json results=json::array();std::cout<<std::setprecision(9);
  for(auto spec:cfg.at("scenarios"))for(auto controller:controllers){
    std::string id=spec.at("name").get<std::string>()+"-"+controller,base=spec.at("map_dir"),stem=spec.at("map_stem");
    float cap=spec.at("speed_cap");int seed=spec.at("seed"),target_laps=spec.value("laps",3),max_steps=spec.value("max_steps",2400);
    if(!(cap>0&&cap<=10)||target_laps<1||max_steps<1)throw std::runtime_error("invalid evaluation limits");
    Track track(base+"/global_waypoints.json");orient_track(track,spec.value("reverse_track",false));speed_profile(track,cap);double length=route_length(track);
    int start=spec.value("start_index",0)%track.points.size();auto start_pose=track.points.at(start);float offset=spec.value("start_lateral_m",0.f),yaw=spec.value("start_yaw_rad",0.f);
    if(fabs(offset)>.3||fabs(yaw)>.25)throw std::runtime_error("start perturbation outside bounds");
    start_pose.x-=sinf(start_pose.yaw)*offset;start_pose.y+=cosf(start_pose.yaw)*offset;start_pose.yaw=wrap(start_pose.yaw+yaw);
    Sim sim(base+"/"+stem,seed,18);Memory memory(base+"/"+stem+".yaml");sim.obstacles(json::array());sim.reset(start_pose);
    if(sim.collision)throw std::runtime_error("invalid colliding initial pose");
    TeacherConfig tc;tc.kind=TeacherKind::Smppi;tc.seed=seed;tc.samples=2048;tc.horizon=40;tc.iterations=2;tc.temperature=.5f;tc.steer_sigma=.035f;tc.speed_sigma=.3f;
    ScanLocalizer localizer(memory);
    Teacher teacher(tc);std::deque<Scan> scans;Control previous;bool in_fallback=false;double safe_since=-1;
    double progress=0,last_s=project_route(track,basic(sim.state)).s,last_cross=0,peak=0,model_error=0,best_progress=0,last_advance=0;
    int steps=0,fallback_steps=0,fallback_entries=0,rejected=0,clipped=0,emergency_steps=0;std::string reason="timeout";
    json failure_diagnostic=nullptr;
    std::ofstream scan_archive;if(cfg.value("record_scans",false)){scan_archive.open(output+"/"+id+".scans",std::ios::binary);uint64_t size=sizeof(Scan);scan_archive.write(reinterpret_cast<const char*>(&size),sizeof(size));}
    std::vector<double> lap_times,infer_ms,control_ms,reference_error;std::ofstream trace(output+"/"+id+".csv");
    std::ofstream shadow;
    if(controller=="tln_only" && cfg.value("shadow_teacher",false)){
      shadow.open(output+"/"+id+"-shadow.csv");
      if(!shadow)throw std::runtime_error("cannot open shadow trace");
      shadow<<"step,x1_m,y1_m,teacher_valid,teacher_steer,teacher_speed,tln_steer,tln_speed\n";
    }
    trace<<"step,observation_time,next_time,x,y,actual_speed,raw_steer,raw_speed,executed_steer,executed_speed,candidate_safe,fallback,collision,progress_m,reference_error_m,inference_ms,control_ms\n";
    for(int step=0;step<max_steps;++step){auto cycle=Clock::now();double t0=sim.stamp;auto before=sim.state;
      Scan sample=sim.scan;
      sample.pose={};
      const auto estimate=localizer.update(sample,track,previous);
      if(!estimate){reason="localization_lost";break;}
      sample.pose=*estimate;
      if(scan_archive.is_open())scan_archive.write(reinterpret_cast<const char*>(&sample),sizeof(Scan));
      memory.update(sample);scans.push_back(sample);while(scans.size()>history)scans.pop_front();
      const auto& car=sample.pose;
      DynamicState inferred{{car.x,car.y,car.steer,car.v,car.yaw,car.yaw_rate,0},{previous.steer,previous.steer}};
      auto observation=observe(scans,car,memory,track,0,0);
      auto p=problem(track,memory,inferred,cap,previous);
      Control raw{},action{};bool safe=true,fallback=false,valid=true;double inference=0;
      if(controller!="smppi"){
        auto began=Clock::now();raw=policy.infer(observation);inference=elapsed_ms(began);infer_ms.push_back(inference);
        if(!std::isfinite(raw.steer)||!std::isfinite(raw.speed)){reason="nonfinite_policy";break;}
        action={clamp(raw.steer,-.4,.4),clamp(raw.speed,0,cap)};if(action.speed!=raw.speed||action.steer!=raw.steer)++clipped;
        safe=teacher_valid(p,{action});if(!safe)++rejected;
      }
      if(controller=="tln_smppi_guarded"){
        if(!safe){safe_since=-1;if(!in_fallback){++fallback_entries;teacher.reset();}in_fallback=true;}
        else if(in_fallback){if(safe_since<0)safe_since=t0;if(t0-safe_since>=.3){in_fallback=false;safe_since=-1;}}
      }
      if(controller=="smppi"||(controller=="tln_smppi_guarded"&&in_fallback)){
        auto plan=teacher.solve(p,initial(p,tc.horizon));valid=plan.valid;action=valid?plan.action:Control{car.steer,0};
        fallback=controller!="smppi";if(fallback)++fallback_steps;if(!valid)++emergency_steps;
        if(!valid){Memory base_memory(base+"/"+stem+".yaml");auto base_problem=problem(track,base_memory,inferred,cap,previous);
          Control candidate{clamp(raw.steer,-.4,.4),clamp(raw.speed,0,cap)};
          failure_diagnostic={{"observation_time",t0},{"step",step},{"current_world_state",sim.state.z},{"current_footprint_margin_0",teacher_footprint(p,p.state,0)},{"current_footprint_margin_004",teacher_footprint(p,p.state,.04f)},
            {"base_only_footprint_margin_0",teacher_footprint(base_problem,base_problem.state,0)},{"base_only_footprint_margin_004",teacher_footprint(base_problem,base_problem.state,.04f)},
            {"candidate_with_observed_memory",command_diagnostic(p,candidate)},{"candidate_base_map_only",command_diagnostic(base_problem,candidate)},
            {"nominal_valid_observed_memory",teacher_valid(p,initial(p,tc.horizon))},{"nominal_valid_base_map_only",teacher_valid(base_problem,initial(base_problem,tc.horizon))}};
          std::ofstream dump(output+"/"+id+"-failure.problem",std::ios::binary);dump.write(reinterpret_cast<const char*>(&p),sizeof(p));
        }
      }
      if(shadow.is_open()){
        auto plan=teacher.solve(p,initial(p,tc.horizon));
        shadow<<step<<','<<10*observation.context[5]<<','<<10*observation.context[6]<<','<<plan.valid<<','<<plan.action.steer<<','<<plan.action.speed<<','<<raw.steer<<','<<raw.speed<<'\n';
      }
      double computation=elapsed_ms(cycle);control_ms.push_back(computation);previous=action;
      for(int j=0;j<5;++j){auto predicted=physics_step(sim.state,action,p.vehicle);sim.step(action);peak=std::max(peak,double(sim.state.z[3]));
        if(!sim.collision)for(int k=0;k<7;++k){double e=k==4?fabs(wrap(predicted.z[k]-sim.state.z[k])):fabs(predicted.z[k]-sim.state.z[k]);model_error=std::max(model_error,e);}
        if(model_error>.003)throw std::runtime_error("simulator/model mismatch > .003");if(sim.collision)break;}
      auto projected=project_route(track,basic(sim.state));double delta=route_delta(projected.s,last_s,length),before_progress=progress;
      if(fabs(delta)>2){reason="progress_discontinuity";break;}progress+=delta;last_s=projected.s;reference_error.push_back(projected.distance);++steps;
      while(progress>=(lap_times.size()+1)*length&&lap_times.size()<size_t(target_laps)){
        double crossing=crossing_time((lap_times.size()+1)*length,before_progress,progress,t0,sim.stamp);lap_times.push_back(crossing-last_cross);last_cross=crossing;
      }
      trace<<step<<','<<t0<<','<<sim.stamp<<','<<before.z[0]<<','<<before.z[1]<<','<<before.z[3]<<','<<raw.steer<<','<<raw.speed<<','<<action.steer<<','<<action.speed<<','<<safe<<','<<fallback<<','<<sim.collision<<','<<progress<<','<<projected.distance<<','<<inference<<','<<computation<<'\n';
      if(step%200==0)std::cout<<id<<" step="<<step<<" laps="<<progress/length<<" speed="<<sim.state.z[3]<<" fallback="<<fallback_steps<<std::endl;
      if(sim.collision){reason="collision";break;}if(!valid){reason="no_feasible_plan";break;}
      if(lap_times.size()>=size_t(target_laps)){reason="completed";break;}
      if(progress>best_progress+.5){best_progress=progress;last_advance=sim.stamp;}if(sim.stamp-last_advance>10){reason="stalled";break;}
    }
    double sum2=0;for(auto e:reference_error)sum2+=e*e;
    json row={{"pose_source","lidar_static_map"},{"id",id},{"controller",controller},{"scenario",spec},{"checkpoint",checkpoint},{"reason",reason},{"steps",steps},{"collision",sim.collision},{"simulation_seconds",sim.stamp},{"track_length_m",length},{"progress_m",progress},{"lap_times_s",lap_times},{"completed_laps",lap_times.size()},{"peak_actual_speed_mps",peak},{"model_error_max",model_error},{"fallback_steps",fallback_steps},{"fallback_entries",fallback_entries},{"candidate_rejected_steps",rejected},{"command_clipped_steps",clipped},{"emergency_steps",emergency_steps},{"inference_ms",distribution(infer_ms)},{"control_ms",distribution(control_ms)},{"reference_error_m",distribution(reference_error)},{"reference_error_rms_m",sqrt(sum2/std::max(size_t(1),reference_error.size()))},{"synchronous",true},{"physics_dt",.01},{"control_dt",.05},{"obstacles",0},{"training_executed",false},{"trace",output+"/"+id+".csv"}};
    row["failure_diagnostic"]=failure_diagnostic;results.push_back(row);std::ofstream(output+"/summary.json")<<results.dump(2);
    std::cout<<"RESULT "<<id<<" "<<reason<<" laps="<<lap_times.size()<<" fallback="<<fallback_steps<<"/"<<steps<<std::endl;
  }
  json report={{"completed",true},{"checkpoint",checkpoint},{"runs",results.size()},{"scope","Offline synchronous F110 Gym closed-loop, static walls and no added obstacles. Guard uses dynamic one-command plus braking check and SMPPI v5 with 0.3s return dwell, not the ROS fallback implementation. No real-time deadline enforcement."},{"lap_timing","Polyline projected signed progress, interpolated full-loop crossings; standing-start first lap included. Incomplete laps excluded and failed runs reported separately."},{"training_executed",false}};
  for(auto controller:controllers)report[controller]=aggregate(results,controller);std::ofstream(output+"/report.json")<<report.dump(2);std::cout<<report.dump(2)<<std::endl;return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}}
