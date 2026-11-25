#include "avoid_support.hpp"
#include <chrono>
#include <sys/resource.h>
#include "el/map_paths.hpp"
#include "el/eval_metrics.hpp"
#include "el/section_router.hpp"
#include "el/scan_localizer.hpp"
#include "offline_snapshot.hpp"
// Offline paired replay. Only the steering changes, before the common speed
// planner, actuator simulation and footprint checks. The teacher sees the
// estimated car and sensor occupancy, never the simulator vehicle state.
struct ReplayRacePolicy {
  Policy& race;
  Teacher* teacher;
  const TeacherConfig& config;
  const Track& track;
  const Memory& memory;
  const State& car;
  Control previous;
  float cap;
  int& steps;
  bool no_plan=false;
  Control infer(const Observation& observation,ControlTiming* timing){
    Control action=race.infer(observation,timing);
    if(!teacher)return action;
    DynamicState estimate{{car.x,car.y,car.steer,car.v,car.yaw,car.yaw_rate,0},
                          {previous.steer,previous.steer}};
    auto p=problem(track,memory,estimate,cap,previous);
    auto plan=teacher->solve(p,initial(p,config.horizon));
    ++steps;
    no_plan=!plan.valid;
    action.steer=plan.valid ? plan.action.steer : std::numeric_limits<float>::quiet_NaN();
    return action;
  }
};

// Count obstacle surfaces actually returned on the beams retained by the policy.
bool jointly_visible(const Sim& sim,const json& boxes){
  Py hits=attr(sim.car.p,"static_beams");
  if(hits.p==Py_None)return false;
  bool seen[2]={};
  float yaw=sim.scan.pose.yaw;
  for(int i=0;i<beams;++i){
    int k=lround(double(i)*(sim.raw_scan.size()-1)/(beams-1));
    if(!sim.scan.valid[i]||sim.scan.ranges[i]>=18||!PyObject_IsTrue(item(hits.p,k).p))continue;
    float a=yaw+sim.scan.angles[i],r=sim.scan.ranges[i];
    float x=sim.scan.pose.x+.27f*cosf(yaw)+r*cosf(a),y=sim.scan.pose.y+.27f*sinf(yaw)+r*sinf(a);
    for(int j=0;j<2;++j){
      auto& b=boxes[j];float h=b[2],dx=x-b[0].get<float>(),dy=y-b[1].get<float>();
      float u=fabsf(dx*cosf(h)+dy*sinf(h))-b[3].get<float>()*.5f;
      float v=fabsf(-dx*sinf(h)+dy*cosf(h))-b[4].get<float>()*.5f;
      if(fabsf(std::max(u,v))<.015f)seen[j]=true;
    }
  }
  return seen[0]&&seen[1];
}
// Place each fixed rectangle by arc distance along the oriented racing line.
json run(const std::string& map,int layout,int count,Policy& policy,bool baseline,const json& settings,const json& spec=json::object(),const std::string& save="",Policy* race=nullptr,const std::string& replay="",bool recovery=false,Policy* prefix=nullptr,const json& runtime=json::object()){
  std::string base = map == "berlin" ? el::default_map_dir : "/home/misys/forza_ws/race_stack/stack_master/maps/"+map;
  auto stem=el::map_stem(base);
  auto router_settings=el::map_router_settings(base);
  router_settings.lap_planning=spec.value("lap_planning",false);
  Track track(base+"/global_waypoints.json");
  orient_track(track,layout%2);
  float cap=spec.value("speed_cap",3.f);
  const bool router_lap=spec.value("full_lap",false) && race && !recovery;
  if(router_lap && (prefix || baseline || !replay.empty()))
    throw std::runtime_error("full-lap router evaluation cannot use teacher replay or oracle handoff");
  const float eval_limit=router_lap ? (spec.value("sim_avoid_max",0.f)>0 ? 10.f : 5.f) : 3.f;
  if (!(cap>0 && cap<=eval_limit)) throw std::runtime_error("invalid evaluation speed cap");
  router_settings.scout_speed_cap=spec.value("scout_speed_cap",router_settings.scout_speed_cap);
  router_settings.plan_lead_time=spec.value("plan_lead_time",router_settings.plan_lead_time);
  router_settings.plan_speed_fraction=spec.value("plan_speed_fraction",router_settings.plan_speed_fraction);
  if (!(router_settings.scout_speed_cap>0 && router_settings.scout_speed_cap<=10 &&
        router_settings.plan_lead_time>=0 && router_settings.plan_lead_time<=2 &&
        router_settings.plan_speed_fraction>0 && router_settings.plan_speed_fraction<=1))
    throw std::runtime_error("invalid lap planning settings");
  router_settings.sim_avoid_min=spec.value("sim_avoid_min",0.f);
  router_settings.sim_avoid_max=spec.value("sim_avoid_max",0.f);
  router_settings.sim_section_speed_profile=spec.value("sim_section_speed_profile",false);
  router_settings.race_sight_speed=spec.value("race_sight_speed",false);
  router_settings.race_lap_speed=spec.value("race_lap_speed",false);
  router_settings.race_exit_speed=spec.value("race_exit_speed",false);
  router_settings.straight_boost=spec.value("straight_boost",false);
  router_settings.avoid_corridor_steer_seed=spec.value("avoid_corridor_steer_seed",false);
  router_settings.avoid_policy_first=spec.value("avoid_policy_first",false);
  router_settings.cluster_entry=spec.value("cluster_entry",false);
  router_settings.entry_candidates=parse_entry_candidates(spec.value("entry_candidates",std::string("both")));
  router_settings.smooth_isolated_exit=spec.value("smooth_isolated_exit",false);
  router_settings.shared_speed_band=spec.value("shared_speed_band",false);
  router_settings.whole_lap_path=spec.value("whole_lap_path",false);
  router_settings.avoid_exit_pace=spec.value("avoid_exit_pace",false);
  router_settings.race_verified_pace=spec.value("race_verified_pace",false);
  if(router_settings.avoid_policy_first && !router_settings.avoid_corridor_steer_seed)
    throw std::runtime_error("Avoid policy-first requires the corridor steering fallback");
  if(router_settings.avoid_corridor_steer_seed && !(router_lap && router_settings.lap_planning && policy.uses_corridor()))
    throw std::runtime_error("corridor steering seed requires full-lap planning and a corridor-conditioned Avoid policy");
  router_settings.sim_race_only=spec.value("sim_race_only",false);
  router_settings.race_brake_preview_s=spec.value("race_brake_preview_s",.18f);
  if(router_settings.shared_speed_band)router_settings.sim_avoid_max=cap;
  router_settings.validate_sim_band(cap,race && spec.value("full_lap",false) && !recovery);
  const int laps=spec.value("laps",1);
  const double horizon=spec.value("duration_s",0.0);
  const bool timed_race=laps==0;
  if(!std::isfinite(horizon) || horizon<0 || (timed_race && (!router_lap || horizon<=0)))
    throw std::runtime_error("timed race requires a positive duration and full-lap router");
  const bool scout_then_cruise=spec.value("scout_then_cruise",false);
  if(laps<0 || laps>20 || (laps>1 && (!race || !spec.value("full_lap",false) || recovery || prefix)))throw std::runtime_error("invalid multi-lap evaluation");
  const bool ros_finish=router_lap && spec.value("ros_finish",false);
  if(!ros_finish)speed_profile(track,cap);
  double length=route_length(track);
  int seed=891000+layout+(map=="teras"?100:0);
  int start=(37+layout*59)%track.points.size();
  start=spec.value("start",start)%track.points.size();
  seed=spec.value("seed",seed);
  auto pose=track.points[start];
  const auto family=spec.value("family",std::string{});
  const float start_d=spec.value("start_d",0.f),start_heading=spec.value("start_heading",0.f);
  pose.x-=sinf(pose.yaw)*start_d;pose.y+=cosf(pose.yaw)*start_d;
  if(start_heading!=0)pose.yaw=wrap(pose.yaw+start_heading);
  double spacing=layout<2?1.5:3.0;
  json boxes=json::array();
  std::vector<double> stations;
  Memory memory(stem+".yaml");
  std::optional<ScanLocalizer> localizer;
  const bool sensor_only=!baseline && !recovery && replay.empty() && !prefix && save.empty();
  if(sensor_only)localizer.emplace(memory);
  for(int i=0;i<count;++i){
    double station=spec.empty()?5.73+i*spacing:spec.at("stations").at(i).get<double>();
    double s=fmod(pose.s+station,length);
    auto it=std::min_element(track.points.begin(),track.points.end(),[s](auto a,auto b){return fabs(a.s-s)<fabs(b.s-s);});
    float d=(i%2?-1:1)*(layout%2?-1:1)*(.083f+.019f*layout);
    if(!spec.empty())d=spec.at("offsets").at(i);
    float x=it->x-sinf(it->yaw)*d,y=it->y+cosf(it->yaw)*d;
    auto dims=spec.contains("shapes")?spec.at("shapes").at(i):json::array({.30,spec.value("box_width",.30),0});
    float box_yaw=it->yaw+dims[2].get<float>(),len=dims[0],width=dims[1];
    float radius=.5f*hypotf(len,width);
    if(!memory.base_free_disc(x,y,radius))return {{"reason","invalid_box"},{"spec",spec}};
    boxes.push_back({x,y,box_yaw,len,width});
    stations.push_back(fmod(it->s-pose.s+length,length));
  }
  auto& first_box=boxes.front();
  bool hidden=memory.wall_hides_disc(pose.x+.27f*cosf(pose.yaw),pose.y+.27f*sinf(pose.yaw),
                                     first_box[0],first_box[1],.5f*hypotf(first_box[3],first_box[4]));
  Sim sim(stem,seed,18);sim.obstacles(boxes);sim.reset(pose);
  TeacherConfig tc;tc.kind=TeacherKind::Svg;tc.seed=seed;
  tc.samples=settings.at("samples");tc.horizon=settings.at("horizon");tc.iterations=settings.value("iterations",2);
  tc.guide_iterations=settings.value("guide_iterations",6);tc.guide_samples=settings.value("guide_samples",128);
  tc.temperature=settings.at("temperature");tc.steer_sigma=settings.at("steer_sigma");tc.speed_sigma=settings.at("speed_sigma");
  std::unique_ptr<Teacher> teacher;
  if(baseline || recovery)teacher=std::make_unique<Teacher>(tc);
  std::vector<Record> demonstrations;
  AvoidCorridor replay_corridor;
  if(!replay.empty()){
    if(baseline||race||save.empty())throw std::runtime_error("invalid offline teacher replay");
    demonstrations=read_dataset(replay+"/"+spec.at("name").get<std::string>()+".elds");
  }
  SectionRouter router;
  json events=json::array();
  Mode previous_mode=Mode::Race;
  int hold_steps=0;
  json handoff=nullptr,learner_switch=nullptr;
  bool taking_over=false;
  int teacher_steps=0,no_guide=0;

  bool early_return=false,started_before=false,returned_after=false;

  std::deque<Scan> scans;Control previous;
  double last=project_route(track,basic(sim.state)).s,progress=0,min_gap=1e9;
  std::vector<Record> records;int visible_frames=0;
  json first_rejection=nullptr;
  float entry_speed=-1, peak_avoid_speed=0;
  double avoid_speed_sum=0, avoid_command_sum=0;
  int avoid_steps=0;
  // These counters cover decisions executed in this invocation, including
  // when the dynamics and historical policy sums resume from a checkpoint.
  int cluster_checks=0,cluster_changes=0,exit_checks=0,exit_changes=0;
  int observed_avoid_steps=0,policy_verified_steps=0,policy_match_steps=0;
  int control_samples=0,over_25ms=0;
  double worst_ms=0,avoid_seconds=0;
  using Clock=std::chrono::steady_clock;
  std::vector<float> obstacle_speeds;
  json crossings=json::array(),slow_controls=json::array(),worst_control=nullptr;
  const bool profile_stages=spec.value("profile_control_stages",false);
  const int profile_step=spec.value("profile_step",-1);
  json watched_control=nullptr;
  std::ifstream schedstat;
  if(spec.value("profile_scheduler",false)){
    if(!profile_stages)throw std::runtime_error("scheduler profiling requires stage profiling");
    schedstat.open("/proc/thread-self/schedstat");
    if(!schedstat)throw std::runtime_error("thread schedstat unavailable");
  }
  struct Scheduling {
    long long queue_ns=0, involuntary=0, voluntary=0, major_faults=0;
    Scheduling operator-(const Scheduling& other)const{
      return {queue_ns-other.queue_ns,involuntary-other.involuntary,
              voluntary-other.voluntary,major_faults-other.major_faults};
    }
  };
  auto scheduling=[&](){
    long long runtime=0,delay=0;
    schedstat.clear();schedstat.seekg(0);
    rusage usage{};
    if(!(schedstat>>runtime>>delay) || getrusage(RUSAGE_THREAD,&usage)!=0)
      throw std::runtime_error("thread scheduler counters unavailable");
    return Scheduling{delay,usage.ru_nivcsw,usage.ru_nvcsw,usage.ru_majflt};
  };
  std::ofstream phase_log;
  double previous_interval_gap=INFINITY; // Judge-only minimum over the preceding physics ticks.
  if(spec.contains("phase_trace")){
    phase_log.exceptions(std::ios::failbit|std::ios::badbit);
    phase_log.open(spec.at("phase_trace").get<std::string>());
    phase_log<<"step,t,s,lap,passed,mode,hazard,past,merge_blocked,lateral,heading,steer,policy,v,adopted,target,policy_speed,curve_cap,lap_cap,preview_brake,entry_checked,entry_blend,prev_interval_gap_m\n";
  }
  int passed=0;std::string reason="timeout";
  float min_moving_speed=std::numeric_limits<float>::infinity();
  bool full_lap=spec.value("full_lap",false);
  if(full_lap && (!race || recovery))throw std::runtime_error("full lap requires router evaluation");
  json lap_results=json::array();
  json contact=nullptr;
  const bool trace_avoid=spec.value("trace_avoid",false);
  json avoid_trace=json::array();
  double previous_cross=0, peak_speed=0, peak_command=0;
  int lap=0;
  // Gym advances in 10 ms quanta. Sample the latest completed frame on a
  // 25 ms clock, so commands span alternating 20/30 ms intervals.
  const bool legacy_replay=!demonstrations.empty() && demonstrations.size()>1 &&
      std::abs(demonstrations[1].stamp-demonstrations[0].stamp-.05)<1e-4;
  const float control_dt=legacy_replay?.05f:.025f;
  const json checkpoint=spec.value("checkpoint",json::object());
  if(!checkpoint.empty() && (!router_lap || prefix || baseline || recovery))
    throw std::runtime_error("checkpoint requires a full-lap router run");
  const std::string restore=checkpoint.value("restore",std::string{}),output=checkpoint.value("save",std::string{});
  const double lookback=checkpoint.value("lookback_s",.60);
  if(!(lookback>0 && lookback<=10))throw std::runtime_error("invalid lookback");
  if(!output.empty() && std::filesystem::exists(output))throw std::runtime_error("checkpoint output exists");
  const json perturb=checkpoint.value("perturb",json::object());
  if(!perturb.empty() && restore.empty())throw std::runtime_error("perturbations require a restored branch");
  const float lateral=perturb.value("lateral_m",0.f),yaw_delta=perturb.value("yaw_rad",0.f),dv=perturb.value("speed_mps",0.f);
  if(!std::isfinite(lateral)||!std::isfinite(yaw_delta)||!std::isfinite(dv))throw std::runtime_error("invalid perturbation");
  const int branch_ticks=checkpoint.value("horizon_ticks",0);
  if(branch_ticks<0 || (branch_ticks && restore.empty()))throw std::runtime_error("invalid branch horizon");
  std::ofstream ticks_log;
  if(checkpoint.contains("ticks")){
    auto path=checkpoint.at("ticks").get<std::string>();
    if(std::filesystem::exists(path))throw std::runtime_error("ticks file exists");
    ticks_log.exceptions(std::ios::failbit|std::ios::badbit);ticks_log.open(path);
  }
  std::unique_ptr<Py> module;
  json identity=spec;identity.erase("name");identity.erase("checkpoint");identity.erase("trace_avoid");identity.erase("phase_trace");identity.erase("profile_control_stages");identity.erase("profile_step");identity.erase("profile_scheduler");
  if(!checkpoint.empty()){
    module=std::make_unique<Py>(snapshot_module());
    auto files=runtime.at("files");
    identity["device"]=runtime.at("device");
    // Paired planner experiments can name the immutable producer binary.
    // Only its bytes enter the original fingerprint; models, map, device,
    // simulation state format and all scenario settings must still match.
    const std::string producer=checkpoint.value("producer_executable",std::string("/proc/self/exe"));
    if(checkpoint.contains("producer_executable") && (restore.empty() || !output.empty() ||
       !std::filesystem::is_regular_file(producer)))throw std::runtime_error("invalid checkpoint producer for paired replay");
    for(auto file:{stem+".yaml",stem+".png",base+"/global_waypoints.json",base+"/router.yaml",sim.module_path,
                   producer,(std::filesystem::path(__FILE__).parent_path()/"offline_gym_state.py").string()})
      if(std::filesystem::is_regular_file(file))files.push_back(file);
    Py signature(PyObject_CallMethod(module->p,"fingerprint","s",files.dump().c_str()));
    identity["fingerprint"]=PyUnicode_AsUTF8(signature.p);
  }
  int step=0;
  // Snapshot after the command decision, before the first actuation tick.
  // A displaced car gets a new physical scan before the next inference.
  auto transfer=[&](json& data,bool reading){
    OfflineSnapshot a{data,reading};
    a.controller(memory,router,scans,track.points.size());if(localizer)a.localization(*localizer);a.simulator(sim,module->p);
    a.field("step",step);a.field("last",last);a.field("progress",progress);a.field("gap",min_gap);
    a.flat("previous",&previous,1);a.field("mode",previous_mode);a.field("events",events);
    a.field("hold",hold_steps);a.field("started",started_before);a.field("early_return",early_return);
    a.field("returned",returned_after);a.field("visible",visible_frames);a.field("entry_speed",entry_speed);
    a.field("peak",peak_avoid_speed);a.field("speed_sum",avoid_speed_sum);a.field("command_sum",avoid_command_sum);
    a.field("avoid_steps",avoid_steps);a.field("obs_speeds",obstacle_speeds);a.field("passed",passed);
    a.field("min_speed",min_moving_speed);a.field("laps",lap_results);a.field("last_cross",previous_cross);
    a.field("lap",lap);
  };
  bool resumed=!restore.empty(),written=false;
  json entry=nullptr;
  if(resumed){
    std::ifstream input(restore,std::ios::binary);if(!input)throw std::runtime_error("missing checkpoint");
    json state=json::from_cbor(input);
    if(state.at("version")!=1 || state.at("identity")!=identity)throw std::runtime_error("incompatible checkpoint scenario or build");
    transfer(state,true);
    auto before=basic(sim.state);auto& w=track.points[track.closest(before)];
    float dx=-sinf(w.yaw)*lateral,dy=cosf(w.yaw)*lateral;
    if(lateral!=0 || yaw_delta!=0 || dv!=0){
      Py moved(PyObject_CallMethod(module->p,"displace","Odddd",sim.env.p,double(dx),double(dy),double(yaw_delta),double(dv)));
      sim.state.z[0]+=dx;sim.state.z[1]+=dy;sim.state.z[4]=wrap(sim.state.z[4]+yaw_delta);
      sim.state.z[3]=std::max(0.f,sim.state.z[3]+dv);
    }
    const auto e=basic(sim.state);
    entry={{"step",step},{"s",progress},{"stamp",sim.stamp},{"x",e.x},{"y",e.y},{"yaw",e.yaw},{"v",e.v},
      {"lateral_m",-(e.x-w.x)*sinf(w.yaw)+(e.y-w.y)*cosf(w.yaw)},{"heading_error_rad",wrap(e.yaw-w.yaw)},
      {"plan_ready",router.lap_plan().ready()}};
  }
  if(checkpoint.contains("race_branch")){
    if(restore.empty() || !router_lap || !router_settings.sim_race_only)
      throw std::runtime_error("Race branch requires a restored Race-only full-lap checkpoint");
    const auto& branch=checkpoint.at("race_branch");
    cap=branch.at("speed_cap").get<float>();
    if(!(cap>0 && cap<=eval_limit))throw std::runtime_error("invalid branch speed cap");
    router_settings.race_sight_speed=branch.at("race_sight_speed").get<bool>();
    router_settings.race_lap_speed=branch.value("race_lap_speed",router_settings.race_lap_speed);
    router_settings.race_exit_speed=branch.at("race_exit_speed").get<bool>();
    router_settings.validate_sim_band(cap,true);
  }
  if(checkpoint.contains("entry_branch")){
    if(restore.empty() || !sensor_only || !router_settings.whole_lap_path || !branch_ticks)
      throw std::runtime_error("entry branch requires a bounded sensor-only whole-lap replay");
    const auto variant=checkpoint.at("entry_branch").get<std::string>();
    router_settings.cluster_entry=variant!="local";
    router_settings.entry_candidates=variant=="local"?EntryCandidates::both:parse_entry_candidates(variant);
    router_settings.validate_sim_band(cap,true);
  }
  const bool steer_replay=checkpoint.value("compare_teacher_steering",false);
  if(steer_replay){
    if(!sensor_only || !router_lap || !branch_ticks || restore.empty() || !router_settings.race_lap_speed)
      throw std::runtime_error("steering comparison requires a bounded sensor-only scout speed plan branch");
    teacher=std::make_unique<Teacher>(tc);
  }
  const int start_step=step;
  const bool manual=checkpoint.contains("at_m");
  if(manual && output.empty())throw std::runtime_error("at_m requires a checkpoint save path");
  std::deque<std::pair<double,json>> recent;
  auto save_before_failure=[&](){
    if(!manual && !written && !recent.empty()){
      write_snapshot(output,recent.front().second);written=true;
    }
  };
  for(;timed_race || step<(full_lap?4800*laps:1600);++step){
    const double interval_start=sim.stamp;
    const int physics_ticks=legacy_replay?5:(step%2?3:2);
    Control cmd=previous;
    if(!resumed){
      if(sim.stamp-sim.scan.stamp>.150){reason="sensor_timeout";save_before_failure();break;}
      // Exclude cold global localization (ROS acquires pose at rest), physics,
      // judge bookkeeping and trace serialization from the control measurement.
      const bool measure=timed_race && localizer && localizer->locked();
      std::optional<ControlTiming> stages;
      const bool track_scheduler=measure && schedstat.is_open();
      Scheduling sense_before{},sense_delta{},route_before{};
      if(track_scheduler)sense_before=scheduling();
      if(profile_stages && measure)stages.emplace();
      const auto sense_start=Clock::now();
      Scan observation=sim.scan;
      State car{};
      if(localizer){
        observation.pose={};
        const auto estimate=localizer->update(observation,track,previous);
        if(!estimate){reason="localization_lost";break;}
        car=*estimate;observation.pose=car;
      }else car=basic(sim.state); // Teacher / training rollouts.
      if(stages)stages->mark(ControlTiming::scan);
      memory.update(observation);scans.push_back(observation);if(scans.size()>history)scans.pop_front();
      const auto sense_end=Clock::now();
      if(stages)stages->mark(ControlTiming::memory);
      if(track_scheduler)sense_delta=scheduling()-sense_before;
      if(count>=2 && progress<stations[0]-.35 && jointly_visible(sim,boxes))++visible_frames;
      if(recovery && !taking_over && previous_mode==Mode::Avoid){
        auto& w=track.points[track.closest(car)];
        const float lateral=-(car.x-w.x)*sinf(w.yaw)+(car.y-w.y)*cosf(w.yaw);
        const float heading=wrap(car.yaw-w.yaw);
        bool activate=false;
        if(family=="offset_entry")activate=passed==0 && (fabsf(lateral)>=.12f || fabsf(heading)>=.12f);
        if(family=="blind_corner")activate=hidden && passed==0;
        if(family=="switchback")activate=passed==1;
        if(family=="blocked_merge")activate=passed>=1 && passed<count && router.merge_blocked();
        if(family=="return_offset")activate=passed==count && (fabsf(lateral)>=.14f || fabsf(heading)>=.12f);
        if(family=="before_stop")activate=progress>=spec.value("takeover_m",1e6);
        if(activate){
          handoff={{"step",step},{"progress_m",progress},{"passed",passed},{"lateral_m",lateral},
                   {"heading_rad",heading},{"speed_mps",car.v},{"previous_merge_blocked",router.merge_blocked()},
                   {"first_hidden_at_reset",hidden},{"previous_command",{previous.steer,previous.speed}}};
          taking_over=true;
        }
      }
      if(taking_over){
        Observation input;
        auto route=router.route_observation(track,memory,car,scans,true,sim.stamp,input);
        auto p=problem(track,memory,sim.state,cap,previous);local_route(p);
        auto plan=teacher->solve(p,initial(p,tc.horizon));
        if(!plan.valid){reason="teacher_no_plan";break;}
        cmd=plan.action;++teacher_steps;
        if(route.mode==Mode::Avoid){
          Record rec;rec.step=step;rec.stamp=sim.stamp;rec.mode=1;rec.valid=1;
          rec.label=rec.executed=cmd;rec.observation=std::move(input);records.push_back(std::move(rec));
        }else if(route.mode==Mode::Emergency)++no_guide;
        previous_mode=route.mode;
        if(passed==count && route.mode==Mode::Race)returned_after=true;
      }else if(!replay.empty()){
        if(size_t(step)>=demonstrations.size())throw std::runtime_error("teacher replay exceeded recorded length");
        auto car=basic(sim.state);const auto& stored=demonstrations.at(step);
        if(std::abs(stored.stamp-sim.stamp)>1e-4 || fabsf(stored.observation.context[0]-car.v/8)>.001f)
          throw std::runtime_error("teacher replay diverged");
        cmd=stored.executed;
        if(replay_corridor.plan(track,track.closest(car),car,memory)){
          Record rec=stored;rec.observation=observe(scans,car,memory,track,1,0);
          replay_corridor.condition(rec.observation,car);records.push_back(std::move(rec));
        }
      }else if(baseline){
        auto p=problem(track,memory,sim.state,cap,previous);local_route(p);
        auto plan=teacher->solve(p,initial(p,tc.horizon));
        if(!plan.valid){reason="no_feasible_plan";break;}
        cmd=plan.action;
        if(!save.empty()){
          Record rec;rec.step=step;rec.stamp=sim.stamp;rec.mode=1;rec.valid=1;rec.label=rec.executed=cmd;
          rec.observation=observe(scans,basic(sim.state),memory,track,1,0);records.push_back(std::move(rec));
        }
      }else if(race){
        // Compare candidate policies from the identical learner-driven prefix.
        if(prefix && learner_switch.is_null() && previous_mode==Mode::Avoid && progress>=spec.at("takeover_m").get<double>())
          learner_switch={{"step",step},{"s",progress},{"passed",passed},{"x",car.x},{"y",car.y},{"yaw",car.yaw},
                          {"v",car.v},{"steer",car.steer},{"yaw_rate",car.yaw_rate},{"delay",{sim.state.delay[0],sim.state.delay[1]}}};
        Policy& actor=prefix && learner_switch.is_null()?*prefix:policy;
        CorridorTrace trace;
        ReplayRacePolicy replay_race{*race,steer_replay?teacher.get():nullptr,tc,track,memory,car,previous,cap,teacher_steps};
        if(track_scheduler)route_before=scheduling();
        if(stages)stages->restart_interval();
        const auto route_start=Clock::now();
        auto result=router.step(track,memory,car,scans,replay_race,actor,(!router.lap_plan().ready() && scout_then_cruise)?std::min(cap,router_settings.scout_speed_cap):cap,control_dt,sim.stamp,router_settings,stages?&*stages:nullptr,ticks_log.is_open()?&trace:nullptr);
        if(measure){
          const double ms=std::chrono::duration<double,std::milli>(sense_end-sense_start+ (Clock::now()-route_start)).count();
          if(stages){
            stages->mark(ControlTiming::safety);
            if(ms>worst_ms || ms>25 || step==profile_step){
              const Scheduling route_delta=track_scheduler?scheduling()-route_before:Scheduling{};
              json frame={{"step",step},{"time_s",sim.stamp},{"mode",int(result.mode)},{"elapsed_ms",ms},
                          {"tln",result.avoid_policy_verified}};
              for(int i=0;i<ControlTiming::count;++i){
                auto stage=static_cast<ControlTiming::Stage>(i);
                frame["stages"][ControlTiming::names[i]]={stages->wall_ms(stage),stages->cpu_ms(stage)};
              }
              if(track_scheduler){
                frame["run_queue_ms"]=(sense_delta.queue_ns+route_delta.queue_ns)*1e-6;
                frame["involuntary_switches"]=sense_delta.involuntary+route_delta.involuntary;
                frame["voluntary_switches"]=sense_delta.voluntary+route_delta.voluntary;
                frame["major_faults"]=sense_delta.major_faults+route_delta.major_faults;
              }
              if(step==profile_step)watched_control=frame;
              if(ms>worst_ms)worst_control=frame;
              if(ms>25)slow_controls.push_back(std::move(frame));
            }
          }
          ++control_samples;
          if(ms>25.)++over_25ms;
          worst_ms=std::max(worst_ms,ms);
        }
        if(replay_race.no_plan){reason="teacher_no_plan";break;}
        cluster_checks+=result.cluster_evaluated;cluster_changes+=result.cluster_changed;
        exit_checks+=result.exit_evaluated;exit_changes+=result.exit_smoothed;
        cmd=result.command;
        if(phase_log.is_open()){
          const auto& w=track.points[track.closest(car)];
          const float lat=-(car.x-w.x)*sinf(w.yaw)+(car.y-w.y)*cosf(w.yaw);
          phase_log<<step<<','<<sim.stamp<<','<<progress<<','<<lap+1<<','<<passed<<','<<int(result.mode)
            <<','<<result.blocked<<','<<router.distance_past_section()<<','<<router.merge_blocked()<<','<<lat
            <<','<<wrap(car.yaw-w.yaw)<<','<<cmd.steer<<','<<result.avoid_policy_steer<<','<<car.v
            <<','<<result.avoid_policy_verified<<','<<cmd.speed<<','<<result.speed_limits.policy
            <<','<<result.speed_limits.curve<<','<<result.speed_limits.lap<<','<<result.speed_limits.preview_brake
            <<','<<result.cluster_evaluated<<','<<result.entry_blend<<','<<previous_interval_gap<<'\n';
        }
        if(ticks_log.is_open()){
          json tick={{"step",step},{"t",sim.stamp},{"s",progress},{"car",{car.x,car.y,car.yaw,car.v,car.steer}},
            {"speed_limits",{{"policy",result.speed_limits.policy},{"lap",result.speed_limits.lap},
              {"curve",result.speed_limits.curve},{"sight",result.speed_limits.sight},{"steering",result.speed_limits.steering},{"race_plan",result.speed_limits.planned},
              {"corridor",result.speed_limits.corridor},{"before_guard",result.speed_limits.before_guard},{"preview_brake",result.speed_limits.preview_brake}}},
            {"policy_steer",trace.policy_steer},{"preview_seed_steer",trace.preview_seed_steer},
            {"command",{cmd.steer,cmd.speed}},{"mode",int(result.mode)},{"reason",result.reason},{"ready",router.lap_plan().ready()},
            {"first_empty",trace.first_empty},{"wide",trace.wide},{"planned_offsets",trace.planned_offsets},{"gate",{trace.gate_entry,trace.gate_exit,trace.gate_offset}},{"subdivisions",trace.subdivisions},{"scan",sim.scan.ranges}};
          if(result.mode==Mode::Emergency && first_rejection.is_null()){
            tick["occupancy"]=memory.obstacle_cells();tick["waypoint"]=trace.waypoint;
            tick["subdivisions"]=trace.subdivisions;tick["free_columns"]=trace.fine_free;
            tick["reachable_columns"]=trace.fine_reachable;
          }
          ticks_log<<tick.dump()<<'\n';
        }
        if(result.mode==Mode::Emergency)save_before_failure();
        if(trace_avoid && router.lap_plan().ready() && (result.mode==Mode::Avoid ||
           (result.mode==Mode::Emergency && first_rejection.is_null()))) {
          const auto& w=track.points[track.closest(car)];
          avoid_trace.push_back({{"s",progress},{"v",car.v},{"steer",car.steer},{"target_steer",cmd.steer},
            {"target_v",cmd.speed},{"lateral",-(car.x-w.x)*sinf(w.yaw)+(car.y-w.y)*cosf(w.yaw)},
            {"yaw_error",wrap(car.yaw-w.yaw)},{"reason",result.reason}});
        }
        if(result.mode==Mode::Emergency && first_rejection.is_null())
          first_rejection={{"reason",result.reason},{"speed_mps",car.v},{"progress_m",progress},
                           {"router_avoiding",router.avoiding()},{"previous_mode",int(previous_mode)}};
        if(result.mode==Mode::Avoid){
          ++observed_avoid_steps;
          if(result.avoid_policy_verified)++policy_verified_steps;
          if(std::abs(cmd.steer-result.avoid_policy_steer)<=1e-6f)++policy_match_steps;
          if(entry_speed<0)entry_speed=car.v;
          peak_avoid_speed=std::max(peak_avoid_speed,car.v);
          avoid_speed_sum+=car.v;avoid_command_sum+=cmd.speed;++avoid_steps;
        }
        if(result.mode!=previous_mode){
          events.push_back({{"time_s",sim.stamp},{"progress_m",progress},{"passed",passed},{"lap",lap+1},{"side",router.side()},{"reason",result.reason},{"mode",int(result.mode)}});
          if(result.mode==Mode::Avoid && progress-lap*length<stations.front()) started_before=true;
          if(result.mode==Mode::Race){if(passed<count)early_return=true;else returned_after=true;}
        }
        if(result.mode==Mode::Avoid && previous_mode==Mode::Avoid)++hold_steps;
        previous_mode=result.mode;
      }else{
        auto input=observe(scans,car,memory,track,1,0);
        if(policy.uses_corridor()){
          if(!replay_corridor.plan(track,track.closest(car),car,memory)){reason="no_corridor";break;}
          replay_corridor.condition(input,car);
        }
        cmd=policy.infer(input);
        if(!std::isfinite(cmd.steer)||!std::isfinite(cmd.speed))throw std::runtime_error("nonfinite action");
        cmd={clamp(cmd.steer,-.4,.4),clamp(cmd.speed,0,cap)};
        if(policy.uses_corridor())cmd.speed=replay_corridor.speed_limit(cmd.speed,car.v);
      }
      previous=cmd;
    } // decision already stored in a restored snapshot
    resumed=false;
    const bool capture_now=manual ? progress>=checkpoint.at("at_m").get<double>() :
        progress>=checkpoint.value("from_m",0.) && (recent.empty() || sim.stamp-recent.back().first>=.24);
    if(!output.empty() && !written && previous_mode!=Mode::Emergency && capture_now){
      json data={{"version",1},{"identity",identity}};
      transfer(data,false);
      if(manual){write_snapshot(output,data);written=true;}
      else{
        recent.emplace_back(sim.stamp,std::move(data));
        while(recent.size()>1 && recent[1].first<=sim.stamp-lookback)recent.pop_front();
      }
    }
    previous_interval_gap=INFINITY;
    float pre_step_speed=0;
    for(int j=0;j<physics_ticks;++j){
      pre_step_speed=basic(sim.state).v;
      sim.step(cmd);
      peak_speed=std::max(peak_speed,static_cast<double>(basic(sim.state).v));
      peak_command=std::max(peak_command,static_cast<double>(cmd.speed));
      if(progress>.5)min_moving_speed=std::min(min_moving_speed,basic(sim.state).v);
      for(auto& box:boxes)previous_interval_gap=std::min(previous_interval_gap,double(separation(sim.state,box)));
      min_gap=std::min(min_gap,previous_interval_gap);
      if(sim.collision||min_gap<=0||(horizon>0 && sim.stamp+1e-8>=horizon))break;
    }
    if(previous_mode==Mode::Avoid)avoid_seconds+=sim.stamp-interval_start;
    if(min_gap<=0||sim.collision){
      std::string contact_kind=sim.collision ? "gym_unknown" : "rectangle_overlap";
      if(sim.collision && PyObject_HasAttrString(sim.car.p,"collision_kind")){
        auto kind=attr(sim.car.p,"collision_kind");
        if(PyUnicode_Check(kind.p))contact_kind=PyUnicode_AsUTF8(kind.p);
      }
      const auto hit_state=basic(sim.state);
      double hit_progress=progress+route_delta(project_route(track,hit_state).s,last,length);
      contact={{"v",hit_state.v},{"pre_step_v",pre_step_speed},{"s",hit_progress},{"t",sim.stamp},
               {"kind",contact_kind},{"gym_collision",sim.collision},
               {"sat_gap",min_gap},{"lap",lap+1}};
      reason="collision";save_before_failure();break;
    }
    auto s=project_route(track,basic(sim.state)).s;
    double previous_progress=progress;
    progress+=route_delta(s,last,length);last=s;
    double lap_progress=progress-lap*length;
    while(obstacle_speeds.size()<size_t(count*(lap+1)) && lap_progress>=stations[obstacle_speeds.size()%count]){
      const auto index=obstacle_speeds.size()%count;
      crossings.push_back({{"lap",lap+1},{"index",index},{"time_s",crossing_time(lap*length+stations[index],previous_progress,progress,interval_start,sim.stamp)}});
      obstacle_speeds.push_back(basic(sim.state).v);
    }
    while(passed<count&&lap_progress>stations[passed]+.6)++passed;
    if(race && !taking_over && previous_mode==Mode::Emergency && fabs(basic(sim.state).v)<.05){reason="stopped";break;}
    if(timed_race && min_moving_speed<.05f){reason="stopped";break;}
    if(!ros_finish && race && lap_progress>stations.back()+8 && !returned_after){reason="no_return";break;}
    if(full_lap && lap_progress>=length){
      if(passed!=count || (!ros_finish && (!returned_after || previous_mode!=Mode::Race || early_return || !started_before))){
        reason="incomplete_traversal";break;
      }
      double crossing=crossing_time((lap+1)*length,previous_progress,progress,interval_start,sim.stamp);
      lap_results.push_back({{"lap",lap+1},{"seconds",crossing-previous_cross},{"passed",passed},
        {"planned_entries",router.lap_plan().entries()},{"brake_steps",router.lap_plan().brake_steps()},
        {"recorded_sections",router.lap_plan().sections()}});
      previous_cross=crossing;
      ++lap;
      if(!timed_race && lap==laps){reason="completed";break;}
      passed=0;returned_after=false;started_before=false;early_return=false;
    }else if(!full_lap && (!race || (returned_after && previous_mode==Mode::Race)) && progress>stations.back()+2){
      reason="completed";break;
    }
    if(horizon>0 && sim.stamp+1e-8>=horizon){reason="time_limit";break;}
    if(branch_ticks && step-start_step>=branch_ticks){reason="branch_horizon";break;}
  }
  bool recovery_ok=recovery && taking_over && passed==count && reason=="completed" && records.size()>=3;
  if(!save.empty() && (recovery?recovery_ok:(reason=="completed" && visible_frames>=3))){
    std::string name=spec.at("name");
    json meta={{"corridor_guided",recovery || !replay.empty()},{"handoff",handoff},{"teacher_steps",teacher_steps},{"no_guide_steps",no_guide},{"name",name},{"status","accepted"},{"teacher","SVG-MPPI"},{"lidar_range_m",18},{"records",records.size()},
      {"joint_frames",visible_frames},{"boxes",boxes},{"scenario",spec}};
    DatasetWriter writer(save+"/"+name+".elds",meta.dump(2));for(auto& r:records)writer.append(r);writer.flush();
  }
  if(spec.contains("scout_route_trace")){
    std::ofstream out(spec.at("scout_route_trace").get<std::string>());
    out.exceptions(std::ios::failbit|std::ios::badbit);
    out<<OfflineSnapshot::scout_route(router.lap_plan())<<'\n';
  }
  return {{"guide_counts",{{"cluster_checks",cluster_checks},{"cluster_changes",cluster_changes},{"exit_checks",exit_checks},{"exit_changes",exit_changes}}},{"whole_lap_path_status",router.lap_plan().route_status()},{"whole_lap_path_ready",router.lap_plan().has_route()},{"configured_avoid_upper",router_settings.sim_avoid_max},{"steering_metrics",{{"avoid_steps",observed_avoid_steps},{"verified_steps",policy_verified_steps},
    {"output_match_steps",policy_match_steps},{"avoid_seconds",avoid_seconds}}},
    {"obstacle_crossings",crossings},{"slow_controls",slow_controls},{"worst_control",worst_control},{"watched_control",watched_control},{"control_wall",{{"samples",control_samples},{"over_25ms",over_25ms},{"max_ms",worst_ms}}},{"peak_speed_mps",peak_speed},{"peak_command_mps",peak_command},{"pose_source",sensor_only?"lidar_static_map":"training_or_diagnostic_simulator"},{"control_ms",control_dt*1000},{"snapshot_written",written},{"branch_entry",entry},{"contact",contact},{"avoid_trace",avoid_trace},{"laps",lap_results},{"first_rejection",first_rejection},{"avoid_entry_mps",entry_speed},{"peak_avoid_mps",peak_avoid_speed},{"mean_avoid_mps",avoid_speed_sum/std::max(1,avoid_steps)},{"mean_avoid_command_mps",avoid_command_sum/std::max(1,avoid_steps)},{"at_obstacles_mps",obstacle_speeds},{"min_speed_after_first_half_m",min_moving_speed},{"avoid_speed_cap_mps",router_settings.avoid_speed_cap},{"learner_switch",learner_switch},{"family",family},{"handoff",handoff},{"accepted_recovery",recovery_ok},{"teacher_steps",teacher_steps},{"no_guide_steps",no_guide},{"labeled_frames",records.size()},{"first_hidden_at_reset",hidden},{"joint_frames",visible_frames},{"spec",spec},{"map",map},{"reverse",bool(layout%2)},{"layout",layout},{"boxes",boxes},{"stations_m",stations},{"seed",seed},{"events",events},{"hold_steps",hold_steps},{"started_before",started_before},{"early_return",early_return},{"returned_after",returned_after},{"final_speed",basic(sim.state).v},{"controller",recovery?"offline_recovery":race?"section_router":baseline?"svg_mppi":"avoid_tln"},{"passed",passed},{"reason",reason},{"progress_m",progress},{"min_sat_gap_m",min_gap},{"time_s",sim.stamp}};
}
int main(int argc,char** argv){try{
  if(argc<3||argc>5)throw std::runtime_error("expected checkpoint and output jsonl");
  if(std::filesystem::exists(argv[2]))throw std::runtime_error("output exists");
  Py_Initialize();
  json batch;
  if(argc>=4){std::ifstream input(argv[3]);input>>batch;}
  const auto device_name=batch.is_object()?batch.value("device",std::string("cuda")):std::string("cuda");
  if(device_name!="cuda" && device_name!="cpu")throw std::runtime_error("unknown inference device");
  const auto device=device_name=="cpu"?InferenceDevice::cpu:InferenceDevice::cuda;
  Policy policy(argv[1],device);json cfg;std::ifstream("/workspace/config/teachers_expanded_v5.json")>>cfg;
  std::ofstream out(argv[2]);
  if(argc>=4){

    bool recovery=batch.value("recovery",false);
    if(recovery && (argc!=5 || !batch.contains("save") || batch.value("teacher",false) || batch.contains("replay")))
      throw std::runtime_error("offline recovery requires learner checkpoints and output directory");
    std::unique_ptr<Policy> race;
    if(argc==5){
      if(batch.value("teacher",false)||(batch.contains("save") && !recovery)) throw std::runtime_error("router evaluation requires inference-only batch");
      race=std::make_unique<Policy>(argv[4],device);
      if(race->uses_corridor())throw std::runtime_error("Race requires an unconditioned checkpoint");
    }
    Policy* race_actor=race.get();
    if(policy.is_unified()){
      if(race_actor || recovery || batch.value("teacher",false) || batch.contains("save") || batch.contains("prefix_checkpoint"))
        throw std::runtime_error("unified evaluation requires one checkpoint and inference-only cases");
      race_actor=&policy;
    }
    std::unique_ptr<Policy> prefix;
    if(batch.contains("prefix_checkpoint")){
      if(!race || recovery || batch.value("teacher",false))throw std::runtime_error("prefix only supported in learner evaluation");
      prefix=std::make_unique<Policy>(batch.at("prefix_checkpoint").get<std::string>(),device);
    }
    for(auto& spec:batch.at("cases")){
      auto row=run(spec.at("map"),spec.at("layout"),spec.at("count"),policy,batch.at("teacher"),cfg,spec.value("original",false)?json::object():spec,batch.value("save",std::string{}),race_actor,batch.value("replay",std::string{}),recovery,prefix.get(),{{"device",device_name},{"files",argc==5?json::array({argv[1],argv[4]}):json::array({argv[1]})}});
      row["case"]=spec.at("name");out<<row.dump()<<std::endl;
    }
    return 0;
  }
  for(bool baseline:{false,true})for(int layout=0;layout<4;++layout)for(int n:{1,3})
    out<<run("berlin",layout,n,policy,baseline,cfg).dump()<<std::endl;
}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}}
