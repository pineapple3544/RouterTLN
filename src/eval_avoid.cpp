#include "avoid_support.hpp"
#include "el/map_paths.hpp"
#include "el/eval_metrics.hpp"
#include "el/section_router.hpp"
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
json run(const std::string& map,int layout,int count,Policy& policy,bool baseline,const json& settings,const json& spec=json::object(),const std::string& save="",Policy* race=nullptr,const std::string& replay="",bool recovery=false,Policy* prefix=nullptr){
  std::string base = map == "berlin" ? el::default_map_dir : "/home/misys/forza_ws/race_stack/stack_master/maps/"+map;
  auto stem=el::map_stem(base);
  auto router_settings=el::map_router_settings(base);
  router_settings.lap_planning=spec.value("lap_planning",false);
  Track track(base+"/global_waypoints.json");
  orient_track(track,layout%2);
  const float cap=spec.value("speed_cap",3.f);
  const bool router_lap=spec.value("full_lap",false) && race && !recovery;
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
  router_settings.validate_sim_band(cap,race && spec.value("full_lap",false) && !recovery);
  const int laps=spec.value("laps",1);
  const bool scout_then_cruise=spec.value("scout_then_cruise",false);
  if(laps<1 || laps>20 || (laps>1 && (!race || !spec.value("full_lap",false) || recovery || prefix)))throw std::runtime_error("invalid multi-lap evaluation");
  speed_profile(track,cap);
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
  for(int i=0;i<count;++i){
    double station=spec.empty()?5.73+i*spacing:spec.at("stations").at(i).get<double>();
    double s=fmod(pose.s+station,length);
    auto it=std::min_element(track.points.begin(),track.points.end(),[s](auto a,auto b){return fabs(a.s-s)<fabs(b.s-s);});
    float d=(i%2?-1:1)*(layout%2?-1:1)*(.083f+.019f*layout);
    if(!spec.empty())d=spec.at("offsets").at(i);
    float x=it->x-sinf(it->yaw)*d,y=it->y+cosf(it->yaw)*d;
    auto dims=spec.contains("shapes")?spec.at("shapes").at(i):json::array({.27,spec.value("box_width",.245),0});
    float box_yaw=it->yaw+dims[2].get<float>(),len=dims[0],width=dims[1];
    float radius=.5f*hypotf(len,width);
    if(!memory.base_free_disc(x,y,spec.contains("shapes")?radius:.19f))return {{"reason","invalid_box"},{"spec",spec}};
    if(spec.contains("shapes"))boxes.push_back({x,y,box_yaw,len,width});
    else boxes.push_back({x,y,it->yaw,.27,spec.value("box_width",.245)});
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
  std::vector<float> obstacle_speeds;
  int passed=0;std::string reason="timeout";
  float min_moving_speed=std::numeric_limits<float>::infinity();
  bool full_lap=spec.value("full_lap",false);
  if(full_lap && (!race || recovery))throw std::runtime_error("full lap requires router evaluation");
  json lap_results=json::array();
  json contact=nullptr;
  const bool trace_avoid=spec.value("trace_avoid",false);
  json avoid_trace=json::array();
  double previous_cross=0;
  int lap=0;
  for(int step=0;step<(full_lap?2400*laps:800);++step){
    memory.update(sim.scan);scans.push_back(sim.scan);if(scans.size()>history)scans.pop_front();
    if(count>=2 && progress<stations[0]-.35 && jointly_visible(sim,boxes))++visible_frames;
    Control cmd;
    auto car=basic(sim.state);
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
      auto result=router.step(track,memory,car,scans,*race,actor,(lap==0 && scout_then_cruise)?std::min(cap,router_settings.scout_speed_cap):cap,.05f,sim.stamp,router_settings);
      cmd=result.command;
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
      auto car=basic(sim.state);
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
    float pre_step_speed=0;
    for(int j=0;j<5;++j){
      pre_step_speed=basic(sim.state).v;
      sim.step(cmd);
      if(progress>.5)min_moving_speed=std::min(min_moving_speed,basic(sim.state).v);
      for(auto& box:boxes)min_gap=std::min(min_gap,double(separation(sim.state,box)));
      if(sim.collision||min_gap<=0)break;
    }
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
      reason="collision";break;
    }
    auto s=project_route(track,basic(sim.state)).s;
    double previous_progress=progress;
    progress+=route_delta(s,last,length);last=s;
    double lap_progress=progress-lap*length;
    while(obstacle_speeds.size()<size_t(count*(lap+1)) && lap_progress>=stations[obstacle_speeds.size()%count])
      obstacle_speeds.push_back(basic(sim.state).v);
    while(passed<count&&lap_progress>stations[passed]+.6)++passed;
    if(race && !taking_over && previous_mode==Mode::Emergency && fabs(basic(sim.state).v)<.05){reason="stopped";break;}
    if(race && lap_progress>stations.back()+8 && !returned_after){reason="no_return";break;}
    if(full_lap && lap_progress>=length){
      if(passed!=count || !returned_after || previous_mode!=Mode::Race || early_return || !started_before){
        reason="incomplete_traversal";break;
      }
      double crossing=crossing_time((lap+1)*length,previous_progress,progress,sim.stamp-.05,sim.stamp);
      lap_results.push_back({{"lap",lap+1},{"seconds",crossing-previous_cross},{"passed",passed},
        {"planned_entries",router.lap_plan().entries()},{"brake_steps",router.lap_plan().brake_steps()},
        {"recorded_sections",router.lap_plan().sections()}});
      previous_cross=crossing;
      if(++lap==laps){reason="completed";break;}
      passed=0;returned_after=false;started_before=false;early_return=false;
    }else if(!full_lap && (!race || (returned_after && previous_mode==Mode::Race)) && progress>stations.back()+2){
      reason="completed";break;
    }
  }
  bool recovery_ok=recovery && taking_over && passed==count && reason=="completed" && records.size()>=3;
  if(!save.empty() && (recovery?recovery_ok:(reason=="completed" && visible_frames>=3))){
    std::string name=spec.at("name");
    json meta={{"corridor_guided",recovery || !replay.empty()},{"handoff",handoff},{"teacher_steps",teacher_steps},{"no_guide_steps",no_guide},{"name",name},{"status","accepted"},{"teacher","SVG-MPPI"},{"lidar_range_m",18},{"records",records.size()},
      {"joint_frames",visible_frames},{"boxes",boxes},{"scenario",spec}};
    DatasetWriter writer(save+"/"+name+".elds",meta.dump(2));for(auto& r:records)writer.append(r);writer.flush();
  }
  return {{"contact",contact},{"avoid_trace",avoid_trace},{"laps",lap_results},{"first_rejection",first_rejection},{"avoid_entry_mps",entry_speed},{"peak_avoid_mps",peak_avoid_speed},{"mean_avoid_mps",avoid_speed_sum/std::max(1,avoid_steps)},{"mean_avoid_command_mps",avoid_command_sum/std::max(1,avoid_steps)},{"at_obstacles_mps",obstacle_speeds},{"min_speed_after_first_half_m",min_moving_speed},{"avoid_speed_cap_mps",router_settings.avoid_speed_cap},{"learner_switch",learner_switch},{"family",family},{"handoff",handoff},{"accepted_recovery",recovery_ok},{"teacher_steps",teacher_steps},{"no_guide_steps",no_guide},{"labeled_frames",records.size()},{"first_hidden_at_reset",hidden},{"joint_frames",visible_frames},{"spec",spec},{"map",map},{"reverse",bool(layout%2)},{"layout",layout},{"boxes",boxes},{"stations_m",stations},{"seed",seed},{"events",events},{"hold_steps",hold_steps},{"started_before",started_before},{"early_return",early_return},{"returned_after",returned_after},{"final_speed",basic(sim.state).v},{"controller",recovery?"offline_recovery":race?"section_router":baseline?"svg_mppi":"avoid_tln"},{"passed",passed},{"reason",reason},{"progress_m",progress},{"min_sat_gap_m",min_gap},{"time_s",sim.stamp}};
}
int main(int argc,char** argv){try{
  if(argc<3||argc>5)throw std::runtime_error("expected checkpoint and output jsonl");
  if(std::filesystem::exists(argv[2]))throw std::runtime_error("output exists");
  Py_Initialize();Policy policy(argv[1]);json cfg;std::ifstream("/workspace/config/teachers_expanded_v5.json")>>cfg;
  std::ofstream out(argv[2]);
  if(argc>=4){
    json batch;std::ifstream(argv[3])>>batch;
    bool recovery=batch.value("recovery",false);
    if(recovery && (argc!=5 || !batch.contains("save") || batch.value("teacher",false) || batch.contains("replay")))
      throw std::runtime_error("offline recovery requires learner checkpoints and output directory");
    std::unique_ptr<Policy> race;
    if(argc==5){
      if(batch.value("teacher",false)||(batch.contains("save") && !recovery)) throw std::runtime_error("router evaluation requires inference-only batch");
      race=std::make_unique<Policy>(argv[4]);
      if(race->uses_corridor())throw std::runtime_error("Race requires an unconditioned checkpoint");
    }
    std::unique_ptr<Policy> prefix;
    if(batch.contains("prefix_checkpoint")){
      if(!race || recovery || batch.value("teacher",false))throw std::runtime_error("prefix only supported in learner evaluation");
      prefix=std::make_unique<Policy>(batch.at("prefix_checkpoint").get<std::string>());
    }
    for(auto& spec:batch.at("cases")){
      auto row=run(spec.at("map"),spec.at("layout"),spec.at("count"),policy,batch.at("teacher"),cfg,spec.value("original",false)?json::object():spec,batch.value("save",std::string{}),race.get(),batch.value("replay",std::string{}),recovery,prefix.get());
      row["case"]=spec.at("name");out<<row.dump()<<std::endl;
    }
    return 0;
  }
  for(bool baseline:{false,true})for(int layout=0;layout<4;++layout)for(int n:{1,3})
    out<<run("berlin",layout,n,policy,baseline,cfg).dump()<<std::endl;
}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}}
