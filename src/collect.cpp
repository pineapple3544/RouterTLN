#include "avoid_support.hpp"
struct Cycler {
  json events=json::array();int passed=0,spawned=0;float pass_progress=0;std::set<int> used_indices;bool rear_only=false;
  std::string profile="legacy",region="any",placement_mode="mixed";int lateral_bias=0;
  float reference_yaw=0,closest_distance=1e9,closest_lateral=0,closest_speed=0;bool nominal_blocked=false;
  void update(Sim& sim,json& obs,const Track& track,const Memory& memory,std::mt19937& rng,float progress,float length,int step){
    auto st=basic(sim.state);float sx=st.x+.27f*cosf(st.yaw),sy=st.y+.27f*sinf(st.yaw);
    if(!obs.empty()){
      float dx=obs[0][0].get<float>()-st.x,dy=obs[0][1].get<float>()-st.y;
      float distance=hypotf(dx,dy);if(distance<closest_distance){closest_distance=distance;closest_lateral=sinf(reference_yaw)*dx-cosf(reference_yaw)*dy;closest_speed=st.v;}
      if(progress>pass_progress+1.5f&&hypotf(dx,dy)>2.f&&cosf(st.yaw)*dx+sinf(st.yaw)*dy<-.5f){
        events.push_back({{"type","remove"},{"step",step},{"time",sim.stamp},{"progress_m",progress},{"obstacle",obs[0]},{"passed_id",spawned},{"passing_side",closest_lateral>.05f?"left":closest_lateral<-.05f?"right":"ambiguous"},{"lateral_at_closest_m",closest_lateral},{"closest_center_distance_m",closest_distance},{"speed_at_closest_mps",closest_speed},{"nominal_route_blocked",nominal_blocked}});
        obs=json::array();sim.obstacles(obs);++passed;
      }else return;
    }
    // Try at 5 Hz; never force placement if all distant candidates are visible.
    if(step%4)return;
    int n=track.points.size(),current=track.closest(st);std::uniform_int_distribution<int> pick(0,n-1);
    auto shape=obstacle_shape(profile);std::uniform_real_distribution<float> length_rng(shape.min_length,shape.max_length),width_rng(shape.min_width,shape.max_width),rotation(-shape.max_rotation,shape.max_rotation);
    std::bernoulli_distribution exact_waypoint(placement_mode=="on_waypoint"?1.:placement_mode=="jittered"?0.:.30);
    std::normal_distribution<float> lateral_noise(0,.18f),longitudinal_noise(0,.12f);
    bool prefer_rear=rear_only||(spawned%2)==1;
    for(int attempt=0;attempt<160;++attempt){int idx=pick(rng);if(used_indices.count(idx/5))continue;float curvature=route_curvature(track,idx);auto actual_region=road_region(curvature);if(region!="any"&&region!=actual_region)continue;float ahead=((idx-current+n)%n)*length/n;
      if(ahead<8||ahead>length-5)continue;auto w=track.points[idx];bool exact=exact_waypoint(rng);
      float off=exact?0:clamp(lateral_noise(rng),-.4f,.4f),along=exact?0:clamp(longitudinal_noise(rng),-.25f,.25f);
      if(!exact&&lateral_bias)off=lateral_bias*fabsf(off);
      float len=length_rng(rng),width=width_rng(rng),yaw=w.yaw+rotation(rng);
      float x=w.x-sinf(w.yaw)*off+cosf(w.yaw)*along,y=w.y+cosf(w.yaw)*off+sinf(w.yaw)*along,radius=.5f*hypotf(len,width)+.10f,distance=hypotf(x-sx,y-sy);
      if(distance<6+radius||!memory.base_free_disc(x,y,radius))continue;
      float angular_radius=asinf(radius/distance),angle=fabsf(wrap(atan2f(y-sy,x-sx)-st.yaw));
      bool rear=angle-angular_radius>2.35f+.12f&&length-ahead>=6&&length-ahead<=18;
      if(rear){int behind=(current-idx+n)%n;for(int j=0;j<=behind;++j)if(fabsf(wrap(track.points[(idx+j)%n].yaw-w.yaw))>.2f){rear=false;break;}}
      bool hidden=distance>=8+radius&&memory.wall_hides_disc(sx,sy,x,y,radius);
      if(rear_only&&!rear)continue;
      if(!rear&&!hidden)continue;if(attempt<100&&prefer_rear&&!rear)continue;if(attempt<100&&!prefer_rear&&!hidden)continue;
      json candidate={x,y,yaw,len,width};if(sim.candidate_visible_beams(candidate)!=0)continue;
      obs.push_back(candidate);sim.obstacles(obs);++spawned;used_indices.insert(idx/5);pass_progress=progress+ahead+along;
      reference_yaw=w.yaw;closest_distance=1e9;closest_lateral=closest_speed=0;nominal_blocked=false;
      for(int j=-8;j<=8;++j){auto ref=track.points[(idx+n+j)%n];DynamicState pose;pose.z[0]=ref.x;pose.z[1]=ref.y;pose.z[4]=ref.yaw;nominal_blocked|=separation(pose,candidate)<=0;}
      events.push_back({{"type","spawn"},{"id",spawned},{"step",step},{"time",sim.stamp},{"progress_m",progress},{"pass_progress_m",pass_progress},{"index",idx},{"obstacle",candidate},
        {"placement",exact?"on_waypoint":"jittered"},{"lateral_offset_m",off},{"longitudinal_offset_m",along},
        {"profile",profile},{"road_region",actual_region},{"route_curvature",curvature},{"nominal_route_blocked",nominal_blocked},
        {"mode",rear&&prefer_rear?"rear_blind_straight":hidden?"wall_occluded":"rear_blind_straight"},{"wall_occluded",hidden},{"rear_blind_straight",rear},{"sensor_distance_m",distance},{"required_min_distance_m",rear&&prefer_rear?6+radius:hidden?8+radius:6+radius},{"visible_beams",0}});
      std::cout<<"spawn id="<<spawned<<" index="<<idx<<" distance="<<distance<<" wall="<<hidden<<" rear="<<rear<<std::endl;return;
    }
  }
};
int main(int argc,char** argv){try{
  if(argc!=2)throw std::runtime_error("Usage: el_collect CONFIG.json");std::ifstream input(argv[1]);json cfg;input>>cfg;
  auto default_base=cfg.value("map_dir",std::string("/home/misys/forza_ws/race_stack/stack_master/maps/hyhall12"));
  auto output=cfg.at("output").get<std::string>();if(std::filesystem::exists(output+"/summary.json"))throw std::runtime_error("collection already exists");
  std::filesystem::create_directories(output);Py_Initialize();json results=json::array();
  for(auto spec:cfg.at("episodes")){
    auto base=spec.value("map_dir",default_base),map_stem=spec.value("map_stem",cfg.value("map_stem",std::string("hyhall12_orig")));
    int seed=spec.at("seed"),mode=spec.at("mode")=="race"?0:1;float cap=spec.value("speed_cap",2.5f);if(cap<=0||cap>10)throw std::runtime_error("speed cap outside (0,10]");
    Track track(base+"/global_waypoints.json");orient_track(track,spec.value("reverse_track",false));
    float length=0;for(size_t i=0;i<track.points.size();++i)length+=hypotf(track.points[i].x-track.points[(i+1)%track.points.size()].x,track.points[i].y-track.points[(i+1)%track.points.size()].y);
    speed_profile(track,cap);int start=spec.value("start_index",0)%track.points.size();float lidar_range=cfg.value("lidar_range_m",18.f);Sim sim(base+"/"+map_stem,seed,lidar_range);Memory memory(base+"/"+map_stem+".yaml");
    json obstacles=json::array();std::mt19937 rng(seed);std::uniform_real_distribution<float> offset(-.25,.25);
    for(auto idx:spec.value("obstacle_indices",std::vector<int>{})){auto w=track.points.at(idx%track.points.size());float d=offset(rng),size=spec.value("obstacle_size",.25f);obstacles.push_back({w.x-sinf(w.yaw)*d,w.y+cosf(w.yaw)*d,w.yaw,size,size});}
    bool cycling=spec.value("cycling_obstacles",false);
    if(mode==0&&(!obstacles.empty()||cycling))throw std::runtime_error("race episode must be obstacle free");if(mode==1&&obstacles.empty()&&!cycling)throw std::runtime_error("avoid episode needs obstacles");
    auto start_pose=track.points[start];float start_offset=spec.value("start_lateral_m",0.f),start_yaw=spec.value("start_yaw_rad",0.f);
    if(fabs(start_offset)>.3f||fabs(start_yaw)>.25f)throw std::runtime_error("start perturbation exceeds collection bounds");
    start_pose.x-=sinf(start_pose.yaw)*start_offset;start_pose.y+=cosf(start_pose.yaw)*start_offset;start_pose.yaw=wrap(start_pose.yaw+start_yaw);
    sim.obstacles(obstacles);sim.reset(start_pose);TeacherConfig tc;tc.kind=mode?TeacherKind::Svg:TeacherKind::Smppi;tc.seed=seed;
    tc.samples=cfg.value("samples",1024);tc.horizon=cfg.value("horizon",40);tc.iterations=cfg.value("iterations",2);tc.guide_iterations=cfg.value("guide_iterations",6);tc.guide_samples=cfg.value("guide_samples",128);
    tc.temperature=cfg.value("temperature",1.f);tc.steer_sigma=cfg.value("steer_sigma",.10f);tc.speed_sigma=cfg.value("speed_sigma",.8f);
    Teacher teacher(tc);std::deque<Scan> history_scans;std::vector<Record> records;Control previous;float progress=0,peak_speed=0,sum_ms=0,max_error=0;int last=track.closest(basic(sim.state)),invalid=0;
    Cycler cycler;cycler.rear_only=spec.value("rear_only",false);cycler.profile=spec.value("obstacle_profile",std::string("legacy"));cycler.region=spec.value("road_region",std::string("any"));cycler.placement_mode=spec.value("placement_mode",std::string("mixed"));cycler.lateral_bias=spec.value("lateral_bias",0);
    obstacle_shape(cycler.profile);if(cycler.region!="any"&&cycler.region!="straight"&&cycler.region!="transition"&&cycler.region!="corner")throw std::runtime_error("invalid road region");
    if(cycler.placement_mode!="mixed"&&cycler.placement_mode!="on_waypoint"&&cycler.placement_mode!="jittered")throw std::runtime_error("invalid placement mode");
    if(abs(cycler.lateral_bias)>1)throw std::runtime_error("invalid lateral bias");
    std::vector<size_t> boundaries;struct StepStats{float progress,peak,ms,gap,heading;};std::vector<StepStats> stats;
    int maxsteps=spec.value("max_steps",1800);float target=length*spec.value("laps",1.f),min_separation=1e9;bool overlap=false;std::string reason="timeout";
    bool metrics_only=cfg.value("metrics_only",false);std::array<int,5> sources{};
    std::string name=spec.at("name"),tracepath=output+"/"+name+".csv";std::ofstream trace(tracepath);trace<<"step,time,x,y,speed,steer,label_steer,label_speed,valid,collision,progress,teacher_ms,source,ess\n";
    for(int step=0;step<maxsteps;++step){if(cycling){int previous_passes=cycler.passed;cycler.update(sim,obstacles,track,memory,rng,progress,length,step);if(cycler.passed>previous_passes)boundaries.push_back(records.size());}
      memory.update(sim.scan);history_scans.push_back(sim.scan);while(history_scans.size()>history)history_scans.pop_front();
      auto p=problem(track,memory,sim.state,cap,previous);if(mode)local_route(p);auto plan=teacher.solve(p,initial(p,tc.horizon));
      ++sources.at(plan.source);
      if(!plan.valid){std::ofstream dump(output+"/"+name+"-failure.problem",std::ios::binary);dump.write(reinterpret_cast<const char*>(&p),sizeof(p));}
      Record r;r.step=step;r.stamp=sim.stamp;r.mode=mode;r.valid=plan.valid;r.label=plan.action;r.executed=plan.action;
      r.observation=observe(history_scans,basic(sim.state),memory,track,mode,0);if(!plan.valid)++invalid;
      auto before=sim.state;float step_peak=0,step_gap=1e9;
      for(int j=0;j<5;++j){auto predicted=physics_step(sim.state,plan.action,p.vehicle);sim.step(plan.action);
        if(!sim.collision){for(int k=0;k<7;++k){float error=k==4?fabsf(wrap(predicted.z[k]-sim.state.z[k])):fabsf(predicted.z[k]-sim.state.z[k]);max_error=std::max(max_error,error);}if(max_error>.003f)throw std::runtime_error("dynamic model differs from simulator > .003; refusing labels");}
        for(auto& o:obstacles){float gap=separation(sim.state,o);step_gap=std::min(step_gap,gap);min_separation=std::min(min_separation,gap);overlap|=gap<=0;}
        step_peak=std::max(step_peak,sim.state.z[3]);peak_speed=std::max(peak_speed,sim.state.z[3]);if(sim.collision||overlap)break;}
      r.collision=sim.collision||overlap;if(r.collision)r.valid=0;records.push_back(r);sum_ms+=plan.compute_ms;previous=plan.action;
      int next=track.closest(basic(sim.state)),delta=next-last,n=track.points.size();if(delta>n/2)delta-=n;if(delta<-n/2)delta+=n;progress+=delta*length/n;last=next;
      stats.push_back({progress,step_peak,plan.compute_ms,step_gap,sim.state.z[4]});
      trace<<step<<','<<sim.stamp<<','<<before.z[0]<<','<<before.z[1]<<','<<before.z[3]<<','<<before.z[2]<<','<<plan.action.steer<<','<<plan.action.speed<<','<<r.valid<<','<<r.collision<<','<<progress<<','<<plan.compute_ms<<','<<plan.source<<','<<plan.effective_samples<<'\n';
      if(step%100==0)std::cout<<name<<" step="<<step<<" progress="<<progress<<" speed="<<sim.state.z[3]<<" valid="<<plan.valid<<" ms="<<plan.compute_ms<<std::endl;
      if(sim.collision||overlap){reason="collision";break;}if(invalid>0){reason="no_feasible_plan";break;}
      if(cycling&&cycler.passed>=spec.value("target_encounters",100000)){reason="completed";break;}if(progress>=target){reason="completed";break;}
    }
    if(reason=="completed"&&cycling&&cycler.passed<spec.value("minimum_passes",2))reason="insufficient_obstacle_passes";
    bool accepted=reason=="completed"&&invalid==0;json meta={{"name",name},{"teacher",mode?"SVG-MPPI":"SMPPI"},{"implementation","bounded dynamic C++/CUDA adaptation v2"},{"status",accepted?"accepted":"quarantine"},{"reason",reason},{"records",records.size()},{"progress_m",progress},{"laps",progress/length},{"speed_cap_mps",cap},{"peak_actual_speed_mps",peak_speed},{"collisions",sim.collision||overlap?1:0},{"invalid_plans",invalid},{"mean_teacher_ms",sum_ms/std::max(size_t(1),records.size())},{"max_model_error",max_error},{"seed",seed},{"scenario",spec},{"obstacles",obstacles},{"simulator_module",sim.module_path},{"physics_dt",.01},{"control_dt",.05},{"synchronous",true},{"obstacle_input","LiDAR memory only; scenario coordinates excluded from controller"},{"label_timing","observation before action, collision after action"},{"teacher_config",cfg},{"privileged_teacher_state","GT pose, velocity, yaw rate, slip angle, steering delay; student inputs are ELDS v1 observations"}};
    meta["lidar_range_m"]=lidar_range;meta["obstacle_events"]=cycler.events;meta["obstacles_passed"]=cycler.passed;meta["obstacles_spawned"]=cycler.spawned;
    meta["implementation"]="teacher improvement v5: stable world grid and exact sampled ray angles";meta["plan_sources_weighted_warm_nominal_guide_sample"]=sources;meta["metrics_only"]=metrics_only;
    meta["map_dir"]=base;meta["map_stem"]=map_stem;meta["track_length_m"]=length;meta["start_pose"]={start_pose.x,start_pose.y,start_pose.yaw};
    meta["geometry_overlap"]=overlap;meta["min_obstacle_separating_axis_gap_m"]=min_separation>1e8?json(nullptr):json(min_separation);
    if(cycling){
      std::filesystem::create_directories(output+"/attempts");std::ofstream attempt(output+"/attempts/"+name+".json");attempt<<meta.dump(2);
      size_t begin=0;int encounter=0;
      auto save_segment=[&](size_t end,bool complete){if(end<=begin)return;++encounter;size_t original_begin=begin;json m=meta;std::string segment=name+"-encounter-"+std::to_string(encounter);
        float window=spec.value("approach_window_m",0.f);if(complete&&window>0){for(auto event:cycler.events)if(event["type"]=="spawn"&&event["id"]==encounter){float threshold=event["pass_progress_m"].get<float>()-window;while(begin+40<end&&stats[begin].progress<threshold)++begin;}}
        float peak=0,ms=0,gap=1e9;int bad=0,collided=0;for(size_t i=begin;i<end;++i){peak=std::max(peak,stats[i].peak);ms+=stats[i].ms;gap=std::min(gap,stats[i].gap);bad+=!records[i].valid;collided+=records[i].collision;}
        bool good=complete&&bad==0&&collided==0&&end-begin>=40;float distance=stats[end-1].progress-(begin?stats[begin-1].progress:0);
        m["name"]=segment;m["source_attempt"]=name;m["source_first_step"]=begin;m["source_time_s"]=records[begin].stamp;m["status"]=good?"accepted":"quarantine";
        m["omitted_cruise_prefix_records"]=begin-original_begin;m["approach_window_m"]=window;
        m["reason"]=good?"encounter_completed":reason=="completed"?"unfinished_encounter":reason;m["records"]=end-begin;m["progress_m"]=distance;m["laps"]=distance/length;m["peak_actual_speed_mps"]=peak;m["mean_teacher_ms"]=ms/(end-begin);
        m["invalid_plans"]=bad;m["collisions"]=collided;m["geometry_overlap"]=collided>0;m["obstacles_passed"]=good?1:0;m["obstacles_spawned"]=1;m["scenario"]["minimum_passes"]=1;
        m["min_obstacle_separating_axis_gap_m"]=gap>1e8?json(nullptr):json(gap);m["obstacle_events"]=json::array();
        for(auto event:cycler.events)if((event["type"]=="spawn"&&event["id"]==encounter)||(event["type"]=="remove"&&event["passed_id"]==encounter))m["obstacle_events"].push_back(event);
        std::string path=output+(good?"/accepted/":"/quarantine/")+segment+".elds";if(!metrics_only){DatasetWriter writer(path,m.dump(2));for(size_t i=begin;i<end;++i){auto r=records[i];r.step=i-begin;r.stamp-=records[begin].stamp;writer.append(r);}writer.flush();}
        m.erase("teacher_config");m["path"]=path;results.push_back(m);begin=end;
      };
      for(auto end:boundaries)save_segment(end,true);save_segment(records.size(),false);
      std::ofstream summary(output+"/summary.json");summary<<results.dump(2);
      std::cout<<name<<" "<<reason<<" completed_encounters="<<boundaries.size()<<" records="<<records.size()<<std::endl;continue;
    }
    std::string path=output+(accepted?"/accepted/":"/quarantine/")+name+".elds";if(!metrics_only){DatasetWriter writer(path,meta.dump(2));for(auto& r:records)writer.append(r);writer.flush();}
    meta.erase("teacher_config");meta["path"]=path;results.push_back(meta);std::ofstream summary(output+"/summary.json");summary<<results.dump(2);
    std::cout<<name<<" "<<reason<<" records="<<records.size()<<" laps="<<progress/length<<" peak="<<peak_speed<<std::endl;
  }
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}}

