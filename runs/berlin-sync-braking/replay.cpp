#include "avoid_support.hpp"
#include "el/eval_metrics.hpp"
#include "study_router.hpp"
#include "offline_snapshot.hpp"
using namespace el;
namespace fs=std::filesystem;
const fs::path root="/workspace";
json load(const fs::path& p){json j; std::ifstream f(p); if(!f)throw std::runtime_error(p);f>>j;return j;}
void scan_angles(Scan& s){
  s.range_max=18;s.explicit_angles=true;s.angle_min=-2.35f;s.angle_max=2.35f;
  for(int k=0;k<beams;++k)s.angles[k]=s.angle_min+float(4.7/1079)*std::lround(k*1079./(beams-1));
}
DynamicState fitted(const json& r){
  DynamicState state;
  for(int i=0;i<7;++i)state.z[i]=r["z"][i];
  for(int i=0;i<2;++i)state.delay[i]=r["delay"][i];
  return state;
}
Control output(const json&r){return {r["steer"],r["target"]};}
struct Study {
  std::string key;int offset;json data,fit;Track track; Memory memory;SectionRouter router;
  std::deque<Scan> scans;
  Policy race,avoid;RouterSettings settings;
  Sim sim;Py module;std::unique_ptr<Teacher> teacher;
  Study(std::string k,int o):key(k),offset(o),data(load(k+".json")),fit(load(k+"-fit.json")),
    track("/workspace/maps/berlin/global_waypoints.json"),memory("/workspace/maps/berlin/berlin.yaml"),
    race("/workspace/models/race-berlin-wsp.pt",InferenceDevice::cpu),avoid("/workspace/models/avoid-corridor-v2.pt",InferenceDevice::cpu),
    settings(map_router_settings("/workspace/maps/berlin")),sim("/workspace/maps/berlin/berlin",12345,30),module(snapshot_module()){
    orient_track(track,false);settings.lap_planning=true;settings.sim_avoid_min=2.7f;settings.sim_avoid_max=3;
    sim.obstacles(data["boxes"]);sim.reset(track.points[37]);
  }
  void ingest(Scan scan){memory.update(scan);scans.push_back(scan);if(scans.size()>history)scans.pop_front();}
  void save(int i,Scan scan,Control previous){
    auto& row=data["controls"][i];int frame=row["scan"]; sim.state=fitted(fit[frame]);
    sim.stamp=row["stamp"];sim.scan=scan;sim.collision=false;
    Py helper(PyImport_ImportModule("render"));
    auto state=fit[frame];Py z(Py_BuildValue("[ddddddd]",double(sim.state.z[0]),double(sim.state.z[1]),double(sim.state.z[2]),double(sim.state.z[3]),double(sim.state.z[4]),double(sim.state.z[5]),double(sim.state.z[6])));
    Py delay(Py_BuildValue("[dd]",double(sim.state.delay[0]),double(sim.state.delay[1])));
    Py installed(PyObject_CallMethod(helper.p,"install","OOOii",sim.env.p,z.p,delay.p,frame,offset));
    json snapshot={{"key",key},{"row",i},{"previous",{previous.steer,previous.speed}}};
    OfflineSnapshot a{snapshot,false};a.controller(memory,router,scans,track.points.size());a.simulator(sim,module.p);
    std::ofstream file(key+"-"+std::to_string(i)+".cbor",std::ios::binary);json::to_cbor(snapshot,file);
  }
  void shadow(const std::set<int>& checkpoints){
    std::ifstream input(key+"-"+std::to_string(offset)+".bin",std::ios::binary);
    std::ofstream out(key+"-shadow-"+std::to_string(offset)+".jsonl");
    Control previous{};
    for(int i=0;i<int(data["controls"].size());++i){
      const auto& row=data["controls"][i];int obs=row["scan"];
      auto st=basic(fitted(fit[obs]));if(row["scan"]==row["odom"])st.v=row["v"];st.steer=fabsf(st.v)>.2f?clamp(atanf(.3302f*st.yaw_rate/st.v),-.4f,.4f):previous.steer;
      Scan scan;scan.pose=st;scan.stamp=row["stamp"];scan_angles(scan);
      float ranges[beams];if(!input.read(reinterpret_cast<char*>(ranges),sizeof(ranges)))throw std::runtime_error("scan truncated");
      for(int k=0;k<beams;++k){scan.valid[k]=ranges[k]>0;scan.ranges[k]=scan.valid[k]?clamp(ranges[k],0,18):18;}
      if(checkpoints.count(i))save(i,scan,previous);
      ingest(scan);CorridorTrace tr;
      auto u=router.step(track,memory,st,scans,race,avoid,3,.025f,scan.stamp,settings,nullptr,&tr);
      out<<json{{"i",i},{"s",row["progress"]},{"reason",u.reason},{"mode",int(u.mode)},{"side",router.side()},
        {"policy",tr.policy_steer},{"preview",tr.candidate_steer},{"command",{u.command.steer,u.command.speed}},
        {"gate",{tr.gate_entry,tr.gate_exit,tr.gate_offset}},{"offsets",tr.planned_offsets},{"stored",memory.occupied()}}.dump()<<'\n';
      previous=output(row);router.follow_record(st,previous);
    }
  }
  void branch(int start,const std::string& arm){
    std::ifstream file(key+"-"+std::to_string(start)+".cbor",std::ios::binary);
    auto snap=json::from_cbor(file);OfflineSnapshot a{snap,true};a.controller(memory,router,scans,track.points.size());a.simulator(sim,module.p);
    Control previous{snap["previous"][0],snap["previous"][1]};
    bool expert=arm=="teacher" || arm=="teacher-direct";
    study_direct=arm=="teacher-direct" || arm=="tln-direct";
    if(expert){
      auto cfg=load(root/"config/teachers_expanded_v5.json");TeacherConfig tc;tc.kind=TeacherKind::Svg;tc.seed=12345;
      tc.samples=cfg.at("samples");tc.horizon=cfg.at("horizon");tc.iterations=cfg.value("iterations",2);
      tc.temperature=cfg.at("temperature");tc.steer_sigma=cfg.at("steer_sigma");tc.speed_sigma=cfg.at("speed_sigma");
      tc.guide_samples=cfg.value("guide_samples",128);tc.guide_iterations=cfg.value("guide_iterations",6);
      teacher=std::make_unique<Teacher>(tc);
    }
    const bool alt=std::getenv("STUDY_ALTERNATE_TAIL")!=nullptr;
    std::ofstream out(key+"-"+std::to_string(start)+"-"+arm+(alt?"-alt":"")+".jsonl");double progress=data["controls"][start]["progress"],gap=1e3;
    double last=project_route(track,basic(sim.state)).s,length=route_length(track);
    DynamicState before_scan=sim.state;
    for(int tick=0;tick<650;++tick){
      int i=start+tick;auto& controls=data["controls"];bool logged=i<int(controls.size());double now=sim.scan.stamp;
      Scan scan=sim.scan;scan_angles(scan);
      State st=scan.pose;
      const bool synchronize=arm=="tln-sync" || (arm=="one-sync" && i==908);
      if(arm=="one-raw")study_direct=(i==908);
      if(tick || synchronize){
        const int skew=logged?controls[i]["odom"].get<int>()-controls[i]["scan"].get<int>():0;
        if(skew!=0 && skew!=-1)throw std::runtime_error("unexpected odometry skew");
        st=basic(!synchronize && skew==-1?before_scan:sim.state);st.steer=fabsf(st.v)>.2f?clamp(atanf(.3302f*st.yaw_rate/st.v),-.4f,.4f):previous.steer;scan.pose=st;}
      double age=logged?controls[i]["latency"].get<double>():0.;
      double odom_age=age;
      if(logged && !synchronize){
        const auto& frames=data["physics"];
        odom_age+=frames[controls[i]["scan"].get<int>()][0].get<double>()-frames[controls[i]["odom"].get<int>()][0].get<double>();
      }
      if(age>.150 || odom_age>.150 || sim.stamp-scan.stamp>.150)throw std::runtime_error("sensor_timeout");
      ingest(scan);CorridorTrace tr;float chosen=0,speed=0;bool feasible=true;
      study_steering=[&](float original,float target){
        speed=target;chosen=original;
        if(expert){
          auto p=problem(track,memory,sim.state,router.lap_plan().cap(),previous);
          local_route(p);
          auto sol=teacher->solve(p,initial(p,40));feasible=sol.valid;
          if(feasible)chosen=sol.action.steer;
        }
        return chosen;
      };
      auto u=router.step(track,memory,st,scans,race,avoid,3,.025f,now,settings,nullptr,&tr);
      std::string reason=feasible?u.reason:"teacher_no_plan";
      json rec={{"i",i},{"t",now},{"s",progress},{"state",{st.x,st.y,st.yaw,st.v,st.steer}},{"policy",tr.policy_steer},
        {"source_steer",chosen},{"preview",tr.candidate_steer},{"target_before_guard",speed},{"steer",u.command.steer},{"speed",u.command.speed},
        {"reason",reason},{"mode",int(u.mode)},{"side",router.side()},{"gate",{tr.gate_entry,tr.gate_exit,tr.gate_offset}}};
      if(!feasible){rec["source_steer"]=nullptr;rec["preview"]=nullptr;rec["steer"]=nullptr;}
      if(u.mode==Mode::Emergency || !feasible){out<<rec.dump()<<'\n';break;}
      int nt=(tick+int(alt))%2?3:2,delay=0;
      if(i+1<int(controls.size())){
        int fr=controls[i]["scan"];nt=controls[i+1]["scan"].get<int>()-fr;
        for(int j=1;j<=nt && fit[fr+j]["command"].get<int>()<i;++j)++delay;
      }
      for(int j=0;j<nt;++j){
        before_scan=sim.state;sim.step(j<delay?previous:u.command);
        for(auto& box:data["boxes"])gap=std::min(gap,double(separation(sim.state,box)));
        if(sim.collision || gap<=0)break;
      }
      previous=u.command;
      double curr=project_route(track,basic(sim.state)).s;progress+=route_delta(curr,last,length);last=curr;
      rec["min_box_gap"]=gap;
      if(sim.collision || gap<=0)rec["reason"]="collision";
      else if(progress>=43.5)rec["reason"]="cleared";
      out<<rec.dump()<<'\n';
      if(sim.collision || gap<=0 || progress>=43.5)break;
    }
    study_steering={};
  }
};
int main(int argc,char**argv){try{Py_Initialize();Study s(argv[1],std::stoi(argv[2]));
  if(std::string(argv[3])=="shadow"){std::set<int> rows;for(int i=4;i<argc;++i)rows.insert(std::stoi(argv[i]));s.shadow(rows);}
  else s.branch(std::stoi(argv[3]),argv[4]);
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
