#pragma once
#include <cstring>
#include <type_traits>
#include <unistd.h>

namespace el {
// Same-build binary fields keep native floating point, including unseen NaNs.
// The on-disk envelope identifies the evaluator, policies, Gym and map files.
struct OfflineSnapshot {
  json& data;
  bool reading;
  template<class T> void field(const char* name,T& value){
    if(reading)data.at(name).get_to(value);else data[name]=value;
  }
  template<class T> void flat(const char* name,T* ptr,size_t n){
    static_assert(std::is_trivially_copyable<T>::value);
    if(reading){
      const auto& bytes=data.at(name).get_binary();
      if(bytes.size()!=n*sizeof(T))throw std::runtime_error(std::string("snapshot size: ")+name);
      if(n)std::memcpy(ptr,bytes.data(),bytes.size());
    }else{
      std::vector<uint8_t> bytes(n*sizeof(T));
      if(n)std::memcpy(bytes.data(),ptr,bytes.size());
      data[name]=json::binary(std::move(bytes));
    }
  }
  template<class T> void vec(const char* name,std::vector<T>& values,size_t limit){
    if(reading){
      size_t bytes=data.at(name).get_binary().size();
      if(bytes%sizeof(T) || bytes/sizeof(T)>limit)throw std::runtime_error("invalid snapshot vector");
      values.resize(bytes/sizeof(T));
    }
    flat(name,values.data(),values.size());
  }
  void localization(ScanLocalizer& l){
    field("localization_locked",l.locked_);field("localization_stamp",l.stamp_);
    flat("localization_pose",&l.pose_,1);
    if(reading)l.used_motion_prior_=data.value("localization_used_motion_prior",false);
    else field("localization_used_motion_prior",l.used_motion_prior_);
  }
  void controller(Memory& m,SectionRouter& r,std::deque<Scan>& scans,size_t points){
    const size_t cells=m.base_.size();
    flat("odds",m.odds_.data(),cells);flat("features",m.feature_odds_.data(),cells);
    flat("seen",m.seen_.data(),cells);
    if(reading){
      std::vector<int> hits;
      vec("hits",hits,cells);
      for(int k:hits)if(k<0 || size_t(k)>=cells)throw std::runtime_error("invalid snapshot hit index");
      m.replace_current_hits(std::move(hits));
    }else vec("hits",m.current_hits_,cells);
    field("wall_returns",m.associated_wall_returns_);
    std::vector<Scan> history_copy(scans.begin(),scans.end());
    vec("scans",history_copy,history);
    if(reading)scans.assign(history_copy.begin(),history_copy.end());
    if(r.stop_reason_)throw std::runtime_error("checkpoint must precede latched stop");
    auto& plan=r.lap_plan_;
    auto& corridor=r.corridor_;
    vec("sections",plan.sections_,points);
    flat("pending",&plan.pending_,1);vec("scout_line",plan.scout_lateral_,points);
    if(!reading || data.contains("race_speed2")){
      vec("race_speed2",plan.race_speed2_,points);field("race_cruise",plan.race_cruise_);
    }else{plan.race_speed2_.clear();plan.race_cruise_=0;}
    field("use_line",plan.use_line_);field("recording",plan.recording_);field("ready",plan.ready_);
    field("scout_stopped",plan.scout_stopped_);field("cap",plan.cap_);
    field("entries",plan.planned_entries_);field("brake_steps",plan.brake_steps_);
    vec("previous_path",corridor.previous_,points);
    field("generation",corridor.generation_);field("path_valid",corridor.valid_);
    flat("path",corridor.path_.data(),corridor.path_.size());
    flat("offsets",corridor.offsets_.data(),corridor.offsets_.size());
    flat("last_pose",&r.last_pose_,1);flat("last_command",&r.last_command_,1);
    field("have_pose",r.have_pose_);field("merge_blocked",r.merge_blocked_);
    field("last_station",r.last_station_);field("position",r.position_);field("end",r.end_);
    field("clear_since",r.clear_since_);field("side",r.side_);field("avoiding",r.avoiding_);
    if(reading && ((!plan.scout_lateral_.empty() && plan.scout_lateral_.size()!=points) ||
                   (!corridor.previous_.empty() && corridor.previous_.size()!=points)))
      throw std::runtime_error("snapshot track dimensions differ");
  }
  void simulator(Sim& sim,PyObject* module){
    field("stamp",sim.stamp);field("collision",sim.collision);
    flat("state",&sim.state,1);flat("scan",&sim.scan,1);vec("ranges",sim.raw_scan,100000);
    if(reading){
      auto text=data.at("gym").get<std::string>();
      Py result(PyObject_CallMethod(module,"restore","Os",sim.env.p,text.c_str()));
    }else{
      Py result(PyObject_CallMethod(module,"capture","O",sim.env.p));
      data["gym"]=PyUnicode_AsUTF8(result.p);
    }
  }
};

inline Py snapshot_module(){
  auto dir=std::filesystem::path(__FILE__).parent_path().string();
  Py path(PyUnicode_FromString(dir.c_str()));
  if(PyList_Insert(PySys_GetObject("path"),0,path.p))throw std::runtime_error("snapshot module path");
  return Py(PyImport_ImportModule("offline_gym_state"));
}

inline void write_snapshot(const std::filesystem::path& path,const json& data){
  auto temp=path;temp+=".tmp."+std::to_string(getpid());
  try{
    std::ofstream out(temp,std::ios::binary);
    out.exceptions(std::ios::badbit|std::ios::failbit);
    json::to_cbor(data,out);out.close();
    // Publish atomically and never overwrite another run's checkpoint.
    std::filesystem::create_hard_link(temp,path);
    std::filesystem::remove(temp);
  }catch(...){std::filesystem::remove(temp);throw;}
}
}
