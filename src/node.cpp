#include "el/map_paths.hpp"
#include "el/core.hpp"
#include "el/section_router.hpp"
#include "el/policy.hpp"
#include "el/scenarios.hpp"
#include "el/lap_epoch.hpp"
#include "el/scenario_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "el/scan_localizer.hpp"

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <atomic>
#include <thread>
#include <chrono>
#include <fstream>
#include <algorithm>
#include <csignal>
using namespace el;
static double steady(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
class Controller:public rclcpp::Node {
 public:
  Controller():Node("el_native_controller"){
    auto root=declare_parameter<std::string>("map_dir",el::default_map_dir);
    track_=std::make_unique<Track>(root+"/global_waypoints.json");memory_=std::make_unique<Memory>(el::map_stem(root)+".yaml");
    const auto& start=track_->points.front();
    finish_={start.x,start.y,start.yaw,track_->start_left,track_->start_right,track_->points.back().s};
    orient_track(*track_,false);
    localizer_=std::make_unique<ScanLocalizer>(*memory_);
    router_settings_=map_router_settings(root);
    const auto mode=declare_parameter<std::string>("mode","moe");
    scenario_mode_=mode=="scenario";
    if(mode!="moe"&&!scenario_mode_)throw std::runtime_error("mode must be moe or scenario");
    local_sim_clock_=declare_parameter<bool>("local_steady_sim_clock",false);
    if (local_sim_clock_ && !get_parameter("use_sim_time").as_bool())
      throw std::runtime_error("local_steady_sim_clock requires use_sim_time");
    const auto device_name=declare_parameter<std::string>("inference_device",(scenario_mode_||local_sim_clock_)?"cpu":"cuda");
    if(device_name!="cuda" && device_name!="cpu")throw std::runtime_error("inference_device must be cpu or cuda");
    const auto device=device_name=="cpu" ? InferenceDevice::cpu : InferenceDevice::cuda;
    RCLCPP_INFO(get_logger(),"TLN inference device: %s",device_name.c_str());
    const auto shared_path=declare_parameter<std::string>("unified_checkpoint","");
    scenario_race_=scenario_mode_&&declare_parameter<bool>("scenario_race_tln",true);
    if(scenario_mode_&&scenario_race_){
      if(device_name!="cpu")throw std::runtime_error("scenario RaceTLN must use CPU inference");
      race_=std::make_shared<Policy>(declare_parameter<std::string>("race_checkpoint","/workspace/models/v1.5/race.pt"),InferenceDevice::cpu);
      if(race_->uses_corridor())throw std::runtime_error("scenario RaceTLN needs an unconditioned Race checkpoint");
    }
    if(!scenario_mode_){
    if(shared_path.empty()){
      race_=std::make_shared<Policy>(declare_parameter<std::string>("race_checkpoint",router_settings_.race_checkpoint),device);
      avoid_=std::make_shared<Policy>(declare_parameter<std::string>("avoid_checkpoint","/workspace/models/avoid-corridor-v2.pt"),device);
      if(race_->uses_corridor())throw std::runtime_error("Race requires an unconditioned checkpoint");
    }else{
      race_=std::make_shared<Policy>(shared_path,device);
      if(!race_->is_unified())throw std::runtime_error("unified_checkpoint requires schema 5");
      avoid_=race_;
    }
    }
    vmax_=declare_parameter<double>("max_speed",scenario_mode_?3.5:3.); deadline_=declare_parameter<double>("deadline_ms",25);
    router_settings_.lap_planning=declare_parameter<bool>("lap_planning",false);
    if(!(vmax_>0 && vmax_<=10 && deadline_>0 && deadline_<=150))throw std::runtime_error("invalid speed or deadline");
    router_settings_.sim_avoid_min=declare_parameter<double>("sim_avoid_min",0.);
    router_settings_.sim_avoid_max=declare_parameter<double>("sim_avoid_max",0.);
    router_settings_.sim_section_speed_profile=declare_parameter<bool>("sim_section_speed_profile",false);
    router_settings_.race_sight_speed=declare_parameter<bool>("race_sight_speed",false);
    router_settings_.race_lap_speed=declare_parameter<bool>("race_lap_speed",false);
    router_settings_.race_exit_speed=declare_parameter<bool>("race_exit_speed",false);
    router_settings_.straight_boost=declare_parameter<bool>("straight_boost",false);
    router_settings_.avoid_corridor_steer_seed=declare_parameter<bool>("avoid_corridor_steer_seed",false);
    router_settings_.avoid_policy_first=declare_parameter<bool>("avoid_policy_first",false);
    router_settings_.cluster_entry=declare_parameter<bool>("cluster_entry",false);
    router_settings_.entry_candidates=parse_entry_candidates(declare_parameter<std::string>("entry_candidates","both"));
    router_settings_.smooth_isolated_exit=declare_parameter<bool>("smooth_isolated_exit",false);
    router_settings_.shared_speed_band=declare_parameter<bool>("shared_speed_band",false);
    router_settings_.whole_lap_path=declare_parameter<bool>("whole_lap_path",false);
    router_settings_.avoid_exit_pace=declare_parameter<bool>("avoid_exit_pace",false);
    router_settings_.race_verified_pace=declare_parameter<bool>("race_verified_pace",false);
    if(router_settings_.avoid_policy_first && !router_settings_.avoid_corridor_steer_seed)
      throw std::runtime_error("Avoid policy-first requires the corridor steering fallback");
    if(router_settings_.avoid_corridor_steer_seed && !(router_settings_.lap_planning && avoid_ && avoid_->uses_corridor()))
      throw std::runtime_error("corridor steering seed requires lap planning and a corridor-conditioned Avoid policy");
    router_settings_.sim_race_only=declare_parameter<bool>("sim_race_only",false);
    router_settings_.race_brake_preview_s=declare_parameter<double>("race_brake_preview_s",.18);
    if(router_settings_.shared_speed_band)router_settings_.sim_avoid_max=vmax_;
    router_settings_.validate_sim_band(vmax_,local_sim_clock_ || declare_parameter<bool>("official_simulator",false));
    if(scenario_mode_){
      if(router_settings_.lap_planning||router_settings_.sim_section_speed_profile||router_settings_.whole_lap_path||
         router_settings_.avoid_policy_first||!shared_path.empty())throw std::runtime_error("scenario mode does not use legacy TLN/lap options");
      if(finish_.left<=0 || finish_.right<=0 || finish_.length<=0)throw std::runtime_error("scenario finish line requires map bounds and length");
      scenario::Settings cfg;
      cfg.request_period=declare_parameter<double>("scenario_period_s",.10);
      cfg.optional_budget_ms=declare_parameter<double>("scenario_optional_budget_ms",55.);
      cfg.plan_age=declare_parameter<double>("scenario_max_age_s",.55);
      cfg.lateral_accel=declare_parameter<double>("scenario_lateral_accel",5.5);
      cfg.accel=declare_parameter<double>("scenario_accel",2.5);
      cfg.brake=declare_parameter<double>("scenario_brake",3.);
      cfg.nominal_size=declare_parameter<double>("scenario_nominal_size_m",cfg.nominal_size);
      cfg.position_step=declare_parameter<double>("scenario_position_step_m",.20);
      cfg.assume_passable=declare_parameter<bool>("scenario_assume_passable",false);
      cfg.visibility_enabled=declare_parameter<bool>("scenario_visibility",true);
      cfg.visibility_cost_weight=declare_parameter<double>("scenario_visibility_cost_weight",.12);
      cfg.residual_checkpoint=declare_parameter<std::string>("residual_checkpoint","/workspace/models/v3/berlin.rcn");
      cfg.no_path_snapshot=declare_parameter<std::string>("scenario_no_path_snapshot","");
      cfg.gpu_profile=declare_parameter<std::string>("scenario_gpu_profile","");
      cfg.gate_capture=declare_parameter<std::string>("scenario_gate_capture","");
      cfg.record_path=declare_parameter<std::string>("scenario_record","");
      scenario_run_=declare_parameter<std::string>("scenario_run_id","local");
      scenario_=std::make_unique<scenario::Driver>(*track_,*memory_,cfg);
      RCLCPP_INFO(get_logger(),"scenario planner: asynchronous CUDA, CPU RaceTLN %s, deterministic Avoid tracker, RCN %s",scenario_race_?"enabled":"disabled",cfg.residual_checkpoint.empty()?"disabled":"enabled");
    }
    auto metrics=declare_parameter<std::string>("metrics","/workspace/runs/control.csv");
    std::filesystem::create_directories(std::filesystem::path(metrics).parent_path());metrics_.open(metrics);
    if(declare_parameter<bool>("trace_corridor",false)){
      corridor_log_.open(metrics+".corridor.jsonl");
      scan_log_.open(metrics+".receive.csv");scan_log_.precision(15);
      if(!corridor_log_ || !scan_log_)throw std::runtime_error("cannot create diagnostic trace");
    }
    if(!metrics_)throw std::runtime_error("cannot create metrics");
    metrics_<<"step,stamp,x,y,speed,mode,hazard,valid,compute_ms,steer,target_speed,collision,remembered_cells,progress_m,reason,side,plan_ready,planned_entries,plan_brake_steps";
    ControlTiming::header(metrics_);
    metrics_<<",scan_source_age_ms,pose_source_age_ms,policy_speed,lap_cap,corridor_cap,before_guard_speed,preview_brake,curve_cap,sight_cap,steering_cap,race_plan_cap,avoid_policy_steer,avoid_policy_verified,whole_lap_path_ready,cluster_evaluated,cluster_changed,exit_evaluated,exit_smoothed\n";
    if(scenario_mode_){scenario_metrics_.open(metrics+".scenario.csv");if(!scenario_metrics_)throw std::runtime_error("cannot create scenario metrics");
      scenario_metrics_<<"step,stamp,epoch,generation,paths,candidate,plan_valid,plan_ms,age_ms,visible_m,base_steer,base_speed,residual_steer,residual_speed,desired_steer,desired_speed,final_steer,final_speed,reason,visibility_candidates,visibility_completed,first_visible_lead_m,response_slack_s,reveal_eta_s\n";}
    stop_laps_=declare_parameter<int>("stop_after_laps",0);
    drive_=create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive",1);
    debug_=create_publisher<std_msgs::msg::String>("/el/status",1);
    scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){pending_scans_.push_back(m);while(pending_scans_.size()>32)pending_scans_.pop_front();scan_received_=steady();
      if(scan_log_.is_open())scan_log_<<scan_received_<<','<<rclcpp::Time(m->header.stamp).seconds()<<'\n';});
    reset_=create_service<std_srvs::srv::Trigger>("/el/reset_memory",[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
      response->success=(!localizer_->locked() || (!scans_.empty() && fabs(scans_.back().pose.v)<.1));
      if(response->success){if(scenario_){scenario_->invalidate();++scenario_epoch_;scenario_clear_since_=-1;}held_scan_=-1;finish_.reset();memory_->reset();router_.reset();localizer_->reset();scans_.clear();pending_scans_.clear();last_scan_=-1;last_progress_=-1;progress_=0;last_={};reported_fault_=false;finished_=false;}
      response->message=response->success?"memory reset":"stop vehicle before memory reset";
    });
    last_publish_=steady();watchdog_=std::thread([this]{while(alive_){std::this_thread::sleep_for(std::chrono::milliseconds(20));if(rclcpp::ok()&&steady()-last_publish_.load()>.18){try{publish({0,0});}catch(...){}}}});
    timer_=create_wall_timer(std::chrono::milliseconds(25),[this]{tick();});
    if(!scenario_mode_)RCLCPP_INFO(get_logger(),"section router: Race/Avoid TLN and latched STOP");
  }
  ~Controller(){alive_=false;if(watchdog_.joinable())watchdog_.join();if(rclcpp::ok())publish({0,0});metrics_.flush();}
  void stop(){finished_=true;publish({0,0});}
 private:
  void publish(Control u){ackermann_msgs::msg::AckermannDriveStamped msg;msg.header.stamp=now();msg.drive.steering_angle=u.steer;msg.drive.speed=u.speed;drive_->publish(msg);last_publish_=steady();}
  // In the native simulator the sensor stamps and this clock share the same
  // host monotonic epoch. A stalled or queued /clock cannot hide an old scan.
  double source_now() {return local_sim_clock_ ? sim_clock_.now().seconds() : now().seconds();}
  void refuse(const char* reason){
    router_.stop(reason);
    if(!reported_fault_){
      RCLCPP_ERROR(get_logger(),"fault_stamp=%.9f control_fault=%s",source_now(),reason);
      reported_fault_=true;
    }
    last_={0,0};publish(last_);
  }
  void tick(){try{
    if(finished_){publish({0,0});return;}
    const auto tick_time=steady();
    const double scan_age=tick_time-scan_received_;
    if(pending_scans_.empty() || scan_age>.15){
      if(last_scan_>=0)refuse("sensor_timeout");else publish({0,0});
      return;
    }
    if(count_publishers("/drive")>1){refuse("drive_publisher_conflict");RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),3000,"multiple /drive publishers; refuse control");return;}
    const auto scan_=pending_scans_.back();
    const double stamp=rclcpp::Time(scan_->header.stamp).seconds();
    if(source_now()-stamp>.15 || (local_sim_clock_ && stamp>source_now())){
      refuse("scan_timeout");return;
    }
    if(stamp<=last_scan_ || stamp==held_scan_)return;
    auto begin=steady();ControlTiming timing;
    Scan current;current.stamp=stamp;current.angle_min=scan_->angle_min;current.angle_max=scan_->angle_max;
    if(scan_->ranges.size()<beams){refuse("missing_beams");return;}
    current.explicit_angles=true;
    int good=0;for(int b=0;b<beams;++b){size_t index=std::lround(double(b)*(scan_->ranges.size()-1)/(beams-1));float r=scan_->ranges[index];current.angles[b]=scan_->angle_min+scan_->angle_increment*index;
      bool valid=!std::isnan(r)&&r>scan_->range_min&&r!= -INFINITY;
      current.range_max=18;current.ranges[b]=valid?clamp(r,0,18):18;current.valid[b]=valid;good+=valid;}
    if(good<beams*.8){refuse("invalid_ranges");return;}
    const bool initializing=!localizer_->locked();
    const auto estimate=localizer_->update(current,*track_,last_);
    if(!estimate){
      if(!initializing){
        const std::string reason=localizer_->rejection();
        if(scenario_ && reason=="constrained_residual" && held_scan_<0 &&
           source_now()-last_scan_<=.15){
          // A single ambiguous longitudinal fit cannot authorize motion.
          // Brake now. Only an independently accepted subsequent scan may resume.
          held_scan_=stamp;last_={last_.steer,0};publish(last_);
          RCLCPP_WARN(get_logger(),"holding for fresh scan after constrained wall fit");
          return;
        }
        if(!reported_fault_)RCLCPP_ERROR(get_logger(),"scan localizer rejected frame: %s dt=%.4f",reason.c_str(),stamp-last_scan_);
        refuse("localization_lost");
      }else publish({0,0});
      return;
    }
    held_scan_=-1;
    const State state=*estimate;current.pose=state;
    // Acquire global localization at rest. Only fresh tracking cycles may drive.
    if(initializing){last_scan_=stamp;publish({0,0});return;}
    timing.mark(ControlTiming::scan);
    // A geometric lap epoch from scan-localized progress, never a GT/spawner topic.
    int n=track_->closest(state);float position=track_->points[n].s;
    float length=track_->points.back().s+hypotf(track_->points.back().x-track_->points.front().x,track_->points.back().y-track_->points.front().y);
    if(scenario_&&finish_.update(state.x,state.y,track_->points[n].map_s)){
      memory_->reset();scans_.clear();scenario_->invalidate();++scenario_epoch_;scenario_clear_since_=-1;
    }
    memory_->update(current);scans_.push_back(current);while(scans_.size()>history)scans_.pop_front();last_scan_=stamp;
    timing.mark(ControlTiming::memory);
    CorridorTrace trace;
    scenario::Action action;
    SectionDecision result{};
    if(scenario_){
      action=scenario_->step(*track_,*memory_,current,vmax_);
      timing.mark(ControlTiming::corridor);
      Mode proposed=Mode::Avoid;
      if(scenario_race_&&action.race_clear){
        if(scenario_clear_since_<0)scenario_clear_since_=tick_time;
        if(tick_time-scenario_clear_since_>=.20){
          auto input=observe(scans_,state,*memory_,*track_,0,0);
          auto cmd=race_->infer(input,&timing);
          if(std::isfinite(cmd.steer)&&std::isfinite(cmd.speed)){
            cmd.speed=std::min({cmd.speed*router_settings_.race_speed_scale,float(vmax_),action.race_speed_cap});
            // Avoid the learned correction when the Race expert is in charge.
            action.base=cmd;action.correction={};action.command=cmd;action.reason="scenario_race_tln";
            proposed=Mode::Race;
          }
        }
      }else scenario_clear_since_=-1;
      result=router_.guard_external(*memory_,state,action.command,vmax_,.025f,action.valid_plan||proposed==Mode::Race,action.reason,proposed);
    }else result=router_.step(*track_,*memory_,state,scans_,*race_,*avoid_,vmax_,.025f,stamp,router_settings_,&timing,corridor_log_.is_open()?&trace:nullptr);
    timing.mark(ControlTiming::safety);
    auto selected=result.mode;auto candidate=result.command;bool hazard=result.blocked,valid=selected!=Mode::Emergency;
    float ms=(steady()-begin)*1000;
    const double source_time=source_now();
    const char* refusal=nullptr;
    if(ms>deadline_) refusal="deadline";
    else if(source_time-stamp>.15) refusal="scan_timeout";
    else if(local_sim_clock_ && stamp>source_time) refusal="sensor_clock_mismatch";
    if(refusal){
      router_.stop(refusal);
      if(selected!=Mode::Emergency)result.reason=refusal;
      candidate={state.steer,0};selected=Mode::Emergency;valid=false;
    }
    if(last_progress_>=0){float delta=position-last_progress_;if(delta< -length/2)delta+=length;if(delta>length/2)delta-=length;if(fabs(delta)<2)progress_+=delta;}last_progress_=position;
    if(stop_laps_>0&&progress_>=length*stop_laps_){finished_=true;candidate={state.steer,0};valid=false;}
    last_=candidate;publish(candidate);
    if(scenario_){
      scenario_metrics_<<step_<<','<<std::fixed<<stamp<<','<<scenario_epoch_<<','<<action.generation<<','<<action.hypotheses<<','<<action.chosen<<','<<action.valid_plan<<','<<action.planner_ms<<','<<action.age_ms<<','<<action.visible_m<<','<<action.base.steer<<','<<action.base.speed<<','<<action.correction.steer<<','<<action.correction.speed<<','<<action.command.steer<<','<<action.command.speed<<','<<candidate.steer<<','<<candidate.speed<<','<<result.reason<<','<<action.visibility_candidates<<','<<action.visibility_completed<<','<<action.first_visible_lead<<','<<action.response_slack<<','<<action.reveal_eta<<'\n';
      if(step_%40==0)scenario_metrics_.flush();
      if(valid&&selected==Mode::Avoid)scenario_->record(*memory_,current,action,scenario_run_);
    }
    if(trace.waypoint>=0){
      corridor_log_<<nlohmann::json{{"step",step_},{"stamp",stamp},{"progress",progress_},
        {"pose",{state.x,state.y,state.yaw}},{"speed",state.v},{"waypoint",trace.waypoint},
        {"brake_event",trace.brake_event},{"policy_steer",trace.policy_steer},{"preview_seed_steer",trace.preview_seed_steer},{"candidate_steer",trace.candidate_steer},
        {"brake_pose",{trace.brake_contact.x,trace.brake_contact.y,trace.brake_contact.yaw}},
        {"wide",trace.wide},{"subdivisions",trace.subdivisions},{"free_columns",trace.fine_free},{"reachable_columns",trace.fine_reachable},{"origin",trace.origin},{"free",trace.free},{"reachable",trace.reachable},
        {"first_empty",trace.first_empty},{"obstacle_cells",memory_->obstacle_cells()}}.dump()<<'\n';
    }
    metrics_<<step_<<','<<std::fixed<<stamp<<','<<state.x<<','<<state.y<<','<<state.v<<','<<(int)selected<<','<<hazard<<','<<valid<<','<<ms<<','<<candidate.steer<<','<<candidate.speed<<','<<""<<','<<memory_->occupied()<<','<<progress_<<','<<result.reason<<','<<router_.side()<<','<<router_.lap_plan().ready()<<','<<router_.lap_plan().entries()<<','<<router_.lap_plan().brake_steps();
    timing.write(metrics_);
    metrics_<<','<<(source_time-stamp)*1000<<','<<(source_time-stamp)*1000
      <<','<<result.speed_limits.policy<<','<<result.speed_limits.lap<<','<<result.speed_limits.corridor
      <<','<<result.speed_limits.before_guard<<','<<result.speed_limits.preview_brake<<','<<result.speed_limits.curve<<','<<result.speed_limits.sight<<','<<result.speed_limits.steering<<','<<result.speed_limits.planned<<','<<result.avoid_policy_steer<<','<<(valid && selected==Mode::Avoid && result.avoid_policy_verified && candidate.steer==result.avoid_policy_steer)<<','<<router_.lap_plan().has_route()<<','<<result.cluster_evaluated<<','<<result.cluster_changed<<','<<result.exit_evaluated<<','<<result.exit_smoothed<<'\n';
    if(finished_)metrics_.flush();
    if(step_%40==0){metrics_.flush();std_msgs::msg::String msg;
      msg.data=nlohmann::json({{"mode",(int)selected},{"hazard",hazard},{"valid",valid},{"compute_ms",ms},{"progress_m",progress_},{"finished",finished_}}).dump();debug_->publish(msg);
      RCLCPP_INFO(get_logger(),"step=%lu mode=%d v=%.2f target=%.2f progress=%.1f ms=%.2f",step_,(int)selected,state.v,candidate.speed,progress_,ms);}
    ++step_;
  }catch(const std::exception& e){refuse("control_exception");RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"control rejected: %s",e.what());}}
  std::unique_ptr<Track> track_;std::unique_ptr<Memory> memory_;std::shared_ptr<Policy> race_,avoid_;
  rclcpp::Clock sim_clock_{RCL_STEADY_TIME};
  bool local_sim_clock_=false,scenario_mode_=false,scenario_race_=false;
  double scenario_clear_since_=-1;
  std::unique_ptr<scenario::Driver> scenario_;std::string scenario_run_;uint64_t scenario_epoch_=1;
  std::ofstream scenario_metrics_;
  double held_scan_=-1;
  LapEpoch finish_;
  SectionRouter router_;RouterSettings router_settings_;double vmax_=0,deadline_=0,last_scan_=-1,scan_received_=0;Control last_;
  std::unique_ptr<ScanLocalizer> localizer_;std::deque<Scan> scans_;std::deque<sensor_msgs::msg::LaserScan::ConstSharedPtr> pending_scans_;
  int stop_laps_=0;bool finished_=false,reported_fault_=false;uint64_t step_=0;float progress_=0,last_progress_=-1;
  std::ofstream metrics_,corridor_log_,scan_log_;std::atomic<bool> alive_{true};std::atomic<double> last_publish_{0};std::thread watchdog_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;rclcpp::TimerBase::SharedPtr timer_;
};
static volatile std::sig_atomic_t stop_requested=0;
static void request_stop(int){stop_requested=1;}
int main(int argc,char** argv){
  rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,request_stop);std::signal(SIGTERM,request_stop);
  try{auto node=std::make_shared<Controller>();rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
    while(rclcpp::ok()&&!stop_requested){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    node->stop();executor.remove_node(node);node.reset();rclcpp::shutdown();return 0;
  }catch(const std::exception& e){fprintf(stderr,"%s\n",e.what());rclcpp::shutdown();return 1;}
}
