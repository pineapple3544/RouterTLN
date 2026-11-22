#include "el/map_paths.hpp"
#include "el/core.hpp"
#include "el/section_router.hpp"
#include "el/policy.hpp"
#include "el/scenarios.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

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
    orient_track(*track_,false);
    router_settings_=map_router_settings(root);
    if(declare_parameter<std::string>("mode","moe")!="moe")throw std::runtime_error("online controller requires mode=moe");
    local_sim_clock_=declare_parameter<bool>("local_steady_sim_clock",false);
    if (local_sim_clock_ && !get_parameter("use_sim_time").as_bool())
      throw std::runtime_error("local_steady_sim_clock requires use_sim_time");
    const auto device_name=declare_parameter<std::string>("inference_device",local_sim_clock_?"cpu":"cuda");
    if(device_name!="cuda" && device_name!="cpu")throw std::runtime_error("inference_device must be cpu or cuda");
    const auto device=device_name=="cpu" ? InferenceDevice::cpu : InferenceDevice::cuda;
    RCLCPP_INFO(get_logger(),"TLN inference device: %s",device_name.c_str());
    race_=std::make_unique<Policy>(declare_parameter<std::string>("race_checkpoint",router_settings_.race_checkpoint),device);
    avoid_=std::make_unique<Policy>(declare_parameter<std::string>("avoid_checkpoint","/workspace/models/avoid-corridor-v2.pt"),device);
    if(race_->uses_corridor())throw std::runtime_error("Race requires an unconditioned checkpoint");
    vmax_=declare_parameter<double>("max_speed",3); deadline_=declare_parameter<double>("deadline_ms",25);
    router_settings_.lap_planning=declare_parameter<bool>("lap_planning",false);
    if(!(vmax_>0 && vmax_<=10 && deadline_>0 && deadline_<=150))throw std::runtime_error("invalid speed or deadline");
    router_settings_.sim_avoid_min=declare_parameter<double>("sim_avoid_min",0.);
    router_settings_.sim_avoid_max=declare_parameter<double>("sim_avoid_max",0.);
    router_settings_.validate_sim_band(vmax_,local_sim_clock_);
    auto metrics=declare_parameter<std::string>("metrics","/workspace/runs/control.csv");
    std::filesystem::create_directories(std::filesystem::path(metrics).parent_path());metrics_.open(metrics);
    if(!metrics_)throw std::runtime_error("cannot create metrics");
    metrics_<<"step,stamp,x,y,speed,mode,hazard,valid,compute_ms,steer,target_speed,collision,remembered_cells,progress_m,reason,side,plan_ready,planned_entries,plan_brake_steps";
    ControlTiming::header(metrics_);
    metrics_<<",scan_source_age_ms,odom_source_age_ms\n";
    stop_laps_=declare_parameter<int>("stop_after_laps",0);
    drive_=create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive",1);
    debug_=create_publisher<std_msgs::msg::String>("/el/status",1);
    scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){scan_=m;scan_received_=steady();});
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/car_state/odom_GT",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
      State s;auto& p=m->pose.pose;s.x=p.position.x;s.y=p.position.y;auto q=p.orientation;
      s.yaw=atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));s.v=m->twist.twist.linear.x;s.yaw_rate=m->twist.twist.angular.z;
      s.steer=fabs(s.v)>.2?clamp(atan(.3302*s.yaw_rate/s.v),-.4,.4):last_.steer;
      if(!odoms_.empty()&&hypot(s.x-odoms_.back().second.x,s.y-odoms_.back().second.y)>2){memory_->reset();router_.reset();scans_.clear();last_progress_=-1;}
      odoms_.push_back({rclcpp::Time(m->header.stamp).seconds(),s});while(odoms_.size()>100)odoms_.pop_front();odom_received_=steady();
    });
    collision_sub_=create_subscription<std_msgs::msg::String>("/sim/collision",10,[this](std_msgs::msg::String::ConstSharedPtr m){collision_=m->data!="none";});
    reset_=create_service<std_srvs::srv::Trigger>("/el/reset_memory",[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
      response->success=odoms_.empty()||fabs(odoms_.back().second.v)<.1;
      if(response->success){memory_->reset();router_.reset();scans_.clear();}
      response->message=response->success?"memory reset":"stop vehicle before memory reset";
    });
    last_publish_=steady();watchdog_=std::thread([this]{while(alive_){std::this_thread::sleep_for(std::chrono::milliseconds(20));if(rclcpp::ok()&&steady()-last_publish_.load()>.18){try{publish({0,0});}catch(...){}}}});
    timer_=create_wall_timer(std::chrono::milliseconds(25),[this]{tick();});
    RCLCPP_INFO(get_logger(),"section router: Race/Avoid TLN and latched STOP");
  }
  ~Controller(){alive_=false;if(watchdog_.joinable())watchdog_.join();if(rclcpp::ok())publish({0,0});metrics_.flush();}
  void stop(){finished_=true;publish({0,0});}
 private:
  void publish(Control u){ackermann_msgs::msg::AckermannDriveStamped msg;msg.header.stamp=now();msg.drive.steering_angle=u.steer;msg.drive.speed=u.speed;drive_->publish(msg);last_publish_=steady();}
  // In the native simulator the sensor stamps and this clock share the same
  // host monotonic epoch. A stalled or queued /clock cannot hide an old scan.
  double source_now() {return local_sim_clock_ ? sim_clock_.now().seconds() : now().seconds();}
  void tick(){try{
    if(finished_){publish({0,0});return;}
    const auto tick_time=steady();
    const double scan_age=tick_time-scan_received_, odom_age=tick_time-odom_received_;
    if (!scan_ || odoms_.empty() || scan_age>.15 || odom_age>.15) {
      if (last_scan_>=0) {
        router_.stop("sensor_timeout");
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,
                            "sensor_timeout: scan age %.1f ms, odometry age %.1f ms",
                            1000*scan_age,1000*odom_age);
      }
      publish({0,0});
      return;
    }
    if(count_publishers("/drive")>1){router_.stop("drive_publisher_conflict");publish({0,0});RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),3000,"multiple /drive publishers; refuse control");return;}
    double stamp=rclcpp::Time(scan_->header.stamp).seconds();
    if(stamp<=last_scan_){
      const double source_age=source_now()-stamp;
      if(source_age>.15){
        router_.stop("scan_timeout");publish({0,0});
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,
            "scan_timeout: source age %.1f ms, scan receipt age %.1f ms, odom receipt age %.1f ms",
            1000*source_age,1000*scan_age,1000*odom_age);
      }
      return;
    }
    auto aligned=std::min_element(odoms_.begin(),odoms_.end(),[&](auto& a,auto& b){return fabs(a.first-stamp)<fabs(b.first-stamp);});
    if(fabs(aligned->first-stamp)>.03){
      // Scan and odometry arrive separately. Wait for the matching sample
      // within the existing freshness deadline; never execute a mismatched pair.
      if(source_now()-stamp>.15){router_.stop("odom_sync_timeout");publish({0,0});}
      return;
    }
    auto begin=steady();ControlTiming timing;State state=aligned->second;
    if(!std::isfinite(state.x)||!std::isfinite(state.y)||!std::isfinite(state.yaw)||!std::isfinite(state.v)){router_.stop("invalid_state");publish({0,0});return;}
    Scan current;current.pose=state;current.stamp=stamp;current.angle_min=scan_->angle_min;current.angle_max=scan_->angle_max;
    if(scan_->ranges.size()<beams){router_.stop("missing_beams");publish({0,0});return;}
    current.explicit_angles=true;
    int good=0;for(int b=0;b<beams;++b){size_t index=std::lround(double(b)*(scan_->ranges.size()-1)/(beams-1));float r=scan_->ranges[index];current.angles[b]=scan_->angle_min+scan_->angle_increment*index;
      bool valid=!std::isnan(r)&&r>scan_->range_min&&r!= -INFINITY;
      current.range_max=18;current.ranges[b]=valid?clamp(r,0,18):18;current.valid[b]=valid;good+=valid;}
    if(good<beams*.8){router_.stop("invalid_ranges");publish({0,0});return;}
    timing.mark(ControlTiming::scan);
    memory_->update(current);scans_.push_back(current);while(scans_.size()>history)scans_.pop_front();last_scan_=stamp;
    timing.mark(ControlTiming::memory);
    if(collision_)router_.stop("sim_collision");
    auto result=router_.step(*track_,*memory_,state,scans_,*race_,*avoid_,vmax_,.025f,stamp,router_settings_,&timing);
    timing.mark(ControlTiming::safety);
    auto selected=result.mode;auto candidate=result.command;bool hazard=result.blocked,valid=selected!=Mode::Emergency;
    float ms=(steady()-begin)*1000;
    const double source_time=source_now();
    const char* refusal=nullptr;
    if(ms>deadline_) refusal="deadline";
    else if(source_time-stamp>.15) refusal="scan_timeout";
    else if(source_time-aligned->first>.15) refusal="odom_sync_timeout";
    else if(local_sim_clock_ && (stamp>source_time || aligned->first>source_time)) refusal="sensor_clock_mismatch";
    if(refusal){
      router_.stop(refusal);
      if(selected!=Mode::Emergency)result.reason=refusal;
      candidate={state.steer,0};selected=Mode::Emergency;valid=false;
    }
    int n=track_->closest(state);float position=track_->points[n].s;
    float length=track_->points.back().s+hypotf(track_->points.back().x-track_->points.front().x,track_->points.back().y-track_->points.front().y);
    if(last_progress_>=0){float delta=position-last_progress_;if(delta< -length/2)delta+=length;if(delta>length/2)delta-=length;if(fabs(delta)<2)progress_+=delta;}last_progress_=position;
    if(stop_laps_>0&&progress_>=length*stop_laps_){finished_=true;candidate={state.steer,0};valid=false;}
    last_=candidate;publish(candidate);
    metrics_<<step_<<','<<std::fixed<<stamp<<','<<state.x<<','<<state.y<<','<<state.v<<','<<(int)selected<<','<<hazard<<','<<valid<<','<<ms<<','<<candidate.steer<<','<<candidate.speed<<','<<collision_<<','<<memory_->occupied()<<','<<progress_<<','<<result.reason<<','<<router_.side()<<','<<router_.lap_plan().ready()<<','<<router_.lap_plan().entries()<<','<<router_.lap_plan().brake_steps();
    timing.write(metrics_);
    metrics_<<','<<(source_time-stamp)*1000<<','<<(source_time-aligned->first)*1000<<'\n';
    if(finished_)metrics_.flush();
    if(step_%40==0){metrics_.flush();std_msgs::msg::String msg;
      msg.data=nlohmann::json({{"mode",(int)selected},{"hazard",hazard},{"valid",valid},{"compute_ms",ms},{"progress_m",progress_},{"finished",finished_}}).dump();debug_->publish(msg);
      RCLCPP_INFO(get_logger(),"step=%lu mode=%d v=%.2f target=%.2f progress=%.1f ms=%.2f",step_,(int)selected,state.v,candidate.speed,progress_,ms);}
    ++step_;
  }catch(const std::exception& e){router_.stop("control_exception");publish({0,0});RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"control rejected: %s",e.what());}}
  std::unique_ptr<Track> track_;std::unique_ptr<Memory> memory_;std::unique_ptr<Policy> race_,avoid_;
  rclcpp::Clock sim_clock_{RCL_STEADY_TIME};
  bool local_sim_clock_=false;
  SectionRouter router_;RouterSettings router_settings_;double vmax_=0,deadline_=0,last_scan_=-1,scan_received_=0,odom_received_=0;Control last_;
  std::deque<std::pair<double,State>> odoms_;std::deque<Scan> scans_;sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  int stop_laps_=0;bool collision_=false,finished_=false;uint64_t step_=0;float progress_=0,last_progress_=-1;
  std::ofstream metrics_;std::atomic<bool> alive_{true};std::atomic<double> last_publish_{0};std::thread watchdog_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;rclcpp::Subscription<std_msgs::msg::String>::SharedPtr collision_sub_;
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
