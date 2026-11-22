#include "el/map_paths.hpp"
#include "el/core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <random>
class Scenario:public rclcpp::Node {
 public:Scenario():Node("el_scenario"){
    auto dir=declare_parameter<std::string>("map_dir",el::default_map_dir);
    track_=std::make_unique<el::Track>(dir+"/global_waypoints.json");
    seed_=declare_parameter<int>("seed",2026);count_=declare_parameter<int>("obstacle_count",1);start_=declare_parameter<int>("start_index",0);
    index_=declare_parameter<int>("obstacle_index",100);offset_=declare_parameter<double>("lateral_offset",0.0);
    auto offsets=declare_parameter<std::vector<double>>("obstacle_offsets",std::vector<double>{});
    if(!offsets.empty() && offsets.size()!=size_t(count_))throw std::runtime_error("expected one offset per obstacle");
    int stride=declare_parameter<int>("obstacle_stride",200);
    if(stride<1||count_<0||count_>20||start_<0||index_<0)throw std::runtime_error("invalid scenario");
    std::mt19937 rng(seed_);std::uniform_real_distribution<float> noise(-.12,.12);
    obstacles_.header.frame_id="map";
    for(int i=0;i<count_;++i){auto w=track_->points[(index_+i*stride)%track_->points.size()];geometry_msgs::msg::Pose p;float d=offsets.empty()?offset_+noise(rng):offsets[i];
      p.position.x=w.x-sinf(w.yaw)*d;p.position.y=w.y+cosf(w.yaw)*d;p.orientation.z=sinf(w.yaw/2);p.orientation.w=cosf(w.yaw/2);obstacles_.poses.push_back(p);}
    pub_=create_publisher<geometry_msgs::msg::PoseArray>("/sim/obstacles",10);reset_=create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",10);
    if (declare_parameter<bool>("publish_steady_clock",false)) {
      // The local Gym runs in real time. Wall-clock corrections must not pause its timers.
      clock_pub_=create_publisher<rosgraph_msgs::msg::Clock>("/clock",rclcpp::ClockQoS());
      clock_timer_=create_wall_timer(std::chrono::milliseconds(5),[this] {
        rosgraph_msgs::msg::Clock sample;
        sample.clock=sim_clock_.now();
        clock_pub_->publish(sample);
      });
    }
    timer_=create_wall_timer(std::chrono::milliseconds(500),[this]{obstacles_.header.stamp=now();pub_->publish(obstacles_);
      if(!reset_done_&&reset_->get_subscription_count()>0){auto w=track_->points[start_%track_->points.size()];geometry_msgs::msg::PoseWithCovarianceStamped m;m.header.frame_id="map";m.header.stamp=now();m.pose.pose.position.x=w.x;m.pose.pose.position.y=w.y;m.pose.pose.orientation.z=sinf(w.yaw/2);m.pose.pose.orientation.w=cosf(w.yaw/2);reset_->publish(m);reset_done_=true;}
    });
    RCLCPP_INFO(get_logger(),"seed=%d count=%d index=%d offset=%.2f; simulator-only ground truth; controller does not subscribe",seed_,count_,index_,offset_);
  }
 private:std::unique_ptr<el::Track> track_;int seed_,count_,start_,index_;double offset_;bool reset_done_=false;geometry_msgs::msg::PoseArray obstacles_;
  rclcpp::Clock sim_clock_{RCL_STEADY_TIME};
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::TimerBase::SharedPtr clock_timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_;rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr reset_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Scenario>());rclcpp::shutdown();return 0;}catch(const std::exception& e){fprintf(stderr,"%s\n",e.what());rclcpp::shutdown();return 1;}}
