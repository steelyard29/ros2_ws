// Domain-176 comparison or explicit domain-0 sensor candidate. No PX4 outputs.
// Reuses the existing APF C++ implementation; path lookahead and final limiting
// are new adapters, NOT covered by the user's historical flight experience.
#include "velocity_apf_controller.hpp"
#include "path_guidance.hpp"
#include "domain_policy.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <cstdlib>
#include <algorithm>

class LegacyShadow : public rclcpp::Node {
 public:
  LegacyShadow():Node("legacy_apf_shadow"),controller_(this) {
    real_inputs_=declare_parameter<bool>("real_sensor_inputs",false);
    const bool isolated_task_test=declare_parameter<bool>("isolated_task_test",false);
    arrival_tolerance_=declare_parameter<double>("arrival_tolerance",.10);
    const char *domain=std::getenv("ROS_DOMAIN_ID");
    if(!apf_domain::configured_allowed(domain,real_inputs_,isolated_task_test))
      throw std::runtime_error("APF domain mismatch: domain0 requires real_sensor_inputs; domain186 requires isolated_task_test");
    if(!std::isfinite(arrival_tolerance_)||arrival_tolerance_<=0||arrival_tolerance_>.10
       ||(real_inputs_&&arrival_tolerance_>.08))
      throw std::runtime_error("arrival tolerance incompatible with 8cm task arrival");
    path_guided_=declare_parameter<bool>("path_guided",false);
    if(real_inputs_&&!path_guided_)
      throw std::runtime_error("real sensor candidate requires A* path guidance");
    const auto &pid=controller_.getPIDConfig();const auto &apf=controller_.getAvoidanceConfig();
    // Refuse silent fallback to another legacy/default configuration.
    if(std::abs(pid.position_tolerance-.05)>.00001||std::abs(apf.obstacle_distance_threshold-.9)>.00001
       ||std::abs(apf.avoidance_strength_gain-.8)>.00001)
      throw std::runtime_error("reviewed shadow APF profile was not loaded");
    RCLCPP_WARN(get_logger(),"SHADOW ONLY: legacy profile loaded, tolerance=%.3f influence=%.3f gain=%.3f output_cap=0.15",
                pid.position_tolerance,apf.obstacle_distance_threshold,apf.avoidance_strength_gain);
    RCLCPP_WARN(get_logger(),"path_guided=%s; output still requires independent map stopping gate",path_guided_?"true":"false");
    pub_=create_publisher<geometry_msgs::msg::Twist>("/robocup/legacy_apf/cmd_vel_odom",10);
    candidate_pub_=create_publisher<std_msgs::msg::String>("/robocup/legacy_apf/candidate",10);
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>(
      "/visual_slam/tracking/odometry",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr m){odom_=m;});
    scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>(
      "/robocup/legacy_apf/scan",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr m){scan_=m;});
    path_sub_=create_subscription<nav_msgs::msg::Path>("/robocup/legacy_apf/path",10,
      [this](nav_msgs::msg::Path::SharedPtr m){
        if(m->header.frame_id!="odom"||m->poses.empty()) {path_.reset();return;}
        for(auto &p:m->poses)if(!std::isfinite(p.pose.position.x)||!std::isfinite(p.pose.position.y)){
          path_.reset();return;}
        path_=m;index_=0;target_valid_=false;
      });
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{tick();});
  }
 private:
  static int64_t ns(const builtin_interfaces::msg::Time &t) {
    return int64_t(t.sec)*1000000000LL+t.nanosec;
  }
  void publish(const geometry_msgs::msg::Twist &out,bool valid=false) {
    pub_->publish(out);  // Retained only for old comparison tools, not real mission input.
    std::ostringstream s;s<<std::setprecision(17);
    s<<"{\"schema\":1,\"valid\":"<<(valid?"true":"false")
     <<",\"stamp_ns\":"<<now().nanoseconds()<<",\"path_stamp_ns\":"<<(path_?ns(path_->header.stamp):0)
     <<",\"odom_stamp_ns\":"<<(odom_?ns(odom_->header.stamp):0)
     <<",\"scan_stamp_ns\":"<<(scan_?ns(scan_->header.stamp):0)
     <<",\"frame_id\":\"odom\",\"velocity_xy\":["<<out.linear.x<<","<<out.linear.y<<"]}";
    std_msgs::msg::String msg;msg.data=s.str();candidate_pub_->publish(msg);
  }
  bool fresh(const builtin_interfaces::msg::Time &t,double limit) {
    double age=(now()-rclcpp::Time(t)).seconds();return age>=0&&age<=limit;
  }
  void tick() {
    geometry_msgs::msg::Twist out;
    if(!path_||!odom_||!scan_||!fresh(odom_->header.stamp,.25)||!fresh(scan_->header.stamp,.25)
       ||odom_->header.frame_id!="odom"||odom_->child_frame_id!="base_link"
       ||scan_->header.frame_id!="base_link"||(real_inputs_&&!fresh(path_->header.stamp,.5))) {publish(out);return;}
    // Never turn a legacy 'invalid scan => zero' result back into forward
    // motion in path-guided mode. Match the loaded legacy validity ratio.
    if(path_guided_){
      if(scan_->ranges.size()<90||!std::isfinite(scan_->range_min)||!std::isfinite(scan_->range_max)
          ||scan_->range_min<0||scan_->range_max<=scan_->range_min
          ||!std::isfinite(scan_->angle_min)||!std::isfinite(scan_->angle_max)
          ||!std::isfinite(scan_->angle_increment)||scan_->angle_increment<=0
          ||scan_->angle_max-scan_->angle_min<6.1
          ||std::abs(scan_->angle_max-scan_->angle_min-(scan_->ranges.size()-1)*scan_->angle_increment)>.02){publish(out);return;}
      size_t valid=0;
      for(float r:scan_->ranges)valid+=std::isfinite(r)&&r>=scan_->range_min&&r<=scan_->range_max;
      if(float(valid)/scan_->ranges.size()<controller_.getAvoidanceConfig().min_valid_ranges_ratio){publish(out);return;}
    }
    auto p=odom_->pose.pose.position;auto q=odom_->pose.pose.orientation;
    if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)
       ||!std::isfinite(q.x+q.y+q.z+q.w)
       ||std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.01){publish(out);return;}
    const auto &goal=path_->poses.back().pose.position;
    if(std::hypot(p.x-goal.x,p.y-goal.y)<arrival_tolerance_){publish(out,true);return;}
    auto distance=[&](size_t i){auto &t=path_->poses[i].pose.position;return std::hypot(t.x-p.x,t.y-p.y);};
    // Monotonic closest point then arc-length lookahead. Collision/stopping
    // approval remains a separate map-backed gate in the comparison harness.
    while(index_+1<path_->poses.size()&&distance(index_+1)<=distance(index_))++index_;
    size_t look=index_;double arc=0;
    while(look+1<path_->poses.size()&&arc<.65){
      auto a=path_->poses[look].pose.position;auto b=path_->poses[++look].pose.position;
      arc+=std::hypot(b.x-a.x,b.y-a.y);
    }
    auto target=path_->poses[look].pose.position;
    if(!target_valid_||std::hypot(target.x-target_x_,target.y-target_y_)>.10
       ||(look+1==path_->poses.size()&&target_index_!=look)){
      // FLU odom to the legacy local FRD/NED numeric convention, fixed z.
      controller_.setWaypointSequence({{float(target.x),float(-target.y),float(-p.z),"A* lookahead"}});
      target_x_=target.x;target_y_=target.y;target_valid_=true;target_index_=look;
    }
    VelocityAPFController::InputData in{};
    in.current_x=p.x;in.current_y=-p.y;in.current_z=-p.z;
    in.current_yaw=-std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    in.scan_data=scan_;in.avoidance_enabled=true;
    auto v=controller_.calculateVelocity(in);
    if(std::isfinite(v.vx)&&std::isfinite(v.vy)){
      double vx=v.vx,vy=-v.vy;
      if(path_guided_){auto guided=path_guided_velocity(target.x-p.x,target.y-p.y,vx,vy);vx=guided[0];vy=guided[1];}
      double speed=std::hypot(vx,vy);
      double scale=speed>.15?std::nextafter(.15,0.)/speed:1.;
      out.linear.x=vx*scale;out.linear.y=vy*scale;
    }
    // Fixed heading and altitude are owned by the shadow message pipeline.
    publish(out,std::isfinite(v.vx)&&std::isfinite(v.vy));
  }
  VelocityAPFController controller_;
  bool path_guided_=false;
  bool real_inputs_=false;
  double arrival_tolerance_=.10;
  nav_msgs::msg::Path::SharedPtr path_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr candidate_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t index_=0,target_index_=0;bool target_valid_=false;double target_x_=0,target_y_=0;
};

int main(int argc,char **argv){
  const char *domain=std::getenv("ROS_DOMAIN_ID");
  if(!apf_domain::entry_allowed(domain))return 64;
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<LegacyShadow>());
  rclcpp::shutdown();return 0;
}
