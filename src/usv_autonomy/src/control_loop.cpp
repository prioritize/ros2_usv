#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.h"
#include <deque>
#include <functional>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <vector>
#include <cmath>
#include <numbers>



class Controller : public rclcpp::Node
{
public:
  Controller() : Node("controller") 
  {
     goal_pos_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/goal_pose", rclcpp::SensorDataQoS(), std::bind(&Controller::goal_pose_callback, this, std::placeholders::_1));
    gps_fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/wamv/sensors/gps/gps/fix", rclcpp::SensorDataQoS(), std::bind(&Controller::gps_fix_callback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/wamv/sensors/imu/imu/data", rclcpp::SensorDataQoS(), std::bind(&Controller::imu_callback, this, std::placeholders::_1));
    left_thruster_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thusters/left/thrust", 10);
    right_thruster_pub_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thusters/right/thrust", 10);
  }
private:
  // need a goal subscription
  // need a position subscription
  // need a heading subscription 
  // need to calcuate velocity or subscribe to it
  //
  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr goal_pos_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thruster_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thruster_sub_;

  // goals
  float to_radians_ = std::numbers::pi/180;
  float to_degrees_ = 180 / std::numbers::pi;
  float goal_lat_;
  float goal_lon_;
  float goal_lat_rad_;
  float goal_lon_rad_;
  
  sensor_msgs::msg::NavSatFix goal_pos_;
  float goal_heading_;
  float goal_heading_rad_;
  float goal_speed_;
  bool goal_set_ = false;
  float yaw_;
  
  // gains
  float p_ = 0;
  float i_ = 0;
  float d_ = 0;


  // imu data
  geometry_msgs::msg::Quaternion orientation_;


  // positions
  std::vector<float> lat_;
  std::vector<float> lon_;
  sensor_msgs::msg::NavSatFix current_pos_;
  float current_lat_rad_;
  float current_lon_rad_;
  std::deque<float> gps_history_;
  const size_t max_history_size_ = 50;

  void goal_pose_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    goal_lat_ = msg->latitude;
    goal_lon_ = msg->longitude;
    goal_lat_rad_ = msg->latitude*to_radians_;
    goal_lon_rad_ = msg->longitude*to_radians_;
    goal_pos_ = *msg;
    goal_set_ = true;
  }
  void gps_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "lat:%.2f, lon:%.2f", msg->latitude, msg->longitude);
    current_pos_ = *msg;
    current_lat_rad_ = msg->latitude*to_radians_;
    current_lon_rad_ = msg->longitude*to_radians_;
    if(goal_set_)
    {
      calc_heading();
    }
  }
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    orientation_ = msg->orientation;
    yaw_ = std::atan2( 2*(orientation_.w * orientation_.z + orientation_.x*orientation_.y), (1 - 2*(orientation_.y*orientation_.y + orientation_.z * orientation_.z)) );
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500, "x: %.2f, y: %.2f, z:%.2f", orientation_.x, orientation_.y, orientation_.z);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500, "yaw: %.2f", yaw_*to_degrees_);
  }
  void calc_heading()

  {
    float delta_lat = goal_lat_rad_ - current_lat_rad_;
    float delta_lon = goal_lon_rad_- current_lon_rad_;
    goal_heading_rad_ = std::atan2(sin(delta_lon) * cos(goal_lat_rad_),
                      cos(current_lat_rad_)*sin(goal_lat_rad_) -
                      sin(current_lat_rad_)*cos(goal_lat_rad_)*cos(delta_lon));
    goal_heading_ = goal_heading_rad_*to_degrees_;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500, "delta_lat: %.2f, delta_lon: %.2f", delta_lat, delta_lon);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500, "current_lat: %.2f, current_lon: %.2f", current_pos_.latitude, current_pos_.longitude);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500, "goal_lat: %.2f, goal_lon: %.2f", goal_pos_.latitude, goal_pos_.longitude);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500, "goal_heading_: %.2f", goal_heading_);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500, "goal_heading_rad_: %.2f", goal_heading_rad_);
    
  }


};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}
