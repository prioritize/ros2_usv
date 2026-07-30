#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "rclcpp/rclcpp.hpp"
#include <deque>
#include <functional>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <vector>



class Controller : public rclcpp::Node
{
public:
  Controller() : Node("controller") 
  {
     goal_pos_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/goal_pose", rclcpp::SensorDataQoS(), std::bind(&Controller::goal_pose_callback, this, std::placeholders::_1));
    gps_fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/wamv/sensors/gps/gps/fix", rclcpp::SensorDataQoS(), std::bind(&Controller::gps_fix_callback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/wamv/sensors/imu/imu/data", rclcpp::SensorDataQoS(), std::bind(&Controller::imu_callback, this, std::placeholders::_1));
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

  // goals
  float goal_lat_;
  float goal_lon_;
  sensor_msgs::msg::NavSatFix goal_pos_;
  float goal_heading_;
  float goal_speed_;
  bool goal_set_ = false;
  
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
  std::deque<float> gps_history_;
  const size_t max_history_size_ = 50;

  void goal_pose_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    goal_lat_ = msg->latitude;
    goal_lon_ = msg->longitude;
    goal_pos_ = *msg;
    goal_set_ = true;
  }
  void gps_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "lat:%.2f, lon:%.2f", msg->latitude, msg->longitude);
    if(goal_set_)
    {
      calc_heading(current_pos_, goal_pos_);
    }
  }
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    orientation_ = msg->orientation;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "x: %.2f, y: %.2f, z:%.2f", orientation_.x, orientation_.y, orientation_.z);
  }
  float calc_heading(sensor_msgs::msg::NavSatFix current, sensor_msgs::msg::NavSatFix goal)
  {
    float delta_lat = goal.latitude - current.latitude;
    float delta_lon = goal.longitude - current.longitude;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "delta_lat: %.2f, delta_lon: %.2f", delta_lat, delta_lon);
    return 1.0;
  }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}
