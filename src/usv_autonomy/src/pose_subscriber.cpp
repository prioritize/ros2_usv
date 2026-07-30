#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>

class PoseSubscriber : public rclcpp::Node {
public:
  PoseSubscriber() : Node("pose_subscriber") {
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/wamv/sensors/gps/gps/fix", rclcpp::SensorDataQoS(), std::bind(&PoseSubscriber::gps_callback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/wamv/sensors/imu/imu/data", rclcpp::SensorDataQoS(), std::bind(&PoseSubscriber::imu_callback, this, std::placeholders::_1));
    goal_pose_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/usv_autonomy/goal_gps", rclcpp::SensorDataQoS(), std::bind(&PoseSubscriber::goal_callback, this, std::placeholders::_1));
    subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/wamv/pose", 10,
        std::bind(&PoseSubscriber::pose_callback, this, std::placeholders::_1));
  }

private:
  void pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) const {
    for (const auto & tf : msg->transforms) {
      if(tf.child_frame_id == "wamv/wamv/base_link") {
        RCLCPP_INFO(this->get_logger(),
                    "x=%.2f y=%.2f z=%.2f",
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z);

      }
    }
  }
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const
  {
        RCLCPP_INFO(this->get_logger(), "lat=%.6f lon%.6f alt=%.2f",
          msg->latitude, msg->longitude, msg->altitude);
  }
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
  {
      RCLCPP_INFO(this->get_logger(), "orientation: x=%.2f y=%.2f z=%.2f w=%.2f", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  }
  void goal_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "goal_pose: lat=%.6f, lon=%.6f alt=%.2f", msg->latitude, msg->longitude, msg->altitude);
    goal_lat_ = msg->latitude;
    goal_lon_ = msg->longitude;
    has_goal_ = true;
  }

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr goal_pose_sub_;
  
  double goal_lat_ {0.0};
  double goal_lon_ {0.0};
  double has_goal_{false};
  
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseSubscriber>());
  rclcpp::shutdown();
  return 0;
}
