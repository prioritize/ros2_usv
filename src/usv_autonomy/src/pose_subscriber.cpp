#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class PoseSubscriber : public rclcpp::Node {
public:
  PoseSubscriber() : Node("pose_subscriber") {
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/wamv/pose", 10,
        std::bind(&PoseSubscriber::pose_callback, this, std::placeholders::_1));
  }

private:
  void
  pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "x: %.2f, y: %.2f", msg->pose.position.x,
                msg->pose.position.y);
  }
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      subscription_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseSubscriber>());
  rclcpp::shutdown();
  return 0;
}
