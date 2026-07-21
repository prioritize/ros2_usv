#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "rclcpp/rclcpp.hpp"

class PoseSubscriber : public rclcpp::Node {
public:
  PoseSubscriber() : Node("pose_subscriber") {
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
        // RCLCPP_INFO(this->get_logger(),
        //             "%s -> %s : x=%.2f y=%.2f z=%.2f",
        //             tf.header.frame_id.c_str(),
        //             tf.child_frame_id.c_str(),
        //             tf.transform.translation.x,
        //             tf.transform.translation.y,
        //             tf.transform.translation.z);
    }
    // RCLCPP_INFO(this->get_logger(), "x: %.2f, y: %.2f", msg->transforms[],
    //             msg->pose.position.y);
  }
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr
      subscription_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseSubscriber>());
  rclcpp::shutdown();
  return 0;
}
