#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

class LocateSubscriber : public rclcpp::Node
{
public:
  LocateSubscriber()
  : Node("locate_subscriber")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/centerpage_marker", 10, std::bind(&LocateSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const geometry_msgs::msg::PoseStamped & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: position(x: %f, y: %f, z: %f), orientation(x: %f, y: %f, z: %f, w: %f)",
                msg.pose.posit2ion.x, msg.pose.position.y, msg.pose.position.z,
                msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
  }
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocateSubscriber>());
  rclcpp::shutdown();
  return 0;
}
