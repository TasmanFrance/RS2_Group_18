#include <functional>
#include <memory>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

class PathTransformer : public rclcpp::Node
{
public:
  PathTransformer()
  : Node("path_transformer")
  {
    camera_frame_origin_received_ = false;

    pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/poses", 10, std::bind(&PathTransformer::pose_callback, this, _1));

    marker_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/centerpage_marker", 10, std::bind(&PathTransformer::marker_callback, this, _1));

    transformed_path_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("transformed_path", 10);
  }

private:
  void marker_callback(const geometry_msgs::msg::PoseStamped& msg)
  {
    camera_frame_origin_ = msg.pose;
    camera_frame_origin_received_ = true;
  }

void pose_callback(const geometry_msgs::msg::PoseArray& msg)
{
    if (!camera_frame_origin_received_) {
        // Wait for the camera frame origin
        return;
    }

    // Define the origin of the path data frame
    geometry_msgs::msg::Pose path_data_origin;
    path_data_origin.position.x = 0.0;  // Set the desired origin for x
    path_data_origin.position.y = 0.0;  // Set the desired origin for y
    path_data_origin.position.z = 0.0;  // Set the desired origin for z

    // Calculate the transformation (translation and rotation)
    Eigen::Vector2d translation(camera_frame_origin_.position.x - path_data_origin.position.x,
                                camera_frame_origin_.position.y - path_data_origin.position.y);

    Eigen::Rotation2D<double> rotation(camera_frame_origin_.orientation.z);

    // Scale the path data
    double scaling_factor = 0.01; // Example: path data in meters, camera frame in centimeters

    // Transform and publish the path data
    geometry_msgs::msg::PoseArray transformed_path;
    for (const auto& pose : msg.poses) {
        Eigen::Vector2d original_position(pose.position.x, pose.position.y);
        Eigen::Vector2d scaled_position = (original_position - Eigen::Vector2d(path_data_origin.position.x,
                                                                               path_data_origin.position.y)) * scaling_factor;
        //Eigen::Vector2d transformed_position = rotation * scaled_position + translation;
        Eigen::Vector2d transformed_position = scaled_position + translation;

        geometry_msgs::msg::Pose transformed_pose;
        transformed_pose.position.x = transformed_position.x();
        transformed_pose.position.y = transformed_position.y();
        transformed_pose.position.z = pose.position.z; // Keep the original z coordinate
        // Set the orientation if needed
        transformed_path.poses.push_back(transformed_pose);
    }

     // Publish the transformed path data
    RCLCPP_INFO(this->get_logger(), "Publishing transformed paths to /transformed_path topic");
    transformed_path_publisher_->publish(transformed_path);
}
  geometry_msgs::msg::Pose camera_frame_origin_;
  bool camera_frame_origin_received_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr transformed_path_publisher_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathTransformer>());
  rclcpp::shutdown();
  return 0;
}