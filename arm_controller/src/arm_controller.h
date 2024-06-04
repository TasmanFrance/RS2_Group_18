#ifndef ARMCONTROLLER_H
#define ARMCONTROLLER_H

#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <std_msgs/msg/bool.hpp>


///////////////////////////////////////////////////////////////////////////////


class ArmController : public rclcpp::Node
{


/*---------------------------------------------------------------------------*/


public:
    ArmController();

    ~ArmController();


/*---------------------------------------------------------------------------*/


private:
//Thread
    std::thread* thread_;
    void threadRun();
//Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_armReady_;
//Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_targetPoses_;
//Pub / Sub Variables
    geometry_msgs::msg::PoseArray targetPoses_;
    std::atomic<bool> runPath_;
//Pub / Sub Mutex
    std::mutex targetPosesMtx_;
// Callback functions
    void target_callback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg);
//Functions
    void setNextPose(geometry_msgs::msg::Pose pose);
    // bool moveToNextPose();
//Moveit constant variables
    // moveit::planning_interface::MoveGroupInterface move_group_interface_;
    const std::string PLANNING_GROUP_ARM_ = "ur_manipulator";
};


///////////////////////////////////////////////////////////////////////////////


#endif // ARMCONTROLLER_H