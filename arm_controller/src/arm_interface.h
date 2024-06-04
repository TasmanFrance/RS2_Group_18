#ifndef ARMINTERFACE_H
#define ARMINTERFACE_H

#include <chrono>
#include <thread>
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


///////////////////////////////////////////////////////////////////////////////


class ArmInterface : public rclcpp::Node
{


/*---------------------------------------------------------------------------*/


public:
    ArmInterface();

    ~ArmInterface();


/*---------------------------------------------------------------------------*/


private:
//marker types
    enum markerType
    {
        TPP_PEN_UP,
        TPP_PEN_DOWN,
        LOCAL_CANVAS_CENTRE
    };
//Thread
    std::thread* thread_;
    void threadProcess(void);
    std::atomic<int> threadSleepSec_;
//Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_targetPoses_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_vis_marker_;
//Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_centrePage_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_drawPoses_;
// Callback functions
    void canvas_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg);
    void draw_callback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg);
//Pub / Sub Variables
    geometry_msgs::msg::PoseStamped canvasPose_;
    geometry_msgs::msg::PoseStamped targetPose_;
    geometry_msgs::msg::PoseArray drawPoses_;
//Pub / Sub Mutex
    std::mutex canvasPoseMtx_;
    std::mutex targetPoseMtx_;
    std::mutex drawPosesMtx_;
//Marker functions and variables
    visualization_msgs::msg::MarkerArray addMarker( const geometry_msgs::msg::PoseArray &poses, markerType type);
    visualization_msgs::msg::MarkerArray markers_;
    unsigned int markerCount_;
    double markerTolerance_;
    bool drawPathUpdated_;
    bool canvasCentreUpdated_;
//Coordinate translation functions, buffer and listener
    geometry_msgs::msg::PoseArray poseArrayTransform(const std::string &currentFrameID, const std::string &desiredFrameID, const geometry_msgs::msg::PoseArray &poses, const double &scaling, const double &zHeight);
    geometry_msgs::msg::PoseStamped poseStampedTransform(const std::string &currentFrameID, const std::string &desiredFrameID, const geometry_msgs::msg::PoseStamped &pose);
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::atomic<double> canvasDist_;
    std::mutex canvasDistMtx_;

    // geometry_msgs::msg::Point nextWaypoint_;
    // std::Mutex nextwaypointMtx_;
    // std::unique_ptr<ArmController> armControllerPtr_;
};


///////////////////////////////////////////////////////////////////////////////


#endif // ARMINTERFACE_H