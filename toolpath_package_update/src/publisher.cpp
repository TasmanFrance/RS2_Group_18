#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"


#include "processor.h"

using namespace std::chrono_literals;

class PathPublisher : public rclcpp::Node
{
public:
    PathPublisher()
    : Node("path_publisher"), count_(0)
    {
        // Define your paths here
       std::vector<std::string> paths = {
        "107 171 108 170 125 170 126 171 128 171 129 172",
        "76 138 76 141 75 142 75 147 73 149",
        "224 135 223 136 222 136 226 136 227 137 229 137 230 138 234 138 235 139",
        "197 132 197 133 196 134 197 134 198 133",
        "373 121 373 124 372 125 372 148 373 149 373 156 374 157"
        };  

        processor = std::make_unique<Processor>(paths); // the constructor of processor is called here

        publisher_.reset();

        //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("poses", 10);

        timer_ = this->create_wall_timer(
            5000ms, std::bind(&PathPublisher::timer_callback, this));

    }

private:

void timer_callback()
{
    auto message = geometry_msgs::msg::PoseArray();
    // Process the path and get the result
    int startNode = 2; // Change this to the desired start node
    auto [path, totalDistance] = processor->processPath(startNode);

    // Get the nodedata vector
    const auto& nodedata = processor->getNodeData();

    // std::cout << "TSP Path starting from node " << startNode << ":" << std::endl;

    // // Print the path vectors
    // for (int node : path) {
    //     const std::vector<Point>& points = nodedata[node].second;
    //     std::cout << "Node " << node << ": ";
    //     for (const Point& p : points) {
    //         std::cout << "(" << p.x << ", " << p.y << ", " << (p.active ? 1 : 0) << ") ";
    //     }
    //     std::cout << std::endl;
    // }

    // std::cout << "Total distance traveled: " << totalDistance << std::endl;

        // Construct the PoseArray message
        for (int node : path) {
            const std::vector<Point>& points = nodedata[node].second;
            for (const Point& p : points) {
                geometry_msgs::msg::Pose pose;
                pose.position.x = p.x;
                pose.position.y = p.y;
                pose.position.z = p.z;
                message.poses.push_back(pose);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Publishing path vectors to /poses topic");
        publisher_->publish(message);
    }


    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseArray>> publisher_;
    std::unique_ptr<Processor> processor;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher>());
    rclcpp::shutdown();
    return 0;
}
