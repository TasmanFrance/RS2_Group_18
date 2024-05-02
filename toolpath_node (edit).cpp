#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include "graph.h"
#include "greedy.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "toolpath_node");
    ros::NodeHandle nh;

    // Publisher for the Point messages
    ros::Publisher point_pub = nh.advertise<geometry_msgs::Point>("toolpath_points", 10);

    // Create a Rate object to control the publishing rate
    ros::Rate rate(1); // Publish at 1 Hz

    std::cout << "Node graph\n";

    Graph inst;

    // node points
    std::vector<point_ref::Point> points = {
        {5, 5}, {5, 3}, {1, 5}, {3, 6}, {3, 7}, {8, 7}, {8, 6}, {10, 5}
    };

    Graph::AdjacencyList adjacencyList = inst.buildGraph(points);

    Greedy tspSolver(adjacencyList);

    int startNode = 2; // Starting from node 0

    while (ros::ok()) {
        // Solve TSP
        auto [path, totalDistance] = tspSolver.solve(startNode);

        // Publish each point in the path
        for (int node : path) {
            geometry_msgs::Point point;
            point.x = points[node].x;
            point.y = points[node].y;
            point.z = 0; // Assuming 2D points

            point_pub.publish(point);

            // Sleep for a while to control the publishing rate
            rate.sleep();
        }

        // Log total distance
        ROS_INFO("Total distance traveled: %f", totalDistance);

        ros::spinOnce(); // Handle ROS callbacks
    }

    return 0;
}