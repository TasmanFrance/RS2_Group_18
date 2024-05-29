#include "graph.h"
#include <cmath>
#include <iostream>


Graph::Graph() {}

double distance(const Point& p1, const Point& p2) {
    return sqrt(pow((p2.x - p1.x), 2) + pow((p2.y - p1.y), 2));
}

Graph::AdjacencyList Graph::buildGraph(const std::vector<std::pair<int, std::vector<Point>>>& nodedata) {
    AdjacencyList graph;

    for (const auto& [nodeIndex, path] : nodedata) {
        if (!path.empty()) {
            graph.emplace_back(std::vector<Edge>()); // Add a new node for the current path
            int currentNode = nodeIndex;

            for (const auto& [otherNodeIndex, otherPath] : nodedata) {
                if (nodeIndex != otherNodeIndex && !otherPath.empty()) {
                    double dist = distance(path.back(), otherPath.front());
                    dist = std::round(dist * 100) / 100.0;
                    graph[currentNode].push_back({otherNodeIndex, dist}); // Add an edge to the other path
                }
            }
        }
    }

    return graph;
}