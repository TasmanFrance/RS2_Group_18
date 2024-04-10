#include "graph.h"
#include <cmath>
#include <iostream>

Graph::Graph() {}

// Function to calculate the distance between two points
double distance(const point_ref::Point& p1, const point_ref::Point& p2) {
    return sqrt(pow((p2.x - p1.x), 2) + pow((p2.y - p1.y), 2));
}

// Function to build the adjacency list from XY Cartesian coordinates
Graph::AdjacencyList Graph::buildGraph(std::vector<point_ref::Point>& points) {
    int n = points.size();
    AdjacencyList graph(n);

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i != j) {
                double dist = distance(points.at(i), points.at(j));
                dist = std::round(dist * 100) / 100.0;
                graph.at(i).push_back({j, dist});
            }
        }
    }
    return graph;
}
