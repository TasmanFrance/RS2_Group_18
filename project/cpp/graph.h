#ifndef GRAPH_H
#define GRAPH_H

#include <vector>

namespace point_ref {
    // Struct to represent a point with Cartesian coordinates
    struct Point {
        int x;
        int y;
    };
}

class Graph {
public:
    Graph();

    // Struct to represent an edge between two nodes
    struct Edge {
        int destination; // Index of the destination node
        double weight;   // Weight of the edge
    };

    // Type alias for adjacency list representation of the graph
    using AdjacencyList = std::vector<std::vector<Edge>>;

    // Function to build the adjacency list from XY Cartesian coordinates
    AdjacencyList buildGraph(std::vector<point_ref::Point>& points);
};

#endif // GRAPH_H
