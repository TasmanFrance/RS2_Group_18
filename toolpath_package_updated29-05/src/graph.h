#ifndef GRAPH_H
#define GRAPH_H

#include <vector>

struct Point {
    int x;
    int y;
    int z; // Add the z-coordinate
    bool active; // Add the active/inactive state
};

class Graph {
public:
    Graph();

    struct Edge {
        int destination;
        double weight;
    };

    using AdjacencyList = std::vector<std::vector<Edge>>;

    AdjacencyList buildGraph(const std::vector<std::pair<int, std::vector<Point>>>& nodedata);
};

#endif // GRAPH_H
