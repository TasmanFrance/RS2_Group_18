#include "processor.h"
#include <sstream>
#include <iostream>
#include <iomanip>

std::vector<Point> parsePath(const std::string& pathData) {
    std::vector<Point> points;
    std::istringstream iss(pathData);
    int x, y;
    while (iss >> x >> y) {
        points.push_back({x, y, 1, true}); // Set z = 1 and active = true initially
    }

    // Repeat the last point with active = false
    if (!points.empty()) {
        Point lastPoint = points.back();
        lastPoint.z = 0; // Set z = 0 for the last point to indicate inactive
        lastPoint.active = false;
        points.push_back(lastPoint);
    }

    return points;
}

Processor::Processor(const std::vector<std::string>& paths) {
    for (int i = 0; i < paths.size(); ++i) {
        std::vector<Point> points = parsePath(paths.at(i));
        nodedata.push_back({i, points});
    }

    adjacencyList = Graph().buildGraph(nodedata);
}

std::pair<std::vector<int>, double> Processor::processPath(int startNode) {
    Greedy tspSolver(adjacencyList);
    auto [path, totalDistance] = tspSolver.solve(startNode);
    return {path, totalDistance};
}

const std::vector<std::pair<int, std::vector<Point>>>& Processor::getNodeData() const {
    return nodedata;
}