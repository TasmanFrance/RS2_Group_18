#include "processor.h"
#include <sstream>
#include <iostream>
#include <iomanip>

std::vector<Point> parsePath(const std::string& pathData) {
    std::vector<Point> points;
    std::istringstream iss(pathData);
    int x, y;
    while (iss >> x >> y) {
        points.push_back({x, y});
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