#ifndef PROCESSOR_H
#define PROCESSOR_H

#include <vector>
#include <utility>
#include <string>

#include "graph.h"
#include "greedy.h"

class Processor {
public:
    Processor(const std::vector<std::string>& paths);
    std::pair<std::vector<int>, double> processPath(int startNode);

private:
    std::vector<std::pair<int, std::vector<Point>>> nodedata;
    Graph::AdjacencyList adjacencyList;
};

#endif