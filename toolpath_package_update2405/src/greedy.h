#ifndef GREEDY_H
#define GREEDY_H

#include "graph.h"

class Greedy {

public:
    Greedy(Graph::AdjacencyList& graph);
    std::pair<std::vector<int>, double> solve(int startNode);

private:
    Graph::AdjacencyList& graph_;
    int getNextNode(int currentNode, const std::vector<bool>& visited);
    double getEdgeWeight(int node1, int node2);
};

#endif // GREEDY_H
