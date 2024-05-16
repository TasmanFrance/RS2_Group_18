#include "greedy.h"
#include <limits>

Greedy::Greedy(Graph::AdjacencyList& graph) : graph_(graph) {}

std::pair<std::vector<int>, double> Greedy::solve(int startNode) {
    int numNodes = graph_.size();
    std::vector<int> path;
    std::vector<bool> visited(numNodes, false);
    path.reserve(numNodes);
    double totalDistance = 0.0;

    // Start with the specified start node
    int currentNode = startNode;
    path.push_back(currentNode);
    visited[currentNode] = true;

    // Continue until all nodes are visited
    while (path.size() < numNodes) {
        int nextNode = getNextNode(currentNode, visited);
        totalDistance += getEdgeWeight(currentNode, nextNode);
        path.push_back(nextNode);
        visited[nextNode] = true;
        currentNode = nextNode;
    }

    // Return to the starting node
    totalDistance += getEdgeWeight(path.back(), startNode);
    path.push_back(startNode);

    return {path, totalDistance};
}

int Greedy::getNextNode(int currentNode, const std::vector<bool>& visited) {
    int nextNode = -1;
    double minWeight = std::numeric_limits<double>::max();

    for (const auto& edge : graph_[currentNode]) {
        if (!visited[edge.destination] && edge.weight < minWeight) {
            minWeight = edge.weight;
            nextNode = edge.destination;
        }
    }

    return nextNode;
}

double Greedy::getEdgeWeight(int node1, int node2) {
    for (const auto& edge : graph_[node1]) {
        if (edge.destination == node2) {
            return edge.weight;
        }
    }
    return std::numeric_limits<double>::infinity(); // Edge not found, return infinity
}
