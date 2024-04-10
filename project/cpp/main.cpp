#include <iostream>
#include <vector>
#include <iomanip>   //#include "asdf.h"
#include <limits>

#include "graph.h"
#include "greedy.h"



int main(int, char**){
    std::cout << "Hello, from Jacooooob!\n";

    //Asdf inst;
    Graph inst;

    // Given Cartesian points
    std::vector<point_ref::Point> points = {
        {5, 5}, {5, 3}, {1, 5}, {3, 6}, {3, 7}, {8, 7}, {8, 6}, {10, 5}
    };

      // Build the graph
    Graph::AdjacencyList adjacencyList = inst.buildGraph(points);

    // Print the adjacency list with distances rounded to two decimal places
    for (int i = 0; i < adjacencyList.size(); ++i) {
        std::cout << "Node " << i << ":";
        for (const auto& edge : adjacencyList.at(i)) {
            std::cout << " Node " << edge.destination << " [weight: "<< std::fixed << std::setprecision(2) << edge.weight << "]";
        }
        std::cout << std::endl;
    }

    // Create a TSP solver instance
    Greedy tspSolver(adjacencyList);

    // Specify the start node
    int startNode = 2; // Starting from node 0

    // Solve TSP
    auto [path, totalDistance] = tspSolver.solve(startNode);

    // Print the path
    std::cout << "TSP Path starting from node " << startNode << ":\n";
    for (int node : path) {
        std::cout << node << " ";
    }
    std::cout << "\nTotal distance traveled: " << totalDistance << std::endl;

    return 0;
}
