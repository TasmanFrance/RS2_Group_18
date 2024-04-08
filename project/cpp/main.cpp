#include <iostream>
#include <vector>
#include "asdf.h"

int main(int, char**){
    std::cout << "Hello, from Jacooooob!\n";

    Asdf inst;

    //std::cout << inst.getDescription() << std::endl;



// Build the graph - Added here self connections to make it easier for our search
    AdjacencyList graph = {
        {{0,0},{1,50},{2,80},{3,40}},       // Node 0 is connected to nodes 0,1,2 and 3 (via edges with weights)
        {{0,50},{1,0},{2,10},{3,100}},      // Node 1 is connected to nodes 0,1,2 and 3 (via edges)
        {{0,80},{1,10},{2,0},{3,20}},       // Node 2 is connected to ....
        {{0,40},{1,100},{2,20},{3,0}},      // Node 3 is connected to ....
    };

    std::vector<int> path;

    std::cout << "Adjacency list: " << std::endl;
    inst.print_adjacency_list(graph);

    path = inst.brute_force_tsp(graph, 0);
    std::cout << "Path found by tsp search: " << std::endl;
    inst.print_path(graph,path);

    return 0;
}
