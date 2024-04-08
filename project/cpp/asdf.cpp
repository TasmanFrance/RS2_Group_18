#include <iostream>
#include <vector>
#include <stack>
#include <set>
#include <queue>
#include <map>
#include <algorithm>

#include "asdf.h"

#include <algorithm>

Asdf::Asdf() {}

//Printing the graph
void Asdf::print_adjacency_list(std::vector<std::vector<std::pair<int,double>>> graph)
{
    // If stack is empty then return
    if (graph.empty())
        return;

    unsigned int nodeNum=0;
    for(auto edges : graph){
        std::cout << "[" << nodeNum++ << "] ";
        for(auto nodes : edges){
            std::cout << nodes.first << ":" << nodes.second << " ";
        }
        std::cout << std::endl ;
    }   
}

//Printing the path
void Asdf::print_path(std::vector<std::vector<std::pair<int,double>>> graph, std::vector<int> path)
{
    // If graph is empty then return
    if (graph.empty())
        return;

    // If path is empty then return
    if (path.empty())
        return;

    for(unsigned int i=0;i<path.size()-1;i++){
        unsigned int node1 = path.at(i);
        unsigned int node2 = path.at(i+1);
        std::cout << "[" << node1 << "] to " ;
        for(unsigned int j=0;j<graph.at(node1).size();j++){
            if(graph.at(node1).at(j).first==node2){
                std::cout << "[" << graph.at(node1).at(j).first << "] = " << 
                            graph.at(node1).at(j).second << std::endl;
                break;
            }
        }
    }   
}


std::vector<int> Asdf::brute_force_tsp(std::vector<std::vector<std::pair<int,double>>>& graph, int start) {

    std::vector<int> order;
    //Let's list the nodes in a vector, and we can do permutations of
    //it as all of the nodes are connected
    std::vector<int> nodes;
    for (unsigned int i=0;i< graph.size();i++){
        nodes.push_back(i);
    }

    //We save the total path as a very large value (default)
    double minDistance = 1e6;

    //This while loop creates all possible permutations of the nodes
    //We can use this while loop to create an order of ID's visited
    //Let's look for te total path to visit nodes in current node order 
    do
    {
        bool OK=true;//We will use this to abolish search if needed
        unsigned int idx=1;//Let's start from index 1 to end 
        double dist=0; // Current distance that we have travelled throug nodes
        while((idx<nodes.size())&&(OK)){
            //We have two nodes 
            unsigned int node1 = nodes.at(idx-1);
            unsigned int node2 = nodes.at(idx);
            //We find in adjacency list node 1 connection to node2 and second element
            //in teh pair is the distance between these nodes
            dist+=graph.at(node1).at(node2).second;
            // std::cout << dist << " "; // This printed distance in debug mode
            //we can abolish search if we are already over the min distance
            if(dist>minDistance){
                OK=false;
            }
            idx++; // Otherwise we increment to next node
        }
        std::cout << std::endl;
        if(dist<minDistance){
            minDistance=dist; // Save minimum distance
            order.clear(); // clear the current order of nodes
            order=nodes; // Save the order of nodes
        }
    } 
    while (std::next_permutation(nodes.begin(), nodes.end()));

    return order;
}
