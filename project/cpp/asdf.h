#ifndef ASDF_H
#define ASDF_H

#include <string>
#include <vector>

/*!
 *  \ingroup   ac_shapre Shape
 *  \brief     Shape base class
 *  \details
 *  This class is the base class for all shapes.\n
 */

typedef std::vector<std::vector<std::pair<int, double>>> AdjacencyList;

class Asdf
{
public:
    Asdf();
    void print_adjacency_list(std::vector<std::vector<std::pair<int,double>>> graph);
    void print_path(std::vector<std::vector<std::pair<int,double>>> graph, std::vector<int> path);
    std::vector<int> brute_force_tsp(std::vector<std::vector<std::pair<int,double>>>& graph, int start);

    
protected:
    
private:

};

#endif // ASDF_H