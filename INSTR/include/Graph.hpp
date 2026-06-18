#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include "Target.hpp"

class Graph {

public:
    int root; // id of root node
    std::vector<Target*> target; // vertex instance
    std::vector<int> weight; // stores weight of vertex with corresponding index
    
    // Constructor
    Graph( int input_id );

    // Backup Constructor
    Graph();

    // Destructor
    ~Graph();

    void addVertex(Target*, float);

    Target* getVertexPtr(int);

    Target* getVertexPtrByID(int);

    int getVertexID(size_t);

    int getVertexWeight(size_t);

    int getVertexWeightByID(int);

    void sortByWeight();

};

#endif