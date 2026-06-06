#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <Target.hpp>

class Graph {

public:
    int root; // id of root node
    std::vector<Target*> target; // vertex instance
    std::vector<float> weight; // stores weight of vertex with corresponding index
    
    // Constructor
    Graph( int input_id );

    // Backup Constructor
    Graph();

    void addVertex(Target*, float);

    Target* getVertexPtr(int);

    Target* getVertexPtrByID(int);

    float getVertexWeight(int);

    float getVertexWeightByID(int);

    void sortByWeight();

};

#endif