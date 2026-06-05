#ifndef GRAPH_H
#define GRAPH_H

#include <vector>

class Graph {

public:
    int root; // id of root node
    std::vector<int> id; // vertex id
    std::vector<float> weight; // stores weight of vertex with corresponding index
    
    // Constructor
    Graph( int input_id );

    // Backup Constructor
    Graph();

    void addVertex(int, float);

    int getVertexID(int);

    float getVertexWeight(int);

    float getVertexWeightByID(int);

    void sortByWeight();

};

#endif