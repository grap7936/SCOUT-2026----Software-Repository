#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include <omp.h>
#include "Target.hpp"

class Graph {
private:
    int root_id; // id of root node
    int root_x, root_y;
    int root_nx, root_ny;

    std::vector<Target*> targets; // vertex instance
    std::vector<int> weights; // stores weight of vertex with corresponding index

public:
    
    // Constructor with root target and target list
    Graph( Target &root, const std::vector<Target*>& targets );

    // Constructor with root target
    Graph( Target &root );

    // Backup Constructor
    Graph();

    void rebuild( Target &root, const std::vector<Target*>& next );


    int getRootID(); // new

    int getRootX(); // new

    int getRootY(); // new

    int getRootNX(); // new

    int getRootNY(); // new

    int getVertexID(size_t index);

    int getVertexWeight(size_t index);

    int getVertexWeightByID(int id); 

    Target* getVertexPtr(int index);

    Target* getVertexPtrByID(int id);

    std::vector<Target*> getTargets();

    std::vector<int> getWeights();


    void addVertex(Target*);

    void addVertex(Target*, int weight); // new

    void addVerticesFromList(const std::vector<Target*>& next_targets);

    void addVerticesFromList(const std::vector<Target*>& next_targets, const std::vector<int>& weight); // new

    void calcWeight(float gain1);

    void calcWeightOMP(float gain1);

    void sortByWeight();

};

#endif