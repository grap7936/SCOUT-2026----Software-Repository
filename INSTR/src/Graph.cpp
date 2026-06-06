#include <Graph.hpp>

// Constructor
Graph::Graph( int input_id ) {
    root = input_id;
}

// Backup Constructor
Graph::Graph() {
    root = -1;
}

// Destructor
Graph::~Graph() {
    while ( target.size() > 0){
        target.pop_back();
    }
    while ( weight.size() > 0){
        weight.pop_back();
    }
}

void Graph::addVertex( Target* vertex, float vertex_weight ) {
    target.push_back( vertex );
    weight.push_back( vertex_weight );
}

Target* Graph::getVertexPtr( int index ){
    return target[index];
}

Target* Graph::getVertexPtrByID( int vertex_id ){
    int size = target.size();
    for ( int i = 0; i < size; i++ ) {
        if ( target[i]->id == vertex_id ) {
            return target[i];
        }
    }
    return NULL;
}

int Graph::getVertexID( int index ){
    if (index >= target.size()) {
        return -2;
    }
    return target[index]->id;
}

float Graph::getVertexWeight( int index ){
    if (index >= weight.size()) {
        return -2;
    }
    return weight[index];
}

float Graph::getVertexWeightByID( int vertex_id ){
    int size = target.size();
    for ( int i = 0; i < size; i++ ) {
        if ( target[i]->id == vertex_id ) {
            return weight[i];
        }
    }
    return -1;
}

// I think this would be an Insert Sort algorithm?
void Graph::sortByWeight() {
    // vector size
    int size = weight.size();
    // duplicate
    std::vector<float> copy_weight;
    std::vector<Target*> copy_target;

    // iterate through
    for ( int i = 1; i < size; i++ ) {
        // update duplicate
        copy_weight = weight;
        copy_target = target;
        
        // compare to previous elements to find index
        int index = i;
        while (weight[i] > weight[index-1] && index > 0) {
            index--;
        }

        // insert
        weight[index] = copy_weight[i];
        target[index] = copy_target[i];
        for ( int k = 0; k < i - index; k++) {
            weight[index+k+1] = copy_weight[index+k];
            target[index+k+1] = copy_target[index+k];
        }
    }
}