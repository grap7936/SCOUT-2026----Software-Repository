#include <Graph.hpp>

// Constructor
Graph::Graph( int input_id ) {
    root = input_id;
};

// Backup Constructor
Graph::Graph() {
    root = -1;
};

void Graph::addVertex( Target* vertex, float vertex_weight ) {
    target.push_back( vertex );
    weight.push_back( vertex_weight );
}

Target* Graph::getVertexPtr( int index ){
    return target[index];
}

Target* Graph::getVertexPtrByID( int vertex_id ){
    int size = target.capacity();
    for ( int i = 0; i < size; i++ ) {
        if ( target[i]->id == vertex_id ) {
            return target[i];
        }
    }
    return NULL;
}

float Graph::getVertexWeight( int index ){
    if (index >= weight.capacity()) {
        return -2;
    }
    return weight[index];
}

float Graph::getVertexWeightByID( int vertex_id ){
    int size = target.capacity();
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
    int size = weight.capacity();
    // duplicate
    std::vector<float> copy;

    // iterate through
    for ( int i = 1; i < size; i++ ) {
        // update duplicate
        copy = weight;
        
        // compare to previous elements to find index
        int index = i;
        while (weight[i] > weight[index-1] && index > 0) {
            index--;
        }

        // insert
        weight[index] = copy[i];
        for ( int k = 0; k < i - index; k++) {
            weight[index+k+1] = copy[index+k];
        }
    }
}