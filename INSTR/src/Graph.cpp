#include <Graph.hpp>

// Constructor initializing graph with a tracking root target ID
Graph::Graph( int input_id ) {
    root = input_id;
}

// Backup constructor fallback initializing an unlinked root (-1)
Graph::Graph() {
    root = -1;
}

// Destructor cleaning up internal tracking vectors to prevent memory fragmentation
Graph::~Graph() {
    // Pop elements sequentially to safely clear vector capacities
    while ( target.size() > 0){
        target.pop_back();
    }
    while ( weight.size() > 0){
        weight.pop_back();
    }
}

/* Function addVertex( Target* vertex, float vertex_weight )
 * description:
 *  Appends a new target destination and its associated proximity cost weight into parallel tracking vectors.
 * inputs:
 *  Target* vertex      - Pointer to the target destination node being evaluated.
 *  float vertex_weight - Calculated physical/Kalman Euclidean path distance score.
 * returns:
 *  void - Modifies the target and weight internal vectors in-place.
 */
void Graph::addVertex( Target* vertex, float vertex_weight ) {
    target.push_back( vertex );
    weight.push_back( vertex_weight );
}

// Returns the target pointer at a specific index location
Target* Graph::getVertexPtr( int index ){
    return target[index];
}

/* Function getVertexPtrByID( int vertex_id )
 * description:
 *  Linearly searches the target list to find and retrieve a target node matching a specific tracking ID.
 * inputs:
 *  int vertex_id - The unique integer identifier assigned to the desired target.
 * returns:
 *  Target* - Returns a pointer to the matching Target if found; otherwise returns nullptr.
 */
Target* Graph::getVertexPtrByID( int vertex_id ){
    int size = target.size();
    
    // Linear scan to locate the node containing the target ID match
    for ( int i = 0; i < size; i++ ) {
        if ( target[i]->id == vertex_id ) {
            return target[i]; // Return pointer to found node instance
        }
    }
    return nullptr; // Fallback indicator that no match exists in this graph
}

// Returns the target ID at a specific index with an out-of-bounds safety check
int Graph::getVertexID( size_t index ){
    if (index >= target.size()) {
        return -2; // Error boundary sentinel flag
    }
    return target[index]->id;
}

// Returns the assignment weight at a specific index with an out-of-bounds safety check
int Graph::getVertexWeight( size_t index ){
    if (index >= weight.size()) {
        return -2; // Error boundary sentinel flag
    }
    return weight[index];
}

/* Function getVertexWeightByID( int vertex_id )
 * description:
 *  Linearly searches the vertex tracking array to extract the proximity weight associated with a given target ID.
 * inputs:
 *  int vertex_id - The unique integer identifier assigned to the target node.
 * returns:
 *  int - Returns the scalar matching weight cost; returns -1 if the target ID is not in the graph.
 */
int Graph::getVertexWeightByID( int vertex_id ){
    int size = target.size();
    
    // Scan matching structures to determine proximity cost association values
    for ( int i = 0; i < size; i++ ) {
        if ( target[i]->id == vertex_id ) {
            return weight[i]; // Return the matching scalar value index pair
        }
    }
    return -1; // Out-of-bounds fallback signaling no target match detected
}

/* Function sortByWeight()
 * description:
 *  Executes an in-place Insertion Sort algorithm that reorganizes both the weight and target vectors simultaneously in ascending order, positioning the lowest-cost tracks first.
 * inputs:
 *  none
 * returns:
 *  void - Rearranges target and weight elements sequentially within internal container allocations.
 */
void Graph::sortByWeight() {
    int size = weight.size();
    
    // Duplicate snapshot arrays to hold state contexts while doing in-place shifting
    std::vector<int> copy_weight;
    std::vector<Target*> copy_target;

    // Insertion sort loop beginning at index 1 and parsing forward
    for ( int i = 1; i < size; i++ ) {
        
        // Refresh the backup mirrors with the current state of vectors before moving positions
        copy_weight = weight;
        copy_target = target;
        
        // Scan backwards through the sorted sub-list to find where element 'i' fits
        int index = i;
        while ( index > 0 && weight[i] < weight[index-1] ) {
            index--; // Walk down until finding a smaller weight value
        }

        // Drop the target element into its newly determined insertion slot
        weight[index] = copy_weight[i];
        target[index] = copy_target[i];
        
        // Right-shift remaining elements downstream to close the gap left by the insertion
        for ( int k = 0; k < i - index; k++) {
            weight[index+k+1] = copy_weight[index+k];
            target[index+k+1] = copy_target[index+k];
        }
    }
}