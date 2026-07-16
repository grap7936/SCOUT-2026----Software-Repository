/////////////////////////////////////////////////////////////
/*
Code Summary:
A Graph is a lightweight, single-root cost structure used for data association. One
Graph is built per previous-frame track: its "root" is that track (stored as plain
copies of the root's id and its measured/predicted coordinates), and its vertices are
the current frame's candidate detections held in two parallel vectors - targets[]
(the candidate pointers) and weights[] (the matching cost to each candidate, same index).

calcWeight() fills weights[] with blended measured/predicted distances; the Selector then
stacks every track's weights[] into a cost matrix for the Hungarian algorithm. Helper
accessors expose vertices and weights by index or by target id, and sortByWeight() can
order the vertices cheapest-first.
*/
/////////////////////////////////////////////////////////////

#include <Graph.hpp>

// Constructor initializing the graph with a root target and its candidate vertex list
Graph::Graph( Target &root, const std::vector<Target*>& targets ) {

    this->root_id = root.getID();
    this->root_x = root.getX();
    this->root_y = root.getY();
    this->root_nx = root.getNx();
    this->root_ny = root.getNy();

    addVerticesFromList(targets);

}

// Constructor initializing the graph with a root target only (empty vertex list)
Graph::Graph( Target &root ) {

    this->root_id = root.getID();
    this->root_x = root.getX();
    this->root_y = root.getY();
    this->root_nx = root.getNx();
    this->root_ny = root.getNy();

    this->targets = {};
    this->weights = {};

}

// Backup constructor fallback initializing an unlinked root (-1)
Graph::Graph() {
    this->root_id = -1;
    this->root_x = -1;
    this->root_y = -1;
    this->root_nx = -1;
    this->root_ny = -1;

    this->targets = {};
    this->weights = {};
}


// rebuild graph without deleting & reallocating
void Graph::rebuild( Target &root, const std::vector<Target*>& next ) {
    root_id = root.getID();
    root_x  = root.getX();
    root_y  = root.getY();
    root_nx = root.getNx();
    root_ny = root.getNy();

    targets.clear();          // retains capacity — no free, no realloc
    weights.clear();
    addVerticesFromList( next );
}


// Returns the target id of the root target
int Graph::getRootID() {
    return root_id;
}

// Returns the x coordinate of the root target
int Graph::getRootX() {
    return root_x;
}

// Returns the y coordinate of the root target
int Graph::getRootY() {
    return root_y;
}

// Returns the next x coordinate of the root target
int Graph::getRootNX() {
    return root_nx;
}

// Returns the next y coordinate of the root target
int Graph::getRootNY() {
    return root_ny;
}

// Returns the target pointer at a specific index location
Target* Graph::getVertexPtr( int index ){
    return targets[index];
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
    int size = targets.size();
    
    // Linear scan to locate the node containing the target ID match
    for ( int i = 0; i < size; i++ ) {
        if ( targets[i]->getID() == vertex_id ) {
            return targets[i]; // Return pointer to found node instance
        }
    }
    return nullptr; // Fallback indicator that no match exists in this graph
}

// Returns the target ID at a specific index with an out-of-bounds safety check
int Graph::getVertexID( size_t index ){
    if (index >= targets.size()) {
        return -2; // Error boundary sentinel flag
    }
    return targets[index]->getID();
}

// Returns the assignment weight at a specific index with an out-of-bounds safety check
int Graph::getVertexWeight( size_t index ){
    if (index >= weights.size()) {
        return -2; // Error boundary sentinel flag
    }
    return weights[index];
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
    int size = targets.size();
    
    // Scan matching structures to determine proximity cost association values
    for ( int i = 0; i < size; i++ ) {
        if ( targets[i]->getID() == vertex_id ) {
            return weights[i]; // Return the matching scalar value index pair
        }
    }
    return -1; // Out-of-bounds fallback signaling no target match detected
}

// Returns target list
std::vector<Target*> Graph::getTargets() {
    return targets;
}

// Returns weight list
std::vector<int> Graph::getWeights() {
    return weights;
}

/* Function addVertex( Target* vertex )
 * description:
 *  Appends a new target into tracking vector.
 * inputs:
 *  Target* vertex - Pointer to the target being evaluated.
 * returns:
 *  void - Modifies the targets vector in-place.
 */
void Graph::addVertex( Target* vertex ) {
    targets.push_back( vertex );
    weights.push_back ( 0 );
}

/* Function addVertex( Target* vertex, int weight )
 * description:
 *  Appends a new target into tracking vector.
 * inputs:
 *  Target* vertex - Pointer to the target being evaluated.
 *  int weight - custom weight value, usually from graphTest.cpp
 * returns:
 *  void - Modifies the targets vector in-place.
 */
void Graph::addVertex( Target* vertex, int weight ) {
    targets.push_back( vertex );
    weights.push_back ( weight );
}

/* Function addVerticesFromList( std::vector<Target*> vertices )
 * description:
 *  Appends several new targets into tracking vector.
 * inputs:
 *  std::vector<Target*> vertices - List of pointers to targets being evaluated.
 * returns:
 *  void - Modifies the targets vector in-place.
 */
void Graph::addVerticesFromList( const std::vector<Target*>& vertices ) {
    const size_t size = vertices.size();
    targets.reserve(targets.size() + size);
    weights.reserve(weights.size() + size);
    for (size_t i = 0; i < size; i++) {
        addVertex( vertices[i] );
    }
    
}

/* Function addVerticesFromList( std::vector<Target*> vertices, std::vector<int> weight )
 * description:
 *  Appends several new targets into tracking vector.
 * inputs:
 *  std::vector<Target*> vertices - List of pointers to targets being evaluated.
 *  std::vector<int> weight - List of weights corresponding to vertices list
 * returns:
 *  void - Modifies the targets vector in-place.
 */
void Graph::addVerticesFromList( const std::vector<Target*>& vertices, const std::vector<int>& weight ) {
    
    const size_t size = vertices.size();
    targets.reserve(targets.size() + size);
    weights.reserve(weights.size() + size);
    for (size_t i = 0; i < size; i++) {
        addVertex( vertices[i], weight[i] );
    }
   
}

/* Function calcWeight( float gain1 )
 * description:
 * Fills this graph's weights[] with the assignment cost from the root target to every
 * vertex (candidate detection). Each cost blends two Euclidean distances: the distance
 * from the root's last measured position (root_x, root_y) to the candidate, and the
 * distance from the root's Kalman-predicted position (root_nx, root_ny) to the
 * candidate. gain1 weights the measured-position term and gain2 = 1 - gain1 weights
 * the predicted-position term. Distances are scaled by 10 so they map cleanly onto the
 * integer cost matrix the Hungarian algorithm consumes.
 * inputs:
 * float gain1 - blend factor (0..1); measured-position term weight (predicted term gets 1-gain1).
 * returns:
 * void - overwrites this graph's weights[] in place (parallel to targets[]).
 */
void Graph::calcWeight( float gain1 ) {

    float gain2 = 1.0 - gain1;

    // Evaluate physical spacing between the root and every candidate detection in this graph
    for (size_t i = 0; i < targets.size(); i++) {

        // Step 1: Distance between the root's last measured position and this candidate
        int dx = root_x - targets[i]->getX();
        int dy = root_y - targets[i]->getY();
        float norm1 = std::sqrt( dx*dx + dy*dy );
        int weight1 = gain1 * norm1 * 10; // Scaling factor for cost matrix compliance

        // Step 2: Distance between the root's Kalman-predicted position (nx, ny) and this candidate
        int dnx = root_nx - targets[i]->getX();
        int dny = root_ny - targets[i]->getY();
        float norm2 = std::sqrt( dnx*dnx + dny*dny );
        int weight2 = gain2 * norm2 * 10;

        // Step 3: Store the blended cost at this vertex's index
        weights[i] = weight1 + weight2;

    }

}

/* Function calcWeightOMP( float gain1 )
 * description:
 * Same as above but parallelized with OpenMP
 * inputs:
 * float gain1 - blend factor (0..1); measured-position term weight (predicted term gets 1-gain1).
 * returns:
 * void - overwrites this graph's weights[] in place (parallel to targets[]).
 */
void Graph::calcWeightOMP( float gain1 ) {

    float gain2 = 1.0 - gain1;

    // Evaluate physical spacing between the root and every candidate detection in this graph
    #pragma omp parallel for 
    for (size_t i = 0; i < targets.size(); i++) {

        // Step 1: Distance between the root's last measured position and this candidate
        int dx = root_x - targets[i]->getX();
        int dy = root_y - targets[i]->getY();
        float norm1 = std::sqrt( dx*dx + dy*dy );
        int weight1 = gain1 * norm1 * 10; // Scaling factor for cost matrix compliance

        // Step 2: Distance between the root's Kalman-predicted position (nx, ny) and this candidate
        int dnx = root_nx - targets[i]->getX();
        int dny = root_ny - targets[i]->getY();
        float norm2 = std::sqrt( dnx*dnx + dny*dny );
        int weight2 = gain2 * norm2 * 10;

        // Step 3: Store the blended cost at this vertex's index
        weights[i] = weight1 + weight2;

    }

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
    int size = weights.size();
    
    // Duplicate snapshot arrays to hold state contexts while doing in-place shifting
    std::vector<int> copy_weights;
    std::vector<Target*> copy_targets;

    // Insertion sort loop beginning at index 1 and parsing forward
    for ( int i = 1; i < size; i++ ) {
        
        // Refresh the backup mirrors with the current state of vectors before moving positions
        copy_weights = weights;
        copy_targets = targets;
        
        // Scan backwards through the sorted sub-list to find where element 'i' fits
        int index = i;
        while ( index > 0 && weights[i] < weights[index-1] ) {
            index--; // Walk down until finding a smaller weight value
        }

        // Drop the target element into its newly determined insertion slot
        weights[index] = copy_weights[i];
        targets[index] = copy_targets[i];
        
        // Right-shift remaining elements downstream to close the gap left by the insertion
        for ( int k = 0; k < i - index; k++) {
            weights[index+k+1] = copy_weights[index+k];
            targets[index+k+1] = copy_targets[index+k];
        }
    }
}