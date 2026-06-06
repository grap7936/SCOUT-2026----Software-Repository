#include <Selector.hpp>

// Constructor

Selector::Selector( int w, int h ) {
    frame_w = w;
    frame_h = h;
    max_norm = sqrt( pow(w, 2) + pow(h, 2) );
}

std::vector<Target>* Selector::getPrevTargetsPtr() {
    return prev_targets;
}

void Selector::setPrevTargetsPtr( std::vector<Target>* new_ptr ) {
    prev_targets = new_ptr;
}

std::vector<Target>* Selector::getNextTargetsPtr() {
    return next_targets;
}

void Selector::setNextTargetsPtr( std::vector<Target>* new_ptr ) {
    next_targets = new_ptr;
}

void Selector::weight( Target* root ) {

    float gain1 = 0.5;
    float gain2 = 1.0 - gain1;

    // populate graph
    Graph* graph = new Graph(root->id);
    for (int i = 0; i < next_targets->capacity(); i++) {
        // find normalized distance to x,y
        int dx = root->x - (*next_targets)[i].x;
        int dy = root->y - (*next_targets)[i].y;
        float norm1 = sqrt( pow(dx, 2) + pow(dy, 2) );
        float weight1 = gain1 * (max_norm - norm1) / max_norm;

        // find normalized distance to nx,ny
        int dnx = root->nx - (*next_targets)[i].x;
        int dny = root->ny - (*next_targets)[i].y;
        float norm2 = sqrt( pow(dnx, 2) + pow(dny, 2) );
        float weight2 = gain2 * (max_norm - norm2) / max_norm;

        // clamp weight to 1.0 max
        if ( weight1 + weight2 > 1 ){
            graph->addVertex( &(*next_targets)[i], 1.0 );
        } else {
            graph->addVertex( &(*next_targets)[i], weight1 + weight2 );
        }
    }

    graph->sortByWeight();
    root->proximity = graph;

}

void Selector::handleConflict( Target* obj1, Target* obj2 ) {
    // TO DO
}

void Selector::connect( float threshold ) {
    std::vector<int> used_ids = {};
    for (int i = 0; i < prev_targets->capacity(); i++) {

        Target* root = &(*prev_targets)[i];
        int used_ids_size = used_ids.capacity();
        // check if above weight threshold
        if ( root->proximity->getVertexWeight(0) > threshold ) {
            // connect pointer
            root->next_instance = root->proximity->getVertexPtr(0);
            // check if instance is already a "next instance"
            bool match_found = false;
            for ( int j = 0; j < used_ids_size; j++) {
                // check if id is used
                if ( used_ids[j] == root->next_instance->id ) {
                    match_found = true;
                    break;
                }
            }
            
            if ( match_found ) {
                // handle tracking conflict
                handleConflict( root, root->next_instance->prev_instance );
            } else {
                // update values
                root->next_instance->prev_instance = root;
                used_ids.push_back( root->next_instance->id );
            }
                       
        }
    }
}

void Selector::scan( std::vector<Target>* prev, std::vector<Target>* next ) {

    setNextTargetsPtr(next);
    setPrevTargetsPtr(prev);

    int next_size = next->capacity();
    for ( int i = 0; i < next_size; i++) {
        weight( &(*next)[i] );
    }

    connect( 0.8 );

}
