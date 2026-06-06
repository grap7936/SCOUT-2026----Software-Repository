#include <Selector.hpp>

// Constructor
Selector::Selector( int w, int h, float thresh ) {
    frame_w = w;
    frame_h = h;
    max_norm = sqrt( pow(w, 2) + pow(h, 2) );
    threshold = thresh;
}

std::vector<Target*>* Selector::getPrevTargetsPtr() {
    return prev_targets;
}

void Selector::setPrevTargetsPtr( std::vector<Target*>* new_ptr ) {
    prev_targets = new_ptr;
}

std::vector<Target*>* Selector::getNextTargetsPtr() {
    return next_targets;
}

void Selector::setNextTargetsPtr( std::vector<Target*>* new_ptr ) {
    next_targets = new_ptr;
}

/* Function weight( Target* root )
 * description:
 *      Creates and populates a proximity graph for a provided root target.
 *      Calculates a weight for each target in nextTargets[]
 *      using distance to the provided root target.
 *      This weight is bound between 0.0 and 1.0.
 * inputs:
 *      Target* root - pointer to a single target to compare to (from prevTargets[]).
 * returns:
 *      void - sets the root.proximity pointer to a created proximity graph
 */
void Selector::weight( Target* root ) {

    float gain1 = 0.6;
    float gain2 = 1.0 - gain1;

    // populate graph
    Graph* graph = new Graph(root->id);
    for (int i = 0; i < next_targets->size(); i++) {
        // find normalized distance to x,y
        int dx = root->x - (*next_targets)[i]->x;
        int dy = root->y - (*next_targets)[i]->y;
        float norm1 = sqrt( pow(dx, 2) + pow(dy, 2) );
        float weight1 = gain1 * (max_norm - norm1) / max_norm;

        // find normalized distance to nx,ny
        int dnx = root->nx - (*next_targets)[i]->x;
        int dny = root->ny - (*next_targets)[i]->y;
        float norm2 = sqrt( pow(dnx, 2) + pow(dny, 2) );
        float weight2 = gain2 * (max_norm - norm2) / max_norm;

        // clamp weight to 1.0 max
        if ( weight1 + weight2 > 1 ){
            graph->addVertex( (*next_targets)[i], 1.0 );
        } else {
            graph->addVertex( (*next_targets)[i], weight1 + weight2 );
        }
    }

    graph->sortByWeight();
    root->proximity = graph;

}

/* Function handleConflict( Target* obj1, Target* obj2 )
 * description:
 *      Handles the case where multiple prevTargets weight a nextTarget above the 'threshold'.
 *      Assigns Target.nextInstance pointers for both obj1 and obj2.
 * inputs:
 *      Target* obj1 - target with currently assigned next_instance pointer
 *      
 *      Target* obj2 - target contesting assigned next_instance
 * 
 * returns:
 *      void - sets Target.next_instance pointers for targets obj1 and obj2.
 */
void Selector::handleConflict( Target* obj1, Target* obj2 ) {

    float backup_weight1 = obj1->proximity->getVertexWeight(1); 
    float backup_weight2 = obj2->proximity->getVertexWeight(1); 
    if ( obj2->proximity->getVertexWeight(0) > obj1->proximity->getVertexWeight(0) ) {
        if ( backup_weight1 > threshold ) {
            obj1->next_instance->prev_instance = obj2;
            obj1->next_instance = obj2->proximity->getVertexPtr(1);
            obj2->next_instance->id = obj2->id;

        } else if ( backup_weight2 > threshold ) {
            obj2->next_instance = obj2->proximity->getVertexPtr(1);
            obj2->next_instance->prev_instance = obj2;
            obj2->next_instance->id = obj2->id;

        } else {
            obj1->next_instance->prev_instance = obj2;
            obj2->next_instance = obj1->next_instance;
            obj2->next_instance->id = obj2->id;
            obj1->next_instance = NULL;
        } 
    } else if ( backup_weight2 > threshold ) {
        obj2->next_instance = obj2->proximity->getVertexPtr(1);
        obj2->next_instance->prev_instance = obj2;
        obj2->next_instance->id = obj2->id;

    } else if ( backup_weight1 > threshold ) {
        obj1->next_instance = obj1->proximity->getVertexPtr(1);
        obj2->next_instance->prev_instance = obj2;
        obj2->next_instance->id = obj2->id;
    
    } else {
        obj2->next_instance = NULL;

    }
    
    
}

/* Function connect()
 * description:
 *      Connects all related targets from prevTargets[] and nextTargets[].
 * inputs:
 *      none
 * returns:
 *      void - sets Target.nextInstance pointers for targets in prevTargets[].
 *           - sets Target.prevInstance pointers for targets in nextTargets[].
 *           - updates Target.id values for targets in nextTargets[] whose
 *             proximity value exceeds 'threshold' (see Constructor).
 */
void Selector::connect() {
    std::vector<int> used_ids = {};
    for (int i = 0; i < prev_targets->size(); i++) {

        Target* root = (*prev_targets)[i];
        int used_ids_size = used_ids.size();
        // check if above weight threshold
        if ( root->proximity->getVertexWeight(0) > threshold ) {
            // connect pointer
            root->next_instance = root->proximity->getVertexPtr(0);
            // check if instance is already a "next instance"
            bool duplicate_match_found = false;
            for ( int j = 0; j < used_ids_size; j++) {
                // check if id is used
                if ( used_ids[j] == root->next_instance->id ) {
                    duplicate_match_found = true;
                    break;
                }
            }
            
            if ( duplicate_match_found ) {
                // handle tracking conflict
                handleConflict( root->next_instance->prev_instance, root );
            } else {
                // update values
                root->next_instance->prev_instance = root;
                root->next_instance->id = root->id;
                used_ids.push_back( root->id );
            }
                       
        } else {
            root->next_instance = NULL;
        }
    }
}

/* Function scan( vector<Target*>* prev, vector<Target*>* next )
 * description:
 *      Scans all targets from prevTargets[] and nextTargets[] to determine 
 *      which targets exist across both sets.
 *      Updates target ids for related targets and connects LL pointers.
 * inputs:
 *      vector<Target*>* prev - pointer to vector of Target pointers to targets 
 *                              in 'previous' frame
 *      vector<Target*>* next - pointer to vector of Target pointers to targets 
 *                              in 'next' frame
 * returns:
 *      void - updates all ids as well as nextInstance and prevInstance 
 *             pointers for targets that exist in both sets.
 */
void Selector::scan( std::vector<Target*>* prev, std::vector<Target*>* next ) {

    setNextTargetsPtr(next);
    setPrevTargetsPtr(prev);

    int prev_size = prev->size();
    for ( int i = 0; i < prev_size; i++) {
        weight( (*prev)[i] );
    }

    connect();

}
