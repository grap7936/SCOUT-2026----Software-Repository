/////////////////////////////////////////////////////////////
/*
Code Summary:
The Selector is the data-association and state-estimation stage of the pipeline. Given
the previous frame's tracks (prev_targets) and the current frame's fresh detections
(next_targets), it decides which detection continues which track and advances each
track's Kalman filter. The work happens in scan(), which runs four phases:
  1.) estimateNextState() - Kalman PREDICT for every previous track (fills nx, ny).
  2.) computeWeights()    - build a proximity-cost graph from each previous track to
                            every current detection (delegated to the Graph class).
  3.) connect()           - solve the assignment with the Hungarian algorithm, link the
                            matched prev/next instances, Kalman CORRECT the matches, and
                            spawn brand-new tracks for unmatched detections.
  4.) (correction now happens inside connect() via updateEstimate()).

The Kalman filters are stored here in kf_list, an external std::vector indexed by Target
id (kf_list[id]). This array is kept parallel to the master full_list: initTarget() does
setID(full_list->size()) and then pushes onto BOTH lists, so kf_list[id] is always valid
for any initialized track.

The Selector also caches the current frame's "relevant" tracks (those with multi-frame
history) and their group median velocity, which the Sentry uses as the motion baseline
for debris scoring.
*/
/////////////////////////////////////////////////////////////

#include <Selector.hpp>

// Constructor with custom proximity threshold and occlusion timeout
Selector::Selector( int thresh, int timeout, int weight_comp) {
    this->THRESHOLD = thresh;
    this->FRAME_TIMEOUT = timeout;
    this->WEIGHT_COMPOSITION = weight_comp;
    this->relevant_target_list_offset = 0;

    this->prev_targets = nullptr;
    this->next_targets = nullptr;
    this->full_list = nullptr;
    this->kf_list = {};

    this->current_frame_num = 0;
    this->current_relevant_targets = {};
    this->current_mean_vx = 0.0;
    this->current_mean_vy = 0.0;
    this->current_median_vx = 0.0; 
    this->current_median_vy = 0.0;
}

// Default constructor 
Selector::Selector() {
    this->THRESHOLD = 1000;
    this->FRAME_TIMEOUT = 5;
    this->WEIGHT_COMPOSITION = 0.5;
    this->relevant_target_list_offset = 0;

    this->prev_targets = nullptr;
    this->next_targets = nullptr;
    this->full_list = nullptr;
    this->kf_list = {};

    this->current_frame_num = 0;
    this->current_relevant_targets = {};
    this->current_mean_vx = 0.0;
    this->current_mean_vy = 0.0;
    this->current_median_vx = 0.0; 
    this->current_median_vy = 0.0;
}

// Returns threhsolding value for connecting targets
int Selector::getThreshold() {
    return THRESHOLD;
}

// Sets the threshold value for connecting targets (pixel distance * 10)
void Selector::setThreshold(int thresh) {
    THRESHOLD = thresh;
}

// Returns the occlusion grace window (how many frames a track may go unmatched)
int Selector::getFrameTimeout() {
    return FRAME_TIMEOUT;
}

// Sets the occlusion grace window (how many frames a track may go unmatched)
void Selector::setFrameTimeout(int timeout) {
    FRAME_TIMEOUT = timeout;
}

// Returns the gain for weight of x,y in Graph.weight()
float Selector::getWeightComposition() {
    return WEIGHT_COMPOSITION;
}

// Sets the gain for weight of x,y in Graph.weight()
void Selector::setWeightComposition(float gain1) {
    WEIGHT_COMPOSITION = gain1;
}

// Returns the frame number currently being processed
int Selector::getCurrentFrameNum() {
    return current_frame_num;
}

// Sets the frame number currently being processed
void Selector::setCurrentFrameNum(int frame_num) {
    current_frame_num = frame_num;
}

// Returns current offset in the full target list for relevant targets scan
int Selector::getTargetListOffset() {
    return relevant_target_list_offset;
}

// Sets the current offset in the full target list for relevant targets scan
void Selector::setTargetListOffset(int offset) {
    relevant_target_list_offset = offset;
}

// Returns a pointer to the previous frame's targets vector
std::vector<Target*>* Selector::getPrevTargets() {
    return prev_targets;
}

// Sets the pointer tracking the previous frame's targets vector
void Selector::setPrevTargets( std::vector<Target*>* targets ) {
    prev_targets = targets;
}

// Returns a pointer to the next frame's targets vector
std::vector<Target*>* Selector::getNextTargets() {
    return next_targets;
}

// Sets the pointer tracking the next frame's targets vector
void Selector::setNextTargets( std::vector<Target*>* targets ) {
    next_targets = targets;
}

// Returns a pointer to the master target tracking history list
std::vector<Target*>* Selector::getFullTargetList() {
    return full_list;
}

// Sets the pointer tracking the master target tracking history list
void Selector::setFullTargetList( std::vector<Target*>* targets ) {
    full_list = targets;
}

// Returns the external Kalman filter array (one filter per track, indexed by Target id)
std::vector<cv::KalmanFilter> Selector::getKFList() {
    return kf_list;
}



// Returns a list of pointers to relevant targets for the current frame comparison
std::vector<Target*> Selector::getRelevantTargets() {
    return current_relevant_targets;
}

// Returns a 2-element float array representing the global average [mean_vx, mean_vy]
std::vector<float> Selector::getMeanTargetVelocity() {
    std::vector<float> pair = { current_mean_vx, current_mean_vy };
    return pair;
}

// Returns a 2-element float array representing the global median [median_vx, median_vy]
std::vector<float> Selector::getMedianTargetVelocity() {
    std::vector<float> pair = { current_median_vx, current_median_vy };
    return pair;
}

/* Function determineRelevantTargets()
 * description:
 *  Rebuilds the cached list of "relevant" tracks for the current frame. A track is
 *  relevant if it has a verified multi-frame history (its prev_instance is non-null,
 *  so it is not a brand-new detection) and it was seen recently enough to still be
 *  live (within frame_timeout of the current frame). The result is stored in the
 *  current_relevant_targets member for reuse by the velocity calculators and the Sentry.
 * inputs:
 *  none
 * returns:
 *  void - overwrites the current_relevant_targets member.
 */
void Selector::determineRelevantTargets() {

    // Drop last frame's cached list and rebuild it from scratch
    current_relevant_targets.clear();

    int size = full_list->size();

    int relevant_target_list_offset = 0;

    // backup check in case of a dump or reset
    if (size == 0) { relevant_target_list_offset = 0; }

    // resize timed_out tracker ahead of usage
    while ( size > (int)is_timed_out.size() ) {
        is_timed_out.push_back(false);
    }

    // Keep only tracks that have a historical link chain and are still within the live window
    for (int i = relevant_target_list_offset; i < size; i++) {
        Target* current_instance = (*full_list)[i];
        // check if linked
        if (current_instance->getPrevInstancePtr() != nullptr) {
            // check if linked target is too old
            if (current_instance->getFrameNum() >= current_frame_num - FRAME_TIMEOUT) {
                current_relevant_targets.push_back( current_instance );
            } 
        } 
        // update timed_out vector values
        if (current_instance->getFrameNum() < current_frame_num - FRAME_TIMEOUT) {
            is_timed_out[i] = true;
        }
    }

    // update offset for next relevant targets iteration
    updateRTOffset();

}

void Selector::updateRTOffset() {

    int i = relevant_target_list_offset;
    // safety check for i = 0 edge case. if current id is not timed out, don't update
    if ( i == 0 ) { return; }

    // iterate through timed_out list until i+1 (next id) is not timed out
    while ( i+1 < (int)is_timed_out.size() && is_timed_out[i+1] ) { i++; }

    relevant_target_list_offset = i;
}

/* Function calculateMeanVelocity()
 * description:
 *  Computes the mean velocity components (vx, vy) across all relevant tracks to
 *  establish a group motion baseline, caching the result in current_mean_vx/vy.
 *  NOTE: this mean path is currently unused by the pipeline - the Sentry's debris
 *  scoring uses the MEDIAN baseline (calculateMedianVelocity) because it is more
 *  robust to a few fast outliers. Kept available as an alternative baseline.
 * inputs:
 *  none
 * returns:
 *  void - updates current_mean_vx and current_mean_vy.
 */
void Selector::calculateMeanVelocity() {
    int size = current_relevant_targets.size();

    // Prevent division-by-zero if there are no relevant tracks this frame
    if (size == 0) {
        current_mean_vx = 0.0;
        current_mean_vy = 0.0;
        return;
    }

    float x_sum = 0.0;
    float y_sum = 0.0;

    // Accumulate total vector components across the active pool
    for (int i = 0; i < size; i++) {
        x_sum += current_relevant_targets[i]->getVx();
        y_sum += current_relevant_targets[i]->getVy();
    }

    // Average the accumulated components (size guaranteed non-zero by the guard above)
    current_mean_vx = x_sum / size;
    current_mean_vy = y_sum / size;

}

/* Function calculateMedianVelocity()
 * description:
 *  Computes the median velocity components (vx, vy) across all relevant tracks to
 *  establish a group motion baseline, caching the result in current_median_vx/vy.
 *  The median is more robust than the mean when a small number of fast-moving
 *  outliers (e.g. genuine debris) would otherwise pull the baseline away from what
 *  the majority of tracked points (e.g. background stars) are actually doing.
 *  vx and vy are sorted and ranked independently.
 * inputs:
 *  none (reads the cached current_relevant_targets member).
 * returns:
 *  void - updates current_median_vx and current_median_vy.
 */
void Selector::calculateMedianVelocity() {
    int size = current_relevant_targets.size();

    // Prevent division-by-zero errors if the target vector list is empty
    if (size == 0) { 
        current_median_vx = 0.0;
        current_median_vy = 0.0;
        return;
    }

    // Collect each component separately so they can be sorted and ranked independently
    std::vector<float> vx_values;
    std::vector<float> vy_values;
    vx_values.reserve(size);
    vy_values.reserve(size);

    for (int i = 0; i < size; i++) {
        vx_values.push_back( current_relevant_targets[i]->getVx() );
        vy_values.push_back( current_relevant_targets[i]->getVy() );
    }

    std::sort(vx_values.begin(), vx_values.end());
    std::sort(vy_values.begin(), vy_values.end());

    if ( size % 2 == 0 ) {
        // Even count: average the two middle elements
        current_median_vx = ( vx_values[size/2 - 1] + vx_values[size/2] ) / 2.0f;
        current_median_vy = ( vy_values[size/2 - 1] + vy_values[size/2] ) / 2.0f;
    } else {
        // Odd count: take the single middle element
        current_median_vx = vx_values[size/2];
        current_median_vy = vy_values[size/2];
    }

}

/* Function scan( vector<Target*>* prev, vector<Target*>* next, vector<Target*>* full )
 * description:
 * Scans all targets from prevTargets[] and nextTargets[] to determine which targets exist across both sets. 
 * Updates target ids for related targets and connects data pointers.
 * inputs:
 * vector<Target*>* prev - pointer to vector of Target pointers in 'previous' frame
 * vector<Target*>* next - pointer to vector of Target pointers in 'next' frame
 * vector<Target*>* full - pointer to the global master registry vector tracking all targets
 * returns:
 * void - updates all ids as well as nextInstance and prevInstance pointers for targets that exist in both sets.
 */
void Selector::scan( std::vector<Target*>* prev, std::vector<Target*>* next, std::vector<Target*>* full, int frame_num ) {

    // Cache structural parameters safely locally within class state pointers
    setNextTargets(next);
    setPrevTargets(prev);
    setFullTargetList(full);
    setCurrentFrameNum(frame_num);

    // Phase 1: Advance historical tracking equations forward to match current time frame
    estimateNextState();

    // Phase 2: Compute bipartite graph matching edges using Euclidean offsets
    computeWeights();

    // Phase 3: Solve the cost allocation problem and update node linking identities
    connect();

}

/* Function estimateNextState()
 * description:
 * Runs the linear prediction step of the Kalman filter for all active targets tracked in the previous frame to project their next locations.
 * inputs:
 * none
 * returns:
 * void - populates fields nx and ny for each target within prev_targets.
 */
void Selector::estimateNextState() {

    // Break early if no historical entities exist to project forwards
    if ( prev_targets->size() == 0 ) { return; }

    // Propagate all elements into their projected state windows
    #pragma omp parallel for
    for (size_t i = 0; i < prev_targets->size(); i++) {

        Target* target = (*prev_targets)[i];

        // Advance state tracking equations forward in time (Prediction stage)
        cv::Mat prediction = kf_list[target->getID()].predict();

        // Extract projected location predictions and store inside node properties
        target->setNx( prediction.at<double>(0,0) ); // Predicted X position coordinate
        target->setNy( prediction.at<double>(1,0) ); // Predicted Y position coordinate
    }  
}

/* Function computeWeights( float gain1 )
 * description:
 * Builds the proximity-cost graph for every previous track. For each target in
 * prev_targets it constructs a Graph rooted at that target and containing all
 * current detections as vertices, then calls calcWeight() to fill each edge with a
 * blended distance cost (gain1 weights the last-known-position distance, 1-gain1
 * weights the Kalman-predicted-position distance). The finished graph is attached
 * to the track via setProximity() and later read by connect().
 * inputs:
 * none - uses WEIGHT_COMPOSITION
 * returns:
 * void - sets the proximity graph pointer on each target in prev_targets.
 *
 * NOTE (memory): each call heap-allocates one Graph per previous track and overwrites
 * the previous frame's proximity pointer without deleting it. See the review report -
 * these graphs are never freed, so this leaks once per track per frame.
 */
void Selector::computeWeights() {

    // Check which vector is larger to determine the better parallelization method
    if ( prev_targets->size() > next_targets->size() ) {
        // parallelize loop around prev_targets as it is larger
        #pragma omp parallel for 
        // loop through
        for ( size_t i = 0; i < prev_targets->size(); i++) {

            // allocate graph on heap, constuct from current target lists, and calculate weight
            Graph* new_graph = new Graph(*(*prev_targets)[i], (*next_targets));
            new_graph->calcWeight( WEIGHT_COMPOSITION );
            
            // if there is a graph tied to this target for some reason, delete it
            // shouldn't happen but this ensures no memory leak.
            if ( (*prev_targets)[i]->getProximity() != nullptr ) {
                delete (*prev_targets)[i]->getProximity();
            }

            // link graph to target
            (*prev_targets)[i]->setProximity(new_graph);
        }
    } else {
        // parallelize loop around next_targets as it is larger
        for ( size_t i = 0; i < prev_targets->size(); i++) {

            // allocate graph on heap, constuct from current target lists, and calculate weight
            Graph* new_graph = new Graph(*(*prev_targets)[i], (*next_targets));
            // new OMP calculation function parallelizing around next_targets
            new_graph->calcWeightOMP( WEIGHT_COMPOSITION );
            
            // if there is a graph tied to this target for some reason, delete it
            // shouldn't happen but this ensures no memory leak.
            if ( (*prev_targets)[i]->getProximity() != nullptr ) {
                delete (*prev_targets)[i]->getProximity();
            }

            // link graph to target
            (*prev_targets)[i]->setProximity(new_graph);
        }
    }

    
}

/* Function hungarianAlgorithm( std::vector<std::vector<int>>& cost_matrix )
 * description:
 * Implementation of a hungarian algorithm to match prev_targets to next_targets
 * inputs:
 * std::vector<std::vector<int>> cost_matrix - 2D vector matrix of prevTarget->proximity values
 * returns:
 * std::vector<int> - a vector where result[i] contains the column index assigned to row i
 */
std::vector<int> Selector::hungarianAlgorithm( std::vector<std::vector<int>> cost_matrix ) {
    if (cost_matrix.empty()) return {};

    const int INF = 1e9; // Representation of infinity
    
    int n = cost_matrix.size();
    int m = cost_matrix[0].size(); // Works for rectangular matrices where n <= m

    // Rectangular processing: Pad out columns if rows outnumber columns to preserve sizing constraints
    if ( n > m ) {
        int pad = n - m;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < pad; j++) {
                cost_matrix[i].push_back( 0 );
            }
        }
        m = cost_matrix[0].size();
    }

    // Potentials for rows (u) and columns (v)
    std::vector<int> u(n + 1, 0), v(m + 1, 0);
    // Tracks assignments: p[j] = i means column j is assigned to row i
    std::vector<int> p(m + 1, 0);
    // Tracks the minimum delta for each column during BFS step
    std::vector<int> way(m + 1, 0);

    // Main optimization loop iterating across each individual row element matrix frame
    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int min_in_row = 0;
        std::vector<int> min_v(m + 1, INF);
        std::vector<bool> used(m + 1, false);
        
        // Execute shortest augmenting path generation
        do {
            used[min_in_row] = true;
            int current_row = p[min_in_row];
            int delta = INF;
            int next_column = 0;
            
            // Loop updates row vectors to find minimal cost delta columns
            for (int j = 1; j <= m; ++j) {
                if (!used[j]) {
                    // Calculate current reduced cost
                    int current_cost = cost_matrix[current_row - 1][j - 1] - u[current_row] - v[j];
                    if (current_cost < min_v[j]) {
                        min_v[j] = current_cost;
                        way[j] = min_in_row;
                    }
                    if (min_v[j] < delta) {
                        delta = min_v[j];
                        next_column = j;
                    }
                }
            }
            
            // Update dual variables/potentials along the tracked path step
            for (int j = 0; j <= m; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    min_v[j] -= delta;
                }
            }
            min_in_row = next_column;
        } while (p[min_in_row] != 0);
        
        // Remap alternating path back through tracking traces to establish link chains
        do {
            int previous_column = way[min_in_row];
            p[min_in_row] = p[previous_column];
            min_in_row = previous_column;
        } while (min_in_row != 0);
    }

    // Format output: match row indexes to their chosen columns
    std::vector<int> assignment(n);
    for (int j = 1; j <= m; ++j) {
        if (p[j] != 0) {
            assignment[p[j] - 1] = j - 1;
        }
    }
    return assignment;
}

/* Function connect()
 * description:
 * Connects all related targets from prevTargets[] and nextTargets[].
 * inputs:
 * none
 * returns:
 * void - sets Target.nextInstance pointers for targets in prevTargets[].
 * - sets Target.prevInstance pointers for targets in nextTargets[].
 * - updates Target.id values for targets in nextTargets[] whose proximity value exceeds 'THRESHOLD' (see Constructor).
 */
void Selector::connect() {
    int prev_size = prev_targets->size();
    int next_size = next_targets->size();

    // Allocation tracking bitset to map which prospective destinations have been locked
    std::vector<bool> next_targets_used(next_targets->size(), 0);

    // Simple connect, check if one or less objects within threshold.


    // Connect remaining objects using hungarian algorithm

    // Build the 2D linear assignment matrix (Rows = Previous targets, Columns = Next targets)
    std::vector<std::vector<int>> proximity_matrix;
    for ( int i = 0; i < prev_size; i++ ) {
        proximity_matrix.push_back( (*prev_targets)[i]->getProximity()->getWeights() );
    }

    // Execute Hungarian Optimizer to discover the globally minimized cost association path
    std::vector<int> col_from_row = hungarianAlgorithm(proximity_matrix);

    // Evaluate associations assigned to each historical trace entry
    for ( int j = 0; j < prev_size; j++ ) {
        int connect_index = col_from_row[j];
        
        // Safeguard matching outputs against structural rectangular padding indexes
        if ( connect_index > next_size - 1 ) {
            continue;
        }

        // Branching check: If error variance exceeds threshold, do not connect
        if ( (*prev_targets)[j]->getProximity()->getVertexWeight(connect_index) > THRESHOLD) {
            continue;
        } else {
            // Valid track association: Tie linked lists together across frames
            (*prev_targets)[j]->setNextInstancePtr( (*prev_targets)[j]->getProximity()->getVertexPtr(connect_index) );
            (*prev_targets)[j]->getNextInstancePtr()->setPrevInstancePtr( (*prev_targets)[j] );
            
            // Forward identity attributes down the matched track line
            (*prev_targets)[j]->getNextInstancePtr()->setID( (*prev_targets)[j]->getID() );
            
            // Mark destination node as handled
            next_targets_used[connect_index] = 1;
            
            // Overwrite master registry pointer to point to the newest active instance
            (*full_list)[(*prev_targets)[j]->getNextInstancePtr()->getID()] = (*prev_targets)[j]->getNextInstancePtr();
        }
    }

    // Update kalman filter velocity estimate prior to calculating median velocity
    updateEstimate();

    // Prior to cleanup pass, determine median velocity
    determineRelevantTargets();
    calculateMedianVelocity();

    // Cleanup pass: Unassigned elements in the new frame are spawned as fresh tracking sources
    for (size_t i = 0; i < next_targets->size(); i++) {
        if ( next_targets_used[i] == true ) {
            continue; // Node already successfully bound to a previous path
        } else {
            std::vector<float> estimated_velocity = getMedianTargetVelocity();
            initTarget((*next_targets)[i], estimated_velocity[0], estimated_velocity[1]); // Brand new detection, initialize path history
        }
    }
}

/* Function initTarget( Target* new_target, float est_vx, float est_vy )
 * description:
 * Promotes a brand-new detection into a tracked target: assigns it the next sequential
 * ID, appends it to full_list, and builds its own 4D constant-velocity Kalman filter
 * (state [x, y, vx, vy]) which is appended to kf_list. Because the ID is set to the
 * pre-push size of full_list and both lists are pushed once here, the invariant
 * kf_list[id] == this target's filter holds for the life of the track.
 * inputs:
 * Target* new_target - pointer to the uninitialized target.
 * float est_vx, est_vy - initial velocity estimate seeded into the filter's state
 *                        (the group median velocity, so new tracks start with the
 *                        swarm's motion rather than zero).
 * returns:
 * void - updates fields within new_target, appends to full_list and kf_list.
 */
void Selector::initTarget( Target* new_target, float est_vx, float est_vy ) {

    // Assign a unique ID based on the current number of tracked targets, then store it
    new_target->setID( getFullTargetList()->size() );
    new_target->setKx( new_target->getX() );
    new_target->setKy( new_target->getY() );
    getFullTargetList()->push_back(new_target);

    // Initialize Kalman Filter parameters
    // State vector: [x, y, vx, vy]^T -> 4 dimensions (Position & Velocity)
    // Measurement vector: [x, y]     -> 2 dimensions (Observed Camera Coordinates)
    int stateDim = 4;
    int measDim = 2;
    int ctrlDim = 0;
    
    cv::KalmanFilter KF(stateDim, measDim, ctrlDim, CV_64F);

    // Define the State Transition Matrix (A) assuming a constant velocity model:
    // x_next  = 1*x  + 0*y  + 1*vx + 0*vy
    // y_next  = 0*x  + 1*y  + 0*vx + 1*vy
    // vx_next = 0*x  + 0*y  + 1*vx + 0*vy
    // vy_next = 0*x  + 0*y  + 0*vx + 1*vy
    KF.transitionMatrix = (cv::Mat_<double>(4,4) << 1.0,0.0,1.0,0.0, 0.0,1.0,0.0,1.0, 0.0,0.0,1.0,0.0, 0.0,0.0,0.0,1.0);

    // Define Measurement Matrix (H) to isolate observed variables:
    // Extracts directly measured position [x, y] from the 4D state vector
    KF.measurementMatrix = (cv::Mat_<double>(2,4) << 1.0,0.0,0.0,0.0, 0.0,1.0,0.0,0.0);

    // Tune filter noise models

    // Process noise covariance (Q) - increasing this makes KF learn new speeds faster
    KF.processNoiseCov = cv::Mat::eye(stateDim, stateDim, CV_64F) * 1e-2; // was 1e-4
    
    // Measurement noise covariance (R) - noise pixel position
    KF.measurementNoiseCov = cv::Mat::eye(measDim, measDim, CV_64F) * 1e-1;
    
    // Posteriori error estimate covariance (P) - noise error in first estimate (initialization)
    KF.errorCovPost = cv::Mat::eye(stateDim, stateDim, CV_64F) * 3.0; // was 1.0

    // Seed the filter with the initial position coordinates; velocity starts at the swarm's current mean velocity
    KF.statePost = cv::Mat_<double>(4,1);
    KF.statePost.at<double>(0, 0) = static_cast<double>(new_target->getX());
    KF.statePost.at<double>(1, 0) = static_cast<double>(new_target->getY());
    KF.statePost.at<double>(2, 0) = static_cast<double>(est_vx);
    KF.statePost.at<double>(3, 0) = static_cast<double>(est_vy);

    // Store the filter in the external array; its index now matches this target's id
    kf_list.push_back( KF );
}

/* Function updateEstimate()
 * description:
 * Executes the measurement update step of the Kalman Filter using new camera data, refining tracking positions and resolving velocities for matched active instances.
 * inputs:
 * none
 * returns:
 * void - updates target fields (kx, ky) and speed calculations (vx, vy).
 */
void Selector::updateEstimate() {
    
    // Cycle active listings to reconcile real visual measurements against filter predictions
    #pragma omp parallel for
    for (size_t i = 0; i < next_targets->size(); i++) {
        Target* target = (*next_targets)[i];
        
        // Only run filter corrections if this node is verified to continue an existing path
        if ( target->getPrevInstancePtr() != nullptr ) {
            
            // Format incoming visual tracking hits into standard OpenCV structural containers
            cv::Mat measurement = cv::Mat::zeros(2, 1, CV_64F);
            measurement.at<double>(0, 0) = static_cast<double>(target->getX());
            measurement.at<double>(1, 0) = static_cast<double>(target->getY());

            // Execute Measurement Correction Step (Correct stage) to balance prediction vs observations
            cv::Mat estimated = kf_list[target->getID()].correct(measurement);
            
            // Unpack optimal state estimations onto target memory fields
            target->setKx( estimated.at<double>(0, 0) ); // Optimal calculated position X
            target->setKy( estimated.at<double>(1, 0) ); // Optimal calculated position Y
            target->setVx( estimated.at<double>(2, 0) ); // Smoothed velocity component X
            target->setVy( estimated.at<double>(3, 0) ); // Smoothed velocity component Y
        }
    }
}

