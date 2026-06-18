#include <Selector.hpp>

// Constructor with custom proximity threshold
Selector::Selector( int thresh ) {
    threshold = thresh;
}

// Default constructor (threshold = 1000)
Selector::Selector() {
    threshold = 1000;
}

// Returns a pointer to the previous frame's targets vector
std::vector<Target*>* Selector::getPrevTargetsPtr() {
    return prev_targets;
}

// Sets the pointer tracking the previous frame's targets vector
void Selector::setPrevTargetsPtr( std::vector<Target*>* new_ptr ) {
    prev_targets = new_ptr;
}

// Returns a pointer to the next frame's targets vector
std::vector<Target*>* Selector::getNextTargetsPtr() {
    return next_targets;
}

// Sets the pointer tracking the master target tracking history list
void Selector::setFullTargetListPtr( std::vector<Target*>* new_ptr ) {
    full_list = new_ptr;
}

// Returns a pointer to the master target tracking history list
std::vector<Target*>* Selector::getFullTargetListPtr() {
    return full_list;
}

// Sets the pointer tracking the next frame's targets vector
void Selector::setNextTargetsPtr( std::vector<Target*>* new_ptr ) {
    next_targets = new_ptr;
}

/* Function initTarget( Target* new_target )
 * description:
 * Assigns a unique ID to a new target, adds it to the full list, and initializes its 4D Kalman Filter tracking engine.
 * inputs:
 * Target* new_target - pointer to the uninitialized target.
 * returns:
 * void - updates fields within new_target and appends it to full_list.
 */
void Selector::initTarget( Target* new_target, float mean_vx, float mean_vy ) {

    // Assign a unique ID based on the current number of tracked targets, then store it
    new_target->id = full_list->size();
    new_target->kx = new_target->x;
    new_target->ky = new_target->y;
    full_list->push_back(new_target);

    // Initialize Kalman Filter parameters
    // State vector: [x, y, vx, vy]^T -> 4 dimensions (Position & Velocity)
    // Measurement vector: [x, y]     -> 2 dimensions (Observed Camera Coordinates)
    int stateDim = 4;
    int measDim = 2;
    int ctrlDim = 0;
    
    auto KF = std::make_shared<cv::KalmanFilter>(stateDim, measDim, ctrlDim, CV_64F);

    // Define the State Transition Matrix (A) assuming a constant velocity model:
    // x_next  = 1*x  + 0*y  + 1*vx + 0*vy
    // y_next  = 0*x  + 1*y  + 0*vx + 1*vy
    // vx_next = 0*x  + 0*y  + 1*vx + 0*vy
    // vy_next = 0*x  + 0*y  + 0*vx + 1*vy
    KF->transitionMatrix = (cv::Mat_<double>(4,4) << 1.0,0.0,1.0,0.0, 0.0,1.0,0.0,1.0, 0.0,0.0,1.0,0.0, 0.0,0.0,0.0,1.0);

    // Define Measurement Matrix (H) to isolate observed variables:
    // Extracts directly measured position [x, y] from the 4D state vector
    KF->measurementMatrix = (cv::Mat_<double>(2,4) << 1.0,0.0,0.0,0.0, 0.0,1.0,0.0,0.0);

    // Tune filter noise models
    // Process noise covariance (Q)
    KF->processNoiseCov = cv::Mat::eye(stateDim, stateDim, CV_64F) * 1e-4;
    
    // Measurement noise covariance (R)
    KF->measurementNoiseCov = cv::Mat::eye(measDim, measDim, CV_64F) * 1e-1;
    
    // Posteriori error estimate covariance (P)
    KF->errorCovPost = cv::Mat::eye(stateDim, stateDim, CV_64F) * 1.0;

    // Seed the filter with the initial position coordinates; velocity starts at the swarm's current mean velocity
    KF->statePost = cv::Mat_<double>(4,1);
    KF->statePost.at<double>(0, 0) = static_cast<double>(new_target->x);
    KF->statePost.at<double>(1, 0) = static_cast<double>(new_target->y);
    KF->statePost.at<double>(2, 0) = static_cast<double>(mean_vx);
    KF->statePost.at<double>(3, 0) = static_cast<double>(mean_vy);

    // Link the filter instance directly onto the target object payload
    new_target->kf = KF;
}

/* Function weight( Target* root )
 * description:
 * Creates and populates a proximity graph for a provided root target.
 * Calculates the distance to each target in nextTargets[] from the provided root target.
 * inputs:
 * Target* root - pointer to a single target to compare to (from prevTargets[]).
 * returns:
 * void - sets the root.proximity pointer to a created proximity graph
 */
void Selector::weight( Target* root ) {

    // Instantiate a new tracking weight graph anchored to the root target's ID
    Graph* graph = new Graph(root->id);

    // Evaluate physical spacing between the root and every target found in the incoming frame
    for (size_t i = 0; i < next_targets->size(); i++) {

        float gain1 = 0.35;
        float gain2 = 1.0 - gain1;
        
        // Step 1: Compute distance between previous known position and next raw position
        int dx = root->x - (*next_targets)[i]->x;
        int dy = root->y - (*next_targets)[i]->y;
        float norm1 = sqrt( pow(dx, 2) + pow(dy, 2) );
        int weight1 = gain1 * norm1 * 10; // Scaling factor for cost matrix compliance

        // Step 2: Compute distance between Kalman predicted position (nx, ny) and next raw position
        int dnx = root->nx - (*next_targets)[i]->x;
        int dny = root->ny - (*next_targets)[i]->y;
        float norm2 = sqrt( pow(dnx, 2) + pow(dny, 2) );
        int weight2 = gain2 * norm2 * 10;

        // Composite cost combines actual past offset and projected tracking path proximity
        graph->addVertex( (*next_targets)[i], weight1 + weight2 );
    }

    // Assign the completed graph weights onto the root node for match parsing
    root->proximity = graph;
}

/* Function connect()
 * description:
 * Connects all related targets from prevTargets[] and nextTargets[].
 * inputs:
 * none
 * returns:
 * void - sets Target.nextInstance pointers for targets in prevTargets[].
 * - sets Target.prevInstance pointers for targets in nextTargets[].
 * - updates Target.id values for targets in nextTargets[] whose proximity value exceeds 'threshold' (see Constructor).
 */
void Selector::connect( float mean_vx, float mean_vy ) {
    int prev_size = prev_targets->size();
    int next_size = next_targets->size();

    // Allocation tracking bitset to map which prospective destinations have been locked
    std::vector<bool> next_targets_used = {};
    for (size_t i = 0; i < next_targets->size(); i++) {
        next_targets_used.push_back ( 0 );
    }

    // Build the 2D linear assignment matrix (Rows = Previous targets, Columns = Next targets)
    std::vector<std::vector<int>> proximity_matrix;
    for ( int i = 0; i < prev_size; i++ ) {
        proximity_matrix.push_back( (*prev_targets)[i]->proximity->weight );
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
        if ( (*prev_targets)[j]->proximity->getVertexWeight(connect_index) > threshold) {
            continue;
        } else {
            // Valid track association: Tie linked lists together across frames
            (*prev_targets)[j]->next_instance = (*prev_targets)[j]->proximity->getVertexPtr(connect_index);
            (*prev_targets)[j]->next_instance->prev_instance = (*prev_targets)[j];
            
            // Forward identity attributes down the matched track line
            (*prev_targets)[j]->next_instance->id = (*prev_targets)[j]->id;
            (*prev_targets)[j]->next_instance->kf = (*prev_targets)[j]->kf;
            
            // Mark destination node as handled
            next_targets_used[connect_index] = 1;
            
            // Overwrite master registry pointer to point to the newest active instance
            (*full_list)[(*prev_targets)[j]->next_instance->id] = (*prev_targets)[j]->next_instance;
        }
    }

    // Cleanup pass: Unassigned elements in the new frame are spawned as fresh tracking sources
    for (size_t i = 0; i < next_targets->size(); i++) {
        if ( next_targets_used[i] == true ) {
            continue; // Node already successfully bound to a previous path
        } else {
            initTarget((*next_targets)[i], mean_vx, mean_vy); // Brand new detection, initialize path history
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
        for (int i = 0; i < n-m; i++) {
            for (int j = 0; j < n; j++) {
                cost_matrix[j].push_back( INF );
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
    for (size_t i = 0; i < prev_targets->size(); i++) {

        Target* target = (*prev_targets)[i];
        std::shared_ptr<cv::KalmanFilter> KF = target->kf;

        // Advance state tracking equations forward in time (Prediction stage)
        cv::Mat prediction = KF->predict();

        // Extract projected location predictions and store inside node properties
        target->nx = prediction.at<double>(0,0); // Predicted X position coordinate
        target->ny = prediction.at<double>(1,0); // Predicted Y position coordinate
    }  
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
    for (size_t i = 0; i < next_targets->size(); i++) {
        Target* target = (*next_targets)[i];
        
        // Only run filter corrections if this node is verified to continue an existing path
        if ( target->prev_instance != nullptr ) {
            
            // Format incoming visual tracking hits into standard OpenCV structural containers
            cv::Mat measurement = cv::Mat::zeros(2, 1, CV_64F);
            measurement.at<double>(0, 0) = static_cast<double>(target->x);
            measurement.at<double>(1, 0) = static_cast<double>(target->y);

            // Execute Measurement Correction Step (Correct stage) to balance prediction vs observations
            cv::Mat estimated = target->kf->correct(measurement);
            
            // Unpack optimal state estimations onto target memory fields
            target->kx = estimated.at<double>(0, 0); // Optimal calculated position X
            target->ky = estimated.at<double>(1, 0); // Optimal calculated position Y
            target->vx = estimated.at<double>(2, 0); // Smoothed velocity component X
            target->vy = estimated.at<double>(3, 0); // Smoothed velocity component Y
        }
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
void Selector::scan( std::vector<Target*>* prev, std::vector<Target*>* next, std::vector<Target*>* full, float mean_vx, float mean_vy ) {

    // Cache structural parameters safely locally within class state pointers
    setNextTargetsPtr(next);
    setPrevTargetsPtr(prev);
    setFullTargetListPtr(full);

    int prev_size = prev->size();

    // Phase 1: Advance historical tracking equations forward to match current time frame
    estimateNextState();

    // Phase 2: Compute bipartite graph matching edges using Euclidean offsets
    for ( int i = 0; i < prev_size; i++) {
        weight( (*prev)[i] );
    }

    // Phase 3: Solve the cost allocation problem and update node linking identities
    connect( mean_vx, mean_vy );

    // Phase 4: Correct Kalman track matrices using real optical observations
    updateEstimate();
}
