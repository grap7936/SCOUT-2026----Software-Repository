#include <Selector.hpp>

// Constructor
Selector::Selector( int thresh ) {
    threshold = thresh;
}

// default Constructor
Selector::Selector() {
    threshold = 1000;
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

void Selector::setFullTargetListPtr( std::vector<Target*>* new_ptr ) {
    full_list = new_ptr;
}

std::vector<Target*>* Selector::getFullTargetListPtr() {
    return full_list;
}

void Selector::setNextTargetsPtr( std::vector<Target*>* new_ptr ) {
    next_targets = new_ptr;
}

void Selector::initTarget( Target* new_target ) {

    new_target->id = full_list->size();
    full_list->push_back(new_target);
    // Initialize Kalman Filter
    // State: [x, y, vx, vy]^T -> 4 dimensions
    // Measurement: [x, y] -> 2 dimensions
    int stateDim = 4;
    int measDim = 2;
    int ctrlDim = 0;
    
    cv::KalmanFilter* KF = new cv::KalmanFilter(stateDim, measDim, ctrlDim, CV_32F);

    // 2. Define Matrices
    // Transition matrix (Constant Velocity: x = x0 + vx*df, y = y0 + vy*df)
    // float df = 1;
    
    // float A[] = {
    //     1, 0, df, 0,
    //     0, 1, 0, df,
    //     0, 0, 1, 0,
    //     0, 0, 0, 1
    // };
    // KF->transitionMatrix = cv::Mat(4, 4, CV_32F, A);
    KF->transitionMatrix = (cv::Mat_(4,4, CV_32F) << 1,0,1,0, 0,1,0,1, 0,0,1,0, 0,0,0,1);

    // Measurement matrix (We measure the position directly)
    // float H[] = {
    //     1, 0, 0, 0,
    //     0, 1, 0, 0
    // };
    // KF->measurementMatrix = cv::Mat(2, 4, CV_32F, H);
    KF->measurementMatrix = (cv::Mat_(2,4, CV_32F) << 1,0,0,0, 0,1,0,0);

    // 3. Set Noise Covariances
    // Process noise (system uncertainty)
    setIdentity(KF->processNoiseCov, cv::Scalar::all(1e-4));
    // Measurement noise (optical detection noise)
    setIdentity(KF->measurementNoiseCov, cv::Scalar::all(1e-1));
    // Posteriori error estimate covariance
    setIdentity(KF->errorCovPost, cv::Scalar::all(1));

    // Initial state setup
    // KF->statePost = cv::Mat_(4,1, CV_32F);
    // KF->statePost.at<float>(0) = new_target->x; // Initial X
    // KF->statePost.at<float>(1) = new_target->y; // Initial Y
    // KF->statePost.at<float>(2) = 0;   // Initial Vx
    // KF->statePost.at<float>(3) = 0;   // Initial Vy
    KF->statePost = (cv::Mat_(4,1, CV_32F) << new_target->x, new_target->y, 0.0, 0.0);

    new_target->kf = KF;
}

/* Function weight( Target* root )
 * description:
 *      Creates and populates a proximity graph for a provided root target.
 *      Calculates the distance to each target in nextTargets[]
 *      from the provided root target.
 * inputs:
 *      Target* root - pointer to a single target to compare to (from prevTargets[]).
 * returns:
 *      void - sets the root.proximity pointer to a created proximity graph
 */
void Selector::weight( Target* root ) {

    // populate graph
    Graph* graph = new Graph(root->id);
    for (int i = 0; i < next_targets->size(); i++) {
        // find normalized distance to x,y
        int dx = root->x - (*next_targets)[i]->x;
        int dy = root->y - (*next_targets)[i]->y;
        float norm1 = sqrt( pow(dx, 2) + pow(dy, 2) );
        int weight1 = norm1*10;

        // find normalized distance to nx,ny
        int dnx = root->nx - (*next_targets)[i]->x;
        int dny = root->ny - (*next_targets)[i]->y;
        float norm2 = sqrt( pow(dnx, 2) + pow(dny, 2) );
        int weight2 = norm2*10;

        graph->addVertex( (*next_targets)[i], weight1+weight2 );

    }

    root->proximity = graph;

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
    int prev_size = prev_targets->size();
    int next_size = next_targets->size();

    std::vector<bool> next_targets_used = {};
    for (int i = 0; i < next_targets->size(); i++) {
        next_targets_used.push_back ( 0 );
    }

    std::vector<std::vector<int>> proximity_matrix;

    for ( int i = 0; i < prev_size; i++ ) {
        proximity_matrix.push_back( (*prev_targets)[i]->proximity->weight );
    }

    std::vector<int> col_from_row = hungarianAlgorithm(proximity_matrix);

    for ( int j = 0; j < prev_size; j++ ) {
        int connect_index = col_from_row[j];
        if ( connect_index > next_size-1 ) {
            continue;
        }

        if ( (*prev_targets)[j]->proximity->getVertexWeight(connect_index) > threshold ) {
            initTarget((*prev_targets)[j]->proximity->getVertexPtr(connect_index));
            next_targets_used[connect_index] = 1;
            continue;
        } else {
            (*prev_targets)[j]->next_instance = (*prev_targets)[j]->proximity->getVertexPtr(connect_index);
            (*prev_targets)[j]->next_instance->prev_instance = (*prev_targets)[j];
            (*prev_targets)[j]->next_instance->id = (*prev_targets)[j]->id;
            (*prev_targets)[j]->next_instance->kf = (*prev_targets)[j]->kf;
            next_targets_used[connect_index] = 1;
            (*full_list)[(*prev_targets)[j]->next_instance->id] = (*prev_targets)[j]->next_instance;

        }
    }
    for (int i = 0; i < next_targets->size(); i++) {
        if ( next_targets_used[i] == true ) {
            continue;
        } else {
            initTarget((*next_targets)[i]);
        }
    }
    
}

/* Function hungarianAlgorithm( std::vector<std::vector<int>>& cost_matrix )
 * description:
 *      Implementation of a hungarian algorithm to match prev_targets to next_targets
 * inputs:
 *      std::vector<std::vector<int>> cost_matrix - 2D vector matrix of prevTarget->proximity values
 * returns:
 *      std::vector<int> - a vector where result[i] contains the column index assigned to row i
 */
std::vector<int> Selector::hungarianAlgorithm( std::vector<std::vector<int>> cost_matrix ) {
    if (cost_matrix.empty()) return {};

    const int INF = 1e9; // Representation of infinity
    
    int n = cost_matrix.size();
    int m = cost_matrix[0].size(); // Works for rectangular matrices where n <= m

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

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int min_in_row = 0;
        std::vector<int> min_v(m + 1, INF);
        std::vector<bool> used(m + 1, false);
        
        do {
            used[min_in_row] = true;
            int current_row = p[min_in_row];
            int delta = INF;
            int next_column = 0;
            
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
            
            // Update potentials
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
        
        // Remap alternating path
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

void Selector::estimateNextState() {

    if ( prev_targets->size() == 0 ) { return; }

    for (int i = 0; i < prev_targets->size(); i++) {

        Target* target = (*prev_targets)[i];

        cv::KalmanFilter* KF = target->kf;

        cv::Mat prediction = KF->predict();

        target->nx = prediction.at<float>(0,0);
        target->ny = prediction.at<float>(1,0);

    }  
        
}

void Selector::updateEstimate() {
    for (int i = 0; i < next_targets->size(); i++) {
        Target* target = (*next_targets)[i];
        if ( target->prev_instance != NULL ) {
            // Camera measurement
            cv::Mat measurement = cv::Mat::zeros(2, 1, CV_32F);
            
            measurement.at<float>(0) = target->x;
            measurement.at<float>(1) = target->y;

            // corrected estimate of original x,y in prev_target
            cv::Mat estimated = target->kf->correct(measurement);
            target->kx = estimated.at<float>(0);
            target->ky = estimated.at<float>(1);
            target->vx = estimated.at<float>(2);
            target->vy = estimated.at<float>(3);
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
void Selector::scan( std::vector<Target*>* prev, std::vector<Target*>* next, std::vector<Target*>* full ) {

    setNextTargetsPtr(next);
    setPrevTargetsPtr(prev);
    setFullTargetListPtr(full);

    int prev_size = prev->size();
    int next_size = next->size();
    if ( prev_size == 0 ) {
        for (int i = 0; i < next_size; i++) {
            initTarget((*next)[i]);
        }
    } else {

        estimateNextState();

        for ( int i = 0; i < prev_size; i++) {
            weight( (*prev)[i] );
        }

        connect();

        updateEstimate();
    }

      

}
