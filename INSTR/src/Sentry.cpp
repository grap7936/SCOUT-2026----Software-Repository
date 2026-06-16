#include "Sentry.hpp"

// Constructor initializing tracking pipelines and sub-module settings
Sentry::Sentry(int thresh) : selector(thresh) {
    full_target_list = {};
    target_debris_count = {};
    prev_targets = {};
    next_targets = {};
    detector = *(new Detector);
    selector = *(new Selector(thresh));
    current_frame_number = -1;
}

// Initial frame setup routine utilizing the detection engine
void Sentry::init( cv::Mat frame ) {
    setNextFrame( frame );
    current_frame_number = 0;
    detector.scan( frame, next_targets );
    selector.setFullTargetListPtr(&full_target_list);
    for (size_t i = 0; i < next_targets.size(); i++) {
        next_targets[i]->setFrameNum(current_frame_number);
        selector.initTarget(next_targets[i]);
        target_debris_count.push_back( 0 );
    }
}

/* Function pageFrame( cv::Mat frame )
 * description:
 *  Cycles the internal video processing buffers by shuffling the previous 'next' frame metrics into historical registers and executing fresh spatial scans on the incoming matrix.
 * inputs:
 *  cv::Mat frame - The latest raw image matrix containing current target positions.
 * returns:
 *  void - Mutates internal frame caches, target pools, and updates historical debris counters.
 */
void Sentry::pageFrame( cv::Mat frame ) {

    // Step 1: Promote the current tracking frame to be the new baseline history window
    setPrevFrame( getNextFrame() ); 
    setNextFrame( frame ); 

    // Step 2: Migrate current detections downstream to act as the tracking reference
    clearPrevTargets();
    for (size_t i = 0; i < next_targets.size(); i++) {
        prev_targets.push_back(next_targets[i]);
    }
    
    // Step 3: Clear the next array buffer and query the detector for incoming updates
    clearNextTargets();
    detector.scan( frame, next_targets );
    
    // Step 4: Run the matching pipeline to establish links between frame instances
    int old_full_list_size = full_target_list.size();
    selector.scan(&prev_targets, &next_targets, &full_target_list);
    int new_full_list_size = full_target_list.size();
    
    // Step 5: Expand the debris score vector to account for any brand new tracking nodes
    for (int j = old_full_list_size; j < new_full_list_size; j++ ) {
        target_debris_count.push_back( 0 );
    }

    // Set frame number
    for (size_t j = 0; j < next_targets.size(); j++) {
        next_targets[j]->setFrameNum(current_frame_number);
    }
}

// Sets the internal storage pointer for the newest image frame matrix
void Sentry::setNextFrame( cv::Mat frame ) {
    next_frame = frame;
}

// Returns a reference pointer tracking the master global target log
std::vector<Target*>* Sentry::getFullListPtr() {
    return &full_target_list;
}

// Returns a pointer to the previous frame's isolated target pool
std::vector<Target*>* Sentry::getPrevTargetPtr() {
    return &prev_targets;
}

// Returns a pointer to the upcoming frame's unassigned target pool
std::vector<Target*>* Sentry::getNextTargetPtr() {
    return &next_targets;
}

// Fetches the image matrix container tracking the current video frame
cv::Mat Sentry::getNextFrame() {
    return next_frame;
}

// Sets the storage matrix for the historical video reference frame
void Sentry::setPrevFrame( cv::Mat frame ) {
    prev_frame = frame;
}

// Fetches the image matrix container tracking the previous video frame
cv::Mat Sentry::getPrevFrame() {
    return prev_frame;
}

// Extract scalar position data for a specific element via index querying
std::vector<int> Sentry::getTargetCoords( int id ) {
    std::vector<int> position = {0, 0};
    position[0] = full_target_list[id]->x;
    position[1] = full_target_list[id]->y;
    return position;
}

// Returns total number of tracks currently managed in the registry
int Sentry::getNumTargets() {
    return full_target_list.size();
}

// Loops and de-allocates indices tracking old frame targets
void Sentry::clearPrevTargets() {
    while ( prev_targets.size() > 0 ) {
        prev_targets.pop_back();
    }
}

// Loops and de-allocates indices tracking incoming target buffers
void Sentry::clearNextTargets() {
    while ( next_targets.size() > 0 ) {
        next_targets.pop_back();
    }
}

/* Function findDebris( cv::Mat frame, int debris_id )
 * description:
 *  Primary execution routine. Pages the video pipeline, processes target kinematic histories, and maps outliers against existing target identities to isolate anomaly or debris traces.
 * inputs:
 *  cv::Mat frame - Raw video framework container context matrix.
 *  int debris_id - Current tracking integer identifying established debris; use -1 if tracking from scratch.
 * returns:
 *  int - The target identifier matching suspected anomaly parameters; returns -1 if tracking remains steady.
 */
int Sentry::findDebris( cv::Mat frame, int debris_id ) {
    
    // Safety check: Boot pipeline tracking immediately if no records exist yet
    if ( full_target_list.size() == 0 ) {
        init( frame );
        return -1;
    }

    // Increment frame counter
    current_frame_number++;

    // Progress data buffers forward by one step sequence
    pageFrame( frame );

    // Extract nodes that possess a verified historical context track
    std::vector<Target*> relevant_targets = getRelevantTargets();

    // Re-evaluate velocity trends to highlight target kinetic deviations
    updateDebrisLikelihood( relevant_targets );

    Target* saved_alt_target = nullptr;
    
    // Parse through tracked metrics to identity anomaly candidate conditions
    for ( size_t j = 0; j < relevant_targets.size(); j++) {
        Target* target = relevant_targets[j];

        // Core conditional gate: Flag entity if its outlier threshold score has been breached
        if ( target_debris_count[target->id] > 2 ) {
            
            // Branch A: System is actively seeking a target; return the first match found
            if ( debris_id == -1 ){
                return target->id;
            } 
            // Branch B: Confirmed persistence; the anomalous candidate aligns with our tracking target
            else if ( target->id == debris_id ) {
                return debris_id;
            } 
            // Branch C: Anomaly found but its ID conflicts with what we are currently monitoring
            else {
                // Buffer the first alternative candidate found as a fallback trace option
                if ( saved_alt_target == nullptr ) {
                    saved_alt_target = target;
                } else {
                    continue;
                }
            }
        }
    }
    
    // Fallback: If primary target drops but an alternative candidate was buffered, switch tracking focus
    if ( saved_alt_target != nullptr ) {
        return saved_alt_target->id;
    }

    return -1;
}

/* Function getRelevantTargets()
 * description:
 *  Loops through the global target registry to filter out newly spawned tracks, isolating only targets that possess a verified past tracking history.
 * inputs:
 *  none
 * returns:
 *  std::vector<Target*> - A list of target pointer references that have multi-frame continuity.
 */
std::vector<Target*> Sentry::getRelevantTargets() {
    int size = full_target_list.size();
    std::vector<Target*> relevant_targets = {};

    // Filter list down to objects with historical link chains
    for (int i = 0; i < size; i++) {
        Target* curr_instance = full_target_list[i];
        if (curr_instance->prev_instance != nullptr && curr_instance->getFrameNum() >= current_frame_number-6) {
            relevant_targets.push_back( curr_instance );
        }
    }

    return relevant_targets;
}

/* Function getMeanTargetVelocity( std::vector<Target*> relevant_targets )
 * description:
 *  Computes the mean velocity components (vx, vy) across all actively tracked persistent targets to establish a group motion baseline.
 * inputs:
 *  std::vector<Target*> relevant_targets - Filtered array list of ongoing historical track targets.
 * returns:
 *  std::vector<float> - A 2-element float array representing the global average [mean_vx, mean_vy].
 */
std::vector<float> Sentry::getMeanTargetVelocity( std::vector<Target*>& relevant_targets ) {
    int size = relevant_targets.size();
    float x_sum = 0.0;
    float y_sum = 0.0;

    // Accumulate total vector components across active pool
    for (int i = 0; i < size; i++) {
        x_sum += relevant_targets[i]->getVx();
        y_sum += relevant_targets[i]->getVy();
    }

    // Prevent division-by-zero errors if the target vector list is empty
    if (size == 0) return {0.0f, 0.0f};

    std::vector<float> mean = { x_sum / size, y_sum / size };
    return mean;
}

/* Function updateDebrisLikelihood( std::vector<Target*> relevant_targets )
 * description:
 *  Compares individual target vector components against the group's mean velocity.
 *  Targets moving counter to the group flow have their debris likelihood metrics incremented.
 * inputs:
 *  std::vector<Target*> relevant_targets - Filtered vector tracking multi-frame persistent entities.
 * returns:
 *  void - Directly updates object parameters and synchronizes local scoring vectors.
 */
void Sentry::updateDebrisLikelihood( std::vector<Target*>& relevant_targets ) {
    
    // Establish the reference group movement baseline metrics
    std::vector<float> mean_velocity = getMeanTargetVelocity( relevant_targets );

    // Break processing early if group flows are perfectly static to avoid division errors
    if ( mean_velocity[0] == 0.0 || mean_velocity[1] == 0.0 ) {
        return;
    }

    // Relative percentage bounds defining drift thresholds (0.5 = 50% deviation)
    float vx_thresh = 0.5;
    float vy_thresh = 0.5;

    // Check individual velocities against global average trend lines
    for (size_t i = 0; i < relevant_targets.size(); i++) {
        Target* target = relevant_targets[i];
        
        // Calculate normalized variation percentage along the X axis component
        float vx_diff = ( mean_velocity[0] - target->getVx() ) / mean_velocity[0];
        if (vx_diff < 0) { vx_diff = -vx_diff; } // Absolute value step
        
        // Calculate normalized variation percentage along the Y axis component
        float vy_diff = ( mean_velocity[1] - target->getVy() ) / mean_velocity[1];
        if (vy_diff < 0) { vy_diff = -vy_diff; } // Absolute value step

        // If kinematics deviate past threshold bounds, mark this target as an anomaly
        if ( vx_diff > vx_thresh || vy_diff > vy_thresh ) {
            target->incDebrisLikelihood();
            
            // Sync internal payload values back up with the local tracking metrics array
            target_debris_count[target->id] = target->getDebrisLikelihood();
        }
    }
}