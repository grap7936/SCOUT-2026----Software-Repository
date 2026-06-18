#include "Sentry.hpp"

// Constructor initializing tracking pipelines and sub-module settings
Sentry::Sentry(int thresh) {
    full_target_list = {};
    target_debris_count = {};
    prev_targets = {};
    next_targets = {};
    detector = *(new Detector);
    selector = *(new Selector(thresh));
    current_frame_number = -1;

    // How many consecutive frames a track is allowed to go undetected before it's
    // considered permanently lost. Measured against real footage: dense, dim star
    // fields flicker in and out of detection a lot - over a quarter of brief dropouts
    // lasted longer than the old window of 4, while raising it to 10 catches roughly
    // 90% of them. For a near-static point this is safe even when the gap is long,
    // since its Kalman-predicted position barely drifts while vx/vy stay near zero,
    // so it's still the closest candidate whenever it reappears. Re-tune if your
    // scene's detection dropout characteristics differ.
    frame_timeout = 8;
}

// Initial frame setup routine utilizing the detection engine
void Sentry::init( cv::Mat frame ) {
    setNextFrame( frame );
    current_frame_number = 0;
    detector.scan( frame, next_targets, current_frame_number );
    selector.setFullTargetListPtr(&full_target_list);

    // No tracks exist yet on a cold start, so this naturally evaluates to {0,0} -
    // but compute and thread it explicitly rather than relying on initTarget's
    // default, to stay consistent with how pageFrame() seeds brand-new tracks.
    std::vector<Target*> prior_relevant_targets = getRelevantTargets();
    std::vector<float> prior_median_velocity = getMedianTargetVelocity( prior_relevant_targets );

    for (size_t i = 0; i < next_targets.size(); i++) {
        next_targets[i]->setFrameNum(current_frame_number);
        selector.initTarget(next_targets[i], prior_median_velocity[0], prior_median_velocity[1]);
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
void Sentry::pageFrame( cv::Mat frame, float mean_vx, float mean_vy ) {

    // Step 1: Promote the current tracking frame to be the new baseline history window
    setPrevFrame( getNextFrame() ); 
    setNextFrame( frame ); 

    // Step 2: Retain any tracks that went unmatched last round but are still within the
    // occlusion grace window (missed for frame_timeout frames or fewer), then add last
    // frame's detections as additional matching candidates for this round.
    std::vector<Target*> retained_targets = {};
    for (size_t i = 0; i < prev_targets.size(); i++) {
        Target* t = prev_targets[i];
        if ( t->next_instance == nullptr && (current_frame_number - t->getFrameNum()) <= frame_timeout ) {
            retained_targets.push_back(t);
        }
    }

    clearPrevTargets();
    for (size_t i = 0; i < retained_targets.size(); i++) {
        prev_targets.push_back(retained_targets[i]);
    }
    for (size_t i = 0; i < next_targets.size(); i++) {
        prev_targets.push_back(next_targets[i]);
    }
    
    // Step 3: Clear the next array buffer and query the detector for incoming updates
    clearNextTargets();
    detector.scan( frame, next_targets, current_frame_number );
    
    // Step 4: Run the matching pipeline to establish links between frame instances
    int old_full_list_size = full_target_list.size();
    selector.scan(&prev_targets, &next_targets, &full_target_list, mean_vx, mean_vy);
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

    // Snapshot the swarm's current mean velocity from tracks as they stood entering this
    // frame, so any brand-new tracks spawned while paging can be seeded with it rather
    // than starting from zero velocity.
    std::vector<Target*> prior_relevant_targets = getRelevantTargets();
    std::vector<float> prior_median_velocity = getMedianTargetVelocity( prior_relevant_targets );
    
    // Progress data buffers forward by one step sequence
    pageFrame( frame, prior_median_velocity[0], prior_median_velocity[1] );

    // Extract nodes that possess a verified historical context track
    std::vector<Target*> relevant_targets = getRelevantTargets();

    // Re-evaluate velocity trends to highlight target kinetic deviations
    updateDebrisLikelihood( relevant_targets );

    // Color frame
    // array of colors for tracking frame boxes  red, orange, yellow, green, blue, purple, pink
    std::vector<cv::Scalar> COLORS = { cv::Scalar(0,0,255),cv::Scalar(0,127,255),cv::Scalar(0,255,255),cv::Scalar(0,255,0),cv::Scalar(255,0,0),cv::Scalar(127,0,127),cv::Scalar(191,191,255) };
    for (size_t i = 0; i < next_targets.size(); i++) {
        if ( next_targets[i]->prev_instance == nullptr ) {
            cv::circle(frame, cv::Point(next_targets[i]->getX(), next_targets[i]->getY()), 10, COLORS[next_targets[i]->id % 7], -1);
        } else {
            cv::circle(frame, cv::Point(next_targets[i]->getX(), next_targets[i]->getY()), 15, COLORS[next_targets[i]->id % 7], -1);
        }
    }

    Target* saved_alt_target = nullptr;
    
    // Parse through tracked metrics to identity anomaly candidate conditions
    for ( size_t j = 0; j < relevant_targets.size(); j++) {
        Target* target = relevant_targets[j];

        // Core conditional gate: Flag entity if its outlier threshold score has been breached
        if ( target_debris_count[target->id] > 4 ) {
            
            // Branch A: Confirmed persistence; the anomalous candidate aligns with our tracking target
            if ( target->id == debris_id && debris_id != -1 ) {
                cv::circle(frame, cv::Point(target->getX(), target->getY()), 30, cv::Scalar(255, 255, 255), -1);
                return debris_id;
            } 
            // Branch B: Anomaly found but its ID conflicts with what we are currently monitoring
            else if ( debris_id == -1 || next_targets[0]->frame_num-frame_timeout > full_target_list[debris_id]->frame_num ) {
                // Buffer the first alternative candidate found as a fallback trace option
                if ( saved_alt_target == nullptr ) {
                    saved_alt_target = target;
                } else {
                    if (target->getDebrisLikelihood() > saved_alt_target->getDebrisLikelihood() ) {
                        saved_alt_target = target;
                    }
                }
            }
        }
    }
    
    // Fallback: If primary target drops but an alternative candidate was buffered, switch tracking focus
    if ( saved_alt_target != nullptr ) {
        cv::circle(frame, cv::Point(saved_alt_target->getX(), saved_alt_target->getY()), 30, cv::Scalar(255, 255, 255), -1);
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
        if (curr_instance->prev_instance != nullptr && curr_instance->getFrameNum() >= current_frame_number-frame_timeout) {
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

/* Function getMedianTargetVelocity( std::vector<Target*> relevant_targets )
 * description:
 *  Computes the median velocity components (vx, vy) across all actively tracked persistent
 *  targets to establish a group motion baseline. More robust than the mean when a small
 *  number of fast-moving outliers (e.g. genuine debris) would otherwise pull the baseline
 *  away from what the majority of tracked points (e.g. background stars) are actually doing.
 * inputs:
 *  std::vector<Target*> relevant_targets - Filtered array list of ongoing historical track targets.
 * returns:
 *  std::vector<float> - A 2-element float array representing the global median [median_vx, median_vy].
 */
std::vector<float> Sentry::getMedianTargetVelocity( std::vector<Target*>& relevant_targets ) {
    int size = relevant_targets.size();

    // Prevent out-of-bounds access if the target vector list is empty
    if (size == 0) return {0.0f, 0.0f};

    // Collect each component separately so they can be sorted and ranked independently
    std::vector<float> vx_values;
    std::vector<float> vy_values;
    vx_values.reserve(size);
    vy_values.reserve(size);

    for (int i = 0; i < size; i++) {
        vx_values.push_back( relevant_targets[i]->getVx() );
        vy_values.push_back( relevant_targets[i]->getVy() );
    }

    std::sort(vx_values.begin(), vx_values.end());
    std::sort(vy_values.begin(), vy_values.end());

    float median_vx, median_vy;
    if ( size % 2 == 0 ) {
        // Even count: average the two middle elements
        median_vx = ( vx_values[size/2 - 1] + vx_values[size/2] ) / 2.0f;
        median_vy = ( vy_values[size/2 - 1] + vy_values[size/2] ) / 2.0f;
    } else {
        // Odd count: take the single middle element
        median_vx = vx_values[size/2];
        median_vy = vy_values[size/2];
    }

    std::vector<float> median = { median_vx, median_vy };
    return median;
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
    std::vector<float> median_velocity = getMedianTargetVelocity( relevant_targets );

    // Break processing early if group flows are perfectly static to avoid division errors
    if ( median_velocity[0] == 0.0 || median_velocity[1] == 0.0 ) {
        return;
    }

    // Relative percentage bounds defining drift thresholds (0.5 = 50% deviation)
    float vx_thresh = 1.5;
    float vy_thresh = 1.5;

    // Check individual velocities against global average trend lines
    for (size_t i = 0; i < relevant_targets.size(); i++) {
        Target* target = relevant_targets[i];
        
        // Calculate normalized variation percentage along the X axis component
        float vx_diff = (median_velocity[0] - target->getVx()) / median_velocity[0];
        if (vx_diff < 0) { vx_diff = -vx_diff; } // Absolute value step
        
        // Calculate normalized variation percentage along the Y axis component
        float vy_diff = (median_velocity[1] - target->getVy()) / median_velocity[1];
        if (vy_diff < 0) { vy_diff = -vy_diff; } // Absolute value step

        // If kinematics deviate past threshold bounds, mark this target as an anomaly
        if ( vx_diff > vx_thresh || vy_diff > vy_thresh ) {
            
            // Sync internal payload values back up with the local tracking metrics array
            target_debris_count[target->id]++;
            target->setDebrisLikelihood(target_debris_count[target->id]);
        } else {
            if ( target_debris_count[target->id] > 0 ) {
                target_debris_count[target->id]--;
                target->setDebrisLikelihood(target_debris_count[target->id]);
            }
        }
    }
}
