#include "Sentry.hpp"

// Constructor initializing tracking pipelines and sub-module settings
Sentry::Sentry(int thresh) {
    full_target_list = {};
    target_debris_count = {};
    prev_targets = {};
    next_targets = {};
    frame_timeout = 4;
    detector = *(new Detector);
    selector = *(new Selector(thresh, frame_timeout));
    current_frame_number = -1;
    DEBRIS_THRESHOLD = 12;
    REFRESH_FREQUENCY = 15;

    // How many consecutive frames a track is allowed to go undetected before it's
    // considered permanently lost. Measured against real footage: dense, dim star
    // fields flicker in and out of detection a lot - over a quarter of brief dropouts
    // lasted longer than the old window of 4, while raising it to 10 catches roughly
    // 90% of them. For a near-static point this is safe even when the gap is long,
    // since its Kalman-predicted position barely drifts while vx/vy stay near zero,
    // so it's still the closest candidate whenever it reappears. Re-tune if your
    // scene's detection dropout characteristics differ.
    // frame_timeout = 8;
}

// Initial frame setup routine utilizing the detection engine
void Sentry::init( cv::Mat frame ) {
    setNextFrame( frame );
    current_frame_number = 0;
    detector.scan( frame, next_targets, current_frame_number );
    selector.setFullTargetList(&full_target_list);

    // No tracks exist yet on a cold start, so this naturally evaluates to {0,0} -
    // but compute and thread it explicitly rather than relying on initTarget's
    // default, to stay consistent with how pageFrame() seeds brand-new tracks.
    // std::vector<Target*> prior_relevant_targets = getRelevantTargets(); ***!!! MOVED TO Sentry.cpp !!!***
    // std::vector<float> prior_median_velocity = getMedianTargetVelocity( prior_relevant_targets );

    for (size_t i = 0; i < next_targets.size(); i++) {
        selector.initTarget(next_targets[i], 0.0, 0.0);
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

    // Step 2: Retain any tracks that went unmatched last round but are still within the
    // occlusion grace window (missed for frame_timeout frames or fewer), then add last
    // frame's detections as additional matching candidates for this round.
    std::vector<Target*> retained_targets = {};
    for (size_t i = 0; i < prev_targets.size(); i++) {
        Target* t = prev_targets[i];
        if ( t->getNextInstancePtr() == nullptr && (current_frame_number - t->getFrameNum()) <= frame_timeout ) {
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
    selector.scan(&prev_targets, &next_targets, &full_target_list, current_frame_number);
    int new_full_list_size = full_target_list.size();
    
    // Step 5: Expand the debris score vector to account for any brand new tracking nodes
    for (int j = old_full_list_size; j < new_full_list_size; j++ ) {
        target_debris_count.push_back( 0 );
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


Detector* Sentry::getDetectorPtr() {
    return &detector;
}

Selector* Sentry::getSelectorPtr() {
    return &selector;
}

// Extract scalar position data for a specific element via index querying
std::vector<int> Sentry::getTargetCoords( int id ) {
    Target* t = full_target_list[id];
    // Return four coordinates so the caller can choose the right one for the
    // track's state:
    //   [0],[1] = kx, ky : last Kalman-CORRECTED position. Smoothed and accurate
    //             while the track is being matched each frame; freezes at the last
    //             correction if the track goes temporarily unmatched.
    //   [2],[3] = nx, ny : Kalman PREDICTION. Keeps advancing along the track's
    //             trajectory every frame even while unmatched (each predict() rolls
    //             the state forward), so this is the non-frozen estimate to use
    //             during an occlusion gap.
    std::vector<int> position = {0, 0, 0, 0};
    position[0] = static_cast<int>( t->getKx() );
    position[1] = static_cast<int>( t->getKy() );
    position[2] = t->getNx();
    position[3] = t->getNy();
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

    if ( current_frame_number % REFRESH_FREQUENCY == 0 ) {
        detector.startCalibration();
    }

    // Snapshot the swarm's current mean velocity from tracks as they stood entering this
    // frame, so any brand-new tracks spawned while paging can be seeded with it rather
    // than starting from zero velocity. ***!!!! MOVED TO Selector.cpp !!!!***
    // std::vector<Target*> prior_relevant_targets = getRelevantTargets();
    // std::vector<float> prior_median_velocity = getMedianTargetVelocity( prior_relevant_targets );
    
    // Progress data buffers forward by one step sequence
    pageFrame( frame );

    // Extract nodes that possess a verified historical context track
    std::vector<Target*> relevant_targets = selector.getRelevantTargets();

    // Re-evaluate velocity trends to highlight target kinetic deviations
    updateDebrisLikelihood();
    
    // Quick reset check to cut frame timeout early for a bad candidate
    if ( debris_id != -1 && target_debris_count[debris_id] == 0 ) {
        debris_id = -1;
    }

    // Color frame
    // array of colors for tracking frame boxes  red, orange, yellow, green, blue, purple, pink
    std::vector<cv::Scalar> COLORS = { cv::Scalar(0,0,255),cv::Scalar(0,127,255),cv::Scalar(0,255,255),cv::Scalar(0,255,0),cv::Scalar(255,0,0),cv::Scalar(127,0,127),cv::Scalar(191,191,255) };
    for (size_t i = 0; i < next_targets.size(); i++) {
        if ( next_targets[i]->getPrevInstancePtr() == nullptr ) {
            cv::circle(frame, cv::Point(next_targets[i]->getX(), next_targets[i]->getY()), 5, COLORS[next_targets[i]->getID() % 7], -1);
        } else {
            cv::circle(frame, cv::Point(next_targets[i]->getX(), next_targets[i]->getY()), 10, COLORS[next_targets[i]->getID() % 7], 2);
        }
        
    }

    Target* saved_alt_target = nullptr;
    
    // Parse through tracked metrics to identity anomaly candidate conditions
    for ( size_t j = 0; j < relevant_targets.size(); j++) {
        Target* target = relevant_targets[j];

        // Core conditional gate: Flag entity if its outlier threshold score has been breached
        if ( target_debris_count[target->getID()] > DEBRIS_THRESHOLD ) {
            
            // Branch A: Confirmed persistence; the anomalous candidate aligns with our tracking target
            if ( target->getID() == debris_id && debris_id != -1 ) {
                cv::circle(frame, cv::Point(target->getX(), target->getY()), 17, cv::Scalar(255, 255, 255), 7);
                return debris_id;
            } 
            // Branch B: Anomaly found but its ID conflicts with what we are currently monitoring
            else {
                // Buffer the highest-likelihood alternative candidate found as a fallback trace option
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

    // Sticky tracking: if the locked target isn't over-gate or present this frame but is
    // still within its occlusion grace window, retain it rather than dropping the lock.
    if ( debris_id != -1 && full_target_list[debris_id]->getFrameNum() + frame_timeout >= current_frame_number ) {
        return debris_id;
    }
    
    // Fallback: If primary target drops but an alternative candidate was buffered, switch tracking focus
    if ( saved_alt_target != nullptr ) {
        cv::circle(frame, cv::Point(saved_alt_target->getX(), saved_alt_target->getY()), 17, cv::Scalar(255, 255, 255), 7);
        return saved_alt_target->getID();
    }

    return -1;
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
void Sentry::updateDebrisLikelihood() {

    // --- Tuning knobs -------------------------------------------------------
    // SPEED_NOISE_FLOOR: how far (pixels/frame) a target must exceed the group's
    // median speed before any score is added. This absorbs star jitter and small
    // Kalman wobble so slow stars never creep over the threshold. Stars in this
    // footage drift < 3 px/frame, so a floor around there isolates real outliers.
    const float SPEED_NOISE_FLOOR = 0.5f;
    // SCORE_GAIN: scales excess-speed (in px/frame, above the floor) into score
    // units added per frame. With gain 1.0, an object moving 5 px/frame faster
    // than the floor adds ~5 per frame and clears a threshold of ~12 in 3 frames.
    const float SCORE_GAIN = 1.0f;
    // DECAY: how much score bleeds off per frame for a target sitting below the
    // floor, so a brief mis-track doesn't leave a star stuck with stale score.
    const int DECAY = 6;
    // -----------------------------------------------------------------------

    // Establish the reference group movement baseline as a speed magnitude.
    std::vector<float> median_velocity = selector.getMedianTargetVelocity();

    // Compare each target's speed against the group baseline.
    std::vector<Target*> relevant_targets = selector.getRelevantTargets();
    for (size_t i = 0; i < relevant_targets.size(); i++) {
        Target* target = relevant_targets[i];

        // Target velocity difference magnitude (direction-agnostic: fast in any direction counts).
        
        float velocity_diff = std::sqrt( std::pow( (median_velocity[0]-target->getVx()) ,2) + std::pow( (median_velocity[1]-target->getVy()) ,2) );

        // Excess speed above the group, with the noise floor subtracted out.
        float excess = velocity_diff - SPEED_NOISE_FLOOR;

        if ( excess > 0.0f ) {
            // Proportional reward: faster relative motion -> bigger score jump.
            int gain = static_cast<int>( excess * SCORE_GAIN );
            //if ( gain < 1 ) { gain = 1; } // a clear outlier always earns at least 1
            target_debris_count[target->getID()] += gain;
        } else {
            // Below the floor: decay toward zero, clamped so scores never go negative.
            target_debris_count[target->getID()] -= DECAY;
            if ( target_debris_count[target->getID()] < 0 ) {
                target_debris_count[target->getID()] = 0;
            }
        }

        target->setDebrisLikelihood( target_debris_count[target->getID()] );
    }
}
