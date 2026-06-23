#include "Sentry.hpp"
#include "Detector.hpp"


Sentry::Sentry(int w, int h, float thresh) {
    full_target_list = {};
    std::vector<int> target_debris_count = {};
    prev_targets = {};
    next_targets = {};
    Detector detector;
    Selector selector(thresh);
}

void Sentry::init( cv::Mat frame ) {
    setNextFrame( frame );
    detector.scan( frame, next_targets, 0 );
    for (int i = 0; i < next_targets.size(); i++) {
        full_target_list.push_back( next_targets[i] );
        target_debris_count.push_back( 0 );
    }

}

void Sentry::pageFrame( cv::Mat frame ) {

    setPrevFrame( getNextFrame() ); // set previous frame to be current next_frame
    setNextFrame( frame ); // set new frame to next frame

    clearPrevTargets();
    for (int i = 0; i < next_targets.size(); i++) {
        prev_targets.push_back(next_targets[i]);
    }
    clearNextTargets();
    detector.scan( frame, next_targets );
    int old_full_list_size = full_target_list.size();
    selector.scan(&prev_targets, &next_targets, &full_target_list);
    int new_full_list_size = full_target_list.size();
    for (int j = old_full_list_size; j < new_full_list_size; j++ ) {
        target_debris_count.push_back( 0 );
    }

}

void Sentry::setNextFrame( cv::Mat frame ) {
    next_frame = frame;
}

cv::Mat Sentry::getNextFrame() {
    return next_frame;
}

void Sentry::setPrevFrame( cv::Mat frame ) {
    prev_frame = frame;
}

cv::Mat Sentry::getPrevFrame() {
    return prev_frame;
}

std::vector<int> Sentry::getTargetCoords( int id ) {
    std::vector<int> position = {0, 0};
    position[0] = full_target_list[id]->x;
    position[1] = full_target_list[id]->y;
    return position;
}

int Sentry::getNumTargets() {
    return full_target_list.size();
}

void Sentry::clearPrevTargets() {
    while ( prev_targets.size() > 0 ) {
        prev_targets.pop_back();
    }
}

void Sentry::clearNextTargets() {
    while ( next_targets.size() > 0 ) {
        next_targets.pop_back();
    }
}

int Sentry::findDebris( cv::Mat frame ) {
    // check if initial frame
    if ( full_target_list.size() == 0 ) {
        init( frame );
        return -1;
    }

    pageFrame( frame );

    std::vector<Target*> relevant_targets = getRelevantTargets();

    updateDebrisLikelihood( relevant_targets );

    for ( int j = 0; j < relevant_targets.size(); j++) {
        Target target = *(relevant_targets[j]);

        if ( target_debris_count[j] > 2 ) {
            return target.id;
        }
    }
    return -1;

}

std::vector<Target*> Sentry::getRelevantTargets() {
    int size = full_target_list.size();

    std::vector<Target*> relevant_targets = {};

    for (int i = 0; i < size; i++) {
        Target* curr_instance = full_target_list[i];
        if (curr_instance->prev_instance != NULL) {
            relevant_targets.push_back( curr_instance );
        }
    }

    return relevant_targets;
}

std::vector<float> Sentry::getMeanTargetVelocity( std::vector<Target*> relevant_targets ) {
    int size = relevant_targets.size();
    float x_sum = 0.0;
    float y_sum = 0.0;

    for (int i = 0; i < size; i++) {
        x_sum += relevant_targets[i]->getVx();
        y_sum += relevant_targets[i]->getVy();
    }

    std::vector<float> mean = { x_sum/size, y_sum/size };
    return mean;
}

void Sentry::updateDebrisLikelihood( std::vector<Target*> relevant_targets ) {

    std::vector<float> mean_velocity = getMeanTargetVelocity( relevant_targets );

    if ( mean_velocity[0] == 0.0 || mean_velocity[1] == 0.0 ) {
        return;
    }

    float vx_thresh = 0.5;
    float vy_thresh = 0.5;

    for (int i = 0; i < relevant_targets.size(); i++) {
        Target* target = relevant_targets[i];
        float vx_diff = ( mean_velocity[0] - target->getVx() ) / mean_velocity[0];
        if (vx_diff < 0) { vx_diff = -vx_diff; }
        float vy_diff = ( mean_velocity[1] - target->getVy() ) / mean_velocity[1];
        if (vy_diff < 0) { vy_diff = -vy_diff; }

        if ( vx_diff > vx_thresh || vy_diff > vy_thresh ) {
            target->incDebrisLikelihood();
            target_debris_count[target->id] = target->getDebrisLikelihood();
        }
    }
}

void Sentry::writeTargetsToFile(std::vector<Target*> full_target_list) {

// Define output stream -- preallocate
std::ofstream Saved_Target_Data; 

// If this is the FIRST time data is being saved, then: Define output stream -- txt file to write target pointer list data
if (is_first_save == true)
    {
    Saved_Target_Data.open("Saved_Target_Data.txt"); // opens/creates necessary text file for inputting data into
    is_first_save = false; // set is_first_save parameter to false so that every subsequent time this function is called it appends data and doesn't create any new text file to write into
    }

else 
    {
    Saved_Target_Data.open("Saved_Target_Data.txt", std::ios::app); // append data to the text file
    }


    // Test if stream operatiopn failed
    if (Saved_Target_Data.fail()) 
    {
	    std::cout << "Error opening the input file."; 
	    return;
    }

// Write Target data to created text file by looping through all entries -- uses range based for loop
for (Target* target : full_target_list)
{

    // Create a pointer to go through the current linked list when reading through the list of linked lists
    Target* current = target;

    // Loop through the linked list until reaching the end which is signified by a nullptr
    while (current != nullptr)
    {
        Saved_Target_Data << "Target ID: " << current->id << "\t" << "Position: (" << current->x << ", " << current->y << ")\n"
                          << "Velocity: (" << current->getVx() << ", " << current->getVy() << ") \n"
                          << "Debris Likelihood: " << current->getDebrisLikelihood() << "\n";

        // Move to the next target in this linked list by accessing the forward pointer defined in the target class (target.hpp)
        current = current->next_instance;
    }

}

Saved_Target_Data.close(); // close text file being written into until next function call occurs and appends more information.

}




