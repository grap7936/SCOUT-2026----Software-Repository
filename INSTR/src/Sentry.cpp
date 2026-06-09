#include "Sentry.hpp"
#include "Detector.hpp"

Sentry::Sentry(int w, int h, float thresh) {
    full_target_list = {};
    prev_targets = {};
    next_targets = {};
    Detector detector;
    Selector selector(thresh);
}

void Sentry::init( cv::Mat frame ) {
    setNextFrame( frame );
    detector.scan( frame, next_targets );
}

void Sentry::pageFrame( cv::Mat frame ) {

    setPrevFrame( next_frame ); // set current next frame to be previous frame
    setNextFrame( frame ); // set new frame to next frame

    clearPrevTargets();
    for (int i = 0; i < next_targets.size(); i++) {
        prev_targets.push_back(next_targets[i]);
    }
    clearNextTargets();
    //detector.scan( frame, &next_targets )
    selector.scan(&prev_targets, &next_targets, &full_target_list);

}

void Sentry::setNextFrame( cv::Mat frame ) {
    next_frame = frame;
}

void Sentry::setPrevFrame( cv::Mat frame ) {
    prev_frame = frame;
}

std::vector<int> Sentry::getTargetCoords( int id ) {
    std::vector<int> position = {0, 0};
    position[0] = full_target_list[id]->x;
    position[1] = full_target_list[id]->y;
    return position;
}

void Sentry::appendTarget( Target* new_target ) {
    full_target_list.push_back(new_target);
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