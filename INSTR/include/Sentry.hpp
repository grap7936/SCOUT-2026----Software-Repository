#ifndef SENTRY_HPP
#define SENTRY_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include "Target.hpp"
#include "Selector.hpp"
//#include "Detector.hpp"

class Sentry {

public:
    cv::Mat prev_frame;
    cv::Mat next_frame;
    std::vector<Target*> full_target_list;
    std::vector<Target*> prev_targets;
    std::vector<Target*> next_targets;
    std::vector<int> target_debris_count;
    Detector detector;
    Selector selector;

    Sentry(int, int, float);

    void init( cv::Mat );

    void pageFrame( cv::Mat );

    void setNextFrame( cv::Mat );

    cv::Mat getNextFrame();

    void setPrevFrame( cv::Mat );

    cv::Mat getPrevFrame();

    std::vector<int> getTargetCoords( int );

    int getNumTargets();

    void clearPrevTargets();

    void clearNextTargets();

    int findDebris( cv::Mat );

    std::vector<Target*> getRelevantTargets();

    std::vector<float> getMeanTargetVelocity( std::vector<Target*> relevant_targets );

    void updateDebrisLikelihood( std::vector<Target*> relevant_targets );
};

#endif