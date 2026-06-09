#ifndef SENTRY_HPP
#define SENTRY_HPP

#include <vector>
#include <opencv4/opencv2/opencv.hpp>
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
    Detector detector;
    Selector selector;

    Sentry(int, int, float);

    void init( cv::Mat );

    void pageFrame( cv::Mat );

    void setNextFrame( cv::Mat );

    void setPrevFrame( cv::Mat );

    std::vector<int> getTargetCoords( int );

    void appendTarget( Target* );

    void removeTarget( int );

    int getNumTargets();

    void clearPrevTargets();

    void clearNextTargets();
};

#endif