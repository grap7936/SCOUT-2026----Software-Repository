#ifndef SENTRY_HPP
#define SENTRY_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include "Target.hpp"
#include "Selector.hpp"
#include "Detector.hpp"

class Sentry {
private:

    int DEBRIS_THRESHOLD;
    int REFRESH_FREQUENCY;

    int current_frame_number;
    int frame_timeout;
    cv::Mat prev_frame;
    cv::Mat next_frame;
    std::vector<Target*> full_target_list;
    std::vector<Target*> prev_targets;
    std::vector<Target*> next_targets;
    std::vector<int> target_debris_count;
    Detector detector;
    Selector selector;

public:

    Sentry(int);

    void init( cv::Mat );

    void pageFrame( cv::Mat );

    std::vector<Target*>* getFullListPtr();

    std::vector<Target*>* getPrevTargetPtr();

    std::vector<Target*>* getNextTargetPtr();

    void setNextFrame( cv::Mat );

    cv::Mat getNextFrame();

    void setPrevFrame( cv::Mat );

    cv::Mat getPrevFrame();

    Detector* getDetectorPtr();

    Selector* getSelectorPtr();

    std::vector<int> getTargetCoords( int );

    int getNumTargets();

    void clearPrevTargets();

    void clearNextTargets();

    int findDebris( cv::Mat, int);


    void updateDebrisLikelihood();
};

#endif