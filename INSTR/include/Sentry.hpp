#ifndef SENTRY_HPP
#define SENTRY_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include "Target.hpp"
#include "Selector.hpp"
#include "Detector.hpp"

class Sentry {

public:
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

    Sentry(int);

    void init( cv::Mat );

    void pageFrame( cv::Mat, float, float );

    void setNextFrame( cv::Mat );

    std::vector<Target*>* getFullListPtr();

    std::vector<Target*>* getPrevTargetPtr();

    std::vector<Target*>* getNextTargetPtr();

    cv::Mat getNextFrame();

    void setPrevFrame( cv::Mat );

    cv::Mat getPrevFrame();

    std::vector<int> getTargetCoords( int );

    int getNumTargets();

    void clearPrevTargets();

    void clearNextTargets();

    int findDebris( cv::Mat, int);

    std::vector<Target*> getRelevantTargets();

    std::vector<float> getMeanTargetVelocity( std::vector<Target*>& relevant_targets );
    std::vector<float> getMedianTargetVelocity( std::vector<Target*>& relevant_targets );

    void updateDebrisLikelihood( std::vector<Target*>& relevant_targets );
};

#endif
