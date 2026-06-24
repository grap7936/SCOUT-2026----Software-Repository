#ifndef SENTRY_HPP
#define SENTRY_HPP

#include <vector>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Target.hpp"
#include "Selector.hpp"
#include "Detector.hpp"

class Sentry {
private:

    int TRACKER_DEBRIS_THRESHOLD;
    int TRACKER_DECAY;
    float TRACKER_SPEED_NOISE_FLOOR;
    float TRACKER_SCORE_GAIN;
    

    int DETECTOR_BG_REFRESH_FREQUENCY;
    int DETECTOR_BLUR_KERNEL_SIZE;
    int DETECTOR_BG_THRESHOLD_MARGIN;
    int DETECTOR_DILATION_ITERATIONS;
    int DETECTOR_MAX_CONTOUR_SIZE;

    int SELECTOR_CLOSENESS_THRESHOLD;
    int SELECTOR_FRAME_TIMEOUT;
    float SELECTOR_WEIGHT_COMPOSITION;
    

    int current_frame_number;
    bool is_first_save = true; // used in writeTargetsToFile to determine if a new text file must be created to write information into 
                               // -- this starts as true so that the 1st save creates new info and then is changed in the riteTargetsToFile() 
                               // function to False for every subsequent case.
    cv::Mat prev_frame;
    cv::Mat next_frame;
    std::vector<Target*> full_target_list;
    std::vector<Target*> prev_targets;
    std::vector<Target*> next_targets;
    std::vector<int> target_debris_count;
    Detector detector;
    Selector selector;

public:

    Sentry();

    void setTrackerParams( int thresh, int decay, float noise_floor, float score_gain );

    void setDetectorParams( int refresh_freq, int blur_size, int thresh_margin, int dilation_iter, int contour_size );

    void setSelectorParams( int close_thresh, int frame_timeout, float weight_comp );

    void setAllParams( int thresh, int decay, float noise_floor, float score_gain, int refresh_freq, int blur_size, int thresh_margin, int dilation_iter, int contour_size, int close_thresh, int frame_timeout, float weight_comp );

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

    void writeTargetsToFile(std::vector<Target*> full_target_list);

    void dumpOldTargets();
};

#endif