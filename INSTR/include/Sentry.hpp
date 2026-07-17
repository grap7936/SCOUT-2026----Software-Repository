#ifndef SENTRY_HPP
#define SENTRY_HPP

#include <vector>
#include <fstream>
#include <iostream>
#include <omp.h>
#include <opencv2/opencv.hpp>
#include "Target.hpp"
#include "Selector.hpp"
#include "Detector.hpp"

class Sentry {
private:

    std::string DEBRIS_LOG_FILENAME;
    std::string TARGET_LOG_FILENAME;

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
    

    long long int current_frame_number;
    
    // cv::Mat prev_frame;
    // cv::Mat next_frame;
    std::vector<Target*> full_target_list;
    std::vector<Target*> prev_targets;
    std::vector<Target*> next_targets;
    std::vector<int> target_debris_count;
    Detector detector;
    Selector selector;

public:

    Sentry();

    // Setters for primary parameters
    void setTrackerParams( int thresh, int decay, float noise_floor, float score_gain );

    void setDetectorParams( int refresh_freq, int blur_size, int thresh_margin, int dilation_iter, int contour_size );

    void setSelectorParams( int close_thresh, int frame_timeout, float weight_comp );

    void setAllParams( int thresh, int decay, float noise_floor, float score_gain, int refresh_freq, int blur_size, int thresh_margin, int dilation_iter, int contour_size, int close_thresh, int frame_timeout, float weight_comp );


    void init( cv::Mat&, int);

    void pageFrame( cv::Mat& );

    // getters for primary lists
    std::vector<Target*>* getFullListPtr();

    std::vector<Target*>* getPrevTargetPtr();

    std::vector<Target*>* getNextTargetPtr();

    // setters and getters for frames, selector, and detecctor
    // void setNextFrame( cv::Mat );

    // cv::Mat getNextFrame();

    // void setPrevFrame( cv::Mat );

    // cv::Mat getPrevFrame();

    Detector* getDetectorPtr();

    Selector* getSelectorPtr();

    // returns a target's coordinates indexed by id
    std::vector<int> getTargetCoords( int );

    int getNumTargets();

    // clear lists
    void clearPrevTargets();

    void clearNextTargets();

    // primary function
    int findDebris( cv::Mat&, int, long long int);

    void updateDebrisLikelihood();

    // storage management
    void writeTargetsToFile(std::vector<Target*> target_list, std::string filename, bool print_all_instances);

    void dumpOldTargets(int cutoff_index);
};

#endif