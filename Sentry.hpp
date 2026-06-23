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

public:
    cv::Mat prev_frame;
    cv::Mat next_frame;
    std::vector<Target*> full_target_list;
    std::vector<Target*> prev_targets;
    std::vector<Target*> next_targets;
    std::vector<int> target_debris_count;
    Detector detector;
    Selector selector;

private:
    bool is_first_save = true; // used in writeTargetsToFile to determine if a new text file must be created to write information into 
                               // -- this starts as true so that the 1st save creates new info and then is changed in the riteTargetsToFile() 
                               // function to False for every subsequent case.

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

    void pageFrame( cv::Mat frame, int frame_num );
    
    int findDebris( cv::Mat frame, int frame_num );

    std::vector<Target*> getRelevantTargets();

    std::vector<float> getMeanTargetVelocity( std::vector<Target*> relevant_targets );

    void updateDebrisLikelihood( std::vector<Target*> relevant_targets );

    void Sentry::writeTargetsToFile(std::vector<Target*> full_target_list);

    void Sentry::dumpOldTargets();  
};

#endif
