#ifndef SELECTOR_HPP
#define SELECTOR_HPP

#include <vector>
#include <math.h>
#include <algorithm>
#include <memory>
#include <opencv2/opencv.hpp>
#include "Target.hpp"
#include "Graph.hpp"

class Selector {
private:
    int threshold;

    std::vector<Target*>* prev_targets;
    std::vector<Target*>* next_targets;
    std::vector<Target*>* full_list;
    std::vector<cv::KalmanFilter> kf_list;

    
    int current_frame_num, frame_timeout;
    std::vector<Target*> current_relevant_targets;
    float current_mean_vx, current_mean_vy;
    float current_median_vx, current_median_vy;

public:

    Selector( int thresh, int timeout );

    Selector();

    int getThreshold();

    void setThreshold( int thresh );

    std::vector<Target*>* getPrevTargets();

    void setPrevTargets( std::vector<Target*>* targets );

    std::vector<Target*>* getNextTargets();

    void setNextTargets( std::vector<Target*>* targets );

    std::vector<Target*>* getFullTargetList();

    void setFullTargetList( std::vector<Target*>* targets );

    std::vector<cv::KalmanFilter> getKFList(); // new


    int getCurrentFrameNum();

    void setCurrentFrameNum(int frame_num);

    int getFrameTimeout();

    void setFrameTimeout(int timeout);

    void determineRelevantTargets();

    std::vector<Target*> getRelevantTargets();

    void calculateMeanVelocity();

    std::vector<float> getMeanTargetVelocity();

    void calculateMedianVelocity();

    std::vector<float> getMedianTargetVelocity();


    void initTarget( Target* new_target, float est_vx, float est_vy );

    //void weight( Target* ); sent to Graph Class

    void computeWeights( float gain1 );

    void connect();

    std::vector<int> hungarianAlgorithm( std::vector<std::vector<int>> cost_matrix );

    void estimateNextState();

    void updateEstimate();

    void scan( std::vector<Target*>* prev, std::vector<Target*>* next, std::vector<Target*>* full, int frame_num );

};

#endif