#ifndef SELECTOR_HPP
#define SELECTOR_HPP

#include <vector>
#include <math.h>
#include <omp.h>
#include <algorithm>
#include <memory>
#include <array>
#include <climits>
#include <opencv2/opencv.hpp>
#include "Target.hpp"
#include "Graph.hpp"

class Selector {
private:
    int THRESHOLD;
    int FRAME_TIMEOUT;
    float WEIGHT_COMPOSITION;
    std::string TARGET_LOG_FILENAME;

    std::vector<Target*>* prev_targets;
    std::vector<Target*>* next_targets;
    std::vector<Target*>* full_list;
    std::vector<cv::KalmanFilter> kf_list;

    int relevant_target_list_offset;
    std::vector<bool> is_timed_out;
    
    long long int current_frame_num;
    std::vector<Target*> current_relevant_targets;
    float current_mean_vx, current_mean_vy;
    float current_median_vx, current_median_vy;

public:

    // Constructor
    Selector( int thresh, int timeout, int weight_comp );

    // Backup constructor
    Selector();

    // Getters and Setters of control parameters and variables
    int getThreshold();

    void setThreshold( int thresh );

    int getFrameTimeout();

    void setFrameTimeout(int timeout);

    float getWeightComposition();

    void setWeightComposition(float gain1);

    int getCurrentFrameNum();

    void setCurrentFrameNum(int frame_num);

    int getTargetListOffset();

    void setTargetListOffset(int offset);


    // Getters and Setters of Lists (all pointers except KF)
    std::vector<Target*>* getPrevTargets();

    void setPrevTargets( std::vector<Target*>* targets );

    std::vector<Target*>* getNextTargets();

    void setNextTargets( std::vector<Target*>* targets );

    std::vector<Target*>* getFullTargetList();

    void setFullTargetList( std::vector<Target*>* targets );

    std::vector<cv::KalmanFilter> getKFList(); // new


    // Calculate and return functions of linked targets and their speed metrics
    std::vector<Target*> getRelevantTargets();

    std::vector<float> getMeanTargetVelocity();
    
    std::vector<float> getMedianTargetVelocity();

    void determineRelevantTargets();

    void updateRTOffset(); // helper function to determineRelevantTargets()

    void calculateMeanVelocity();

    void calculateMedianVelocity();


    //////////////// Member Functions that do the heavy lifting ///////////////////////////////

    // parent function that compares target lists to link ids across frames
    void scan( std::vector<Target*>* prev, std::vector<Target*>* next, std::vector<Target*>* full, int frame_num );

    // estimate the next position of targets in prev_targets
    void estimateNextState();

    // compute graph weights for hungarian selector
    void computeWeights();

    // compute vector matching targets based on solution
    std::vector<int> hungarianAlgorithm( std::vector<std::vector<int>> cost_matrix );

    // connect target prev/next instance pointers
    void connect(); // also calls determineRelevantTargets() and calculateMedianVelocity()
    

    // intializer for unlinked targets
    void initTarget( Target* new_target, float est_vx, float est_vy );

    // update position estimate of prev_targets based on connect() results
    void updateEstimate();

};

#endif