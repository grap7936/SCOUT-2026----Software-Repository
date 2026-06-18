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

public:

    float threshold;

    std::vector<Target*>* prev_targets;
    std::vector<Target*>* next_targets;
    std::vector<Target*>* full_list;

    Selector( int );

    Selector();

    std::vector<Target*>* getPrevTargetsPtr();

    void setPrevTargetsPtr( std::vector<Target*>* );

    std::vector<Target*>* getNextTargetsPtr();

    void setNextTargetsPtr( std::vector<Target*>* );

    std::vector<Target*>* getFullTargetListPtr();

    void setFullTargetListPtr( std::vector<Target*>* );

    void initTarget( Target*, float, float );

    void weight( Target* );

    void connect( float, float );

    std::vector<int> hungarianAlgorithm( std::vector<std::vector<int>> );

    void estimateNextState();

    void updateEstimate();

    void scan( std::vector<Target*>*, std::vector<Target*>*, std::vector<Target*>*, float, float );

};

#endif