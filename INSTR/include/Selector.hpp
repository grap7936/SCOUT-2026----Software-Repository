#ifndef SELECTOR_HPP
#define SELECTOR_HPP

#include <vector>
#include <math.h>
#include "Target.hpp"
#include "Graph.hpp"

class Selector {

public:

    int frame_w;
    int frame_h;
    float max_norm;
    float threshold;

    std::vector<Target*>* prev_targets;
    std::vector<Target*>* next_targets;

    Selector( int, int, float );

    std::vector<Target*>* getPrevTargetsPtr();

    void setPrevTargetsPtr( std::vector<Target*>* );

    std::vector<Target*>* getNextTargetsPtr();

    void setNextTargetsPtr( std::vector<Target*>* );

    void weight( Target* );

    void handleConflict( Target*, Target* );

    void connect();

    // void starWeight(); move to Sentry Class?

    void scan( std::vector<Target*>*, std::vector<Target*>* );

};

#endif