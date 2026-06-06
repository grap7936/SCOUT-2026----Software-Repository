#ifndef SELECTOR_H
#define SELECTOR_H

#include <vector>
#include <math.h>
#include <Target.hpp>
#include <Graph.hpp>

class Selector {

public:

    int frame_w;
    int frame_h;
    float max_norm;

    std::vector<Target>* prev_targets;
    std::vector<Target>* next_targets;

    Selector( int, int );

    std::vector<Target>* getPrevTargetsPtr();

    void setPrevTargetsPtr( std::vector<Target>* );

    std::vector<Target>* getNextTargetsPtr();

    void setNextTargetsPtr( std::vector<Target>* );

    void weight( Target* );

    void handleConflict( Target*, Target* );

    void connect( float );

    // void starWeight(); move to Sentry Class?

    void scan( std::vector<Target>*, std::vector<Target>* );

};

#endif