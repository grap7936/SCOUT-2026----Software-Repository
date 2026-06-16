#include <iostream>
#include <vector>
#include <cassert> // Required for the assert() command
#include "Graph.hpp"
#include "Target.hpp"
#include "Selector.hpp"

/* Compile command from INSTR/ directory
    g++ -I ./include -I C:/msys64/ucrt64/include/opencv4 testing/selectorTest.cpp src/Graph.cpp src/Selector.cpp src/Target.cpp -o selectorTest.out -LC:/msys64/ucrt64/lib -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_video -lopencv_videoio -lopencv_imgcodecs -lopencv_objdetect -static
*/

// Function declarations
void initKF(Target* new_target);

Selector defineSelector(int threshold, 
                       std::vector<int>& id_f1, std::vector<int>& coords_f1, 
                       std::vector<int>& id_f2, std::vector<int>& coords_f2, 
                       std::vector<Target*>& target_f1, std::vector<Target*>& target_f2, std::vector<Target*>& target_full);

void printResults(std::vector<Target*>& target_f1, std::vector<Target*>& target_f2);

void cleanupTestMemory(std::vector<Target*>& target_f1, std::vector<Target*>& target_f2, Selector selector);


int main() {
    float threshold = 3500;

    // =========================================================================
    // TEST CONFIGURATION 1
    // =========================================================================
    std::vector<int> id1_f1 = { 0, 1, 2 };
    std::vector<int> coords1_f1 = { 120,200, 340,10, 420,370 };
    std::vector<int> id1_f2 = { 3, 4, 5 };
    std::vector<int> coords1_f2 = { 360,100, 430,460, 150,220 };
    std::vector<Target*> target1_f1 = {};
    std::vector<Target*> target1_f2 = {};
    std::vector<Target*> target1_full = {};

    Selector selector1 = defineSelector(threshold, id1_f1, coords1_f1, id1_f2, coords1_f2, target1_f1, target1_f2, target1_full);
    std::vector<int> expected_f2_ids_t1 = { 1, 2, 0 }; 
    std::vector<bool> expected_has_next_t1 = { true, true, true };

    // =========================================================================
    // TEST CONFIGURATION 2
    // =========================================================================
    std::vector<int> id2_f1 = { 0, 1, 2, 3 };
    std::vector<int> coords2_f1 = { 150,210, 330,30, 520,20, 210,440 };
    std::vector<int> id2_f2 = { 4, 5, 6, 7, 8, 9 };
    std::vector<int> coords2_f2 = { 160,300, 530,120, 400,80, 240,10, 10,350, 400,320 };
    std::vector<Target*> target2_f1 = {};
    std::vector<Target*> target2_f2 = {};
    std::vector<Target*> target2_full = {};

    Selector selector2 = defineSelector(threshold, id2_f1, coords2_f1, id2_f2, coords2_f2, target2_f1, target2_f2, target2_full);
    std::vector<int> expected_f2_ids_t2 = { 0, 2, 1, 4, 5, 6 }; 
    std::vector<bool> expected_has_next_t2 = { true, true, true, false };

    // =========================================================================
    // TEST CONFIGURATION 3
    // =========================================================================
    std::vector<int> id3_f1 = { 0, 1, 2, 3 };
    std::vector<int> coords3_f1 = { 110,100, 150,100, 240,350, 350,400 };
    std::vector<int> id3_f2 = { 4, 5, 6, 7, 8 };
    std::vector<int> coords3_f2 = { 130,190, 160,200, 10,300, 250,450, 500,20 };
    std::vector<Target*> target3_f1 = {};
    std::vector<Target*> target3_f2 = {};
    std::vector<Target*> target3_full = {};

    Selector selector3 = defineSelector(threshold, id3_f1, coords3_f1, id3_f2, coords3_f2, target3_f1, target3_f2, target3_full);
    std::vector<int> expected_f2_ids_t3 = { 0, 1, 4, 3, 5 }; 
    std::vector<bool> expected_has_next_t3 = { true, true, false, true };

    // =========================================================================
    // TEST CONFIGURATION 4
    // =========================================================================
    std::vector<int> id4_f1 = { 0, 1, 2, 3 };
    std::vector<int> coords4_f1 = { 110,100, 150,100, 240,350, 10,10 };
    std::vector<int> id4_f2 = { 4, 5, 6 };
    std::vector<int> coords4_f2 = { 130,190, 250,450, 500,20 };
    std::vector<Target*> target4_f1 = {};
    std::vector<Target*> target4_f2 = {};
    std::vector<Target*> target4_full = {};

    Selector selector4 = defineSelector(threshold, id4_f1, coords4_f1, id4_f2, coords4_f2, target4_f1, target4_f2, target4_full);
    std::vector<int> expected_f2_ids_t4 = { 0, 2, 4 }; 
    std::vector<bool> expected_has_next_t4 = { true, false, true, false };


    // =========================================================================
    // TEST CONFIGURATION 5 (HARD: Path Intersections / Criss-Cross Mapping)
    // =========================================================================
    // Target 0 (100,100) moves to (130,130) -> dist^2 = 1800 (Linkable)
    // Target 1 (130,100) moves to (100,130) -> dist^2 = 1800 (Linkable)
    // Distance from (100,100) to (100,130) is 900. Checks if vector tracking prevents mis-association.
    std::vector<int> id5_f1 = { 0, 1 };
    std::vector<int> coords5_f1 = { 100,100, 130,100 };
    std::vector<int> id5_f2 = { 2, 3 };
    std::vector<int> coords5_f2 = { 130,130, 100,130 };
    std::vector<Target*> target5_f1 = {};
    std::vector<Target*> target5_f2 = {};
    std::vector<Target*> target5_full = {};

    Selector selector5 = defineSelector(threshold, id5_f1, coords5_f1, id5_f2, coords5_f2, target5_f1, target5_f2, target5_full);
    std::vector<int> expected_f2_ids_t5 = { 0, 1 }; // Presuming stable tracker keeps assignments
    std::vector<bool> expected_has_next_t5 = { true, true };


    // =========================================================================
    // TEST CONFIGURATION 6 (EDGE: All Targets Leave Frame / Sudden Drop to 0)
    // =========================================================================
    std::vector<int> id6_f1 = { 0, 1 };
    std::vector<int> coords6_f1 = { 50,50, 600,400 };
    std::vector<int> id6_f2 = {}; // Suddenly empty frame
    std::vector<int> coords6_f2 = {}; 
    std::vector<Target*> target6_f1 = {};
    std::vector<Target*> target6_f2 = {};
    std::vector<Target*> target6_full = {};

    Selector selector6 = defineSelector(threshold, id6_f1, coords6_f1, id6_f2, coords6_f2, target6_f1, target6_f2, target6_full);
    std::vector<int> expected_f2_ids_t6 = {}; 
    std::vector<bool> expected_has_next_t6 = { false, false };


    // =========================================================================
    // TEST CONFIGURATION 7 (EDGE: Perfect Distance Tie-Breaker)
    // =========================================================================
    // Target 0 is at (200, 200). Frame 2 has targets at (200, 230) and (200, 170). 
    // Both are exactly a distance of 30 pixels away (dist^2 = 900). Evaluates tie-breaking criteria.
    std::vector<int> id7_f1 = { 0 };
    std::vector<int> coords7_f1 = { 200,200 };
    std::vector<int> id7_f2 = { 1, 2 }; 
    std::vector<int> coords7_f2 = { 200,230, 200,170 }; 
    std::vector<Target*> target7_f1 = {};
    std::vector<Target*> target7_f2 = {};
    std::vector<Target*> target7_full = {};

    Selector selector7 = defineSelector(threshold, id7_f1, coords7_f1, id7_f2, coords7_f2, target7_f1, target7_f2, target7_full);
    std::vector<int> expected_f2_ids_t7 = { 0, 1 }; // Checks mapping structure execution
    std::vector<bool> expected_has_next_t7 = { true };


    // =========================================================================
    // TEST CONFIGURATION 8 (EDGE: Ultimate Corner Boundaries / 639,479 Maxima)
    // =========================================================================
    // Frame 1 has targets sitting on extreme coordinate boundaries.
    std::vector<int> id8_f1 = { 0, 1 };
    std::vector<int> coords8_f1 = { 0,0, 639,479 };
    std::vector<int> id8_f2 = { 2, 3 }; 
    std::vector<int> coords8_f2 = { 10,10, 629,469 }; // Small diagonal creep away from boundary edges
    std::vector<Target*> target8_f1 = {};
    std::vector<Target*> target8_f2 = {};
    std::vector<Target*> target8_full = {};

    Selector selector8 = defineSelector(threshold, id8_f1, coords8_f1, id8_f2, coords8_f2, target8_f1, target8_f2, target8_full);
    std::vector<int> expected_f2_ids_t8 = { 0, 1 }; 
    std::vector<bool> expected_has_next_t8 = { true, true };


    // =========================================================================
    // RUN ALGORITHM SCANS
    // =========================================================================
    selector1.scan( &target1_f1, &target1_f2, &target1_full );
    selector2.scan( &target2_f1, &target2_f2, &target2_full );
    selector3.scan( &target3_f1, &target3_f2, &target3_full );
    selector4.scan( &target4_f1, &target4_f2, &target4_full );
    selector5.scan( &target5_f1, &target5_f2, &target5_full );
    selector6.scan( &target6_f1, &target6_f2, &target6_full );
    selector7.scan( &target7_f1, &target7_f2, &target7_full );
    selector8.scan( &target8_f1, &target8_f2, &target8_full );
    

    // =========================================================================
    // PRINT LOG OUTPUTS
    // =========================================================================
    std::cout << "Test 1 Results:" << std::endl; printResults ( target1_f1, target1_f2 );
    std::cout << "\nTest 2 Results:" << std::endl; printResults ( target2_f1, target2_f2 );
    std::cout << "\nTest 3 Results:" << std::endl; printResults ( target3_f1, target3_f2 );
    std::cout << "\nTest 4 Results:" << std::endl; printResults ( target4_f1, target4_f2 );
    std::cout << "\nTest 5 Results (Cross Over):" << std::endl; printResults ( target5_f1, target5_f2 );
    std::cout << "\nTest 6 Results (Mass Exit):" << std::endl; printResults ( target6_f1, target6_f2 );
    std::cout << "\nTest 7 Results (Tie-Breaker):" << std::endl; printResults ( target7_f1, target7_f2 );
    std::cout << "\nTest 8 Results (Boundaries):" << std::endl; printResults ( target8_f1, target8_f2 );


    // =========================================================================
    // AUTOMATED ASSERTIONS BLOCK
    // =========================================================================
    std::cout << "\nExecuting Automated Assertions..." << std::endl;

    // Test 1
    for (size_t i = 0; i < target1_f2.size(); ++i) { assert(target1_f2[i]->id == expected_f2_ids_t1[i]); }
    for (size_t i = 0; i < target1_f1.size(); ++i) { assert((target1_f1[i]->next_instance != nullptr) == expected_has_next_t1[i]); }
    assert(target1_full.size() == 3);

    // Test 2
    for (size_t i = 0; i < target2_f2.size(); ++i) { assert(target2_f2[i]->id == expected_f2_ids_t2[i]); }
    for (size_t i = 0; i < target2_f1.size(); ++i) { assert((target2_f1[i]->next_instance != nullptr) == expected_has_next_t2[i]); }
    assert(target2_full.size() == 7);

    // Test 3
    for (size_t i = 0; i < target3_f2.size(); ++i) { assert(target3_f2[i]->id == expected_f2_ids_t3[i]); }
    for (size_t i = 0; i < target3_f1.size(); ++i) { assert((target3_f1[i]->next_instance != nullptr) == expected_has_next_t3[i]); }
    assert(target3_full.size() == 6);

    // Test 4
    for (size_t i = 0; i < target4_f2.size(); ++i) { assert(target4_f2[i]->id == expected_f2_ids_t4[i]); }
    for (size_t i = 0; i < target4_f1.size(); ++i) { assert((target4_f1[i]->next_instance != nullptr) == expected_has_next_t4[i]); }
    assert(target4_full.size() == 5);

    // Test 5
    for (size_t i = 0; i < target5_f1.size(); ++i) { assert((target5_f1[i]->next_instance != nullptr) == expected_has_next_t5[i]); }

    // Test 6
    for (size_t i = 0; i < target6_f1.size(); ++i) { assert((target6_f1[i]->next_instance != nullptr) == expected_has_next_t6[i]); }
    assert(target6_f2.empty());

    // Test 7
    for (size_t i = 0; i < target7_f1.size(); ++i) { assert((target7_f1[i]->next_instance != nullptr) == expected_has_next_t7[i]); }

    // Test 8
    for (size_t i = 0; i < target8_f1.size(); ++i) { assert((target8_f1[i]->next_instance != nullptr) == expected_has_next_t8[i]); }

    std::cout << "All tracking tests passed successfully!" << std::endl;

    // =========================================================================
    // MEMORY CLEANUP
    // =========================================================================
    cleanupTestMemory( target1_f1, target1_f2, selector1 );
    cleanupTestMemory( target2_f1, target2_f2, selector2 );
    cleanupTestMemory( target3_f1, target3_f2, selector3 );
    cleanupTestMemory( target4_f1, target4_f2, selector4 );
    cleanupTestMemory( target5_f1, target5_f2, selector5 );
    cleanupTestMemory( target6_f1, target6_f2, selector6 );
    cleanupTestMemory( target7_f1, target7_f2, selector7 );
    cleanupTestMemory( target8_f1, target8_f2, selector8 );

    return 0;
}

// Seeds tracker filter matrix coordinates
void initKF(Target* new_target) {
    int stateDim = 4;
    int measDim = 2;
    int ctrlDim = 0;
    
    auto KF = std::make_shared<cv::KalmanFilter>(stateDim, measDim, ctrlDim, CV_32F);
    new_target->kf = KF;

    KF->statePost.at<float>(0) = new_target->x; 
    KF->statePost.at<float>(1) = new_target->y; 
    KF->statePost.at<float>(2) = 0;            
    KF->statePost.at<float>(3) = 0;            
}

// Configures and populates frame target pointers passed by reference
Selector defineSelector(int threshold, 
                        std::vector<int>& id_f1, std::vector<int>& coords_f1, 
                        std::vector<int>& id_f2, std::vector<int>& coords_f2, 
                        std::vector<Target*>& target_f1, std::vector<Target*>& target_f2, std::vector<Target*>& target_full) {
    
    for (size_t i = 0; i < id_f1.size(); i++) {
        target_f1.push_back( new Target(coords_f1[2*i], coords_f1[2*i+1], 1) );
        target_f1[i]->id = id_f1[i];
        
        int x_index = 2*i;
        int y_index = 2*i + 1;
        target_f1[i]->x = coords_f1[x_index];
        target_f1[i]->y = coords_f1[y_index];
        
        initKF(target_f1[i]);
        target_full.push_back( target_f1[i] );
    }
    
    for (size_t i = 0; i < id_f2.size(); i++) {
        target_f2.push_back( new Target(coords_f2[2*i], coords_f2[2*i+1], 1) );
        target_f2[i]->id = id_f2[i];
        int x_index = 2*i;
        int y_index = 2*i + 1;
        target_f2[i]->x = coords_f2[x_index];
        target_f2[i]->y = coords_f2[y_index];
    }
    
    Selector selector(threshold);
    return selector;
}

void printResults (std::vector<Target*>& target_f1, std::vector<Target*>& target_f2) {
    for (size_t j = 0; j < target_f1.size(); j++) {
        std::cout << "id: " << target_f1[j]->id << " x,y: " << target_f1[j]->x << "," << target_f1[j]->y;
        if (target_f1[j]->proximity != nullptr) {
            target_f1[j]->proximity->sortByWeight();
            std::cout << " closest_weight: " << target_f1[j]->proximity->getVertexWeight(0);
            std::cout << " closest_id: " << target_f1[j]->proximity->getVertexID(0);
        }
        std::cout << std::endl;
    }
    for (size_t j = 0; j < target_f2.size(); j++) {
        std::cout << "id: " << target_f2[j]->id << " x,y: " << target_f2[j]->x << "," << target_f2[j]->y << std::endl;
    }
}

void cleanupTestMemory(std::vector<Target*>& target_f1, std::vector<Target*>& target_f2, Selector selector) {
    for (size_t i = 0; i < target_f1.size(); i++) {
        int index = target_f1.size() - i - 1;
        if (target_f1[index]->proximity != nullptr) {
            target_f1[index]->proximity->~Graph();
        }
        delete target_f1[index];
        target_f1.pop_back();
        
        if (selector.getNextTargetsPtr() != nullptr && !selector.getNextTargetsPtr()->empty()) {
            selector.getNextTargetsPtr()->pop_back();
        }
    }
    for (size_t i = 0; i < target_f2.size(); i++) {
        int index = target_f2.size() - i - 1;
        delete target_f2[index];
        target_f2.pop_back();
        
        if (selector.getPrevTargetsPtr() != nullptr && !selector.getPrevTargetsPtr()->empty()) {
            selector.getPrevTargetsPtr()->pop_back();
        }
    }
}
