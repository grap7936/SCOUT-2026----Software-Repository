#include <iostream>
#include <vector>
#include <cassert> // Required for the assert() command
#include "Graph.hpp"
#include "Target.hpp"
#include "Selector.hpp"

/* Compile command from INSTR/ directory
    g++ -I ./include -I C:/msys64/ucrt64/include/opencv4 testing/selectorTest.cpp src/Graph.cpp src/Selector.cpp src/Target.cpp -o selectorTest.out -LC:/msys64/ucrt64/lib -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_video -lopencv_videoio -lopencv_imgcodecs -lopencv_objdetect -static
*/

// Function declarations (Updated to accept vectors by reference where needed)
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
    
    // Expected post-scan values
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
    
    // Expected post-scan values
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
    
    // Expected post-scan values
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
    
    // Expected post-scan values
    std::vector<int> expected_f2_ids_t4 = { 0, 2, 4 }; 
    std::vector<bool> expected_has_next_t4 = { true, false, true, false };


    // =========================================================================
    // RUN ALGORITHM SCANS
    // =========================================================================
    selector1.scan( &target1_f1, &target1_f2, &target1_full );
    selector2.scan( &target2_f1, &target2_f2, &target2_full );
    selector3.scan( &target3_f1, &target3_f2, &target3_full );
    selector4.scan( &target4_f1, &target4_f2, &target4_full );
    

    // =========================================================================
    // PRINT LOG OUTPUTS
    // =========================================================================
    std::cout << "Test 1 Results:" << std::endl;
    printResults ( target1_f1, target1_f2 );

    std::cout << "\nTest 2 Results:" << std::endl;
    printResults ( target2_f1, target2_f2 );
    
    std::cout << "\nTest 3 Results:" << std::endl;
    printResults ( target3_f1, target3_f2 );
    
    std::cout << "\nTest 4 Results:" << std::endl;
    printResults ( target4_f1, target4_f2 );


    // =========================================================================
    // AUTOMATED ASSERTIONS BLOCK
    // =========================================================================
    std::cout << "\nExecuting Automated Assertions..." << std::endl;

    // Verification Test 1
    for (size_t i = 0; i < target1_f2.size(); ++i) {
        assert(target1_f2[i]->id == expected_f2_ids_t1[i]);
    }
    for (size_t i = 0; i < target1_f1.size(); ++i) {
        assert((target1_f1[i]->next_instance != nullptr) == expected_has_next_t1[i]);
    }
    assert(target1_full.size() == 3);

    // Verification Test 2
    for (size_t i = 0; i < target2_f2.size(); ++i) {
        assert(target2_f2[i]->id == expected_f2_ids_t2[i]);
    }
    for (size_t i = 0; i < target2_f1.size(); ++i) {
        assert((target2_f1[i]->next_instance != nullptr) == expected_has_next_t2[i]);
    }
    assert(target2_full.size() == 7);

    // Verification Test 3
    for (size_t i = 0; i < target3_f2.size(); ++i) {
        assert(target3_f2[i]->id == expected_f2_ids_t3[i]);
    }
    for (size_t i = 0; i < target3_f1.size(); ++i) {
        assert((target3_f1[i]->next_instance != nullptr) == expected_has_next_t3[i]);
    }
    assert(target3_full.size() == 6);

    // Verification Test 4
    for (size_t i = 0; i < target4_f2.size(); ++i) {
        assert(target4_f2[i]->id == expected_f2_ids_t4[i]);
    }
    for (size_t i = 0; i < target4_f1.size(); ++i) {
        assert((target4_f1[i]->next_instance != nullptr) == expected_has_next_t4[i]);
    }
    assert(target4_full.size() == 5);

    std::cout << "All tests passed successfully!" << std::endl;


    // =========================================================================
    // MEMORY CLEANUP
    // =========================================================================
    cleanupTestMemory( target1_f1, target1_f2, selector1 );
    cleanupTestMemory( target2_f1, target2_f2, selector2 );
    cleanupTestMemory( target3_f1, target3_f2, selector3 );
    cleanupTestMemory( target4_f1, target4_f2, selector4 );

    return 0;
}

// Seeds the tracker filter matrix coordinates
void initKF(Target* new_target) {
    int stateDim = 4;
    int measDim = 2;
    int ctrlDim = 0;
    
    cv::KalmanFilter* KF = new cv::KalmanFilter(stateDim, measDim, ctrlDim, CV_32F);
    new_target->kf = KF;

    KF->statePost.at<float>(0) = new_target->x; // Initial X
    KF->statePost.at<float>(1) = new_target->y; // Initial Y
    KF->statePost.at<float>(2) = 0;             // Initial Vx
    KF->statePost.at<float>(3) = 0;             // Initial Vy
}

// Configures and populates frame target pointers passed by reference
Selector defineSelector(int threshold, 
                       std::vector<int>& id_f1, std::vector<int>& coords_f1, 
                       std::vector<int>& id_f2, std::vector<int>& coords_f2, 
                       std::vector<Target*>& target_f1, std::vector<Target*>& target_f2, std::vector<Target*>& target_full) {
    
    // Allocate and seed frame 1 entities
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
    
    // Allocate and seed frame 2 entities
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

// Dumps terminal results logs out for analysis
void printResults (std::vector<Target*>& target_f1, std::vector<Target*>& target_f2) {
    for (size_t j = 0; j < target_f1.size(); j++) {
        std::cout << "id: " << target_f1[j]->id;
        std::cout << " x,y: " << target_f1[j]->x << "," << target_f1[j]->y;
        
        // Ensure proximity graph memory exists before attempting sort
        if (target_f1[j]->proximity != nullptr) {
            target_f1[j]->proximity->sortByWeight();
            std::cout << " closest_weight: " << target_f1[j]->proximity->getVertexWeight(0);
            std::cout << " closest_id: " << target_f1[j]->proximity->getVertexID(0);
        }
        std::cout << std::endl;
    }
    for (size_t j = 0; j < target_f2.size(); j++) {
        std::cout << "id: " << target_f2[j]->id;
        std::cout << " x,y: " << target_f2[j]->x << "," << target_f2[j]->y << std::endl;
    }
}

// Clear dynamically allocated memory allocations systematically to prevent leaks
void cleanupTestMemory(std::vector<Target*>& target_f1, std::vector<Target*>& target_f2, Selector selector) {
    for (size_t i = 0; i < target_f1.size(); i++) {
        int index = target_f1.size() - i - 1;
        
        // Destructor invocation verification safely handled
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