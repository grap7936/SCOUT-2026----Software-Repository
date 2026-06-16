#include <iostream>
#include <vector>
#include <cassert> // Required for the assert() automated checking command
#include "Graph.hpp"
#include "Target.hpp"

/* Compile command from INSTR/ directory
    g++ -I ./include testing/graphTest.cpp src/Graph.cpp src/Target.cpp -o graphTest.out
*/

int main() {

    // =========================================================================
    // TEST CONFIGURATION 1
    // =========================================================================
    std::vector<int> id1 = { 2, 3, 4, 5, 6, 7, 8, 9 };
    std::vector<int> weight1 = { 44, 53, 65, 87, 23, 45, 12, 5 };
    std::vector<Target*> target1 = {};
    
    // Allocate targets and assign their IDs sequentially
    for ( size_t i = 0; i < id1.size(); i++) {
        target1.push_back( new Target(0, 0, 0) );
        target1.at(target1.size()-1)->setID(id1[i]);
    }

    // Expected ascending weights: 5, 12, 23, 44, 45, 53, 65, 87
    // Mapping their corresponding IDs yields the array below
    std::vector<int> expected_ids_t1 = { 9, 8, 6, 2, 7, 3, 4, 5 };
    std::vector<int> expected_weights_t1 = { 5, 12, 23, 44, 45, 53, 65, 87 };


    // =========================================================================
    // TEST CONFIGURATION 2 (Duplicate Weights - Testing Stability)
    // =========================================================================
    std::vector<int> id2 = { 45, 46, 54, 55, 56, 62 };
    std::vector<int> weight2 = { 92, 69, 69, 52, 30, 1 };
    std::vector<Target*> target2 = {};
    
    // Allocate targets and assign their IDs sequentially
    for ( size_t i = 0; i < id2.size(); i++) {
        target2.push_back( new Target(0, 0, 0) );
        target2.at(target2.size()-1)->setID(id2[i]);
    }

    // Expected ascending weights: 1, 30, 52, 69, 69, 92
    // Mapping their corresponding IDs yields the array below
    std::vector<int> expected_ids_t2 = { 62, 56, 55, 46, 54, 45 }; // Assuming stable sort preserves 46 before 54
    std::vector<int> expected_weights_t2 = { 1, 30, 52, 69, 69, 92 };


    // =========================================================================
    // TEST CONFIGURATION 3 (Edge Case: Single Element)
    // =========================================================================
    std::vector<int> id3 = { 99 };
    std::vector<int> weight3 = { 42 };
    std::vector<Target*> target3 = {};
    
    for ( size_t i = 0; i < id3.size(); i++) {
        target3.push_back( new Target(0, 0, 0) );
        target3.at(target3.size()-1)->setID(id3[i]);
    }
    std::vector<int> expected_ids_t3 = { 99 };
    std::vector<int> expected_weights_t3 = { 42 };


    // =========================================================================
    // TEST CONFIGURATION 4 (Edge Case: Already Sorted Array)
    // =========================================================================
    std::vector<int> id4 = { 10, 11, 12, 13 };
    std::vector<int> weight4 = { 100, 200, 300, 400 };
    std::vector<Target*> target4 = {};
    
    for ( size_t i = 0; i < id4.size(); i++) {
        target4.push_back( new Target(0, 0, 0) );
        target4.at(target4.size()-1)->setID(id4[i]);
    }
    std::vector<int> expected_ids_t4 = { 10, 11, 12, 13 };
    std::vector<int> expected_weights_t4 = { 100, 200, 300, 400 };


    // =========================================================================
    // TEST CONFIGURATION 5 (Edge Case: Completely Reverse Sorted)
    // =========================================================================
    std::vector<int> id5 = { 20, 21, 22, 23 };
    std::vector<int> weight5 = { 50, 40, 30, 20 };
    std::vector<Target*> target5 = {};
    
    for ( size_t i = 0; i < id5.size(); i++) {
        target5.push_back( new Target(0, 0, 0) );
        target5.at(target5.size()-1)->setID(id5[i]);
    }
    std::vector<int> expected_ids_t5 = { 23, 22, 21, 20 };
    std::vector<int> expected_weights_t5 = { 20, 30, 40, 50 };


    // =========================================================================
    // TEST CONFIGURATION 6 (Edge Case: All Weights Identical)
    // =========================================================================
    std::vector<int> id6 = { 70, 71, 72, 73 };
    std::vector<int> weight6 = { 15, 15, 15, 15 };
    std::vector<Target*> target6 = {};
    
    for ( size_t i = 0; i < id6.size(); i++) {
        target6.push_back( new Target(0, 0, 0) );
        target6.at(target6.size()-1)->setID(id6[i]);
    }
    // Assuming stable sort (like insertion sort), the exact initial ordering of IDs should be preserved.
    std::vector<int> expected_ids_t6 = { 70, 71, 72, 73 };
    std::vector<int> expected_weights_t6 = { 15, 15, 15, 15 };


    // =========================================================================
    // EXECUTE OPERATIONS
    // =========================================================================
    
    Graph test1(1);
    for (size_t i = 0; i < id1.size(); i++) { test1.addVertex( target1[i], weight1[i] ); }
    test1.sortByWeight();

    Graph test2(2);
    for (size_t i = 0; i < id2.size(); i++) { test2.addVertex( target2[i], weight2[i] ); }
    test2.sortByWeight();

    Graph test3(3);
    for (size_t i = 0; i < id3.size(); i++) { test3.addVertex( target3[i], weight3[i] ); }
    test3.sortByWeight();

    Graph test4(4);
    for (size_t i = 0; i < id4.size(); i++) { test4.addVertex( target4[i], weight4[i] ); }
    test4.sortByWeight();

    Graph test5(5);
    for (size_t i = 0; i < id5.size(); i++) { test5.addVertex( target5[i], weight5[i] ); }
    test5.sortByWeight();

    Graph test6(6);
    for (size_t i = 0; i < id6.size(); i++) { test6.addVertex( target6[i], weight6[i] ); }
    test6.sortByWeight();


    // =========================================================================
    // PRINT OUTPUT LOGS
    // =========================================================================

    std::cout << "Test 1 Results (Sorted):" << std::endl;
    for (size_t j = 0; j < weight1.size(); j++) {
        std::cout << "id: " << test1.getVertexID(j) << " weight: " << test1.getVertexWeight(j) << std::endl;
    }

    std::cout << "\nTest 2 Results (Sorted):" << std::endl;
    for (size_t j = 0; j < weight2.size(); j++) {
        std::cout << "id: " << test2.getVertexID(j) << " weight: " << test2.getVertexWeight(j) << std::endl;
    }

    std::cout << "\nTest 3 Results (Single Element):" << std::endl;
    for (size_t j = 0; j < weight3.size(); j++) {
        std::cout << "id: " << test3.getVertexID(j) << " weight: " << test3.getVertexWeight(j) << std::endl;
    }

    std::cout << "\nTest 4 Results (Already Sorted):" << std::endl;
    for (size_t j = 0; j < weight4.size(); j++) {
        std::cout << "id: " << test4.getVertexID(j) << " weight: " << test4.getVertexWeight(j) << std::endl;
    }

    std::cout << "\nTest 5 Results (Reverse Sorted):" << std::endl;
    for (size_t j = 0; j < weight5.size(); j++) {
        std::cout << "id: " << test5.getVertexID(j) << " weight: " << test5.getVertexWeight(j) << std::endl;
    }

    std::cout << "\nTest 6 Results (Identical Weights):" << std::endl;
    for (size_t j = 0; j < weight6.size(); j++) {
        std::cout << "id: " << test6.getVertexID(j) << " weight: " << test6.getVertexWeight(j) << std::endl;
    }


    // =========================================================================
    // RUN AUTOMATED ASSERTIONS
    // =========================================================================
    std::cout << "\nExecuting Automated Assertions..." << std::endl;

    for (size_t j = 0; j < weight1.size(); j++) {
        assert(test1.getVertexID(j) == expected_ids_t1[j]);
        assert(test1.getVertexWeight(j) == expected_weights_t1[j]);
    }

    for (size_t j = 0; j < weight2.size(); j++) {
        assert(test2.getVertexID(j) == expected_ids_t2[j]);
        assert(test2.getVertexWeight(j) == expected_weights_t2[j]);
    }

    for (size_t j = 0; j < weight3.size(); j++) {
        assert(test3.getVertexID(j) == expected_ids_t3[j]);
        assert(test3.getVertexWeight(j) == expected_weights_t3[j]);
    }

    for (size_t j = 0; j < weight4.size(); j++) {
        assert(test4.getVertexID(j) == expected_ids_t4[j]);
        assert(test4.getVertexWeight(j) == expected_weights_t4[j]);
    }

    for (size_t j = 0; j < weight5.size(); j++) {
        assert(test5.getVertexID(j) == expected_ids_t5[j]);
        assert(test5.getVertexWeight(j) == expected_weights_t5[j]);
    }

    for (size_t j = 0; j < weight6.size(); j++) {
        assert(test6.getVertexID(j) == expected_ids_t6[j]);
        assert(test6.getVertexWeight(j) == expected_weights_t6[j]);
    }

    std::cout << "All Graph sorting tests passed successfully!" << std::endl;


    // =========================================================================
    // MEMORY CLEANUP
    // =========================================================================
    
    while ( target1.size() > 0 ){ int index = target1.size() - 1; delete target1[index]; target1.pop_back(); }
    while ( target2.size() > 0 ){ int index = target2.size() - 1; delete target2[index]; target2.pop_back(); }
    while ( target3.size() > 0 ){ int index = target3.size() - 1; delete target3[index]; target3.pop_back(); }
    while ( target4.size() > 0 ){ int index = target4.size() - 1; delete target4[index]; target4.pop_back(); }
    while ( target5.size() > 0 ){ int index = target5.size() - 1; delete target5[index]; target5.pop_back(); }
    while ( target6.size() > 0 ){ int index = target6.size() - 1; delete target6[index]; target6.pop_back(); }

    return 0;
}