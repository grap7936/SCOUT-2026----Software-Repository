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
    // TEST CONFIGURATION 2
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
    // EXECUTE OPERATIONS
    // =========================================================================
    
    // Initialize Graph 1, register vertex details, and perform insertion sort
    Graph test1(1);
    for (size_t i = 0; i < id1.size(); i++) {
        test1.addVertex( target1[i], weight1[i] );
    }
    test1.sortByWeight();

    // Initialize Graph 2, register vertex details, and perform insertion sort
    Graph test2(2);
    for (size_t i = 0; i < id2.size(); i++) {
        test2.addVertex( target2[i], weight2[i] );
    }
    test2.sortByWeight();


    // =========================================================================
    // PRINT OUTPUT LOGS
    // =========================================================================

    // Display parsed Graph 1 elements
    std::cout << "Test 1 Results (Sorted):" << std::endl;
    for (size_t j = 0; j < weight1.size(); j++) {
        std::cout << "id: " << test1.getVertexID(j) << " weight: " << test1.getVertexWeight(j) << std::endl;
    }

    // Display parsed Graph 2 elements
    std::cout << "\nTest 2 Results (Sorted):" << std::endl;
    for (size_t j = 0; j < weight2.size(); j++) {
        std::cout << "id: " << test2.getVertexID(j) << " weight: " << test2.getVertexWeight(j) << std::endl;
    }


    // =========================================================================
    // RUN AUTOMATED ASSERTIONS
    // =========================================================================
    std::cout << "\nExecuting Automated Assertions..." << std::endl;

    // Validate Graph 1 Sorted Constraints
    for (size_t j = 0; j < weight1.size(); j++) {
        assert(test1.getVertexID(j) == expected_ids_t1[j]);
        assert(test1.getVertexWeight(j) == expected_weights_t1[j]);
    }

    // Validate Graph 2 Sorted Constraints
    for (size_t j = 0; j < weight2.size(); j++) {
        assert(test2.getVertexID(j) == expected_ids_t2[j]);
        assert(test2.getVertexWeight(j) == expected_weights_t2[j]);
    }

    std::cout << "All Graph sorting tests passed successfully!" << std::endl;


    // =========================================================================
    // MEMORY CLEANUP
    // =========================================================================
    
    // Deallocate local Graph 1 target instance objects safely
    while ( target1.size() > 0 ){
        int index = target1.size() - 1;
        delete target1[index];
        target1.pop_back();
    }

    // Deallocate local Graph 2 target instance objects safely
    while ( target2.size() > 0 ){
        int index = target2.size() - 1;
        delete target2[index];
        target2.pop_back();
    }

    return 0;
}
