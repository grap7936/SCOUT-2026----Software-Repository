#include <iostream>
#include <vector>
#include "Graph.hpp"
#include "Target.hpp"

/*  Compile command from INSTR/ directory
    g++ -I ./include testing/graphTest.cpp src/Graph.cpp -o graphTest.out
*/


int main() {

    std::vector<int> id1 = { 2, 3, 4, 5, 6, 7, 8, 9 };
    std::vector<int> weight1 = { 44, 53, 65, 87, 23, 45, 12, 5 };
    std::vector<Target*> target1 = {};
    for ( int i = 0; i < id1.size(); i++) {
        target1.push_back( new Target(0, 0, 0) );
        target1.at(target1.size()-1)->setID(id1[i]);
    }

    std::vector<int> id2 = { 45, 46, 54, 55, 56, 62 };
    std::vector<int> weight2 = { 92, 69, 69, 52, 30, 1 };
    std::vector<Target*> target2 = {};
    for ( int i = 0; i < id2.size(); i++) {
        target2.push_back( new Target(0, 0, 0) );
        target2.at(target2.size()-1)->setID(id2[i]);
    }

    //test1
    Graph test1(1);
    for (int i = 0; i < id1.size(); i++) {
        test1.addVertex( target1[i], weight1[i] );
    }
    test1.sortByWeight();

    //test2
    Graph test2(2);
    for (int i = 0; i < id2.size(); i++) {
        test2.addVertex( target2[i], weight2[i] );
    }
    test2.sortByWeight();


    // print

    //test1
    std::cout << "Test 1 Results:" << std::endl;
    for (int j = 0; j < weight1.size(); j++) {
        std::cout << "id: " << test1.getVertexID(j) << " weight: " << test1.getVertexWeight(j) << std::endl;
    }

    //test2
    std::cout << "Test 2 Results:" << std::endl;
    for (int j = 0; j < weight2.size(); j++) {
        std::cout << "id: " << test2.getVertexID(j) << " weight: " << test2.getVertexWeight(j) << std::endl;
    }

    // memory cleanup
    // test1
    while ( target1.size() > 0 ){
        int index = target1.size() -1;
        delete target1[index];
        target1.pop_back();
    }

    // test2
    while ( target2.size() > 0 ){
        int index = target2.size() -1;
        delete target2[index];
        target2.pop_back();
    }

    return 0;
}