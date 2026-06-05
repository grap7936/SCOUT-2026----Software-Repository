#include <iostream>
#include <vector>
#include <Graph.hpp>

/*  Compile command from INSTR/ directory
    g++ -I ./include testing/graphTest.cpp src/Graph.cpp -o graphTest.out
*/

int main() {

    std::vector<int> id1 = { 2, 3, 4, 5, 6, 7, 8, 9 };
    std::vector<float> weight1 = { 0.05,  0.12, 0.45, 0.23, 0.87, 0.65, 0.53, 0.44 };

    std::vector<int> id2 = { 45, 46, 54, 55, 56, 62 };
    std::vector<float> weight2 = {0.01, 0.30, 0.52, 0.69, 0.69, 0.92 };

    //test1
    Graph test1(1);
    for (int i = 0; i < id1.capacity(); i++) {
        test1.addVertex( id1[i], weight1[i] );
    }
    test1.sortByWeight();

    //test2
    Graph test2(2);
    for (int i = 0; i < id2.capacity(); i++) {
        test2.addVertex( id2[i], weight2[i] );
    }
    test2.sortByWeight();


    // print

    //test1
    std::cout << "Test 1 Results:" << std::endl;
    for (int j = 0; j < weight1.capacity(); j++) {
        std::cout << "id: " << test1.getVertexID(j) << " weight: " << test1.getVertexWeight(j) << std::endl;
    }

    //test2
    std::cout << "Test 2 Results:" << std::endl;
    for (int j = 0; j < weight2.capacity(); j++) {
        std::cout << "id: " << test2.getVertexID(j) << " weight: " << test2.getVertexWeight(j) << std::endl;
    }



    return 0;
}