#include <iostream>
#include <vector>
#include "Graph.hpp"
#include "Target.hpp"
#include "Selector.hpp"

/*  Compile command from INSTR/ directory
    g++ -I ./include -I C:/msys64/ucrt64/include/opencv4 testing/selectorTest.cpp src/Graph.cpp src/Selector.cpp src/Target.cpp -o selectorTest.out -LC:/msys64/ucrt64/lib -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_video -lopencv_videoio -lopencv_imgcodecs -lopencv_objdetect -static

    */

void initKF(Target* new_target) {
    // Initialize Kalman Filter
    // State: [x, y, vx, vy]^T -> 4 dimensions
    // Measurement: [x, y] -> 2 dimensions
    int stateDim = 4;
    int measDim = 2;
    int ctrlDim = 0;
    
    cv::KalmanFilter* KF = new cv::KalmanFilter(stateDim, measDim, ctrlDim, CV_32F);
    new_target->kf = KF;
    // Initial state setup
    KF->statePost.at<float>(0) = new_target->x; // Initial X
    KF->statePost.at<float>(1) = new_target->y; // Initial Y
    KF->statePost.at<float>(2) = 0;   // Initial Vx
    KF->statePost.at<float>(3) = 0;   // Initial Vy
}

int main() {

    float threshold = 3500;

    // definition 1
    std::vector<int> id1_f1 = { 0, 1, 2 };
    std::vector<int> coords1_f1 = { 120,200, 340,10, 420,370 };
    std::vector<int> id1_f2 = { 3, 4, 5 };
    std::vector<int> coords1_f2 = { 360,100, 430,460, 150,220 };
    std::vector<Target*> target1_f1 = {};
    std::vector<Target*> target1_full = {};
    for ( int i = 0; i < id1_f1.size(); i++) {
        target1_f1.push_back( new Target(coords1_f1[2*i], coords1_f1[2*i+1], 1) );
        target1_f1[i]->id = id1_f1[i];
        initKF(target1_f1[i]);
        int x_index = 2*i;
        int y_index = 2*i + 1;
        target1_f1[i]->x = coords1_f1[x_index];
        target1_f1[i]->y = coords1_f1[y_index];
    }
    std::vector<Target*> target1_f2 = {};
    for ( int i = 0; i < id1_f2.size(); i++) {
        target1_f2.push_back( new Target(coords1_f2[2*i], coords1_f2[2*i+1], 1) );
        target1_full.push_back( target1_f1[i] );
        target1_f2[i]->id = id1_f2[i];
        int x_index = 2*i;
        int y_index = 2*i + 1;
        target1_f2[i]->x = coords1_f2[x_index];
        target1_f2[i]->y = coords1_f2[y_index];
    }
    Selector selector1(threshold);

    // definition 2
    std::vector<int> id2_f1 = { 0, 1, 2, 3 };
    std::vector<int> coords2_f1 = { 150,210, 330,30, 520,20, 210,440 };
    std::vector<int> id2_f2 = { 4, 5, 6, 7, 8, 9 };
    std::vector<int> coords2_f2 = { 160,300, 530,120, 400,80, 240,10, 10,350, 400,320 };
    std::vector<Target*> target2_f1 = {};
    std::vector<Target*> target2_full = {};
    for ( int i = 0; i < id2_f1.size(); i++) {
        target2_f1.push_back( new Target(coords2_f1[2*i], coords2_f1[2*i+1], 1) );
        target2_full.push_back( target2_f1[i] );
        target2_f1[i]->id = id2_f1[i];
        initKF(target2_f1[i]);
        int x_index = 2*i;
        int y_index = 2*i + 1;
        target2_f1[i]->x = coords2_f1[x_index];
        target2_f1[i]->y = coords2_f1[y_index];
    }
    std::vector<Target*> target2_f2 = {};
    for ( int i = 0; i < id2_f2.size(); i++) {
        target2_f2.push_back( new Target(coords2_f2[2*i], coords2_f2[2*i+1], 1) );
        target2_f2[i]->id = id2_f2[i];
        int x_index = 2*i;
        int y_index = 2*i + 1;
        target2_f2[i]->x = coords2_f2[x_index];
        target2_f2[i]->y = coords2_f2[y_index];
    }
    Selector selector2(threshold);

    // definition 3
    std::vector<int> id3_f1 = { 0, 1, 2, 3 };
    std::vector<int> coords3_f1 = { 110,100, 150,100, 240,350, 350,400 };
    std::vector<int> id3_f2 = { 4, 5, 6, 7, 8 };
    std::vector<int> coords3_f2 = { 130,190, 160,200, 10,300, 250,450, 500,20 };
    std::vector<Target*> target3_f1 = {};
    std::vector<Target*> target3_full = {};
    for ( int i = 0; i < id3_f1.size(); i++) {
        target3_f1.push_back( new Target(coords3_f1[2*i], coords3_f1[2*i+1], 1) );
        target3_full.push_back( target3_f1[i] );
        target3_f1[i]->id = id3_f1[i];
        initKF(target3_f1[i]);
        int x_index = 2*i;
        int y_index = 2*i + 1;
        target3_f1[i]->x = coords3_f1[x_index];
        target3_f1[i]->y = coords3_f1[y_index];
    }
    std::vector<Target*> target3_f2 = {};
    for ( int i = 0; i < id3_f2.size(); i++) {
        target3_f2.push_back( new Target(coords3_f2[2*i], coords3_f2[2*i+1], 1) );
        target3_f2[i]->id = id3_f2[i];
        int x_index = 2*i;
        int y_index = 2*i + 1;
        target3_f2[i]->x = coords3_f2[x_index];
        target3_f2[i]->y = coords3_f2[y_index];
    }
    Selector selector3(threshold);

    // definition 4
    std::vector<int> id4_f1 = { 0, 1, 2, 3 };
    std::vector<int> coords4_f1 = { 110,100, 150,100, 240,350, 10,10};
    std::vector<int> id4_f2 = { 4, 5, 6 };
    std::vector<int> coords4_f2 = { 130,190, 250,450, 500,20 };
    std::vector<Target*> target4_f1 = {};
    std::vector<Target*> target4_full = {};
    for ( int i = 0; i < id4_f1.size(); i++) {
        target4_f1.push_back( new Target(coords4_f1[2*i], coords4_f1[2*i+1], 1) );
        target4_full.push_back( target4_f1[i] );
        target4_f1[i]->id = id4_f1[i];
        initKF(target4_f1[i]);
        int x_index = 2*i;
        int y_index = 2*i + 1;
        target4_f1[i]->x = coords4_f1[x_index];
        target4_f1[i]->y = coords4_f1[y_index];
    }
    std::vector<Target*> target4_f2 = {};
    for ( int i = 0; i < id4_f2.size(); i++) {
        target4_f2.push_back( new Target(coords4_f2[2*i], coords4_f2[2*i+1], 1) );
        target4_f2[i]->id = id4_f2[i];
        int x_index = 2*i;
        int y_index = 2*i + 1;
        target4_f2[i]->x = coords4_f2[x_index];
        target4_f2[i]->y = coords4_f2[y_index];
    }
    Selector selector4(threshold);
    

    //test1
    selector1.scan( &target1_f1, &target1_f2, &target1_full );

    //test2
    selector2.scan( &target2_f1, &target2_f2, &target2_full);

    //test3
    selector3.scan( &target3_f1, &target3_f2, &target3_full );

    //test4
    selector4.scan( &target4_f1, &target4_f2, &target4_full );
    
    // print

    //test1
    std::cout << "Test 1 Results:" << std::endl;
    for (int j = 0; j < target1_f1.size(); j++) {
        std::cout << "id: " << target1_f1[j]->id;
        std::cout << " x,y: " << target1_f1[j]->x << "," << target1_f1[j]->y;
        target1_f1[j]->proximity->sortByWeight();
        std::cout << " closest_weight: " << target1_f1[j]->proximity->getVertexWeight(0) << std::endl;
    }
    for (int j = 0; j < target1_f2.size(); j++) {
        std::cout << "id: " << target1_f2[j]->id;
        std::cout << " x,y: " << target1_f2[j]->x << "," << target1_f2[j]->y << std::endl;
    }

    //test2
    std::cout << "Test 2 Results:" << std::endl;
    for (int j = 0; j < target2_f1.size(); j++) {
        std::cout << "id: " << target2_f1[j]->id;
        std::cout << " x,y: " << target2_f1[j]->x << "," << target2_f1[j]->y;
        target2_f1[j]->proximity->sortByWeight();
        std::cout << " closest_weight: " << target2_f1[j]->proximity->getVertexWeight(0) << std::endl;
    }
    for (int j = 0; j < target2_f2.size(); j++) {
        std::cout << "id: " << target2_f2[j]->id;
        std::cout << " x,y: " << target2_f2[j]->x << "," << target2_f2[j]->y << std::endl;
    }

    //test3
    std::cout << "Test 3 Results:" << std::endl;
    for (int j = 0; j < target3_f1.size(); j++) {
        std::cout << "id: " << target3_f1[j]->id;
        std::cout << " x,y: " << target3_f1[j]->x << "," << target3_f1[j]->y;
        target3_f1[j]->proximity->sortByWeight();
        std::cout << " closest_weight: " << target3_f1[j]->proximity->getVertexWeight(0) << std::endl;
    }
    for (int j = 0; j < target3_f2.size(); j++) {
        std::cout << "id: " << target3_f2[j]->id;
        std::cout << " x,y: " << target3_f2[j]->x << "," << target3_f2[j]->y << std::endl;
    }
    
    //test4
    std::cout << "Test 4 Results:" << std::endl;
    for (int j = 0; j < target4_f1.size(); j++) {
        std::cout << "id: " << target4_f1[j]->id;
        std::cout << " x,y: " << target4_f1[j]->x << "," << target4_f1[j]->y;
        target4_f1[j]->proximity->sortByWeight();
        std::cout << " closest_weight: " << target4_f1[j]->proximity->getVertexWeight(0) << std::endl;
    }
    for (int j = 0; j < target4_f2.size(); j++) {
        std::cout << "id: " << target4_f2[j]->id;
        std::cout << " x,y: " << target4_f2[j]->x << "," << target4_f2[j]->y << std::endl;
    }

    // memory cleanup

    //test1
    for (int i = 0; i < target1_f1.size(); i++) {
        int index = target1_f1.size() - i - 1;
        target1_f1[index]->proximity->~Graph();
        delete target1_f1[index];
        target1_f1.pop_back();
        selector1.getNextTargetsPtr()->pop_back();
    }
    for (int i = 0; i < target1_f2.size(); i++) {
        int index = target1_f2.size() - i - 1;
        delete target1_f2[index];
        target1_f2.pop_back();
        selector1.getPrevTargetsPtr()->pop_back();
    }

    //test2
    for (int i = 0; i < target2_f1.size(); i++) {
        int index = target2_f1.size() - i - 1;
        target2_f1[index]->proximity->~Graph();
        delete target2_f1[index];
        target2_f1.pop_back();
        selector2.getNextTargetsPtr()->pop_back();
    }
    for (int i = 0; i < target2_f2.size(); i++) {
        int index = target2_f2.size() - i - 1;
        delete target2_f2[index];
        target2_f2.pop_back();
        selector2.getPrevTargetsPtr()->pop_back();
    }

    //test3
    for (int i = 0; i < target3_f1.size(); i++) {
        int index = target3_f1.size() - i - 1;
        target3_f1[index]->proximity->~Graph();
        delete target3_f1[index];
        target3_f1.pop_back();
        selector3.getNextTargetsPtr()->pop_back();
    }
    for (int i = 0; i < target3_f2.size(); i++) {
        int index = target3_f2.size() - i - 1;
        delete target3_f2[index];
        target3_f2.pop_back();
        selector3.getPrevTargetsPtr()->pop_back();
    }

    //test4
    for (int i = 0; i < target4_f1.size(); i++) {
        int index = target4_f1.size() - i - 1;
        target4_f1[index]->proximity->~Graph();
        delete target4_f1[index];
        target4_f1.pop_back();
        selector4.getNextTargetsPtr()->pop_back();
    }
    for (int i = 0; i < target4_f2.size(); i++) {
        int index = target4_f2.size() - i - 1;
        delete target4_f2[index];
        target4_f2.pop_back();
        selector4.getPrevTargetsPtr()->pop_back();
    }


    return 0;
}