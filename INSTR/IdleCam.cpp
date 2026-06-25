#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <omp.h>
#include "Graph.hpp"
#include "Target.hpp"
#include "Detector.hpp"
#include "Selector.hpp"
#include "Sentry.hpp"

//void writeToPID(int id, int x, int y, int nx, int ny);

int main() {

    // Start video capture
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: could not open video capture\n";
        return -1;
    }

    // set parallelization thread count
    omp_set_num_threads(4);

    Sentry sentry;

    int debris_id = -1;
    cv::Mat frame;
    while (true) {
        if (!cap.read(frame)) { // reads next frame into 'frame'
            break; // exit on failure / end of stream
        }

        debris_id = sentry.findDebris( frame, debris_id );

        // (optional) show the frame
        cv::imshow("Frame", frame);
        if (cv::waitKey(1) == 27) { // exit if ESC pressed
            break;
        }


    }


    return 0;
}
