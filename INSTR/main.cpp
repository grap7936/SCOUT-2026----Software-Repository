#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "Graph.hpp"
#include "Target.hpp"
#include "Detector.hpp"
#include "Selector.hpp"
#include "Sentry.hpp"

void writeToPID();

int main() {

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: could not open video capture\n";
        return -1;
    }

    int selector_closeness_threshold = 3000;

    Sentry sentry(selector_closeness_threshold);

    cv::Mat frame;
    while (true) {
        if (!cap.read(frame)) { // reads next frame into 'frame'
            break; // exit on failure / end of stream
        }

        int debris_id = -1;
        debris_id = sentry.findDebris( frame, debris_id );

        if ( debris_id != -1 ){
            std::vector<int> debris_xy = sentry.getTargetCoords(debris_id);
            writeToPID(debris_xy[0], debris_xy[1]);
        }

        // (optional) show the frame
        cv::imshow("Frame", frame);
        if (cv::waitKey(1) == 27) { // exit if ESC pressed
            break;
        }


    }





    return 0;
}

void writeToPID(int x, int y) {
    std::cout << "sending coords to arduino B) -- ";
    std::cout << "x: " << x << " y: " << y << std::endl; 
}