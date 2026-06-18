#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "Graph.hpp"
#include "Target.hpp"
#include "Detector.hpp"
#include "Selector.hpp"
#include "Sentry.hpp"

void writeToPID(int, int, int);

int main() {

    cv::VideoCapture cap("testing/testVideo3.mp4");
    if (!cap.isOpened()) {
        std::cerr << "Error: could not open video capture\n";
        return -1;
    }

    int selector_closeness_threshold = 500;

    Sentry sentry(selector_closeness_threshold);

    int debris_id = -1;
    cv::Mat frame;
    while (true) {
        if (!cap.read(frame)) { // reads next frame into 'frame'
            break; // exit on failure / end of stream
        }

        debris_id = sentry.findDebris( frame, debris_id );

        if ( debris_id != -1 ){
            std::vector<int> debris_xy = sentry.getTargetCoords(debris_id);
            writeToPID(debris_id, debris_xy[0], debris_xy[1]);
        }

        // (optional) show the frame
        cv::imshow("Frame", frame);
        if (cv::waitKey(1) == 27) { // exit if ESC pressed
            break;
        }


    }





    return 0;
}

void writeToPID(int id, int x, int y) {
    std::cout << "sending coords to arduino (" << id << ") -- ";
    std::cout << "x: " << x << " y: " << y << std::endl; 
}
