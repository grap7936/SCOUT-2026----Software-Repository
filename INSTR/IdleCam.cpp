#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <omp.h>
#include <VmbCPP/VmbCPP.h> // Vimba X include
#include "CameraWrapper.hpp"
#include "Graph.hpp"
#include "Target.hpp"
#include "Detector.hpp"
#include "Selector.hpp"
#include "Sentry.hpp"

using namespace VmbCPP;

int main() {

    CameraWrapper cam;

    // Set parallelization thread count
    omp_set_num_threads(4);

    Sentry sentry;

    int debris_id = -1;
    cv::Mat frame;

    int timeout = 2000; // ms
    
    // fetch frame from the camera stream to check (2000ms timeout)
    frame = cam.getFrame(timeout);

    if ( frame.empty() ) {
        // null return from getFrame() meaning no frame captured
        std::cerr << "No frame received from Camera stream. Exiting..." << std::endl;
        return -1;
    }

    while (true) {
        // Synchronously fetch exactly one frame from the camera stream (2000ms timeout)
        frame = cam.getFrame(timeout);

        if ( frame.empty() ) {
            // null return from getFrame() meaning no frame captured
            break;
        }

        // Process frame through the debris tracking pipeline
        debris_id = sentry.findDebris(frame, debris_id);

        // (optional) show the frame
        cv::imshow("Frame", frame);
        if (cv::waitKey(1) == 27) { // exit if ESC pressed
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
