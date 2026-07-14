#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <omp.h>
#include <VmbCPP/VmbCPP.h> // Vimba X include
#include "CameraWrapper.hpp"
#include "KeyInput.hpp"
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

    // Non-blocking terminal input: press 'q' (or ESC) to quit.
    KeyInput keys;
    std::cout << "IdleCam running. Press 'q' to quit." << std::endl;
    
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
        cv::waitKey(1); // let the GUI window repaint; exit is handled below

        // Exit on 'q' typed into the launching terminal
        if (keys.quitPressed()) {
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}