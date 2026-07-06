#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <VmbCPP/VmbCPP.h> // Vimba X include
#include "CameraWrapper.hpp"

using namespace VmbCPP;

int main() {

    CameraWrapper cam;

    cv::Mat frame;

    int timeout = 2000; // ms
    
    // fetch frame from the camera stream to check (2000ms timeout)
    frame = cam.getFrame(timeout);

    if ( frame.empty() ) {
        // null return from getFrame() meaning no frame captured
        std::cerr << "No frame received from Camera stream. Exiting..." << std::endl;
        return -1;
    } else {
        std::string filepath = "/home/scout/Desktop/INSTR_sh/testCamImage.png";
        std::cout << "Saving image to " << filepath << std::endl;
        cv::imwrite(filepath, frame);
        // (optional) show the frame
        cv::imshow("Frame", frame);
        while (true) {
            if (cv::waitKey(1) == 27) { // exit if ESC pressed
                break;
            }
        }
        cv::destroyAllWindows();
    }

    return 0;
}
