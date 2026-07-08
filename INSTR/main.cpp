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
#include "ArduinoSend.hpp"
#include <unistd.h>  // For sleep() and usleep()

using namespace VmbCPP;

void writeToPID(ArduinoSend& sender, int id, int x, int y, int nx, int ny);

const int FPS = 30; // defined by user to match camera

void setupArduino(ArduinoSend& sender);

int main() {

    // Set up arduino connection

    // Define the serial port node. On a Raspberry Pi, an Arduino UNO typically populates as "/dev/ttyACM0" or "/dev/ttyACM1".
    std::string serial_port = "/dev/ttyACM0"; 
    
    std::cout << "Initializing ArduinoSend on serial port: " << serial_port << std::endl; // output port connection information to the console
    ArduinoSend sender(serial_port); // create instance of the ArduinoSend function configured with the correct serial port
    
    setupArduino(sender);

    // Initialize camera
    CameraWrapper cam;

    // set parallelization thread count
    omp_set_num_threads(4);

    // Retrieve camera frame dimensions and frame rate
    int frame_width = cam.getWidth();
    int frame_height = cam.getHeight();

    // Define the codec using FourCC and initialize the VideoWriter
    // 'mp4v' or 'avc1' are highly reliable for MP4 containers
    int codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    cv::Size frame_size(frame_width, frame_height);
    std::string filename = "/home/scout/Desktop/INSTR_sh/debrisTestResults.mp4";

    cv::VideoWriter writer(filename, codec, FPS, frame_size, true);
    if (!writer.isOpened()) {
        std::cerr << "Error: Could not open the video writer." << std::endl;
        return -1;
    }

    Sentry sentry;
    int timeout = 2000; //ms

    int debris_id = -1;
    cv::Mat frame;
    while (true) {
        // Synchronously fetch exactly one frame from the camera stream (2000ms timeout)
        frame = cam.getFrame(timeout);

        if ( frame.empty() ) {
            // null return from getFrame() meaning no frame captured
            break;
        }

        debris_id = sentry.findDebris( frame, debris_id );

        if ( debris_id != -1 ){
            std::vector<int> debris_xy = sentry.getTargetCoords(debris_id);
            writeToPID(sender, debris_id, debris_xy[0], debris_xy[1], debris_xy[2], debris_xy[3]);
        }

        writer.write(frame);

        // (optional) show the frame
        cv::imshow("Frame", frame);
        if (cv::waitKey(1) == 27) { // exit if ESC pressed
            break;
        }


    }

    
    return 0;
}

void writeToPID(ArduinoSend& sender, int id, int x, int y, int nx, int ny) {

    // Transmit coordinates down to the UNO and automatically catch the echo
    bool success = false;

    if ( nx == -1 ) {
        success = sender.sendTargetCoordinates(id, x, y);
    } else {
        success = sender.sendTargetCoordinates(id, nx, ny);
    }
    
    if (!success) { // if target coordinates have not been sent (i.e bool success = false) then output an error message to the console
        std::cerr << "[ERROR] Pipeline broken during transmit phase." << std::endl;
    }

}

void setupArduino(ArduinoSend& sender) {

    // Establish port connection
    if (!sender.initializePort()) { // sends error message if the port cannot be effectively initialized
        std::cerr << "[FATAL] Could not initialize communication pipeline. Exiting." << std::endl;
        return;
    }

    // Manage Hardware Auto-Reset
    // Opening the port forces the Arduino UNO to reset. We must sleep here to give the bootloader time to finish before sending data frames.
    std::cout << "Serial pipeline established. Waiting 2 seconds for Arduino UNO boot sequence." << std::endl;
    sleep(2); 

    // flush system cache before running 1st instance
    sender.flushCache();

    std::cout << "[SYSTEM READY] Pipeline active. Ready for coordinate injection stream.\n" << std::endl;

}
