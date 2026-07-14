#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <omp.h>
#include "Graph.hpp"
#include "Target.hpp"
#include "Detector.hpp"
#include "Selector.hpp"
#include "Sentry.hpp"
#include "KeyInput.hpp"
#include "ArduinoSend.hpp"
#include <unistd.h>  // For sleep() and usleep()
#include <cstdlib> // For hasDisplay()

void writeToPID(ArduinoSend& sender, int id, int x, int y, int nx, int ny);

void setupArduino(ArduinoSend& sender);

bool hasDisplay() {
    const char* d = std::getenv("DISPLAY");
    return d != nullptr && d[0] != '\0';
}

static const bool GUI = hasDisplay();

int main() {

    // Set up data output files
    std::string VIDEO_FILENAME = "fullTestResults.mp4";
    std::string DEBRIS_LOG_FILENAME = "debrisLog.txt";
    std::string TARGET_LOG_FILENAME = "oldTargetsLog.txt";

    std::ofstream All_Target_Data;
    All_Target_Data.open(TARGET_LOG_FILENAME); // opens/creates necessary text file for inputting data into
    All_Target_Data << "id, x,y, kx,ky, vx,vy, score\n";
    All_Target_Data.close();
    
    std::ofstream Debris_Data;
    Debris_Data.open(DEBRIS_LOG_FILENAME); // opens/creates necessary text file for inputting data into
    Debris_Data << "motor_pos, id, x,y, kx,ky, vx,vy, score\n";
    Debris_Data.close();


    // Set up arduino connection
    // Define the serial port node. On a Raspberry Pi, an Arduino UNO typically populates as "/dev/ttyACM0" or "/dev/ttyACM1".
    std::string serial_port = "/dev/ttyACM0"; 
    
    std::cout << "Initializing ArduinoSend on serial port: " << serial_port << std::endl; // output port connection information to the console
    ArduinoSend sender(serial_port); // create instance of the ArduinoSend function configured with the correct serial port
    
    setupArduino(sender);

    // Start video capture
    cv::VideoCapture cap("/home/scout/Desktop/INSTR/testing/fullTestVideo.mp4");
    if (!cap.isOpened()) {
        std::cerr << "Error: could not open video capture\n";
        return -1;
    }

    // set parallelization thread count
    omp_set_num_threads(4);

    // Retrieve camera frame dimensions and frame rate
    int frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    double fps = cap.get(cv::CAP_PROP_FPS);
    
    // Fallback if the camera driver returns 0 for FPS
    if (fps <= 0) { 
        fps = 30.0; 
    }

    // Define the codec using FourCC and initialize the VideoWriter
    // 'mp4v' or 'avc1' are highly reliable for MP4 containers
    int codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    cv::Size frame_size(frame_width, frame_height);

    cv::VideoWriter writer(VIDEO_FILENAME, codec, fps, frame_size, true);
    if (!writer.isOpened()) {
        std::cerr << "Error: Could not open the video writer." << std::endl;
        return -1;
    }

    Sentry sentry;

    // Non-blocking terminal input: press 'q' (or ESC) to quit.
    KeyInput keys;
    std::cout << "fullTest running. Press 'q' to quit." << std::endl;

    int debris_id = -1;
    cv::Mat frame;
    while (true) {
        // read frame
        if (!cap.read(frame)) {
            break; // exit on failure / end of stream
        }

        // read motor position
        std::string m_pos = "";//replace with sender.readSerial();

        debris_id = sentry.findDebris( frame, debris_id );

        if ( debris_id != -1 ){
            // write to file
            Debris_Data.open(DEBRIS_LOG_FILENAME, std::ios::app);
            Target* current = (*sentry.getFullListPtr())[debris_id];
            Debris_Data << m_pos << ", " << debris_id
                    << ", " << current->getX() << "," << current->getY()
                    << ", " << current->getKx() << "," << current->getKy()
                    << ", " << current->getVx() << "," << current->getVy()
                    << ", " << current->getDebrisLikelihood() << "\n";

            // write to Arduino
            std::vector<int> debris_xy = sentry.getTargetCoords(debris_id);
            writeToPID(sender, debris_id, debris_xy[0], debris_xy[1], debris_xy[2], debris_xy[3]);
        }

        writer.write(frame);

        // (optional) show the frame
        if (GUI) {
            cv::imshow("Frame", frame);
            cv::waitKey(1); // repaint the window; only meaningful with a display
        }

        // Exit on 'q' typed into the launching terminal (works headless too)
        if (keys.quitPressed()) {
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