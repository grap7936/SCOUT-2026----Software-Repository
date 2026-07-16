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

#include "Timer.hpp"

extern TimerStats t_filter, t_contours, t_centroids;
extern TimerStats t_estimate, t_weights, t_connect;

TimerStats t_frame, t_serial, t_detect, t_write;

void writeToPID(ArduinoSend& sender, int id, int x, int y, int nx, int ny);

void setupArduino(ArduinoSend& sender);

bool hasDisplay() {
    const char* d = std::getenv("DISPLAY");
    return d != nullptr && d[0] != '\0';
}

static const bool GUI = hasDisplay();

int main() {

    // Set up data output files
    std::string VIDEO_FILENAME = "/home/scout/Desktop/INSTR/debrisTestResults.mp4";
    std::string MOTOR_LOG_FILENAME = "/home/scout/Desktop/INSTR/motorLog.txt";
    std::string DEBRIS_LOG_FILENAME = "/home/scout/Desktop/INSTR/debrisLog.txt";
    std::string TARGET_LOG_FILENAME = "/home/scout/Desktop/INSTR/oldTargetsLog.txt";

    std::ofstream All_Target_Data;
    All_Target_Data.open(TARGET_LOG_FILENAME); // opens/creates necessary text file for inputting data into
    All_Target_Data << "id, x,y, kx,ky, vx,vy, score\n";
    All_Target_Data.close();
    
    
    std::ofstream Debris_Data;
    Debris_Data.open(DEBRIS_LOG_FILENAME); // opens/creates necessary text file for inputting data into
    Debris_Data << "frame_num, id, x,y, kx,ky, vx,vy, score\n";
    Debris_Data.close();

    std::ofstream Motor_Data;
    Motor_Data.open(DEBRIS_LOG_FILENAME); // opens/creates necessary text file for inputting data into
    Motor_Data << "frame_num, motor_pos\n";
    Motor_Data.close();


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
    cv::setNumThreads(1);

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
    
    // open file write streams
    Motor_Data.open(MOTOR_LOG_FILENAME, std::ios::app);
    Debris_Data.open(DEBRIS_LOG_FILENAME, std::ios::app);
    All_Target_Data.open(TARGET_LOG_FILENAME, std::ios::app);

    int debris_id = -1;
    cv::Mat frame;
    long long fid = 0;
    while (true) {

        Timer _frame(t_frame);

        // read frame
        if (!cap.read(frame)) {
            break; // exit on failure / end of stream
        }

        // read motor position
        std::vector<double> raw;
        { Timer _(t_serial); raw = sender.readMotorPosition(Motor_Data); }
        double m_pos = raw[1];
        double ard_frame_num = raw[0];

        { Timer _(t_detect); debris_id = sentry.findDebris(frame, debris_id, fid); }

        if ( debris_id != -1 ){
            // write to file
            Target* current = (*sentry.getFullListPtr())[debris_id];
            Debris_Data << m_pos << ", " << debris_id
                    << ", " << current->getX() << "," << current->getY()
                    << ", " << current->getKx() << "," << current->getKy()
                    << ", " << current->getVx() << "," << current->getVy()
                    << ", " << current->getDebrisLikelihood() << "\n" << std::flush;

            // write to Arduino
            std::vector<int> debris_xy = sentry.getTargetCoords(debris_id);
            writeToPID(sender, debris_id, debris_xy[0], debris_xy[1], debris_xy[2], debris_xy[3]);
        } else {
            writeToPID(sender, -1, -1, -1, -1, -1);
        }

        { Timer _(t_write); writer.write(frame); }

        fid++;

        // inside the while loop, after fid++:
        if (fid % 30 == 0) {
            fprintf(stderr, "--- frame %lld ---\n", fid);
            reportTimer("frame",     t_frame);
            reportTimer("serial",    t_serial);
            reportTimer("findDebris",t_detect);
            reportTimer("  filter",  t_filter);
            reportTimer("  contours",t_contours);
            reportTimer("  centroid",t_centroids);
            reportTimer("  estimate",t_estimate);
            reportTimer("  weights", t_weights);
            reportTimer("  connect", t_connect);
            reportTimer("videoWrite",t_write);
        }

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

    // Close file write streams
    Motor_Data.close();
    All_Target_Data.close();
    Debris_Data.close();

    // Close openCV stuffs
    writer.release(); // Finishes the MP4 container structure and flushes everything to disk
    cap.release();    // Properly releases the camera/video file handle

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

    // switch arduino out of test mode
    sender.sendTargetCoordinates(0,0,-5);

    sender.flushCache();
    sleep(1);

    std::cout << "[SYSTEM READY] Pipeline active. Ready for coordinate injection stream.\n" << std::endl;

    
}