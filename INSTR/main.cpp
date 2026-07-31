/////////////////////////////////////////////////////////////
/*

Code Summary:  Runs all necessary functions for debris detection and tracking as well as serial UART communication between the Jetson and Arduino.

Author: Zachary Dyre, Graeme Appel

Last Updated: 7/31/2026
*/

/////////////////////////////////////////////////////////////


#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <omp.h>
#include <VmbCPP/VmbCPP.h> // Vimba X include -- library for the camera that handles talking to the camera
#include "CameraWrapper.hpp"
#include "KeyInput.hpp"
#include "Graph.hpp"
#include "Target.hpp"
#include "Detector.hpp"
#include "Selector.hpp"
#include "Sentry.hpp"
#include "ArduinoSend.hpp"
#include <unistd.h>  // For sleep() and usleep()
#include <cstdlib> // For hasDisplay()

using namespace VmbCPP;


// necessary function declarations
void writeToPID(ArduinoSend& sender, int id, int x, int y, int nx, int ny);

const float FPS = 50.0; // defined by user, Alvium Camera Max FPS is 79.0

void setupArduino(ArduinoSend& sender);

// checks if the Jetson has a screen to display to so that it won't try to display the video stream unless there is an actual way to do so via a monitor or VNC connection
bool hasDisplay() {
    const char* d = std::getenv("DISPLAY");
    return d != nullptr && d[0] != '\0';
}

static const bool GUI = hasDisplay(); // runs function to get a constant variable to determine if there is a display

int main() {

    // Set up data output files
    std::string VIDEO_FILENAME = "/home/scout/Desktop/INSTR/debrisTestResults.ts";
    std::string MOTOR_LOG_FILENAME = "/home/scout/Desktop/INSTR/motorLog.txt";
    std::string DEBRIS_LOG_FILENAME = "/home/scout/Desktop/INSTR/debrisLog.txt";
    std::string TARGET_LOG_FILENAME = "/home/scout/Desktop/INSTR/oldTargetsLog.txt";

    std::ofstream All_Target_Data;
    All_Target_Data.open(TARGET_LOG_FILENAME); // opens/creates necessary text file for inputting data into
    // adds text to the beginning of the designated text file to enable ease of functionality and accessing each file by variable name using the readtable() in MATLAB
    All_Target_Data << "id, x,y, kx,ky, vx,vy, score\n";
    All_Target_Data.close();
    
    
    std::ofstream Debris_Data;
    Debris_Data.open(DEBRIS_LOG_FILENAME); // opens/creates necessary text file for inputting data into
    // adds text to the beginning of the designated text file to enable ease of functionality and accessing each file by variable name using the readtable() in MATLAB
    Debris_Data << "frame_num, id, x,y, kx,ky, vx,vy, score\n";
    Debris_Data.close();

    std::ofstream Motor_Data;
    Motor_Data.open(MOTOR_LOG_FILENAME); // opens/creates necessary text file for inputting data into
    // adds text to the beginning of the designated text file to enable ease of functionality and accessing each file by variable name using the readtable() in MATLAB
    Motor_Data << "frame_num, motor_pos, gap\n";
    Motor_Data.close();

    // Set up arduino connection

    // Define the serial port node. On a Raspberry Pi, an Arduino UNO typically populates as "/dev/ttyCH341USB0" or "/dev/ttyACM1".
    std::string serial_port = "/dev/ttyCH341USB0"; 
    
    std::cout << "Initializing ArduinoSend on serial port: " << serial_port << std::endl; // output port connection information to the console
    ArduinoSend sender(serial_port); // create instance of the ArduinoSend function configured with the correct serial port
    
    setupArduino(sender);

    // Initialize camera
    CameraWrapper cam(FPS);

    // set parallelization thread count -- for multithreading
    omp_set_num_threads(4); // sets total number of CPU threads the system uses for multithreading the main algorithm
    cv::setNumThreads(1); // saets the number of CPU threads that the openCV code can use. -- only set as 1 because it shouldn't need to multithread on the CPU at all but just the GPU -- so this ensures no CPU threads are created which would take more time to recombine

    // Grab a real first frame and derive dimensions / channel count from it.
    // Deriving from an actual frame (rather than getWidth()/getHeight() before
    // any frame has arrived) guarantees the VideoWriter matches what we write.
    cv::Mat first = cam.getFrame(5000);
    if (first.empty()) {
        std::cerr << "Error: no initial frame from camera. Restarting...." << std::endl;
        cam.restart();
        first = cam.getFrame(5000);
        if (first.empty()) {
            std::cerr << "Error: no initial frame from camera. Exiting." << std::endl;
            return -1;
        }
    }

    // gets frame values and sets boolean (is_color) to check if camera stream is RGB (3 streams) of monochrome which is 1 stream
    int frame_width  = first.cols;
    int frame_height = first.rows;
    bool is_color    = (first.channels() == 3);

    // only the diagnostic recording is downscaled -- saves a video it records and is resolution is downscaled by half to make it save faster
    int enc_width  = frame_width  / 2;
    int enc_height = frame_height / 2;

    // Round pixel resolution to an even number — H.264 requires even dimensions in both axes.
    enc_width  &= ~1;
    enc_height &= ~1;

    // Define video file output frame size
    cv::Size frame_size(enc_width, enc_height);

    // Hardware-encoded pipeline using the Orin's NVENC block via GStreamer.
    // appsrc                       -> receives frames from writer.write()
    // video/x-raw, format=GRAY8    -> Inform GStreamer OpenCV is pushing 8-bit grayscale
    // queue                        -> prevent frames being skipped by adding to queue
    // nvvidconv                    -> Uses Jetson's VIC engine to cleanly convert GRAY8 to NVMM memory
    // video/x-raw(memory:NVMM)     -> Scales down frame for faster writing to disc
    // nvvidconv (2)                -> Moves data out of NVMM back to system memory for the CPU encoder
    // video/x-raw, format=I420     -> Outputs H.264 compatible grayscale-mapped color space
    // x264enc                      -> the actual hardware H.264 encoder
    // speed-preset/tune            -> performance speedups
    // key-int-max, bframes, ref    -> recommended tuning
    // h264parse/mpegtsmux          -> wrap the stream into an .ts container
    // config-interval              -> Essential for crash recovery in MPEG-TS
    // video/x-h264, stream-format  -> Required by mpegtsmux
    //
    // Encode at half resolution. findDebris() still runs on the full-res frame;


    // GST pipeline is what sends the stream to the output file
    std::string gst_pipeline = "appsrc ! "
    "video/x-raw, format=GRAY8 ! " 
    "queue ! "
    "nvvidconv ! " 
    "video/x-raw(memory:NVMM), width=" + std::to_string(enc_width) + ", height=" + std::to_string(enc_height) + " ! "
    "nvvidconv ! " 
    "video/x-raw, format=I420 ! " 
    "x264enc speed-preset=ultrafast tune=zerolatency bitrate=4000 key-int-max=30 bframes=0 ref=1 aud=false ! "
    "h264parse config-interval=1 ! " 
    "video/x-h264, stream-format=byte-stream ! "
    "queue ! "
    "mpegtsmux ! " 
    "filesink location=" + VIDEO_FILENAME;

    // NOTE: 4th arg is cv::CAP_GSTREAMER, telling OpenCV to treat the string
    // as a pipeline rather than a filename. FPS must match your capture rate.
    cv::VideoWriter writer(gst_pipeline, cv::CAP_GSTREAMER, 0, FPS, frame_size, is_color);
    if (!writer.isOpened()) {
        std::cerr << "Error: Could not open the GStreamer video writer." << std::endl;
        return -1;
    }

    Sentry sentry(TARGET_LOG_FILENAME); // sets up sentry class by calling the constructor
    int timeout = 2000; //ms

    // Non-blocking terminal input: press 'q' (or ESC) to quit. Works whether
    // or not a display is attached (main can run headless on the Jetson).
    KeyInput keys;
    std::cout << "DebrisTracking running. Press 'q' to quit." << std::endl;

    // open file write streams -- output file streams and appends necessary data
    Motor_Data.open(MOTOR_LOG_FILENAME, std::ios::app);
    Debris_Data.open(DEBRIS_LOG_FILENAME, std::ios::app);
    //All_Target_Data.open(TARGET_LOG_FILENAME, std::ios::app); opened in WriteToFile() in Sentry.cpp
    
    // set default debris id to -1
    int debris_id = -1;
    cv::Mat frame; // declare frame variable that we will be working with

    // END OF MAIN SETUP
    //////////////////////////////////////////////////////////////////////////////////////

    // PRIMARY CODE LOOP STARTS HERE:

    //////////////////////////////////////////////////////////////////////////////////////

    while (true) {
        // Synchronously fetch exactly one frame from the camera stream (2000ms timeout)
        frame = cam.getFrame(timeout); 

        if ( frame.empty() ) {
            // null return from getFrame() meaning no frame captured
            break;
        }
        
        // read motor position
        std::vector<double> raw = sender.readMotorPosition(Motor_Data); // saves frame number and motor position to the text file
        //double m_pos = raw[1];
        int ard_frame_num = static_cast<int>(raw[0]);

        long long fid = cam.getFrameID();// gets frame ID from camera

        long long gap = fid - ard_frame_num; // defines gap between what camera reports the frameID as and what the Arduino reports the frame ID as
                                             // theoretically this value should be 0 since the Arduino counter will start after the camera counter starts

        Motor_Data << std::setw(12) << gap << "\n"; // logs gap data onto the necessay text file

        std::cout << "Frame ID gap: " << gap << std::endl; // lists gap value if it exists


        debris_id = sentry.findDebris(frame, debris_id, fid); // one line call to the ENTIRE debris finding algorithm

        if ( debris_id != -1 ){ // if debris ID is not the default ID (meaning that tracking has started) then save all necessary information to the data file and send all necessary info to the Arduino side
            // write to file
            Target* current = (*sentry.getFullListPtr())[debris_id];
            Debris_Data << std::setw(12) << fid << "," << std::setw(12) << debris_id
                    << "," << std::setw(12) << current->getX() << "," << std::setw(12) << current->getY()
                    << "," << std::setw(12) << current->getKx() << "," << std::setw(12) << current->getKy()
                    << "," << std::setw(12) << current->getVx() << "," << std::setw(12) << current->getVy()
                    << "," << std::setw(12) << current->getDebrisLikelihood() << "\n" << std::flush;

            // write to Arduino
            std::vector<int> debris_xy = sentry.getTargetCoords(debris_id); 
            writeToPID(sender, debris_id, debris_xy[0], debris_xy[1], debris_xy[2], debris_xy[3]);
        } else {
            writeToPID(sender, -1, -1, -1, -1, -1);
        }

        writer.write(frame); // saves the video frame

        // (optional) show the frame
        // if (GUI) {
        //     cv::imshow("Frame", frame);
        //     cv::waitKey(1); // repaint the window; only meaningful with a display
        // }

        // Exit on 'q' typed into the launching terminal (works headless too)
        if (keys.quitPressed()) {
            break;
        }

    }

    // Close file write streams
    Motor_Data.close();
    //All_Target_Data.close(); handled fully in Sentry.cpp
    Debris_Data.close();

    // Close openCV stuffs
    writer.release(); // Finishes the MP4 container structure and flushes everything to disk
    
    return 0;
}


void writeToPID(ArduinoSend& sender, int id, int x, int y, int nx, int ny) {

    // Transmit coordinates down to the UNO and automatically catch the echo
    bool success = false;

    if ( nx == -1 ) { // checks if there is a predicted next step from the kalman filter and will only have that if the object is currently not found in the frame but was in previous frame 
                      // -- essentially if object flickers or passes out of sight in a given frame then this uses the expected position from the kalman filter instead
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
