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
#include "ArduinoSend.hpp"
#include <unistd.h>  // For sleep() and usleep()
#include <cstdlib> // For hasDisplay()

#include "Timer.hpp"

extern TimerStats t_filter, t_contours, t_centroids;
extern TimerStats t_estimate, t_weights, t_connect;

TimerStats t_frame, t_serial, t_detect, t_write;

using namespace VmbCPP;

void writeToPID(ArduinoSend& sender, int id, int x, int y, int nx, int ny);

const int FPS = 79; // defined by user to match camera

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

    // Initialize camera
    CameraWrapper cam;

    // set parallelization thread count
    omp_set_num_threads(4);
    cv::setNumThreads(1);

    // Grab a real first frame and derive dimensions / channel count from it.
    // Deriving from an actual frame (rather than getWidth()/getHeight() before
    // any frame has arrived) guarantees the VideoWriter matches what we write.
    cv::Mat first = cam.getFrame(5000);
    if (first.empty()) {
        std::cerr << "Error: no initial frame from camera. Exiting." << std::endl;
        return -1;
    }
    int frame_width  = first.cols;
    int frame_height = first.rows;
    bool is_color    = (first.channels() == 3);

    // only the diagnostic recording is downscaled.
    int enc_width  = frame_width  / 2;
    int enc_height = frame_height / 2;

    // Round to even — H.264 requires even dimensions in both axes.
    enc_width  &= ~1;
    enc_height &= ~1;

    // Define video file output
    cv::Size frame_size(enc_width, enc_height);

    // Hardware-encoded pipeline using the Orin's NVENC block via GStreamer.
    // appsrc         -> receives frames from writer.write()
    // videoconvert   -> BGR (what OpenCV hands over) to a format nvvidconv accepts
    // x264enc  -> the actual hardware H.264 encoder
    // speed-preset/tune -> performance speedups
    // key-int-max, bframes, ref -> recommended tuning
    // h264parse/qtmux -> wrap the stream into an .mp4 container
    // Encode at half resolution. findDebris() still runs on the full-res frame;
    std::string gst_pipeline =
        "appsrc ! videoconvert ! "
        "nvvidconv ! video/x-raw(memory:NVMM),width=" + std::to_string(enc_width) +
        ",height=" + std::to_string(enc_height) + " ! "
        "nvvidconv ! video/x-raw,format=I420 ! "
        "x264enc speed-preset=ultrafast tune=zerolatency "
        "bitrate=4000 key-int-max=30 bframes=0 ref=1 aud=false ! "
        "h264parse ! qtmux ! filesink location=" + VIDEO_FILENAME;

    // NOTE: 4th arg is cv::CAP_GSTREAMER, telling OpenCV to treat the string
    // as a pipeline rather than a filename. FPS must match your capture rate.
    cv::VideoWriter writer(gst_pipeline, cv::CAP_GSTREAMER, 0, FPS, frame_size, is_color);
    if (!writer.isOpened()) {
        std::cerr << "Error: Could not open the GStreamer/NVENC video writer." << std::endl;
        return -1;
    }

    Sentry sentry;
    int timeout = 2000; //ms

    // Non-blocking terminal input: press 'q' (or ESC) to quit. Works whether
    // or not a display is attached (main can run headless on the Jetson).
    KeyInput keys;
    std::cout << "DebrisTracking running. Press 'q' to quit." << std::endl;

    // open file write streams
    Motor_Data.open(MOTOR_LOG_FILENAME, std::ios::app);
    Debris_Data.open(DEBRIS_LOG_FILENAME, std::ios::app);
    All_Target_Data.open(TARGET_LOG_FILENAME, std::ios::app);
    

    int debris_id = -1;
    cv::Mat frame;
    while (true) {
        Timer _frame(t_frame);

        // Synchronously fetch exactly one frame from the camera stream (2000ms timeout)
        frame = cam.getFrame(timeout);

        if ( frame.empty() ) {
            // null return from getFrame() meaning no frame captured
            break;
        }
        
        // read motor position
        std::vector<double> raw;
        { Timer _(t_serial); raw = sender.readMotorPosition(Motor_Data); }
        double m_pos = raw[1];
        double ard_frame_num = raw[0];

        long long fid = cam.getFrameID();
        { Timer _(t_detect); debris_id = sentry.findDebris(frame, debris_id, fid); }

        if ( debris_id != -1 ){
            // write to file
            Target* current = (*sentry.getFullListPtr())[debris_id];
            Debris_Data << fid << ", " << debris_id
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
    All_Target_Data.close();
    Debris_Data.close();

    // Close openCV stuffs
    writer.release(); // Finishes the MP4 container structure and flushes everything to disk
    
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
