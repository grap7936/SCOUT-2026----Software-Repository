#ifndef DETECTOR_HPP
#define DETECTOR_HPP
/////////////////////////////////////////////////////////////
/*
Code Summary:
Takes in a singular image and using member functions this class (and associated functions) helps to find targets in distinct phases.
1.) filter() --> uses OpenCV pre-processing functions including operations: blur, background subtraction, thresholding, dilating,
and contour framing to prepare any given frame for extracting and identifying contours of moving regions/objects in the next step.
2.) contours() --> uses either OpenCV function findContours() or other researched method to create a framework/array of different contours
and to create arrays of points and then arrays of contours to help organize original visualization of regions of movement and objects being tracked.
3.) scan() --> Note: this will be the "parent" member function of both filter() and contours() and these will run inside of the scan function.
The other functionality of this function besides using the processes defined above for filter() and contours() is creating the memory allocation 
for for each instance of the target class that is passed in and identified in each image. Additionally, for each contour make a target and populate
the entirety of the target properties (i.e x,y, size, ID, nx, ny e.t.c). 

Author: Graeme Appel

Last Updated: 6/18/2026
*/

/////////////////////////////////////////////////////////////

// Define All Necessary Libraries:

// C++ Standard Libraries:

#include <vector> // standard library for using the vector container --> Note that this enables
                  // us to make vectors which can be constantly overwritten to store new information/nodes
                  // as the built in camera takes more frames.

#include <chrono> // standard library for using time functions. For this application it can be used for both
                  // calculating the time taken to process each frame and display it on the camera feed as well
                  // as to record and list timestamps for each frame.

// OpenCV Libraries:
//  // main OpenCV library which includes all following necessary functions and libraries for this application
#include <opencv2/opencv.hpp>      

#include <omp.h>
#include <utility> // library that allows access for std::pair
#include "Target.hpp"

/////////////////////////////////////////////////////////////

// Start of Class and UDF definitions in the code

// This structure must be defined to find the same box dimensions as are used in the associated Python Code
struct BoxDim {

    // Box variables:
    // x = x coordinate pixel of where a given contour box is drawn in the frame of the picture
    // y = y coordinate pixel of where a given contour box is drawn in the frame of the picture
    // w = width of the contour box in number of pixels
    // h = height of the contour box in number of pixels
    // size is the total number of pixels that each box encapsulated (area)

    int x, y, w, h;
    double size;
};

/////////////////////////////////////////////////////////////

class Detector 
{
private:
    // Class Properties
    int BLUR_KERNEL_SIZE;
    int BG_THRESHOLD_MARGIN;
    int DILATION_ITERATIONS;
    int MAX_CONTOUR_SIZE;

    int end_calibration_period;    // frame index at which background calibration stops
    double global_background_noise; // current estimated background brightness to subtract
    int current_frame_num;          // frame index most recently passed to scan()
    /*
      The old per-object identity fields (next_object_ID counter and the
      tracked_objects_centr map) were removed once track identity moved to the Selector.
      The Detector is now stateless with respect to identity and just reports raw
      detections each frame.
    */

public:

    // Contructor (Equivalent to Python's __init__)
    Detector(int blur_size, int thresh_margin, int dilation_iter, int contour_size);

    // Backup/Default Constructor (Equivalent to Python's __init__)
    Detector();

// Getter and Setter functions for control parameters

    void setBlurKernelSize(int blur_size);

    int getBlurKernelSize();

    void setBGThresholdMargin(int thresh_margin);

    int getBGThresholdMargin();

    void setDilationIterations(int dilation_iter);

    int getDilationIterations();

    void setMaxContourSize(int contour_size);

    int getMaxContourSize();

    void setFrameNum(int frame_num);

    int getFrameNum();

// background noise functions

    double getBackgroundNoise();

    void startCalibration();

    void calibrateBackgroundNoise(const cv::Mat& frame);

// Member functions for image characterization (same as described at the top of the code)

    cv::Mat filter(const cv::Mat& frame);

    std::pair<std::vector<std::vector<cv::Point>>, std::vector<BoxDim>> contours(const cv::Mat& dilated);

    std::vector<float> computeCentroid(const cv::Mat& frame, int x, int y, int w, int h);

    void scan(cv::Mat& frame, std::vector<Target*>& targets, int frame_num);
};


#endif
