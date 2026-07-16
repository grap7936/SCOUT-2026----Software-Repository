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

// OpenCV CUDA modules for GPU-accelerated frame preprocessing on Jetson.
// core: GpuMat container; cudaimgproc: cvtColor/threshold; cudaarithm: subtract;
// cudafilters: median blur + morphology (dilate). These require an OpenCV build
// with WITH_CUDA=ON and the opencv_contrib cudafilters/cudaimgproc modules.
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>

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
    long long int current_frame_num;          // frame index most recently passed to scan()

    /*
      GPU-side preprocessing state (Jetson).
      These device buffers persist across frames so we don't reallocate GPU memory
      30x/second. Each stage of filter() writes into a dedicated GpuMat; only the
      final dilated mask is copied back to the host for findContours().

        d_frame   - uploaded raw frame (BGR or already-mono)
        d_mono    - single-channel grayscale
        d_blur    - median-blurred grayscale
        d_cleaned - blur minus global background noise
        d_thresh  - binary thresholded
        d_dilated - dilated final mask (downloaded to host)
    */
    cv::cuda::GpuMat d_frame, d_mono, d_blur, d_cleaned, d_thresh, d_dilated;

    /*
      Pre-built CUDA filter primitives. Constructing these allocates internal GPU
      scratch buffers, so we build them ONCE (in initCudaFilters, called from the
      constructors) and reuse every frame. They depend only on kernel/iteration
      parameters, which are fixed after construction.
    */
    cv::Ptr<cv::cuda::Filter> d_median_filter;   // median blur
    cv::Ptr<cv::cuda::Filter> d_dilate_filter;   // morphological dilation

    // Builds the CUDA filter primitives from the current BLUR_KERNEL_SIZE /
    // DILATION_ITERATIONS parameters. Called from both constructors.
    void initCudaFilters();
    /*
      The old per-object identity fields (next_object_ID counter and the
      tracked_objects_centr map) were removed once track identity moved to the Selector.
      The Detector is now stateless with respect to identity and just reports raw
      detections each frame.
    */

    // Shared (mapped, zero-copy) host/device buffers for the Jetson unified memory path.
    // HostMem(SHARED) allocates one physical buffer exposed as BOTH a cv::Mat (host view)
    // and a cv::cuda::GpuMat (device view). No memcpy happens between the two views.
    cv::cuda::HostMem h_frame_shared;   // input frame,  shared
    cv::cuda::HostMem h_dilated_shared; // output mask,  shared
    bool shared_bufs_ready = false;     // lazily sized on first frame

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