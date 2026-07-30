/////////////////////////////////////////////////////////////

/*
Code Summary:
Takes in a single image frame and utilizes class member functions to identify and process targets across distinct phases. 
Note: The Detector is now stateless with respect to target identity (ID tracking has been moved to the Selector).

1.) Calibration (startCalibration & calibrateBackgroundNoise) --> Analyzes initial frames to determine the global background noise mode via histogram tallying, establishing a baseline to subtract from future frames.
2.) filter() --> Utilizes OpenCV CUDA acceleration and shared host memory to efficiently apply GPU-based preprocessing: grayscale conversion, background subtraction, binary thresholding, and morphological dilation.
3.) contours() --> Uses cv::findContours() to extract external bounding boxes and contour point arrays. Filters out noise by enforcing MAX and MIN pixel size limits to isolate relevant objects (e.g., small orbital debris).
4.) computeCentroid() --> Iterates through the clamped Region of Interest (ROI) for each bounding box to calculate a highly precise, intensity-weighted sub-pixel center of mass based on grayscale pixel brightness.
5.) scan() --> The "parent" function that coordinates the pipeline. It manages initial calibration, executes the GPU filter and contour extraction, and utilizes OpenMP multithreading to concurrently instantiate and populate raw Target objects (x/y centroid, size, frame number) for the current frame.

Author: Graeme Appel with later modifications made by Zachary Dyre (initCudaFilters and get functions)

Last Updated: 7/30/2026
*/

/////////////////////////////////////////////////////////////

#include "Detector.hpp"

/////////////////////////////////////////////////////////////

/*
  The active class properties are now just the calibration/state scalars below. The old
  per-object ID bookkeeping fields (next_object_ID counter and the tracked_objects_centr
  map) were removed once track identity moved to the Selector - the Detector is stateless
  with respect to identity and only reports raw detections each frame.

     Class Properties:
     1.)  current_frame_num = the frame index most recently handed to scan()

     2.) end_calibration_period == variable that determines the total number of frames used to determine and later remove 
     the global background noise. This is initialized to 0 and later modified in the startCalibration() member function. 
     This number can also be altered in the same function if needed.

     3.) global_background_noise == double variable that tracks the global background brightness (between 0 - 255) that should
      be subtracted to add picture clarity across frames. This number is determined using the calibrateBackgroundNoise() member function
*/

/////////////////////////////////////////////////////////////

// Constructor 
Detector::Detector( int blur_size, int thresh_margin, int dilation_iter, int contour_size ) {
    BLUR_KERNEL_SIZE = blur_size;
    BG_THRESHOLD_MARGIN = thresh_margin;
    DILATION_ITERATIONS = dilation_iter;
    MAX_CONTOUR_SIZE = contour_size;
    MIN_CONTOUR_SIZE = 2;
    
    end_calibration_period = 0;
    global_background_noise = 0.0;
    current_frame_num = 0;

    // Build the GPU filter primitives once from the parameters set above.
    initCudaFilters();
}

// Backup/Default Constructor (Equivalent to Python's __init__)
Detector::Detector() {
    BLUR_KERNEL_SIZE = 5;
    BG_THRESHOLD_MARGIN = 10;
    DILATION_ITERATIONS = 1;
    MAX_CONTOUR_SIZE = 1000;

    end_calibration_period = 0;
    global_background_noise = 0.0;
    current_frame_num = 0;

    // Build the GPU filter primitives once from the default parameters set above.
    initCudaFilters();
}

/////////////////////////////////////////////////////////////

/* 1.) Function initCudaFilters()
 * description:
 *  Constructs the reusable CUDA filter primitives (median blur + dilation) from the
 *  current control parameters. Called once per Detector from each constructor so the
 *  per-frame filter() path never has to (re)allocate these GPU objects.
 *
 *  Median blur: single-channel 8-bit input, window == BLUR_KERNEL_SIZE (must be odd
 *  and >= 3; OpenCV enforces odd). Matches the CPU cv::medianBlur behavior.
 *
 *  Dilation: a 3x3 rectangular structuring element reproduces the CPU call
 *  cv::dilate(..., cv::Mat(), cv::Point(-1,-1), DILATION_ITERATIONS), where an empty
 *  cv::Mat() kernel defaults to a 3x3 rect with center anchor. iterations is baked in
 *  here rather than passed per-call.
 */

 /////////////////////////////////////////////////////////////
void Detector::initCudaFilters() {

    // Median blur operates on CV_8UC1 (grayscale). Window size must be odd; the
    // constructors pass odd BLUR_KERNEL_SIZE values (default 5).
    d_median_filter = cv::cuda::createMedianFilter(CV_8UC1, BLUR_KERNEL_SIZE);

    // 3x3 rectangular structuring element with center anchor, applied
    // DILATION_ITERATIONS times - equivalent to the empty-kernel CPU dilate.
    cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    d_dilate_filter = cv::cuda::createMorphologyFilter(
        cv::MORPH_DILATE, CV_8UC1, dilate_kernel, cv::Point(-1, -1), DILATION_ITERATIONS);
}

// Necessary Get Functions to setup CudaFilters and other necessary parameters

// Sets the medianBlur kernel size
void Detector::setBlurKernelSize(int blur_size) {
    BLUR_KERNEL_SIZE = blur_size;
}

// Returns the medianBlur kernel size
int Detector::getBlurKernelSize() {
    return BLUR_KERNEL_SIZE;
}

// Sets the threshold value to use after BG subtraction
void Detector::setBGThresholdMargin(int thresh_margin) {
    BG_THRESHOLD_MARGIN = thresh_margin;
}

// Returns the threshold value to use after BG subtraction
int Detector::getBGThresholdMargin() {
    return BG_THRESHOLD_MARGIN;
}

// Sets the number of dilations to perform on thresholded frame
void Detector::setDilationIterations(int dilation_iter) {
    DILATION_ITERATIONS = dilation_iter;
}

// Returns the number of dilations to perform on thresholded frame
int Detector::getDilationIterations() {
    return DILATION_ITERATIONS;
}

// Sets the maximum contour size to consider 
void Detector::setMaxContourSize(int contour_size) {
    MAX_CONTOUR_SIZE = contour_size;
}

// Returns the maximum contour size to consider 
int Detector::getMaxContourSize() {
    return MAX_CONTOUR_SIZE;
}

// Sets the frame index the detector is currently working on
void Detector::setFrameNum(int frame_num) {
    current_frame_num = frame_num;
}

// Returns the frame index the detector is currently working on
int Detector::getFrameNum() {
    return current_frame_num;
}

// Returns the current estimated background-noise brightness level
double Detector::getBackgroundNoise() {
    return global_background_noise;

}
/////////////////////////////////////////////////////////////

// Member functions (same as described at the top of the code)

// 2.) startCalibration() member function

/*
 Function summary: Uses the current frame count inside of the sentry class (which starts from when the camera stream starts running) 
 and adds a thresholding number (defined arbitrarily for now) to create an integer variable which will determine the number of frames 
 used in the next function to determine an overall global background noise. This will be passed into the next function which will use 
 this variable to loop through and average global background noise until reaching the limit defined by this function.

NOTE: if the background of a region that the cubesat is viewing changes drastically, this function will be called again to set a new 
global background noise value to be subtracted.

 Inputs:
 None - it reads the member current_frame_num (set via setFrameNum / scan) rather than taking an argument.

 Outputs:
 None - sets end_calibration_period and resets global_background_noise to 0.
*/

/////////////////////////////////////////////////////////////

void Detector::startCalibration() {

    end_calibration_period = current_frame_num + 2; // Calibrate over the next 2 frames (current_frame_num up to end_calibration_period - 1).
                                            // The "+2" window length is arbitrary for now and could be promoted to a parameter later
                                            // if calibration needs to span more or fewer frames.

    // Reset the running estimate so the upcoming calibration frames start fresh.
    global_background_noise = 0.0;                                         
}

/////////////////////////////////////////////////////////////

// 3.) calibrateBackgroundNoise() member function

/*
 Function summary: Estimates the background brightness of the current frame and stores it
 in global_background_noise for the filter() stage to subtract. Rather than averaging a
 masked mean (the earlier approach, now commented out below), it takes the statistical MODE
 of the blurred grayscale frame - the single most common pixel intensity - which for a
 mostly-empty star field is the dark background level. A small fixed margin (+5) is added so
 faint background texture is also subtracted away. This runs once per calibration frame
 (frames before end_calibration_period); the last call's value is the one used.

 Inputs: 
 1.)  frame = "newest" frame of the camera view for tracking

 Outputs:
 1.)  global_background_noise == overall global background noise averaged over X number of frames determined by the threshold in the previous function; 
                                 estimated background brightness (mode intensity + 5).
*/

/////////////////////////////////////////////////////////////

void Detector::calibrateBackgroundNoise(const cv::Mat& frame) {


        cv::Mat mono, blur;// thresh_temp, bg_mask; // makes a container of objects to store the modified filtered frame for each stage (basically preallocating)

        if (frame.channels() == 1) {
            mono = cv::Mat(frame);
        } else if (frame.channels() >= 3) {
            // foregound mask that converts the background subtractor image to a non-colored background 
            cv::cvtColor(frame, mono, cv::COLOR_BGR2GRAY);
        }

        // Applies median Blur to foreground mask from last step (kernel size is 5 --> higher kernel size means more overall blur) --> this can be adjusted based on the initial overall noise that needs to be filtered out
        cv::medianBlur(mono, blur, BLUR_KERNEL_SIZE);
        
        // // The grayscale image from the previous line is altered with a binary threshold that forces all "gray" pixels with a brightness greater than 25 to become pure white (255) and all pixels with a brightness less than or equal to 25 to become pure black (0).
        // cv::threshold(blur, thresh_temp, 30, 255, cv::THRESH_BINARY);

        // Build an intensity histogram (0..255) by tallying how many pixels have each
        // grayscale value across the whole blurred frame.
        
        // Uses OpenMP parallelization to speed up histogram generation across frame rows
        int histogram[256] = {0};
        #pragma omp parallel for reduction(+:histogram[:256])
        for (int r = 0; r < blur.rows; r++) {
            // One row-pointer lookup per row, then flat contiguous access across the row.
            // Replaces per-pixel bounds-checked blur.at<>() with direct pointer indexing.
            const unsigned char* pRow = blur.ptr<unsigned char>(r);
            for (int c = 0; c < blur.cols; c++) {
                histogram[ pRow[c] ]++;
            }
        }
        

        // The mode (most frequent intensity) is the dominant background level in a
        // mostly-empty frame - find the bin with the highest count.
        int mode_intensity = 0;
        int mode_count = 0;
        for (int i = 0; i < 256; i++) {
            if (histogram[i] > mode_count) { mode_count = histogram[i]; mode_intensity = i; }
        }
        double new_global_background_noise = static_cast<double>(mode_intensity);

        global_background_noise = new_global_background_noise;

        // // Create the background mask which the temporary threshold passes onto
        // cv::bitwise_not(thresh_temp, bg_mask);

        // // Find the global background noise by applying the foreground and background images on top of each other to isolate white pixels on specifically the dark background
        // double new_global_background_noise = cv::mean(fg_mask, bg_mask)[0]; 


        // // Compute averaged global back_ground_noise as (total_noise)/(num_frames_used)
        // global_background_noise = (global_background_noise + new_global_background_noise) / (2);

}


/////////////////////////////////////////////////////////////

// 4.) filter() member function

/*
Function summary: at top of code but repasted here:  uses OpenCV pre-processing functions including operations: blur, background subtraction, thresholding, dilating,
and contour framing to prepare any given frame for extracting and identifying contours of moving regions/objects in the next step. All specific steps are detailed more below.

Inputs: 
1.) frame = "newest" frame of the camera view for tracking

Outputs:
1.) dilated = fully filtered image that is now blurred, black and white, with removed/thresholded pixels that are expanded to most fully identify objects
*/

/////////////////////////////////////////////////////////////

// Data type of return variable is cv::Mat which takes in an image, processing its pixels and outputs an a processed image
cv::Mat Detector::filter(const cv::Mat& frame) { // note that cv::Mat is an image matrix and we pass by reference so that no new copies of the image are created in storage which would lower frame rate


  // The image processing pipeline now relies on GPU acceleration (CUDA) mapped to shared memory buffers.

  // Stages:
  // 1.) Shared Memory Check: Allocates cv::cuda::HostMem blocks if they haven't been created yet to allow zero-copy host-to-device transfers.
  // 2.) Device Mapping: Maps the CPU frame directly to the GPU (d_in) avoiding expensive deep copies.
  // 3.) Grayscale Conversion: Converts the mapped GPU frame to single-channel (d_mono) using cv::cuda::cvtColor.
  // 4.) Background Subtraction: Subtracts the global_background_noise scalar from the mono frame on the GPU (d_cleaned).
  // 5.) Thresholding: Applies a binary threshold to isolate moving pixels above the background margin (d_thresh).
  // 6.) Dilation: Applies the pre-initialized CUDA dilation filter to bridge gaps in the contours (d_out).

    // Lazily allocate the shared buffers once we know the frame geometry.
    // Reused every subsequent frame -> zero per-frame allocation.
    if (!shared_bufs_ready ||
        h_frame_shared.size() != frame.size() ||
        h_frame_shared.type() != frame.type()) {
        h_frame_shared   = cv::cuda::HostMem(frame.size(), frame.type(),
                                             cv::cuda::HostMem::SHARED);
        h_dilated_shared = cv::cuda::HostMem(frame.size(), CV_8UC1,
                                             cv::cuda::HostMem::SHARED);
        shared_bufs_ready = true;
    }

    // Copy the incoming frame into the shared host view ONCE. After this, the
    // device sees the same bytes with no upload. (See note below about removing
    // even this copy at the camera boundary.)
    frame.copyTo(h_frame_shared.createMatHeader());

    // Device views onto the shared buffers -- these are handles, not copies.
    cv::cuda::GpuMat d_in  = h_frame_shared.createGpuMatHeader();
    cv::cuda::GpuMat d_out = h_dilated_shared.createGpuMatHeader();

    // Grayscale (skip if already mono).
    if (frame.channels() == 1) {
        d_mono = d_in;
    } else {
        cv::cuda::cvtColor(d_in, d_mono, cv::COLOR_BGR2GRAY);
    }

    //d_median_filter->apply(d_mono, d_blur); too slow for real-time
    cv::cuda::subtract(d_mono, cv::Scalar(global_background_noise), d_cleaned);
    cv::cuda::threshold(d_cleaned, d_thresh, BG_THRESHOLD_MARGIN, 255, cv::THRESH_BINARY);

    // Dilate straight into the shared output buffer. No download needed --
    // the host view is already valid once the stream syncs.
    d_dilate_filter->apply(d_thresh, d_out);

    // The shared/mapped path does NOT implicitly sync the way download() did.
    // Force the device work to finish before the host reads the buffer.
    cv::cuda::Stream::Null().waitForCompletion();

    // Clone into a caller-owned Mat. findContours() modifies its input in place
    // and the next frame's filter() reuses h_dilated_shared, so the caller must
    // NOT hold a bare header into the shared buffer.
    return h_dilated_shared.createMatHeader();//.clone(); // .clone was old and is probably not needed
}


/////////////////////////////////////////////////////////////

// 5.) contours() member function

/*
    contours() -- contours function will later be used in the Scan function so it is placed here first -- explained above

    Function summary: at top of code

    Inputs: 
    1.) frame = "newest" frame of the camera view for tracking
    2.) dilated = fully filtered image that is now blurred, black and white, with removed/thresholded pixels that are expanded to most fully identify objects

    Outputs: 
    1.) contours = final array/vector of contours which are bounded, identified areas that could be objects in the filtered frames
    2.) box_dim = list of box dimensions for each contour formed in the contour function such as x and y position and


*/

    // Note: In C++ in order to return 2 variables by a function as is done in the python script before this you must use std::pair, or by passing references.
    
/////////////////////////////////////////////////////////////

std::pair<std::vector<std::vector<cv::Point>>, std::vector<BoxDim>> Detector::contours(const cv::Mat& dilated) {
    

    std::vector<std::vector<cv::Point>> contours_list; // cv::Point represents a point in 2D with x and y coordinates. This is put into a vector to represent a contour which is a collection of points that form the outline of a moving object. This is then put into another vector to represent all the contours in the frame.
                                                    // i.e each contour is a vector of points (x,y) and each contour is put into a vector to keep track of each contour/moving object.
    // findContours expects a hierarchy output, even if we don't explicitly use it
    std::vector<cv::Vec4i> hierarchy;  //cv::Vec4i --> This stands for a Vector of 4 Integers. For every contour found, OpenCV stores 4 specific numbers tracking its relationships: [Next contour at the same level, Previous contour at the same level, First Child contour inside it, Parent contour outside it]. 
                                        // hierarchy: This master list pairs up pixel-for-pixel with your contours vector, acting as a structural family tree so the program knows which shapes are holes inside other shapes.


    cv::findContours(dilated, contours_list, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); 
    // dilated image is frame input where contours are found from
    // contours_list is the vector (dynamically changing array) of every contour stored
    // hierarchy provides information in how each contour is formed i.e if one is inside the other e.t.c -- only needed to input for this application so no errors are thrown
    // RETR_EXTERNAL -- external retrieval. This addition means that pixels only on the outside of objects are detected but not any holes or other things inside of full objects
    // CHAIN_APPROX_SIMPLE -- stores coordinate points to remember the shape of each contour boundary. In this case, the APPROX means it does not store every pixel but intead removes all redundant data points along straight lines

    // initialize box dimensions vector for use later for inputting/storing necessary information
    std::vector<BoxDim> box_dims;

    for (const auto& contour : contours_list) { // auto effectively makes the computer assume the correct data type for contour --> otherwise passing by reference would look like: std::vector<cv::Point>& contour : contours_list
                                                // const makes sure that the contours do not change inside the loop which can prevent errors 
        double size = cv::contourArea(contour); // uses contourArea to return total number of pixels (i.e size) that each contour/bounding box envelopes

        if (size < MAX_CONTOUR_SIZE && size > MIN_CONTOUR_SIZE) { // sets parameter (size limit) for size to see where contours are made. In this case, all objects less than 1000 total pixels --> this is done with the intent of seeking out mostly small objects as small orbital debris is the main concern of our cubeSat.

            // Creates a bounding rectangle around the contour
            // Only uncomment if wanting to see the overall bounding boxes for each object displayed in frame -- not that this will make the realtime viewing experience much slower
            // cv::Rect rect = cv::boundingRect(contour); // boundingRect reads through all (x,y) coordinates in a given contour and finds the leftmost and uppermost x,y coordinate and also width and height to make bounding boxes
                                                        // creates a rect data type to store the bounding box temporarily for each contour

            // Draw the rectangle on the original frame
            // Color: Green (0, 255, 0), Thickness: 2
            cv::rectangle(frame, rect, cv::Scalar(0, 255, 0), 2); // creates a rectangle on the given frame using the previously defined rect object. Makes it green with thickness of 2

            // Store elements in our vector of structs
            box_dims.push_back({rect.x, rect.y, rect.width, rect.height, size}); // push_back does the same as the append function in python and pushes each new element to the end of the box_dims vector set of custom dimensions defined in the struct above.
        }
    }

    return std::make_pair(contours_list, box_dims); // returns both list of each contour as well as bounding box dimensions for each specific contour (x and y will be extracted from this for input into each target instance)
}

// 6.) computeCentroid() member function

/*
 Function summary: Calculates the intensity-weighted centroid (center of mass) of a detected object
 within a bounding box. Instead of simply taking the geometric center of the box, it iterates over 
 the pixels in the Region of Interest (ROI) and computes the weighted average of X and Y coordinates 
 based on pixel brightness. This provides a more accurate sub-pixel center for irregular shapes.

 Inputs:
 1.) mono = single-channel (grayscale) image frame.
 2.) x, y = starting top-left coordinates of the bounding box.
 3.) w, h = width and height of the bounding box.

 Outputs: 
 1.) std::vector<float> = a 2-element vector containing the precise [x, y] sub-pixel centroid coordinates.

 */ 

/////////////////////////////////////////////////////////////

std::vector<float> Detector::computeCentroid(const cv::Mat& mono, int x, int y, int w, int h) {

    // if there is no grayscale image frame detected in the mono object then return zeros for each x and y centroid coordinates as they are not relevant.
    if (mono.empty()) {
        return {0.0, 0.0};
    }

    // Set the Region of Interest (ROI) to the image/boundary box boundaries as defined previously in the contours function
    int startX = std::max(0, x); // sets the leftmost boundary of a given frame so that "if a bounding box starts off screen it will start reading x positions starting from the leftmost side of the frame"
    int startY = std::max(0, y); // sets topmost boyndary of a given frame so that "..."

    int endX = std::min(mono.cols, x + w); // sets rightmost boundary of a given frame by adding frame width so that "..."
    int endY = std::min(mono.rows, y + h); // sets bottom-most boundary of a given frame by adding frame height so that "..."

    // initialize necessary accumulator variables
    float sumI = 0.0; // The total Intensity (brightness) of all pixels added together. Think of this as the total "mass" of the object.
    float sumX = 0.0; // The running total of every pixel's X-coordinate multiplied by that pixel's brightness.
    float sumY = 0.0; // The running total of every pixel's Y-coordinate multiplied by that pixel's brightness.

    // loops through every single pixel inside the Region of Interest (ROI) to calculate the object's intensity-weighted center of mass.
    for (int row = startY; row < endY; ++row) // This is the outer loop. It scans the bounding box vertically, starting from the top edge (startY) and moving down row by row to 
                                              // the bottom edge (endY). In image processing, the row corresponds to the Y-coordinate.
    {
        const uint8_t* pRow = mono.ptr<uint8_t>(row); // This is a critical performance optimization. Instead of asking OpenCV to calculate the memory address of every single pixel individually (which is slow), this 
                                                      // line grabs a direct memory pointer to the very beginning of the current row. uint8_t tells the compiler to expect standard 8-bit grayscale pixel values (where brightness ranges from 0 to 255).

        for (int col = startX; col < endX; ++col) // This is the inner loop. For the current row, it scans horizontally from the left edge (startX) to the right edge (endX). The col corresponds to the X-coordinate.
        {
            float I = static_cast<float>(pRow[col]); // This line extracts the actual brightness (intensity) of the specific pixel being looked at.
                                                     // pRow[col] uses the fast memory pointer to grab the 8-bit value.
                                                     // static_cast<float>(...) converts that whole number into a decimal (float) so the subsequent math calculations are precise and don't accidentally truncate.

            sumI += I; // This adds the current pixel's brightness to the running total. This represents the total "mass" of the detected object.
            sumX += col * I; // This calculates the pixel's "pull" on the X-axis. By multiplying the X-coordinate (col) by the brightness (I), a very bright pixel will pull the final calculated center of mass much closer to its own X-position than a dim pixel would.
            sumY += row * I; //This does the exact same calculation as the previous line, but for the Y-axis. It multiplies the Y-coordinate (row) by the brightness (I) to calculate the vertical pull of that specific pixel.
        }
    }

    // If there is no intensity in the ROI, return its center
    if (sumI <= 0.0) // checks if the total intensity ("mass") of the pixels inside the bounding box is zero. This would only happen if every single pixel inside the region was completely black.
    {
        // If sumI is equal to 0 or negative then the later centroiding calculation becomes either impossible (undefined) or a negative number which would return an error
        // Instead, the following lines return the center of a given bounding box as the object centroid if sumI is <= 0
        float tempX = startX + (endX - startX) / 2.0; 
        float tempY = startY + (endY - startY) / 2.0;
        return
        {
            tempX,
            tempY
        }; // This returns that safe, geometric middle coordinate if the failsafe was triggered, ending the function early.
    }

    return
    {

        //  final computations of center of mass formulas given by:
        // Centroid_x = sumX / sumI          and           Centroid_y = sumY / sumI  
        sumX / sumI, 
        sumY / sumI
    };
}

/////////////////////////////////////////////////////////////

// 7.) scan() Function -- "parent" function of the previous two member functions

/*

Function summary: filter() and contours() both run for each set of two frames but then immediately forget all the information
passed and calculated so we must set up a memory system to track and store objects and assign dimensions pertaining to the target 
class such as x and y position of "past frame"/frame_1, object size, object ID, and new x and y position of the "new frame"/frame_2.

Inputs:
1.) frame = "newest" frame of the camera view for tracking

Outputs: 
1.) targets = list of targets detected in each 


*/

///////////////////////////////////////////////////////////// 

void Detector::scan(cv::Mat& frame, std::vector<Target*>& targets, int frame_num) {


    // Convert the frame to grayscale a SINGLE time before the parallel loop.
    // If the camera is Mono8 this is a no-op shallow reference; if color, one
    // conversion for the whole frame instead of one per contour.
    cv::Mat mono_frame;
    if (frame.channels() == 1) {
        mono_frame = frame;
    } else {
        cv::cvtColor(frame, mono_frame, cv::COLOR_BGR2GRAY);
    }


    // initializer for the calibration process
    if (frame_num == 0) {

    startCalibration();
    } 

    setFrameNum(frame_num);

    // Call the calibrate background noise function if the current frame number is less than the end_calibration_period limit

    if (frame_num < end_calibration_period) {
        calibrateBackgroundNoise(mono_frame);
    }
    

    // Call filter() function for passed in frame
    cv::Mat frame_dilated = filter(mono_frame);

    // Call contours() to get contours_list as well as each box dimensions struct for each contour (variables must be extracted from here)
    auto [contours_list, box_dims] = contours(frame_dilated); // note, in this case auto is easier than its equivalent: std::pair<std::vector<std::vector<cv::Point>>, std::vector<BoxDim>>

    // Loop through all box dimensions for each different contour identified to extract x and y centroid position and size for input into each target instance
      int size = box_dims.size();
    targets.resize(size);

    // Uses OpenMP parallelization/multithreading to create and populate target instances concurrently for better performance
    #pragma omp parallel for
    for (int i = 0; i < size; i++) { 

        auto& box = box_dims[i];

        // define centroid position using the before defined light intensity centering function computeCentoid() as defined above
        std::vector<float> centroid = computeCentroid(mono_frame, box.x, box.y, box.w, box.h);

        // Construct new instance of target. Unassigned values default to std::nullopt as defined in the target class (this is basically the same as assigning to NONE equivalently in Python)
        Target* new_target = new Target(centroid[0], centroid[1], box.size);
        new_target->setFrameNum(current_frame_num);

        targets[i] = new_target; // push_back works the same as the append function in Python and puts each new target at the end of the dynamically changing array/vector that targets is initialzied as
    }

}

