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

Last Updated: 6/23/2026
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
     1.) end_calibration_period   = frame index at which background-noise calibration stops
     2.) global_background_noise  = current estimated background brightness to subtract
     3.) current_frame_num        = the frame index most recently handed to scan()
*/


// Constructor (Equivalent to Python's __init__)
Detector::Detector( int blur_size, int thresh_margin, int dilation_iter, int contour_size ) {
    BLUR_KERNEL_SIZE = blur_size;
    BG_THRESHOLD_MARGIN = thresh_margin;
    DILATION_ITERATIONS = dilation_iter;
    MAX_CONTOUR_SIZE = contour_size;
    
    end_calibration_period = 0;
    global_background_noise = 0.0;
    current_frame_num = 0;
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
}

// Member functions (same as described at the top of the code)

// startCalibration() member function

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


void Detector::startCalibration() {

    // Calibrate over the next 2 frames (current_frame_num up to end_calibration_period - 1).
    // The "+2" window length is arbitrary for now and could be promoted to a parameter later
    // if calibration needs to span more or fewer frames.
    end_calibration_period = current_frame_num + 2;

    // Reset the running estimate so the upcoming calibration frames start fresh.
    global_background_noise = 0.0;
}

// calibrateBackgroundNoise() member function

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
 1.)  global_background_noise == estimated background brightness (mode intensity + 5)
*/

void Detector::calibrateBackgroundNoise(const cv::Mat& frame) {


        
        cv::Mat fg_mask, blur;// thresh_temp, bg_mask; // makes a container of objects to store the modified filtered frame for each stage (basically preallocating)

        // foregound mask that converts the background subtractor image to a non-colored background 
        cv::cvtColor(frame, fg_mask, cv::COLOR_BGR2GRAY);

        // Applies median Blur to foreground mask from last step (kernel size is 5 --> higher kernel size means more overall blur) --> this can be adjusted based on the initial overall noise that needs to be filtered out
        cv::medianBlur(fg_mask, blur, BLUR_KERNEL_SIZE);

        // // The grayscale image from the previous line is altered with a binary threshold that forces all "gray" pixels with a brightness greater than 25 to become pure white (255) and all pixels with a brightness less than or equal to 25 to become pure black (0).
        // cv::threshold(blur, thresh_temp, 30, 255, cv::THRESH_BINARY);

        // Build an intensity histogram (0..255) by tallying how many pixels have each
        // grayscale value across the whole blurred frame.
        int histogram[256] = {0};
        for (int r = 0; r < blur.rows; r++) {
            for (int c = 0; c < blur.cols; c++) {
                histogram[ blur.at<unsigned char>(r, c) ]++;
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



// filter() member function

/*
Function summary: at top of code

Inputs: 
1.) frame = "newest" frame of the camera view for tracking

Outputs:
1.) dilated = fully filtered image that is now blurred, black and white, with removed/thresholded pixels that are expanded to most fully identify objects
*/


// Data type of return variable is cv::Mat which takes in an image, processing its pixels and outputs an a processed image
cv::Mat Detector::filter(const cv::Mat& frame) { // note that cv::Mat is an image matrix and we pass by reference so that no new copies of the image are created in storage which would lower frame rate


  // Makes a matrix/image output in 7 separate stages/objects which track the evolution of the image overtime through a "pipeline" of image processing steps.  These steps/stages are all used in the code later.
  // Stages:
  // 1.) fg_mask == the foreground mask which is created by applying the background subtractor to the blurred frame. This shows the moving objects in white and the background in black.
  // 2.) blur == median blur is applied to the foreground mask to remove extraneous camera noise.
  // 3.) thresh_temp == the thresholded version of the foreground mask which is created by applying a binary threshold to the foreground mask.  This is temporary and will be overwritten later when subtracting overall background noise 
  // An arbitrary threshold helps forces all pixels to be either white (255) or black (0), which forces the foreground to be either completely white or completely black which makes it easier to detect contours.
  // 4.) bg_mask == this background mask is created by inverting the colors of the temporarily thresholded image to be later applied onto the foreground mask and then subtracted off to the get the global noise level.
  // 5.)
  // 6.) thresh == final thresholded frame after the global noise subtraction has been subtracted
  // 7.) dilated == the dilated version of the thresholded image which is created by applying a dilation operation to the thresholded image. This helps to bridge any gaps in the contours by expanding the white pixels of the moving objects which makes it easier to detect contours.
  
    cv::Mat fg_mask, blur, thresh_temp, bg_mask, thresh, dilated; // makes a container of objects to store the modified filtered frame for each stage (basically preallocating)

    // foregound mask that converts the background subtractor image to a non-colored background 
    cv::cvtColor(frame, fg_mask, cv::COLOR_BGR2GRAY);

    // Applies median Blur to foreground mask from last step (kernel size is 3 --> higher kernel size means more overall blur) --> this can be adjusted based on the initial overall noise that needs to be filtered out
    cv::medianBlur(fg_mask, blur, BLUR_KERNEL_SIZE);

    // Now subtract global background noise by 1st putting background noise into an array to subtract at each point -- use basic matrix subtraction
    cv::Mat cleaned_blur = blur - cv::Scalar(global_background_noise);

   // Apply final thresholding -- note this smaller binary thresholded value can be used here as the image has now had more noise/brightness removed from subtracting the background noise and so the threshold used should be slightly lower to detect the same objects as would be detected before.
    cv::threshold(cleaned_blur, thresh, BG_THRESHOLD_MARGIN, 255, cv::THRESH_BINARY);


    // Dilate the image
    // thresh is passed in as this is the image being dilated; dilated stores the new dilated image as an object. 
    // Dilation works by sliding a small matrix (a kernel) over the image. If the kernel hits a white pixel, it turns the surrounding pixels white. By passing an empty cv::Mat() as an input, the kernel defaults to a 3x3 rectangular element
    // cv::Point(-1, -1) puts each kernel's anchor point (point relative to the current pixel being processed) in the center of each kernel (this is just a necessary input)
    cv::dilate(thresh, dilated, cv::Mat(), cv::Point(-1, -1), DILATION_ITERATIONS);

    return dilated;
}

// contours() member function

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
    
std::pair<std::vector<std::vector<cv::Point>>, std::vector<BoxDim>> Detector::contours(cv::Mat& frame, const cv::Mat& dilated) {
    

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

        if (size < MAX_CONTOUR_SIZE) { // sets parameter (size limit) for size to see where contours are made. In this case, all objects less than 1000 total pixels --> this is done with the intent of seeking out mostly small objects as small orbital debris is the main concern of our cubeSat.

            // Creates a bounding rectangle around the contour
            cv::Rect rect = cv::boundingRect(contour); // boundingRect reads through all (x,y) coordinates in a given contour and finds the leftmost and uppermost x,y coordinate and also width and height to make bounding boxes
                                                        // creates a rect data type to store the bounding box temporarily for each contour

            // Draw the rectangle on the original frame
            // Color: Green (0, 255, 0), Thickness: 2
            //cv::rectangle(frame, rect, cv::Scalar(0, 255, 0), 2); // creates a rectangle on the given frame using the previously defined rect object. Makes it green with thickness of 2

            // Store elements in our vector of structs
            box_dims.push_back({rect.x, rect.y, rect.width, rect.height, size}); // push_back does the same as the append function in python and pushes each new element to the end of the box_dims vector set of custom dimensions defined in the struct above.
        }
    }

    return std::make_pair(contours_list, box_dims); // returns both list of each contour as well as bounding box dimensions for each specific contour (x and y will be extracted from this for input into each target instance)
}


// scan() Function -- "parent" function of the previous two member functions

/*

Function summary: filter() and contours() both run for each set of two frames but then immediately forget all the information
passed and calculated so we must set up a memory system to track and store objects and assign dimensions pertaining to the target 
class such as x and y position of "past frame"/frame_1, object size, object ID, and new x and y position of the "new frame"/frame_2.

Inputs:
1.) frame = "newest" frame of the camera view for tracking

Outputs: 
1.) targets = list of targets detected in each 


*/


void Detector::scan(cv::Mat& frame, std::vector<Target*>& targets, int frame_num) {

    // initializer for the calibration process
    if (frame_num == 0) {
        startCalibration();
    } 
    setFrameNum(frame_num);

    // Call the calibrate background noise function if the current frame number is less than the end_calibration_period limit

    if (frame_num < end_calibration_period) {
        calibrateBackgroundNoise(frame);
    }
    

    // Call filter() function for passed in frame
    cv::Mat frame_dilated = filter(frame);

    // Call contours() to get contours_list as well as each box dimensions struct for each contour (variables must be extracted from here)
    auto [contours_list, box_dims] = contours(frame, frame_dilated); // note, in this case auto is easier than its equivalent: std::pair<std::vector<std::vector<cv::Point>>, std::vector<BoxDim>>

    // Loop through all box dimensions for each different contour identified to extract x and y centroid position and size for input into each target instance
    for (const auto& box : box_dims) { 

        // define centroid position. Note that each default x and y position is at the top left corner of a given object. 
        // By adding width/2 and height/2 of the given bounding box to the x and y dimensions (situated in the upper left corner) 
        // we center each x and y position in the center/centroid of each box

        int x_centr_pos = box.x + (box.w / 2); 
        int y_centr_pos = box.y + (box.h / 2);
        // ***@@@!!! Replace with future centroid(box) function that returns (x,y) of brightest pixel -------------------------------------------------------------------------------------------------------------------------------------------------------

        // Construct new instance of target. Unassigned values default to std::nullopt as defined in the target class (this is basically the same as assigning to NONE equivalently in Python)
        Target* new_target = new Target(x_centr_pos, y_centr_pos, box.size);
        new_target->setFrameNum(current_frame_num);

        targets.push_back(new_target); // push_back works the same as the append function in Python and puts each new target at the end of the dynamically changing array/vector that targets is initialzied as
    }

}




// ***************************************** Look at this example filter code for tomorrow to see if the filtering method needs to be changed 
// or altered to this new method -- or test functionality of this program compared to the current filter version to see if it eliminates the "brightness" problem



// cv::Mat Detector::filter(const cv::Mat& frame) {

//     // 1. Convert current frame to grayscale
//     cv::Mat current_gray, last_gray_frame, fg_mask, blur, thresh, dilated; // initialize matrices to store frame data
//     cv::cvtColor(frame, current_gray, cv::COLOR_BGR2GRAY); // convert the active frame input's background to grayscale/monochrome

//     // 2. Handle the very first frame edge-case
//     if (last_gray_frame.empty()) {
//         last_gray_frame = current_gray.clone(); // sets the last_frame to a gray frame in the case that there is no previous frame (only for the 1st frame read in)
//         // Return an empty matrix because we don't have a baseline comparison yet
//         return cv::Mat::zeros(frame.size(), CV_8UC1); 
//     }

//     // 3. True Temporal Background Subtraction -- note that background subtraction differs from the previous filtering method in that it analyzes multiple frames
//     // and can subtract stationary stars to see where an object is if there is one distinct object moving across a frame.
//     cv::absdiff(current_gray, last_gray_frame, fg_mask);

//     // Update the history baseline for the NEXT frame execution
//     last_gray_frame = current_gray.clone(); // stores the current iteration of the frame into the last_gray_frame item and then current_gray_frame will be overwritten
//                                             // as the filter function reads in each subsequent frame input.

//     // Apply Median Blur to the isolated foreground mask
//     cv::medianBlur(fg_mask, blur, 5);

//     // 5. Calculate Local Noise Level dynamically 
//     // Now, cv::mean only calculates the leftover sensor noise floor, 
//     // entirely unaffected by bright stars or blooming targets.
//     double global_background_noise = cv::mean(blur)[0]; 

//     // 6. Final Thresholding
//     // Standardize your threshold ceiling now that the background is completely flattened
//     cv::threshold(blur, thresh, 15 + global_background_noise, 255, cv::THRESH_BINARY);

//     // 7. Dilate to bridge tracking structural gaps
//     cv::dilate(thresh, dilated, cv::Mat(), cv::Point(-1, -1), 2);

//     return dilated;
// }

