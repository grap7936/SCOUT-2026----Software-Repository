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

Last Updated: 6/10/2026
*/

/////////////////////////////////////////////////////////////

#include "Detector.hpp"

/////////////////////////////////////////////////////////////

/*
  We might not need these property definitions in here unless we want to track different object ID's to number them or track them across frames
  but for now I will keep it in because it doesn't effect the overall output or funtionality of the code.
    
     Class Properties:
     1.) next_object_ID = counter variable to keep track of different object ID's later in the scan() function
     2.) tracked_objects_centr = stores center point of the object to be able to assign it a given object ID -- { object_id: (x_center, y_center) }
*/


// Constructor (Equivalent to Python's __init__)
Detector::Detector() {
    next_object_ID = 0;
    // tracked_objects_centr initializes automatically as an empty map!
}

// Member functions (same as described at the top of the code)

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
  
    cv::Mat fg_mask, blur, thresh_temp, bg_mask,  thresh, dilated; // makes a container of objects to store the modified filtered frame for each stage (basically preallocating)

    // foregound mask that converts the background subtractor image to a non-colored background 
    cv::cvtColor(frame, fg_mask, cv::COLOR_BGR2GRAY);

    // Applies median Blur to foreground mask from last step (kernel size is 5 --> higher kernel size means more overall blur) --> this can be adjusted based on the initial overall noise that needs to be filtered out
    cv::medianBlur(fg_mask, blur, 5);

    // The grayscale image from the previous line is altered with a binary threshold that forces all "gray" pixels with a brightness greater than 25 to become pure white (255) and all pixels with a brightness less than or equal to 25 to become pure black (0).
    cv::threshold(blur, thresh_temp, 25, 255, cv::THRESH_BINARY);

    // Create the background mask which the temporary threshold passes onto
    cv::bitwise_not(bg_mask, thresh_temp);

    // Find the global background noise by applying the foreground and background images on top of each other to isolate white pixels on specifically the dark background regions as the mean function only analyzes white pixels
    double global_background_noise = cv::mean(fg_mask, bg_mask)[0]; 

    // Now subtract global background noise by 1st putting background noise into an array to subtract at each point -- use basic matrix subtraction
    cv::Mat cleaned_blur = blur - cv::Scalar(global_background_noise);

    // Apply final thresholding -- note this smaller binary thresholded value can be used here as the image has now had more noise removed from subtracting the background noise
    cv::threshold(cleaned_blur, thresh,15, 255, cv::THRESH_BINARY);


    // Dilate the image
    // thresh is passed in as this is the image being dilated; dilated stores the new dilated image as an object. 
    // Dilation works by sliding a small matrix (a kernel) over the image. If the kernel hits a white pixel, it turns the surrounding pixels white. By passing an empty cv::Mat() as an input, the kernel defaults to a 3x3 rectangular element
    // cv::Point(-1, -1) puts each kernel's anchor point (point relative to the current pixel being processed) in the center of each kernel (this is just a necessary input)
    cv::dilate(thresh, dilated, cv::Mat(), cv::Point(-1, -1), 2);

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

        if (size < 1000) { // sets parameter (size limit) for size to see where contours are made. In this case, all objects less than 1000 total pixels --> this is done with the intent of seeking out mostly small objects as small orbital debris is the main concern of our cubeSat.

            // Creates a bounding rectangle around the contour
            cv::Rect rect = cv::boundingRect(contour); // boundingRect reads through all (x,y) coordinates in a given contour and finds the leftmost and uppermost x,y coordinate and also width and height to make bounding boxes
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


void Detector::scan(cv::Mat& frame, std::vector<Target*>& targets) {

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

        // Construct new instance of target. Unassigned values default to std::nullopt as defined in the target class (this is basically the same as assigning to NONE equivalently in Python)
        Target* new_target = new Target(x_centr_pos, y_centr_pos, box.size);

        targets.push_back(new_target); // push_back works the same as the append function in Python and puts each new target at the end of the dynamically changing array/vector that targets is initialzied as
    }

}






