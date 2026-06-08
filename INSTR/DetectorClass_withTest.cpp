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

Last Updated: 6/8/2026
*/

/////////////////////////////////////////////////////////////

// Define All Necessary Libraries:

// C++ Standard Libraries:
#include <iostream> // standard library for input and output streams

#include <vector> // standard library for using the vector container --> Note that this enables
                  // us to make vectors which can be constantly overwritten to store new information/nodes
                  // as the built in camera takes more frames.

#include <chrono> // standard library for using time functions. For this application it can be used for both
                  // calculating the time taken to process each frame and display it on the camera feed as well
                  // as to record and list timestamps for each frame.

// OpenCV Libraries:
//  // main OpenCV library which includes all following necessary functions and libraries for this application
#include <opencv2/opencv.hpp>      

#include <map> // allows for mapping an ID to specific coordinates in the properties of the detector class
#include <utility> // library that allows access for std::pair
#include <optional> // allows for "optional" inputs into the target class as this program only acquired x,y, and size and other parameters are likely found in other associated code

/////////////////////////////////////////////////////////////

// Start of Class and UDF definitions in the code

class target
{

    /*
    Definition of relevant properties:
    1.) x = x position of target in previous camera frame 
    2.) y = y position of  target in previous camera frame  
    3.) size = size in amount of pixels of the detected object 
    4.) ID = each object in every frame will have an object ID which will be tracked and noted across frames. 
    5.) nx = new x position of target in the next camera frame 
    6.) ny = new y position of target in the next camera frame 
    7.) isStar = float between 0.0 and 1.0 where the closer to 1.0 the more likely it is a star
    */ 

// Above defined properties must be accessible anywhere in the code (hence public)
public:
    int x;
    int y;
    int size;

    // Using std::optional to mimic Python's "None" for unassigned values
    std::optional<int> ID;
    std::optional<int> nx;
    std::optional<int> ny;
    std::optional<double> isStar;

    // Constructor
    target(int x_pos, int y_pos, int size_obj, 
           std::optional<int> obj_ID = std::nullopt, 
           std::optional<int> nx_pos = std::nullopt, 
           std::optional<int> ny_pos = std::nullopt, 
           std::optional<double> isStar_in = std::nullopt)
        : x(x_pos), y(y_pos), size(size_obj), ID(obj_ID), nx(nx_pos), ny(ny_pos), isStar(isStar_in) {}

};

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

class detector 
{

/*
  We might not need these property definitions in here unless we want to track different object ID's to number them or track them across frames
  but for now I will keep it in because it doesn't effect the overall output or funtionality of the code.
    
     Class Properties:
     1.) next_object_ID = counter variable to keep track of different object ID's later in the scan() function
     2.) tracked_objects_centr = stores center point of the object to be able to assign it a given object ID -- { object_id: (x_center, y_center) }
*/

public:

// These property lines likely are not necessary for this application but I will keep them in for completeness for the time being
// Class Properties 
    int next_object_ID;
    
    // Maps an object ID (int) to a center coordinate (x, y) using std::pair<int, int>
    std::map<int, std::pair<int, int>> tracked_objects_centr;

    // Constructor (Equivalent to Python's __init__)
    detector() {
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
cv::Mat filter(const cv::Mat& frame) { // note that cv::Mat is an image matrix and we pass by reference so that no new copies of the image are created in storage which would lower frame rate


  // Makes a matrix/image output in 7 separate stages/objects which track the evolution of the image overtime through a "pipeline" of image processing steps.  These steps/stages are all used in the code later.
  // Stages:
  // 1.) fgmask == the foreground mask which is created by applying the background subtractor to the blurred frame. This shows the moving objects in white and the background in black.
  // 2.) thresh == the thresholded version of the foreground mask which is created by applying a binary threshold to the foreground mask. 
  // An arbitrary threshold helps forces all pixels to be either white (255) or black (0), which forces the foreground to be either completely white or completely black which makes it easier to detect contours.
  // 3.) dilated == the dilated version of the thresholded image which is created by applying a dilation operation to the thresholded image. This helps to bridge any gaps in the contours by expanding the white pixels of the moving objects which makes it easier to detect contours.
  
    cv::Mat fg_mask, blur, thresh, dilated; // makes a container of objects to store the modified filtered frame for each stage (basically preallocating)

    // foregound mask that converts the background subtractor image to a non-colored background 
    cv::cvtColor(frame, fg_mask, cv::COLOR_BGR2GRAY);

    // Applies median Blur to foreground mask from last step (kernel size is 5 --> higher kernel size means more overall blur) --> this can be adjusted based on the initial overall noise that needs to be filtered out
    cv::medianBlur(fg_mask, blur, 5);

    // The grayscale image from the previous line is altered with a binary threshold that forces all "gray" pixels with a brightness greater than 25 to become pure white (255) and all pixels with a brightness less than or equal to 25 to become pure black (0).
    cv::threshold(blur, thresh, 25, 255, cv::THRESH_BINARY);

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
    // Here this function returns a pair containing the contours list and custom output variables of the box_dim structs.
    std::pair<std::vector<std::vector<cv::Point>>, std::vector<BoxDim>> 
    contours(cv::Mat& frame, const cv::Mat& dilated) {
        

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
    };


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


    std::vector<target> scan(cv::Mat& frame) {

        // Call filter() function for passed in frame
        cv::Mat frame_dilated = filter(frame);

        // Call contours() to get contours_list as well as each box dimensions struct for each contour (variables must be extracted from here)
        auto [contours_list, box_dims] = contours(frame, frame_dilated); // note, in this case auto is easier than its equivalent: std::pair<std::vector<std::vector<cv::Point>>, std::vector<BoxDim>>

        std::vector<target> targets; // preallocate vector to store information for each target

        // Loop through all box dimensions for each different contour identified to extract x and y centroid position and size for input into each target instance
        for (const auto& box : box_dims) { 

            // define centroid position. Note that each default x and y position is at the top left corner of a given object. 
            // By adding width/2 and height/2 of the given bounding box to the x and y dimensions (situated in the upper left corner) 
            // we center each x and y position in the center/centroid of each box

            int x_centr_pos = box.x + (box.w / 2); 
            int y_centr_pos = box.y + (box.h / 2);

            // Construct new instance of target. Unassigned values default to std::nullopt as defined in the target class (this is basically the same as assigning to NONE equivalently in Python)
            target new_target(x_centr_pos, y_centr_pos, box.size);

            targets.push_back(new_target); // push_back works the same as the append function in Python and puts each new target at the end of the dynamically changing array/vector that targets is initialzied as
        }

        return targets;
    }
};




int main() {

    // Define your test image path
    // Note: Use forward slashes '/' or escaped backslashes '\\' in C++ paths
    std::string image_path = "C:/Users/Graeme/Downloads/Star_test_image.jpg"; // defines string that will be input to imread later

    // Initialize detector class
    detector my_detector;

    // Load in the test image
    cv::Mat frame = cv::imread(image_path);
    if (frame.empty()) {
        std::cout << "Error: Could not load image from " << image_path << std::endl;
        return -1;
    }

    // Create a copy of the frame so the original image is not altered completely
    cv::Mat display_frame = frame.clone(); // clone() handles deep copy in C++

    try { // try and catch blocks help to process errors. For all the code executed inside of the try blocks, if there is an error that occurs for any reason,
          // it will not throw an error to the console but will instead go to the catch blocks which will simply output that an error occured to the console.

        // Call the scan function to access the specific frame and compile the list of targets
        std::vector<target> targets = my_detector.scan(display_frame);

        // note that this outputs if the code has no errors indicated by the try and catch blocks so if this outputs it is by default successful
        std::cout << "--- Scan Successful! ---" << std::endl;
        std::cout << "Detected " << targets.size() << " target(s) matching size criteria." << std::endl; // outputs total number of targets detected

        // Loops through all indices (targets) of the target vector and outputs their center coordinates (x,y) and size in pixels
        for (size_t idx = 0; idx < targets.size(); ++idx) {
            std::cout << "Target " << idx 
                      << ": Center=(" << targets[idx].x << ", " << targets[idx].y << "), "
                      << "Size=" << targets[idx].size << "px" << std::endl;
        }

        // Creates matrix/image which is the new frame with all of the filtering methods up through dilation added -- this will then output to the window after resizing
        cv::Mat dilated_mask = my_detector.filter(frame);


        // NOTE! -- This section (lines 328-349) is only needed if the uploaded image's pixel size is greater than the pixel size of your computer screen.
        
        // Define target window vertical height
        int target_screen_height = 650;

        // Calculate aspect ratio (width / height)
        // Static casting to double prevents integer truncation during division
        double aspect_ratio = static_cast<double>(frame.cols) / static_cast<double>(frame.rows);

        // Calculate the new scaled width based on target height
        int target_screen_width = static_cast<int>(target_screen_height * aspect_ratio);

        // Window 1: Original Frame with Bounding Boxes
        std::string win_name_1 = "1. Original Frame with Bounding Boxes";
        cv::namedWindow(win_name_1, cv::WINDOW_NORMAL); 
        cv::resizeWindow(win_name_1, target_screen_width, target_screen_height);
        cv::imshow(win_name_1, display_frame);

        // Window 2: Dilated Mask
        std::string win_name_2 = "2. Dilated Mask (What the detector sees)";
        cv::namedWindow(win_name_2, cv::WINDOW_NORMAL);
        cv::resizeWindow(win_name_2, target_screen_width, target_screen_height);
        cv::imshow(win_name_2, dilated_mask);

    

        // outputs instructions to command window
        std::cout << "\nPress any key on the image windows to close and exit." << std::endl;

        // gets rid of all windows when instructions output above are followed.
        cv::waitKey(0);
        cv::destroyAllWindows();

    }
    catch (const std::exception& e) {
        std::cout << "An error occurred during testing: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}












