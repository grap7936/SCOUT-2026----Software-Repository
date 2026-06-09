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
    Detector();

// Member functions (same as described at the top of the code)

    cv::Mat filter(const cv::Mat& frame);

    std::pair<std::vector<std::vector<cv::Point>>, std::vector<BoxDim>> contours(cv::Mat& frame, const cv::Mat& dilated);

    void scan(cv::Mat& frame, std::vector<Target*>& targets);
};
