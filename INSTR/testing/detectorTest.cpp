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
#include "Target.hpp"
#include "Detector.hpp"

int main() {

    // Define your test image path
    // Note: Use forward slashes '/' or escaped backslashes '\\' in C++ paths
    std::string image_path = "C:/Users/Graeme/Downloads/Star_test_image.jpg"; // defines string that will be input to imread later

    // Initialize detector class
    Detector my_detector;

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
        std::vector<Target*> targets = {};
        my_detector.scan(display_frame, targets);

        // note that this outputs if the code has no errors indicated by the try and catch blocks so if this outputs it is by default successful
        std::cout << "--- Scan Successful! ---" << std::endl;
        std::cout << "Detected " << targets.size() << " target(s) matching size criteria." << std::endl; // outputs total number of targets detected

        // Loops through all indices (targets) of the target vector and outputs their center coordinates (x,y) and size in pixels
        for (size_t idx = 0; idx < targets.size(); ++idx) {
            std::cout << "Target " << idx 
                      << ": Center=(" << targets[idx]->x << ", " << targets[idx]->y << "), "
                      << "Size=" << targets[idx]->size << "px" << std::endl;
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

