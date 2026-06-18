// Define All Necessary Libraries:

// C++ Standard Libraries:
#include <iostream> // standard library for input and output streams

#include <vector> // standard library for using the vector container --> Note that this enables
                  // us to make vectors which can be constantly overwritten to store new information/nodes
                  // as the built in camera takes more frames.

#include <chrono> // standard library for using time functions. For this application it can be used for both
                  // calculating the time taken to process each frame and display it on the camera feed as well
                  // as to record and list timestamps for each frame.

#define _USE_MATH_DEFINES // needed for VS code to use the PI constant by including the below library
#include <cmath> // to be able to use value of PI and the pow() function

#include <cassert> // to be able to use the assert function for testing functionality later in the code

// OpenCV Libraries:
//  // main OpenCV library which includes all following necessary functions and libraries for this application
#include <opencv2/opencv.hpp>      

// #include <map> // allows for mapping an ID to specific coordinates in the properties of the detector class
// #include <utility> // library that allows access for std::pair
// #include <optional> // allows for "optional" inputs into the target class as this program only acquired x,y, and size and other parameters are likely found in other associated code
#include "Target.hpp"
#include "Detector.hpp"
#include "Graph.hpp"


int main() {

    // Add temporary pause for debugging
std::cout << "PRESS ENTER TO START DEBUGGING..." << std::endl;
    std::cin.get(); // Halts the program temporarily

// Define your test image path
// Note: Use forward slashes '/' or escaped backslashes '\\' in C++ paths
// std::string image_path = "C:/Users/Graeme/Downloads/Star_test_image.jpg"; // defines string that will be input to imread later
std::string image_path = "testing/exact_test_image.png";


int square_side_length = 200;
int circle_radius = 50;


// Expected sizes of both rectangle objects detected for testing functionality using the "assert" function later in this code
// This is formatted for the moment as the pixel size for a square first and then a circle
std::vector<double> expected_sizes = {pow(square_side_length,2), M_PI*pow(circle_radius,2)};

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

try {
    // try and catch blocks help to process errors. For all the code executed inside of the try blocks, if there is an error that occurs for any reason,
    // it will not throw an error to the console but will instead go to the catch blocks which will simply output that an error occured to the console.

    // Call the scan function to access the specific frame and compile the list of targets
    std::vector<Target*> targets;
    my_detector.scan(display_frame, targets);

    // note that this outputs if the code has no errors indicated by the try and catch blocks so if this outputs it is by default successful
    std::cout << "--- Scan Successful! ---" << std::endl;
    std::cout << "Detected " << targets.size() << " target(s) matching size criteria." << std::endl; // outputs total number of targets detected

            // 1. ASSERT TARGET COUNT
        // Ensure we actually found the exact number of expected objects before looping
        assert(targets.size() == expected_sizes.size() && "Target count mismatch! Did not find the expected number of objects.");

    // Loops through all indices (targets) of the target vector and outputs their center coordinates (x,y) and size in pixels
    for (size_t idx = 0; idx < targets.size(); ++idx) {
        std::cout << "Target " << idx 
                    << ": Center=(" << targets[idx]->x << ", " << targets[idx]->y << "), "
                    << "Size=" << targets[idx]->size << "px" << std::endl;

                      // 2. ASSERT MEMORY & POINTER VALIDITY
            assert(targets[idx] != nullptr && "Target pointer is null!");

            // 3. ASSERT BOUNDARY CONDITIONS (Coordinates within frame bounds)
            assert(targets[idx]->x >= 0 && targets[idx]->x < frame.cols && "Target X coordinate is out of frame bounds!");
            assert(targets[idx]->y >= 0 && targets[idx]->y < frame.rows && "Target Y coordinate is out of frame bounds!");

            // 4. ASSERT REASONABLE REAL-WORLD METRICS
            assert(targets[idx]->size > 0.0 && "Target size cannot be zero or negative!");

            // 5. IMPROVED SIZE ASSERTION
            // In real image processing, contour sizes fluctuate slightly due to anti-aliasing/thresholding.
            // A 1.0-pixel tolerance can fail on real camera feeds. 5% tolerance is standard, but keeping 1.0 if using crisp synthetic images.
            double delta = std::abs(expected_sizes[idx] - targets[idx]->size);
            assert(delta < 1.0 && "Target size does not match expected pixel area!");
        }


    

    // Creates matrix/image which is the new frame with all of the filtering methods up through dilation added -- this will then output to the window after resizing
    cv::Mat dilated_mask = my_detector.filter(frame);

        assert(!dilated_mask.empty() && "Filter step returned an empty mask!");
        assert(dilated_mask.channels() == 1 && "Dilated mask should be a single-channel binary image!");



        //         // Clean up memory to avoid leaks (highly critical for video processing/looping)
        // for (auto target : targets) {
        //     delete target;
        // }
        // targets.clear();


    // NOTE! -- This section (lines 328-349) is only needed if the uploaded image's pixel size is greater than the pixel size of your computer screen.
        
    // Define target window vertical height
    // int target_screen_height = 650;

    // Calculate aspect ratio (width / height)
    // Static casting to double prevents integer truncation during division
    // double aspect_ratio = static_cast<double>(frame.cols) / static_cast<double>(frame.rows);

    // Calculate the new scaled width based on target height
    // int target_screen_width = static_cast<int>(target_screen_height * aspect_ratio);

    // Window 1: Original Frame with Bounding Boxes
    std::string win_name_1 = "1. Original Frame with Bounding Boxes";
    // cv::namedWindow(win_name_1, cv::WINDOW_NORMAL); 
    // cv::resizeWindow(win_name_1, target_screen_width, target_screen_height);
    // cv::imshow(win_name_1, display_frame);

    // Window 2: Dilated Mask
    std::string win_name_2 = "2. Dilated Mask (What the detector sees)";
    // cv::namedWindow(win_name_2, cv::WINDOW_NORMAL);
    // cv::resizeWindow(win_name_2, target_screen_width, target_screen_height);
    // cv::imshow(win_name_2, dilated_mask);

    

    // outputs instructions to command window
    std::cout << "\nPress any key on the image windows to close and exit." << std::endl;

    // gets rid of all windows when instructions output above are followed.
    // cv::waitKey(0);
    // cv::destroyAllWindows();


    // Find min and max size values of targets in the targets vector to be output to the console
    if (!targets.empty()) {
    // std::minmax_element finds both the min and max iterators in a single pass.
    // We pass a custom lambda function to compare the '.size' member of the Target pointers.
    auto [min_it, max_it] = std::minmax_element(targets.begin(), targets.end(),
        [](const Target* a, const Target* b) {
            return a->size < b->size;
        });

    // Dereference the iterators and access their size property
    std::cout << "Minimum detected size: " << (*min_it)->size << "px\n";
    std::cout << "Maximum detected size: " << (*max_it)->size << "px\n";
} else {
    std::cout << "No targets detected to calculate min/max sizes.\n";
}



// Classify and print objects of different size ranges (based on pixels), also print out total number of objects

// Count the number of objects of different sizes for the given picture -- this is determined arbitrarily for now and can be changed to any other values for if new data or ranges are determined.
// Initialize variables to hold each number of objects
int small_count = 0;   // 10 - 75 px
int medium_count = 0;  // 75 - 150 px
int large_count = 0;   // 150 - 500 px
int massive_count = 0; // 500+ px
int ignored_count = 0; // Under 10 px

for (const auto* tgt : targets) {
    if (tgt->size >= 10 && tgt->size < 75) {
        small_count++;
    } else if (tgt->size >= 75 && tgt->size < 150) {
        medium_count++;
    } else if (tgt->size >= 150 && tgt->size < 500) {
        large_count++;
    } else if (tgt->size >= 500) {
        massive_count++;
    } else {
        ignored_count++;
    }
}

// Total object count
size_t total_count = targets.size();

// Calculate percentages of object sizes compared to the total number of objects
// Static casting to double prevents integer division truncation
double percent_small = (small_count / static_cast<double>(total_count)) * 100.0;
double percent_medium = (medium_count / static_cast<double>(total_count)) * 100.0;
double percent_large = (large_count / static_cast<double>(total_count)) * 100.0;
double percent_massive = (massive_count / static_cast<double>(total_count)) * 100.0;

// Output different sizes of objects and percents of total to the console -- all percentages only with 2 digits after the decimal point
std::cout << "There are " << total_count << " total objects detected in this image\n";

// Set stream to fixed formatting with 2 decimal places
std::cout << std::fixed << std::setprecision(2);

std::cout << "There are " << small_count << " objects between 10 - 75 pixels and represents " << percent_small << " percent of the total objects.\n";
std::cout << "There are " << medium_count << " objects between 75 - 150 pixels and represents " << percent_medium << " percent of the total objects.\n";
std::cout << "There are " << large_count << " objects between 150 - 500 pixels and represents " << percent_large << " percent of the total objects.\n";
std::cout << "There are " << massive_count << " objects over 500 pixels and represents " << percent_massive << " percent of the total objects.\n";


    // **************************** start here after lunch
    // apply looping for targets to test functionality of the code
    for (size_t i = 0; i < targets.size(); i++) {
        double delta = std::abs(expected_sizes[i] - targets[i]->size);
        
        // Assert that the detected size is within 1 pixel of our expected size
        assert(delta < 1.0 && "Target size does not match expected pixel area!");





    
    }


// 3. FINAL CLEANUP: Safely deallocate heap pointers to guarantee 0% memory leaks
        for (auto* tgt : targets) {
            delete tgt;
        }
        targets.clear(); // Empty the vector tracking addresses
        

    }
    catch (const std::exception& e) {
        std::cout << "An error occurred during testing: " << e.what() << std::endl;
        return -1;
    }

return 0;

}












