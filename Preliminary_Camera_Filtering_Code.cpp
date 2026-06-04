/*

//  Code function summary:
// 1.) Activate Video capture with cv2 library. Note that this overall camera capture has green squares showing overall bulk motion
// 2.) Create Forground-Background (fgbg) view by using input function CreateBackgroundSubtractorMOG2() which creates a camera view
//  which creates foreground or moving objects by detecting motion so moving or foreground objects are in white and background obkects
//  are displayed in black.
// 3.) Apply Blur on all frames to decrease noise and make it easier to detect motion. This is done by using the medianBlur function 
//  which takes in the frame and a kernel size of 5.
// 4.) Add looping statement to create overall contours of motion by using the findContours function which takes in the dilated image 
// and retrieves only the external contours and uses a simple approximation method to save memory. After this, use the looping statement
// to filter out contours which are too small on some arbitrarily defined threshold.
// 5.) Other steps and details TBD


// General effect summary from 1st image to the final image:
// 1st image shows broad squares which just demonstrate general movement, and the 
// final contours plot has applied all the previous commented changes to track all
//  the different contours detected in the camera.
// Note: this code does not detect motion but instead redraws contours each time there is motion detected.
//  So, note that there is no memory for any previous motion but it simply recalculates contour at each time.

// Author(s): Converted/Reformatted by Graeme Appel

// Last Revised: 6/01/2026

*/

// Define All Necessary Libraries:

// C++ Standard Libraries:
#include <iostream> // standard library for input and output streams

#include <vector> // standard library for using the vector container --> Note that this enables
                  //us to make vectors which can be constantly overwritten to store new information/nodes
                  // as the built in camera takes more frames.

#include <chrono> // standard library for using time functions. For this application it can be used for both
                  // calculating the time taken to process each frame and display it on the camera feed as well
                  // as to record and list timestamps for each frame.

// OpenCV Libraries:
//  // main OpenCV library which includes all following necessary functions and libraries for this application
#include <opencv2/opencv.hpp>                                 // Notably, for this application, this enables: cv::Mat, cv::VideoCapture, cv::createBackgroundSubtractorMOG2, cv::medianBlur,  and cv::findContours


// Define classes and user defined functions for use in the main function of the code:

// 1.) Define a Node class to keep track of object position in x and y as well as when that position occurs in time. This can be used later to compute velocity and acceleration and to track any given object
class Node {

  // Fields defined: 
  // 1.) x is an exact integer number of the exact 2D pixel coordinates in the frame of the computer's camera along the x-axis
  // 2.) y is an exact integer number of the exact 2D pixel coordinates in the frame of the computer's camera along the y-axis
  // 3.) t is a decimal number of the exact time in seconds that the object is at the given x and y coordinates. Measured from t = 0 when the camera starts recording.

  // Note that generally, (0,0) is the top left corner of the camera frame and so (x_max, y_max) for any given camera specs is the bottom right corner 
 
public: // Means this can be used in all of main code, not just within the class definition
    int x; // x coordinate of any given object measured in [# of pixels]
    int y; // y coordinate of any given object measured in [# of pixels]
    double t; // Time stamp in [decimal seconds]

    Node(int nx, int ny, double nt) : x(nx), y(ny), t(nt) {} // outlines syntax for how to create a node object with the previously provided fields.
    // Note that syntax x = nx is just a syntax where nx is "new x" so that the computer doesn't get confused between the field name x and the input of x position when making the object. This is the same for y and t
};

// User defined functions:
//  1.) Helper function to grab current system time in decimal seconds (equivalent to Python's time.time())
double getCurrentTimeSeconds() {

  // Purpose: Determine the current time of the system in decimal seconds. This can be used to timestamp each frame and to calculate the time taken to process each frame.
  // This value is then input into each node class to help the computer track the time of each new object position.

  // Inputs:
  //None
  // Outputs: 
  // 1.) A decimal number of the current time in seconds (used for each specific node). Measured from t = 0 when the camera starts recording.

  // Note that this is equivalent to python's time.time() function which returns current time 1/1/1970. This is useful because most all other telemetry and data systems are also synced to this time point.


    auto now = std::chrono::system_clock::now().time_since_epoch(); // uses standard library (std) and then time tracking library (chrono) to link to the computer's clock to get time since 1/1/1970 using time_since_epoch() function.
                                                                    // This then assigns that value to an object called now who's type is determined automatically by the key word auto. This returns object with units of [nanoseconds].

    return std::chrono::duration_cast<std::chrono::duration<double>>(now).count(); // takes previous "now" object and converts it to a decimal number of seconds by using the duration_cast function to convert from nanoseconds to seconds
                                                                                   // and then using the count() function to get the value of that duration in seconds. This is then returned as the output of the function.
}


// Main function of the code:
int main()


{
  

    // A.) Initialize Video Capture (0 = Default Webcam) and video capture object cap.
    cv::VideoCapture cap(0); // starts video capture using default webcam by using cv (computer vision libary) and then VideoCapture function which takes in the index of the camera to use (0 for default webcam).
                             //This creates an object called cap which can be used to access the video feed from the webcam.

    // Note that the VideoCapture function is an OpenCV function which creates an object that is called cap in this case.
    // Cap has 2 fields .get and .release.
    // Field 1.) .get can be used to get the properties of the video feed such as frame width and height which can be used to set the dimensions of the recording canvas and to make sure that the recording canvas is the same size as the video feed.
    // Field 2.) .release can be used to release the video feed and to free up the webcam for other applications when the program is done running.

    if (!cap.isOpened()) { // throws error to let you know if camera cap isn't open i.e if webcam is not accessible.
        std::cerr << "Error: Could not open the webcam stream." << std::endl;
        return -1; // exits the program with an error code of -1 if the webcam stream cannot be opened.
    }

    // B.) Find and store dimensions of the system's camera feed in exact integer values of pixels.
    // recall that cap is the object creates at the start of the main code which is linked to the webcam feed and so we can use the get function to get the properties of that feed. 
    // The field .get paired with the OpenCV function CAP_PROP_FRAME_WIDTH and CAP_PROP_FRAME_HEIGHT are used to get the width and height of the video feed in pixels. However, this gives a specific value in decimal.
    // static_cast<int> is used to convert that decimal value to an integer value which is necessary for the dimensions of the recording canvas and for other functions that require integer values for dimensions.
    // This is then stored in the variables frame_W and frame_H for later use.
    int frame_W = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int frame_H = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    // C.) Create Video Recording Structures

    // uses OpenCV VideoWriter function to create a video recording object called out_mp4 which will save the recorded video as "newCap.mp4" in the same directory as the code.
    // The fourcc function is used to specify the codec (coder-decoder method used) to put the video recording in usable form. In this case 'mp4v' is the codec which is a common codec for mp4 files.

    // Inputs:
    // 1.) "newCap.mp4" is the name of the output video file. This can be changed to any name you want as long as it ends with .mp4 to ensure it is saved as an mp4 file.
    // 2.) fourcc is the 4-character code of the codec used to compress the frames. For example, VideoWriter::fourcc('P','I','M','1') is a MPEG-1 codec, VideoWriter::fourcc('M','J','P','G') is a motion-jpeg codec etc.
    //  List of codes can be obtained at https://docs.microsoft.com/en-us/windows/win32/medfound/video-fourccs page or with this https://fourcc.org/codecs.php page of the fourcc site for a more complete list). 
    // FFMPEG backend with MP4 container natively uses other values as fourcc code: see ObjectType http://mp4ra.org/#/codecs, so you may receive a warning message from OpenCV about fourcc code conversion.
    // 3.) 30 is the frame rate of the created video stream. This can be changed to any value you want but 30 is a common frame rate for videos and is a good starting point.
    // 4.) Size(frame_W, frame_H) is the size of the video frames. This is set to the same dimensions as the video feed from the webcam to ensure that the recording canvas is the same size as the video feed and uses the stored dimensions from the previous step.

    // Outputs: 
    // 1.) A video recording object called out_mp4 which can be used to write frames to the video file "newCap.mp4" as the program runs. 
    //This object has a .write function which can be used to write frames to the video file and a .release function which can be used to release the video file when the program is done running.

    cv::VideoWriter out_mp4("newCap.mp4", 
                            cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 
                            30, 
                            cv::Size(frame_W, frame_H));
    
    bool recording = false; // initializes the recording to "false" meaning that the recording is not on right now but is activated later in the program.


    // D.) Create Memory structure to store and track data

    // Creates traceImg which is a cv::Mat object which is a matrix that can be used to store and manipulate images.
    // In this case, it is initialized as a blank canvas of the same size as the video feed (frame_W x frame_H). 
    // CV_8UC3 is a configuration inside OpenCV which denotes that each pixel is an 8 bit unsigned integer (0-255) and has 3 color inputs for RGB.
    // Note that by inputing it into zeros (which creates a matrix full of entirely zeros), this initalizes all RGB values to (0,0,0) which starts the memory with an entirely blank canvas/feed.
    // This will be used to draw the trajectory of the moving objects by plotting points on this canvas as they move.
    cv::Mat traceImg = cv::Mat::zeros(frame_H, frame_W, CV_8UC3);

    // Creates nodeArray which is a vector full of Node objects as defined previously. This will be used to store the x and y coordinates and timestamps of the moving objects as their motion is detected in each frame. 
    std::vector<Node> nodeArray;

    // E. Create Background Subtractor Object
    // cv::Ptr <___> is a smart pointer in OpenCV which is used to manage the memory of the created object. It automatically handles the deallocation of memory when the object is no longer needed (i.e) it deletes the objects stored in memory when the program finishes.
    // fgbg = cv:: ... uses an OpenCV function called createBackgroundSubtractorMOG2() (Mixture of Gaussians 2) that takes into account many past frames (~500) to make a probability model for each pixel.
    // If a pixel's color changes drastically due to movement, this is then identified as part of the foreground and distinguished from the parts with very little motion which are then deemed as the background.
    cv::Ptr<cv::BackgroundSubtractorMOG2> fgbg = cv::createBackgroundSubtractorMOG2();

    // Makes a matrix/image output in 7 separate stages/objects which track the evolution of the image overtime through a "pipeline" of image processing steps.  These steps/stages are all used in the code later.
    // Stages:
    // 1.) frame == the raw video feed from the webcam which is captured in each loop iteration 
    // 2.) blank == a blank canvas which is used to draw the bounding boxes of the moving objects when recording is toggled on. 
    // This is initialized as a blank canvas in each loop iteration to ensure that it only shows the current frame's bounding boxes and not any previous frames' bounding boxes.
    // 3.) blur == the blurred version of the raw video feed which is used to reduce noise and make it easier to detect motion. This is created by applying a median blur to the raw frame.
    // 4.) fgmask == the foreground mask which is created by applying the background subtractor to the blurred frame. This shows the moving objects in white and the background in black.
    // 5.) thresh == the thresholded version of the foreground mask which is created by applying a binary threshold to the foreground mask. 
    // An arbitrary threshold helps forces all pixels to be either white (255) or black (0), which forces the foreground to be either completely white or completely black which makes it easier to detect contours.
    // 6.) dilated == the dilated version of the thresholded image which is created by applying a dilation operation to the thresholded image. This helps to bridge any gaps in the contours by expanding the white pixels of the moving objects which makes it easier to detect contours.
    // 7.) cont_frame == the frame with contours drawn on it which is created by cloning the raw frame and then drawing the contours on the cloned frame. This is used to display the contours of the moving objects in a separate window.
    cv::Mat frame, blank, blur, fgmask, thresh, dilated, cont_frame;

    // Tells monitor about keys to use (which are later defined in the code).
    std::cout << "Streaming active. Press 'r' to toggle tracking record, 'q' to quit." << std::endl;



    while (true) { // creates infinite loop (i.e continuously running camera feed) which executes commands while recording runs.

        cap >> frame; // uses extractor operator (>>) to read the next frame from the video feed and store it in the variable "frame" as defined above. This is done in each loop iteration to get the current frame from the webcam feed.
        if (frame.empty()) break; // if frame is empty this means there is an issue with the video feed (i.e webcam is not accessible) and so the loop breaks and the program ends.

        // Uses zeris matrix to define an entirely black canvas which is stored in the variable "blank" as defined above. This is done in each loop iteration to reset the blank canvas for each new frame.

        // F.) Image Preprocessing Pipeline
        cv::medianBlur(frame, blur, 5);           // Applies blur using medianBlur OpenCV function to reduce camera high-frequency noise 
        fgbg->apply(blur, fgmask);                 // Passes the new blur object/frame over the fbgb and compares newest blurred frame to history of past frames to create foreground mask (fgmask) which shows moving objects in white and background in black as a grayscale image.

        // The grayscale image from the previous line is altered with a binary threshold that forces all "gray" pixels with a brightness greater than 25 to become pure white (255) and all pixels with a brightness less than or equal to 25 to become pure black (0).
        cv::threshold(fgmask, thresh, 25, 255, cv::THRESH_BINARY); 

        // Expands white pixels in the previous thresholded image to bridge small gaps in moving white in the foreground that could be detected as objects. This helps to distinguish objects as 1 full object rather than mistakenly identifying them as many tiny objects
        cv::dilate(thresh, dilated, cv::Mat(), cv::Point(-1, -1), 2); // Bridge structural gaps between pixels

        // G.) Compute Structural Coordinates (Contours)
        std::vector<std::vector<cv::Point>> contours; // cv::Point represents a point in 2D with x and y coordinates. This is put into a vector to represent a contour which is a collection of points that form the outline of a moving object. This is then put into another vector to represent all the contours in the frame.
                                                      // i.e each contour is a vector of points (x,y) and each contour is put into a vector to keep track of each contour/moving object.
        std::vector<cv::Vec4i> hierarchy; //cv::Vec4i --> This stands for a Vector of 4 Integers. For every contour found, OpenCV stores 4 specific numbers tracking its relationships: [Next contour at the same level, Previous contour at the same level, First Child contour inside it, Parent contour outside it]. 
                                          // hierarchy: This master list pairs up pixel-for-pixel with your contours vector, acting as a structural family tree so the program knows which shapes are holes inside other shapes.

        cv::findContours(dilated, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        // Finds necessary contours by searching all inputs to define specific boundaries for contours for clarity and for saving memory in the computer.
        // 1.) Dilated input is the source image where we look for contours and this function searches for borders between white and black pixels.
        // 2.) contours is the output vector of contours. Each contour is stored as a vector of points (x,y) which are the coordinates of the contour. This is used to draw the contours and to calculate the bounding boxes of the moving objects.
        // 3.) hierarchy is the output vector of the hierarchy of contours. This is used to keep track of the relationships between contours (i.e which contours are inside other contours) and to filter out contours that are not relevant (i.e small contours that are likely just noise).
        // Note that contours and hierarchy are both empty variables that are filled by the findContours function based on the input image and the parameters for contour retrieval and approximation.
        // 4.) cv::RETR_EXTERNAL is a flag that tells the function to retrieve only the external contours. This means that it will only retrieve the contours that are not inside any other contours. I.e only tracks the outline of a moving object, not all the internally moving parts.
        // 5.) cv::CHAIN_APPROX_SIMPLE is a flag that tells the function to compress horizontal, vertical, and diagonal segments and leave only their end points. This saves memory by reducing the number of points stored for each contour while still keeping the overall shape of the contour.

        // H.) Iterate and filter motion contours
        for (size_t i = 0; i < contours.size(); i++) { // outer loop loops through each specific contour in the contours vector to apply the following steps to each contour.
            if (cv::contourArea(contours[i]) < 500) { // inner loop loops through each point in every specific contour to apply the following steps. contourArea filters out contours that are too small (i.e noise) by  calculating the area of each contour and comparing it to an arbitrary threshold (in this case 500). If the area of the contour is less than the threshold, it is skipped and not processed further.
                continue; // Skip small ambient noise modifications
            }

            // Extract bounding coordinate geometry
            // converts very complex outlines of contours into simply boxes used to track the objects. This is done by using the boundingRect function which takes in a contour and returns the smallest rectangle that can enclose that contour. 
            // This is then stored in the variable boundingBox which is a cv::Rect object that contains the x and y coordinates of the top left corner of the rectangle, as well as the width and height of the rectangle.
            cv::Rect boundingBox = cv::boundingRect(contours[i]);
            
            // Draw tracking indicator boxes directly onto live frame
            // Takes in "frame" (raw video feed) and the bounding box specs from previous line to draw rectangles directly on the raw video feed. In this case (0,255,0) is green. 
            cv::rectangle(frame, boundingBox, cv::Scalar(0, 255, 0), 2); // line thickness 2.

            if (recording) { // only activates if recording is toggled on. This is initiated by pressing "r".

              // The boundingBox coordinates given above are measured in relation to (0,0) which is at the top left corner of each frame of each measured object.
              // If an object moves or expands, this can ruin the results and coordinates being tracked. Thus, these 2 lines find each boundingBox/object's center point by taking 
              //the x and y coordinates of the top left corner of the bounding box and adding half of the width and height of the bounding box respectively to get the center coordinates of the object. 
              // This is then stored to be used later.
                int center_x = boundingBox.x + boundingBox.width / 2;
                int center_y = boundingBox.y + boundingBox.height / 2;

                // Log dot path to track trajectory historical path map
                // This takes the center of each coordinate of a detecting object and plots it as a red dot on the traceImg canvas (which does not overwrite itself and thus stacks up over time to track object movement). 
                // Tthe circle function takes in the traceImg canvas, the center coordinates of the object, a radius of 1 (to make it a dot), a color of (0,0,255) which is red, and a thickness of -1 which fills in the circle to make it a solid dot.
                cv::circle(traceImg, cv::Point(center_x, center_y), 1, cv::Scalar(0, 0, 255), -1);

                // Stores the actual precise coordinate and timestamp data for each detected object in the nodeArray vector as a Node object with the center coordinates and the current time in seconds.
                // This is used to keep track of the exact coordinates and timestamps of each detected object for later use in calculating velocity, acceleration, or other needed parameters.
                nodeArray.push_back(Node(center_x, center_y, getCurrentTimeSeconds()));

                // Takes previous green box contours determined from before and draws them on the blank (fully black) canvas so that the movement of the objects can be specifically seen without the camera feed.
                cv::rectangle(blank, boundingBox, cv::Scalar(0, 255, 0), 2);
            }
        }

        // Record tracking canvas frame if toggled active
        if (recording) { // if recording is off, camera feed continues but no info is saved. 
                         // If recording is on, the following line saves the current blank canvas with the bounding boxes of the moving objects to the video file "newCap.mp4" using the out_mp4 video writer object created at the start of the code.
            out_mp4.write(blank);
        }

        // Clone current frame safely to render standard contour overlays separately
        cont_frame = frame.clone(); // creates clone of current frame to draw contours on to display in a separate spot without altering original data/frame.

        // Draws exact contours on the cloned frame using the contours vector obtained from the findContours function. This is used to display the contours of the moving objects in a separate window without altering the original frame.
        // Instead of just drawing green boxes, this function reads in contours from findContours, and uses drawContours to draw the exact contours of the moving objects on the cloned frame. 
        // This is done by taking in the cloned frame, the contours vector, -1 to indicate that all contours should be drawn, a color of (0,255,0) which is green, and a thickness of 3 to make the contours more visible.
        cv::drawContours(cont_frame, contours, -1, cv::Scalar(0, 255, 0), 3);

        // I.) Render Multi-Window Display Layouts
        // imshow creates each window and displays and names which window/stage is being displayed.
        cv::imshow("Motion Detection", frame);
        cv::imshow("Foreground Mask", fgmask);
        cv::imshow("Threshold Filter", thresh);
        cv::imshow("Dilation", dilated);
        cv::imshow("Contours", cont_frame);

        // J.) Capture Interactive Keyboard Intercepts
        // sets commands to start and stop the recording using 'r' and 'q' to quit the program. This is done by using the waitKey function which waits for a key press for a specified amount of time (in this case 30 milliseconds) and returns the ASCII value of the key pressed.
        int waitKey = cv::waitKey(30) & 0xFF;
        if (waitKey == 'q') {
            break;
        } else if (waitKey == 'r') {
            recording = !recording;
            std::cout << "Recording Status Altered: " << (recording ? "RECORDING ACTIVE" : "STOPPED") << std::endl;
        }
    }

    // Clean up hardware handles and release assets safely
    cap.release(); // disconnects camera feed at end of code.
    out_mp4.release(); // disconnects video writer and saves final file.
    cv::destroyAllWindows(); // closes all windows and stops logging memory when program is ended by pressing 'q'.

    return 0;
}
