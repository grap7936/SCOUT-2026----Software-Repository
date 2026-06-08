###############################################################
# Code Summary:
# Takes in a singular image and using member functions this class (and associated functions) helps to find targets in distinct phases.
# 1.) filter() --> uses OpenCV pre-processing functions including operations: blur, background subtraction, thresholding, dilating,
#  and contour framing to prepare any given frame for extracting and identifying contours of moving regions/objects in the next step.
# 2.) contours() --> uses either OpenCV function findContours() or other researched method to create a framework/array of different contours
#  and to create arrays of points and then arrays of contours to help organize original visualization of regions of movement and objects being tracked.
# 3.) scan() --> Note: this will be the "parent" member function of both filter() and contours() and these will run inside of the scan function.
#  The other functionality of this function besides using the processes defined above for filter() and contours() is creating the memory allocation 
# for for each instance of the target class that is passed in and identified in each image. Additionally, for each contour make a target and populate
#  the entirety of the target properties (i.e x,y, size, ID, nx, ny e.t.c). 

# Author: Graeme Appel

# Last Updated: 6/8/2026

###############################################################

# Necessary Libraries

import cv2 # provides access to all OpenCV functions on the computer (provided it is downloaded)
import numpy as np # provides all embedded numerical functions
import time # provides all embedded time returning and manipulation functions



###############################################################
# Start of Class and UDF definitions/Main Code

class target:

    # Definition of relevant properties:
    # 1.) x = x position of target in previous camera frame 
    # 2.) y = y position of  target in previous camera frame  
    # 3.) size = size in amount of pixels of the detected object 
    # 4.) ID = each object in every frame will have an object ID which will be tracked and noted across frames. 
    # 5.) nx = new x position of target in the next camera frame 
    # 6.) ny = new y position of target in the next camera frame 
    # 7.) isStar = float between 0.0 and 1.0 where the closer to 1.0 the more likely it is a star

    def __init__(self, x, y, size, ID, nx, ny, isStar):
        self.x = x
        self.y = y
        self.size = size
        self.ID = ID
        self.nx = nx
        self.ny = ny
        self.isStar = isStar

###############################################################





class detector:

    # Class Properties:
    # 1.) next_object_ID = counter variable to keep track of different object ID's later in the scan() function
    # 2.) tracked_objects_centr = stores center point of the object to be able to assign it a given object ID -- { object_id: (x_center, y_center) }

    def __init__(self):
        self.next_object_ID = 0 
        self.tracked_objects_centr = {} # curly brackets denote an empty dictionary of the form: { object_id: (x_center, y_center) }


# Member Functions:

# filter() -- Filter function will later be used in the Scan function so it is placed here first -- explained above

# Function summary: at top of code

# Inputs: 
# 1.) frame = "newest" frame of the camera view for tracking

# Outputs:
# 1.) dilated = fully filtered image that is now blurred, black and white, with removed/thresholded pixels that are expanded to most fully identify objects

    def filter(self, frame): # 1st of the inner functions as part of the overall scan function -- passes in 2 frames to get the visual difference for filtering
            
            # Frames are directly passed in so we can begin by applying background subtraction
            ### for access if you need to do 2 frames # frame_difference = cv2.absdiff(frame_1, frame_2) # absdiff function acts as a background subtractor simply with 2 images not a videostream 
            fg_mask = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # foregound mask that converts the background subtractor image to a non-colored background 

            # Apply filtering methods
            blur = cv2.medianBlur(fg_mask, 5) # Applies blur using medianBlur OpenCV function to reduce camera high-frequency noise 
            _, thresh = cv2.threshold(blur, 25, 255, cv2.THRESH_BINARY) # The grayscale image from the previous line is altered with a binary threshold that forces all "gray" pixels with a brightness greater than 25 to become pure white (255) and all pixels with a brightness less than or equal to 25 to become pure black (0).
            dilated = cv2.dilate(thresh, None, iterations=2) # Expands white pixels in the previous thresholded image to bridge small gaps in moving white in the foreground that could be detected as objects. This helps to distinguish objects as 1 full object rather than mistakenly identifying them as many tiny objects

            # Return all different filtered frames if needed for use later. (Note that for this application, likely the final dilated frame is the most filtered and most useful)
            # returns final filtered, dilated frame for use in contour member function
            return dilated
    
    # contours() -- contours function will later be used in the Scan function so it is placed here first -- explained above

    # Function summary: at top of code

    # Inputs: 
    # 1.) frame = "newest" frame of the camera view for tracking
    # 2.) dilated = fully filtered image that is now blurred, black and white, with removed/thresholded pixels that are expanded to most fully identify objects

    # Outputs: 
    # 1.) contours = final array/vector of contours which are bounded, identified areas that could be objects in the filtered frames
    # 2.) box_dim = list of box dimensions for each contour formed in the contour function such as x and y position and


    def contours(self, frame, dilated):   # passes in dilated frame as well as previous 2 stored frames from the system to begin forming contours

        contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # defines contours 
        # RETR_EXTERNAL -- external retrieval. This addition means that pixels only on the outside of objects are detected but not any holes or other things inside of full objects
        # CHAIN_APPROX_SIMPLE -- stores coordinate points to remember the shape of each contour boundary. In this case, the APPROX means it does not store every pixel but intead removes all redundant data points along straight lines

        # create empty list for box dimensions (box_dim) to be able to store above variables (x, y, w, h)
        box_dim = []



        ####### NOTE! WE might add in more parameters to filter which objects are put into the contours array -- for example, for right now it is only sifting through things based off of size constraints
        # Note for the contours array, it stores a contour implicitly as each element in the array which is what has the box and information pertaining to each detected "object" in a given frame. Thus, contours is an array of different detected objects
        for contour in contours:

            size = cv2.contourArea(contour) # finds size of each box using contour area of each specific contour. Output is an int or float in square pixels

            if size < 1000: # sets parameter for size to see where contours are made
                (x, y, w, h) = cv2.boundingRect(contour) # creates dimensions for rectangle to bound object in frame 

                # for this code, x = specific x coordinate of every given pixel input; y = specific y coordinate of every given pixel input; w = width of bounding rectangle; h = height of bounding rectangle
                # Input (x,y) sets the top left corner of the frame at given x, y where both are equal to 0.
                # Input (x + w, y + h) sets the bottom right corner of the box by adding the total width and height of the object/bounding box frame
                # Input (0, 255, 0) -- box color is green
                # Input 2 = linewidth = 2 pixels thick
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # defines bounded rectangle
                
                # Stores all elements of each rectangle x, y, w, h, and size into a dynamic list called box_dim which can be access later when finding necessary variables for the target class in the scan() function
                # append simply adds each new contour/"object" ot the end of this list so all information for object is stored by index 0 to total # of objects
                box_dim.append( (x, y, w, h, size) )


        return contours, box_dim # returns contours to best identify bounding boxes and identified area that could be objects
            

# scan() -- "parent" function of the previous two member functions

# Function summary: filter() and contours() both run for each set of two frames but then immediately forget all the information
#  passed and calculated so we must set up a memory system to track and store objects and assign dimensions pertaining to the target 
# class such as x and y position of "past frame"/frame_1, object size, object ID, and new x and y position of the "new frame"/frame_2.

# Inputs:
# 1.) frame = "newest" frame of the camera view for tracking

# Outputs: 
# 1.) targets = list of targets detected in each 


    def scan(self, frame):
          
        # Call filter()
        frame_dilated = self.filter(frame)

        # Call contours() to get contours and box_dimensions of the image
        contours, box_dim = self.contours(frame, frame_dilated)

        # Define list of target objects that will be filled with necessary parameters later
        targets = []

        # Loop through all box dimensions to extract needed values to input into the target class
        for (x, y, w, h, size) in box_dim:
            x_centr_pos = int(x + w/2) # apply basic centroiding to find centroid coordinates of each detected "object"
            y_centr_pos = int(y + h/2)

            # Create each new instance of the target class putting in the necessary parameters of centroided x and y position and size
            # Note: All parameters that say NONE are either determined in other code or may be future improvements or iterations to this code
            new_target = target(
                    x = x_centr_pos, 
                    y= y_centr_pos, 
                    size = size, 
                    ID = None, 
                    nx = None, 
                    ny = None, 
                    isStar = None
                )
            
            targets.append(new_target) # add each new target to the before defined list of targets 

        return targets
            



        










