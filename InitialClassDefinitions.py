# Definition of Necessary Libraries:

import cv2 # Library to access all necessary OpenCV functions
import numpy as np # Library to access all necessary NumPy functions
import time  # Library to access all necessary time functions




# Initial Class Definitions and flow of code

# Parent Class -- Sentry --> Can use super() function if any future classes need to inherit any properties or member functions defined
class sentry:

    # Definition of relevant properties:

    # **** I don't know if there are different properties and so I just put this as a placeholder
    def __init__(self, frame, target):
           self.frame
           self.target

    
 # Member Functions:

 # 1.) targetList()
 # Function Summary: 
 # list of pointers to Linked Lists of targets (note: linked lists in Python do not technically 
 # exist and thus must be created in its own class to have the desired functionality) 

 # Inputs:
 # TBD

 # Outputs:
 # TBD

# *** Returns 0 for now because we haven't written this yet (remove this line when function is written)
    def targetList(self):
        return 0


# 2.) nextFrame()
# Function Summary:
# Returns true if a non-star target is found.

# Inputs:
# TBD

# Outputs:
# TBD

# *** Returns 0 for now because we haven't written this yet (remove this line when function is written)
    def nextFrame(self, frame): # I know that these inputs are incorrect currently but I don't know what they need to be
        return 0
    

# 3.) 
    







     











# Class -- Target
class target:

    # Definition of relevant properties:
    # 1.) x  = x position of target in previous camera frame 
    # 2.) y  = y position of  target in previous camera frame  
    # 3.) size = size in amount of pixels of the detected object 
    # 4.) ID = each object in every frame will have an object ID which will be tracked and noted across frames. 
    # 5.) nx = new x position of target in the next camera frame 
    # 6.) ny = new y position of target in the next camera frame 

    def __init__(self, x, y, size, ID, nx, ny, isStar):
        self.x = x
        self.y = y
        self.size = size
        self.ID = ID
        self.nx = nx
        self.ny = ny
        self.isStar = isStar

 # Member functions:

# 1.) getPos()
# Function Summary: 
# Return x and y position of any given instance of the target class (for the previous frame) 

# Inputs: 
# None 

# Outputs: 
# 1.) x = x position of a given target from previous frame. 
# 2.) y = y position of a given target from previous frame. 
# Currently outputs x and y as a 2D point

    def getPos(self):
        return self.x, self.y
    

# 2.) setPos()
# Function Summary: 
# Sets/stores x and y positions for the previous frame (x and y) -- possibly based around the centroid pixels (I think this is still in deciding if this is the best option). 

 
# Inputs: 
# None 

# Outputs: 
# TBD 

# *** Returns 0 for now because we haven't written this yet (remove this line when function is written)
    def setPos(self):
        return 0 
    

# 3.) getNextPos()
# Function Summary: 
# Return nx and ny position of any given instance of the target class (for the previous frame) 

# Inputs: 
# None 

# Outputs: 
# 1.) nx = x position of a given target from the next frame. 
# 2.) ny = y position of a given target from the next frame. 
# Currently outputs nx and ny as a 2D point
        
    def getNextPos(self):
         return self.nx, self.ny
        
# 4.) setNextPos()
# Function Summary: 
# Sets/stores nx and ny positions for the next frame -- possibly based around the centroid pixels  

 
# Inputs: 
# None 

#Outputs: 
# TBD 

# *** Returns 0 for now because we haven't written this yet (remove this line when function is written)
    def setNextPos(self):
        return 0
    

# 5.) setNextPos()
# Function Summary: 
# Sets/stores the ID assigned to any instance of the target class. 

# Inputs: 
# TBD 

# Outputs: 
# TBD 

# *** Returns 0 for now because we haven't written this yet (remove this line when function is written)
    def setID(self):
        return 0
    

# 6.) getSize()
# Function Summary: 
# Returns the size of any instance of the target class (in pixels?)  

# Inputs: 
# TBD 

# Outputs: 
# TBD 

    def getSize(self):
        return self.size
    
# 7.) setSize()
# Function Summary: 
# Sets/stores the size of any instance of the target class (in pixels?) 

# Inputs: 
# TBD 

# Outputs: 
# TBD 
    
# *** Returns 0 for now because we haven't written this yet (remove this line when function is written)
    def setSize(self):
        return 0
    



    
