
###############################################################
# Code Summary:
# This test code loads in an image of the tester's choosing and applies the detector class to output windows which show both the fully filtered (dilated) image and then create another window with all of the bounding boxes applied.
# Additionally, the total number of objects, and objects of various pixel sizes are output in terms of their total number and percent of the entire number of objects detected.

# Author(s): Originally formulated using Gemini but edited and altered by Graeme Appel

# Last Edited: 6/9/2026


# Including Necessary file names
import cv2
from DetectorClass import detector  # Reading in detector class 

def test_on_single_image(image_path):

    # 1. Initialize your detector class
    my_detector = detector()

    # 2. Load a test image
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Error: Could not load image from {image_path}")
        return

    # 3. Create a copy so we don't permanently alter the original
    display_frame = frame.copy()

    try:
        # 4. Run the parent scan function
        targets = my_detector.scan(display_frame)
        
        print(f"--- Scan Successful! ---")
        print(f"Detected {len(targets)} target(s) matching size criteria.")
        
        # 5. Print out text results to verify object instantiation -- uncomment if wanting to see specific target data, otherwise this is saved to the console.
        # for idx, tgt in enumerate(targets):
        #     print(f"Target {idx}: Center=({tgt.x}, {tgt.y}), Size={tgt.size}px")


        # Print max and minimum sizes of all objects detected

        if targets:
            sizes = [tgt.size for tgt in targets]
            min_size = min(sizes)
            max_size = max(sizes)
            print(f"Minimum detected size: {min_size}px")
            print(f"Maximum detected size: {max_size}px")
        else:
            print("No targets detected to calculate min/max sizes.")



        # Classify and print objects of different size ranges (based on pixels), also print out total number of objects

        # Count the number of objects of different sizes for the given picture -- this is determined arbitrarily for now and can be changed to any other values for if new data or ranges are determined.
        # Initialize variables to hold each number of objects
        small_count = 0   # 10 - 75 px
        medium_count = 0  # 75 - 150 px
        large_count = 0   # 150 - 500 px
        massive_count = 0 # 500+ px
        ignored_count = 0 # Under 10 px

        for tgt in targets:
            if 10 <= tgt.size < 75:
                small_count += 1
            elif 75 <= tgt.size < 150:
                medium_count += 1
            elif 150 <= tgt.size < 500:
                large_count += 1
            elif tgt.size >= 500:
                massive_count += 1
            else:
                ignored_count += 1 


        # Total object count
        total_count = len(targets)

        # Calculate percentages of object sizes compared to the total number of objects
        percent_small = (small_count / total_count) * 100
        percent_medium = (medium_count / total_count) * 100
        percent_large = (large_count / total_count) * 100
        percent_massive = (massive_count / total_count) * 100

        # Output different sizes of objects and percents of total to the console -- all percentages only with 2 digits after the decimal point
        print(f"There are {total_count} total objects detected in this image")
        print(f"There are {small_count} objects between 10 - 75 pixels and represents {percent_small: .2f} percent of the total objects.")
        print(f"There are {medium_count} objects between 75 - 150 pixels and represents {percent_medium: .2f} percent of the total objects.")
        print(f"There are {large_count} objects between 150 - 500 pixels and represents {percent_large: .2f} percent of the total objects.")
        print(f"There are {massive_count} objects over 500 pixels and represents {percent_massive: .2f} percent of the total objects.")


        # 6. Visual verification of the pipeline stages
        dilated_mask = my_detector.filter(frame)
        
        # --- SCREEN WINDOW RESIZING LOGIC START ---
        
        # Define how much of your vertical screen height you want the window to occupy (e.g., 60%)
        target_screen_height = 650 
        
        # Calculate the aspect ratio of your image (width / height)
        img_h, img_w = frame.shape[:2]
        aspect_ratio = img_w / img_h
        
        # Calculate the new scaled width based on your target height
        target_screen_width = int(target_screen_height * aspect_ratio)
        
        # Window 1: Original Frame with Bounding Boxes
        cv2.namedWindow("1. Original Frame with Bounding Boxes", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("1. Original Frame with Bounding Boxes", target_screen_width, target_screen_height)
        cv2.imshow("1. Original Frame with Bounding Boxes", display_frame)
        
        # Window 2: Dilated Mask
        cv2.namedWindow("2. Dilated Mask (What the detector sees)", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("2. Dilated Mask (What the detector sees)", target_screen_width, target_screen_height)
        cv2.imshow("2. Dilated Mask (What the detector sees)", dilated_mask)
        
        # --- SCREEN WINDOW RESIZING LOGIC END ---
        
        print("\nPress any key on the image windows to close and exit.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    except Exception as e:
        print(f"An error occurred during testing: {e}")

if __name__ == "__main__":
    test_on_single_image(r"C:\Users\Graeme\Downloads\Star_test_image.jpg")
