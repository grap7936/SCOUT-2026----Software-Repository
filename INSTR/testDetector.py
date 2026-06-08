import cv2
from DetectorClass import detector  # Replace with your actual file name

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
        
        # 5. Print out text results to verify object instantiation
        for idx, tgt in enumerate(targets):
            print(f"Target {idx}: Center=({tgt.x}, {tgt.y}), Size={tgt.size}px")

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
    test_on_single_image(r"C:\Users......")
