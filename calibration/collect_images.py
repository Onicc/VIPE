import cv2
import os
from datetime import datetime

# Open the camera, set the camera ID to 0
cap = cv2.VideoCapture(0)

# Set the resolution to 1920x1080 (1080p)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Set the frame rate to 30fps
cap.set(cv2.CAP_PROP_FPS, 30)

# Check if the camera is opened successfully
if not cap.isOpened():
    print("Cannot open the camera")
    exit()

# Folder path to save the images
save_folder = "./images"
if not os.path.exists(save_folder):
    # os.makedirs(save_folder)
    print("Please check if the working directory is correct")
    exit()

# Start reading and displaying the frames
while True:
    ret, frame = cap.read()

    # Exit the loop if the frame cannot be read
    if not ret:
        print("Cannot receive frames, exiting")
        break

    # Display the real-time video
    cv2.imshow('Camera - Press "s" to save, "q" to quit', frame)

    # Wait for a key event
    key = cv2.waitKey(1) & 0xFF

    # Save the image if 's' key is pressed
    if key == ord('s'):
        # Get the current timestamp and format it as a string
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        image_path = os.path.join(save_folder, f'image_{timestamp}.png')
        
        # Save the image file
        cv2.imwrite(image_path, frame)
        print(f"Image saved to: {image_path}")

    # Exit the program if 'q' key is pressed
    if key == ord('q'):
        break

# Release the camera resources and close the window
cap.release()
cv2.destroyAllWindows()