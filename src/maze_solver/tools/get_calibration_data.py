import cv2

def main():
    """This file is dedicated to showing a video feed from our camera and allowing us to save frames from it for calibration
    The core code is loosely based on an answer from google search AI but has since been substantially adapted to our use case"""
    camera_index = "/dev/video4"
    
    # Initialize the video capture object
    cap = cv2.VideoCapture(camera_index)

    # Create a window to display output. 
    # Without creating the window explicitly it would sometimes be too small and wouldn't resize
    cv2.namedWindow('USB Video Feed', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('USB Video Feed', 1280, 720)


    # Check if the webcam/USB camera opened successfully
    if not cap.isOpened():
        print(# Specify error window details if troubleshooting multiple nodes
            f"Error: Could not open video device at index {camera_index}."
        )
        return

    print("Press 'q' to exit the video feed.")

    # We track home many frames have been recorded for naming
    frame_count = 0

    while True:
        # Capture current video frame
        ret, frame = cap.read()

        # If the frame was not grabbed successfully, break the loop
        if not ret:
            print("Error: Failed to grab frame.")
            break

        # Display the video
        cv2.imshow('USB Video Feed', frame)

        # delay and wait for key press
        key = cv2.waitKey(1) & 0xFF

        # If we pressed q stop
        if key == ord('q'):
            break

        # If we pressed save the current frame to the test_frame directry in tools
        if key == ord('s'):
            cv2.imwrite(f"src/maze_solver/tools/test_frames/calibration_img_{frame_count}.jpg", frame)
            frame_count += 1

    # When everything is done, release the capture object and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
