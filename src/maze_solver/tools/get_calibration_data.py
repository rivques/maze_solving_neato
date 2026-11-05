import cv2

def main():
    # 0 is typically the default built-in webcam or the first connected USB camera.
    # If using multiple cameras, try changing this index to 1, 2, etc., 
    # or pass a device path string like "/dev/video0" on Linux.
    camera_index = "/dev/video4"
    
    # Initialize the video capture object
    cap = cv2.VideoCapture(camera_index)
    
    # Optional: Set custom frame width and height
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    # actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    # actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    # print(f"Requested: 1920x1080 | Actual: {actual_width}x{actual_height}")

    
    cv2.namedWindow('USB Video Feed', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('USB Video Feed', 1280, 720)


    # Check if the webcam/USB camera opened successfully
    if not cap.isOpened():
        print(# Specify error window details if troubleshooting multiple nodes
            f"Error: Could not open video device at index {camera_index}."
        )
        return

    print("Press 'q' to exit the video feed.")

    frame_count = 0

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # If the frame was not grabbed successfully, break the loop
        if not ret:
            print("Error: Failed to grab frame.")
            break

        # Display the resulting frame in a window named 'USB Video Feed'
        cv2.imshow('USB Video Feed', frame)

        key = cv2.waitKey(1) & 0xFF

        # Wait for 1 millisecond and check if the user pressed the 'q' key
        if key == ord('q'):
            break

        if key == ord('s'):
            cv2.imwrite(f"src/maze_solver/tools/test_frames/calibration_img_{frame_count}.jpg", frame)
            frame_count += 1

    # When everything is done, release the capture object and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
