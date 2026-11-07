from typing import List

import cv2
import numpy as np
from dt_apriltags import Detector, Detection

def main():
    # 0 is typically the default built-in webcam or the first connected USB camera.
    # If using multiple cameras, try changing this index to 1, 2, etc., 
    # or pass a device path string like "/dev/video0" on Linux.
    camera_index = "/dev/video4"
    
    # Initialize the video capture object
    cap = cv2.VideoCapture(camera_index)
    
    # Optional: Set custom frame width and height
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"Requested: 1920x1080 | Actual: {actual_width}x{actual_height}")

    
    cv2.namedWindow('USB Video Feed', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('USB Video Feed', 1280, 720)

    cv2.namedWindow('Calibrated Feed', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Calibrated Feed', 1280, 720)


    # Check if the webcam/USB camera opened successfully
    if not cap.isOpened():
        print(# Specify error window details if troubleshooting multiple nodes
            f"Error: Could not open video device at index {camera_index}."
        )
        return

    print("Press 'q' to exit the video feed.")

    frame_count = 0

    calibration_data = np.load("src/maze_solver/tools/calibration_data.npz")
    mtx = calibration_data['matrix']
    dist = calibration_data['distortion']

    h,  w = int(actual_height), int(actual_width)
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))  # pyright: ignore[reportCallIssue]

    at_detector = Detector()

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # If the frame was not grabbed successfully, break the loop
        if not ret:
            print("Error: Failed to grab frame.")
            break

        # Display the resulting frame in a window named 'USB Video Feed'

        cv2.imshow('USB Video Feed', frame)


        # undistort
        dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)

        # crop the image
        x, y, w, h = roi
        # print(x, y, w, h)
        dst = dst[y:y+h, x:x+w]

        cv2.imshow('Calibrated Feed', dst)

        image = dst

        mtx = calibration_data['matrix']
        dist = calibration_data['distortion']

        if image is not None:
            size_tuple = image.shape[1::-1] 

        # Adjust the camera matrix for the cropped image.
        cropped_mtx = mtx.copy()

        cropped_mtx[0, 2] -= x
        cropped_mtx[1, 2] -= y

        fx = cropped_mtx[0, 0]
        fy = cropped_mtx[1, 1]
        cx = cropped_mtx[0, 2]
        cy = cropped_mtx[1, 2]

        camera_params = [fx, fy, cx, cy]
        # at_size = 0.12065 # m

        
        at_size = 0.15875

        # cv2.namedWindow('output', cv2.WINDOW_NORMAL)
        # cv2.resizeWindow('output', 1280, 720)

        if image is None:
            print("Can't find image")
            return None
        else:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            tags: List[Detection] = at_detector.detect(gray, True, camera_params, at_size)
            output = image.copy()
            if len(tags) == 4:
                cv2.imwrite(f"src/maze_solver/tools/photo_maze_mac2.jpg", dst)
                print("auto save")
            for tag in tags:
                contour = tag.corners.reshape((-1, 1, 2)).astype(np.int32) # pyright: ignore[reportAttributeAccessIssue]
                # print(contour)
                # print(tag.tag_id)
                cv2.drawContours(output, [contour], -1, (0, 255, 0), 2) # pyright: ignore[reportArgumentType, reportCallIssue]
                center = tag.center
                pos = tag.pose_t
                cv2.putText(output, f"ID: {tag.tag_id}, Pos: ({pos})", (int(center[0]), int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1) # type: ignore
            cv2.imshow('output', output)

        key = cv2.waitKey(1) & 0xFF

        # Wait for 1 millisecond and check if the user pressed the 'q' key
        if key == ord('q'):
            break

        if key == ord('s'):
            cv2.imwrite(f"src/maze_solver/tools/photo_maze_mac2.jpg", dst)

    # When everything is done, release the capture object and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
