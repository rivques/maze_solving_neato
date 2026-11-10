from typing import List

from ament_index_python import get_package_share_directory
import cv2
import numpy as np
from dt_apriltags import Detector, Detection
from os.path import join

from yaml import safe_load

def main():
    """This file contains code to save a calibrated photo of our maze. 
    To ensure we can properly detect all april tags in the image we add an 
    overlay to show detected AprilTags
    
    This code was loosely based on the same AI code used in get calibration data
    but has been altered significantly for our purposes"""

    share_dir = get_package_share_directory("maze_solver")
    yaml_path = join(share_dir, "config/maze_detection_config.yaml")

    """Yaml file structure:
    
    at_size: 0.15875
    H_min: 61
    S_min: 14
    V_min: 149
    H_max: 98
    S_max: 28
    V_max: 190
    threshold_min: 183
    size: 9
    photo_name: "maze_mac2"
    neato_rad: 0.2
    im_x_width: int
    """

    with open(yaml_path, "r") as f:
        data = safe_load(f)
        name = data["photo_name"]
        at_size = data["at_size"]

    camera_index = "/dev/video4"
    
    # Initialize the video capture object
    cap = cv2.VideoCapture(camera_index)
    
    # We set our video capture size to 1080p. 
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    # We verify that the resolution is actually 1080p to ensure our camera supports that
    actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"Requested: 1920x1080 | Actual: {actual_width}x{actual_height}")

    
    cv2.namedWindow('USB Video Feed', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('USB Video Feed', 1280, 720)


    # Check if the webcam/USB camera opened successfully
    if not cap.isOpened():
        print(# Specify error window details if troubleshooting multiple nodes
            f"Error: Could not open video device at index {camera_index}."
        )
        return


    # Load and use the calibration data
    calibration_data = np.load("src/maze_solver/tools/calibration_data.npz")
    mtx = calibration_data['matrix']
    dist = calibration_data['distortion']

    h,  w = int(actual_height), int(actual_width)
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))  # pyright: ignore[reportCallIssue]

    # Initialize our AprilTag detector
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

        # Adjust the camera matrix for the cropped image.
        cropped_mtx = mtx.copy()

        cropped_mtx[0, 2] -= x
        cropped_mtx[1, 2] -= y

        fx = cropped_mtx[0, 0]
        fy = cropped_mtx[1, 1]
        cx = cropped_mtx[0, 2]
        cy = cropped_mtx[1, 2]

        camera_params = [fx, fy, cx, cy]

        # This block is used to find an annotate the detected april tag locations
        gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
        tags: List[Detection] = at_detector.detect(gray, True, camera_params, at_size)
        output = dst.copy()

        if len(tags) == 4:
            # If all 4 tags are detected we automatically save the image.
            cv2.imwrite(f"src/maze_solver/tools/photo_{name}.jpg", dst)
            print("auto save")

        for tag in tags:
            # For each tag we draw its bounding box and label it by its ID
            contour = tag.corners.reshape((-1, 1, 2)).astype(np.int32) # pyright: ignore[reportAttributeAccessIssue]

            cv2.drawContours(output, [contour], -1, (0, 255, 0), 2) # pyright: ignore[reportArgumentType, reportCallIssue]
            center = tag.center
            pos = tag.pose_t
            cv2.putText(output, f"ID: {tag.tag_id}, Pos: ({pos})", (int(center[0]), int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1) # type: ignore
        cv2.imshow('output', output)

        # Delay and listen for key presses
        key = cv2.waitKey(1) & 0xFF

        # If 'q' is pressed stop
        if key == ord('q'):
            break

        # If 's' is pressed save
        if key == ord('s'):
            cv2.imwrite(f"src/maze_solver/tools/photo_{name}.jpg", dst)

    # When everything is done, release the capture object and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
