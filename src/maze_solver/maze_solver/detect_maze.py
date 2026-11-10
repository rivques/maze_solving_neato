from ament_index_python import get_package_share_directory
import cv2 as cv
import numpy as np
from april_tag_detect import unwarp_image
import math
from yaml import safe_dump, safe_load
from os.path import join


def main():
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
        H_min = data["H_min"]
        S_min = data["S_min"]
        V_min = data["V_min"]
        H_max = data["H_max"]
        S_max = data["S_max"]
        V_max = data["V_max"]
        threshold_min = data["threshold_min"]
        size = data["size"]
        neato_rad = data["neato_rad"]
        at_size = data["at_size"]
        im_x_width = data["im_x_width"]

    image_path = f"src/maze_solver/tools/photo_{name}.jpg"
    calibration_path = "src/maze_solver/tools/calibration_data.npz"
    
    # Use the function provided in the april_tag_detect.py file to get the top down view of the image.
    output = unwarp_image(image_path, calibration_path, at_size, im_x_width)

    # Check if the returned image is None.
    if output is not None:
        image, res, im_size = output 
        cv.imwrite("assets/corrected_maze_image.jpg", image)
    else:
        image = None
        res = None
        print("can't get processed image")


    # These images are all displayed to show important snapshots from within the filtering process
    cv.namedWindow('Masked Image', cv.WINDOW_NORMAL)
    cv.resizeWindow('Masked Image', 1280, 720)

    cv.namedWindow('White Mask', cv.WINDOW_NORMAL)
    cv.resizeWindow('White Mask', 1280, 720)

    cv.namedWindow('combined', cv.WINDOW_NORMAL)
    cv.resizeWindow('combined', 1280, 720)

    cv.namedWindow('output', cv.WINDOW_NORMAL)
    cv.resizeWindow('output', 1280, 720)

    cv.namedWindow('output map', cv.WINDOW_NORMAL)
    cv.resizeWindow('output map', 1280, 720)


    # Uncomment the  below code to include trackbars to calibrate in different conditions.
    # If using this code uncomment the corresponding section to pull values in the while loop.
    # cv.createTrackbar('H_min', 'Masked Image', H_min, 180, lambda i: None)
    # cv.createTrackbar('S_min', 'Masked Image', S_min, 255, lambda i: None)
    # cv.createTrackbar('V_min', 'Masked Image', V_min, 255, lambda i: None)
    # cv.createTrackbar('H_max', 'Masked Image', H_max, 180, lambda i: None)
    # cv.createTrackbar('S_max', 'Masked Image', S_max, 255, lambda i: None)
    # cv.createTrackbar('V_max', 'Masked Image', V_max, 255, lambda i: None)
    # cv.createTrackbar('white_thres', 'White Mask', threshold_min, 255, lambda i: None)
    # cv.createTrackbar('size', 'cleaned', size, 10, lambda i: None)

    if image is not None and res is not None:
        while True:
            # Uncomment these lines if calibrating with track bars.
            # H_min = cv.getTrackbarPos('H_min', 'Masked Image')
            # S_min = cv.getTrackbarPos('S_min', 'Masked Image')
            # V_min = cv.getTrackbarPos('V_min', 'Masked Image')
            # H_max = cv.getTrackbarPos('H_max', 'Masked Image')
            # S_max = cv.getTrackbarPos('S_max', 'Masked Image')
            # V_max = cv.getTrackbarPos('V_max', 'Masked Image')
            # threshold_min = cv.getTrackbarPos('white_thres', 'White Mask')
            # size = cv.getTrackbarPos('size', 'combined')

            # Get the hsv representation of the image.
            hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

            # Define range for blue color in HSV
            lower_blue = np.array([H_min, S_min, V_min])
            upper_blue = np.array([H_max, S_max, V_max])

            # Create and image masking for the blue checkers
            mask = cv.inRange(hsv, lower_blue, upper_blue)

            # Create an image masking for the white checkers
            white_mask = cv.inRange(hsv, np.array([0, 0, threshold_min]), np.array([180, 255, 255]))

            # Show frames
            cv.imshow('Masked Image', mask)
            cv.imshow('White Mask', white_mask)

            # We blur both masked images and take a binary and to get the region where they overlap
            # This is almost entirely on the checkered tape when tunned well
            kernel = np.ones((size, size), np.uint8)
            dilated1 = cv.dilate(mask, kernel, iterations=1)
            dilated2 = cv.dilate(white_mask, kernel, iterations=1)

            combined = cv.bitwise_and(dilated1, dilated2)

            # Next apply some AI suggested filtering to clean up the result.
            # We found experimentally that both kernels can be the same size 
            kernel = cv.getStructuringElement(cv.MORPH_RECT, (size, size))
            cleaned = cv.morphologyEx(combined, cv.MORPH_CLOSE, kernel)
            cleaned = cv.morphologyEx(cleaned, cv.MORPH_OPEN, kernel)

            cv.imshow('combined', cleaned)

            # Now we find contours within the image
            contours, _ = cv.findContours(cleaned, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

            # We calculate the radius of the neato in pixels to use later when making the map
            neato_rad_pix =  math.ceil(neato_rad / res)

            output_vis = image.copy()

            # We create a new entirely white image to be the base for our map output
            output_map = np.full((im_size[0], im_size[1], 1), 255, dtype=np.uint8)
            
            for cnt in contours:
                # we ensure the area of the contour is high enough to not be background noise
                area = cv.contourArea(cnt)
                if area > 1000:
                    # We additionally test if the perimeter is high 
                    # (true for long rectangles not noisy blobs)
                    perimeter = cv.arcLength(cnt, True)
                    if perimeter > 300: 
                        np.round(cnt.astype(np.int32))
                        for point in cnt:
                            # at each point in a valid contour we draw a circle of radius 
                            # equal to the neato's radius (+ a error margin)
                            x, y = point[0]
                            cv.circle(output_map, (int(x), int(y)), neato_rad_pix, color=0, thickness=1, lineType=cv.LINE_AA)
                        # We draw the entire contour as well for debugging purposes
                        cv.drawContours(output_vis, [cnt], -1, (0, 255, 0), 2)

            # The output map as some noise due to anti-aliasing. If we turn off
            # AA we get really bad circles so instead we just filter all mostly 
            # white pixels to white and everything else to black
            _, output_map = cv.threshold(output_map, 245, 255, cv.THRESH_BINARY)

            cv.imshow('output', output_vis)
            cv.imshow('output map', output_map)

            # Delay and listen for key press
            key = cv.waitKey(1) & 0xFF

            # If 'q' is pressed we quit
            if  key == ord('q'):
                break
            # If 's' is pressed we save the maze map file as a png and a 
            # corresponding map yaml
            elif key == ord('s'):
                print("saving")
                cv.imwrite(f'src/maze_solver/maps/{name}.png', output_map)
                yaml_path  = f"src/maze_solver/maps/{name}.yaml"
                yaml_data = {
                    'image': f'{name}.png',
                    'mode': 'trinary',
                    'resolution': float(res),
                    'origin': [0,0,0],
                    'negate': 0,
                    'occupied_thresh': 0.65,
                    'free_thresh': 0.25
                }

                # saves the yaml file
                with open(yaml_path, "w", encoding="utf-8") as file:
                    safe_dump(yaml_data, file, default_flow_style=False, sort_keys=False)




if __name__ == "__main__":
    main()