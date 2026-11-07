import cv2 as cv
import numpy as np
from april_tag_detect import unwarp_image
import math
from yaml import safe_dump

name = "maze_mac2"

image_path = f"src/maze_solver/tools/photo_{name}.jpg"
calibration_path = "src/maze_solver/tools/calibration_data.npz"
at_size = 0.15875

output = unwarp_image(image_path, calibration_path, at_size, 1000)

if output is not None:
    image, res, im_size = output 
    print(res)
else:
    image = None
    res = None
    print("can't get processed image")

# cv.namedWindow('Maze Photo', cv.WINDOW_NORMAL)
# cv.resizeWindow('Maze Photo', 1280, 720)

cv.namedWindow('Masked Image', cv.WINDOW_NORMAL)
cv.resizeWindow('Masked Image', 1280, 720)
cv.createTrackbar('Hmin', 'Masked Image', 61, 180, lambda i: None)
cv.createTrackbar('Smin', 'Masked Image', 14, 255, lambda i: None)
cv.createTrackbar('Vmin', 'Masked Image', 149, 255, lambda i: None)
cv.createTrackbar('Hmax', 'Masked Image', 98, 180, lambda i: None)
cv.createTrackbar('Smax', 'Masked Image', 28, 255, lambda i: None)
cv.createTrackbar('Vmax', 'Masked Image', 190, 255, lambda i: None)

Hmin = 61
Smin = 14
Vmin = 149
Hmax = 98
Smax = 28
Vmax = 190

cv.namedWindow('White Mask', cv.WINDOW_NORMAL)
cv.resizeWindow('White Mask', 1280, 720)
cv.createTrackbar('white_thres', 'White Mask', 183, 255, lambda i: None)

threshold_min = 183

cv.namedWindow('cleaned', cv.WINDOW_NORMAL)
cv.resizeWindow('cleaned', 1280, 720)
cv.createTrackbar('size', 'cleaned', 9, 10, lambda i: None)

size = 9

cv.namedWindow('output', cv.WINDOW_NORMAL)
cv.resizeWindow('output', 1280, 720)

cv.namedWindow('output map', cv.WINDOW_NORMAL)
cv.resizeWindow('output map', 1280, 720)

if image is not None and res is not None:
    # cv.imshow('Maze Photo', image)
    b, _, _ = cv.split(image)
    while True:
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        Hmin = cv.getTrackbarPos('Hmin', 'Masked Image')
        Smin = cv.getTrackbarPos('Smin', 'Masked Image')
        Vmin = cv.getTrackbarPos('Vmin', 'Masked Image')
        Hmax = cv.getTrackbarPos('Hmax', 'Masked Image')
        Smax = cv.getTrackbarPos('Smax', 'Masked Image')
        Vmax = cv.getTrackbarPos('Vmax', 'Masked Image')

        # Define range for blue color in HSV
        lower_blue = np.array([Hmin, Smin, Vmin])
        upper_blue = np.array([Hmax, Smax, Vmax])

        # Create mask
        mask = cv.inRange(hsv, lower_blue, upper_blue)
        
        # Filter the blue region
        # result = cv.bitwise_and(image, image, mask=mask)

        threshold_min = cv.getTrackbarPos('white_thres', 'White Mask')

        white_mask = cv.inRange(hsv, np.array([0, 0, threshold_min]), np.array([180, 255, 255]))

        # Show frames
        cv.imshow('Masked Image', mask)
        cv.imshow('White Mask', white_mask)

        size = cv.getTrackbarPos('size', 'cleaned')

        kernel = np.ones((size, size), np.uint8)
        dilated1 = cv.dilate(mask, kernel, iterations=1)
        dilated2 = cv.dilate(white_mask, kernel, iterations=1)


        combined = cv.bitwise_and(dilated1, dilated2)
        # cv.imshow('combined', combined)


        # # Create a kernel for morphological dilation
        # kernel = np.ones((n, n), np.uint8)/9

        # # Dilate both masks slightly to capture the immediate boundary
        # dilated = cv.dilate(combined, kernel, iterations=1)

        # cv.imshow('dilated', dilated)

        kernel = cv.getStructuringElement(cv.MORPH_RECT, (size, size))
        cleaned = cv.morphologyEx(combined, cv.MORPH_CLOSE, kernel)
        cleaned = cv.morphologyEx(cleaned, cv.MORPH_OPEN, kernel)

        cv.imshow('cleaned', cleaned)

        contours, _ = cv.findContours(cleaned, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

        neato_rad = 0.2 #m
        neato_rad_pix =  math.ceil(neato_rad / res)

        output_vis = image.copy()
        output_map = np.full((im_size[0], im_size[1], 1), 255, dtype=np.uint8)
        for cnt in contours:
            # print("got contour")
            # Filter by size to ignore background noise
            area = cv.contourArea(cnt)
            if area > 1000:  # Adjust based on image resolution
                # Approximate shape to check for squares/rectangles
                peri = cv.arcLength(cnt, True)
                if peri > 300:
                    np.round(cnt.astype(np.int32))
                    for point in cnt:
                        x, y = point[0]
                        cv.circle(output_map, (int(x), int(y)), neato_rad_pix, color=0, thickness=1, lineType=cv.LINE_AA)
                    approx = cv.approxPolyDP(cnt, 0.04 * peri, True)
                    
                    cv.drawContours(output_vis, [cnt], -1, (0, 255, 0), 2)

        _, output_map = cv.threshold(output_map, 245, 255, cv.THRESH_BINARY)

        cv.imshow('output', output_vis)
        cv.imshow('output map', output_map) # type: ignore

        key = cv.waitKey(1) & 0xFF

        if  key == ord('q'):
            break
        elif key == ord('s'):
            print("saveing")
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

            with open(yaml_path, "w", encoding="utf-8") as file:
                safe_dump(yaml_data, file, default_flow_style=False, sort_keys=False)


