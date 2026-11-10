from typing import List

import cv2 as cv
import numpy as np
from numpy.typing import NDArray
from dt_apriltags import Detector, Detection

def unwarp_image(path, calibration_path, at_size, x_pixels_out, debug=False):
    """
    unwarp_image takes a calibrated image and calculates a homography matrix
    to change the perspective to being top down. The math used rectify the 
    ground plane is covered in depth in the README.md, as such it will only 
    be briefly commented here.
    
    Arguments:
        path: The path to the image to process
        calibration_path: The path to the calibration data for the camera used.
        at_size: The size of the april tags used.
        x_pixels_out: The width of the returned image.
        debug: option parameter, if true end the function early and display the image with all 
            detected april tags outlined and labeled
    
    Returns:
        A Tuple containing: the new image, the resolution (m / pixel), and a tuple(width height) in pixels.
      
    """

    image = cv.imread(path)

    # calibration_data = np.load("src/maze_solver/tools/calibration_data.npz")
    calibration_data = np.load(calibration_path)
    mtx = calibration_data['matrix']
    dist = calibration_data['distortion']

    
    if image is not None:
        size_tuple = image.shape[1::-1] 


    # The algorithm I use depends on the intrinsic matrix of the camera. 
    # We calculate what the new intrinsic matrix is after calibrating here
    mtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, size_tuple, 1, size_tuple)

    # The image is cropped in the calibration process we need to recenter 
    # our matrix for the cropped image. AI gave me some ideas for this 
    # approach but I implemented it. 
    x, y, _, _ = roi

    mtx[0, 2] -= x
    mtx[1, 2] -= y

    fx = mtx[0, 0]
    fy = mtx[1, 1]
    cx = mtx[0, 2]
    cy = mtx[1, 2]

    # These params are needed by the april tag detector
    camera_params = [fx, fy, cx, cy]

    at_detector = Detector()

    if image is None:
        print("Can't find image")
        return None
    else:
        # The program wants a grayscale image
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        # By giving the detector the april tag size we get estimated positions relative to the camera
        tags: List[Detection] = at_detector.detect(gray, True, camera_params, at_size)
        output = image.copy()
        for tag in tags:
            # I tried to find the proper reshape incantation to get this into the 
            # form np wants but in the end I had to use AI
            contour = tag.corners.reshape((-1, 1, 2)).astype(np.int32) # pyright: ignore[reportAttributeAccessIssue]

            cv.drawContours(output, [contour], -1, (0, 255, 0), 2) # pyright: ignore[reportArgumentType, reportCallIssue]
            center = tag.center
            pos = tag.pose_t
            cv.putText(output, f"ID: {tag.tag_id}, Pos: ({pos})", (int(center[0]), int(center[1])), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1) # type: ignore
        if debug:
            # Debug code that can be run to make sure all april tags are detected
            cv.imshow('output', output)
            cv.waitKey(0)
            return

        wld_pose: NDArray = np.array([tag.pose_t.flatten() for tag in tags]) # type: ignore
        
        # This section is the implementation of the math derived in MatLab. 
        # This section is done entirely without AI, and comes straight from my brain.
        # I say this because I am proud of it -Harlan. 
        X = wld_pose[:, 0]
        Y = wld_pose[:, 1]
        Z = wld_pose[:, 2]

        A = np.column_stack((np.ones((len(X), 1)), X.T, Y.T))
        # print(A)
        coeffs, residuals, _, _ = np.linalg.lstsq(A, Z, rcond=None)
        print(residuals)
        # print(coeffs)

        x_mag = 1 / np.sqrt(1+coeffs[1]**2)
        y_mag = 1 / np.sqrt(1+coeffs[2]**2)

        # calculate a basis vector on the plain in both the x and y axis
        x_basis = np.array([x_mag, 0, coeffs[1]*x_mag])
        y_basis = np.array([0, y_mag, coeffs[2]*y_mag])

        P = np.array([X, Y, Z-coeffs[0]])

        # Find the dot product of each points in space and each basis vector
        p_on_x = x_basis @ P
        p_on_y = y_basis @ P

        # find the min and maximum x, y coefficient
        x_min_c = min(p_on_x)
        x_max_c = max(p_on_x)
        y_min_c = min(p_on_y)
        y_max_c = max(p_on_y)

        # Find the range, in the plane, of the x and y values.
        x_range_c = x_max_c - x_min_c
        x_range = np.linalg.norm(x_basis[:, np.newaxis]*x_range_c)
        y_range_c = y_max_c - y_min_c
        y_range = np.linalg.norm(y_basis[:, np.newaxis]*y_range_c)

        # Calculate the size of the image
        x_pixels = x_pixels_out;
        y_pixels: NDArray = x_pixels * (y_range / x_range);

        # find the pixel positions for each tag from there position realize
        # to the min max on each basis
        x_pix =  ( p_on_x - x_min_c ) / x_range_c * x_pixels;
        y_pix =  ( p_on_y - y_min_c ) / y_range_c * y_pixels;

        # Accumulate an array of source points (in the original image)
        # and destination points in the desired image
        src_points = np.array([tag.center for tag in tags])
        dst_points = np.array([
            [x_pix[0], y_pix[0]],
            [x_pix[1], y_pix[1]],
            [x_pix[2], y_pix[2]],
            [x_pix[3], y_pix[3]]
        ])

        # This function takes in pixel coordinates in the image you have 
        # (source points) and a list of corresponding pixel coordinates in the
        # image you want to have. This function then returns a matrix that 
        # transforms the whole image in a way that best fits each source point 
        # to its destination. All of the math was to find where each april tag 
        # center should end up in a top down version of the image where scale 
        # is maintained.
        H, _ = cv.findHomography(src_points, dst_points, cv.RANSAC, 3.0)


        # We then use teh holography matrix to warp the image
        warped = cv.warpPerspective(image, H, (x_pixels, y_pixels.astype(int).item()))
        return (warped, x_range / x_pixels, (y_pixels.astype(int).item(), x_pixels))


if __name__ == "__main__":
    # This file can be run directly to test if we detect the april tags in a 
    # specific image. This is just used as a quick hack for testing purposes 
    # and is not called during normal use.
    name = "maze_mac2"
    image_path = f"src/maze_solver/tools/photo_{name}.jpg"
    calibration_path = "src/maze_solver/tools/calibration_data.npz"
    at_size = 0.15875

    unwarp_image(image_path, calibration_path, at_size, 1000, True)