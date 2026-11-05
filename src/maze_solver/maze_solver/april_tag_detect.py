from typing import List

import cv2 as cv
import numpy as np
import math
from numpy.typing import NDArray
from dt_apriltags import Detector, Detection

def unwarp_image(path, calibration_path, at_size, x_pixles_out):
    # image_path = "src/maze_solver/tools/maze_photo_test.jpg"
    image = cv.imread(path)

    # calibration_data = np.load("src/maze_solver/tools/calibration_data.npz")
    calibration_data = np.load(calibration_path)
    mtx = calibration_data['matrix']
    dist = calibration_data['distortion']

    if image is not None:
        size_tuple = image.shape[1::-1] 

    mtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, size_tuple, 1, size_tuple)

    # Extract the crop coordinates.
    x, y, roi_width, roi_height = roi

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

    at_detector = Detector()

    # cv.namedWindow('output', cv.WINDOW_NORMAL)
    # cv.resizeWindow('output', 1280, 720)

    if image is None:
        print("Can't find image")
        return None
    else:
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        tags: List[Detection] = at_detector.detect(gray, True, camera_params, at_size)
        output = image.copy()
        for tag in tags:
            contour = tag.corners.reshape((-1, 1, 2)).astype(np.int32) # pyright: ignore[reportAttributeAccessIssue]
            # print(contour)
            # print(tag.tag_id)
            cv.drawContours(output, [contour], -1, (0, 255, 0), 2) # pyright: ignore[reportArgumentType, reportCallIssue]
            center = tag.center
            pos = tag.pose_t
            cv.putText(output, f"ID: {tag.tag_id}, Pos: ({pos})", (int(center[0]), int(center[1])), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1) # type: ignore
        # cv.imshow('output', output)

        wld_pose: NDArray = np.array([tag.pose_t.flatten() for tag in tags]) # type: ignore
        # print(wld_pose)

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

        x_basis = np.array([x_mag, 0, coeffs[1]*x_mag])
        y_basis = np.array([0, y_mag, coeffs[2]*y_mag])

        P = np.array([X, Y, Z-coeffs[0]])

        # print(P)

        p_on_x = x_basis @ P
        p_proj_x = x_basis[:, np.newaxis] *  p_on_x
        p_on_y = y_basis @ P
        p_proj_y = y_basis[:, np.newaxis] *  p_on_y

        # print(p_proj_x)
        # print(p_proj_y)

        x_min_c = min(p_on_x)
        x_max_c = max(p_on_x)
        y_min_c = min(p_on_y)
        y_max_c = max(p_on_y)

        bounds_x_c = np.array([x_min_c, x_min_c, x_max_c, x_max_c, x_min_c])
        bounds_y_c = np.array([y_min_c, y_max_c, y_max_c, y_min_c, y_min_c])
        bound_points = x_basis[:, np.newaxis]*bounds_x_c+y_basis[:, np.newaxis]*bounds_y_c+np.array([0, 0, coeffs[0]])[:, np.newaxis]
        x_range_c = x_max_c - x_min_c
        x_range = np.linalg.norm(x_basis[:, np.newaxis]*x_range_c)
        # x_min = min(bound_points[0, :])
        # y_min = min(bound_points[1, :])
        y_range_c = y_max_c - y_min_c
        y_range = np.linalg.norm(y_basis[:, np.newaxis]*y_range_c)

        x_pixles = x_pixles_out;
        y_pixles: NDArray = x_pixles * (y_range / x_range);

        x_pix =  ( p_on_x - x_min_c ) / x_range_c * x_pixles;
        y_pix =  ( p_on_y - y_min_c ) / y_range_c * y_pixles;

        src_points = np.array([tag.center for tag in tags])
        dst_points = np.array([
            [x_pix[0], y_pix[0]],
            [x_pix[1], y_pix[1]],
            [x_pix[2], y_pix[2]],
            [x_pix[3], y_pix[3]]
        ])
        # print(src_points)
        # print(dst_points)

        H, _ = cv.findHomography(src_points, dst_points, cv.RANSAC, 3.0)
        # print(x_range / x_pixles)
        # print(y_range / y_pixles)


        warped = cv.warpPerspective(image, H, (x_pixles, y_pixles.astype(int).item()))
        return (warped, x_range / x_pixles, (y_pixles.astype(int).item(), x_pixles))
        # cv.imshow("warped", warped)
        
        # cv.waitKey(0)