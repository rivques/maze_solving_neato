"""This code is taken from the demo found here: https://docs.opencv.org/4.13.0/dc/dbb/tutorial_py_calibration.html
The code has been modified to save our data and be correct for the grid size we are using for testing"""

import numpy as np
import cv2 as cv
import glob

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((10*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:10].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('src/maze_solver/tools/test_frames/calibration_img_*.jpg')
# print (images)

for fname in images:
    img = cv.imread(fname)
    if img is None:
        continue

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (7,10), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(img, (7,10), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)

cv.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# We save the calibration data as NP arrays in a .npz file. This file is used by our other image capturing functions
np.savez("src/maze_solver/tools/calibration_data", matrix = mtx, distortion = dist)

img = cv.imread(images[0])
if img is None:
    print("bad img")
else:
    h,  w = img.shape[:2]
    print(mtx, dist)
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h)) 
    # undistort
    dst = cv.undistort(img, mtx, dist, None, newcameramtx)

    # crop the image
    x, y, w, h = roi
    cv.imwrite('src/maze_solver/tools/test_frames/calib_result.png', dst)

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2_in, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    imgpoints2 = imgpoints2_in.squeeze()
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error

print( "mean error: {}".format(mean_error/len(objpoints)) )