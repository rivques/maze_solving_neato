# How to Use Our Tools

This will serve as a breif overiew of how to use our tools. The purpose of these tools is to take a calibrated photo of our maze for future processing.

## Contents

This directory contians 3 scripts (in the order they should be used):
1. `get_calibration_data.py` - Gets photos to use for calibration
2. `calibrate_camera.py` - Find the intrinsic matrix for our camera
3. `get_maze_photo.py` - takes a calibrated photo of our maze

Additonaly this folder contains:
- A folder of calibration data images in `test_frames/`
- The intrinsic matrix data in `calibration_data.npz`
- calibrated maze photos taken by `get_maze_photo` (all top level .jpg files)

## How to run
As a prerequisite to running any of these files pixi must be set up and dependencies must be installed through it
### get_calibration_data.py
To run this file you will need to be connected to a USB camera. Make sure to change the camera_index variable to reflect the connection to your camera.

To run this file from the top level of this directory run `pixi run python3 src/maze_solver/tools/get_calibration_data.py`

To collect calibration data you will need to print our a calibration grid using this site: [calib.io](https://calib.io/pages/camera-calibration-pattern-generator). Hold the grid in front of the camera and move it around to a verity of different positions and angles. When you want to take a photo press the 's' key with the video feed in focus. Press 'q' to stop.

Calibration images are saved under `tools/test_frames/calibration_img_N.jpg` where N is the number of images that have already been taken.
This script can be bypassed using any camera utility of your choice as long as the naming conversions are maintained

### calibrate_camera.py
This script relies on having calibration data under the folder `tools/test_frames/`

To run this script run `pixi run python3 src/maze_solver/tools/calibrate_camera.py`

This script will go through each calibration image and for half a second show you the image with the identified grid lines displayed. Once the process is done it will save a file `tools/calibration_data.npz`. This file uses numpy's saving format and contains the calibration data for the camera.

### get_maze_photo.py
To run this file you will need to be connected to a USB camera. Make sure to change the camera_index variable to reflect the connection to your camera.

To run this script run `pixi run python3 src/maze_solver/tools/get_maze_photo.py`.

When the script is running it will show you a video feed from the camera with the detected april tags overlayed. The video feed as a very high latency. To save a maze photo press 's'. Additionally, if four april tags are detected the script will auto save. Press 'q' to quit. 

The name of the saved maze will be set by the name variable in the maze_detection_config.yaml file. If you change the name variable there it will save with the name expected by all other scripts.