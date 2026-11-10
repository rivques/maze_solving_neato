# How to Use Our Tools

This will serve as a breif overiew of how to use our tools. The purpose of these tools is to take a calibrated photo of our maze for future processing.

## Contents

This directory contians 3 scripts (in the order they should be used):
1. `get_calibration_data.py` - Gets photos to use for calibration
2. `calibrate_camera.py` - Find the intrisic matrix for our camera
3. `get_maze_photo.py` - takes a calibrated photo of our maze

Additonaly this folder contains:
- A folder of calibration data images in `test_frames/`
- The intrisic matrix data in `calibration_data.npz`
- calibrated maze photos taken by `get_maze_photo` (all top level .jpg files)