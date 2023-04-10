This is a project on calibration and augmented reality implemented in C++ using OpenCV functions. The main goal of the project is to calibrate a camera and then use the calibration to generate virtual objects in a scene. The end result is a program capable of detecting a target to create and project 3D objects that can move and orient themselves relative to the motion of the camera and the target.

Features:

Detect and Extract Chessboard Corners
A function is programmed to scan a video frame while detecting and extracting the chessboard corners from it. These corners are then drawn on the chessboard virtually for validation.

Select Calibration Images
A function that lets the user specify that a particular image should be used for the calibration and save the corner locations and the corresponding 3D world points. The user can continue pressing ‘S’ to store calibration images which are saved to the source folder as long as there is a chessboard detected in it.

Calibrate the Camera
If the user selects at least 5 calibration images, then he can successfully use the calibration function that computes the intrinsic properties of the camera being used and stores them to an external XML file in the source folder. These intrinsic properties will later be used for projection onto 2D spaces.

Calculate Current Position of the Camera
The solvePNP function is then utilized to get the board’s pose (rotation and translation) using the camera calibration parameters from the intrinsic.xml stored earlier.

Project Outside Corners or 3D Axes
Three functions are written here. Pressing 3 will let the user see the borders on the chessboard with the corners marked. Pressing 4 will show the 3D axes on the chessboard. Pressing 5 will show circles around the 4 vertices of the chessboard.

Create a Virtual Object
A 3D square pyramid with the edges coloured differently is projected onto the center of the chessboard. The edges are coloured differently to make it easier to see the rotation of the object when rotating the camera.

Detect Robust Features
A different program is written to exhibit the detection of Harris Corners. This implementation is faster than the earlier implementation. A video is included to demonstrate its working.

Extensions
Two functions are implemented to explore placement and scaling relative to the target with the utilization of corners acquired from the OpenCV functions. One function accurately places a cube on the first square of the chessboard. The second function implements a color-varying barrier/shield that masks the chessboard completely to give the disguise of an actual object placed on a surface.

To run the program, change the address string in main to use a different camera. Augment.cpp is the main function to view calibration, chessboard corners, 3D and 2D projections with extensions. Aug.h and Aug.cpp are function files for Augment. robust_Aug.cpp is the main file for Harris corner detection.

References are also provided in the README.

Two videos, one report, and one example of a calibration image are attached.