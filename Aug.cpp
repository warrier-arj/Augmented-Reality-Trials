// Arjun Rajeev Warrier
// Spring 2023 CS 5330
// Project 4: Calibration and Augmented Reality  

//Aug.cpp
// custom created function definitions
// function defining comments inside function blocks


#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <string>
#include <numeric>
#include "Aug.h"
#include <chrono>
#include <cstdlib>
#include <ctime>



//Some prior declarations
std::vector<cv::Vec3f> point_set;
std::vector<std::vector<cv::Vec3f> > point_list;
std::vector<std::vector<cv::Point2f> > corner_list;
int calib_image_counter = 0;
const float squareSize = 0.025f; // Set square size to 25mm
cv::Size boardSize(9, 6);
cv::Mat distortion_coefficients = cv::Mat::zeros(5, 1, CV_64FC1);
cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64FC1);
cv::Mat rvec, tvec; //For the SolvePnP functions outputs---> rotation vec and translation vec
int heights[54];
std::chrono::time_point<std::chrono::high_resolution_clock> programStart = std::chrono::high_resolution_clock::now();


int key_check(cv::Mat& frame, cv::Mat& dest, char filt) {

	/*
	A pipeline for program. The main functin passes the frame from the feed and the user key press.
	the functions are then called accordingly from a switch case

	Input : frame from camera feed, filt = key pressed
	Output : returns an integer value that helps main decide to quit or continue
	*/

	cv::Mat latest_calib;
	int ctr = 0;
	
	switch (filt) {
	case -1: break;
	case 'e':
	case 'E':
		return -1;
		break;
	case '1': extract_chess(frame, dest);
		cv::imshow("Corners", dest);
		return -1;
		break;
	case 's':
	case 'S':
		save_calib_image(frame, dest);
		return 3;
		break;
	case 'c':
	case 'C':
		calibration(frame);
		return 3;
		break;
	case '2':
		detect_chessboard(frame);
		return -1;
		break;
	case '3':
		detect_chessboard(frame);
		project_points(frame,dest);
		cv::imshow("Corners", dest);
		return -1;
		break;
	case '4':
		detect_chessboard(frame);
		draw_axes(frame, dest);
		cv::imshow("Axes", dest);
		return -1;
		break;
	case '5':
		detect_chessboard(frame);
		project_points_circles(frame, dest);
		cv::imshow("Corners", dest);
		return -1;
		break;
	case '6': detect_chessboard(frame);
		
		changeHeight();
		drawVirtualObject(frame, dest);
		cv::imshow("Virtual 3D Object", dest);
		return -1;
		break;
	case '7': detect_chessboard(frame);

		changeHeight();
		drawVirtualObject_coloured_cube(frame, dest);
		cv::imshow("Virtual 3D Object", dest);
		return -1;
		break;
	case '8': detect_chessboard(frame);

		changeHeight();
		drawVirtualObject_coloured_cubes(frame, dest);
		cv::imshow("Virtual 3D Object", dest);
		return -1;
		break;
	case 'q':
	case 'Q': return 2;
	}
}

int extract_chess(cv::Mat& src, cv::Mat& dst){
	/*
	Function to retrieve and draw chessboard corners.

	Input : frame from camera feed, and a dest matrix to store values in called by reference
	Output : modifies the dest matrix
	*/
	std::vector<cv::Point2f> pointBuf;
	bool found;
	dst = src.clone();
	found = findChessboardCorners(src, boardSize, pointBuf);
	//found = findChessboardCorners(src, boardSize, pointBuf, cv::CALIB_CB_FAST_CHECK);
	if (found) {
		cv::Mat srcGray;
		cv::cvtColor(src, srcGray, cv::COLOR_BGR2GRAY);
		cv::cornerSubPix(srcGray, pointBuf, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
		drawChessboardCorners(dst, boardSize, pointBuf, found);

		//To print out the endpoints of the corners detected.
		/*std::cout << "Found " << pointBuf.size()
			<< " corners. First corner coordinates: (" 
			<< pointBuf[0].x << ", " << pointBuf[0].y << ")" << std::std::endl;*/

	}
}

int save_calib_image(cv::Mat& src, cv::Mat calib) {

	/*
	Function to retrieve and save points and corner lists. It also stores the calibration image in the source folder.

	Input : frame from camera feed, and a calib matrix to store the saved image in
	Output : modifies the global points and corner lists
	*/

	std::vector<cv::Point2f> pointBuf;
	bool found;
	calib = src.clone();
	found = findChessboardCorners(src, boardSize, pointBuf);
	
	if (!found) {
		std::cout << "\nChessboard not found. Try again.\n";
	}
	else{
		cv::Mat srcGray;
		cv::cvtColor(src, srcGray, cv::COLOR_BGR2GRAY);
		cv::cornerSubPix(srcGray, pointBuf, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
		drawChessboardCorners(calib, boardSize, pointBuf, found);
		corner_list.push_back(pointBuf); // save the corner coordinates to the corner_list
		for (int i = 0; i < boardSize.height; i++) {
			for (int j = 0; j < boardSize.width; j++) {
				cv::Point3f v = { float(j), float(-i), float(0) };
				point_set.push_back(v); // add the 3D world point for the corner to the point_set
			}
		}
		point_list.push_back(point_set);
		printf("\n Image saved");
		//Save the current frame
		cv::imwrite("calib_img_" + std::to_string(calib_image_counter) + ".jpg", calib);
		calib_image_counter++;
	}
	std::cout << point_list.size() << " " << corner_list.size()<<std::endl;
		std::cout << pointBuf.size() << "   " << corner_list.size() << "    " << point_list.size() <<std::endl;
		point_set.clear();
	cv::imshow("Image considered:", calib);
	return 0;
}


int calibration(cv::Mat src) {


	/*
	Function for calibration. returns the camera matrix, distortion values and other intrinsics. 
	It stores the intrinsics to an xml file in the source folder. Also stores them as global variable
	Only works if there are minimum 5 saved calibration images.

	Input : frame from camera feed
	Output : modifies the global intrinsic matrices and vectors
	*/

	const int MIN_CALIB_IMAGES = 5;

	camera_matrix.at<double>(0, 2) = src.cols / 2;
	camera_matrix.at<double>(1, 2) = src.rows / 2;
	//Camera calibration matrix before calibration
	std::cout << "Camera Matrix before calibration" << std::endl;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			std::cout << camera_matrix.at<double>(i, j) << " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

	if (point_list.size() < MIN_CALIB_IMAGES) {
		std::cout << "Not enough calibration images. Please capture at least " << MIN_CALIB_IMAGES << " images." << std::endl;
		return -1;
	}

	std::cout <<"   " << corner_list.size() << "    " << point_list.size() << std::endl;
	std::vector<cv::Mat> rotations, translations;
	double reprojection_error;
	reprojection_error = cv::calibrateCamera(point_list, corner_list, cv::Size(9, 6), camera_matrix, distortion_coefficients, rotations, translations, cv::CALIB_SAME_FOCAL_LENGTH);

	std::cout << "Before calibration: " << std::endl;
	std::cout << "Camera matrix: " << std::endl << camera_matrix << std::endl;
	std::cout << "Distortion coefficients: " << std::endl << distortion_coefficients << std::endl;

	std::cout << "After calibration: " << std::endl;
	std::cout << "Camera matrix: " << std::endl << camera_matrix << std::endl;
	std::cout << "Distortion coefficients: " << std::endl << distortion_coefficients << std::endl;
	std::cout << "Reprojection error: " << reprojection_error << std::endl;

	// Write the intrinsic parameters to a file
	cv::FileStorage fs("intrinsics.xml", cv::FileStorage::WRITE);
	if (fs.isOpened()) {
		fs << "camera_matrix" << camera_matrix;
		fs << "distortion_coefficients" << distortion_coefficients;
		fs.release();
	}

	return 0;
}

void detect_chessboard(cv::Mat& frame) {


	/*
	Retrieves intrinsics from saved file and computes the rotation and translation vectors.
	Used later for projection of 3d points,

	Input : frame from camera feed, and a calib matrix to store the saved image in
	Output : modifies the global parameters
	*/

	cv::Mat gray;
	cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

	// Detect chessboard corners
	std::vector<cv::Point2f> corners;
	bool found = cv::findChessboardCorners(gray, boardSize, corners,
		cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

	if (found) {
		// Refine corner locations
		cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
			cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

		// Draw chessboard corners
		cv::drawChessboardCorners(frame, boardSize, corners, found);

		// Get 3D world coordinates of chessboard corners
		std::vector<cv::Point3f> objectPoints;
		for (int i = 0; i < boardSize.height; i++) {
			for (int j = 0; j < boardSize.width; j++) {
				objectPoints.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
			}
		}

		// Get camera matrix and distortion coefficients from file
		cv::FileStorage fs("intrinsics.xml", cv::FileStorage::READ);
		fs["camera_matrix"] >> camera_matrix;
		fs["distortion_coefficients"] >> distortion_coefficients;

		// Solve PnP problem to get chessboard pose
		cv::solvePnP(objectPoints, corners, camera_matrix, distortion_coefficients, rvec, tvec);

		// Print rotation and translation vectors
		std::cout << "Rotation vector:" << std::endl << rvec << std::endl;
		std::cout << "Translation vector:" << std::endl << tvec << std::endl;
	}
}

void project_points(cv::Mat& frame, cv::Mat& dst) {


	/*
	Function to plot the borders of the chessboard detected.

	Input : frame from camera feed, and a dst matrix to store the modified image in
	Output : modifies dst called by ref
	*/

	dst = frame.clone();

	// Define 3D world coordinates of the four outside corners of the chessboard
	std::vector<cv::Point3f> objectPoints;
	objectPoints.push_back(cv::Point3f(0.0f, 0.0f, 0.0f));
	objectPoints.push_back(cv::Point3f((boardSize.width - 1) * squareSize, 0.0f, 0.0f));
	objectPoints.push_back(cv::Point3f(0.0f, (boardSize.height - 1) * squareSize, 0.0f));
	objectPoints.push_back(cv::Point3f((boardSize.width - 1) * squareSize, (boardSize.height - 1) * squareSize, 0.0f));

	// Project 3D points to image plane
	std::vector<cv::Point2f> imagePoints;
	cv::projectPoints(objectPoints, rvec, tvec, camera_matrix, distortion_coefficients, imagePoints);

	// Draw projected points on the frame
	cv::line(dst, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 2);
	cv::line(dst, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 2);
	cv::line(dst, imagePoints[1], imagePoints[3], cv::Scalar(255, 0, 0), 2);
	cv::line(dst, imagePoints[2], imagePoints[3], cv::Scalar(255, 255, 0), 2);

}

void draw_axes(cv::Mat& frame, cv::Mat& dst) {

	/*
	Function to plot the axes of the chessboard detected at the corner.

	Input : frame from camera feed, and a dst matrix to store the modified image in
	Output : modifies dst called by ref
	*/


	dst = frame.clone();

	// Define 3D world coordinates of the origin and unit axes
	std::vector<cv::Point3f> objectPoints;
	objectPoints.push_back(cv::Point3f(0.0f, 0.0f, 0.0f));
	objectPoints.push_back(cv::Point3f(squareSize, 0.0f, 0.0f));
	objectPoints.push_back(cv::Point3f(0.0f, squareSize, 0.0f));
	objectPoints.push_back(cv::Point3f(0.0f, 0.0f, squareSize));

	// Project 3D points to image plane
	std::vector<cv::Point2f> imagePoints;
	cv::projectPoints(objectPoints, rvec, tvec, camera_matrix, distortion_coefficients, imagePoints);

	// Draw 3D axes on the frame
	cv::line(dst, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 2);
	cv::line(dst, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 2);
	cv::line(dst, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 2);
}

void project_points_circles(cv::Mat& frame, cv::Mat& dst) {

	/*
	Function to plot circles at the vertices of the chessboard detected.

	Input : frame from camera feed, and a dst matrix to store the modified image in
	Output : modifies dst called by ref
	*/


	dst = frame.clone();

	// Define 3D world coordinates of the four outside corners of the chessboard
	std::vector<cv::Point3f> objectPoints;
	objectPoints.push_back(cv::Point3f(0.0f, 0.0f, 0.0f));
	objectPoints.push_back(cv::Point3f((boardSize.width - 1) * squareSize, 0.0f, 0.0f));
	objectPoints.push_back(cv::Point3f(0.0f, (boardSize.height - 1) * squareSize, 0.0f));
	objectPoints.push_back(cv::Point3f((boardSize.width - 1) * squareSize, (boardSize.height - 1) * squareSize, 0.0f));

	// Project 3D points to image plane
	std::vector<cv::Point2f> imagePoints;
	cv::projectPoints(objectPoints, rvec, tvec, camera_matrix, distortion_coefficients, imagePoints);

	// Draw projected points on the frame
	cv::circle(dst, imagePoints[0], 10, cv::Scalar(0, 0, 255), 2);
	cv::circle(dst, imagePoints[1], 10, cv::Scalar(0, 0, 255), 2);
	cv::circle(dst, imagePoints[2], 10, cv::Scalar(0, 0, 255), 2);
	cv::circle(dst, imagePoints[3], 10, cv::Scalar(0, 0, 255), 2);

}

void drawVirtualObject(cv::Mat& src, cv::Mat& image)
{
	/*
	Function to project a 3D assymetrically cedge coloured pyramid at the center of the chessboard.

	Input : frame from camera feed, and a dst matrix to store the modified image in
	Output : modifies dst called by ref
	*/


	image = src.clone();

	// Calculate the center of the chessboard in 3D space
	cv::Point3f chessboardCenter((boardSize.width - 1) * squareSize / 2.0f, (boardSize.height - 1) * squareSize / 2.0f, 0.0f);

	// Define the 3D points of the virtual object and scale them down by a factor of 0.5
	float scale = 0.05;
	std::vector<cv::Point3f> objectPoints = { cv::Point3f(-1, -1, 0), cv::Point3f(1, -1, 0),
											   cv::Point3f(1, 1, 0), cv::Point3f(-1, 1, 0),
											   cv::Point3f(0, 0, -2) };
	for (int i = 0; i < objectPoints.size(); i++) {
		(objectPoints[i] *= scale) += chessboardCenter;;
	}

	// Define the 2D points of the object corners in the image
	std::vector<cv::Point2f> imagePoints;
	cv::projectPoints(objectPoints, rvec, tvec, camera_matrix, distortion_coefficients, imagePoints);

	// Draw the lines connecting the object corners in the image
	cv::line(image, imagePoints[0], imagePoints[1], cv::Scalar(255, 0, 255), 4, cv::LINE_AA);
	cv::line(image, imagePoints[1], imagePoints[2], cv::Scalar(0, 255, 255), 4, cv::LINE_AA);
	cv::line(image, imagePoints[2], imagePoints[3], cv::Scalar(0, 0, 255), 4, cv::LINE_AA);
	cv::line(image, imagePoints[3], imagePoints[0], cv::Scalar(0, 255, 0), 4, cv::LINE_AA);
	cv::line(image, imagePoints[0], imagePoints[4], cv::Scalar(255, 0, 0), 4, cv::LINE_AA);
	cv::line(image, imagePoints[1], imagePoints[4], cv::Scalar(255, 255, 0), 4, cv::LINE_AA);
	cv::line(image, imagePoints[2], imagePoints[4], cv::Scalar(0, 0, 255), 4, cv::LINE_AA);
	cv::line(image, imagePoints[3], imagePoints[4], cv::Scalar(0, 0, 255), 4, cv::LINE_AA);
}

void drawVirtualObject_coloured_cube(cv::Mat& src, cv::Mat& image)
{

	/*
	Function to project a 3D assymetrically cedge coloured cube at the first square of the chessboard.

	Input : frame from camera feed, and a dst matrix to store the modified image in
	Output : modifies dst called by ref
	*/

	image = src.clone();
	// Define the dimensions of the chessboard
	cv::Size boardSize(8, 6);
	float squareSize = 0.025;

	// Calculate the center of the chessboard in 3D space
	//cv::Point3f chessboardCenter((boardSize.width - 1) * squareSize / 2.0f, (boardSize.height - 1) * squareSize / 2.0f, 0.0f);
	cv::Point3f chessboardCenter(0.5f * squareSize, 0.5f * squareSize, 0.0f);
	// Define the 3D points of the cube vertices
	float scale = 0.01;
	std::vector<cv::Point3f> cubePoints = { cv::Point3f(-1, -1, 1), cv::Point3f(1, -1, 1),
											 cv::Point3f(1, 1, 1), cv::Point3f(-1, 1, 1),
											 cv::Point3f(-1, -1, -1), cv::Point3f(1, -1, -1),
											 cv::Point3f(1, 1, -1), cv::Point3f(-1, 1, -1) };
	for (int i = 0; i < cubePoints.size(); i++) {
		(cubePoints[i] *= scale) += chessboardCenter;;
	}

	// Define the 2D points of the cube vertices in the image
	std::vector<cv::Point2f> imagePoints;
	cv::projectPoints(cubePoints, rvec, tvec, camera_matrix, distortion_coefficients, imagePoints);

	// Define the cube faces using the vertex indices
	std::vector<std::vector<int>> faces = { {0, 1, 2, 3}, {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}, {4, 5, 6, 7} };

	// Define the colors for each face
	std::vector<cv::Scalar> colors = { cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0),
									   cv::Scalar(255, 255, 0), cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 255) };

	// Draw the cube faces using the fillPoly function
	for (int i = 0; i < faces.size(); i++) {
		std::vector<cv::Point> facePoints;
		for (int j = 0; j < faces[i].size(); j++) {
			facePoints.push_back(imagePoints[faces[i][j]]);
		}
		cv::fillPoly(image, std::vector<std::vector<cv::Point>> { facePoints }, colors[i]);
	}

	// Draw the edges of the cube
	for (int i = 0; i < cubePoints.size(); i++) {
		int j = (i + 1) % cubePoints.size();
		cv::line(image, imagePoints[i], imagePoints[j], cv::Scalar(255, 255, 255), 2);
	}
}

void drawVirtualObject_coloured_cubes(cv::Mat& src, cv::Mat& image)
{

	/*
	Function to project a 3D Disco Block/Shield over the chessboard. Primary use to hide the chessboard and the surface it is on.

	Input : frame from camera feed, and a dst matrix to store the modified image in
	Output : modifies dst called by ref
	*/

	image = src.clone();
	// Define the dimensions of the chessboard
	cv::Size boardSize(8, 6);
	float squareSize = 0.1;

	// Define the 3D points of the cube vertices
	float scale = 0.095;
	std::vector<cv::Point3f> cubePoints = { cv::Point3f(-1, -1, 1), cv::Point3f(1, -1, 1),
											 cv::Point3f(1, 1, 1), cv::Point3f(-1, 1, 1),
											 cv::Point3f(-1, -1, -1), cv::Point3f(1, -1, -1),
											 cv::Point3f(1, 1, -1), cv::Point3f(-1, 1, -1) };
	
	// Loop through each square of the chessboard and draw the cube for each square
	for (int i = 0; i < boardSize.height - 1; i++) {
		for (int j = 0; j < boardSize.width - 1; j++) {
			// Calculate the center of the current square in 3D space
			
			cv::Point3f squareCenter(j * squareSize, i * squareSize, 0.0f);

			// Calculate the 3D coordinates of the cube vertices for the current square
			std::vector<cv::Point3f> squareCubePoints(cubePoints.size());
			for (int k = 0; k < cubePoints.size(); k++) {
				(squareCubePoints[k] = cubePoints[k] * scale) += squareCenter;
			}
			// Project the 3D coordinates of the cube vertices onto the 2D image plane
			std::vector<cv::Point2f> imagePoints;
			cv::projectPoints(squareCubePoints, rvec, tvec, camera_matrix, distortion_coefficients, imagePoints);
		
			// Define the cube faces using the vertex indices
			std::vector<std::vector<int>> faces = { {0, 1, 2, 3}, {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}, {4, 5, 6, 7} };

			// Define the colors for each face
			std::vector<cv::Scalar> colors = { cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0),
											   cv::Scalar(255, 255, 0), cv::Scalar(255, 0, 255), cv::Scalar(0, 255, 255) };

			// Draw the cube faces using the fillPoly
			for (int k = 0; k < faces.size(); k++) {
				std::vector<cv::Point> facePoints;
				for (int l = 0; l < faces[k].size(); l++) {
					facePoints.push_back(imagePoints[faces[k][l]]);
				}
				cv::Scalar color(rand() % 256, rand() % 256, rand() % 256);
				cv::fillPoly(image, std::vector<std::vector<cv::Point>> { facePoints }, color);
			}

			// Draw the edges of the cube
			int n = imagePoints.size(); // add this line
			if (n == cubePoints.size()) { // add this line
				for (int k = 0; k < cubePoints.size(); k++) {
					int l = (k + 1) % cubePoints.size();
					cv::line(image, imagePoints[k], imagePoints[l], cv::Scalar(255, 255, 255), 2);
				}
			} // add this line
		}
	}
}



extern std::chrono::time_point<std::chrono::high_resolution_clock> programStart; // Declare programStart as an external variable


void changeHeight() {
	/*
	Function to vary heights every 2 seconds.

	*/
	static auto lastUpdate = std::chrono::high_resolution_clock::now();
	auto now = std::chrono::high_resolution_clock::now();
	auto timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::seconds>(now - lastUpdate);
	if (timeSinceLastUpdate.count() >= 2) { // Only update heights if 2 seconds have passed
		auto timeSinceStart = std::chrono::duration_cast<std::chrono::seconds>(now - programStart);
		std::srand(std::time(nullptr)); // Seed the random number generator
		for (int i = 0; i < 54; ++i) {
			heights[i] = std::rand() % 101; // Initialize height with random value between 0 and 100
			heights[i] += timeSinceStart.count() * (i + 1); // Increase height by (i+1) units per second
		}
		lastUpdate = now; // Update last update time
	}
}
