
//Aug.h -- header file for storing function prototypes to be referred in Augment.cpp
// Function description comments have been included in Aug.cpp with the function definition

#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <string>
#include <numeric>

int key_check(cv::Mat& frame, cv::Mat& dest, char filt);
//Task 1
int extract_chess(cv::Mat& src, cv::Mat& dst);
//Task 2
int save_calib_image(cv::Mat& src, cv::Mat calib);
//Task 3
int calibration(cv::Mat src);
//Task 4
void detect_chessboard(cv::Mat& frame);
//Task 5
void project_points(cv::Mat& frame, cv::Mat& dst);
void draw_axes(cv::Mat& frame, cv::Mat& dst);
void project_points_circles(cv::Mat& frame, cv::Mat& dst);
//Task 6
void drawVirtualObject(cv::Mat& src, cv::Mat& image);
//Task 7 stored in different program
//Extensions
void drawVirtualObject_coloured_cube(cv::Mat& src, cv::Mat& image);
void drawVirtualObject_coloured_cubes(cv::Mat& src, cv::Mat& image);
void changeHeight();