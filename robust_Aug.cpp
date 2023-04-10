// Arjun Rajeev Warrier
// Spring 2023 CS 5330
// Project 4: Calibration and Augmented Reality 

// robust_Aug.cpp : This is the program to find Harris Corner features. This is done to exhibit a robust feature detector.

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <iostream>

int main()
{
    cv::VideoCapture* vid;


    // open the video device
    vid = new cv::VideoCapture(0);
    if (!vid->isOpened()) {
        printf("Unable to open video device\n");
        return(-1);
    }


    // get some properties of the image
    cv::Size refS((int)vid->get(cv::CAP_PROP_FRAME_WIDTH),
        (int)vid->get(cv::CAP_PROP_FRAME_HEIGHT));
    printf("Expected size: %d %d\n", refS.width, refS.height);




    // Initialisation of some variables
    cv::namedWindow("Video", 1); // identifies a window
    cv::Mat frame, dest;
    char QK = 1, key = -1, filt = 0;
    int bar_flag = 0;
    std::string caption;


    // Main Video Loop for frame by frame
    while (QK) {


        *vid >> frame; // get a new frame from the camera, treat as a stream
        if (frame.empty()) {
            printf("frame is empty\n");
            break;
        }
        cv::imshow("Video", frame);


        // see if there is a waiting keystroke
        key = cv::waitKey(10);
        if (key != -1) { // Only enters loop if a key is pressed and resets some flags
            filt = key;
            bar_flag = 0;
            cv::destroyAllWindows();
        }

            //Converting to greyscale
            cv::Mat gray,dest;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            dest = frame.clone();


            // Detect Harris corner features
            cv::Mat corners = cv::Mat::zeros(gray.size(), CV_32FC1);
            cv::cornerHarris(gray, corners, 2, 3, 0.04, cv::BORDER_DEFAULT);
            //cv::imshow("Gray",gray);



            // Adjust this to control the number of corners detected
            double min, max;
            cv::minMaxLoc(corners, &min, &max);
            float threshold = 0.01 * max;

            // Draw circles around the Harris corners
            for (int y = 0; y < corners.rows; y++)
            {
                for (int x = 0; x < corners.cols; x++)
                {
                    if (corners.at<float>(y, x) > threshold)
                    {
                        cv::circle(dest, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), 2);
                    }
                }
            }
            cv::imshow("Harris Corners", dest);
        }
    


    delete vid;
    return(0);
}


// End of program