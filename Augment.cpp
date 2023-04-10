// Arjun Rajeev Warrier
// Spring 2023 CS 5330
// Project 4: Calibration and Augmented Reality 

// Augment.cpp : This file contains the 'main' function. Program execution begins and ends there. Modify address to change camera port.
//

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <iostream>
#include "Aug.h"

int main()
{
    cv::VideoCapture* vid;


    // open the video device
    std::string address = "http://10.110.17.157:4747/video";
    vid = new cv::VideoCapture(address);
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


    // Printing the Key Options
    std::cout << "\n ------Menu------\n"
        << "1. Detect and display corners\n" << "2. Get rotation and translattion vectors"
        << "\n3. Draw border of chessboard" << "\n4. Draw axes"
        << "\n5. Draw circles at vertices"
        << "\n6. Project a 3D assyetrically edge coloured square pyramid"
        << "\n7. Project a 3D Cube on the first square of the chessboard"
        << "\n8. Project a 3D DISCO Baarrier/Shield to mask the chessboard and surface its on (modify parameters to adjust size in function)"
        << "\nS. Save a calibration image."
        << "\nC. Calibrate camera";
    printf("\n\n Enter option: ");


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

        // switch case to check Key pressed and display output
        switch (key_check(frame, dest, filt)) {

        case -1: break;    // if no key is pressed or if keys "g,h,b,x,y,m,l,c,n,e" are pressed
            // function calls happen in the key_check function


        case 2: cv::destroyAllWindows();  // if key 'q' is pressed ----- Stop runtime and close all windows
            return 0;

        case 3: filt = -1;
            break;
        }

    }   // end of switch case
    delete vid;
    return(0);
}


// End of program