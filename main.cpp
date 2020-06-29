//
// Created by Zahorack (Oliver Hollý)
//

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

#include "stereoscan.h"
#include "AudioTrigger.h"
#include <windows.h>



int main(int argc, char* argv[]) try
{
    rs2::pipeline pipe;
    Stereoscan stereoscan(pipe);
    AudioTrigger audioTrigger;

    for (auto i = 0; i < 20; ++i) {
        pipe.wait_for_frames();
    }


    //check usb cameras
    /*cv::VideoCapture camera;
    int device_counts = 0;
    cv::Mat fr;
    while (true) {
        camera.set(cv::CAP_PROP_FORMAT, CV_16U);
        if (!camera.open(device_counts++)) {
            break;
        }
        //camera.read(fr);
        //std::cout<<"Device: "<<device_counts << "  Size " << fr.size << std::endl;
    }
    camera.release();
    std::cout << "devices count : " << device_counts - 1 << std::endl;*/


    /*cv::VideoCapture cap(1);
    cap.set(cv::CAP_PROP_FORMAT, CV_16U);

 
    /*if (cap.isOpened() == false)
    {
        std::cout << "Cannot open the video camera" << std::endl;
        std::cin.get(); //wait for any key press
    }*/


    cv::Mat rgb_frame;

    while (cv::waitKey(1) < 0) 
    {

        audioTrigger.update();

        if (audioTrigger.check(Events::Pass)) {
            std::cout << "Passing\n";
            stereoscan.update();
            audioTrigger.clear(Events::Pass);
        }

        if (audioTrigger.check(Events::Warning)) {
            std::cout << "Warning\n";

            audioTrigger.clear(Events::Warning);
        }

        //bool bSuccess = cap.read(rgb_frame);
        //cv::imshow("rgb", rgb_frame);

        //stereoscan.update();


    }
    return EXIT_SUCCESS;
}

catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}

catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

