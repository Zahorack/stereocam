//
// Created by Zahorack (Oliver Hollý)
//

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

#include "stereoscan.h"
#include "AudioTrigger.h"
#include "Logger.h"
#include <windows.h>
#include "cv-helpers.h"


static int countConnectedCameras();
static int getIndexOfFirstCameraWithResolution(int width, int height);




int main(int argc, char* argv[]) try
{


    int device_counts = countConnectedCameras();

    AudioTrigger audioTrigger;

    // If there is Thermal camera and 2 intel relasense, than use only them 
    //TODO: In NUC relase use device_counts >=3
    if (device_counts >= 3) {
        rs2::pipeline pipe;
        Stereoscan stereoscan(pipe);

        for (auto i = 0; i < 20; ++i) {
            pipe.wait_for_frames();
        }

        while (cv::waitKey(1) < 0)
        {
            audioTrigger.update();

            if (audioTrigger.check(Events::Pass)) {
                std::cout << "Passing\n";
                audioTrigger.clear(Events::Pass);
            }
            if (audioTrigger.check(Events::Warning)) {
                stereoscan.update();
                audioTrigger.clear(Events::Warning);
            }
        }
    }
    else { //else, use RGB camera from thermal module

        cv::VideoCapture cap(getIndexOfFirstCameraWithResolution(480, 640));
        cv::Mat frame;
        Logger logger;

        while (cv::waitKey(1) < 0)
        {
            audioTrigger.update();

            if (audioTrigger.check(Events::Pass)) {
                std::cout << "Passing\n";
                audioTrigger.clear(Events::Pass);
            }
            if (audioTrigger.check(Events::Warning)) {

                FaceDetection faceDetection;
                int iterations = 10;
                do {
                    bool bSuccess = cap.read(frame);

                    std::cout << "RGB capture\n";
                    faceDetection.update(frame);

                } while (!faceDetection.available() && iterations--);

                auto faces = faceDetection.crops(1.4, 1.3);

                for (int i = 0; i < faces.size(); i++) {

                    cv::Mat faceImage = frame(faces[i]);
                    logger.updateRGB(faceImage);
                }
                audioTrigger.clear(Events::Warning);
            }

            //cv::imshow("frame", frame);
            audioTrigger.clear(Events::Warning);
        }
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




static int countConnectedCameras() {

    cv::VideoCapture camera;
    int device_counts = 0;
    while (true) {
        if (!camera.open(device_counts)) {
            break;
        }
        else
            device_counts++;
    }
    camera.release();
    std::cout << "devices count : " << device_counts << std::endl;

    return device_counts;
}


static int getIndexOfFirstCameraWithResolution(int height, int width) {

    cv::VideoCapture camera;
    int device_counts = 0;
    int device_id = -1;

    while (true) {
        cv::Mat frame;
        if (!camera.open(device_counts)) {
            break;
        }
        else {
            camera.read(frame);

            std::cout << "Frame size: " << frame.size <<"  heiht:  "<<frame.size().height<< std::endl;
            if (frame.size().height == height && frame.size().width == width) {
                device_id = device_counts;
                break;
            }
            device_counts++;
        }
    }
    camera.release();
    std::cout << "devices id : " << device_id << std::endl;

    return device_id;
}
