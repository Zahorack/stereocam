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
#include "cv-helpers.h"

namespace Cameras {
    enum Enum {
        Stereo = 0,
        RGB,

        Size
    };
}

Cameras::Enum camera_id = Cameras::Enum::RGB;

long scale(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


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

    while (true) {
        cv::Mat fr;
        //camera.set(cv::CAP_PROP_FORMAT, CV_16U);
        if (!camera.open(device_counts++)) {
            break;
        }
       // camera.read(fr);
       // std::cout<<"Device: "<<device_counts << "  Size " << fr.size << std::endl;
    }
    camera.release();
    std::cout << "devices count : " << device_counts - 1 << std::endl;
    
    getchar();*/

    //cv::VideoCapture cap(0);
  

    while (cv::waitKey(1) < 0) 
    {

        /*
        audioTrigger.update();

        if (audioTrigger.check(Events::Pass)) {
            std::cout << "Passing\n";

            audioTrigger.clear(Events::Pass);
        }

        if (audioTrigger.check(Events::Warning)) {
            std::cout << "Warning\n";

            if(camera_id == Cameras::Enum::Stereo)
                stereoscan.update();

            if (camera_id == Cameras::Enum::RGB) {
                FaceDetection faceDetection;

                do {
                    bool bSuccess = cap.read(rgb_frame);

                    std::cout << "RGB capture\n";
                    faceDetection.update(rgb_frame);

                } while (!faceDetection.available());

                auto faces = faceDetection.crops(1.4, 1.3);

                for (int i = 0; i < faces.size(); i++) {

                    std::string filename = "";
                    static int filenumber = 0;
                    std::stringstream ssfn;
                    ssfn << "faces/RGB/" << filenumber << ".jpg";
                    filename = ssfn.str();
                    cv::imwrite(filename, rgb_frame(faces[i]));
                    filenumber++;
                }
            }

            audioTrigger.clear(Events::Warning);
        }*/
        


        stereoscan.update();


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

