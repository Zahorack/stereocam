// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

#include "stereoscan.h"

rs2::pipeline pipe;

int main(int argc, char* argv[]) try
{

    Stereoscan stereoscan(pipe);

    //for (auto i = 0; i < 20; ++i)
     //pipe.wait_for_frames();



    while (cv::waitKey(1) < 0) 
    {
        rs2::frameset data = pipe.wait_for_frames();

        static int last_frame_number = 0;
        if (data.get_frame_number() == last_frame_number)
            continue;
        last_frame_number = data.get_frame_number();


        stereoscan.update(data);

        //cv::imshow("img", frame_to_mat(data.get_color_frame()));
         //int c = cv::waitKey(10);
            // if ((char)c == 'c') { break; }
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

