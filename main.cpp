// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"


cv::String face_cascade_name = "haarcascade_frontalface_default.xml";

using namespace std;
using namespace cv;

CascadeClassifier face_cascade;


void detectAndDisplay(Mat frame);

int main(int argc, char* argv[]) try
{
    //face_cascade.load(face_cascade_name);
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();


    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);




    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
        rs2::frame color = data.get_color_frame();

        rs2::depth_frame depth_frame = data.get_depth_frame();
        const void* ptr = depth_frame.get_data();



        cout << "Deepth Heiht" << depth_frame.get_height();
        cout << "Deepth Width" << depth_frame.get_width();

        cout << "Data size" << color.get_data_size();

        /* for (auto i = 0; i < color.get_data_size(); i++) {
             for (auto j = 0; j < color.get_height(); j++) {
                 cout << depth_frame.get_distance(i, j)<<"  ";
             }
             cout << "\n";
         }*/

        getchar();


        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();


        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

        // Update the window with new data
        imshow(window_name, image);


        // detectAndDisplay(image);

        // int c = waitKey(10);
           //  if ((char)c == 'c') { break; }
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

/*
void detectAndDisplay(Mat frame)
{
    std::vector<Rect> faces;
    Mat frame_gray;

    cvtColor(frame, frame_gray, CV_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);

    //-- Detect faces
    face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));

    for (size_t i = 0; i < faces.size(); i++)
    {
        Point center(faces[i].x + faces[i].width * 0.5, faces[i].y + faces[i].height * 0.5);
        ellipse(frame, center, Size(faces[i].width * 0.5, faces[i].height * 0.5), 0, 0, 360, Scalar(255, 0, 255), 4, 8, 0);

        Mat faceROI = frame_gray(faces[i]);
        std::vector<Rect> eyes;

    }
    //-- Show what you got
    //imshow("Image", frame);
}
*/