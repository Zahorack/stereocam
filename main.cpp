// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include "cv-helpers.h"

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

#include <fstream>              // File IO
#include <iostream>             // Terminal IO
#include <sstream>              // Stringstreams


#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

cv::String face_cascade_name = "haarcascade_frontalface_default.xml";


using namespace std;
using namespace cv;


// Declare RealSense pipeline, encapsulating the actual device and sensors
rs2::pipeline pipe;

CascadeClassifier face_cascade;


void faceDetection(Mat frame);
void detectAndDisplay(Mat frame);
void saveImagesToPng();


auto filenumber = 0;
string filename;

int main(int argc, char* argv[]) try
{

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    rs2::rates_printer printer;


    // Start streaming with default recommended configuration
    pipe.start();


    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    // Capture 30 frames to give autoexposure, etc. a chance to settle


    if (!face_cascade.load(face_cascade_name)) {
        cout << "--(!)Error loading face cascade\n";
    }

    for (auto i = 0; i < 20; ++i)
        pipe.wait_for_frames();


    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset data = pipe.wait_for_frames().apply_filter(printer).apply_filter(color_map);

        
        rs2::frame color = data.get_color_frame();

        static int last_frame_number = 0;
        if (color.get_frame_number() == last_frame_number)
            continue;
        last_frame_number = color.get_frame_number();


        //if(event from theraml camera)
        //    saveImagesToPng();

       /* rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
        rs2::depth_frame depth_frame = data.get_depth_frame();
        const void* ptr = depth_frame.get_data();
        */
        /* for (auto i = 0; i < color.get_data_size(); i++) {
             for (auto j = 0; j < color.get_height(); j++) {
                 cout << depth_frame.get_distance(i, j)<<"  ";
             }
             cout << "\n";
         }*/

        //getchar();

        // Query frame size (width and height)
        const int w = color.as<rs2::video_frame>().get_width();
        const int h = color.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        //Mat image(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        auto image = frame_to_mat(color);

        // Update the window with new data
        imshow(window_name, image);

        //faceDetection(image);
        detectAndDisplay(image);

         int c = waitKey(10);
             if ((char)c == 'c') { break; }
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


void faceDetection(Mat frame)
{
    std::vector<Rect> faces;
    Mat frame_gray;

    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);

    //-- Detect faces
    face_cascade.detectMultiScale(frame_gray, faces, 1.5, 5, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));
    //face_cascade.detectMultiScale(frame_gray, faces);

    auto scale = 1;

    for (size_t i = 0; i < faces.size(); i++)
    {
        Point center(faces[i].x + faces[i].width * 0.5, faces[i].y + faces[i].height * 0.5);
        //ellipse(frame, center, Size(faces[i].width * 0.5, faces[i].height * 0.5), 0, 0, 360, Scalar(255, 0, 255), 4, 8, 0);

        rectangle(frame, Point(cvRound(faces[i].x * scale), cvRound(faces[i].y * scale)), Point(cvRound((faces[i].x +
            faces[i].width - 1) * scale), cvRound((faces[i].y + faces[i].height - 1) * scale)), Scalar(255, 0, 255), 3, 8, 0);
    

        Mat faceROI = frame_gray(faces[i]);
    }
    imshow("img", frame);
}



void saveImagesToPng() {

    for (auto&& frame : pipe.wait_for_frames()) {
        if (auto vf = frame.as<rs2::video_frame>()) {
            auto stream = frame.get_profile().stream_type();
            // Use the colorizer to get an rgb image for the depth stream
            rs2::colorizer color_map;
            if (vf.is<rs2::depth_frame>()) vf = color_map.process(frame);

            // Write images to disk
            std::stringstream png_file;
            png_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name() << ".png";
            stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
                vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
            std::cout << "Saved " << png_file.str() << std::endl;
        }
    }
}

void detectAndDisplay(Mat frame)
{
    std::vector<Rect> faces;
    Mat frame_gray;
    Mat crop;
    Mat res;
    Mat gray;
    string text;
    stringstream sstm;

    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);

    // Detect faces
    face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));

    // Set Region of Interest
    cv::Rect roi_b;
    cv::Rect roi_c;

    size_t ic = 0; // ic is index of current element
    int ac = 0; // ac is area of current element

    size_t ib = 0; // ib is index of biggest element
    int ab = 0; // ab is area of biggest element

    for (ic = 0; ic < faces.size(); ic++) // Iterate through all current elements (detected faces)

    {
        roi_c.x = faces[ic].x;
        roi_c.y = faces[ic].y;
        roi_c.width = (faces[ic].width);
        roi_c.height = (faces[ic].height);

        ac = roi_c.width * roi_c.height; // Get the area of current element (detected face)

        roi_b.x = faces[ib].x;
        roi_b.y = faces[ib].y;
        roi_b.width = (faces[ib].width);
        roi_b.height = (faces[ib].height);

        ab = roi_b.width * roi_b.height; // Get the area of biggest element, at beginning it is same as "current" element

        if (ac > ab)
        {
            ib = ic;
            roi_b.x = faces[ib].x;
            roi_b.y = faces[ib].y;
            roi_b.width = (faces[ib].width);
            roi_b.height = (faces[ib].height);
        }

        crop = frame(roi_b);
        resize(crop, res, Size(128, 128), 0, 0, INTER_LINEAR); // This will be needed later while saving images
        cvtColor(crop, gray, COLOR_BGR2GRAY); // Convert cropped image to Grayscale

        // Form a filename
        filename = "faces";
        stringstream ssfn;
        ssfn << filename << "/" << filenumber << ".jpg";
        filename = ssfn.str();
        filenumber++;

        imwrite(filename, crop);

        Point pt1(faces[ic].x, faces[ic].y); // Display detected faces on main window
        Point pt2((faces[ic].x + faces[ic].height), (faces[ic].y + faces[ic].width));
        rectangle(frame, pt1, pt2, Scalar(0, 255, 0), 2, 8, 0);
    }

    // Show image
    sstm << "Crop area size: " << roi_b.width << "x" << roi_b.height << " Filename: " << filename;
    text = sstm.str();

    putText(frame, text, Point(30, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 0, 255), 1, LINE_AA);
    imshow("original", frame);

    if (!crop.empty())
    {
        imshow("detected", crop);
    }
    else
        destroyWindow("detected");
}