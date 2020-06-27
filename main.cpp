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

#include <fstream>             
#include <iostream>             
#include <sstream>              
#include <algorithm>



#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

cv::String face_cascade_name = "haarcascade_frontalface_default.xml";


using namespace std;
using namespace cv;


// Declare RealSense pipeline, encapsulating the actual device and sensors
rs2::pipeline pipe;

CascadeClassifier face_cascade;

rs2::pipeline_profile profile;

void faceDetection(Mat frame);
void detectAndDisplay(rs2::frameset data);
void saveImagesToPng();


void remove_background(rs2::video_frame& other, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist, cv::Rect roi);
float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);


auto filenumber = 0;
string filename;

struct float2 { float x, y; };
struct rect
{
    float x, y;
    float w, h;

    // Create new rect within original boundaries with give aspect ration
    rect adjust_ratio(float2 size) const
    {
        auto H = static_cast<float>(h), W = static_cast<float>(h)* size.x / size.y;
        if (W > w)
        {
            auto scale = w / W;
            W *= scale;
            H *= scale;
        }

        return{ x + (w - W) / 2, y + (h - H) / 2, W, H };
    }
};



int main(int argc, char* argv[]) try
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    rs2::rates_printer printer;

    // Start streaming with default recommended configuration
    profile = pipe.start();

    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    // Capture 30 frames to give autoexposure, etc. a chance to settle


    if (!face_cascade.load(face_cascade_name)) {
        cout << "--(!)Error loading face cascade\n";
    }
  //  for (auto i = 0; i < 20; ++i)
      //  pipe.wait_for_frames();

    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset data = pipe.wait_for_frames(); // .apply_filter(printer).apply_filter(color_map);

        static int last_frame_number = 0;
        if (data.get_frame_number() == last_frame_number)
            continue;
        last_frame_number = data.get_frame_number();


        //if(event from theraml camera)
        //    saveImagesToPng();

        //imshow(window_name, image);

        //faceDetection(image);
  
        detectAndDisplay(data);

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

void detectAndDisplay(rs2::frameset data)
{

    rs2::video_frame color_frame = data.get_color_frame();
    rs2::depth_frame depth_frame = data.get_depth_frame();

    auto color_mat = frame_to_mat(color_frame);
    auto depth_mat = frame_to_mat(depth_frame);


    std::vector<Rect> faces;
    Mat gray_mat;
    Mat crop_color, crop_depth;
    Mat res;
    Mat gray;
    Mat face_threshold;
    string text;
    stringstream sstm;

    /*Rgb to gray*/
    cvtColor(color_mat, gray_mat, COLOR_BGR2GRAY);
    equalizeHist(gray_mat, gray_mat);

    // Detect faces
    face_cascade.detectMultiScale(gray_mat, faces, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(30, 30));

    // Set Region of Interest
    cv::Rect roi_b;

    float scale = 1.4;
    float height_scale = 1.3;

    // Iterate through all current detected faces
    for (auto ic = 0; ic < faces.size(); ic++) {


        /*Center of face*/
        Point center_point(faces[ic].x + faces[ic].width / 2, faces[ic].y + faces[ic].height / 2);
        auto center_distance = depth_frame.get_distance(center_point.x, center_point.y);

        roi_b.width = cvRound(faces[ic].width * scale);
        roi_b.height = cvRound(faces[ic].height * scale* height_scale);
        roi_b.x = cvRound(center_point.x - roi_b.width /2);
        roi_b.y = cvRound(center_point.y - roi_b.height /2);


        if ((roi_b.x + roi_b.width) >= color_mat.cols)
            roi_b.width = color_mat.cols - roi_b.x;

        if ((roi_b.y + roi_b.height) >= color_mat.rows)
            roi_b.height = color_mat.rows - roi_b.y;


        roi_b.x = clamp<int>(roi_b.x, 0, color_mat.cols-1);
        roi_b.y = clamp<int>(roi_b.y, 0, color_mat.rows-1);
        roi_b.width = clamp<int>(roi_b.width, 0, color_mat.cols-1);
        roi_b.height = clamp<int>(roi_b.height, 0, color_mat.rows-1);

        Point pt1(roi_b.x, roi_b.y); // Display detected faces on main window
        Point pt2((roi_b.x + roi_b.width), (roi_b.y + roi_b.height));
        rectangle(color_mat, pt1, pt2, Scalar(0, 255, 0), 2, 8, 0);

        float depth_scale = get_depth_scale(profile.get_device());
        rs2_stream align_to = find_stream_to_align(profile.get_streams());
        rs2::align aligninig(align_to);
        //NOW 3D SCAN SAVING
        float depth_clipping_distance = center_distance + 0.1;

        if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
        {
            //If the profile was changed, update the align object, and also get the new device's depth scale
            profile = pipe.get_active_profile();
            align_to = find_stream_to_align(profile.get_streams());
            aligninig = rs2::align(align_to);
            depth_scale = get_depth_scale(profile.get_device());
        }

        //Get processed aligned frame
        auto processed = aligninig.process(data);

        // Trying to get both other and aligned depth frames
        rs2::video_frame other_frame = processed.first(align_to);
        rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

        //If one of them is unavailable, continue iteration
        if (!aligned_depth_frame || !other_frame) {
            continue;
        }
        remove_background(other_frame, aligned_depth_frame, depth_scale, depth_clipping_distance, roi_b);

        //rs2::colorizer color_map;
        //cv::imshow("aligned", frame_to_mat(other_frame));



        rs2::pointcloud pc;
        pc.map_to(other_frame);
        rs2::points points = pc.calculate(aligned_depth_frame);

        //SAVING
        filename = "faces";
        stringstream ssf;
        ssf << filename << "/" << filenumber << ".ply";
        points.export_to_ply(ssf.str(), color_frame);

        auto other_mat = frame_to_mat(other_frame);
        stringstream ssfn;
        ssfn << filename << "/" << filenumber << ".jpg";
        filename = ssfn.str();
        imwrite(filename, other_mat(roi_b));
        filenumber++;
    }

    // Show image
    /*sstm << "Crop area size: " << roi_b.width << "x" << roi_b.height << " Filename: " << filename;
    text = sstm.str();
    putText(color_mat, text, Point(30, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 0, 255), 1, LINE_AA);
    */
    cv::imshow("original", color_mat);
}

float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}


rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)         //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if (!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}


void remove_background(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist, cv::Rect roi)
{
    const uint16_t* p_depth_frame_to_read = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
    uint16_t* p_depth_frame = reinterpret_cast<uint16_t*>(const_cast<void*>(depth_frame.get_data()));

    uint8_t* p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

    int width = other_frame.get_width();
    int height = other_frame.get_height();
    int other_bpp = other_frame.get_bytes_per_pixel();
    int deep_bpp = depth_frame.get_bytes_per_pixel();


#pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
    for (int y = 0; y < height; y++)
    {
        auto depth_pixel_index = y * width;
        for (int x = 0; x < width; x++, ++depth_pixel_index)
        {
            auto pixels_distance = depth_scale * p_depth_frame_to_read[depth_pixel_index];
            // Calculate the offset in other frame's buffer to current pixel
            auto offset = depth_pixel_index * other_bpp;

            if (x > roi.x&& x < (roi.x + roi.width) && y > roi.y&& y < (roi.y + roi.height)) {
                if (pixels_distance <= 0.f || pixels_distance > clipping_dist)
                {
                    p_depth_frame[depth_pixel_index] = 0;
                    // Set pixel to "background" color (0x999999)
                    std::memset(&p_other_frame[offset], 255, other_bpp);
                }
            }
            else {
                p_depth_frame[depth_pixel_index] = 0;
                // Set pixel to "background" color (0x999999)
                //std::memset(&p_other_frame[offset], 255, other_bpp);
            }
            // Check if the depth value is invalid (<=0) or greater than the threashold
        }
    }
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
    cv::imshow("img", frame);
}


bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}
