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
void detectAndDisplay(rs2::frameset data);
void saveImagesToPng();


void remove_background(rs2::video_frame& other, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist);
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

rs2::pipeline_profile profile;

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

    for (auto i = 0; i < 20; ++i)
        pipe.wait_for_frames();


    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset data = pipe.wait_for_frames(); // .apply_filter(printer).apply_filter(color_map);

        rs2::frame color = data.get_color_frame();
        rs2::depth_frame depth_frame = data.get_depth_frame();

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

       /* std::cout << "Color width: " << color.as<rs2::video_frame>().get_width() << "\n";
        std::cout << "Color height: " << color.as<rs2::video_frame>().get_height() << "\n";
        std::cout << "depth_frame width: " << depth_frame.as<rs2::video_frame>().get_width() << "\n";
        std::cout << "depth_frame height: " << depth_frame.as<rs2::video_frame>().get_height() << "\n";*/

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        //Mat image(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

        auto image = frame_to_mat(color);
        auto depth_image = frame_to_mat(depth_frame);

        // Update the window with new data
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

    rs2::frame color_frame = data.get_color_frame();
    rs2::depth_frame depth_frame = data.get_depth_frame();

    auto color_mat = frame_to_mat(color_frame);
    auto depth_mat = frame_to_mat(depth_frame);


    rs2::pointcloud pc;
    pc.map_to(color_frame);
    rs2::points points = pc.calculate(depth_frame);

    /*Export whole scene*/
    points.export_to_ply("ply1.ply", color_frame);

    std::vector<Rect> faces;
    Mat frame_gray;
    Mat crop_color, crop_depth;
    Mat res;
    Mat gray;
    Mat face_threshold;
    string text;
    stringstream sstm;

    /*Rgb to gray*/
    cvtColor(color_mat, frame_gray, COLOR_BGR2GRAY);
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

    float scale = 1;

    // Iterate through all current detected faces
    for (ic = 0; ic < faces.size(); ic++) {

        /*roi_c.x = cvRound(faces[ic].x *scale);
        roi_c.y = cvRound(faces[ic].y * scale);
        roi_c.width = cvRound(faces[ic].width * scale);
        roi_c.height = cvRound(faces[ic].height * scale);

        ac = roi_c.width * roi_c.height; // Get the area of current element (detected face)*/

        roi_b.x = cvRound(faces[ib].x );
        roi_b.y = cvRound(faces[ib].y );
        roi_b.width = cvRound(faces[ib].width * scale);
        roi_b.height = cvRound(faces[ib].height * scale);


        //Advanced ROI implementation
        /*  ab = roi_b.width * roi_b.height; // Get the area of biggest element, at beginning it is same as "current" element

        if (ac > ab)
        {
            ib = ic;
            roi_b.x = cvRound(faces[ib].x * scale);
            roi_b.y = cvRound(faces[ib].y * scale);
            roi_b.width = cvRound(faces[ib].width * scale);
            roi_b.height = cvRound(faces[ib].height * scale);
        }

  
        
        if ((roi_b.x + roi_b.width) >= color_mat.cols)
            roi_b.width = color_mat.cols - roi_b.x - 1;

        if ((roi_b.y + roi_b.height) >= color_mat.rows)
            roi_b.height = color_mat.rows - roi_b.y - 1;

        */

        crop_color = color_mat(roi_b);
        crop_depth = depth_mat(roi_b);

        //SAVE FACE IMAGE
        filename = "faces";
        stringstream ssfn;
        ssfn << filename << "/" << filenumber << ".jpg";
        filename = ssfn.str();
        filenumber++;

        imwrite(filename, crop_color);

        Point pt1(faces[ic].x, faces[ic].y); // Display detected faces on main window
        Point pt2((faces[ic].x + faces[ic].height), (faces[ic].y + faces[ic].width));
        rectangle(color_mat, pt1, pt2, Scalar(0, 255, 0), 2, 8, 0);


        //NOW 3D SCAN SAVING

        // This will be needed later while saving images and maybe NOT
        //resize(crop_color, res, Size(128, 128), 0, 0, INTER_LINEAR); 

        //Thresholding of face// NOT SO GOOD
        //cvtColor(crop, gray, COLOR_BGR2GRAY); // Convert cropped image to Grayscale
            /* 0: Binary
             1: Binary Inverted
             2: Threshold Truncated
             3: Threshold to Zero
             4: Threshold to Zero Inverted
            */
        //threshold(gray, face_threshold, 50, 255, 1);


        /*Center of face*/
        Point center_point(roi_b.x + crop_depth.rows/2, roi_b.y + crop_depth.cols/2);
        auto center_distance = depth_frame.get_distance(center_point.x, center_point.y);

        //Compute neareswt point of facce, better use center distance
        /*
        auto closest_point = 65536;
        for (int y = 0; y < crop_depth.rows; y++) {
            for (int x = 0; x < crop_depth.cols; x++) {
                auto dep = crop_depth.at<uint16_t>(y, x);

               //std::cout << dep << " ";
                if (dep && dep < closest_point)
                    closest_point = dep;
            }
        }
        */

       //Remove color background by distance
       /* for (int y = 0; y < crop.rows; y++) {
            for (int x = 0; x < crop.cols; x++) {
                Vec3b& col = crop.at<Vec3b>(y, x);
                auto dep = crop_depth.at<uint16_t>(y, x);

               if (!dep || dep > (closest_point + 100)) {
                //if(dep > 500) {
                    col[0] = 255;
                    col[1] = 255;
                    col[2] = 255;
                }

                // set pixel
                //image.at<Vec3b>(Point(x,y)) = color;
                //if you copy value
            }
        }*/

        float depth_scale = get_depth_scale(profile.get_device());
        rs2_stream align_to = find_stream_to_align(profile.get_streams());
        rs2::align align(align_to);


        float depth_clipping_distance = 1.0;

        if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
        {
            //If the profile was changed, update the align object, and also get the new device's depth scale
            profile = pipe.get_active_profile();
            align_to = find_stream_to_align(profile.get_streams());
            align = rs2::align(align_to);
            depth_scale = get_depth_scale(profile.get_device());
        }


        //Get processed aligned frame
        auto processed = align.process(data);

        // Trying to get both other and aligned depth frames
        rs2::video_frame other_frame = processed.first(align_to);
        rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

        //If one of them is unavailable, continue iteration
        if (!aligned_depth_frame || !other_frame)
        {
            continue;
        }
        // Passing both frames to remove_background so it will "strip" the background
        // NOTE: in this example, we alter the buffer of the other frame, instead of copying it and altering the copy
        //       This behavior is not recommended in real application since the other frame could be used elsewhere
        remove_background(other_frame, aligned_depth_frame, depth_scale, depth_clipping_distance);

        rs2::colorizer color_map;
        imshow("aligned", frame_to_mat(other_frame));

        rs2::pointcloud pc2;
        pc2.map_to(other_frame);
        rs2::points points2 = pc2.calculate(aligned_depth_frame);

        

        points2.export_to_ply("ply_aligned.ply", other_frame);

        // Taking dimensions of the window for rendering purposes
        float w = static_cast<float>(crop_color.rows);
        float h = static_cast<float>(crop_color.cols);

        // At this point, "other_frame" is an altered frame, stripped form its background
        // Calculating the position to place the frame in the window
        rect altered_other_frame_rect{ 0, 0, w, h };
        altered_other_frame_rect = altered_other_frame_rect.adjust_ratio({ static_cast<float>(other_frame.get_width()),static_cast<float>(other_frame.get_height()) });

        // Render aligned image
        //renderer.render(other_frame, altered_other_frame_rect);

        // The example also renders the depth frame, as a picture-in-picture
        // Calculating the position to place the depth frame in the window
        rect pip_stream{ 0, 0, w / 5, h / 5 };
        pip_stream = pip_stream.adjust_ratio({ static_cast<float>(aligned_depth_frame.get_width()),static_cast<float>(aligned_depth_frame.get_height()) });
        pip_stream.x = altered_other_frame_rect.x + altered_other_frame_rect.w - pip_stream.w - (std::max(w, h) / 25);
        pip_stream.y = altered_other_frame_rect.y + (std::max(w, h) / 25);

        // Render depth (as picture in pipcture)
       // renderer.upload(c.process(aligned_depth_frame));
       // renderer.show(pip_stream);

        // Using ImGui library to provide a slide controller to select the depth clipping distance
       // ImGui_ImplGlfw_NewFrame(1);
       // render_slider({ 5.f, 0, w, h }, depth_clipping_distance);
       // ImGui::Render();
    }

    // Show image
    sstm << "Crop area size: " << roi_b.width << "x" << roi_b.height << " Filename: " << filename;
    text = sstm.str();

    putText(color_mat, text, Point(30, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 0, 255), 1, LINE_AA);
    imshow("original", color_mat);

    if (!crop_color.empty())
    {
        imshow("detected", crop_color);
    }
    else
        destroyWindow("detected");

    if (!crop_depth.empty())
    {
        imshow("depth_crop", crop_depth);
    }
    else
        destroyWindow("depth_crop");
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


void remove_background(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist)
{
    const uint16_t* p_depth_frame_to_read = reinterpret_cast<const uint16_t*>(depth_frame.get_data());

    uint16_t* p_depth_frame = reinterpret_cast<uint16_t*>(const_cast<void*>(depth_frame.get_data()));

    uint8_t* p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

    int width = other_frame.get_width();
    int height = other_frame.get_height();
    int other_bpp = other_frame.get_bytes_per_pixel();


    int deep_bpp = depth_frame.get_bytes_per_pixel();

    std::cout << "other_bpp " << other_bpp << endl;
    std::cout << "deep_bpp " << deep_bpp << endl;

#pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
    for (int y = 0; y < height; y++)
    {
        auto depth_pixel_index = y * width;
        for (int x = 0; x < width; x++, ++depth_pixel_index)
        {
            // Get the depth value of the current pixel
            auto pixels_distance = depth_scale * p_depth_frame_to_read[depth_pixel_index];

            //std::cout << "depth: " << p_depth_frame_to_read[depth_pixel_index] << std::endl;

            // Check if the depth value is invalid (<=0) or greater than the threashold
            if (pixels_distance <= 0.f || pixels_distance > clipping_dist)
            {
                // Calculate the offset in other frame's buffer to current pixel
                auto offset = depth_pixel_index * other_bpp;
                auto offset_d = depth_pixel_index * deep_bpp;

                p_depth_frame[depth_pixel_index] = 0;
                // Set pixel to "background" color (0x999999)
                std::memset(&p_other_frame[offset], 0x21, other_bpp);
                //std::memset(&p_depth_frame[depth_pixel_index], 0x0000, 1);
            }
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
    imshow("img", frame);
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
