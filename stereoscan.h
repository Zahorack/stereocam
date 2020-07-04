//
// Created by Zahorack (Oliver Hollý)
//

#pragma once


#ifndef STEREOSCAN
#define STEREOSCAN

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include "cv-helpers.h"

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <fstream>             
#include <iostream>             
#include <sstream>              
#include <algorithm>

#include "FaceDetection.h"


class Stereoscan {

	rs2::frameset m_data;
    rs2::pipeline m_pipeline;
    rs2::pipeline_profile m_profile;

    float depth_scale;
    rs2_stream align_to;
    rs2::align aligninig;

    int filenumber = 0;

    bool is_enabled = false;

public:

	Stereoscan(rs2::pipeline& pipe);
    ~Stereoscan();

    void start();

    void initialize();
    void update();
    void process(FaceDetection faceDetection);

    void save();

    void enable() { is_enabled = true;};
};



#endif // !STEREOSCAN
