//
// Created by Zahorack (Oliver Hollý)
//

#pragma once
#include <windows.h>
#include <iostream>
#include <opencv2/opencv.hpp> 
#include <librealsense2/rs.hpp>

static const std::string PATH = "E:\\data\\";

class Logger {

	std::string m_date;
	std::string m_time;

public:
	Logger();
	~Logger();


	void updateRGB(cv::Mat &frame, int num);
	void updateRGB_FACES(cv::Mat& frame, int num);
	void update3D(rs2::video_frame& color, rs2::depth_frame& depth, int num);
	void update3D_FACES(rs2::video_frame& color, rs2::depth_frame& depth, int num);

};