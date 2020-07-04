//
// Created by Zahorack (Oliver Hollý)
//

#pragma once
#include <windows.h>
#include <iostream>
#include <opencv2/opencv.hpp> 
#include <librealsense2/rs.hpp>

static const std::string PATH = "C:\\data\\";

class Logger {

	std::string m_date;
	std::string m_time;

public:
	Logger();
	~Logger();


	void updateRGB(cv::Mat &frame);
	void update3D(rs2::video_frame& color, rs2::depth_frame& depth);

};