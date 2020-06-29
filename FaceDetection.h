//
// Created by Zahorack (Oliver Hollý)
//

#pragma once

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


#include <vector>
#include <iostream>

#include "cv-helpers.h"



class FaceDetection {

	std::vector<cv::Rect> m_faces;
	cv::Mat m_frame;

public:
	FaceDetection();
	FaceDetection(cv::Mat& frame);
	~FaceDetection();

	std::vector<cv::Rect> crops(double scale, double height_scale);
	std::vector<cv::Point> centers();

	void update(cv::Mat& frame);

	bool available();
};

