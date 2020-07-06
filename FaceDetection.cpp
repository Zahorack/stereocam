//
// Created by Zahorack (Oliver Hollý)
//

#include "FaceDetection.h"


static cv::String face_cascade_name = "haarcascade_frontalface_default.xml";
static cv::CascadeClassifier s_face_cascade(face_cascade_name);


FaceDetection::FaceDetection()
{}


FaceDetection::FaceDetection(cv::Mat& frame) :
	m_frame(frame)
{
	cv::Mat gray;
	cvtColor(m_frame, gray, cv::COLOR_BGR2GRAY);
	equalizeHist(gray, gray);

	s_face_cascade.detectMultiScale(gray, m_faces, 1.2, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

	std::cout << "found faces:: "<<m_faces.size() << std::endl;
}


void FaceDetection::update(cv::Mat& frame)
{
	m_frame = frame;
	cv::Mat gray;
	cvtColor(m_frame, gray, cv::COLOR_BGR2GRAY);
	equalizeHist(gray, gray);

	s_face_cascade.detectMultiScale(gray, m_faces, 1.2, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

	std::cout << "found faces:: " << m_faces.size() << std::endl;
}

FaceDetection::~FaceDetection()
{}


std::vector<cv::Rect> FaceDetection::crops(double scale, double height_scale) {

	std::vector<cv::Rect> roi;

	if (m_faces.size())
		for (auto ic = 0; ic < m_faces.size(); ic++) {
			//std::cout << "roi: " << ic << std::endl;
			cv::Point center_point(m_faces[ic].x + m_faces[ic].width / 2, m_faces[ic].y + m_faces[ic].height / 2);

			cv::Rect roi_a;
			roi_a.width = cvRound(m_faces[ic].width * scale);
			roi_a.height = cvRound(m_faces[ic].height * scale * height_scale);
			roi_a.x = cvRound(center_point.x - roi_a.width / 2);
			roi_a.y = cvRound(center_point.y - roi_a.height / 2);

			//std::cout << "roi_a: " << roi_a << std::endl;
			roi.push_back(roi_a);

			//std::cout << "roi: " << roi[ic] << std::endl;

			if ((roi[ic].x + roi[ic].width) >= m_frame.cols)
				roi[ic].width = m_frame.cols - roi[ic].x;

			if ((roi[ic].y + roi[ic].height) >= m_frame.rows)
				roi[ic].height = m_frame.rows - roi[ic].y;


			roi[ic].x = clamp<int>(roi[ic].x, 0, m_frame.cols - 1);
			roi[ic].y = clamp<int>(roi[ic].y, 0, m_frame.rows - 1);
			roi[ic].width = clamp<int>(roi[ic].width, 0, m_frame.cols - 1);
			roi[ic].height = clamp<int>(roi[ic].height, 0, m_frame.rows - 1);
		}

	return roi;
}

std::vector<cv::Point> FaceDetection::centers() {

	std::vector<cv::Point> centers;

	if (m_faces.size())
		for (auto ic = 0; ic < m_faces.size(); ic++) {
			cv::Point center_point(m_faces[ic].x + m_faces[ic].width / 2, m_faces[ic].y + m_faces[ic].height / 2);
		
			centers.push_back(center_point);
		}

	return centers;
}

bool FaceDetection::available()
{
	if (m_faces.size())
		return true;

	return false;
}