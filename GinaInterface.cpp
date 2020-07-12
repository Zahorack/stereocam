
#include "GinaInterface.h"
#include <opencv2/opencv.hpp> 

#include "base64.h"

GinaInterface::GinaInterface()
{
	std::string encoded_png;
	cv::Mat img; // Load an image here

	std::vector<uchar> buf;
	cv::imencode(".png", img, buf);
	auto base64_png = reinterpret_cast<const unsigned char*>(buf.data());
	encoded_png = "data:image/jpeg;base64," + base64_encode(base64_png, buf.size());


	/*std::vector<uchar> buf;
	auto base64_png = reinterpret_cast<const unsigned char*>(buf.data());
	std::string encoded_png = "data:image/jpeg;base64," + Base64::Encode(base64_png, buf.size());*/

}