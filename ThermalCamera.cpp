#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

#include "stereoscan.h"
#include "AudioTrigger.h"
#include <windows.h>
#include "cv-helpers.h"

long scaling(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void test() {

    cv::VideoCapture cap(0);
    cv::Mat raw_thermal_mat;
    cap.set(cv::CAP_PROP_CONVERT_RGB, false);

    while (cv::waitKey(1) < 0) {

        using namespace std;

        cap.read(raw_thermal_mat);

        auto frame_size = raw_thermal_mat.size().width;
        uint8_t* p_raw_thermal = reinterpret_cast<uint8_t*>(raw_thermal_mat.data);


        uint16_t raw_thermal_buf[256 * 193];
        uint8_t  little_raw_thermal_buf[256 * 193];

        uint16_t min = 12000;
        uint16_t max = 0;

        int min_i = 0, max_i = 0;

        //musel som dat horny limit (256 * 190) namiesto (256 * 193), aby som spravne nasiel min a max
        for (int i = 0, ii = 0; i < (256 * 190); i++, ii += 2) {
            raw_thermal_buf[i] = static_cast<uint16_t>(p_raw_thermal[ii + 1 + frame_size / 2 + 2] << 8 | p_raw_thermal[ii + frame_size / 2 + 2]);

            if (min > raw_thermal_buf[i]) {
                min = raw_thermal_buf[i];
                min_i = i;
            }

            if (max < raw_thermal_buf[i]) {
                max = raw_thermal_buf[i];
                max_i = i;
            }
        }

        printf("%d    [%d] %d    [%d] %d\n", frame_size, min_i, min, max_i, max);

        for (int i = 0, ii = 0; i < (256 * 190); i++, ii += 2) {
            little_raw_thermal_buf[i] = static_cast<uint8_t>(scaling(raw_thermal_buf[i], min, max + 1, 0, 255));
        }

        cv::Mat thermal_mat(192, 256, CV_8U, little_raw_thermal_buf);
        cv::applyColorMap(thermal_mat, thermal_mat, cv::COLORMAP_JET);
        cv::imshow("heatmap", thermal_mat);
    }
}