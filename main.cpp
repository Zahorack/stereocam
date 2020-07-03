//
// Created by Zahorack (Oliver Hollý)
//

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

#include "stereoscan.h"
#include "AudioTrigger.h"
#include <windows.h>
#include "cv-helpers.h"

namespace Cameras {
    enum Enum {
        Stereo = 0,
        RGB,

        Size
    };
}

Cameras::Enum camera_id = Cameras::Enum::RGB;


int main(int argc, char* argv[]) try
{
    /*rs2::pipeline pipe;
    Stereoscan stereoscan(pipe);
    AudioTrigger audioTrigger;*/

    /*for (auto i = 0; i < 20; ++i) {
        pipe.wait_for_frames();
    }*/


    //check usb cameras
   /*cv::VideoCapture camera;
    int device_counts = 0;
    cv::Mat fr;
    while (true) {
        camera.set(cv::CAP_PROP_FORMAT, CV_16U);
        if (!camera.open(device_counts++)) {
            break;
        }
        camera.read(fr);
        std::cout<<"Device: "<<device_counts << "  Size " << fr.size << std::endl;
    }
    camera.release();
    std::cout << "devices count : " << device_counts - 1 << std::endl;*/
    

    cv::VideoCapture cap(0);
    //cap.set(cv::CAP_PROP_FORMAT, CV_16SC2);
    //cap.set(cv::CAP_PROP_MODE, cv::COLOR_YUV2RGB_YUYV);
    //cap.set(cv::CAP_PROP_FOURCC, FOURCC_LIST);

 
   /* if (cap.isOpened() == false)
    {
        std::cout << "Cannot open the video camera" << std::endl;
        std::cin.get(); //wait for any key press
    }*/


    //cv::Mat rgb_frame;
    //printf("%x\n", d[0]);
    uint16_t buf[192*256*2*3];


    cv::Mat rgb_frame;

    const int resolution = 256 * 192*3;

    cap.set(cv::CAP_PROP_CONVERT_RGB, false);

    while (cv::waitKey(1) < 0) 
    {

        using namespace std;

       bool bSuccess = cap.read(rgb_frame);
        //cap.retrieve(rgb_frame);

        cout << rgb_frame.size() << endl;





        uint16_t *d = reinterpret_cast<uint16_t *>(rgb_frame.data);
        uint8_t* dd = reinterpret_cast<uint8_t*>(rgb_frame.data);
        cout << rgb_frame.type() << endl;
        //cout << rgb.type() << endl;
        //cout << d[0] << endl;

 

        auto end = _byteswap_ushort(d[0]);

        //printf("%x\n", d[0]);
        //printf("%x  ", rgb_frame.at<uint8_t>(0, 0));
        //printf("%x\n", rgb_frame.at<uint8_t>(0, 256 * 192 * 3));

       // printf("%x  ", dd[0 + 197632/2]);
       // printf("%x\n", dd[1+ 197632 / 2]);

        uint16_t t = dd[1 + 197632 / 2] << 8 | dd[0 + 197632 / 2];
       // printf("%f\n", t / 16.0 - 273.15);


        cv::Mat  ram(256, 192, CV_16U);



        uint16_t pole[256 * 193];

        uint8_t pole2[256 * 193];

        int index2 = 0, index = 0;
        for (int i = 0; i < 256; i++) {
            for (int j = 0; j < 192; j++) {


                uint16_t t = dd[index2 + 1 + 197632 / 2] << 8 | dd[index2 + 197632 / 2]>>8;
            
                pole[index] = t;
                pole2[index] = dd[index2 + 1 + 197632 / 2];
                //printf("%d\n", pole[index]);
                index2 += 2;
                index++;

                if (index == 1) {
                    printf("%d\n", pole[index]);
                }
            }
        }

      
        cv::Mat a(192,256 , CV_16U, pole);
        cv::Mat a2(192, 256, CV_16U);

        cv::normalize(a, a2, 0, 16383, cv::NORM_MINMAX);


        uint8_t pole3[256 * 193];
        uint16_t* p_a = reinterpret_cast<uint16_t*>(a.data);

        int ind = 0;
        int max = 0;
        for (int i = 0; i < (256 * 192); i++) {

            if (p_a[ind] > max)
                max = p_a[ind];

            pole3[ind] = p_a[ind]>>8;
            ind++;
        }
         printf("%f\n", max / 16.0 - 273.15);
        cv::Mat r(192, 256, CV_8U, pole3);

        cv::Mat rgb;
        cv::cvtColor(r, rgb, cv::COLOR_GRAY2BGR);
        cout << rgb.size() << endl;


        cv::Mat nn;
        cv::applyColorMap(rgb, nn, cv::COLORMAP_JET);

 
        cv::imshow("raw", nn);
        //cv::imshow("raw2", rgb_frame);
        /*printf("%x  ", dd[resolution+ 0+3]);
        printf("%x  ", dd[resolution +1+3]);
        printf("%x\n\n", dd[resolution +2+3]);*/


        uint16_t temp = 0;

        int rgb_index = resolution;

        /*if (dd[rgb_index + 1] == 0xff) {
            temp = dd[rgb_index + 2];
        }
        else {
            temp = dd[rgb_index + 1]<<8;
        }*/

        //printf("%d\n", temp);
    
  
        /*for (int i = resolution-3; i < resolution +50; i++) {
           
            if (i == resolution) {

            }
            dd[i] = 0;
        }*/
        //cout << dd[0] << endl;
        //cout << dd[1] << endl;

       //cv::imshow("raw2", rgb_frame);

       //cv::imwrite("raw.jpg", rgb_frame);


        /*
        audioTrigger.update();

        if (audioTrigger.check(Events::Pass)) {
            std::cout << "Passing\n";

            audioTrigger.clear(Events::Pass);
        }

        if (audioTrigger.check(Events::Warning)) {
            std::cout << "Warning\n";

            if(camera_id == Cameras::Enum::Stereo)
                stereoscan.update();

            if (camera_id == Cameras::Enum::RGB) {
                FaceDetection faceDetection;

                do {
                    bool bSuccess = cap.read(rgb_frame);

                    std::cout << "RGB capture\n";
                    faceDetection.update(rgb_frame);

                } while (!faceDetection.available());

                auto faces = faceDetection.crops(1.4, 1.3);

                for (int i = 0; i < faces.size(); i++) {

                    std::string filename = "";
                    static int filenumber = 0;
                    std::stringstream ssfn;
                    ssfn << "faces/RGB/" << filenumber << ".jpg";
                    filename = ssfn.str();
                    cv::imwrite(filename, rgb_frame(faces[i]));
                    filenumber++;
                }
            }

            audioTrigger.clear(Events::Warning);
        }
        */



        //stereoscan.update();


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

