// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

#include "stereoscan.h"
#include <windows.h>

#include <Audioclient.h>
#include <endpointvolume.h>
#include <AudioEngineEndpoint.h>

#include <devicetopology.h>
#include <mmdeviceapi.h>


int main(int argc, char* argv[]) try
{


    HRESULT hr;
    IMMDeviceEnumerator* pEnumerator = NULL;
    IMMDevice* pDevice = NULL;
    IAudioMeterInformation* pMeterInfo = NULL;
    
    CoInitialize(NULL);

    // Get enumerator for audio endpoint devices.
    hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), NULL, 
            CLSCTX_INPROC_SERVER,
            __uuidof(IMMDeviceEnumerator),
            (void**)&pEnumerator);

    // Get peak meter for default audio-rendering device.
    hr = pEnumerator->GetDefaultAudioEndpoint(eRender, eConsole, &pDevice);
    hr = pDevice->Activate(__uuidof(IAudioMeterInformation), CLSCTX_ALL, NULL, (void**)&pMeterInfo);

    float peaks[2];
    uint channelCount;
    pMeterInfo->GetMeteringChannelCount(&channelCount);


    rs2::pipeline pipe;
    Stereoscan stereoscan(pipe);

    //for (auto i = 0; i < 20; ++i)
     //pipe.wait_for_frames();


    while (cv::waitKey(1) < 0) 
    {
        //pMeterInfo->GetPeakValue(peaks);
        pMeterInfo->GetChannelsPeakValues(channelCount, peaks);
        std::cout <<"channels:" <<channelCount<< "  peak1: " << peaks[0] << "  peak2: " << peaks[1] << std::endl;

        //rs2::frameset data = pipe.wait_for_frames();

        /*static int last_frame_number = 0;
        if (data.get_frame_number() == last_frame_number)
            continue;
        last_frame_number = data.get_frame_number();*/

        stereoscan.update();
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

