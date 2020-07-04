
#include <windows.h>
#include "cv-helpers.h"


#include <dshow.h>

#pragma comment(lib, "strmiids.lib")

int UVC_camera_id = -1;

HRESULT EnumerateDevices(REFGUID category, IEnumMoniker** ppEnum)
{
    // Create the System Device Enumerator.
    ICreateDevEnum* pDevEnum;
    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL,
        CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&pDevEnum));

    if (SUCCEEDED(hr))
    {
        // Create an enumerator for the category.
        hr = pDevEnum->CreateClassEnumerator(category, ppEnum, 0);
        if (hr == S_FALSE)
        {
            hr = -1;  // The category is empty. Treat as an error.
        }
        pDevEnum->Release();
    }
    return hr;
}


void DisplayDeviceInformation(IEnumMoniker* pEnum)
{
    IMoniker* pMoniker = NULL;
    int iterator = 0;
    while (pEnum->Next(1, &pMoniker, NULL) == S_OK)
    {
        IPropertyBag* pPropBag;
        HRESULT hr = pMoniker->BindToStorage(0, 0, IID_PPV_ARGS(&pPropBag));
        if (FAILED(hr))
        {
            pMoniker->Release();
            continue;
        }

        VARIANT var;
        VariantInit(&var);

        // Get description or friendly name.
        hr = pPropBag->Read(L"Description", &var, 0);
        if (FAILED(hr))
        {
            hr = pPropBag->Read(L"FriendlyName", &var, 0);
        }
        if (SUCCEEDED(hr))
        {
            printf("Name: %S\n", var.bstrVal);



            char device_name[50];
            sprintf_s(device_name, "%S", var.bstrVal);

            const std::string UVC_CAMERA = "UVC Camera";

            if (device_name == UVC_CAMERA) {

                UVC_camera_id = iterator;
                std::cout << "LOLOLOLL\n";
            }

            VariantClear(&var);
        }

        hr = pPropBag->Write(L"FriendlyName", &var);

        // WaveInID applies only to audio capture devices.
        hr = pPropBag->Read(L"WaveInID", &var, 0);
        if (SUCCEEDED(hr))
        {
            printf("WaveIn ID: %d\n", var.lVal);
            VariantClear(&var);
        }

        hr = pPropBag->Read(L"DevicePath", &var, 0);
        if (SUCCEEDED(hr))
        {
            // The device path is not intended for display.
            printf("Device path: %S\n", var.bstrVal);
            VariantClear(&var);
        }

        pPropBag->Release();
        pMoniker->Release();

        iterator++;
    }
}


int test(int argc, char* argv[])
{

    HRESULT hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);
    if (SUCCEEDED(hr))
    {
        IEnumMoniker* pEnum;

        hr = EnumerateDevices(CLSID_VideoInputDeviceCategory, &pEnum);
        if (SUCCEEDED(hr))
        {
            DisplayDeviceInformation(pEnum);
            pEnum->Release();
        }

        CoUninitialize();


    }

    return 0;

}