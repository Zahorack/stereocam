//
// Created by Zahorack (Oliver Hollý)
//

#include "AudioTrigger.h"

#include <Audioclient.h>
#include <endpointvolume.h>
#include <AudioEngineEndpoint.h>

#include <devicetopology.h>
#include <mmdeviceapi.h>

#include <iostream>
#include <windows.h>

static HRESULT hr;
static IMMDeviceEnumerator* pEnumerator = NULL;
static IMMDevice* pDevice = NULL;
static IAudioMeterInformation* pMeterInfo = NULL;


static bool is_initialised = false;
static const double AudioPeakLimit = 0.001;
static const long MaxEventPeriodSec = 2;

#include <ctime>

static int getSeconds() {

    return static_cast<int>(double(std::clock()) / CLOCKS_PER_SEC);
}

AudioTrigger::AudioTrigger()
{
    if (!is_initialised) {
        CoInitialize(NULL);

        // Get enumerator for audio endpoint devices.
        hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), NULL,
            CLSCTX_INPROC_SERVER,
            __uuidof(IMMDeviceEnumerator),
            (void**)&pEnumerator);


        // Get peak meter for default audio-rendering device.
        hr = pEnumerator->GetDefaultAudioEndpoint(eRender, eConsole, &pDevice);
        hr = pDevice->Activate(__uuidof(IAudioMeterInformation), CLSCTX_ALL, NULL, (void**)&pMeterInfo);

        unsigned int channelCount;
        pMeterInfo->GetMeteringChannelCount(&channelCount);

        is_initialised = true;
    }
}

AudioTrigger::~AudioTrigger()
{}

void AudioTrigger::update()
{
    pMeterInfo->GetChannelsPeakValues(2, m_peaks);
    //std::cout << "  peak1: " << m_peaks[0] << "  peak2: " << m_peaks[1] << std::endl;

    float pass_peak = m_peaks[0];
    float warning_peak = m_peaks[1];


    int seconds = getSeconds();

    if (pass_peak > AudioPeakLimit) {
        if ((m_lastEventTime[Events::Pass] + MaxEventPeriodSec) < seconds) {
            m_events[Events::Pass] = true;
            m_lastEventTime[Events::Pass] = seconds;
        }
    }
    
    if (warning_peak > AudioPeakLimit) {
        if ((m_lastEventTime[Events::Warning] + MaxEventPeriodSec) < seconds) {
            m_events[Events::Warning] = true;
            m_lastEventTime[Events::Warning] = seconds;
        }
    }
}


void AudioTrigger::clear()
{
    for (int i = 0; i < Events::Size; i++)
        m_events[i] = false;
}

void AudioTrigger::clear(Events event)
{
    m_events[event] = false;
}


bool AudioTrigger::check(Events event)
{
    return m_events[event];
}

