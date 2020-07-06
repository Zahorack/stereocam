//
// Created by Zahorack (Oliver Hollý)
//

#include "SerialAudioTrigger.h"


#include <iostream>
#include <windows.h>

static HANDLE serialHandle;

static bool is_initialised = false;
static const double AudioPeakLimit = 5;
static const long MaxEventPeriodSec = 2;

#include <ctime>

static int getSeconds() {

    return static_cast<int>(double(std::clock()) / CLOCKS_PER_SEC);
}

SerialAudioTrigger::SerialAudioTrigger()
{
    if (!is_initialised) {
        // Open serial port
    
        LPCWSTR port = L"\\\\.\\COM3";

        serialHandle = CreateFile(port, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);

        // Do some basic settings
        DCB serialParams = { 0 };
        serialParams.DCBlength = sizeof(serialParams);

        GetCommState(serialHandle, &serialParams);
        serialParams.BaudRate = 9600;
        serialParams.ByteSize = 8;
        serialParams.StopBits = ONESTOPBIT;
        serialParams.Parity = NOPARITY;
        SetCommState(serialHandle, &serialParams);

        // Set timeouts
        COMMTIMEOUTS timeout = { 0 };
        timeout.ReadIntervalTimeout = 5;
        timeout.ReadTotalTimeoutConstant = 5;
        timeout.ReadTotalTimeoutMultiplier = 5;
        timeout.WriteTotalTimeoutConstant = 50;
        timeout.WriteTotalTimeoutMultiplier = 10;

        SetCommTimeouts(serialHandle, &timeout);

        is_initialised = true;
    }
}

SerialAudioTrigger::~SerialAudioTrigger()
{
    CloseHandle(serialHandle);
}

void SerialAudioTrigger::update()
{
    DWORD nRead;
    char mark;
    ReadFile(serialHandle, &mark, 1, &nRead, NULL);

    if (nRead) {
        if (mark == 'S') {
            char dL[4], c, dR[4];
            ReadFile(serialHandle, dL, 4, &nRead, NULL);
            ReadFile(serialHandle, &c, 1, &nRead, NULL);
            ReadFile(serialHandle, dR, 4, &nRead, NULL);
            ReadFile(serialHandle, &c, 1, &nRead, NULL);


            double leftValue = atof(dL); //pass
            double rightValue = atof(dR); //warning stop

            m_peaks[Events::Pass] = atof(dL);
            m_peaks[Events::Warning] = atof(dR);

            double pass_peak = m_peaks[Events::Pass];
            double warning_peak = m_peaks[Events::Warning];


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
        else {
            std::cout << "Out of protocol" << std::endl;
        }
    }
}


void SerialAudioTrigger::clear()
{
    for (int i = 0; i < Events::Size; i++)
        m_events[i] = false;
}

void SerialAudioTrigger::clear(Events event)
{
    m_events[event] = false;
}


bool SerialAudioTrigger::check(Events event)
{
    return m_events[event];
}

