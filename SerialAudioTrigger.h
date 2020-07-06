//
// Created by Zahorack (Oliver Hollý)
//

#pragma once

#include "AudioTrigger.h"

class SerialAudioTrigger {

	bool m_events[Events::Size];
	long m_lastEventTime[Events::Size];
	double m_peaks[2];

public:
	SerialAudioTrigger();
	~SerialAudioTrigger();

	void update();

	void clear();
	void clear(Events event);

	bool check(Events event);
};
