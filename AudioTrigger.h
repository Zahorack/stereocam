//
// Created by Zahorack (Oliver Hollý)
//

#pragma once

enum Events : int {
	Pass = 0,
	Warning,

	Size
};

class AudioTrigger {

	bool m_events[Events::Size];
	long m_lastEventTime[Events::Size];
	float m_peaks[2];

public:
	AudioTrigger();
	~AudioTrigger();

	void update();

	void clear();
	void clear(Events event);

	bool check(Events event);
};
