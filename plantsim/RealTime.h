#pragma once
#include <windows.h>
#include <ctime>
#include <stdio.h>

#define WIN_PERFORMANCE_COUNTS  10000000  // tick base 10MHz

class RealTime
{
public:
	RealTime(long hertz);
	~RealTime(void);

	void Start();
	void Sleep(void);
	void Stop(int cycleCount);

private:

	long ticksPerPeriod;
	LARGE_INTEGER hr0;
	clock_t start, stop, last, current;

};

