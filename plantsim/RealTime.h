#pragma once
#include <windows.h>
#include <ctime>
#include <stdio.h>

/* From...
https://learn.microsoft.com/en-us/windows/win32/sysinfo/acquiring-high-resolution-time-stamps
"the performance counter frequency is fixed to 10 MHz."
*/
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

