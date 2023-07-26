#include "RealTime.h"

RealTime::RealTime(long hertz)
{
	ticksPerPeriod = WIN_PERFORMANCE_COUNTS / hertz;
	// set this program to run at a high priority under NT to ensure real-time behavior
	SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);
}

RealTime::~RealTime(void) {

}

void RealTime::Sleep(void)
{
	LARGE_INTEGER pc0;
	long pc_diff;

	QueryPerformanceCounter(&pc0);
	pc_diff = (long)pc0.QuadPart - (long)hr0.QuadPart;

	if (pc_diff > ticksPerPeriod)
	{
		//printf(".");
	}

	while (pc_diff < ticksPerPeriod) {
		QueryPerformanceCounter(&pc0);
		pc_diff = (long)pc0.QuadPart - (long)hr0.QuadPart;
	}

	QueryPerformanceCounter(&hr0); // counts from right NOW
	return;
}


void RealTime::Start(void)
{
	// get system performance counts NOW
	QueryPerformanceCounter(&hr0);
	start = clock();  // start the loop clock start time
}

void RealTime::Stop(int cycleCount)
{
	double elapsed_time = (double)(clock() - start) / CLOCKS_PER_SEC;
	double ffr = ((double)cycleCount) / elapsed_time;
	printf("\n%d cycles in %f sec (%f Hz, %f sec))\n",
		cycleCount, elapsed_time, ffr, 1.0 / ffr);
}