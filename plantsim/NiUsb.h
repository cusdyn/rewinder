#pragma once
#include <NIDAQmx.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <string.h>

#define MAX_TASKS  4
typedef enum {
	input = 0,
	output
}Direction;



class NiUsb
{
public:
	NiUsb();
	~NiUsb();

	int InitializeTask();
	int StartTask(int taskNum);
	int StopTask(int taskNum);
	int AddVoltageChannel(int taskNum, Direction dir, int pin, float vmin, float vmax);
	int ReadAin(int taskNum, double* data);
	int WriteMultiAout(int taskNum, double* data, int num);

private:
	std::vector<std::string> devices;
	std::vector<TaskHandle> task;
	char        errBuff[2048] = { '\0' };
	unsigned short numVin[MAX_TASKS];
	unsigned short numVout[MAX_TASKS];
};

