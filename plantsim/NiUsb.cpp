#include "NiUsb.h"


NiUsb::NiUsb() 
{
	char devs[1000];

	DAQmxGetSysDevNames(devs, 1000);
	std::string str(devs, strlen(devs));

	std::string device;
	std::stringstream ss(str);

	std::cout << "Detected NI Devices..." << std::endl;
	while (getline(ss, device, ',')) {
		devices.push_back(device);
		std::cout << device.c_str() << std::endl;
	}

	// initialize channel counts for any task that will add them
	memset(numVin,0,MAX_TASKS*sizeof(unsigned short));
	memset(numVout, 0, MAX_TASKS * sizeof(unsigned short));

}

NiUsb::~NiUsb()
{
}



int NiUsb::InitializeTask()
{
	task.push_back(new TaskHandle);
	
	int at = task.size()-1;
	
	if (DAQmxCreateTask("", &task.at(at)) < 0) {
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
		std::cout << "DAQmxCreateTask Error:" << errBuff << std::endl;
		at = -1;
	}
	
	return at;
}

int NiUsb::StartTask(int taskNum) 
{
	int ret = -1;
	if (ret = DAQmxStartTask(task.at(taskNum)) < 0) {
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
		std::cout << "DAQmxStartTask Error:" << errBuff;
	}
	return ret;
}


int NiUsb::StopTask(int taskNum)
{
	int ret = -1;
	if (ret = DAQmxStopTask(task.at(taskNum)) < 0) {
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
		std::cout << "DAQmxStopTask Error:" << errBuff;
	}
	return ret;
}


int NiUsb::AddVoltageChannel(int taskNum, Direction dir, int pin, float vmin, float vmax)
{
	char pinName[12];
	int ret = -1;

	if( dir == input)
	{ 
		sprintf_s(pinName, "%s/ai%d", devices.at(0).c_str(), pin);

		if (ret = DAQmxCreateAIVoltageChan(task.at(taskNum), pinName, "", DAQmx_Val_Cfg_Default, vmin, vmax, DAQmx_Val_Volts, NULL) < 0) {
			DAQmxGetExtendedErrorInfo(errBuff, 2048);
			std::cout << "DAQmxCreateAIVoltageChan Error:" << errBuff;
		}
		else 
		{
			numVin[taskNum]++;
		}

	}
	else if (dir == output)
	{
		sprintf_s(pinName, "%s/ao%d", devices.at(0).c_str(), pin);

		if (ret = DAQmxCreateAOVoltageChan(task.at(taskNum), pinName, "", vmin, vmax, DAQmx_Val_Volts, NULL) < 0) {
			DAQmxGetExtendedErrorInfo(errBuff, 2048);
			std::cout << "DAQmxCreateAOVoltageChan Error:" << errBuff;
		}
		else
		{
			numVout[taskNum]++;
		}
	}
	

	return ret;
}

int NiUsb::ReadAin(int taskNum, double* data)
{
	int32   samplesPerChannnelRead;
	int ret = -1;
	if (ret = DAQmxReadAnalogF64(task.at(taskNum), 1, 0,
		                         DAQmx_Val_GroupByChannel,
		                         data, numVin[taskNum], &samplesPerChannnelRead, NULL) < 0)
	{
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
		std::cout << "DAQmxReadAnalogF64 Error:" << errBuff;
	}
	else {
		ret = numVin[taskNum];
	}

	return ret;
}

int NiUsb::WriteMultiAout(int taskNum, double* data, int num) 
{
	int ret = -1;
	int32   samplesPerChanWritten;

	if (num != numVout[taskNum]) 
	{
		std::cout << "WriteMultiAout Error: Channel number mismatch" << std::endl;
	}
	else if (ret = DAQmxWriteAnalogF64(task.at(taskNum), 1, false,0, 
									DAQmx_Val_GroupByChannel, data,
									&samplesPerChanWritten, NULL) < 0)
	{
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
		std::cout << "DAQmxWriteAnalogF64 Error:" << errBuff;
	}
	return ret;
}