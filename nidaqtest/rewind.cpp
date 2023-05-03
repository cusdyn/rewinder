// nidaqtest.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <NIDAQmx.h>
#include <time.h>
#include <windows.h>
#include <conio.h>
#include "rewinder.h"

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

#define SAMPLE_RATE 400
#define PERFORMANCE_COUNTS  10000000 

void sleepfor(LARGE_INTEGER from, long performance_clicks);
static int save_data(const char* fname, float* data, int num);

#define DURATION    80  // seconds
#define RECORDLENGTH (SAMPLE_RATE*DURATION)

#define RACK_LEFT_VOLTAGE  4.9
#define RACK_REST_VOLTAGE  5.0
#define RACK_RIGHT_VOLTAGE  5.5

#define CMD_MIN            -5.0
#define CMD_MIDRANGE       5.0
#define CMD_MAX            5.0

#define POSREF =5  // volt of slide output

#define NESTED 0

#define EDGEERR2LVDTVREF (0.0256)

int main()
{
	int32       error = 0;
	TaskHandle  taskHandle = 0;
	TaskHandle  taskHandleAo = 0;

	int32       read;
	char        errBuff[2048] = { '\0' };

	float     dout[RECORDLENGTH];
	float     vout[RECORDLENGTH];
	float     egout[RECORDLENGTH];
	float     tdout[RECORDLENGTH];
	float     eout[RECORDLENGTH];

	float64     sample[2];

	int ncount = 0;
	long sample_period;
	LARGE_INTEGER hr0;
	clock_t start, stop, last, current;
	double elapsed_time, ffr;
	bool32 readAllAvailable = false;


	
	

	Jog jog = none;

	// rack variables
	float64  rack[2];
	float64  edge;
	float64  edgef;
	float64  rackf;

	float    dr = 0;   // rack derivative
	float    drf = 0;   // rack derivative filtered

	float    ui = 0;
	float    u = 0;
	float64  cmd=RACK_REST_VOLTAGE;
	float64  pcmd = 0;
	float    perr=0;
	float    pref=6;
	float	 err=0;

	float rref = 5.0;

		
	float Kp = 5;
	float Ti = .1;// 0.1875;
	float Ki = Kp/Ti;
	float Kmult = .5;

	bool servo = false;

	Boxcar bcf;
	BoxcarLong ef;
	Boxcar rf;


	memset(&bcf, 0, sizeof(bcf));  // zero the filter
	memset(&ef, 0, sizeof(ef));  // zero the filter
	memset(&rf, 0, sizeof(rf));  // zero the filter

	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
	DAQmxErrChk(DAQmxCreateTask("", &taskHandleAo));
	DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle, "Dev1/ai0", "", DAQmx_Val_Cfg_Default, 0.0, 10.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle, "Dev1/ai1", "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCreateAOVoltageChan(taskHandleAo, "Dev1/ao0","", 0.0, 10.0, DAQmx_Val_Volts, NULL));
	//---*****WARNING!*****---DO NOT CHANGE THE CODE BETWEEN WARNINGS-----------------
	// sample period in NT performance counts	
	sample_period = PERFORMANCE_COUNTS / SAMPLE_RATE;
	// set this program to run at a high priority under NT to ensure real-time behavior
	SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);
	//--***** END WARNING! *****-------------------------------------------------------
	ncount = 0;

	// get system performance counts NOW
	QueryPerformanceCounter(&hr0);

	/*********************************************/
	// DAQmx Start Code
	/*********************************************/
	DAQmxErrChk(DAQmxStartTask(taskHandle));
	DAQmxErrChk(DAQmxStartTask(taskHandleAo));


	// Controller initialization
	DAQmxReadAnalogF64(taskHandle, 1, 0, DAQmx_Val_GroupByChannel, sample, 2, &read, NULL);
	rack[0] = sample[0];  // intialize 'last' rack position

	start = clock();  // start the loop clock start time
	last = start;

	while (1)
	{
		jog = none;
		if (_kbhit())
		{
			char c = _getch();

			if (c == 'q')
				break;

			if (c == 'p')
				jog = right;
			else if (c == 'i')
				jog = left;
			else if (c == 'o')
				jog = halt;  // no manual jog
			else if (c == 'c')
				servo = true;
			else if (c == 'x')
				servo = false;
			else if (c == 't')
			{
				Kp += 0.5;
				printf("Kp=%3.3f\n", Kp);
			}
			else if (c == 'g')
			{
				Kp -= 0.5;
				printf("Kp=%3.3f\n", Kp);
			}
			else if (c == 'y')
			{
				Kmult += .1;
				printf("Kmult=%3.1f Kp=%3.3f\n", Kmult, Kp);
			}
			else if (c == 'h')
			{
				Kmult -= .1;
				printf("Kmult=%3.1f Kp=%3.3f\n", Kmult,Kp);
			}
	
		}


		/*********************************************/
		// DAQmx Read Code
		/*********************************************/
		DAQmxReadAnalogF64(taskHandle, 1, 0, DAQmx_Val_GroupByChannel, sample, 2, &read, NULL);
		rack[1] = sample[0];
		edge    = sample[1];
		dr = (rack[1] - rack[0])*SAMPLE_RATE;
		drf = BoxCarFilter(&bcf, dr, FLEN);
		edgef = BoxCarFilterLong(&ef, edge, LONGFLEN);
		rackf = BoxCarFilter(&rf, rack[1], FLEN);

		rack[0] = rack[1];

		if(servo == true)
		{
			// Edge ref error supplies rref: rack reference... for now take it as rack position
			//rref = pref - rack[1];  // position error
#if NESTED
			// run outer loop 10 times slower
			if (ncount % 20 == 0)
			{
				Ki = Kmult * Kp;

				perr = -edgef;
				u = perr * Kp;

				ui = ui + Ki * perr / SAMPLE_RATE;

				if (ui < CMD_MIN)
					ui = CMD_MIN;
				else if (ui > CMD_MAX)
					ui = CMD_MAX;

				printf("Kp=%3.3f Ki=%3.3f perr=%3.3f u=%3.3f ui=%3.3f  cmd=%3.3f\n", Kp, Ki, perr, u, ui,cmd);

				pcmd = u + ui;
				
			}

			//err = rref - dr; // speed error
			err = pcmd - drf;
			
			cmd = err;    // unity gain feedback on inner loop


#else  // position loop
			if (ncount % 10 == 0)  
			{
				/// Edge*eg_meters-per-volt*lvdt_volts_per_meter is new desired reference for inner loop
				perr = edgef * EDGEERR2LVDTVREF*Kmult;
			}

			if (ncount % 10 == 0)  
			{
				Ki = Kp / Ti;
				// desired rack position is current PLUS edge delta...

	

				pref = rackf - perr;

				//err = pref - rackf;
				err = rref - rackf;

				u = Kp * err;
				ui = ui + Ki * err / SAMPLE_RATE;
				if (ui < CMD_MIN)
					ui = CMD_MIN;
				else if (ui > CMD_MAX)
					ui = CMD_MAX;
#endif
				cmd = u + ui;

				
				/*
				if (cmd < 0)
					cmd -= 0.07;
				else if (cmd > 0)
					cmd += 0.35;
				*/

				cmd = max(cmd, CMD_MIN);
				cmd = min(cmd, CMD_MAX);

				cmd = cmd + CMD_MIDRANGE;
			}

			if (ncount % 100 == 0)
			{
				printf("Kp=%3.3f Ki=%3.3f err=%3.3f u=%3.3f ui=%3.3f  cmd=%3.3f\n", Kp, Ki, err, u, ui, cmd);
			}
		}
		else
		{
		//	cmd = RACK_REST_VOLTAGE;
		}

		// Jog override
		switch (jog) {
		case halt:
			cmd = RACK_REST_VOLTAGE;
			break;
		case left:
			cmd = RACK_LEFT_VOLTAGE;
			break;
		case right:
			cmd = RACK_RIGHT_VOLTAGE;
			break;
		default:
			break;
		}


		// Write command out
		DAQmxWriteAnalogScalarF64(taskHandleAo, false, 0, cmd, NULL);


		// update log records
		if (ncount < RECORDLENGTH)
		{
			dout[ncount]  = rack[1];
			vout[ncount]  = rackf;
			egout[ncount] = edgef;
			eout[ncount]  = err;
		}
		

		sleepfor(hr0, sample_period);
		QueryPerformanceCounter(&hr0); // counts from right NOW

		current = clock();
		elapsed_time = (double)(last - current) / CLOCKS_PER_SEC;
		last = current;

		if (ncount < RECORDLENGTH)
		{
			tdout[ncount] = elapsed_time;
		}

		ncount++;
	}

	DAQmxWriteAnalogScalarF64(taskHandleAo, false, 0, RACK_REST_VOLTAGE, NULL);
	DAQmxStopTask(taskHandle);
	DAQmxStopTask(taskHandleAo);

	stop = clock();

	//---*****WARNING!*****---DO NOT CHANGE THE CODE BETWEEN WARNINGS-----------------
	// calculate how fast the loop ran
	elapsed_time = (double)(stop - start) / CLOCKS_PER_SEC;
	ffr = ((double)ncount) / elapsed_time;
	printf("\n%d cycles in %f sec (%f Hz, %f sec))\n",
		ncount, elapsed_time, ffr, 1.0 / ffr);
	//--***** END WARNING! *****-------------------------------------------------------
	
	save_data("dout", dout, min(ncount, RECORDLENGTH));
	save_data("vout", vout, min(ncount, RECORDLENGTH));
	save_data("tdout", tdout, min(ncount, RECORDLENGTH));
	save_data("egout", egout, min(ncount, RECORDLENGTH));
	save_data("eout", eout, min(ncount, RECORDLENGTH));

	//	printf("Acquired %d points\n", (int)read);

	

Error:
	if (DAQmxFailed(error))
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
	if (taskHandle != 0) {
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
	}
	if (DAQmxFailed(error))
		printf("DAQmx Error: %s\n", errBuff);


	return 0;
}



/* Sample rate delay*/
void sleepfor(LARGE_INTEGER from, long performance_clicks)
{
	LARGE_INTEGER pc0;
	long pc_diff;

	QueryPerformanceCounter(&pc0);
	pc_diff = (long)pc0.QuadPart - (long)from.QuadPart;

	if (pc_diff > performance_clicks)
		printf(".");

	while (pc_diff < performance_clicks) {
		QueryPerformanceCounter(&pc0);
		pc_diff = (long)pc0.QuadPart - (long)from.QuadPart;
	}
	return;
}


static int save_data(const char * fname, float* data, int num)
{
	FILE* datafile = NULL;
	int i;

	char fileName[30] = "data.txt";
	printf("\nType in a file name:\n");
	if (fname == NULL) {
		scanf_s("%s", fileName, sizeof(fileName));
	}
	else {
		strcpy_s(fileName, fname);
	}

	strcat_s(fileName, ".txt");

	fopen_s(&datafile, fileName, "w");
	if (datafile == NULL)
	{
		printf("\nError opening file\n");
		return(-1);
	}

	for (i = 0; i < num; i++)
	{
		fprintf(datafile, "%f\n", data[i]);
	}

	fclose(datafile);

	return(0);
}
