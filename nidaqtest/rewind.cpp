// nidaqtest.cpp : This file contains the 'main' function. Program execution begins and ends there.
#define _USE_MATH_DEFINES
#include <windows.h>
#include <iostream>
#include <NIDAQmx.h>
#include <time.h>
#include <conio.h>
#include <stdio.h>
#include <math.h>
#include "IIRdf2filt.h"
#include "rewinder.h"


#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

#define SAMPLE_RATE         400
//#define LOG_RATE            10
#define PERFORMANCE_COUNTS  10000000 

void sleepfor(LARGE_INTEGER from, long performance_clicks);
static int save_data(const char* fname, float* data, int num);

#define DURATION     60  // seconds
#define RECORDLENGTH (SAMPLE_RATE*DURATION)

#define IDLER_DIA     0.241   // meters=9.5 inches
#define RACK_LEFT_VOLTAGE  4.0
#define RACK_REST_VOLTAGE  5.0
#define RACK_RIGHT_VOLTAGE 5.5

#define CMD_MIN            -5.0
#define CMD_MIDRANGE       5.0
#define CMD_MAX            5.0





#define GAINSCALE 1

#define EDGEERR2LVDTVREF (0.0256)

// gain scaling
#define MAXLAG_OMEGA_FACTOR  (0.5236)   //  30/57.3. divide by time delay
#define WEB_DELAY_LENGTH     (1.5)      //  meter

const double a[] = { 1.000000000000000,
					-3.589733887112177,
					4.851275882519421,
					-2.924052656162462,
					0.663010484385892 };

const double b[] = { 0.000031238976917082,
					 0.000124955907668330,
					 0.000187433861502494,
					 0.000124955907668330,
					 0.000031238976917082 };

// Fc==10 Hz
const double as[] = { 1.000000000000000,
					 -3.794791103079405,
					 5.405166861726164,
					 -3.424747347274239,
					 0.814405997727277 };

const double bs[] = {0.0000021505687372881,
					 0.0000086022749491522,
					 0.0000129034124237283,
					 0.0000086022749491522,
					 0.0000021505687372881 };

// Fc=5 Hz
const double ass[] = {1.000000000000000,
					  -3.897385982468392,
					  5.697390003949064,
					  -3.702467148464776,
					  0.902465387346561};

const double bss[] = {0.000000141272653624686,
					  0.000000565090614498745,
					  0.000000847635921748118,
					  0.000000565090614498745,
	                  0.000000141272653624686 };


int main()
{
	int32       error = 0;
	TaskHandle  taskHandle = 0;
	TaskHandle  taskHandleAo = 0;
	TaskHandle  taskHandleCntr = 0;

	int32       read;
	char        errBuff[2048] = { '\0' };

	int       dataoutcnt = 0;
	float     edgeout[RECORDLENGTH];
	float     rackout[RECORDLENGTH];
	float     rackfout[RECORDLENGTH];
	float     edgeerreout[RECORDLENGTH];
	float     speederreout[RECORDLENGTH];
	float     tdout[RECORDLENGTH];
	float     drout[RECORDLENGTH];
	float     cmdout[RECORDLENGTH];

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
	float    ledge;
	float64  rackf;

	float    drack = 0;   // rack derivative

	float    ui = 0;
	float    u = 0;
	float64  cmd=RACK_REST_VOLTAGE;
	float64  fcmd = cmd;
	float64  pcmd = 0;
	float    perr=0;
	float    serr = 0;
	float    pref=6;
	float	 err=0;

	float rref = 5.0;

	// gain scaling for web speed
	float Kpmax = 7;
	float wcmax = 2;
	float wcdes = wcmax;  // initialize desired wc and don't set it until speed is non-zero
	float tdelay = 0;
	float bz = 0.0842;   // zero location

	float Kpe = .0254;  // Klg fro mmodel position loop gain maps edge guide V/m to lvdt V/m, unity gain
	float Kv = 1.0;     // Valve gain from sysId

	float Kp = Kpmax;  // 1.35;	//float Ti  = .5; // 0.1875;	
	float Ki = bz*Kp;  // Kp / Ti;
	float Kmult = (float)0.2;

	bool servo = false;
	bool sysid = false;
	float sysIdAmp  = 2;
	float sysIdFreq = 2.125;
	float drift_test = 0;

	int N = sizeof(a) / sizeof(double);
	
	uInt32 counter[2];
	float  period=0.0;
	float  webspeed = 0;
	uInt32 speed_ticks = 0;
	
	float sweepamp  = 0;
	float sweepfreq = 0;

	


	IIRdf2filt* rackfilt;
	rackfilt = new IIRdf2filt(N, a, b);

	IIRdf2filt* edgefilt;
	edgefilt = new IIRdf2filt(N, a, b);

	IIRdf2filt* cmdfilt;
	cmdfilt = new IIRdf2filt(N, a, b);

	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
	DAQmxErrChk(DAQmxCreateTask("", &taskHandleAo));
	DAQmxErrChk(DAQmxCreateTask("", &taskHandleCntr));

	DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle, "Dev1/ai0", "", DAQmx_Val_Cfg_Default, 0.0, 10.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle, "Dev1/ai1", "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCreateCICountEdgesChan(taskHandleCntr, "Dev1/ctr0", "", DAQmx_Val_Rising, 0, DAQmx_Val_CountUp));
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
	DAQmxErrChk(DAQmxStartTask(taskHandleCntr));

	//initialize...
#if 1
	DAQmxReadAnalogF64(taskHandle, 1, 0, DAQmx_Val_GroupByChannel, sample, 2, &read, NULL);
	DAQmxReadCounterScalarU32(taskHandleCntr, 0, &counter[0], NULL);  //initialize counter reference

	edge = sample[1];
	ledge = edge;
	rack[0] = sample[0];
	rack[1] = rack[0];

#endif

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
				jog = Jog::right;
			else if (c == 'i')
				jog = Jog::left;
			else if (c == 'o')
				jog = Jog::halt;  // no manual jog
			else if (c == 'c')
			{
				sysid = false;
				servo = true;
			}
			else if (c == 'x')
			{
				servo = false;
				sysid = false;
			}
			else if (c == 't')
			{
				Kpmax += 0.1;
				printf("Kpmax=%3.3f\n", Kp);
			}
			else if (c == 'g')
			{
				Kpmax -= 0.1;
				printf("Kpmax=%3.3f\n", Kp);
			}
			else if (c == 'y')
			{
				Ki += .1;
				printf("Ki=%3.1f Kp=%3.3f\n", Ki, Kp);
			}
			else if (c == 'h')
			{
				Ki -= .1;
				printf("Ki=%3.1f Kp=%3.3f\n", Ki,Kp);
			}
			else if (c == 's') 
			{
				// system ID sin sweep
				servo = false;
				sysid = true;

				// reset the file output counter
				dataoutcnt = 0;

				// put the frequency in the first sample
				rackout[dataoutcnt++] = sysIdFreq;
				// put the amplitude in the second sample
				rackout[dataoutcnt++] = sysIdAmp;

				drift_test = 0;
			}
	
		}

		// Read Idler roller sensor counter...
		DAQmxReadCounterScalarU32(taskHandleCntr, 0, &counter[1], NULL);

		if (counter[1] == counter[0])
		{
			speed_ticks++;
		}
		else if (counter[1] == (counter[0]+1))  // one tick
		{
			period = float(speed_ticks) / SAMPLE_RATE;
			webspeed = M_PI * IDLER_DIA/period;
			
			tdelay = WEB_DELAY_LENGTH / webspeed; // time delay from roll to edge guide
			wcdes = MAXLAG_OMEGA_FACTOR / tdelay;


			//SCALE GAINS Kp = Kpmax * 10 ^ (log10(wc / wmax))
			Kp = min(Kpmax, Kpmax * pow(10, log10(wcdes / wcmax)));
			

			Ki = bz;

			speed_ticks = 0;
		}
		else if (counter[1] > (counter[0] + 1))  // extra ticks?
		{
			printf("Extra Ticks!\n");
		}

		// if current count less than last it wrapped and will reinitialize here,
		// skipping one sample cycle.

		// propagate the counter
		counter[0] = counter[1];
		

		/*********************************************/
		// DAQmx Read Code
		/*********************************************/
		if (DAQmxReadAnalogF64(taskHandle, 1, 0, DAQmx_Val_GroupByChannel, sample, 2, &read, NULL) != 0)
		{
			continue;
		}

		rack[1] = sample[0];

		edge    = sample[1];
#if 0
		if (edge - ledge > 5)
		{
			edge = ledge;
		}
		else {
			ledge = edge;
		}
#endif
		//edge = edgefilt->FilterSample(edge);

		// filter rack feedback for clean(er) derivative?...
		
		rackf = rackfilt->FilterSample(rack[1]);
		//rackf = rack[1];

		//rackf = rack[1];
		drack = (rackf - rack[0]) * SAMPLE_RATE;
		rack[0] = rackf;
		
		// we now have edge guid and rack samples in and filtered

		if(servo == true)
		{
			// Edge ref error supplies rref: rack reference... for now take it as rack position
			//rref = pref - rack[1];  // position error
#if GAINSCALE
			

			perr = -edge*Kpe;   // map edge guide voltage to LVDT equivalent

			// P term
			u = Kp*perr;

			// I term
			ui = ui + (Ki*perr)/SAMPLE_RATE;

			if (ui < CMD_MIN)
				ui = CMD_MIN;
			else if (ui > CMD_MAX)
				ui = CMD_MAX;

			
			// speed error
			serr = (u + ui) - drack;

			// unity gain on inner loop...consider tach feedback gain?
			cmd = serr;

			cmd = max(cmd, CMD_MIN);
			cmd = min(cmd, CMD_MAX);

			cmd = cmd + CMD_MIDRANGE;
#elif NESTEDOLD
			// run outer loop 10 times slower
			//if (ncount % 10 == 0)
			//{
				// position error
			perr = -edge * Kpe;
			//}

			// position error ref input to inner speed loop, filtered drack/dt as feedback.
			serr = perr - drack;
			u = serr * Kp;
			ui = ui + Ki * serr / SAMPLE_RATE;

			if (ui < CMD_MIN)
				ui = CMD_MIN;
			else if (ui > CMD_MAX)
				ui = CMD_MAX;

			printf("Kp=%3.3f Ki=%3.3f perr=%3.3f u=%3.3f ui=%3.3f  cmd=%3.3f\n", Kp, Ki, perr, u, ui, cmd);

			cmd = u + ui;

			cmd = max(cmd, CMD_MIN);
			cmd = min(cmd, CMD_MAX);

			cmd = cmd + CMD_MIDRANGE;

			//fcmd = cmdfilt->FilterSample(cmd);
			//cmd = fcmd;
#else  // position loop
			if (ncount % 10 == 0)
			{
				/// Edge*eg_meters-per-volt*lvdt_volts_per_meter is new desired reference for inner loop
				perr = edgef * EDGEERR2LVDTVREF * Kmult;
			}

			if (ncount % 10 == 0)
			{
				Ki = Kp / Ti;
				// desired rack position is current PLUS edge delta...



				pref = rackf - perr;

				err = pref - rackf;
				//err = rref - rackf;

				u = Kp * err;
				ui = ui + Ki * err / SAMPLE_RATE;
				if (ui < CMD_MIN)
					ui = CMD_MIN;
				else if (ui > CMD_MAX)
					ui = CMD_MAX;

				cmd = u + ui;


				cmd = max(cmd, CMD_MIN);
				cmd = min(cmd, CMD_MAX);

				cmd = cmd + CMD_MIDRANGE;
			}
#endif
			if (ncount % 200 == 0)
			{
				printf("Kp=%3.3f Ki=%3.3f err=%3.3f u=%3.3f ui=%3.3f  cmd=%3.3f\n", Kp, Ki, err, u, ui, cmd);
			}
		}
		else if (sysid == true) {
			cmd = sysIdAmp * sin(2 * M_PI * sysIdFreq * ncount / SAMPLE_RATE); //  +drift_test;
			drift_test += 0.001;
			cmd = max(cmd, CMD_MIN);
			cmd = min(cmd, CMD_MAX);
			cmd = cmd + CMD_MIDRANGE;
		}

		// Jog override
		switch (jog) {
		case Jog::halt:
			cmd = RACK_REST_VOLTAGE;
			break;
		case Jog::left:
			cmd = RACK_LEFT_VOLTAGE;
			break;
		case Jog::right:
			cmd = RACK_RIGHT_VOLTAGE;
			break;
		default:
			break;
		}


		// Write command out
		DAQmxWriteAnalogScalarF64(taskHandleAo, false, 0, cmd, NULL);


		// update log records
		if ( (sysid == true) || ncount%((int)(SAMPLE_RATE)) == 0)
		{
			if (ncount % ((int)(SAMPLE_RATE)) == 0) 
			{
				printf("samples: %d sysIdState=%d count=%d period=%f webspeed=%f Td=%f wc=%f   Kp=%f  Ki=%f\n",
						dataoutcnt, sysid, counter[1], period, webspeed, tdelay, wcdes, Kp, Ki);
			}
//			printf("Counter: %d period:%4.2f\n", counter[1], period );
			if (dataoutcnt < RECORDLENGTH)
			{
				edgeout[dataoutcnt]      = edge;
				rackout[dataoutcnt]      = rack[1];
				rackfout[dataoutcnt]     = rackf;
				edgeerreout[dataoutcnt]  = perr;
				speederreout[dataoutcnt] = serr;
				drout[dataoutcnt]        = drack;
				cmdout[dataoutcnt] = cmd;
				dataoutcnt++;
			}
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
	DAQmxStopTask(taskHandleCntr);

	stop = clock();

	//---*****WARNING!*****---DO NOT CHANGE THE CODE BETWEEN WARNINGS-----------------
	// calculate how fast the loop ran
	elapsed_time = (double)(stop - start) / CLOCKS_PER_SEC;
	ffr = ((double)ncount) / elapsed_time;
	printf("\n%d cycles in %f sec (%f Hz, %f sec))\n",
		ncount, elapsed_time, ffr, 1.0 / ffr);
	//--***** END WARNING! *****-------------------------------------------------------
	
	if (sysid == true) {
		char fileName[30];
		printf("\nType in a file name:\n");
		scanf_s("%s", fileName, sizeof(fileName));
		save_data(fileName, rackout, dataoutcnt);
	}
	else
	{
		save_data("edge", edgeout, dataoutcnt);
		save_data("rack", rackout, dataoutcnt);
		save_data("frack", rackfout, dataoutcnt);
		save_data("drack", drout, dataoutcnt);
		save_data("edgeerr", edgeerreout, dataoutcnt);
		save_data("speederr", speederreout, dataoutcnt);
		save_data("tdout", tdout, dataoutcnt);
		save_data("cmdout", cmdout, dataoutcnt);
	}
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
	{
		//printf(".");
	}

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
