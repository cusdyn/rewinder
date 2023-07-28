// plantsim.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#define _USE_MATH_DEFINES
#include <windows.h>
#include <iostream>
#include <NIDAQmx.h>
#include <time.h>
#include <conio.h>
#include <stdio.h>
#include <math.h>
#include "RealTime.h"
#include "NiUsb.h"
#include "RewinderPlantSim.h"

/*
% Description:
%   Rewinder controller (Automation Direct P1AM-100) outputs 0-10V
%   valve amplifier command. In the actual system the Wandfluh amplifier
%   maps 0-5V to to move the carriage west, 5-10V to move the carriage
%   east. Thus, this is a +/-5 volt flow command input.
%
%   The tranfer function from this command input to LVDT output is known
%   from the system identification result.
%
%   This simulator acts on the input side like the Wandfluh amplifier: it
%   accepts 0-10V input from the P1AM, maps it to +/-5 Volts.
%
%   Given this command, it then processes the command through the
%   plant model to produce an simulated LVDT output voltage.
%
%   In the actual system the LVDT is 4-20mA through 500 ohms to produce
%   a 2-10V signal to the controller, or 8 volts range on a 2-volt bias.
%   However, the control scheme numericaly differentiates the LVDT input
%   so the bias is not a significant detail.
%
%   In the actual system the Edge Guide sub-system is entirely decoupled 
%   from the carriage and the LVDT signal, but the controller accepts
%   it as position state feedback while the numerically differentiated
%   LVDT feedback is the velocity state.
%
%   This state relationship is true for a paper edge that traverses the
%   edge guide sensor as a result of the carriage lateral motion. The
%   edge guide also picks-up edge deviations on the roll.
%
%   Simulation the Edge Guide here:
%
%     The controller is seeking to regulate Edge Guide to zero.
%     In a tightly-coupled state feedback simulation, gain-map Vlvdt
%     to Veg (edge guide). Let the above-described LVDT output feedback
%     to the controller where it will be differentiated and processed
%     as the rate (tach) feedback. The simulated Edge-guide voltage will
%     output from the simulator as +-10V.
%
%     Initialization.
%
%     This simulator acts as the actual LVDT so initialize with a mid-range
%     bias voltage of say 5 V. Add the response voltage out of the model
%     described above to this bias. Again, the controller differentiates
%     it so it doesn't matter on this point, but it represents a mid-range
%     condition compared to the 2-10V LVDT signal in the actual system.
%
%     For the Edge Guide sim, this is about zero so do not apply the bias.
%     map the signed model Vlvdt output through the lvdt-to-edge guide
%     gain adjustment and feed out the second analog output.
%
%     The P1AM controller accepts this as EdgeGuide input.
%
%     When the P1AM loop is closed this is the voltage regulated to zero.
%
%
%     Testing the simulator:
%       command input drives rate, so a position step response here is 
%       a result of an impulse. The "test" input parameter below bipasses
%       the command input read from the PP1AM and instead issues an
%       N-sample impulse
*/

#define SAMPLE_RATE         100
#define DURATION			60  // seconds
#define RECORDLENGTH		(SAMPLE_RATE*DURATION)
#define CMD_AIN             0
#define AIN1                1
#define LVDT_OUT            0
#define EDGE_OUT            1
#define ERROR -1

#define LVDT_MID_VRANGE  (4.0)
#define EDGE_MID_VRANGE  (0.0)

#define IMPULSE_TEST 0

int main()
{
	RealTime realTime(SAMPLE_RATE);
	NiUsb    ni;
	RewinderPlantSim rwSim(RECORDLENGTH,LVDT_MID_VRANGE);

	int    aiTask = -1;
	int    aoTask = -1;
	int    cycleCount = 0;
	double cmdin;
	double vout[2];

	// inisialize the 
	if((aiTask = ni.InitializeTask()) == ERROR) return 0;
	if((aoTask = ni.InitializeTask()) == ERROR) return 0;

	if(ni.AddVoltageChannel(aiTask, input, CMD_AIN, 0.0, 10.0)) return 0;
	if(ni.AddVoltageChannel(aoTask, output, LVDT_OUT, 0.0, 10.0))  return 0;
	if(ni.AddVoltageChannel(aoTask, output, EDGE_OUT, -10, 10.0))  return 0;

	ni.StartTask(aiTask);
	ni.StartTask(aoTask);


	realTime.Start();  // commence pseudo-real-time loop.

	// output LVDT and EdgeGuide Sensor Voltages
	vout[LVDT_OUT] = LVDT_MID_VRANGE;
	vout[EDGE_OUT] = EDGE_MID_VRANGE;
	ni.WriteMultiAout(aoTask, vout, 2);


	while (1)
	{

		if (_kbhit())
		{
			char c = _getch();

			// exit
			if (c == 'q')
				break;

			switch (c) 
			{
			case '0':
				rwSim.SetVedge(0.0);
				break;
			case '1':
				rwSim.SetVedge(-10.0);
				break;
			case '2':
				rwSim.SetVedge(-5.0);
				break;
			case '3':
				rwSim.SetVedge(5.0);
				break;
			case '4':
				rwSim.SetVedge(10.0);
				break;
			default:
				break;
			}
		}
		int numRead = ni.ReadAin(aiTask, &cmdin);

#if IMPULSE_TEST
		if (cycleCount == 0) {
			vlvdt = rwSim.CmdIn(1.0);
		}
		else {
			vlvdt = rwSim.CmdIn(0.0);
		}
#else
		// get AC (signed) component of LVDT output from plant model
		vout[LVDT_OUT] = rwSim.CmdIn(cmdin);
#endif
		vout[EDGE_OUT] = rwSim.EdgeGuideModel(vout[LVDT_OUT]);

		// Add it to the mid-range
		vout[LVDT_OUT] += LVDT_MID_VRANGE;

//		vout[LVDT_OUT] = 2.0;
//		vout[EDGE_OUT] = -1.0;

		// output LVDT and EdgeGuide Sensor Voltages
		ni.WriteMultiAout(aoTask, vout, 2);

		realTime.Sleep();
		cycleCount++;
	}

	realTime.Stop(cycleCount); // stop real-time loop

	ni.StopTask(aiTask);
	ni.StopTask(aoTask);

	rwSim.LogFilesOut();

}

