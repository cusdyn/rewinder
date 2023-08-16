#include "RewinderPlantSim.h"


RewinderPlantSim::RewinderPlantSim(int loglen)
{
	maxlogLen = loglen;
	memset(u, 0, 3 * sizeof(double));
	memset(y, 0, 3 * sizeof(double));

	KlvdtToEg = (float)(KEG / KL);

	pEdge = 0;  // relative to mid range
	vEdge = LVDT_MID_VRANGE;
}

RewinderPlantSim::~RewinderPlantSim()
{

}

void  RewinderPlantSim::SetPedge(float peg)
{
	// Set an Edge guide cener Position relative to nominal
	pEdge = peg;

	// this edge guide center positon in LVDT volt equivalents
	vEdge = (float)(KL * peg + VALVE_AMP_MIDRANGE);
}

float RewinderPlantSim::CmdIn(float cmd)
{
	float yout = 0;
	int      n = (int)cmdIn.size();
	
	// Map Aplifier command voltage to directional voltage: signed
#if 1
	cmd -= VALVE_AMP_MIDRANGE;

	u[2] = cmd;
	//Plant model : difference equation from u to y.
	if (n > 1)
	{
		y[2] = b1 * u[1] + b0 * u[0] - a1 * y[1] - a0 * y[0];
	}
	else if (n == 1)
	{
		y[2] = b1 * u[1] - a1 * y[1];
	}
	else // k = 1
	{
		y[2] = 0;
	}

	yout = y[2];
	y[0] = y[1];
	y[1] = y[2];

	u[0] = u[1];
	u[1] = u[2];
#else
	if (n < YYTESTSZ)
	{
		uu[n] = cmd;
		//Plant model : difference equation from u to y.
		if (n > 1)
		{
			yy[n] = b1 * uu[n - 1] + b0 * uu[n - 2] - a1 * yy[n - 1] - a0 * yy[n - 2];
		}
		else if (n == 1)
		{
			yy[n] = b1 * uu[n - 1] - a1 * yy[n - 1];
		}
		else // k = 1
		{
			yy[n] = 0;
		}

		yout = yy[n];
	}
#endif

	if (cmdIn.size() < maxlogLen)
	{
		cmdIn.push_back(cmd);
	}
	if (lvdtOut.size() < maxlogLen)
	{
		lvdtOut.push_back(y[2]);
	}

	return yout;
}

float RewinderPlantSim::EdgeGuideModel(float vlvdt)
{
	float veg;                // desired output.
	float satlim = (float)(KL * EGZ);  // LVDT equivalent range to saturation of edge guide.
#if 1
	float dv = vlvdt-vEdge; // lvdt pos to edge-guide-equivalent center pos on lvdt
	
	if (dv > satlim) 
	{
		veg = EDGE_GUIDE_VSAT;
	}
	else if (dv < -satlim)
	{
		veg = -EDGE_GUIDE_VSAT;
	}
	else
	{
		veg = KlvdtToEg * dv;
	}
#else
	veg = KlvdtToEg * (vlvdt - LVDT_MID_VRANGE);
#endif
	// you will model a delay here, but for now apply the lvdt equivalent immdiately
	
	// clamp it
	veg = (float)max(-EDGE_GUIDE_VSAT, veg);
	veg = (float)min(EDGE_GUIDE_VSAT, veg);

	edgeOut.push_back(veg);

	return veg;
}

void RewinderPlantSim::LogFilesOut(void)
{
	ToFile("cmd",  &cmdIn[0],   (int)cmdIn.size());
	ToFile("lvdt", &lvdtOut[0], (int)cmdIn.size());
	ToFile("edge", &edgeOut[0], (int)cmdIn.size());
}

int RewinderPlantSim::ToFile(const char* fname, float* data, int num)
{
	FILE* datafile = NULL;
	int i;

	char fileName[30] = "data.txt";
	printf("\nType in a file name:\n");
	if (fname == NULL) {
		scanf_s("%s", fileName, (unsigned int)sizeof(fileName));
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
