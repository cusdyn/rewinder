#include "RewinderPlantSim.h"


RewinderPlantSim::RewinderPlantSim(int loglen, float lvdtMidRange)
{
	maxlogLen = loglen;
	memset(u, 0, 3 * sizeof(double));
	memset(y, 0, 3 * sizeof(double));

	KlvdtToEg = KEG / KL;

	lvdtVmid = lvdtMidRange;
}

RewinderPlantSim::~RewinderPlantSim()
{

}

void  RewinderPlantSim::SetVedge(float veg)
{
	vEdge = veg;
}

float RewinderPlantSim::CmdIn(float cmd)
{
	float yout = 0;
	int      n = cmdIn.size();
	
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

	//yout += lvdtVmid;

	return yout;
}

float RewinderPlantSim::EdgeGuideModel(float vlvdt)
{
	float veg;
	// given LVDT voltage, model the Edge Guide equivalent.
	veg = KlvdtToEg * vlvdt;

	// you will model a delay here, but for now apply the lvdt equivalent immdiately

	veg +=vEdge;
	
	// clamp it
	veg = max(-EDGE_GUIDE_VSAT, veg);
	veg = min(EDGE_GUIDE_VSAT, veg);

	edgeOut.push_back(veg);

	return veg;
}

void RewinderPlantSim::LogFilesOut(void)
{
	ToFile("cmd",  &cmdIn[0],   cmdIn.size());
	ToFile("lvdt", &lvdtOut[0], cmdIn.size());
	ToFile("edge", &edgeOut[0], cmdIn.size());
}

int RewinderPlantSim::ToFile(const char* fname, float* data, int num)
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
