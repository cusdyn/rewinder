#pragma once
#include<windows.h>
#include<vector>

#define MAX_LOG_LEN 1000000
#define YYTESTSZ 10000
#define VALVE_AMP_MIDRANGE (5.0) // 5 volts

#define KL  (8/0.1)          // LVDT volts-per-meter
#define EGZ (0.125 * 0.0254) // (m)Edge guide active zone + -about null(0.125" each way)
#define	EGV (10.0)               // +/ -voltage range
#define	KEG (EGV/EGZ)          // EdgeGuide volts per meter

#define EDGE_GUIDE_VSAT   (10.0)

class RewinderPlantSim
{
public:
	RewinderPlantSim(int loglen, float lvdtMidRange);
	~RewinderPlantSim();
	float CmdIn(float cmd);
	float EdgeGuideModel(float vlvdt);
	void  LogFilesOut(void);
	void  SetVedge(float veg);
private:
	int ToFile(const char* fname, float* data, int num);
	int maxlogLen;
	float KlvdtToEg;
	float lvdtVmid;
	float vEdge;
	std::vector<float> cmdIn;
	std::vector<float> lvdtOut;
	std::vector<float> edgeOut;

	//plant model coefficients
	/*
	double	b1 = 9.216250582849553e-4;
	double  b0 = 8.479686771438059e-4;
	double  a1 = -1.778800783071405;
	double  a0 = 0.778800783071405;
	*/
	float  b1 = 9e-4;
	float  b0 = 9e-4;
	float  a1 = -1.8;
	float  a0 = 0.8;


	float u[3];
	float y[3];

	double yy[YYTESTSZ];
	double uu[YYTESTSZ];

};

