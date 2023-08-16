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

#define LVDT_MID_VRANGE  (4.0)
#define EDGE_MID_VRANGE  (0.0)

class RewinderPlantSim
{
public:
	RewinderPlantSim(int loglen);
	~RewinderPlantSim();
	float CmdIn(float cmd);
	float EdgeGuideModel(float vlvdt);
	void  LogFilesOut(void);
	void  SetPedge(float peg);
private:
	int ToFile(const char* fname, float* data, int num);
	int maxlogLen;
	float KlvdtToEg;
	float pEdge;
	float vEdge;
	std::vector<float> cmdIn;
	std::vector<float> lvdtOut;
	std::vector<float> edgeOut;

	//plant model coefficients from rewinder_discrete.m
	//float  b1 = 9.2163e-04;
	//float  b0 = 8.4797e-04;
	//float  a1 = -1.7788;
	//float  a0 = 0.7788;
	float  b1 = (float)2.3990e-04;
	float  b0 = (float)2.3011e-04;
	float  a1 = (float)-1.8825;
	float  a0 = (float)0.8825;



	float u[3];
	float y[3];

	double yy[YYTESTSZ];
	double uu[YYTESTSZ];

};

