#pragma once
#include <vector>

using namespace std;
class IIRdf2filt
{
public:
	IIRdf2filt(int N, const double* ain, const double* bin);
	~IIRdf2filt();

	double FilterSample(double s);

private:
	vector<double> a;
	vector<double> b;
	vector<double> r;
	int            n;
};

