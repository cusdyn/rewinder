#include "IIRdf2filt.h"

IIRdf2filt::IIRdf2filt(int N, const double* ain, const double* bin)
{
	// initialize filter order
	n = N;

	// initialize filter coefficients
	for (int i = 0; i < N; i++) {
		a.push_back(ain[i]);
		b.push_back(bin[i]);
		r.push_back(0);       // initialize the shift register to zeros
	}
}

double IIRdf2filt::FilterSample(double s) 
{
	double x = 0;
	double y = 0;
	double out = 0; // output

	for (int j = n - 1; j > 0; j--) {
		r[j] = r[j - 1];
		x = x - a[j] * r[j];
		y = y + b[j] * r[j];
	}

	x = x + s;
	out = y + b[0] * x;
	r[0] = x;

	return(out);
}

IIRdf2filt::~IIRdf2filt() {
	a.clear();
	b.clear();
}



