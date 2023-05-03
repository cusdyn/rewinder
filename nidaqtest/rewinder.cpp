#include "rewinder.h"

float BoxCarFilter(Boxcar *b, double d, int n) 
{
	float sum = 0;

	b->d[b->in++] = d;
	if (b->in == n)
		b->in = 0;

	for (int i = 0; i < n; i++) {
		sum += b->d[i];
	}

	return(sum / n);

}

float BoxCarFilterLong(BoxcarLong* b, double d, int n)
{
	float sum = 0;

	b->d[b->in++] = d;
	if (b->in == n)
		b->in = 0;

	for (int i = 0; i < n; i++) {
		sum += b->d[i];
	}

	return(sum / n);

}