#pragma once

#define FLEN 1
typedef struct {
	double  d[FLEN];
	int     in;
}Boxcar;

#define LONGFLEN 20
typedef struct {
	double  d[LONGFLEN];
	int     in;
}BoxcarLong;

float BoxCarFilter(Boxcar* b, double d, int n);
float BoxCarFilterLong(BoxcarLong* b, double d, int n);

typedef enum {
	none=0,
	left,
	right,
	halt
}Jog;



