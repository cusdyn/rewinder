#pragma once

#define BUFLEN 1000

typedef enum {
	none=0,
	left,
	right,
	halt
}Jog;

typedef struct {
	double d[BUFLEN];
	int     in;
	int     out;
}RingBuffer;


