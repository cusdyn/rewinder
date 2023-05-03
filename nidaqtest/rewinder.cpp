#include "rewinder.h"

void AddItem(RingBuffer* rb, double d) 
{
	rb->d[rb->in++] = d;
	if (rb->in > BUFLEN)
		rb->in = 0;
}