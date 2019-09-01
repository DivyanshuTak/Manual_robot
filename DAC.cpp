#include "DAC.h"

void dacinit()
{
	PCLKSEL0 &= ~(0x3<<22);                  //30MHz periferal clock selected
	
	PINSEL1 &= ~(0x3<<20);
	PINSEL1 |= (0x2<<20);
	
	DACR &= (~(0x3FF<<6));
	DACR &= (~(0x1<<16));
	
	DACCTRL &= (~(0x1<<3));	  
}

//Put value between 0-1023 to get analog output ranging from 0V to 3.3V
void DAC_out(int value)
{
	if(value>1023)
		value=1023;
	else if(value<0)
		value=0;
	
	DACR &= ~(0x3FF<<6);
	DACR |= value<<6; 
}
