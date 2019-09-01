#ifndef _PLL_H
#define _PLL_H
//----------------------------------
#include "LPC17xx_1.h"

#define CLK	120000000

void InitPLL();
void waitms(unsigned int);
void wait(unsigned int);
void waitus(unsigned int);

#endif
