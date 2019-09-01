//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
#include<LPC17xx.h>    //Always Put this header file on the top of the code else u will get errors

#include"LPC17xx_1.h"
#include"PLL.h"
#include"NVIC.h"

unsigned short delayflag;
//-------------------------------------------
void InitPLL()
{
	static int pllinit = 0;
	if(pllinit)	return;

    //Clock Configuration
	SCS = 0x00000060;		 			//Enable main oscillator
	CLKSRCSEL=0x01;			 			//Select clock source-Main oscillator
	
	//PLL0 Configuration
	PLL0CON=0x01;						//Disconnect PLL0
	PLL0FEED=0xAA;
	PLL0FEED=0x55;

	PLL0CON=0x0;			 			//Disable PLL0
	PLL0FEED=0xAA;
	PLL0FEED=0x55;
	//-------------------
	PLL0CFG=0x0000000E;		 			//Multipy-Divide PLL ( 2*M *Fin / N )
	PLL0FEED=0xAA;
	PLL0FEED=0x55;
	
	PLL0CON=0x1;
	PLL0FEED=0xAA;
	PLL0FEED=0x55;
		 
	while(!(PLL0STAT & 0x01000000));

	PLL0CON=0x3;	
	PLL0FEED=0xAA;
	PLL0FEED=0x55;	
	//-------------------
	CCLKCFG = 0x00000002;	 			//Divide System Clock
	//-------------------

	pllinit = 1;			  
}
//-------------------------------------------

void waitus(unsigned int delay)
{
	//-----------------------
	delay = 30*delay-22;
	delayflag   =	1;
	PCONP	   |= 	0x00010000;		   //Power
	RICTRL 		=	0x00000000;
	RICOMPVAL 	=	delay;
	RIMASK		=	0x00000000;
	RICTRL 		=	0x0000000E;		  //timer halted for debugging
	while(delayflag);
	RICTRL 		= 	0x00000000;
 	PCONP	   &=  ~0x00010000;
	//-----------------------
}

void waitms(unsigned int delay)
{
	//-----------------------
	delay = 30000*delay-22;
	delayflag 	=	1;
	PCONP	   |=	0x00010000;
	RICTRL 		=	0x00000000;
	RICOMPVAL 	=	delay;
	RIMASK		=	0x00000000;
	RICTRL 		=	0x0000000A;			  
 	while(delayflag);
	RICTRL 		=	0x00000000;
	PCONP	   &=  ~0x00010000;													  
	//-----------------------
}

void wait(unsigned int delay)
{
	//-----------------------
	while(delay--)
		waitms(1000);
	//-----------------------
}
//-------------------------------------------
