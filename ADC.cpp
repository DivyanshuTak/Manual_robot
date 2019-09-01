#include<LPC17xx.h>				 //Always Put this header file on the top of the code else u will get errors
#include"LPC17xx_1.h"

#include"PLL.h"
#include"NVIC.h"
#include"ADC.h"

/* To Start ADC first Enable ADC interrupts in NVIC using
						Enable(ADC);
		Then config the pins we want to use as ADC channels by the function ConfigADC()
		and channel as argument. This uses PINSEL reigsters to select the ADC function on the given pin.	
						ConfigADC(Adc1);
		When we want to start receiving values of ADC conversion we wil have to enable the individual ADC channel interrupts
		For this, use the EnableADC() function as 
						EnableADC(Adc1);
		This line will start the interrupt on desired channel. Whenever the conversion is complete an interrupt is called which
		transfers the value into ADCval[n] variable.
		To stop the ADC channel, simply call the DisableADC() function as follows
						DisableADC(Adc1);
		To get the final value for ADC channel no. n	refer to the variable ADCval[n].			
*/
/*
Last updated on 2/1/2013
*/

void ConfigADC(int channel)
{	 
	PCONP |= 0x00001000;		
  int adc_clkdiv;
	AD0INTEN &= 0xFF;
	switch (channel)	
	{
		case Adc0:
				PINSEL1 &= ~0x0000C000;
				PINSEL1 |=  0x00004000;
			break;
		case Adc1:
				PINSEL1 &= ~0x00030000;
				PINSEL1 |=  0x00010000;
			break;
		case Adc2:
      	PINSEL1 &= ~0x000C0000;
				PINSEL1 |=  0x00040000; 
   	  		break;
		case Adc3:
    		PINSEL1 &= ~0x00300000;
				PINSEL1 |=  0x00100000;
   	  		break;
		case Adc4:
				PINSEL3 &= ~0x30000000;
				PINSEL3 |=  0x30000000;
			break;
		case Adc5:
				PINSEL3 &= ~0xC0000000;
				PINSEL3 |=  0xC0000000;
			break;
		case Adc6:
				PINSEL0 &= ~0x000000C0;
				PINSEL0 |=  0x00000080;
			break;
		case Adc7:
				PINSEL0 &= ~0x00000030;
				PINSEL0 |=  0x00000020;														 
			break;
	}		
	adc_clkdiv = 4;
	AD0CR |= 1<<channel;
	AD0CR |= adc_clkdiv<<8 | 1<<16 | 1<<21;	
}

void EnableADC(int channel)
{
	AD0INTEN |= 1<<channel;
}

void DisableADC(int channel)
{
	switch(channel)
  {
		case Adc0:
			AD0INTEN &= 0xFE;
			break;
		case Adc1:
			AD0INTEN &= 0xFD;
			break;
		case Adc2:
			AD0INTEN &= 0xFB;
			break;
		case Adc3:
			AD0INTEN &= 0xF7;
			break;
		case Adc4:
			AD0INTEN &= 0xEF;
			break;
		case Adc5:
			AD0INTEN &= 0xDF;
			break;
		case Adc6:
			AD0INTEN &= 0xBF;
			break;
		case Adc7:
			AD0INTEN &= 0x7F;
			break;
	}
}


int GetADC(int channel)
{	 
	PCONP |= 0x00001000;	
	int adc_clkdiv;
	
	switch (channel)	
	{
		case Adc0:
				PINSEL1 &= ~0x0000C000;
				PINSEL1 |=  0x00004000;
			break;
		case Adc1:
				PINSEL1 &= ~0x00030000;
				PINSEL1 |=  0x00010000;
			break;
		case Adc2:
      			PINSEL1 &= ~0x000C0000;
				PINSEL1 |=  0x00040000; 
   	  		break;
		case Adc3:
    			PINSEL1 &= ~0x00300000;
				PINSEL1 |=  0x00100000;
   	  		break;
		case Adc4:
				PINSEL3 &= ~0x30000000;
				PINSEL3 |=  0x30000000;
			break;
		case Adc5:
				PINSEL3 &= ~0xC0000000;
				PINSEL3 |=  0xC0000000;
			break;
		case Adc6:
				PINSEL0 &= ~0x00000030;
				PINSEL0 |=  0x00000080;
			break;
		case Adc7:
				PINSEL0 &= ~0x000000C0;
				PINSEL0 |=  0x00000020;														 
			break;
	}

	adc_clkdiv = 4;
	AD0CR = 1<<channel;
	AD0CR |= adc_clkdiv<<8 | 1<<16 | 1<<21;
	waitus(100);
	
	int addr,ADCval=0;
	switch (channel)	
	{
		case Adc0:
				addr  = AD0DR0;
				if(addr & 0x80000000)
					ADCval=((addr & 0x0000FFF0)>>4); 
			break;
		case Adc1:
				addr = AD0DR1;
				if(addr & 0x80000000)
					ADCval=((addr & 0x0000FFF0)>>4);
  			break;	  
		case Adc2:
				addr  = AD0DR2;
				if(addr & 0x80000000)
					ADCval=((addr & 0x0000FFF0)>>4);	
  			break;
		case Adc3:
				addr  = AD0DR3;
				if(addr & 0x80000000)
    				ADCval=((addr & 0x0000FFF0)>>4);
			break;
		case Adc4:
				addr  = AD0DR4;
				if(addr & 0x80000000)
   					ADCval=((addr & 0x0000FFF0)>>4);
			break;
		case Adc5:
				addr  = AD0DR5;
				if(addr & 0x80000000)
    				ADCval=((addr & 0x0000FFF0)>>4);
			break;
		case Adc6:
				addr  = AD0DR6;
				if(addr & 0x80000000)
					ADCval=((addr & 0x0000FFF0)>>4);
			break;
		case Adc7:
				addr  = AD0DR7;
				if(addr & 0x80000000)
					ADCval=((addr & 0x0000FFF0)>>4);
			break;
	}	 
	AD0CR = 0x01;
	return ADCval; 
}

