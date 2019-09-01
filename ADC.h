#ifndef _ADC_H
#define _ADC_H

#define Adc0  	0
#define Adc1  	1
#define Adc2  	2
#define Adc3  	3
#define Adc4  	4
#define Adc5  	5
#define Adc6  	6
#define Adc7  	7

#define NOW 	1

int GetADC(int channel);
void ConfigADC(int channel);
void EnableADC(int channel);
void DisableADC(int channel);


#endif
