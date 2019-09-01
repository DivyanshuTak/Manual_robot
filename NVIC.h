#ifndef _NVIC_H
#define _NVIC_H

#include<LPC17xx.h>
#include"LPC17xx_1.h"
#include"PLL.h"

#define RISING 	1
#define FALLING 2
#define BOTH	3

#define NULL 0
//-----------------------------
#define WDT			0
#define TIMER0		1
#define TIMER1		2
#define TIMER2		3
#define TIMER3		4
#define UART0		5
#define UART1		6
#define UART2		7
#define UART3		8
#define PWM1		9
#define I2C0		10
#define I2C1		11
#define I2C2		12
#define SPI			13
#define SSP0		14
#define SSP1		15
#define PLL0		16
#define RTC 		17
#define EINT0		18
#define EINT1		19
#define EINT2		20
#define EINT3		21
#define GPIO		21	  						//EINT3 channel is shared with GPIO interrupts
#define ADC			22
#define BOD			23
#define USB			24
#define CAN			25
#define GPDMA		26
#define I2S			27
#define ETHERNET	28
#define RIT			29
#define MCPWM		30
#define QEI			31
#define PLL1		32
#define USBAI		33
#define CANAI		34
//-----------------------------
#define _WDINT		0

#define _T0MR0 		1
#define _T0MR1 		2
#define _T0MR2 		3
#define _T0MR3 		4
#define _T0CR0 		5
#define _T0CR1 		6

#define _T1MR0 		7
#define _T1MR1 		8
#define _T1MR2 		9
#define _T1MR3 		10
#define _T1CR0 		11
#define _T1CR1 		12

#define _T2MR0 		13
#define _T2MR1 		14
#define _T2MR2 		15
#define _T2MR3 		16
#define _T2CR0 		17
#define _T2CR1 		18

#define _T3MR0 		19
#define _T3MR1 		20
#define _T3MR2 		21
#define _T3MR3 		22
#define _T3CR0 		23
#define _T3CR1 		24

#define _U0RLS 		25
#define _U0THRE 	26
#define _U0RDA 		27
#define _U0CTI 		28
#define _U0ABEO 	29
#define _U0ABTO 	30

#define _U1RLS 		31
#define _U1THRE 	32
#define _U1RDA 		33
#define _U1CTI 		34
#define _U1ABEO 	35
#define _U1ABTO 	36

#define _U2RLS 		37
#define _U2THRE 	38
#define _U2RDA 		39
#define _U2CTI 		40
#define _U2ABEO 	41
#define _U2ABTO 	42

#define _U3RLS 		43
#define _U3THRE 	44
#define _U3RDA 		45
#define _U3CTI 		46
#define _U3ABEO 	47
#define _U3ABTO 	48

#define _QEIPOS 	50
//-----------------------------

extern int enc1_status;
extern int enc2_status;
extern int enc3_status;

extern signed int enc1_count;
extern signed int enc2_count;
extern signed int enc3_count;

extern signed int enc1_count1;
extern signed int enc2_count1;
extern signed int enc3_count1;

extern int p,sec;

extern int ADCval[8];
extern int val1,val2,val_temp;
extern short int presval;


//-----------------------------
#define _MR0 		0x00000001
#define _MR1 		0x00000002
#define _MR2 		0x00000004
#define _MR3 		0x00000008
#define _CR0 		0x00000010
#define _CR1 		0x00000020
//-----------------------------
#define _RLS 		0x00000006
#define _THRE 		0x00000002
#define _RDA 		0x00000004
#define _CTI 		0x0000000C
#define _ABEO 		0x00000100
#define _ABTO 		0x00000200
//-----------------------------
void On(int interrupt,void (*ISR)());
void Enable(int interrupt,int inttype=NULL);
//-----------------------------

#endif
