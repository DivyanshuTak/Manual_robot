#include<LPC17xx.h>
#include<math.h>
#include"LPC17xx_1.h"

#include"NVIC.h"				   
#include"GPIO.h"
#include"QEI.h"
#include"lcd.h"
#include"ADC.h"
#include"SPI.h"
#include"TIMER.h"
#include"../common.h"


extern unsigned short delayflag;
int enc1_status;
int enc2_status;
int enc3_status;
int p=0;
signed int enc1_count=0;
signed int enc2_count=0;
signed int enc3_count=0;

signed int enc1_count1=0;
signed int enc2_count1=0;
signed int enc3_count1=0;

int sec;

int ADCval[8]={0};

int f_ADCval[8]={0};
int flag_ADC[8]={0};
int curr_ADCval[8]={0};

int val_temp=0,val1=0,val2=0;
//-------------------------
short int presval=0;

//--------------------------------------------------------------------
void (*isrT0MR0Handler)	();
void (*isrT0MR1Handler)	();
void (*isrT0MR2Handler)	();
void (*isrT0MR3Handler)	();
void (*isrT0CR0Handler)	();
void (*isrT0CR1Handler)	();

void (*isrT1MR0Handler)	();
void (*isrT1MR1Handler)	();
void (*isrT1MR2Handler)	();
void (*isrT1MR3Handler)	();
void (*isrT1CR0Handler)	();
void (*isrT1CR1Handler)	();

void (*isrT2MR0Handler)	();
void (*isrT2MR1Handler)	();
void (*isrT2MR2Handler)	();
void (*isrT2MR3Handler)	();
void (*isrT2CR0Handler)	();
void (*isrT2CR1Handler)	();

void (*isrT3MR0Handler)	();
void (*isrT3MR1Handler)	();
void (*isrT3MR2Handler)	();
void (*isrT3MR3Handler)	();
void (*isrT3CR0Handler)	();
void (*isrT3CR1Handler)	();

void (*isrU0RLSHandler)	();
void (*isrU0THREHandler)();
void (*isrU0RDAHandler)	();
void (*isrU0CTIHandler)	();
void (*isrU0ABEOHandler)();
void (*isrU0ABTOHandler)();

void (*isrU1RLSHandler)	();
void (*isrU1THREHandler)();
void (*isrU1RDAHandler)	();
void (*isrU1CTIHandler)	();
void (*isrU1ABEOHandler)();
void (*isrU1ABTOHandler)();

void (*isrU2RLSHandler)	();
void (*isrU2THREHandler)();
void (*isrU2RDAHandler)	();
void (*isrU2CTIHandler)	();
void (*isrU2ABEOHandler)();
void (*isrU2ABTOHandler)();

void (*isrU3RLSHandler)	();
void (*isrU3THREHandler)();
void (*isrU3RDAHandler)	();
void (*isrU3CTIHandler)	();
void (*isrU3ABEOHandler)();
void (*isrU3ABTOHandler)();

void (*isrGPIOHandler)	();
							   
void (*isrQEIPOSHandler) ();
//---------------------------------------------------------------------
void ISR_TIMER0(void)	__irq	{
	
	int interrupt;
	//--------------------------
	if( T0IR & _MR0 )	{
		interrupt = _MR0;
		if(isrT0MR0Handler != NULL)
			(*isrT0MR0Handler)();   					//call Timer0-Match0 handler function
	}
	else if( T0IR & _MR1)	{
		interrupt = _MR1;
		if(isrT0MR1Handler != NULL)
			(*isrT0MR1Handler)();						//call Timer0-Match1 handler function
	}
	else if( T0IR & _MR2)	{
		interrupt = _MR2;
		if(isrT0MR2Handler != NULL)
			(*isrT0MR2Handler)();						//call Timer0-Match2 handler function
	}
	else if( T0IR & _MR3)	{
		interrupt = _MR3;
		if(isrT0MR3Handler != NULL)
			(*isrT0MR3Handler)();						//call Timer0-Match3 handler function
	}
	else if( T0IR & _CR0)	{
		interrupt = _CR0;
		if(isrT0CR0Handler != NULL)
			(*isrT0CR0Handler)();						//call Timer0-Capture0 handler function
	}
	else if( T0IR &  _CR1)	{
		interrupt = _CR1;
		if(*isrT0CR1Handler != NULL)
			(isrT0CR1Handler)();						//call Timer0-Capture1 handler function
	}
	//--------------------------
	T0IR |= interrupt;									//clear interrrupt
	//--------------------------
}
//--------------------------------------------------------------------
void ISR_TIMER1(void)	__irq	{
	
	int interrupt;
	//--------------------------
	if( T1IR & _MR0 )	{
		interrupt = _MR0;
		if(isrT1MR0Handler != NULL)
			(*isrT1MR0Handler)();   					//call Timer1-Match0 handler function
	}
	else if( T1IR & _MR1)	{
		interrupt = _MR1;
		if(isrT1MR1Handler != NULL)
			(*isrT1MR1Handler)();						//call Timer1-Match1 handler function
	}  
	else if( T1IR & _MR2)	{
		interrupt = _MR2;
		if(isrT1MR2Handler != NULL)
			(*isrT1MR2Handler)();						//call Timer1-Match2 handler function
	}
	else if( T1IR & _MR3)	{
		interrupt = _MR3;
		if(isrT1MR3Handler != NULL)
			(*isrT1MR3Handler)();						//call Timer1-Match3 handler function
	}
	else if( T1IR & _CR0)	{
		interrupt = _CR0;
		if(isrT1CR0Handler != NULL)
			(*isrT1CR0Handler)();						//call Timer1-Capture0 handler function
	}
	else if( T1IR &  _CR1)	{
		interrupt = _CR1;
		if(*isrT1CR1Handler != NULL)
			(isrT1CR1Handler)();						//call Timer1-Capture1 handler function
	}	 
	//--------------------------
	T1IR |= interrupt;									//clear interrrupt
	//--------------------------
}				  
//--------------------------------------------------------------------
void ISR_TIMER2(void)	__irq	{
	
	int interrupt;
	//--------------------------
	if( T2IR & _MR0 )	{
		interrupt = _MR0;
		if(isrT2MR0Handler != NULL)
			(*isrT2MR0Handler)();   					//call Timer2-Match0 handler function
	}
	else if( T2IR & _MR1)	{
		interrupt = _MR1;
		if(isrT2MR1Handler != NULL)
			(*isrT2MR1Handler)();						//call Timer2-Match1 handler function
	}
	else if( T2IR & _MR2)	{
		interrupt = _MR2;
		if(isrT2MR2Handler != NULL)
			(*isrT2MR2Handler)();						//call Timer2-Match2 handler function
	}
	else if( T2IR & _MR3)	{
		interrupt = _MR3;
		if(isrT2MR3Handler != NULL)
			(*isrT2MR3Handler)();						//call Timer2-Match3 handler function
	}
	else if( T2IR & _CR0)	{
		interrupt = _CR0;
		if(isrT2CR0Handler != NULL)
			(*isrT2CR0Handler)();						//call Timer2-Capture0 handler function
	}
	else if( T2IR &  _CR1)	{
		interrupt = _CR1;
		if(*isrT2CR1Handler != NULL)
			(isrT2CR1Handler)();						//call Timer2-Capture1 handler function
	}
	//--------------------------
	T2IR |= interrupt;									//clear interrrupt
	//--------------------------
}
//--------------------------------------------------------------------
void ISR_TIMER3(void)	__irq	{
	
	int interrupt;
	//--------------------------
	if( T3IR & _MR0 )	{
		interrupt = _MR0;
		if(isrT3MR0Handler != NULL)
			(*isrT3MR0Handler)();   					//call Timer3-Match0 handler function
	}
	else if( T3IR & _MR1)	{
		interrupt = _MR1;
		if(isrT3MR1Handler != NULL)
			(*isrT3MR1Handler)();						//call Timer3-Match1 handler function
	}
	else if( T3IR & _MR2)	{
		interrupt = _MR2;
		if(isrT3MR2Handler != NULL)
			(*isrT3MR2Handler)();						//call Timer3-Match2 handler function
	}
	else if( T3IR & _MR3)	{
		interrupt = _MR3;
		if(isrT3MR3Handler != NULL)
			(*isrT3MR3Handler)();						//call Timer3-Match3 handler function
	}
	else if( T3IR & _CR0)	{
		interrupt = _CR0;
		if(isrT3CR0Handler != NULL)
			(*isrT3CR0Handler)();						//call Timer3-Capture0 handler function
	}
	else if( T3IR &  _CR1)	{
		interrupt = _CR1;
		if(*isrT3CR1Handler != NULL)
			(isrT3CR1Handler)();						//call Timer3-Capture1 handler function
	}
	//--------------------------
	T3IR |= interrupt;									//clear interrrupt
	//--------------------------
}
//--------------------------------------------------------------------  
void ISR_UART0(void)	__irq	{
	
	int uartiir = U0IIR;
	//--------------------------
	if( (uartiir & _RLS) == _RLS )	{
		if(isrU0RLSHandler != NULL)
			(*isrU0RLSHandler)();   					//call UART0-Rx Line Status handler function
	}
	if( (uartiir & _THRE) == _THRE )	{
		if(isrU0THREHandler != NULL)
			(*isrU0THREHandler)();						//call UART0-Tx Holding Register Enable handler function
	}
	if( (uartiir & _RDA) == _RDA )	{
		if(isrU0RDAHandler != NULL)
			(*isrU0RDAHandler)();						//call UART0-Rx Data Available handler function
	}
	if( (uartiir & _CTI) == _CTI )	{
		if(isrU0CTIHandler != NULL)
			(*isrU0CTIHandler)();						//call UART0-Character Timeout handler function
	}
	if( (uartiir & _ABEO) == _ABEO )	{
		if(isrU0ABEOHandler != NULL)
			(*isrU0ABEOHandler)();						//call UART0-End Of Auto Baud handler function
	}
	if( (uartiir &  _ABTO) == _ABTO )	{
		if(*isrU0ABTOHandler != NULL)
			(isrU0ABTOHandler)();						//call UART0-Auto Baud Timeout handler function
	}
	//--------------------------
}
//--------------------------------------------------------------------  
void ISR_UART1(void)	__irq	{
	
	int uartiir = U1IIR;
	//--------------------------
	if( (uartiir & _RLS) == _RLS )	{
		if(isrU1RLSHandler != NULL)
			(*isrU1RLSHandler)();   					//call UART1-Rx Line Status handler function
	}
	if( (uartiir & _THRE) == _THRE )	{
		if(isrU1THREHandler != NULL)
			(*isrU1THREHandler)();						//call UART1-Tx Holding Register Enable handler function
	}
	if( (uartiir & _RDA) == _RDA )	{
		if(isrU1RDAHandler != NULL)
			(*isrU1RDAHandler)();						//call UART1-Rx Data Available handler function
	}
	if( (uartiir & _CTI) == _CTI )	{
		if(isrU1CTIHandler != NULL)
			(*isrU1CTIHandler)();						//call UART1-Character Timeout handler function
	}
	if( (uartiir & _ABEO) == _ABEO )	{
		if(isrU1ABEOHandler != NULL)
			(*isrU1ABEOHandler)();						//call UART1-End Of Auto Baud handler function
	}
	if( (uartiir &  _ABTO) == _ABTO )	{
		if(*isrU1ABTOHandler != NULL)
			(isrU1ABTOHandler)();						//call UART1-Auto Baud Timeout handler function
	}
	//--------------------------
}
//--------------------------------------------------------------------  
void ISR_UART2(void)	__irq	{
	
	int uartiir = U2IIR;
	//--------------------------
	if( (uartiir & _RLS) == _RLS )	{
		if(isrU2RLSHandler != NULL)
			(*isrU2RLSHandler)();   					//call UART2-Rx Line Status handler function
	}
	if( (uartiir & _THRE) == _THRE )	{
		if(isrU2THREHandler != NULL)
			(*isrU2THREHandler)();						//call UART2-Tx Holding Register Enable handler function
	}
	if( (uartiir & _RDA) == _RDA )	{
		if(isrU2RDAHandler != NULL)
			(*isrU2RDAHandler)();						//call UART2-Rx Data Available handler function
	}
	if( (uartiir & _CTI) == _CTI )	{
		if(isrU2CTIHandler != NULL)
			(*isrU2CTIHandler)();						//call UART2-Character Timeout handler function
	}
	if( (uartiir & _ABEO) == _ABEO )	{
		if(isrU2ABEOHandler != NULL)
			(*isrU2ABEOHandler)();						//call UART2-End Of Auto Baud handler function
	}
	if( (uartiir &  _ABTO) == _ABTO )	{
		if(*isrU2ABTOHandler != NULL)
			(isrU2ABTOHandler)();						//call UART2-Auto Baud Timeout handler function
	}
	//--------------------------
}
//--------------------------------------------------------------------  
void ISR_UART3(void)	__irq	{
	
	int uartiir = U3IIR;
	//--------------------------
	if( (uartiir & _RLS) == _RLS )	{
		if(isrU3RLSHandler != NULL)
			(*isrU3RLSHandler)();   					//call UART3-Rx Line Status handler function
	}
	if( (uartiir & _THRE) == _THRE )	{
		if(isrU3THREHandler != NULL)
			(*isrU3THREHandler)();						//call UART3-Tx Holding Register Enable handler function
	}
	if( (uartiir & _RDA) == _RDA )	{
		if(isrU3RDAHandler != NULL)
			(*isrU3RDAHandler)();						//call UART3-Rx Data Available handler function
	}
	if( (uartiir & _CTI) == _CTI )	{
		if(isrU3CTIHandler != NULL)
			(*isrU3CTIHandler)();						//call UART3-Character Timeout handler function
	}
	if( (uartiir & _ABEO) == _ABEO )	{
		if(isrU3ABEOHandler != NULL)
			(*isrU3ABEOHandler)();						//call UART3-End Of Auto Baud handler function
	}
	if( (uartiir &  _ABTO) == _ABTO )	{
		if(*isrU3ABTOHandler != NULL)
			(isrU3ABTOHandler)();						//call UART3-Auto Baud Timeout handler function
	}
	//--------------------------
}
//--------------------------------------------------------------------
void ISR_RIT(void)		__irq	{
	
	//--------------------------
	delayflag = 0;
	//--------------------------
	RICTRL = 0x00000001;								//clear interrrupt
	//--------------------------
}
//--------------------------------------------------------------------
void ISR_QEI(void)		__irq	{
	
	//--------------------------
	isrQEIPOSHandler();	
	//--------------------------

}

void ISR_EINT3(void)	__irq	{

//-----------------------------------

	if( (IO2IntStatR&0x00002000) == 0x00002000)
	{
		enc1_status=1;
		enc1_count1-=T0TC;
		T0TC=0x00000000; 			
	}
	else if( (IO2IntStatF&0x00002000)== 0x00002000)
	{
		enc1_status=2; 			
		enc1_count1+=T0TC;
		T0TC=0x00000000; 			
	}

//-----------------------------------
/*
	if( (IO2IntStatR&0x00000800) == 0x00000800)
	{
		enc2_status=1;
		enc2_count1-=T2TC;
		T2TC=0x00000000; 			
	}
	else if( (IO2IntStatF&0x00000800)== 0x00000800)
	{
		enc2_status=2; 			
		enc2_count1+=T2TC;
		
		T2TC=0x00000000; 			
	}
	*/
	if( (IO2IntStatR&0x800) == 0x000800)
	{  
		 T1TC=0; 
		StartTimer(TIMER1);
	}
	else if( (IO2IntStatF&0x000800)== 0x000800)
	{
		StopTimer(TIMER1);
    sec=T1TC; 
}	
//-----------------------------------

	if( (IO2IntStatR&0x00001000) == 0x00001000)
	{
		enc3_status=1;
		enc3_count1-=T1TC;
		T1TC=0x009;//000000; 			
	}
	else if( (IO2IntStatF&0x00001000)== 0x00001000)
	{
		enc3_status=2; 			
		enc3_count1+=T1TC;
		T1TC=0x00000000; 			
	}
	
	//---strobe bit--------------------------------
	//From 1/02/2014 the strobe pin has been shifted to Port2_8

	if( (IO2IntStatF&0x0000100) == 0x00000100)
	{		
/*		val_temp=Inputbin(COM2);
		val1=Inputbin(COM2);
		val2=Inputbin(COM2);

		if(val_temp==170)
		{
			if(val1==170)
				val_temp=0;
			else if(val2==170)
				val_temp=0;
		}	
		else if(val1==170)
		{
			if(val2==170)
				val_temp=0;
			else if(val_temp==170)
				val_temp=0;
			
			val1=val2;
			val2=val_temp;
		}
		else if(val2==170)
		{
			if(val2==170)
				val_temp=0;
			else if(val_temp==170)
				val_temp=0;
			
			val2=val1;
			val1=val_temp;
		}
				
				
		if(val_temp!=0)
		{
			presval = (val2 | (val1<<8));
			ang=presval/100.0; 
			mheading=ang*PI/180.0;
		}
	//---------------imu change setting------------------
		mheading=-mheading;
		flag_imu=1;		  
		*/
	}
	
	
	IO2IntClr = 0xFFFFFFFF;
	IO0IntClr = 0xFFFFFFFF;
	
	//-----------------------------------
}  

	/*The Original ISR Function
void ISR_ADC(void) __irq
  {
		int channel = ((AD0GDR & 0x7000000)>>24);
		switch(channel)
		  {
				case Adc0 : 
					ADCval[0] = ((AD0DR0 & 0x0000FFF0)>>4);
					break;
				case Adc1 : 
					ADCval[1] = ((AD0DR1 & 0x0000FFF0)>>4);
					break;
				case Adc2 : 
					ADCval[2] = ((AD0DR2 & 0x0000FFF0)>>4);
					break;
				case Adc3 : 
					ADCval[3] = ((AD0DR3 & 0x0000FFF0)>>4);
					break;
				case Adc4 : 
					ADCval[4] = ((AD0DR4 & 0x0000FFF0)>>4);
				  break;
				case Adc5 : 
					ADCval[5] = ((AD0DR5 & 0x0000FFF0)>>4);
				  break;
				case Adc6 : 
					ADCval[6] = ((AD0DR6 & 0x0000FFF0)>>4);
				  break;
				case Adc7 : 
					ADCval[7] = ((AD0DR7 & 0x0000FFF0)>>4);
				  break;
			}
	}*/
	
	//Modified ADC with Noise Filter for PS2 - edited on 29/12/2013
	void ISR_ADC(void)  __irq
  {
		int channel = ((AD0GDR & 0x7000000)>>24);
		switch(channel)
		  {
				case Adc0 : 
					curr_ADCval[0] = ((AD0DR0 & 0x0000FFF0)>>4);
					f_ADCval[0]+=curr_ADCval[0];
					flag_ADC[0]++;
					break;
				case Adc1 : 
					ADCval[1] = ((AD0DR1 & 0x0000FFF0)>>4);
					break;
				case Adc2 : 
					curr_ADCval[2] = ((AD0DR2 & 0x0000FFF0)>>4);
					f_ADCval[2]+=curr_ADCval[2];
					flag_ADC[2]++;
					break;
				case Adc3 : 
					curr_ADCval[3] = ((AD0DR3 & 0x0000FFF0)>>4);
					f_ADCval[3]+=curr_ADCval[3];
					flag_ADC[3]++;
					break;
				case Adc4 : 
					curr_ADCval[4] = ((AD0DR4 & 0x0000FFF0)>>4);
				  f_ADCval[4]+=curr_ADCval[4];
					flag_ADC[4]++;
					break;
				case Adc5 : 
					curr_ADCval[5] = ((AD0DR5 & 0x0000FFF0)>>4);
					f_ADCval[5]+=curr_ADCval[5];
					flag_ADC[5]++;
				  break;
				case Adc6 : 
					curr_ADCval[6] = ((AD0DR6 & 0x0000FFF0)>>4);
				  f_ADCval[6]+=curr_ADCval[6];
					flag_ADC[6]++;
					break;
			}
			if(flag_ADC[2]==10)
			{
				ADCval[2]=(int)f_ADCval[2]/10;
				flag_ADC[2]=f_ADCval[2]=0;
			}	
			if(flag_ADC[3]==10)
			{
				ADCval[3]=(int)f_ADCval[3]/10;
				flag_ADC[3]=f_ADCval[3]=0;
			}
					
			if(flag_ADC[4]==10)
			{
				ADCval[4]=(int)f_ADCval[4]/10;
				flag_ADC[4]=f_ADCval[4]=0;
			}
			if(flag_ADC[5]==10)
			{
				ADCval[5]=(int)f_ADCval[5]/10;
				flag_ADC[5]=f_ADCval[5]=0;
			}
			if(flag_ADC[0]==10)
			{
				ADCval[0]=(int)f_ADCval[0]/10;  
				flag_ADC[0]=f_ADCval[0]=0;
			}
	}

	void Enable(int interrupt,int inttype)	{
	
	static short vtoractive = 0;
	if(!vtoractive)	{
		
		vtoractive = 1;
		VTOR=0x10005000;
	}	
	unsigned long *VICAddr;
	ISER0 |= (1<<interrupt);
	switch(interrupt)	{

		case TIMER0:
				VICAddr=(unsigned long*)(VTOR+0x00000044);
				*VICAddr=(unsigned long)ISR_TIMER0;
			break;
		case TIMER1:
				VICAddr=(unsigned long*)(VTOR+0x00000048);
				*VICAddr=(unsigned long)ISR_TIMER1;
			break;
		case TIMER2:
				VICAddr=(unsigned long*)(VTOR+0x0000004C);
				*VICAddr=(unsigned long)ISR_TIMER2;
			break;
		case TIMER3:
				VICAddr=(unsigned long*)(VTOR+0x00000050);
				*VICAddr=(unsigned long)ISR_TIMER3;
			break;
		case UART0:
				VICAddr=(unsigned long*)(VTOR+0x00000054);
				*VICAddr=(unsigned long)ISR_UART0;
			break;
		case UART1:
				VICAddr=(unsigned long*)(VTOR+0x00000058);
				*VICAddr=(unsigned long)ISR_UART1;
			break;
		case UART2:
				VICAddr=(unsigned long*)(VTOR+0x0000005C);
				*VICAddr=(unsigned long)ISR_UART2;
			break;
		case UART3:
				VICAddr=(unsigned long*)(VTOR+0x00000060);
				*VICAddr=(unsigned long)ISR_UART3;
			break;
		case RIT:
				VICAddr=(unsigned long*)(VTOR+0x000000B4);
				*VICAddr=(unsigned long)ISR_RIT;
			break;
		case QEI:
				VICAddr=(unsigned long*)(VTOR+0x000000BC);
				*VICAddr=(unsigned long)ISR_QEI;
			break;
		case EINT3:
				VICAddr=(unsigned long*)(VTOR+0x00000094);
				*VICAddr=(unsigned long)ISR_EINT3;
			break;
		case ADC:
			  VICAddr=(unsigned long*)(VTOR+0x00000098);
				*VICAddr=(unsigned long)ISR_ADC;
			break;
	}

}
//--------------------------------------------------------------------
//--------------------------------------------------------------------

void On(int interrupt,void (*ISR)(void)) {
	 	
	switch(interrupt)	{
		//--------------------------------------------------
		//---------------------Timer 0----------------------
		case _T0MR0:
				isrT0MR0Handler = *ISR;
			break;
		case _T0MR1:
				isrT0MR1Handler = *ISR;
			break;
		case _T0MR2:
				isrT0MR2Handler = *ISR;
			break;
		case _T0MR3:
				isrT0MR3Handler = *ISR;
			break;
		case _T0CR0:
				isrT0CR0Handler = *ISR;
			break;
		case _T0CR1:
				isrT0CR1Handler = *ISR;
			break;
		//---------------------Timer 1----------------------
		case _T1MR0:
				isrT1MR0Handler = *ISR;
			break;
		case _T1MR1:
				isrT1MR1Handler = *ISR;
			break;
		case _T1MR2:
				isrT1MR2Handler = *ISR;
			break;
		case _T1MR3:
				isrT1MR3Handler = *ISR;
			break;
		case _T1CR0:
				isrT1CR0Handler = *ISR;
			break;
		case _T1CR1:
				isrT1CR1Handler = *ISR;
			break;
			
		//---------------------Timer 2----------------------
		case _T2MR0:
				isrT2MR0Handler = *ISR;
			break;
		case _T2MR1:
				isrT2MR1Handler = *ISR;
			break;
		case _T2MR2:
				isrT2MR2Handler = *ISR;
			break;
		case _T2MR3:
				isrT2MR3Handler = *ISR;
			break;
		case _T2CR0:
				isrT2CR0Handler = *ISR;
			break;
		case _T2CR1:
				isrT2CR1Handler = *ISR;
			break;
		//---------------------Timer 3----------------------
		case _T3MR0:
				isrT3MR0Handler = *ISR;
			break;
		case _T3MR1:
				isrT3MR1Handler = *ISR;
			break;
		case _T3MR2:
				isrT3MR2Handler = *ISR;
			break;
		case _T3MR3:
				isrT3MR3Handler = *ISR;
			break;
		case _T3CR0:
				isrT3CR0Handler = *ISR;
			break;
		case _T3CR1:
				isrT3CR1Handler = *ISR;
			break;	
		//---------------------UART 0----------------------
		case _U0RLS:
				isrU0RLSHandler = *ISR;
			break;
		case _U0THRE:
				isrU0THREHandler = *ISR;
			break;
		case _U0RDA:
				isrU0RDAHandler = *ISR;
			break;
		case _U0CTI:
				isrU0CTIHandler = *ISR;
			break;
		case _U0ABEO:
				isrU0ABEOHandler = *ISR;
			break;
		case _U0ABTO:
				isrU0ABTOHandler = *ISR;
			break;
		//---------------------UART 1----------------------
		case _U1RLS:
				isrU1RLSHandler = *ISR;
			break;
		case _U1THRE:
				isrU1THREHandler = *ISR;
			break;
		case _U1RDA:
				isrU1RDAHandler = *ISR;
			break;
		case _U1CTI:
				isrU1CTIHandler = *ISR;
			break;
		case _U1ABEO:
				isrU1ABEOHandler = *ISR;
			break;
		case _U1ABTO:
				isrU1ABTOHandler = *ISR;
			break;
		//---------------------UART 2----------------------
		case _U2RLS:
				isrU2RLSHandler = *ISR;
			break;
		case _U2THRE:
				isrU2THREHandler = *ISR;
			break;
		case _U2RDA:
				isrU2RDAHandler = *ISR;
			break;
		case _U2CTI:
				isrU2CTIHandler = *ISR;
			break;
		case _U2ABEO:
				isrU2ABEOHandler = *ISR;
			break;
		case _U2ABTO:
				isrU2ABTOHandler = *ISR;
			break;
		//---------------------UART 3----------------------
		case _U3RLS:
				isrU3RLSHandler = *ISR;
			break;
		case _U3THRE:
				isrU3THREHandler = *ISR;
			break;
		case _U3RDA:
				isrU3RDAHandler = *ISR;
			break;
		case _U3CTI:
				isrU3CTIHandler = *ISR;
			break;
		case _U3ABEO:
				isrU3ABEOHandler = *ISR;
			break;
		case _U3ABTO:
				isrU3ABTOHandler = *ISR;
			break;	
		//--------------------------------------------------
		case _QEIPOS:
				isrQEIPOSHandler = *ISR;
			break;
	}
}
//--------------------------------------------------------------------
