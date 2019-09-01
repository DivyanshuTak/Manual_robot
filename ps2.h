#ifndef _PS2_H
#define _PS2_H

#define Psxdat 	Pin2_3                                         
#define Psxcmd 	Port2_4                                        
#define Psxatn 	Port2_5                                        
#define Psxclk 	Port2_1                                        
#define Psxack 	Pin2_2    


extern unsigned char Bin_value[8];
extern unsigned int P;
//extern unsigned char Tx1 ;
//extern unsigned char Tx2 ;
extern unsigned char Dummy;
extern unsigned char Byte1 ;
extern unsigned char Byte2 ;
extern unsigned char Byte3 ;
extern unsigned char Byte4 ;
extern unsigned char Byte5 ;
extern unsigned char Byte6 ;
extern unsigned char Bytearr1[8] ;
extern unsigned char Bytearr2[8] ;
extern unsigned char Bytearr1count[8] ;
extern unsigned char Bytearr2count[8] ;
extern unsigned char Icount;


//************************** SUB DECLARATION ***************************************************************///
unsigned char Communicate(unsigned char Rxbyte,unsigned char Txbyte ,unsigned char Flag);
void Drive(void);
void Get_bts(void);
void Analog_mode(void);
void Watchdog(void);
extern unsigned char Strtofcomm;
extern unsigned char Status_;

#endif
