#include<LPC17xx.h> 
#include<math.h> 									// For abs() function

#include "GPIO.h"
#include "UART.h"
#include "TIMER.h"
#include "lcd.h"
#include "ps2.h"
#include "../common/common_lpc.h"
//************************* VARIABLE DECLARATION ************************************************************'''
unsigned char Dummy;
unsigned char Bin_value[8]= {1,2,4,8,16,32,64,128};
unsigned int P; 
unsigned char Byte1 ;
unsigned char Byte2 ;
unsigned char Byte3 ;
unsigned char Byte4 ;
unsigned char Byte5 ;
unsigned char Byte6 ;
unsigned char Bytearr1[8] ;
unsigned char Bytearr2[8] ;
unsigned char Bytearr1count[8] ;
unsigned char Bytearr2count[8] ;
unsigned char Strtofcomm=0x01;
unsigned char Status_=0x42;


//**********************************************************************************************************
unsigned char Div_fect ;
unsigned char Lpwm=0 ;
unsigned char Rpwm=0 ;
//unsigned char Tx1 ;
//unsigned char Tx2 ;
unsigned char Icount=0;

//**********************Button Aliases*************************************************************************
#define Selectbt  Bytearr1[0] 
#define L3bt  Bytearr1[1] 
#define R3bt  Bytearr1[2] 
#define Startbt  Bytearr1[3] 
#define Upbt  Bytearr1[4] 
#define Rightbt  Bytearr1[5]
#define Downbt  Bytearr1[6] 
#define Leftbt  Bytearr1[7] 
#define L2bt  Bytearr2[0] 
#define R2bt  Bytearr2[1]
#define L1bt  Bytearr2[2] 
#define R1bt  Bytearr2[3] 
#define Trianglebt  Bytearr2[4]
#define Circlebt  Bytearr2[5] 
#define Crossbt  Bytearr2[6]
#define Squarebt  Bytearr2[7] 

/***********************************************************************************************************************''
*********************************PS2 buttons description*****************************************************************
'L1->       Bytearr2(3)
'L2->       Bytearr2(1)
'L3->
'R1->
'R2->
'R3->       Bytearr2(2)
'SELECT->   Bytearr1(1)
'START->
'TRIANGLE-> Bytearr2(5)
'O->
'X->        Bytearr2(7)
'SQUARE->
'UP->
'RIGHT->
'DOWN->
'LEFT->
'****************************/

//************************** INITIALIZATIONS ***************************************************************


/*
reset(Ch1_a);
reset(Ch1_b);
reset(Ch2_a);
reset(Ch2_b);
reset(Ch3_a);
reset(Ch3_b);
*/
//******************* PS2 PROTOCOL CODE - DON'T CHANGE THIS ************************************************
void Analog_mode(void)
{
cls();
lcd("ps2 Testing");
lowerline();

				waitus(50);

				do                                                          //Go into config mode
				{
				reset(Psxatn);
				Dummy=Communicate(Dummy , Strtofcomm , 0);
				Dummy=Communicate(Dummy , 0x43 , 0);         // 0x43
				if(Dummy == 0x73)                   // 0x73
					goto S_com;
				else if(Dummy == 0x41)   								// 0x41
					break;
				set(Psxatn);
				waitus(50);
				}
				while(1);
				
				Dummy=Communicate(Dummy , 0x00 , 0);
				Dummy=Communicate(Dummy , 0x01 , 0);
				Dummy=Communicate(Dummy , 0x00 , 0);
				set(Psxatn);
				waitus(50);
				lcd("1");
				// '''''***************************************************************"'''''''''
				do                                                          //Turn on analog mode
				{	
						reset(Psxatn);
						Dummy=Communicate(Dummy , Strtofcomm , 0);
						Dummy=Communicate(Dummy , 0x44 , 0);
						if(Dummy == 0xF3)
							 break;
						set(Psxatn);
						waitus(50);
				}
				while(1);
				
				Dummy=Communicate(Dummy , 0x00 , 0);
				Dummy=Communicate(Dummy , 0x01 , 0);
				Dummy=Communicate(Dummy , 0x03 , 0);
				Dummy=Communicate(Dummy , 0x00 , 0);
				Dummy=Communicate(Dummy , 0x00 , 0);
				Dummy=Communicate(Dummy , 0x00 , 0);
				Dummy=Communicate(Dummy , 0x00 , 0);
				set(Psxatn);
				waitus(50);
				lcd("2");
				// ''''''''''''***************************************************************
				do                                                          //Exit config mode
				{	
						reset(Psxatn);
						Dummy=Communicate(Dummy , Strtofcomm , 0);
						Dummy=Communicate(Dummy , 0x43 , 0);
						cls();
						lcd(Dummy);
						if(Dummy == 0xF3)
							 break;
						set(Psxatn);
						waitus(50);
				}
				while(1);
	
				Dummy=Communicate(Dummy , 0x00 , 0);
				Dummy=Communicate(Dummy , 0x00 , 0);
				Dummy=Communicate(Dummy , 0x00 , 0);                          //5A
				Dummy=Communicate(Dummy , 0x00 , 0);                          //5A
				Dummy=Communicate(Dummy , 0x00 , 0);                          //5A
				Dummy=Communicate(Dummy , 0x00 , 0);                          //5A
				Dummy=Communicate(Dummy , 0x00 , 0);                          //5A

				set(Psxatn);
				waitus(50);
//'''''******************************************************************'''''''''
//'****** Beginning ******

S_com:
//'***********************
//'***** LCD Testing******
	lcd("success");
	upperline();
}
void Watchdog(void)
{
	
while(1)
	{
   //'Start Watchdog
   //'******************* PS2 PROTOCOL CODE - DON'T CHANGE THIS ******************
   reset(Psxatn);
   Dummy=Communicate(Dummy , Strtofcomm , 0);
   Dummy=Communicate(Dummy , Status_ , 0);
   if(Dummy != 0x73)
	 {
      Byte1 = 255;
      Byte2 = 255;
      Byte3 = 128;
      Byte4 = 128;
      Byte5 = 128;
      Byte6 = 128;
      Icount++;
      if(Icount > 50)
			{
         reset(Ch1_a);
         reset(Ch1_b);
         reset(Ch2_a);
         reset(Ch2_b);
         reset(Ch3_a);
         reset(Ch3_b);
         
				Tx1 = 64;
				Tx2 = 192;
				Printbin(COM1 , Tx1);
				Printbin(COM1 , Tx2);	
         
				cls();
        lcd("Error");
        Analog_mode();
         
			}
      goto Invalidmode;
		}
   else	
      Icount = 0;

   Dummy=Communicate(Dummy , 0x00 , 0);
   
	 for(P=1;P<=6;P++)
	 {
      Dummy=Communicate(Dummy , 0x00 , P);
      switch(P)
			{
         case 1 :
            Byte1 = Dummy;
						break;
         case 2 :
            Byte2 = Dummy;
						break;
         case 3:
            Byte3 = Dummy;
						break;
         case 4:
            Byte4 = Dummy;
						break;
         case 5:
            Byte5 = Dummy;
						break;
         case 6:
            Byte6 = Dummy;
						break;
			 }
		}

   for(P=0;P<=7;P++)
	 {
      Bytearr1[P] = Byte1%2;
      Byte1 /= 2;
   }
	 for(P=0;P<=7;P++)
	 {
      Bytearr2[P] = Byte2%2;
      Byte2 /= 2;
   }
	 
   Drive();
	 
	 if(!Upbt)
	 {
			cls();
			lcd("UP");
	 }
	 else if(!Downbt)
	 {
			cls();
			lcd("DOWN");
	 }
	 else if(!Rightbt)
	 {
			cls();
		  lcd("RIGHT");
	 }
	 else if(!Leftbt)
	 {
			cls();
		  lcd("LEFT");
	 }
	 else if(!L1bt)
	 {
			cls();
		  lcd("L1");
	 }
	 else if(!L2bt)
	 {
			cls();
		  lcd("L2");
	 }
	 else if(!L3bt)
	 {
			cls();
		  lcd("L3");
	 }
	 else if(!R1bt)
	 {
			cls();
		  lcd("R1");
	 }
	 else if(!R2bt)
	 {
			cls();
		  lcd("R2");
	 }
	 else if(!R3bt)
	 {
			cls();
		  lcd("R3");
	 }
	 else if(!Trianglebt)
	 {
			cls();
		  lcd("Trianglebt");
	 }
	 else if(!Circlebt)
	 {
			cls();
		  lcd("Circlebt");
	 }
	 else if(!Crossbt)
	 {
			cls();
		  lcd("Crossbt");
	 }
	 else if(!Squarebt)
	 {
			cls();
		  lcd("");
	 }
	 else if(!Selectbt)
	 {
			cls();
		  lcd("Selectbt");
	 }
	 else if(!Startbt)
	 {
			cls();
		  lcd("Startbt");
	 }
	 
	 		
//   '******************* PS2 PROTOCOL CODE - DON'T CHANGE THIS ******************
   Invalidmode:
		waitus(50);
		set(Psxclk);
		set(Psxatn);

//'Reset Watchdog
	}
}

unsigned char Communicate(unsigned char Rxbyte,unsigned char Txbyte ,unsigned char Flag)
{
   unsigned int I;
   unsigned char Dummy1;
   Rxbyte = 0;
   waitus(30);
	 
		for(I=0;I<=7;I++)
		{
   
      reset(Psxclk);
      Dummy1 = Txbyte%2;

      if(Dummy1)
         set(Psxcmd);
      else
         reset(Psxcmd);

      waitus(30);
      set(Psxclk);
			
      if(Psxdat)
         Rxbyte += Bin_value[I];

      waitus(30);
      Txbyte /= 2;
		}
   waitus(20);
	 return Rxbyte;
 }

 //'******************** SABERTOOTH SUBROUTINE ***************************************************************
void Drive(void)                                                 //'values changed 100->95 and 156->166
{

/*'   If Bytearr2(2) = 0 Then                                  'R2
'       Div_fect = 2
'   If Bytearr1(4) = 0 Then                                  'START
'       Div_fect = 6
'   Else
'       Div_fect = 3
'   End If */
   cls();
   lcd(Byte4);
   lowerline();
   lcd(Byte6);

    if( Byte6 >= 90 && Byte6 <= 166 )
       Tx1 = 64;
		else if( Byte6 > 166 )                                  //forward
		{
			 Lpwm = Byte6 - 128;
       Lpwm /= Div_fect;

        if( Lpwm >= 62 )
         Tx1 = 1;
				else
         Tx1 = 64 - Lpwm;
		}   

		else if( Byte6 < 90 )                                   //backward
		{		
				Lpwm = 128 - Byte6;
				Lpwm /= Div_fect;

        if( Lpwm >= 62 )
         Tx1 = 127;
				else
         Tx1 = Lpwm + 64;
		}  


    if( Byte4 >= 90 && Byte4 <= 166 )
				Tx2 = 192;
		
		else if( Byte4 > 166 )                                  //forward
		{		
				Rpwm = Byte4 - 128;
				Rpwm /= Div_fect;

        if( Rpwm >= 62 )
					Tx2 = 129;
				else
					Tx2 = 192 - Rpwm;
		}

   else if( Byte4 < 90 )                                   //backward
   {    
				Rpwm = 128 - Byte4;
				Rpwm /= Div_fect;

        if( Rpwm >= 62 )
         Tx2 = 255;
       else
         Tx2 = 192 + Rpwm;
		}

    Printbin(COM1 , Tx1);
    Printbin(COM1 , Tx2);

}
