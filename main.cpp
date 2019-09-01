#include "common.h"
#include<math.h>
///   150 for acc
int main()
{	
	InitPLL();
	Enable(RIT);	
	
	ConfigPort(Port0,0x800173E5);   //d 	// 1000_0000_0111_1111_1111_0111_1110_0101
		//	7:0		1110_0101	:			relay-snsr-snsr-Cap2.0(ENC2)-Adc6(3)-Tx0-Rx3(zone detect for auto)-Tx3
		//	15:8	0111_0011	:			Actuation-Reserved-Reserved-Reserved-Rx2_imu-snsr-snsr-relay
		//	23:16	0000_0001	:			Adc0(7)-Actuation-Actuation-Actuation-Actuation-Actuation-Actuation-Actuation
		//	31:24	1000_0000	:		 	Reserved-Bitwait2-Bitwait3-Bitwait1-Bitwait4-Adc3(4)-(Adc2(5)),Adc1(6)
	
	ConfigPort(Port1,0x3B6838EC);	// 0011_1011_0110_1011_1011_1000_1110_1100
		//	7:0		1110_1100	:			Reserved-Reserved-Reserved-reed3-Reserved-Reserved-reed2-reed1
		//	15:8	0011_1000	:			PS2_cmd-PS2_ack-Reserved-Reserved-Reserved-Proxy2-Proxy1-reed4
		//	23:16	0110_1000	:			proxy-Xtra-E-proxy-Rs-Capture1.0(ENC3)-proxy-proxy
	  //	31:24	0011_1011	:			Adc5(2)-Adc4(1)-Db7-Db4-Db6-Cap0.0(ENC1)-Db5-Xtra
	
	ConfigPort(Port2,0xFFFFC46F);	// 1111_1111_1111_1111_1100_0111_1111_1000
		//	7:0		0110_1111	:			Actuation-Actuation-relay-relay-relay-snsr-snsr-input
		//	15:8	1100_0100	:			Reserved-Reserved-Eint3(ENC1)-Eint2(ENC3)-Eint1(ENC2)-ISP-Actuation-Actuation
	
	ConfigPort(Port3,0xF9FFFFFF);	// 1111_1001_1111_1111_1111_1111_1111_1111
		//	31:24	1111_1001	:			Reserved-Reserved-Reserved-Reserved-Reserved-Xtra(Driving)-PS2_data-Reserved
		
	ConfigPort(Port4,0xFFFFFFFF);	// 1111_1111_1111_1111_1111_1111_1111_1111
		//	31:24	1111_1111	:			Reserved-Reserved-snsr-snsr-Reserved-Reserved-Reserved-Reserved
  
	ConfigPortMode(Port0,PULLUP);		  					  //	Enable Pull down
	PINMODE1 |= 0x03<<12;
	
	ConfigPortMode(Port1,PULLDOWN);								//	Enable Pull up
	ConfigPortMode(Port2,PULLDOWN);								//	Enable Pull up
	ConfigPortMode(Port3,PULLDOWN);								//	Enable Pull up
	ConfigPortMode(Port4,PULLDOWN);								//	Enable Pull up
	
  SPI_MasterInit(CLK_1_66);		                  //Enable SPI at 1.66MHz
	
//------------- Perpheral Config -----------------
	ConfigLcd(Rs,E,Db4,Db5,Db6,Db7);							//	LCD Config

//*********** Timer(s) ******* TIMER3 in IMU values @ 400Hz *****  TIMER0 & TIMER1 in encoder values *** TIMER2 1kHz general Timeer ******
	ConfigTimer(TIMER3,256);
	MRConfig(_T3MR0,_INT_RESET,293);
	On(_T3MR0,get_rate);
	Enable(TIMER3);
	
	ConfigTimer(TIMER2,256);
	MRConfig(_T2MR0,_INT_RESET,117);
	On(_T2MR0,timer);
	Enable(TIMER2);
	StartTimer(TIMER2);
	
	ConfigEncoder(ENC1,TIMER1,CAP0,BOTH);	
	ClearEncoder(ENC1);	 
	StartEncoder(ENC1);	
	
//************************ UART config ***** COM0/1 for driving motors ***** COM2 for BLUETOOTH *********************************
  ConfigUART(COM1,19200,_TX,WORD_LENGTH_8,STOP_BIT_1,PARITY_DISABLE);
	ConfigExtPrintbin(COM1,2);
	
	ExtPrintbin(COM1,1,64);
	ExtPrintbin(COM1,2,192);
	waitms(1);
	ExtPrintbin(COM1,1,64);
	ExtPrintbin(COM1,2,192);
	
	ConfigUART(COM0,19200,_TX,WORD_LENGTH_8,STOP_BIT_1,PARITY_DISABLE);
	ConfigExtPrintbin(COM0,2);
	
	ExtPrintbin(COM0,1,64);
	ExtPrintbin(COM0,2,192);
	waitms(1);
	ExtPrintbin(COM0,1,64);
	ExtPrintbin(COM0,2,192);
	
	ConfigUART(COM2,57600,_RX,WORD_LENGTH_8,STOP_BIT_1,PARITY_DISABLE);
	ConfigExtPrintbin(COM2,2);
	
//===============================================================================================================================
	waitms(5);
	StartTimer(TIMER3);
	ang=0;                       //Initial angle set to be zero
  
	left_dia=50.894;                           //encoder which gives x dis without rotating the bot whatsoeve
  right_dia=left_dia;//50.507;               //encoder which gives y dis without rotating the bot whatsoever
	
	dis_per_count_left=(PI*left_dia)/2048;     //distance calculated per tick of encoder
	dis_per_count_right=(PI*right_dia)/2048;   //distance calculated per tick of encoder
	
	acc=0.001;                //Initial value of accelaration
	lim11=80;               //these limits limit the speed of motors so that it doesn't go beyond a range
	lim12=48;               //these limits limit the speed of motors so that it doesn't go beyond a range
  lim21=208;              //these limits limit the speed of motors so that it doesn't go beyond a range
	lim22=176;              //these limits limit the speed of motors so that it doesn't go beyond a range
		
	lim11_lim=80+30+10;
  lim12_lim=58-30-10;
	lim21_lim=208+30+10;
	lim22_lim=176-30-10;
	
	base_value=10;
	
	kp_line_sense=7;                 //Kp for line follower
	kd_line_sense=2.5;               //Kd for line follower
	ki_line_sense=0; 							   //Ki for line follower
	icontrol_line_sense=35;          //Maximum value for control of Line follow PID - in terms of maximum angle of motion allowed
  
	kp=2;												     //Kp for Omni
	kd=0.6; 	  										 //Kd for Omni
	ki=0;			  										 //Ki for omni
	icontrol=10;                     //Maximum value for control of Omni - in terms of maximum value sent to the motor
	
	turn_speed_limit=10;
	minor_adjustment_speed=11;
	//int attempt = 11;	
	cls();
	lcd((char*)"POINT");
	//for toto and milind's reference
	lowerline();
	lcd((char*)"TEAM INDIA 2018");
	// lcd(attempt);
	waitms(5);
	ang=0;

	//Enable(ADC);
	ConfigADC(Adc5);								//config ADC for sharp for wall follow
	
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//==========================================	ENCODER CHECK================================================///////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
y_dis = 0;
prvvaluer = 0;
T1TC = 0;	

while(0)
		{   
			pos();
		  cls();
      lcd(ang);			
		  lowerline();
		}
////////................................................................................................................
////////.........................................................................................................................


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//==========================================	PROXY  CHECK================================================///////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

while(0)
		{   
			lcd(y_dis);			
		  lowerline();
		}
////////................................................................................................................
////////.........................................................................................................................




		
		
  //encoder ticks check
	//
	/*
	next=0;
	while(!next)
	{
		if(count_cycle>7)
		{
			count_cycle=0;
		  Ps2_val_update();
	  }
		
		if(ps_l1)			
		{
			next=1;
			cls();
		}
		
		control_pid_omni((-4-ang)*Degree_to_Rad,35);		
	}
	
	next=0;
	double ani=0;
	while(!next)
	{
		if(count_cycle>7)
		{
			count_cycle=0;
		  Ps2_val_update();
	  }
		
		ani+=0.04;
		
		if(ani>90)
			lcd("a");
				
		if(ps_r1)
			next=1;
		control_pid_omni((-4-ang)*Degree_to_Rad,35);		
	}
	*/
	//
	/*while(1)
	{
		//temp_val=((((double)4096-((unsigned int)((AD0DR5&0x80000000)?((AD0DR5&0x0000FFF0)>>4):temp_val)))/4096)*3000);
		
    //=============================== Smoothen it out :- Use a moving average filter with order of 10 ==============================
		//value_array[9]=temp_val;		
		//for(int i=0;i<9;i++)
		//{
	//		temp_val+=value_array[i];
//			value_array[i]=value_array[i+1];			
		//}
		//temp_val=temp_val/9;
		cls();
		lcd(ang);//(double)aniruddha/count);  //1230 value
				
		//control_pid_omni(40*PI,20);
	}*/
	//PORT CHECK
	/*while(1)
	{
	  reset(P2_1);  reset(P2_2);	reset(P2_3);	reset(P2_4);	reset(P2_5);	reset(P2_6);
		//1-mr-p2_3
		//2-ms-p2_4
		//3-lr-p2_6
		//4-ls-p2_1
		//5-rr-p2_2
		//6-rs-p2_5
		wait(2);
		
		set(P2_3);
		cls();
		lcd("1");
		wait(2);
		
		set(P2_4);
		cls();
		lcd("2");
		wait(2);
		
		set(P2_6);
		cls();
		lcd("3");
		wait(2);
		
		set(P2_1);
		cls();
		lcd("4");
		wait(2);
		
		set(P2_2);
		cls();
		lcd("5");
		wait(2);
		
		set(P2_5);
		cls();
		lcd("6");
		wait(2);
		//3,4,5,1,2,6
		//reset(Port0_5);		reset(Port0_6);		reset(Port2_1);		reset(Port2_2);		reset(Port4_28);		reset(Port4_29);
	}*/
	//PIN CHECK - LINE SENSOR
	/*while(1)
	{
		
		cls();
		lcd(Pin0_19); //5
		lcd(Pin0_20); //n
		lcd(Pin0_21); //3
		lcd(Pin0_22); //6
		lcd(Pin2_4);  //1
		lcd(Pin2_7);  //4
		
		control_pid_omni(0,10);
		
		
		while(1)
		{
		  cls();
			lcd(sensor_val_update_v());
		}
		//lcd(Pin0_23); 5n3614
	}*/
	
	double heading_1,temp_diff=0;
	
	while(0)
	{
		cls();
		lcd(((((double)4096-((unsigned int)((AD0DR5&0x80000000)?((AD0DR5&0x0000FFF0)>>4):temp_val)))/4096)*3000));//ang);
		lowerline();
	///////////////  =================			

			temp_val = ((((double)4096-((unsigned int)((AD0DR5&0x80000000)?((AD0DR5&0x0000FFF0)>>4):temp_val)))/4096)*3000);
      temp_diff = (temp_val-2300);	  
		     
    if(temp_val>2260 && temp_val<2340) 				
           {
			     		heading_1= (-170)*Degree_to_Rad; 
  		     }	
					 
		 else if(temp_val>2200 && temp_val<2280) 	
       {
			  temp_val = (temp_val-2300);                          /////// 850  assumed
			 heading_1 = ((-1)*(170*Degree_to_Rad) - (temp_val*0.03)*Degree_to_Rad); 	 
   		 }
			 
		 else if(temp_val>2320 && temp_val<2500)
		   {
		 temp_diff = (temp_val-2410);	  
		 heading_1 = ((-1)*(170*Degree_to_Rad) - (temp_val*0.003)*Degree_to_Rad); 	 
		   }		
////////// .........................		
	
			 while(0)
			 {
			 cls();
			 lcd(ang);	 
		   lowerline();
			 }
			 		 
		
		/*
		if(temp_val>2260 && temp_val<2340) 				
       {
				heading_1= (-17)*Degree_to_Rad; 
			 //temp_val = (temp_val-2300);                          /////// 850  assumed
		 	 //heading_1 = ((-1)*(17*Degree_to_Rad) + (temp_val*0.5)*Degree_to_Rad); 
   		 }				
		else if(temp_val>2200 && temp_val<2280) 	 
		{
  		temp_diff = (temp_val-2240);	  
			heading_1 = ((-1)*(17*Degree_to_Rad) + (temp_diff*0.04)*Degree_to_Rad)   ; 
		}	 
		 
			 else if(temp_val>2320 && temp_val<2500) 
       {
			 temp_diff = (temp_val-2410);	  
			 heading_1 = ((-1)*(17*Degree_to_Rad) - (temp_val*0.002)*Degree_to_Rad); 	 
   		 }
	   */
		lcd(heading_1*57.32);
    lcd("  "); 
	  lcd(temp_diff);  
		
	}
	//
		
	set(P2_1);
	RACK_NORMAL;
	set(P2_3);
	set(P2_4);
	set(P2_5);
	set(P2_6);
			
	//while(count<400);
	int sharp_value=0;
int rack_proxy =0,rack_proxy_final = 0;
int temp = 0;
bool fuck_flag =0,pmv_proxy_flag = 0,flag_boolean = 0;

//angle check
while(0)
{
	lcd(ang);
	cls();
}


//proxy- count check
while(0)
{
	cls();
	if(Pin0_23 ==0)
	{
		fuck_flag = 1;
	}
	if(Pin0_23 == 1 && fuck_flag)
	{
		fuck_flag =0;
	rack_proxy++;
	}		
	lcd(rack_proxy);
}

//set-reset check--------------------------------------------------
y_dis = 0;
prvvaluer = 0;
T1TC = 0;	
//almost every check i.e. board check	
while(0)
{
	///////////////////
	lcd("Reset P2_2");
	reset(P2_2);
	waitms(2000);
	cls();
	
	lcd("set P2_2");
	set(P2_2);
	waitms(2000);
	cls();
	//////////////
	lcd("Reset P2_3");
	reset(P2_3);
	waitms(2000);
	cls();
	
	lcd("set P2_3");
	set(P2_3);
	waitms(2000);
	cls();
	////////////	
	
	lcd("Reset P2_4");
	reset(P2_4);
	waitms(2000);
	cls();
	
	lcd("Set P2_4");
	set(P2_4);
	waitms(2000);
  cls();
}	

	


	while(0)
	{
		cls();
    lcd(ang);	
	}
	
y_dis = 0;
prvvaluer = 0;
T1TC = 0;
	
	
	while(0)
	{	
	   pos();
		 cls();
     lcd(y_dis);		
		lowerline();
	}
	
	

	// sharp loop
///for sharp values	
	while(0)
	{
		//	pos();
		//reset(P2_4);
		cls();
		//lcd((double)aniruddha/count);
		lcd((unsigned int)((((double)4096-((unsigned int)((AD0DR5&0x80000000)?((AD0DR5&0x0000FFF0)>>4):temp_val)))/4096)*3000));
		//lcd(y_dis);
	  //	lowerline();
	  //	lcd(Pin1_20);
    //		pos();
		//lowerline();
		//lcd(aniruddha/count);
	}
	
	Ps2_val_update();
	waitms(7);
	Ps2_val_update();
	waitms(7);	
  
  turned=1;

	//towards_rack_normal();
  //towards_manual_normal();
			
	//rack ko upar karne ke liye pehli baar
	BUTTON_ONE=1;
	
/*	
	next=0;	
	while(!next)
	{
		//============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
		if(count_cycle>7)
		{
			count_cycle=0;
		  Ps2_val_update();
	  }
		
		if(ps_l1)
			next=1;
	}
	next=0;
	y_dis=-6000;
	manual_override=0;
	acc=0.007;
	while(!next && !manual_override)
	{
		//============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
		if(count_cycle>7)
		{
			count_cycle=0;
		  Ps2_val_update();
	  }
	
 	//============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
		if((temp_four!=400) && (temp_four!=360))
		{
			hold_angle=0;
			manual_override=1;
		}
		
		lim11+=acc;
		if(lim11>lim11_lim)
			 lim11=lim11_lim;

		lim12-=acc;
		if(lim12<lim12_lim)
		   lim12=lim12_lim;
    
		//limits for channel 2 -> with stop value 192   208,176
	  lim21+=acc;
		if(lim21>lim21_lim)
			lim21=lim21_lim;

		lim22-=acc;
		if(lim22<lim22_lim)
			lim22=lim22_lim;
		
		pos();
		if(y_dis>-4500)
		{
			acc=0.013;
			cls();
			lcd(lim11);
		}
		
		if(y_dis>500)
			next=1;
		
		control_pid_omni(-170*Degree_to_Rad,60);
	}
	
	next=0;
	while(!next && !manual_override)
	{
		//============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
		if(count_cycle>7)
		{
			count_cycle=0;
		  Ps2_val_update();
	  }
	
 	//============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
		if((temp_four!=400) && (temp_four!=360))
		{
			hold_angle=0;
			manual_override=1;
		}
		
		static double y_dis_temp=0;//y_dis;
		
		pos();
		
		if(y_dis>2900)
		{
			next=1;
			//cls();
			//lcd(y_dis-y_dis_temp);
		}
						
		if((y_dis - y_dis_temp)< 0.01)
			next=1;
		
		y_dis_temp=y_dis;
		
		control_pid_omni(-10*Degree_to_Rad,20);
	}
		
	while(!manual_override)
	{
		pos();
		if(y_dis>3010)
			control_pid_omni(-10*Degree_to_Rad,8);
		else if(y_dis<2990)
			control_pid_omni(-170*Degree_to_Rad,6);
		else
			control_pid_omni(40,12);
	}
		
	/*if(!manual_override)
	{
		reset(P2_4);
	  actuate();
	}*/
  
	
	next=0;
	ps_l1=0;
	
	acc=0.0008;
	/*
	bool pmvv =0;
	
	while(1)
	{
		
		if(pmvv=0 && ps_l1)
		{
			without_turn();
	    pmvv =1;
		}
	}
	*/
	
	int select_flag=0;
	while(1)
	{

		show_flag=1;
		Ps2_drive(-PI/2);          //ang*Degree_to_Rad + (PI/2));
	 if(ps_l1 && (turned == 1))
		{
			 manual_override=0;
			 //towards_manual_normal();
 			//golden_load_to_auto();
			   //without_turn();
			 manual_start_to_load();
			//reset(P2_4);
		
			  //golden_second_load();
			//golden_restart();
			
			if(!manual_override) 
			{
				count_cycle1=600;
        BUTTON_ONE=1;				
			//	actuate();
			}
		}
		
		
		if(ps_r1)
		{
		golden_restart();
		}	

		if(ps_square)
		{
	   golden_second_load();
		}	

    if(ps_r2)
		{
		tz2_restart();
		reset(P2_4);	
		}

		if(ps_triangle)
		{
     RACK_TURNED;
		 reset(P2_3);	
     reset(P2_4);	
		}
		
			
		if(ps_circle)
		{
		golden_second_load_2();
   	}
	  
		if(ps_select)
		{
		 set(P2_3);
		 RACK_NORMAL;
		}
		actuate();
	}
}
