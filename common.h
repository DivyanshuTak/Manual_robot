#ifndef _COMMON_H
#define _COMMON_H

#include<LPC17xx.h>
#include<math.h>
#include"lpc/GPIO.h"
#include"lpc/lcd.h"
#include"lpc/NVIC.h"
#include"lpc/PLL.h"
#include"lpc/SPI.h"
#include"lpc/TIMER.h"
#include"lpc/UART.h"
#include"lpc/ADC.h"
#include"lpc/QEI.h"

#define PI 3.1415926535
#define Degree_to_Rad 0.0174532925
#define DIA	56.25
#define CNTPERREV 2048.0

#define DED_RECON 0
#define DED_RECON_SEMI_AUTO 0

//-----------------LCD Aliases-------------------
#define Rs 			Port1_19							// Rsest Pin
#define E 			Port1_21							// Enable Pin 
#define Db4			Port1_28							// Databit 4
#define Db5			Port1_25							// Databit 5
#define Db6			Port1_27							// Databit 6
#define Db7			Port1_29							// Databit 7
//
//-------------10 Pin FRC aliases-----------------
#define R1_1	  Pin4_28     // MAT2.0  TX3
#define R1_2 	  Pin4_29			// 
#define R1_3 	  Pin2_2			// PWM1_2
#define R1_4 	  Pin0_5			// MAT2.0
#define R1_5    Pin0_6			// CAP2.1
#define R1_6    Pin2_1			// PWM1_3

#define R2_1   	Port0_9      // MAT2.3
#define R2_2   	Port0_7   		// MAT2.1
#define R2_3   	Port2_5      // PWM1_6
#define R2_4   	Port2_4			// PWM1_5
#define R2_5   	Port2_3			// PWM1_4
#define R2_6   	Port0_8			// MAT2.2

#define P1_1	Port0_6
#define P1_2	Port0_5
#define P1_3	Port4_28 
#define P1_4	Port4_29
#define P1_5	Port2_2
#define P1_6	Port2_1

#define P2_1   	Port2_6			// PWM1_5   //
#define P2_2   	Port2_3			// PWM1_4   //
#define P2_3   	Port0_9     // MAT2.3   //
#define P2_4   	Port0_7     // MAT2.1   //
#define P2_5   	Port0_8			// MAT2.2   //
#define P2_6   	Port2_5			// PWM1_4   //

#define P3_1    Port0_18 
#define P3_2    Port0_17
#define P3_3    Port0_15
#define P3_4    Port0_16
#define P3_5    Port2_9
#define P3_6    Port2_8

#define P4_1    Port0_22 
#define P4_2    Port0_21
#define P4_3    Port0_20
#define P4_4    Port0_19
#define P4_5    Port2_7            //2.7 2.4 0.19 0.22 0.20 0.21
#define P4_6    Port2_6

#define R3_1		Port0_15	//SCK/TX1
#define R3_2    Port0_16	//RX1
#define R3_3    Port2_8
#define R3_4    Port0_18 
#define R3_5    Port0_17
#define R3_6    Port2_9

#define R4_1    Port0_20
#define R4_2    Port0_19
#define R4_3    Port2_4
#define R4_4    Port0_22
#define R4_5    Port0_21
#define R4_6    Port2_7
//

//------Debug and direction aliases------
#define I2Cinit 0
#define DEBUG 1

#define FORWARD 1
#define BACKWARD 2
#define RIGHT 3
#define LEFT 4

/*#define START_POINT 0
#define ZONE_ONE_START 1
#define ZONE_ONE_SHOOT 2
#define ZONE_TWO_START 3
#define ZONE_TWO_SHOOT 4
#define ZONE_THREE_SHOOT 5

#define START_PROXY 0
#define DIRECTION_PROXY 1
#define ZONE_ONE_PROXY 2
#define ZONE_TWO_PROXY 3
#define ZONE_THREE_PROXY 4

#define Start_proxy Pin0_25
#define Zone_two_proxy Pin0_24
#define Zone_three_proxy Pin0_26
*/

#define BUTTON_ONE ps_square
#define BUTTON_TWO ps_cross
#define BUTTON_THREE ps_circle

#define KHAT_KHAT_DELAY 2500                      //ABHI TAK CHALA THA VAISA = 1 
#define STOP_POINT 8200               ////8460->9000
#define FORWARD_PROXY Pin0_26
#define REVERSE_PROXY Pin1_30
#define Golden_offset 500
#define RACK_NORMAL set(P2_2)
#define RACK_TURNED reset(P2_2)
#define TOTAL_DISTANCE -5200
#define BREAK_DISTANCE -1800
#define BREAK_DISTANCE_GOLDEN -2400
#define GOLDEN_REVERSE_BREAK_POINT 5000
#define START_BRAKE_LOAD_GOLDEN -3300                /////////////////    
#define STOP_BRAKE_LOAD_GOLDEN -3600                 ////////////////   3600  
                  
/////

/*
golden laser pin1_20
tz1 laser pin1_23
*/
//

//-------------------------------------VARIABLES USED THROUGHT THE CODE------------------------------------------
//PS2 variables
extern bool ip[4][8],ps_up,ps_right,ps_down,ps_left,ps_square,ps_triangle,ps_circle,ps_cross,ps_select,ps_start,
	    ps_r1,ps_r2,ps_l1,ps_l2;
extern int ps2_val[4], one,two,three,four,turn_adc,turn_yes;
extern double temp_four;
extern int safety,top_gate;
extern bool universal;
extern int tz2_actuation;
extern int proxy_safety,button_flag;
//

//PID variables
extern double b_heading,hold_angle,value_1,value_2,value_3, proportional, integral,derivative,integrald,rate,
	     control,old_control,icontrol;
extern float kp,ki,kd,manual_kp,manual_ki,manual_kd,auto_kp,auto_ki,auto_kd;
extern char dummyl,dummyr,txt1,txt2,txt3;
//

// LIMITING factors
extern double lim11_lim,lim12_lim,lim21_lim,lim22_lim;
extern double lim11,lim12;             //limits for channel 1 -> with stop value 64    80,48
extern double lim21,lim22;           //limits for channel 2 -> with stop value 192   208,176
extern int control_speed,flag,safety,minor_adjustment_speed;
extern double acc;
extern int base_value;
//

//Flags/ misc.
extern int speed ,count_cycle,mode,stop_count,count_cycle1,turn_speed_limit,turn_speed,count_cycle2 ,
    res[4],ans,time,turn_final,field,pos_flag,sop_flag;
extern double ang,x_hold ,y_hold,angle_change,difference,aniruddha;
extern int plane_select,err_count1,err_count2,centre_point, count,time_limit;
//

//DED recon - encoder
extern int b1,a1,previous,tickl,tickr,prvvaluel,prvvaluer,prevposition,templ ,tempr ;
extern double th;
extern double left_ddis,dis_per_count_left,right_ddis,dis_per_count_right,x_dis,y_dis,left_dia,right_dia;
//

//line sensor - line sensor PID
extern double proportional_line_sense, kp_line_sense, integral_line_sense, integrald_line_sense, ki_line_sense, rate_line_sense, prev_err, derivative_line_sense, kd_line_sense, 
	     control_line_sense, icontrol_line_sense;
extern int sum_sensor_v,sum_sensor_h;
extern bool sensor_value[8];
//

//Theme specific
extern int zone,manual_override;
extern bool next,dis_flag,left_rack,right_rack,centre_rack,turned,zone1_exit;
extern int junction_count,direction,shift_direction,show_flag;
extern double error_v,error_h, velocity,velocity_final,minor_speed, value_array[10], temp_val;
extern int rule_flag;

//

//-------------FUNCTION DECLARATION FOR USE IN WHOLE PROJECT---------
float sensor_val_update_v();
float sensor_val_update_h();
void find_line(char Direction_fin1, char Direction_fin2, int max_speed);
//

//
int proxy_check();
//

//
void towards_auto_golden(void);
void set_speed_limit(void);		
void get_rate(void);
void get_angle(void);
void pos(void);
void angle_90(void);
void angle_0(void);
void wait_pid(int wait_period);
void drive(double);
void towards_rack_normal(void);
void towards_rack_golden(void);
void towards_manual_normal(void);
void towards_rack_golden_auto(void);
void actuate(void);
void Ps2_drive(double);
void timer(void);
void control_pid_omni(double heading,double speed_limit);
void Ps2_val_update(void);
void control_pid_line_follow(char Direction, int max_speed, double error_line);
void golden_load_to_auto(void);
void without_turn(void);
void without_turn_load_golden_aayush(void);
void without_turn_load_golden(void);
void without_turn_golden(void);
void manual_start_to_load(void);
void manual_tz1_to_golden(void);
void golden_restart(void);
void manual_constant_speed(void);
void golden_second_load(void);
void tz2_restart(void);
void golden_second_load_2(void);

//

#endif


/*
void without_turn()
{
	
//-----------------------------------------	
	lim11=80+2;//+5;
	lim12=48-2;//-5;
  lim21=208+2;//+5;
	lim22=176-2;//-5;
		
	lim11_lim=80+40;   //-5; //126; //80
  lim12_lim=48-40;   //+5; //2;   //48
	lim21_lim=208+40;  //-5; //253; //208
	lim22_lim=176-40;
	
//-----------------------------------------	
	
//======================================================= start to load (AYUSH)      =======================================================================================	
	//================================================= START TO LOAD (AYUSH)=======================================================
int rackproxy,temp=2000,proxy_count=0;
double factor,base_speed=30; 
bool load_p=0,load_enc=0,decc=0,fuck_flag=0;
	acc=0.007;
  y_dis = 0; 
	T1TC=0;
	prvvaluer=0;
  next=0;
  manual_override=0;
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
//=============================================================================================================================================	
		pos();                     
		
		if (y_dis<2500)
		{
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
		 
   	control_pid_omni(-175*Degree_to_Rad,60); 
  ///		base_speed = 50;
		}
		
		else 
		{
			base_speed = 50*exp(-(y_dis/1500));
			if (base_speed<10)
				base_speed=8;
			
		  if ((proxy_count>2)&&(decc==0))
		   {
			   decc=1;
			   proxy_count=0;
			  // base_speed=4;
				 fuck_flag=1;
		   }
			if (fuck_flag==0)
			{		
         control_pid_omni(40,5);				
			 //control_pid_omni(-170*Degree_to_Rad,base_speed);  
			}
			else 
			{
				 //control_pid_omni(-175*Degree_to_Rad,4);			
				control_pid_omni(0,8);
			}
		}
			
		//if (base_speed<20)
		//	base_speed=4;
		//base_speed = base_speed*factor;
    
	//	control_pid_omni(-170*Degree_to_Rad,base_speed);
    
		if (Pin0_23==0)
			proxy_count++;
		
	
		
     if ((proxy_count>2)&&(decc==1)) 		//(proxy_count>2)//&&(y_dis>2000))                      //////////////// if rack proxy detected skip  the loop 
     {
		  next=1;
      temp=2000;
			while(temp--)                       ///////////////   BOT DEACCELERATES    
		   {
		   control_pid_omni(0*Degree_to_Rad,5);
			 } 
		  } 	
		

 }
 
 
  int base_value=0;
 // reset(P2_4);
	while(0)
	{
		pos();
		cls();
		lcd(y_dis);
		lowerline();
		lcd(Pin0_23);
		control_pid_omni(40,5);
	}
	
	lim11=80+40;   //-5; //126; //80
  lim12=48-40;   //+5; //2;   //48
	lim21=208+40;  //-5; //253; //208
	lim22=176-40;
	
  next=0;
	
	y_dis = 0; //new one
	T1TC=0;
	prvvaluer=0;
	
  hold_angle = 0;
	acc=0.008;
	int count_p=0,current_dis=0;
	bool fuck_flag2=0;
	//================================================= GO TOWARDS JUNCTION AFTER PICKING UP THE SHUTTLES =======================================================
	while(!next && !manual_override)
	{
//============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
		if(count_cycle>7)
		 {
			 count_cycle=0;
		   Ps2_val_update();
	   }
	  pos();
	
		
//============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
		if((temp_four!=400) && (temp_four!=360))
		 {
			 hold_angle=0;
			 manual_override=1;
		 }
//============================================================================================================================================		
	   if ((Pin1_23==1)||(Pin1_20==1))
		 {
			 count_p++;
		 }
		 
		 if (y_dis>-4500)
		 {
			 fuck_flag=1;
		 }
		 
		 if (fuck_flag==0)
		 {
			 control_pid_omni(-10,50);
       current_dis = y_dis; 			 
		 }
		 else
		 {
        base_speed = 50*exp(-(y_dis/4500)); 
			  control_pid_omni(-10,base_speed);
			  if (base_speed<6)
					base_speed=6;
		 }
		 
		 if (((-1)*y_dis)>STOP_POINT)
		   next=1;
		 // base_value = 35*exp((-3)*(((4139-distance)/4139))); 
			 
		
    //===================== Get SHARP value and then smoothen it using a moving average filter. Set boundaries on the angle so obtained =======================
		/*temp_val=((((double)4096-((unsigned int)((AD0DR5&0x80000000)?((AD0DR5&0x0000FFF0)>>4):temp_val)))/4096)*3000);
		
    value_array[9]=temp_val;
		
		for(int i=0;i<9;i++)
		{
			temp_val+=value_array[i];
			value_array[i]=value_array[i+1];			
		}
		
		temp_val=temp_val/10;
		
		th = 0 - (temp_val-850)/3;		 //1915
	 	

	
	}
	
	while(!manual_override)
	{
		control_pid_omni(40,5);
	}
	
	
	
	
}


*/