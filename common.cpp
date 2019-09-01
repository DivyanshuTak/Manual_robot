#include"common.h"
//PS2 variables
bool ip[4][8],ps_up,ps_right,ps_down,ps_left,ps_square,ps_triangle,ps_circle,ps_cross,ps_select,ps_start,
	    ps_r1,ps_r2,ps_l1,ps_l2;
int ps2_val[4], one=0,two=0,three=0,four=0,turn_adc=0,turn_yes=0,top_gate=0,bottom_gate=0,top_gate_2=0;
double temp_four=0;

bool universal;
//
//PID variables
double b_heading,hold_angle=0,value_1,value_2,value_3, proportional, integral,derivative,integrald,rate,
	     control,old_control,icontrol;
float kp=0,ki=0,kd=0,manual_kp=0,manual_ki=0,manual_kd=0,auto_kp=0,auto_ki=0,auto_kd=0;
char dummyl,dummyr,txt1,txt2,txt3;
//
// LIMITING factors
double lim11_lim,lim12_lim,lim21_lim,lim22_lim;
double lim11=80,lim12=48;             //limits for channel 1 -> with stop value 64    80,48
double lim21=208,lim22=176;           //limits for channel 2 -> with stop value 192   208,176
int control_speed=20,flag=0,safety,minor_adjustment_speed;
double acc=0;
int base_value;
//
//Flags/ misc.
int speed=0,count_cycle=0,mode=0,stop_count=0,count_cycle1=0,turn_speed_limit,turn_speed,count_cycle2=0,
    res[4]={0},ans=0,time=0,turn_final,field,pos_flag=1,sop_flag=0;
double ang=0,x_hold=199,y_hold=6717,angle_change=0,difference;
int plane_select=1, err_count1=0,err_count2=0,centre_point=0,time_limit;
int count=0;
	double aniruddha=0;
//
//DED recon - encoder
int b1,a1,previous,tickl,tickr,prvvaluel,prvvaluer,prevposition,templ=0,tempr=0;
double th=0;
double left_ddis,dis_per_count_left,right_ddis,dis_per_count_right,x_dis,y_dis,left_dia,right_dia;
//
// line sensor + line sensor PID
double proportional_line_sense, kp_line_sense, integral_line_sense, integrald_line_sense, ki_line_sense, rate_line_sense, prev_err, derivative_line_sense, kd_line_sense, 
	     control_line_sense, icontrol_line_sense;
int sum_sensor_v=0,sum_sensor_h=0;
bool sensor_value[8];
//
// THEME specific variables
  int zone;
  bool next=0,dis_flag,left_rack,right_rack,centre_rack,turned=0,zone1_exit=0;
  int junction_count=0,manual_override,direction,shift_direction=LEFT,show_flag=0;
	double error_v,error_h, velocity, velocity_final,minor_speed, value_array[10], temp_val;
  bool flag_TZ3; 
  double temp_tick_enc=0,p_temp_tick_enc=0,diff_tick=0,diff_max=0;
  double target_speed = 0;
  int transfer_flag=0; 
 int tz2_actuation=0;
int proxy_safety=0,button_flag=0,rule_flag=0;
//

void towards_auto_golden()
{	
	set(P2_1);
	set(P2_2);
	set(P2_3);
	set(P2_4);
	set(P2_5);
	set(P2_6);
	
	next=0;
	manual_override=0;
  hold_angle = 0;                                //change 'hold angle' as the wall follower structure tends to change the orientation of robot
	
	lim11=80;               //these limits limit the speed of motors so that it doesn't beyond a range
	lim12=48;               //these limits limit the speed of motors so that it doesn't beyond a range
  lim21=208;              //these limits limit the speed of motors so that it doesn't beyond a range
	lim22=176;              //these limits limit the speed of motors so that it doesn't beyond a range
	
	lim11_lim=80+7+10;
  lim12_lim=58-7-10;
	lim21_lim=208+7+10;
	lim22_lim=176-7-10;
	acc=0.005;
	icontrol=15;
	int flag_detect=0,counter_detect=0;
	
	/*while(!next && !manual_override)
	{
		//================================================Update Ps2 Values every 7ms================================================
		if(count_cycle>7)
		{
			count_cycle=0;
		  Ps2_val_update();
	  }
		
		//================================================Check for any sign of manual override=======================================
		if((temp_four!=400 && temp_four!=360))
		{
			hold_angle=0;
			manual_override=1;
		}		
				
		th = 184;
	  
		//========================================== Limit the angle obtained to attain smooth motion ==================================
		if(th>190)                        //  this was 200
			th=190;
		else if(th<182)
			th=182;                         
		
		//========================== Speed up according to accelaration and after full speed is reached, decellerate ====================		
		switch(flag_detect)
		{
			case 0:																										//case 0: move towards rack until proxy detects the rack structure
							control_pid_omni(th*Degree_to_Rad,15+3);
							
							if(!Pin0_23)
								counter_detect++;
							
							if(counter_detect>50)
							{
								flag_detect=1;
								counter_detect=0;
							}
							break;
											
			case 1:																										//case 1: move towards rack while proxy is detecting the rack structure; exit when the structure is cleared
							control_pid_omni(th*Degree_to_Rad,11+1);
							
							if(Pin0_23)
								counter_detect++;
							
							if(counter_detect>50)
							{
								flag_detect=2;
								counter_detect=0;
							}
							break;
							
			case 2:																									  //case 2: move slowly towards rack till the next point of rack is detected
							control_pid_omni(th*Degree_to_Rad,11+1);
							
							if(!Pin0_23)
								counter_detect++;
							
							if(counter_detect>5)
							{
								flag_detect=3;
								counter_detect=0;
								next=1;
							}
							break;			
	 }
	}
	*/
	//================================================= Stop the robot by giving a jerk in opposite direction ========================================================
	int i = 1000;
	/*while(i-- && !manual_override)
	{
		control_pid_omni(-10*Degree_to_Rad,15);
	  pos();
	}*/
	
	//==============================================================Towards Auto=================================================================================
	lim11=80+5;
	lim12=48-5;
  lim21=208+5;
	lim22=176-5;
		
	lim11_lim=80+40;   //-5; //126; //80
  lim12_lim=48-40;   //+5; //2;   //48
	lim21_lim=208+40;  //-5; //253; //208
	lim22_lim=176-40;
	
	next=0;
	
	y_dis=3100;
	T1TC=0;
	prvvaluer=0;
		
  hold_angle = 0;
	acc=0.003;
	icontrol=60;
	i=0;	
	count_cycle1=0;
	
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
		
		if(y_dis<2600)
			acc=0.009;
		
		if(count_cycle1>1500)    //1700
			next=1;
	      
		if(y_dis<1500)           //1000
		{
			next=1;
		}
		
		//============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
		if((temp_four!=400) && (temp_four!=360))
		{
			hold_angle=ang;
			manual_override=1;
		}
		
		//============================================= Accelerate the robot by increasing the speed limit ========================================================
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
		
    th = 0 - 2;
	 	
		if(th<-4)
			th=-4;
		else if(th>5)
			th=5;
		
		control_pid_omni(th*Degree_to_Rad,60);
	}
		
	icontrol=60;
	count_cycle1=0;
		
	lim11=80+35+5;               //these limits limit the speed of motors so that it doesn't beyond a range
	lim12=48-35-5;               //these limits limit the speed of motors so that it doesn't beyond a range
  lim21=208+35+5;              //these limits limit the speed of motors so that it doesn't beyond a range
	lim22=176-35-5;              //these limits limit the speed of motors so that it doesn't beyond a range
	
	//================================================ DISTANCE COVERED. NOW, TURN ==============================================================
	while(ang>-85 && !manual_override)
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
			hold_angle=ang;
			manual_override=1;
		}
		
		//================= Decrement the hold_angle to change robot orientation so that the shuttle loading mechanism faces the auto =============================
		hold_angle-=0.02;
		
		if(ang<hold_angle)
			hold_angle=ang;
		
		if(hold_angle<-90)
			hold_angle=-90;
		
		//====================================== Standard PID based driving and ange maintaining function for Omni Drive ==========================================
		control_pid_omni((-3-ang)*Degree_to_Rad,60);
	}
	
	if(!manual_override)
		turned=0;
	
	count_cycle1=0;
	
	while((count_cycle1<1000) && !manual_override)
	{
		//================================================Update Ps2 Values every 7ms================================================
		if(count_cycle>7)
		{
			count_cycle=0;
		  Ps2_val_update();
	  }
		
		//================================================Check for any sign of manual override=======================================
		if((temp_four!=400 && temp_four!=360))
		{			
			manual_override=1;
		}			
		control_pid_omni(90*Degree_to_Rad,9);		
	}
	
	if(!manual_override)
	{
		count_cycle1=600;
		hold_angle=-90;
		BUTTON_THREE=1;
		actuate();
	}
	
	//============================== To adjust the minor speed in the robot after the robot has turned beyond a certain degree ==================================
	if(ang<-80)
		turned=0;
	
	icontrol=	60;
	//==============================================================End of:- Towards Manual======================================================================	
	return;	
}
//
void actuate()
{
	int i = 0,actuate_flag=0;
	
	/*
	p2_1 - latch_left
	p2_2 - rack_right
	p2_3 - rack_middle
	p2_4 - latch_middle
	p2_5 - latch_right
	p2_6 - rack_left
	
	//1-mr-p2_3             :r rack.  s shuttle
	//2-ms-p2_4
	//3-lr-p2_6
	//4-ls-p2_1
	//5-rr-p2_2
	//6-rs-p2_5
	*/
	
	if(ps_l2)
	{
		shift_direction = RIGHT;
		manual_override=0;
	}
	else if(ps_r2)
	{
		shift_direction = LEFT;
		manual_override=0;
		
		show_flag = 4;
		set(P2_1);
		set(P2_2);
		set(P2_3);
	//	set(P2_4);
		set(P2_5);
		set(P2_6);
	}
	//
	//
	if(BUTTON_ONE)                                       /////////////&& count_cycle1>500)
	{
		
		if(show_flag!=0)                     //this removes ambiguity whether we have actuated other racks between two actuations of this set or not
		{
			left_rack=0;
		  right_rack=0;
			show_flag=0;
		}
		
		count_cycle1=0;
		if(!left_rack && !centre_rack)
		{
			reset(P2_5);
			set(P2_4);
			
		 /*
			set(P2_5);      //set left rack and its latch
		  set(P2_3);			//set left rack and its latch
		//	set(P2_4);
			
		  left_rack=0;
			
			reset(P2_2);         //rr
			reset(P2_6);         //lr
			
			set(P2_1); 						//ls
		 */
      /*
			reset(P2_5);
			left_rack = 1;
			centre_rack = 1;
		  */ 
			//golden_second_load();
			}
		else
		{
			
			
			/*
			reset(P2_1);
		  i=KHAT_KHAT_DELAY;
			while(i--)
				Ps2_drive(PI);
			set(P2_1);			
		  */
			//if()
			/*
      {set(P2_4);}
			  i=KHAT_KHAT_DELAY;
			while(i--)
				Ps2_drive(PI);
		   */
		 transfer_flag=1;
			
        			
		}
		
	}
	else if(BUTTON_THREE && count_cycle1>500)
	{
		if(show_flag!=1)                //this removes ambiguity whether we have actuated other racks between two actuations of this set or not
		{
			//right_rack=0;
		  right_rack=0;
			show_flag=1;
		}
		count_cycle1=0;
							
		if(!right_rack)
		{
			centre_rack=0;
		  left_rack=0;
			
			set(P2_1); set(P2_5); set(P2_3); set(P2_6);
			
		  reset(P2_2);
			set(P2_4);
			
			right_rack=1;
		}
		else
		{
			reset(P2_4);
			
			i=KHAT_KHAT_DELAY;
			while(i--)
				 Ps2_drive(PI);
			
			set(P2_4);			
		}
	}
	else if(BUTTON_TWO && count_cycle1>500)
	{
		count_cycle1=0;
					
		if(shift_direction==LEFT)
    {			
			if(show_flag!=2)					//this removes ambiguity whether we have actuated other racks between two actuations of this set or not
			{
				left_rack=0;				
				show_flag=2;
			}
			
			if(!left_rack)
			{
				set(P2_5); set(P2_2); set(P2_3); set(P2_4);
			  left_rack=0;
			  centre_rack=0;
				
				reset(P2_6);
				set(P2_1);
								
				left_rack=1;
			}
			else
			{
				reset(P2_1);
				i=KHAT_KHAT_DELAY;
				while(i--)
					 Ps2_drive(PI);
				
				set(P2_1);				
			}
		}
		else
		{			
			if(show_flag!=2)
			{
				left_rack=0;				
				show_flag=2;
			}
		
			if(!left_rack)
			{
				set(P2_2); set(P2_3); set(P2_4); set(P2_5);
			
			  right_rack=0;
			  centre_rack=0;
				
				reset(P2_6);
				set(P2_1);
								
				left_rack=1;
			}
			else
			{
				reset(P2_1);
				
				i=KHAT_KHAT_DELAY;
				while(i--)
					 Ps2_drive(PI);
				
				set(P2_1);				
			}
		}
	}
	return;
}
//
void towards_rack_golden()
{
	// get all racks down.. if not already
	set(P2_1);
	set(P2_2);
	set(P2_3);
	set(P2_4);
	set(P2_5);
	set(P2_6);
	
	right_rack=0;
	centre_rack=0;
	left_rack=0;
	
	show_flag=4;
	
	// ==================================== Prepare for next loop ====================================================
	next=0;
	manual_override=0;
  		
	lim11=80+5;               //these limits limit the speed of motors so that it doesn't beyond a range
	lim12=48-5;               //these limits limit the speed of motors so that it doesn't beyond a range
  lim21=208+5;              //these limits limit the speed of motors so that it doesn't beyond a range
	lim22=176-5;              //these limits limit the speed of motors so that it doesn't beyond a range
	
	lim11_lim=80+35;
  lim12_lim=48-35;
	lim21_lim=208+35;
	lim22_lim=176-35;
		
	icontrol=60;
		
	count_cycle1=0;
	
	y_dis = 0;
	T1TC=0;
	prvvaluer=0;
	int i =1500;
	while(i--)
	{
		pos();
		control_pid_omni(0,12);
	}
	while(y_dis<3500 && !manual_override)
	{
		pos();
		if(count_cycle>7)
		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		}
		control_pid_omni(-170*Degree_to_Rad,30);
	}
	/*	
	while(y_dis>-50 && !manual_override)     //200 previous value
	{
		//================================================Update Ps2 Values every 7ms=====================================================
		if(count_cycle>7)
		{
			count_cycle=0;
			Ps2_val_update();
		}
				
		//================================================Check for any sign of manual override===========================================
		if((temp_four!=400 && temp_four!=360))
		{
		 hold_angle = ang;	
			manual_override=1;
		}		
				
		pos();
		
		control_pid_omni(0*Degree_to_Rad,12);  //the reverse motion
	}
	
	while(y_dis>-300 && !manual_override)
	{
		//================================================Update Ps2 Values every 7ms=====================================================
		if(count_cycle>7)
		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		//================================================Check for any sign of manual override===========================================
		if((temp_four!=400 && temp_four!=360))
		{
			hold_angle = ang;		 	
			manual_override=1;
		}
		
		pos();
		control_pid_omni(-45*Degree_to_Rad,22);
	}
	*/
	icontrol = 60;
	acc=0.007;

/*  turning part	
	while(ang<-2 && !manual_override)
	{
		//================================================Update Ps2 Values every 7ms=====================================================
		if(count_cycle>7)
		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		//================================================Check for any sign of manual override===========================================
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
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
			
			
		hold_angle += 0.02;		
		
		if(hold_angle>0)
			hold_angle=0;
			
		if(ang>hold_angle)
			hold_angle=ang;
		
		control_pid_omni((177-ang)*Degree_to_Rad,30);
	}
	*/

  y_dis = 0;
	T1TC=0;
	prvvaluer=0;
	if(!manual_override)
		turned=1;
	
	//for proxy conditions
	int rack_proxy =0;
	bool fuck_flag = 0;
	
	lim11=80+24;               //these limits limit the speed of motors so that it doesn't beyond a range
	lim12=48-24;               //these limits limit the speed of motors so that it doesn't beyond a range
  lim21=208+24;              //these limits limit the speed of motors so that it doesn't beyond a range
	lim22=176-24;              //these limits limit the speed of motors so that it doesn't beyond a range
	
	count_cycle1 = 0;
	next = 0;
	while(y_dis<1800 && !manual_override )    //3 for golden
	{
		//================================================Update Ps2 Values every 7ms=====================================================
		if(count_cycle>7)
		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		//================================================Check for any sign of manual override===========================================
		if((temp_four!=400 && temp_four!=360))
		{
			hold_angle = ang;
		 	//hold_angle=0;
			manual_override=1;
		}
		
		//if(count_cycle1>1000)
			//next=1;
		
		pos();
		if (y_dis<200)
		  control_pid_omni(-170*Degree_to_Rad,27);   //30
		else
			 control_pid_omni(-170*Degree_to_Rad,27);  //30
	}
	
	if(manual_override)
	{
		cls();
		lcd((char*)"manual");
	}
		lim11=80+40;   //-5; //126; //80
  lim12=48-40;   //+5; //2;   //48
	lim21=208+40;  //-5; //253; //208
	lim22=176-40;
	
	
	//pmv's updates
	int proxy_count=0;
double base_speed; 
	acc=0.007;
  y_dis = 0; 
	T1TC=0;
	prvvaluer=0;
  next=0;
  manual_override=0;
	rack_proxy =0;
	fuck_flag = 0;
	//pmv's	
	//2200
 //2600
 //2950
	 
	while(y_dis<150 && !manual_override)
  {				
  	//============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
		if(count_cycle>7)
		{
			count_cycle=0; 
		  Ps2_val_update();
	  }
	
 	  //============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
		if((temp_four!=400) && (temp_four!=360))
		{hold_angle = ang;
			//hold_angle=0;
			manual_override=1;
		}
    //=============================================================================================================================================	
		pos();
		control_pid_omni(-170*Degree_to_Rad,23);   //30
	}
			
		while(y_dis <(2100+ Golden_offset) && !manual_override)
		{
	    pos();
			
		  //============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
			if(count_cycle>7)
			{
				count_cycle=0;
				Ps2_val_update();
			}
		
			//============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
			if((temp_four!=400) && (temp_four!=360))
			{hold_angle = ang;
				//hold_angle=0;
				manual_override=1;
			}
		
			base_speed = 40*exp(-(y_dis/1600));    //30->40    1500->1650->1500
			
			if (base_speed<17)    //23 to 22 just to solve the rack conditions //15->14
				base_speed=17;
			
			control_pid_omni(-165*Degree_to_Rad,base_speed);
		}
			
  	next=0;
			
	while(!next && !manual_override && rack_proxy<3)
		{
		
	if(Pin0_23 ==0)
	{
		fuck_flag = 1;
	}
	if(Pin0_23 == 1 && fuck_flag)
	{
		fuck_flag =0;
	rack_proxy++;
	}
			pos();
			//============================= Take input of Ps2 every 7 miliseconds. Optimal time obtained by trial and error method ========================================
			if(count_cycle>7)
			{
				count_cycle=0;
				Ps2_val_update();
			}

			//============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
			if((temp_four!=400) && (temp_four!=360))
			{
				hold_angle=ang;
				manual_override=1;
			}

			static double y_dis_temp=0;
			
			if((y_dis - y_dis_temp)<0.1)
			  next=1;
			  				
      if(y_dis>2700)
        next=1;
			
			y_dis_temp=y_dis;

			control_pid_omni(6*Degree_to_Rad,19);  //-10
		 }
			
		 next=0;
		 
			while(!next && !manual_override && rack_proxy<3)
			{
		
				if(Pin0_23 ==0)
						{
							fuck_flag = 1;
						}
				if(Pin0_23 == 1 && fuck_flag)
						{
							fuck_flag =0;
							rack_proxy++;
						}
				pos();
				
  		  control_pid_omni(-170*Degree_to_Rad,4);
		  	 
				 //============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
		     if(count_cycle>7)
		     {
			       count_cycle=0;
		         Ps2_val_update();
	       }
	
     	   //============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
		     if((temp_four!=400) && (temp_four!=360))
		     {
		       	hold_angle=ang;
	          manual_override=1;
		     }
			   //=====================================================================================================
      	 
				 
				 //backup to the backup of proxy_count
				 if(y_dis>3500)
           next=1;				 
			}
	
	/*   older conditions
	
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
		/*
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
		
	//	else 
	
	//start to load
	
			base_speed = 40*exp(-(y_dis/2000));    //30->40    1500->1650->1500
			
			if (base_speed<13)     //15->13
				base_speed=13;
			
		  if ((proxy_count>2)&&(decc==0))
		   {
				  y_dis = 0; 
         	T1TC=0;
	        prvvaluer=0;
				 while((y_dis<2000)&&(!manual_override))
				 {				//============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
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
          if ((y_dis>600)&&(Pin0_23==0))
					{
						next=1;
						break;
					}						
						
					control_pid_omni(175*Degree_to_Rad,6);
          pos();						
				 }
			   decc=1;
			   proxy_count=0;
			  // base_speed=4;
				 fuck_flag=1;
				 break;
		   }
			if (fuck_flag==0)
			{		
        // control_pid_omni(40,5);				
			 control_pid_omni(-170*Degree_to_Rad,base_speed);  
			}
			else 
			{
				 control_pid_omni(-175*Degree_to_Rad,5);			
				//control_pid_omni(0,8);
			}
		
			
		//if (base_speed<20)
		//	base_speed=4;
		//base_speed = base_speed*factor;
    
	//	control_pid_omni(-170*Degree_to_Rad,base_speed);
    
		if (Pin0_23==0)
			proxy_count++;
		
				
     if ((proxy_count>2)&&(y_dis>3200)&&(decc==1)) 		//(proxy_count>2)//&&(y_dis>2000))                      //////////////// if rack proxy detected skip  the loop 
     {
		  next=1;
      temp=2000;
			while(temp--)                       ///////////////   BOT DEACCELERATES    
		   {
		   control_pid_omni(0*Degree_to_Rad,5);     //5->8 expectations
			 } 
		  } 	
		
 }
/*
 while(0)
 {
	 control_pid_omni(40,5);
 }
*/
 golden_load_to_auto();
 
  /*
 //for half a second delay
 int temp_3=1000;
 count_cycle=0;
 while(count_cycle<1)//500)
	 control_pid_omni(40,5);
 //============================================load to auto=============================
 
 // reset(P2_4);

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
	
	
//===============================================================================================================================================	
	// get all racks down.. if not already
	set(P2_1);
	set(P2_2);
	set(P2_3);
  set(P2_5);
	set(P2_6);

	
	// towards_manual_normal();
	
	
	*/
	/*   pmv's conditions;
	*/
	/*while(1)
	{
		control_pid_omni(100,10);
	}
	
	//============================ We are at either TZ1 or TZ2 manual zone. Rotate and move towards racks ==============================+
	while(!next && !manual_override)
	{
		//================================================Update Ps2 Values every 7ms=====================================================
		if(count_cycle>7)
		{
			count_cycle=0;
		  Ps2_val_update();
	  }
		
		//================================================Check for any sign of manual override===========================================
		if((temp_four!=400 && temp_four!=360))
		{
			hold_angle=ang;
			manual_override=1;
		}
				
		//=========================================== Exit if ang falls below a certain value ============================================
		if(ang>-1)
			next=1;		
		
		if(count_cycle1>500)
		{
		//=================================== Increment current holding angle to achieve Translation+Rotation ============================
			if(count_cycle1>600)
			{
				if(ang>hold_angle)
				   hold_angle=ang;
			
				hold_angle+= 0.05; //0.015
				if(hold_angle>0)
					hold_angle=0;
		  }
		
		//========================================================= Accelerate according to value of accelaration ========================
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
		}

    control_pid_omni(PI-((ang)*Degree_to_Rad),60);		
	}
	
	next=0;
	flag_detect = 0;
	
	if(!manual_override)
		hold_angle=0;
	else
		hold_angle=ang;
	
	//================================================= Bot turned, now towards wall ===================================================
	while(!next && !manual_override)
	{
		//================================================Update Ps2 Values every 7ms=====================================================
		if(count_cycle>7)
		{
			count_cycle=0;
		  Ps2_val_update();
	  }
		
		//================================================Check for any sign of manual override===========================================
		if((temp_four!=400 && temp_four!=360))
		{
			hold_angle=0;
			manual_override=1;
		}
		
    //======================================= Get value from SHARP for wall following ================================================
		temp_val=((((double)4096-((unsigned int)((AD0DR5&0x80000000)?((AD0DR5&0x0000FFF0)>>4):temp_val)))/4096)*3000);
		
    //=============================== Smoothen it out :- Use a moving average filter with order of 10 ================================
		value_array[9]=temp_val;		
		for(int i=0;i<9;i++)
		{
			temp_val+=value_array[i];
			value_array[i]=value_array[i+1];			
		}
		temp_val=temp_val/9;
		
		//th = 180 + (temp_val-850)/2;
	  
		//========================================== Limit the angle obtained to attain smooth motion ====================================		
		th=205;
		
		if(temp_val<2000)
			counter_detect++;
				
		if(counter_detect>100)
			next=1;
				
		//======================================Accelerate the robot till its velocity reaches a certain point============================
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
		
		//========================== Speed up according to accelaration and after full speed is reached, decellerate =====================
		switch(flag_detect)
		{
			case 0:																											//move towards rack till proxy doesnt detect anything.
				if(lim11!=lim11_lim)
					control_pid_omni(th*Degree_to_Rad,25);
				else
					control_pid_omni(th*Degree_to_Rad,8);
				
				if(!Pin0_23)
					counter_detect++;
				
				if(counter_detect>25)
				{
					flag_detect=1;
					counter_detect=0;
				}
				break;
							
			case 1:																											//move slowly towards rack till proxy is detecting the rack.
				control_pid_omni(th*Degree_to_Rad,7);
				
				if(Pin0_23)
					counter_detect++;
				
				if(counter_detect>25)
				{
					flag_detect=2;
					counter_detect=0;
				}
				break;
		 case 2:																											//move towards rack till proxy doesnt detect anything. After this, exit the loop.
				control_pid_omni(th*Degree_to_Rad,7);
				
				if(!Pin0_23)
					counter_detect++;
				
				if(counter_detect>25)
				{
					//next=1;
					flag_detect=3;
					hold_angle=0;
					counter_detect=0;
				}
				break;		 
	 }
	}
	
	int i = 2000;
		while(i-- && !manual_override)
		 control_pid_omni(90*Degree_to_Rad,15);
		
  next=0;
  counter_detect=0;
	flag_detect=0;
		
	lim11=80+20;               //these limits limit the speed of motors so that it doesn't beyond a range
	lim12=48-20;               //these limits limit the speed of motors so that it doesn't beyond a range
  lim21=208+20;              //these limits limit the speed of motors so that it doesn't beyond a range
	lim22=176-20;              //these limits limit the speed of motors so that it doesn't beyond a range
  
	lim11_lim=80+20;
  lim12_lim=48-20;
	lim21_lim=208+20;
	lim22_lim=176-20;	
		
  //====================== We have approached near the fence. now move towards the Rack to pick up golden shuttle cocks.==============
	while(!next && !manual_override)
	{
		//================================================Update Ps2 Values every 7ms=====================================================
		if(count_cycle>7)
		{
			count_cycle=0;
		  Ps2_val_update();
	  }
		
		//================================================Check for any sign of manual override===========================================
		if((temp_four!=400 && temp_four!=360))
		{
			hold_angle=0;
			manual_override=1;
		}
		
		//======================================= Get value from SHARP for wall following ================================================
		temp_val=((((double)4096-((unsigned int)((AD0DR5&0x80000000)?((AD0DR5&0x0000FFF0)>>4):temp_val)))/4096)*3000);
		
    //=============================== Smoothen it out :- Use a moving average filter with order of 10 ================================
		value_array[9]=temp_val;		
		for(int i=0;i<9;i++)
		{
			temp_val+=value_array[i];
			value_array[i]=value_array[i+1];			
		}
		
		temp_val=temp_val/9;
		
		th = 185 + (temp_val-825)/2;
	  
		//========================================== Limit the angle obtained to attain smooth motion ====================================
		if(th>195)
			th = 195;
		else if(th<185)
			th=185;
		
		//======================================Accelerate the robot till its velocity reaches a certain point============================
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
	  		
		//========================== Speed up according to accelaration and after full speed is reached, decellerate =====================
		switch(flag_detect)
		{
			case 0:																											//move towards rack till proxy doesnt detect anything.
				if(lim11!=lim11_lim)
					control_pid_omni(th*Degree_to_Rad,25);
				else
					control_pid_omni(th*Degree_to_Rad,8);
				
				if(!Pin0_23)
					counter_detect++;
				
				if(counter_detect>50)
				{
					flag_detect=1;
					counter_detect=0;
				}
				break;
							
			case 1:																											//move slowly towards rack till proxy is detecting the rack.
				control_pid_omni(th*Degree_to_Rad,6);
				
				if(Pin0_23)
					counter_detect++;
				
				if(counter_detect>50)
				{
					flag_detect=2;
					counter_detect=0;
				}
				break;
		 case 2:																											//move towards rack till proxy doesnt detect anything. After this, exit the loop.
				control_pid_omni(th*Degree_to_Rad,6);
				
				if(!Pin0_23)
					counter_detect++;
				
				if(counter_detect>25)
				{
					//next=1;
					flag_detect=3;
					hold_angle=0;
					counter_detect=0;
				}
				break;
		 case 3:																											//move towards rack till proxy doesnt detect anything. After this, exit the loop.
				control_pid_omni(th*Degree_to_Rad,4);
				
				if(Pin0_23)
					counter_detect++;
				
				if(counter_detect>25)
				{					
					flag_detect=4;
					hold_angle=0;
					counter_detect=0;
				}
				break;
		case 4:																											//move towards rack till proxy doesnt detect anything. After this, exit the loop.
				control_pid_omni(th*Degree_to_Rad,4);
				
				if(!Pin0_23)
					counter_detect++;
				
				if(counter_detect>5)
				{
					next=1;
					flag_detect=4;
					hold_angle=0;
					counter_detect=0;
				}
				break;				
	 }		
	}
	
	i = 1000;
	while(i-- && !manual_override)
	{
		control_pid_omni(-10*Degree_to_Rad,15);
	  pos();
	}
	*/	
	/*
	while(1)
	{
	set(P2_4);
	cls();
	lcd(y_dis);
	control_pid_omni(40,5);
}
*/
	
	return;
	//=========================================================== End of:- towards rack golden ==================================================================
}

//
/*
//   forward proxy is for actuatinf the rack in tz1
//   reverse proxy is for actuating the rack in tz2
//   encoder tics are taken on basis of forward proxy
*/

void towards_manual_normal()
{	
	lim11=80+40;   //-5; //126; //80
  lim12=48-40;   //+5; //2;   //48
	lim21=208+40;  //-5; //253; //208
	lim22=176-40;
	
  //======================================================= start to load (PMV)      =======================================================================================	
	//================================================= START TO LOAD (PMV)=======================================================
		int rackproxy,temp=2000,proxy_count=0,rack_proxy =0;
		double factor,base_speed=30; 
		int proxy_count_pmv = 0;
		bool load_p=0,load_enc=0,decc=0,fuck_flag=0,pmv_proxy =0;
		acc=0.007;
		y_dis = 0; 
		T1TC=0;
		prvvaluer=0;
		next=0;
		manual_override=0;

																																																											//pmv's	
																																																											//2200
																																																											//2600
																																																											//2950
	 
	while(y_dis<150 && !manual_override)
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
					control_pid_omni(-170*Degree_to_Rad,30);
				}
			
		while(y_dis<2400 && !manual_override && rack_proxy<2)
			{
					if(Pin0_23 ==0)
					{
					fuck_flag = 1;
					}
					if(Pin0_23 == 1 && fuck_flag)
					{
					fuck_flag =0;
					rack_proxy++;
					}

					pos();

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

					base_speed = 40*exp(-(y_dis/1600));    //30->40    1500->1650->1500

					if (base_speed<23)    //23 to 22 just to solve the rack conditions //15->14
					base_speed=23;

					control_pid_omni(-170*Degree_to_Rad,base_speed);
			}
			
  next=0;
			
	while(!next && !manual_override && rack_proxy<2)
				{
						if(Pin0_23 ==0)
						{
						fuck_flag = 1;
						}
						if(Pin0_23 == 1 && fuck_flag)
						{
						fuck_flag =0;
						rack_proxy++;
						}
						pos();
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

						static double y_dis_temp=0;

						if((y_dis - y_dis_temp)<0.1)
						next=1;
							
						if(y_dis>2750)
						next=1;

						y_dis_temp=y_dis;

						control_pid_omni(-10*Degree_to_Rad,17);
				}
			
		 next=0;
		 
			while(!next && !manual_override && rack_proxy<2)
				{

						if(Pin0_23 ==0)
						{
						fuck_flag = 1;
						}
						if(Pin0_23 == 1 && fuck_flag)
						{
						fuck_flag =0;
						rack_proxy++;
						}		
						pos();

						control_pid_omni(-170*Degree_to_Rad,9);

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
						
						if(y_dis>2900)
						next=1;				 
			}
	
////===========loaded ball initialising i.e getting bot ready for next part of run======================================================
			
		lim11=80+40;   //-5; //126; //80
		lim12=48-40;   //+5; //2;   //48
		lim21=208+40;  //-5; //253; //208
		lim22=176-40;
		
		next=0;
		
		y_dis = 0; //new one
		T1TC=0;
		prvvaluer=0;
		
		acc=0.008;
		int count_p=0,current_dis=0;
		bool fuck_flag2=0;

		if(!manual_override)
			{
					set(P2_2);
					hold_angle =0;
				  set(P2_3);
					set(P2_5);
					set(P2_6);
			}
//#################################################################################################################################################			
//==============================================================TOWARDS AUTO=================================================================================
			lim11=80+2;//+5;
			lim12=48-2;//-5;
			lim21=208+2;//+5;
			lim22=176-2;//-5;
				
			lim11_lim=80+40;   //-5; //126; //80
			lim12_lim=48-40;   //+5; //2;   //48
			lim21_lim=208+40;  //-5; //253; //208
			lim22_lim=176-40;
			
			//-----------------------------------------	
				
			next=0;
			bool pmv =0;
			y_dis = 3200; //new one
			T1TC=0;
			prvvaluer=0;
			
			if(!manual_override)
			hold_angle = 0;
			acc=0.009;
			icontrol=40;
			
			int i=0;
			count_cycle1=0;
			
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

							 	if(y_dis<2500 && count_cycle>500)							 ////// 	if(y_dis<2500 || count_cycle1>500)
								  acc=0.009;

								//if(y_dis<2000 || count_cycle1 > 1550)
								//{
								//next=1;
								//reset(P2_1);
								//}
								
								if(y_dis<-400 && (FORWARD_PROXY))
									next = 1;

								//============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
								if((temp_four!=400) && (temp_four!=360))
								{
								hold_angle=0;
								manual_override=1;
								}

								//========================================== accelerate the robot by increasing the speed limit ======================================================
 
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

								th=0; 
								
                if(y_dis>2500)
										control_pid_omni((th*Degree_to_Rad),60);                   /////TH->5
								else
									  control_pid_omni((th*Degree_to_Rad),30);
						}
									
				icontrol=60;
				count_cycle1=0;
				
				lim11=80+40;
				lim12=48-40;
				lim21=208+40;
				lim22=176-40;
				
		  //the turning part of the previous code		see 1672(current 1631, i.e. 41 lines behind) post uncommenting																						
					/*
																													//================================================ FIRST JUNCTION HAS BEEN DETECTED. NOW, TURN ==============================================================
																													while(ang>-85 && !manual_override)
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
																															hold_angle=ang;
																															manual_override=1;
																														}
																														
																														//================= Decrement the hold_angle to change robot orientation so that the shuttle loading mechanism faces the auto =============================
																														hold_angle-=0.027;
																														
																														if(ang<hold_angle)
																															hold_angle=ang;
																														
																														if(hold_angle<-87)
																															hold_angle=-87;


																														 //============ Slow down the robot by removing driving power. Till the angle reaches a value, the robot just tries to maintain its orientation ===========		
																														 control_pid_omni((10-ang)*Degree_to_Rad,35);
																													}
																													
																													*/
				if(!manual_override)
							{
										reset(P2_3);	                     ////////////////    TO LIFT RACK BEFORE TRANSFER ZONE(AYUSH)  
										reset(P2_4);
								    reset(P2_1);
										hold_angle=ang;                              //change to -89 if turned part is uncommented
										turned=0;
							}

				icontrol=	40;       //60

				cls();
				lcd((char*)"laser reference"); 

				next=0;	
				y_dis = 0; //new one
				T1TC=0;
				prvvaluer=0;
				pmv=0;
        RACK_TURNED;
		while (!next && !manual_override)          ///////laser detect and manual override   this loop sends bot to 30 deg 
				{ 
							pos();
							static int temp_count=0;
							//============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
							if(count_cycle>7)
									{
										count_cycle=0;
										Ps2_val_update();
									}

							if((!Pin1_23||!Pin1_20))
										temp_count++;

							if(temp_count>20)
									{
										next=1;
									}	
							control_pid_omni(30*Degree_to_Rad,17);    //5                   /////// bot moves twrds transfer zone      
							if((temp_four!=400) && (temp_four!=360))
									{			
										manual_override=1;
									}	     
				}
				count_cycle1=600;
				BUTTON_ONE=1;				
				next=0;
				int sharp_value=0;

		/*=====================================Its pmv's mofu==========================================================
		//      author: pmv		
		//	    coauthor:	toto(He is as responsible for this code as anyone;//including me myself\\; can be)
		//      contact: 16bic032@nirmauni.ac.in  || 16bec102@nirmauni.ac.in
    //-------aur kinne personal sawaal pu6oge!! Mujhse nahi but haa toto se pu6 sakte ho!!---------------------------------------------				
		//                 Iss sawaal jawaab ko dhyaan se padhe aur samjhe varna ye aapke aur bot ke sehat ke liye bura, bohot bura hein!! 
		//			Q: What does the next part Does?
		//      A: The next part is really important, it controls the alignment of the bot along line 2770; apply PID and transfer the 
		//		     fucking ball into the auto!
		//			Q: Then what about the above part?
		//      A: It just got the bot in the position on full speed to get to the point where the angle part starts applying!!		
    //      don't change any value until asked to personally or by toto!				
		*///===========================================================================================================
		
    //the blind 120 searching for both sharp value with a particular tick value_distance loop_-=-=-=-=-__+-=-=+_=-=-=+-=__=-+_=-=-==_				
		while(!next && !manual_override)
				{
							pos();
								if(count_cycle>7)
											{
												count_cycle=0;
												Ps2_val_update();
											}
								if((temp_four!=400) && (temp_four!=360))
											{
												hold_angle=ang;
												manual_override=1;
											}
							sharp_value=(unsigned int)((AD0DR5 & 0x80000000)?((AD0DR5&0x0000FFF0)>>4):2048);//GetADC(Adc1);
							control_pid_omni(30*Degree_to_Rad,20);    //5   //17                /////// bot moves twrds transfer zone

								if (sharp_value>2790)         //2770      //////////////around2800
												next=1;
				}

		
		
		y_dis = 0; //new one
		T1TC=0;
		prvvaluer=0;
		int f_p_count=0;
		bool flag_p=0,switch_loop=1;
		double kp_to_auto_tz1 = 0;//3*Degree_to_Rad;	 //modified -. pmv
		int proxy_cond=0,temp_sharp=0;
		int base_value = 0;
		double distance=500,heading_1=0,actual_final=2770;	
		next=0;
	
    // this is the time when the bot used to search for laser reference in the second loop 
    /*
   while(!next && !manual_override) //&& !manual_override)          ///////laser detect and manual override  
				{ 
						sharp_value=(unsigned int)((AD0DR5 & 0x80000000)?((AD0DR5&0x0000FFF0)>>4):2048);//GetADC(Adc1);

						 if(!Pin1_20)                             ////  TZ2
									base_value=15;                        ////15->20    //17
						 else 
									base_value=7;		                     ////8->6   //12

  					 if (switch_loop==1)	 
								{
											 kp_to_auto_tz1 = (( sharp_value - 2770 )*2)*Degree_to_Rad ;   //2890
											 if (kp_to_auto_tz1 > 0.35)//0.7)//2.09)
													 {
															kp_to_auto_tz1 = 0.35;
													 }
											 else if (kp_to_auto_tz1 < -0.35)
													 {
															kp_to_auto_tz1 = -0.35;
													 }
											 control_pid_omni(PI/2 + kp_to_auto_tz1,base_value);
								}
								
						 pos();

						 if(count_cycle>7)
							 {
									count_cycle=0;
									Ps2_val_update();
							 }
						
						 if((temp_four!=400) && (temp_four!=360))
							 {
									 hold_angle = ang;
									 manual_override=1;
							 }

						 static int temp_count=0;

							if((Pin1_23==0) && ((ang<-88.5) && (ang>-89.5)))
									proxy_cond++;

							if(proxy_cond==1)
									next=1;
							
							temp_sharp  = sharp_value;	

				}

	 
	 */
		
	
		if(!manual_override)
				hold_angle = ang;
		next=0;
		int sharp_fir=temp_sharp,count=0;
		
		//PID loop===========-=-=-=-=-=-=-=-=-=-=-=========+======++++++++++++++++++++++++=======================++++++++++++++
		while(!next && !manual_override)//(!next && !manual_override)
				{
					if(!Pin1_20)                             ////  TZ2
							base_value=15;                        ////15->20    //17
					else 
							base_value=7;		                     ////8->6   //12
					
					if(count_cycle>7)
					{
						count_cycle=0;
						Ps2_val_update();
					}
					
					if((temp_four!=400) && (temp_four!=360))
					{
						hold_angle=ang;
						manual_override=1;
					}
					
					sharp_value=(unsigned int)((AD0DR5 & 0x80000000)?((AD0DR5&0x0000FFF0)>>4):2048);//GetADC(Adc1);
					 
					kp_to_auto_tz1 = (( sharp_value - 2770)*2)*Degree_to_Rad ;     //2770  //2870
					if (kp_to_auto_tz1 > 0.35)
						 {
								kp_to_auto_tz1 = 0.35;
						 }
					else if (kp_to_auto_tz1 < -0.35)
						 {
								kp_to_auto_tz1 = -0.35;
						 }
							
					if (FORWARD_PROXY==0 )//&& (sharp_value<2750 || sharp_value>2800))
							{
									if(!manual_override)
									{
										set(P2_4);	
										set(P2_1);										////////////////SET MEANS OPEN TO TRANSFER SHUTTLE                                                 
									}										
									base_value = 9;      //6 //
							}
					
					if (!Pin1_20)
							{
									if(!manual_override)
									{
										  reset(P2_4);
											reset(P2_1);
									}	
									next=1;
							}
					
							
					control_pid_omni(kp_to_auto_tz1 ,base_value);
				
				}

	
		if(!manual_override)
	         reset(P2_4);

  	next=0;
	  bool zone_exit=0;
	  count_cycle1=0;
	
		while(!next && !manual_override)//(!next && !manual_override)
				{
							if(count_cycle>7)
									{
											count_cycle=0;
											Ps2_val_update();
									}

							if((temp_four!=400) && (temp_four!=360))
									{
											hold_angle=ang;
											manual_override=1;
									}
						
							sharp_value=(unsigned int)((AD0DR5 & 0x80000000)?((AD0DR5&0x0000FFF0)>>4):2048);//GetADC(Adc1);
							 
							kp_to_auto_tz1 = (( sharp_value - 2770 )*2)*Degree_to_Rad ;
							 if (kp_to_auto_tz1 > 0.35)
									 {
											kp_to_auto_tz1 = 0.35;
									 }
							 
						   else if (kp_to_auto_tz1 < -0.35)
									 {
											kp_to_auto_tz1 = -0.35;
									 }
						
						 control_pid_omni((-1)*kp_to_auto_tz1,9);
												
							if(REVERSE_PROXY==0 || count_cycle1>2200)
									{
												zone1_exit=1;
												zone_exit=1;
														if(!manual_override)
																set(P2_4);
									}
							
							if((zone_exit==1)&&((FORWARD_PROXY==0)||(count_cycle1>3500)))
							{
									if(!manual_override)
										reset(P2_4);
								next=1;
							}
  			}

		while(!manual_override)
				{ 
						control_pid_omni(40,5); 
				}			

// towards_rack_golden();

	return;	
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-end of towards manual normal-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=



void Ps2_drive(double angle_of_motion)
{
	  if(count_cycle>7)
		{
			count_cycle=0;
		  Ps2_val_update();
	  }		
			
		if(temp_four!=400 && temp_four!=360)
		{
			kp = 2;     //3
			ki = 0.05;
			//kd = 1.5;     //5
			
			//temporary_hold_angle=hold_angle;
			//hold_angle=hold_angle-4*cos(temp_four-(ang*Degree_to_Rad));
			
			/*if(temp_four<(91*Degree	_to_Rad) && temp_four>(89*Degree_to_Rad) && (direction_control_flag == 0))
			{
				temporary_hold_angle=hold_angle;
				hold_angle=hold_angle+4;
				direction_control_flag=1;
			}
      else if(temp_four>(-91*Degree_to_Rad) && temp_four<(-89*Degree_to_Rad) && (direction_control_flag == 0))
			{
				temporary_hold_angle=hold_angle;
				hold_angle=hold_angle-4;
				direction_control_flag=1;
			}*/
		}
		else
		{
			kp = 0.8;
			ki = 0;
			kd = 0;
			//if(direction_control_flag==1)
			//{
				//hold_angle=temporary_hold_angle;
				//direction_control_flag=0;
			
			//}
		}
				
	//===========================================  MINOR ADJUSTMENT CHECK  ======================================================
	  if(ps_down && ps_left)
	  {
			lim11=80;               //these limits limit the speed of motors so that it doesn't beyond a range
			lim12=48;               //these limits limit the speed of motors so that it doesn't beyond a range
			lim21=208;              //these limits limit the speed of motors so that it doesn't beyond a range
			lim22=176;              //these limits limit the speed of motors so that it doesn't beyond a range

			//======================= This condition is theme specific. Not required in general case. ================================
			minor_adjustment_speed=11;

			while(ps_down && ps_left)// && !BUTTON_ONE && !BUTTON_TWO && !BUTTON_THREE)       //BUTTON_ONE ... BUTTON_THREE is theme specific. Remove it when using it as general code
			{
				Ps2_val_update();	
				if(minor_speed < minor_adjustment_speed)
					minor_speed +=0.3;
				if(minor_speed > minor_adjustment_speed)
					minor_speed=minor_adjustment_speed;
			  control_pid_omni(((3*PI)/4)-angle_of_motion,minor_speed);// 61				
			  stop_count=0;
			}
			minor_speed=10;
		}			
		else if(ps_down && ps_right)
		{
			lim11=80;               //these limits limit the speed of motors so that it doesn't beyond a range
			lim12=48;               //these limits limit the speed of motors so that it doesn't beyond a range
			lim21=208;              //these limits limit the speed of motors so that it doesn't beyond a range
			lim22=176;              //these limits limit the speed of motors so that it doesn't beyond a range
			
			//======================= This condition is theme specific. Not required in general case. ================================
			minor_adjustment_speed=11;
			
			while(ps_down && ps_right)// && !BUTTON_ONE && !BUTTON_TWO && !BUTTON_THREE)       //BUTTON_ONE ... BUTTON_THREE is theme specific. Remove it when using it as general code
			{
				Ps2_val_update();		
								 if(minor_speed < minor_adjustment_speed)
									minor_speed +=0.3;                           
								 if(minor_speed > minor_adjustment_speed)
									minor_speed=minor_adjustment_speed;
				control_pid_omni(-(3*PI)/4-angle_of_motion,minor_speed);// 61				   //(ORIGNAL) control_pid_omni(-(3*PI)/4-angle_of_motion,minor_speed);// 61	
				stop_count=0;
			}
							minor_speed=10;
		}			
		else if(ps_up && ps_left)
		{
			lim11=80;               //these limits limit the speed of motors so that it doesn't beyond a range
			lim12=48;               //these limits limit the speed of motors so that it doesn't beyond a range
			lim21=208;              //these limits limit the speed of motors so that it doesn't beyond a range
			lim22=176;              //these limits limit the speed of motors so that it doesn't beyond a range
			
			//======================= This condition is theme specific. Not required in general case. ================================
			minor_adjustment_speed=11;
			
			while(ps_up && ps_left)// && !BUTTON_ONE && !BUTTON_TWO && !BUTTON_THREE)       //BUTTON_ONE ... BUTTON_THREE is theme specific. Remove it when using it as general code)
			{
				Ps2_val_update();								
								 if(minor_speed < minor_adjustment_speed)
									minor_speed +=0.3;                           
								 if(minor_speed > minor_adjustment_speed)
									minor_speed=minor_adjustment_speed;
				control_pid_omni((PI/4)-angle_of_motion,minor_speed);// 61				
				stop_count=0;
			}
							minor_speed=10;
		}			
		
		else if(ps_up && ps_right)
		{
			lim11=80;               //these limits limit the speed of motors so that it doesn't beyond a range
			lim12=48;               //these limits limit the speed of motors so that it doesn't beyond a range
			lim21=208;              //these limits limit the speed of motors so that it doesn't beyond a range
			lim22=176;              //these limits limit the speed of motors so that it doesn't beyond a range
			
			//======================= This condition is theme specific. Not required in general case. ================================
			minor_adjustment_speed=11;
			
			while(ps_up && ps_right)// && !BUTTON_ONE && !BUTTON_TWO && !BUTTON_THREE)       //BUTTON_ONE ... BUTTON_THREE is theme specific. Remove it when using it as general code)
			{
				Ps2_val_update();	
								 if(minor_speed < minor_adjustment_speed)
									minor_speed +=0.3;                           
								 if(minor_speed > minor_adjustment_speed)
									minor_speed=minor_adjustment_speed;
				control_pid_omni(-(PI/4)-angle_of_motion,minor_speed);// 61				
				stop_count=0;
			}
							minor_speed=10;
		}			
		// perpendicular axis
		else if(ps_down)
		{
			lim11=80;               //these limits limit the speed of motors so that it doesn't beyond a range
			lim12=48;               //these limits limit the speed of motors so that it doesn't beyond a range
			lim21=208;              //these limits limit the speed of motors so that it doesn't beyond a range
			lim22=176;              //these limits limit the speed of motors so that it doesn't beyond a range
			
			//======================= This condition is theme specific. Not required in general case. ================================
			if(turned)
				minor_adjustment_speed=8;
			else
				minor_adjustment_speed=14;
			
			while(ps_down)// && !BUTTON_ONE && !BUTTON_TWO && !BUTTON_THREE)       //BUTTON_ONE ... BUTTON_THREE is theme specific. Remove it when using it as general code
			{
				Ps2_val_update();								
								
				if(minor_speed < minor_adjustment_speed)
		  		minor_speed +=0.3;                           
				if(minor_speed > minor_adjustment_speed)
					minor_speed=minor_adjustment_speed;
				if(turned)
				  control_pid_omni((170*Degree_to_Rad)-angle_of_motion,minor_speed);// 61		 //should have been PI, but using this to reduce somr weird error		
				else
					control_pid_omni((180*Degree_to_Rad)-angle_of_motion,minor_speed);// 61		 //should have been PI, but using this to reduce somr weird error		
				stop_count=0;
			}
			minor_speed=14;
		}
		else if(ps_up)
		{
			lim11=80;               //these limits limit the speed of motors so that it doesn't beyond a range
			lim12=48;               //these limits limit the speed of motors so that it doesn't beyond a range
			lim21=208;              //these limits limit the speed of motors so that it doesn't beyond a range
			lim22=176;              //these limits limit the speed of motors so that it doesn't beyond a range
			
			//======================= This condition is theme specific. Not required in general case. ================================
			if(turned)
				minor_adjustment_speed=20;
			else
				minor_adjustment_speed=14;
			
			while(ps_up)// && !BUTTON_ONE && !BUTTON_TWO && !BUTTON_THREE)       //BUTTON_ONE ... BUTTON_THREE is theme specific. Remove it when using it as general code
			{
				Ps2_val_update();	
						
			  if(minor_speed < minor_adjustment_speed)
				  minor_speed +=0.3;                           
			  if(minor_speed > minor_adjustment_speed)
				  minor_speed=minor_adjustment_speed;
				
				if(turned)
					control_pid_omni((10*Degree_to_Rad)-angle_of_motion,minor_speed);				  //should have been zero, but using this to reduce somr weird error
				else
					control_pid_omni((0)-angle_of_motion,minor_speed);				  //should have been zero, but using this to reduce somr weird error
				
				stop_count=0;
			}
							minor_speed=14;
		}
		else if(ps_left)
		{
			lim11=80;               //these limits limit the speed of motors so that it doesn't beyond a range
			lim12=48;               //these limits limit the speed of motors so that it doesn't beyond a range
			lim21=208;              //these limits limit the speed of motors so that it doesn't beyond a range
			lim22=176;              //these limits limit the speed of motors so that it doesn't beyond a range
			
			//======================= This condition is theme specific. Not required in general case. ================================
			minor_adjustment_speed=14;
			minor_speed=14;
			
			while(ps_left)
			{					
				Ps2_val_update();	
				
				if(minor_speed < minor_adjustment_speed)
					minor_speed +=0.3;                           
				if(minor_speed > minor_adjustment_speed)
					minor_speed=minor_adjustment_speed;
				
				control_pid_omni(((-80)*Degree_to_Rad)+angle_of_motion,minor_speed);           /////// ((PI/2)-angle_of_motion,minor_speed)				
				stop_count=0;
			}
			minor_speed=14;
		}
		else if(ps_right)
		{
			lim11=80;               //these limits limit the speed of motors so that it doesn't beyond a range
			lim12=48;               //these limits limit the speed of motors so that it doesn't beyond a range
			lim21=208;              //these limits limit the speed of motors so that it doesn't beyond a range
			lim22=176;              //these limits limit the speed of motors so that it doesn't beyond a range
			
			//======================= This condition is theme specific. Not required in general case. ================================
			minor_adjustment_speed=14;
			minor_speed=14;
			
			while(ps_right)
			{
				Ps2_val_update();	
					
				if(minor_speed < minor_adjustment_speed)
					minor_speed +=0.3;                 
				if(minor_speed > minor_adjustment_speed)
					minor_speed=minor_adjustment_speed;
				
				control_pid_omni(-(100*Degree_to_Rad)-angle_of_motion,minor_speed);	       ///(-(PI/2)-angle_of_motion,minor_speed)			  
				stop_count=0;
			}
			minor_speed=14;
		}		
		else
		{				
		 drive(angle_of_motion);				
		}
			
			//if(temp_four!=400 && temp_four!=360)
//				hold_angle=temporary_hold_angle;
}
//
void towards_rack_normal()
{
	// get all racks down.. if not already
	set(P2_1);
	set(P2_2);
	set(P2_3);
	reset(P2_4);
	set(P2_5); 
	set(P2_6);
	
	//==============================================================towards rack========================================================================	
	next=0;
	hold_angle = 0;
	manual_override=0;
		
	//====================================== Wait for either start command or for manual override =====================================
  while(!next && !manual_override)
	{
		//================================================ Update Ps2 Values every 7ms ==================================================
		if(count_cycle>7)
		{
			count_cycle=0;
		  Ps2_val_update();
	  }
	  
    control_pid_omni(40,10);
		
		if(ps_l1)
		  next=1;
		
		if(temp_four!=360 && temp_four!=400)
		{
			manual_override=1;
		}
	}
	
	count_cycle1=0;
	
	/*while(ang<50)
	{
		*value_1=upperlimit * ((cos(dr_angle)*0.866) - (sin(dr_angle)*0.5));
	  value_2=upperlimit * ((cos(dr_angle)*0.866) + (sin(dr_angle)*0.5));
	  value_3=upperlimit * (sin(dr_angle));		*
		
		ExtPrintbin(COM0,1,64+2);//(((64-value_2)>lim11)?lim11:(((64-value_2)<lim12)?lim12:(64-value_2))));
		
		if(ang<40)
     ExtPrintbin(COM1,1,64-15);
		else
			ExtPrintbin(COM1,1,64-6);
		
    ExtPrintbin(COM0,2,192-2);//(((192-value_1)>lim21)?lim21:(((192-value_1)<lim22)?lim22:(192-value_1))));	  
	}
	while(1)
	{
		hold_angle=50;
		control_pid_omni(50*PI,30);
	}*/
	
	
	
	
	
	next=0;
  hold_angle = 0;                                //change 'hold angle' as the wall follower structure tends to change the orientation of robot
	
	lim11=80+5;//+3;               //these limits limit the speed of motors so that it doesn't beyond a range
	lim12=48-5;//-3;               //these limits limit the speed of motors so that it doesn't beyond a range
  lim21=208+5;//+3;              //these limits limit the speed of motors so that it doesn't beyond a range
	lim22=176-5;//-3;              //these limits limit the speed of motors so that it doesn't beyond a range
	
	lim11_lim=80+7+10;
  lim12_lim=58-7-10;
	lim21_lim=208+7+10;
	lim22_lim=176-7-10;
	acc=0.005;
	icontrol=15;
	
	y_dis=0;
	T1TC=0;
	prvvaluer=0;
	
	int flag_detect=0,counter_detect=0;
	
	while(!next && !manual_override)
	{
		//================================================Update Ps2 Values every 7ms================================================
		if(count_cycle>7)
		{
			count_cycle=0;
		  Ps2_val_update();
	  }
		
		//================================================Check for any sign of manual override=======================================
		if((temp_four!=400 && temp_four!=360))
		{
			hold_angle=0;
			manual_override=1;
		}		
		
    //======================================= Get value from SHARP for wall following ==============================================		
		/*temp_val=((((double)4096-((unsigned int)((AD0DR5&0x80000000)?((AD0DR5&0x0000FFF0)>>4):temp_val)))/4096)*3000);
		
    //=============================== Smoothen it out :- Use a moving average filter with order of 10 ==============================
		value_array[9]=temp_val;		
		for(int i=0;i<9;i++)
		{
			temp_val+=value_array[i];
			value_array[i]=value_array[i+1];
		}
		temp_val=temp_val/10;
		
		th = 180 + (temp_val-825)/2;
	  
		//========================================== Limit the angle obtained to attain smooth motion ==================================
		if(th>190)                        //  this was 200
			th=190;
		else if(th<182)
			th=182;                         //
		*/
		
		th=182+3;
		//===============================================================================================================================
		pos();
		
		//======================================Accelerate the robot till its velocity reaches a certain point===========================
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
				
		//========================== Speed up according to accelaration and after full speed is reached, decellerate ====================
		
		
		switch(flag_detect)
		{
			case 0:																										//case 0: move towards rack until proxy detects the rack structure
							if(y_dis<1500)// && count_cycle1<1725)  //1685
								control_pid_omni(th*Degree_to_Rad,20);
							else
								control_pid_omni(th*Degree_to_Rad,8);
							
							if(!Pin0_23)
								counter_detect++;
							
							if(counter_detect>25)
							{
								flag_detect=1;
								counter_detect=0;
							}
							break;
							
			case 1:																										//case 1: move towards rack while proxy is detecting the rack structure; exit when the structure is cleared
							control_pid_omni(th*Degree_to_Rad,12);
							
							if(Pin0_23)
								counter_detect++;
							
							if(counter_detect>25)
							{
								flag_detect=2;
								counter_detect=0;
							}
							break;
							
			case 2:																									  //case 2: move slowly towards rack till the next point of rack is detected
							control_pid_omni(th*Degree_to_Rad,12);
							
							if(!Pin0_23)
								counter_detect++;
							
							if(counter_detect>4)
							{
								next=1;
								hold_angle=0;
								counter_detect=0;
							}
							break;
	 }
	}
		
	//================================================= Stop the robot by giving a jerk in opposite direction ========================================================
	int i = 1500;
	while(i-- && !manual_override)
	{
		control_pid_omni(-10*Degree_to_Rad,15);
	  pos();
	}
	
	return;
	//============================================================End of:- towards rack normal========================================================================
}
//
/*
int proxy_check()
{	
	int i_local;
	
	while(1)
	{
		control_pid_omni(40,20);
		
		if(Start_proxy==0)
		{	
			i_local=100;
  		while(i_local--)
				control_pid_omni(40,20);
			
			if(Start_proxy==0)
		  {
				i_local=100;
  		  while(i_local--)
					control_pid_omni(40,20);
			  if(Start_proxy==0)
		    {					
					while(Start_proxy==0)
							 control_pid_omni(40,20);
						 
	 				//while(Start_proxy==1)
					//		control_pid_omni(4*PI,20);
					cls();
          lcd((char*)"Start proxy");					
					return START_PROXY;
  			}
			}	  		
		}
		else if(Zone_two_proxy==0)
		{
			i_local=100;
  		while(i_local--)
				control_pid_omni(40,20);
			
			if(Zone_two_proxy==0)
		  {
				i_local=100;
  		  while(i_local--)
					control_pid_omni(40,20);
			  if(Zone_two_proxy==0)
		    {					
					while(Zone_two_proxy==0)
							 control_pid_omni(40,20);
						 
	 				//while(Start_proxy==1)
					//		control_pid_omni(4*PI,20);
					cls();
          lcd((char*)"Two proxy");
					return ZONE_TWO_PROXY;
				}
			}
		}
		else if(Zone_three_proxy==0)
		{
			i_local=100;
  		while(i_local--)
				control_pid_omni(40,20);
			
			if(Zone_three_proxy==0)
		  {
				i_local=100;
  		  while(i_local--)
					control_pid_omni(40,20);
			  if(Zone_three_proxy==0)
		    {					
					while(Zone_three_proxy==0)
							 control_pid_omni(40,20);
						 
	 				//while(Start_proxy==1)
					//		control_pid_omni(4*PI,20);
					cls();
          lcd((char*)"Three proxy");
					
					return ZONE_THREE_PROXY;
				}
			}
		}
	}	
}*/
//
void wait_pid(int value)
{	
	while(value--)
	{
		control_pid_omni(40,20);
	}
	return;
}
//
void angle_90()
{
	next=0;
	while(!next)
	{
		hold_angle-=0.01;
		
		if(hold_angle<=-90 && ang<=-90)
		{
			next=1;
			hold_angle=-90;
		}
				
		control_pid_omni(40,20);
	}
	return;
}
//
void angle_0()
{
	next=0;
	while(!next)
	{
		hold_angle+=0.01;
		if(hold_angle>=0 && ang>=0)
		{
			hold_angle=0;
	    next=1;
		}				
		control_pid_omni(40,20);
	}	
	return;
}
//
void find_line(char Direction_fin1,char Direction_fin2, int max_speed)
{
	/* --------------------FUNCTION FIND_LINE()-------------------------
	Parameters: Direction_fin1 - direction given in direction of previous
                               motion
	            Direction_fin2 - direction which we want to move finally
	            Max_speed      - speed of omni
	Return:     void
	
	Notes:      give direction which is opposite to previous direction of 
	            motion in Direction_fin1 and the direction in which the 
	            robot should move finally should be given in the feild 
	            marked Direction_fin2.
	*/
	bool in_loop=1;
	while(in_loop)
	{		
		pos();
    sensor_val_update_v();
	  sensor_val_update_h();
		
		/*cls();
		lcd(sum_sensor_v);
		lowerline();
		lcd(sum_sensor_h);
		*/
		switch(Direction_fin1)
		{
			case BACKWARD:  switch(Direction_fin2)
			               {
				                case FORWARD:   control_pid_omni(0*Degree_to_Rad,max_speed);
			                                  if(sum_sensor_h>1)
											                    in_loop=0;
													              break;
											  case BACKWARD:  cls();
																				lcd((char*)"err in parameters");
																				control_pid_omni(40,max_speed);                                 //stop condition
																				break;
											  case LEFT:      control_pid_omni(45*Degree_to_Rad,max_speed);
			                                  if(sum_sensor_h>1)
											                    in_loop=0;
																				break;
											  case RIGHT:     control_pid_omni(-45*Degree_to_Rad,max_speed);
			                                  if(sum_sensor_h>1)
											                    in_loop=0;
																				break;
										 }
				             break;
			case FORWARD: switch(Direction_fin2)
			               {
				                case BACKWARD:   control_pid_omni(180*Degree_to_Rad,max_speed);
			                                  if(sum_sensor_h>1)
											                    in_loop=0;
													              break;
											  case FORWARD:   cls();
																				lcd((char*)"err in parameters");
																				control_pid_omni(40,max_speed);                                 //stop condition
																				break;
											  case LEFT:      control_pid_omni(125*Degree_to_Rad,max_speed);
			                                  if(sum_sensor_h>1)
											                    in_loop=0;
																				break;
											  case RIGHT:     control_pid_omni(-125*Degree_to_Rad,max_speed);
			                                  if(sum_sensor_h>1)
											                    in_loop=0;
																				break;
										 }
				             break;
			case RIGHT:     switch(Direction_fin2)
			               {
				                case FORWARD:   control_pid_omni(45*Degree_to_Rad,max_speed);
			                                  if(sum_sensor_v>1)
											                    in_loop=0;
													              break;
											  case BACKWARD:  control_pid_omni(130*Degree_to_Rad,max_speed);
																				if(sum_sensor_v>1)
											                    in_loop=0;
																				break;
											  case LEFT:      control_pid_omni(90*Degree_to_Rad,max_speed);
			                                  if(sum_sensor_v>1)
											                    in_loop=0;
																				break;
											  case RIGHT:     cls();
																				lcd((char*)"err in parameters");
																				control_pid_omni(40,max_speed);			                                  
																				break;
										 }
				             break;
			case LEFT:    switch(Direction_fin2)
			               {
				                case FORWARD:   control_pid_omni(-35*Degree_to_Rad,max_speed);
			                                  if(sum_sensor_v>1)
											                    in_loop=0;
													              break;
											  case BACKWARD:  control_pid_omni(-130*Degree_to_Rad,max_speed);
																				if(sum_sensor_v>1)
											                    in_loop=0;
																				break;
											  case LEFT:      cls();
																				lcd((char*)"err in parameters");
																				control_pid_omni(40,max_speed);			                                  
																				break;
											  case RIGHT:     control_pid_omni(-90*Degree_to_Rad,max_speed);			                                  
																				if(sum_sensor_v>1)
											                    in_loop=0;
																				break;
										 }
				             break;
		}
	}
  return;	
}
//
void timer()
{
	/*_____________________________TIMER()_________________________________
	Uses/Method:      can be used to update number of variables when called
										by ISR of timer. Works// like a RTC.
	Returns :         void
	Scope:            universal                                          */
	
	if(time_limit<400)      //Just to avoid overflow
	  time_limit++;
	count_cycle++;
	count_cycle1++;
	count_cycle2++;	
	
//	if(flag_TZ3==1)
	
		
	if(universal)
		safety++;
}
	


//
float sensor_val_update_h()
{	
	/*_______________________SENSOR_VAL_UPDATE_V()_____________________________
	Uses/Method:      updates the sensor values taken from the line sensor.
                    Vertical direction only!
	                  Calculates the error and sum total of all the sensors
                    currently detecting the line.
	Returns :         void
	Scope:            universal                                          */
	
	static double net_err=0;
	
	//sensor_value[0] = Pin0_7;
	//sensor_value[1] = Pin0_8;
	sensor_value[2] = Pin2_3;
	sensor_value[3] = Pin0_9;
	sensor_value[4] = Pin2_5;
	sensor_value[5] = Pin2_4;
	//sensor_value[6] = Pin1_30;
	//sensor_value[7] = Pin0_3;
	
	sum_sensor_h = 0;
	//cls();
	//lowerline();
	for(int i=2;i<6;i++)
	{
		sum_sensor_h += sensor_value[i];	
		//lcd(sensor_value[i]);
	}
	if(sum_sensor_h!=0)
	{
		if(sum_sensor_h==1)
		{
			if(sensor_value[2])
			   net_err = 4;
			else if(sensor_value[3])
			   net_err = 1.5;
			else if(sensor_value[4])
			   net_err = -1.5;
			else if(sensor_value[5])
			   net_err = -4;
		}
		else if(sum_sensor_h==2)
		{
			if(sensor_value[2] && sensor_value[3])
				 net_err = 3;
			else if(sensor_value[3] && sensor_value[4])
				 net_err = 0;
			else if(sensor_value[4] && sensor_value[5])
				 net_err = -3;
		}
		else if(sum_sensor_h==3)
		{
			if(sensor_value[2] && sensor_value[3] && sensor_value[4])
				 net_err = 1.5;
			else if(sensor_value[3] && sensor_value[4] && sensor_value[5])
				 net_err = -1.5;
		}
	}	
	else
	{
		if(net_err<0)
			net_err=-5;
		else if(net_err>0)
			net_err=5;
	}
	return net_err;
}
//
float sensor_val_update_v()
{	
	/*_______________________SENSOR_VAL_UPDATE_H()_____________________________
	Uses/Method:      updates the sensor values taken from the line sensor.
                    Horizontal direction only!
	                  Calculates the error and sum total of all the sensors
                    currently detecting the line.
	Returns :         void
	Scope:            universal                                               */
	
	static double net_err=0;	
	/*
	lcd(Pin0_19); //5
		lcd(Pin0_20); //n
		lcd(Pin0_21); //3
		lcd(Pin0_22); //6
		lcd(Pin2_4);  //1
		lcd(Pin2_7);  //4
	*/
	//sensor_value[0] = Pin0_7;
	//sensor_value[1] = Pin0_8;
	sensor_value[2] = Pin2_4;
	sensor_value[3] = Pin0_21;
	sensor_value[4] = Pin2_7;
	sensor_value[5] = Pin0_19;
	//sensor_value[6] = Pin0_22;
	//sensor_value[7] = Pin0_3;
	
	sum_sensor_v = 0;
	//cls();
	for(int i=2;i<6;i++)
	{
		sum_sensor_v += sensor_value[i];	
		//lcd(sensor_value[i]);
	}
	if(sum_sensor_v!=0)
	{
		if(sum_sensor_v==1)
		{
			if(sensor_value[2])
			   net_err = -4;
			else if(sensor_value[3])
			   net_err = -2-1;			
			else if(sensor_value[5])
			   net_err = 4;
			else if(sensor_value[4])
			   net_err = 2;
		}
		else if(sum_sensor_v==2)
		{
			if(sensor_value[2] && sensor_value[3])
				 net_err = -3-1;
			else if(sensor_value[3] && sensor_value[4])
				 net_err = 0;
			else if(sensor_value[4] && sensor_value[5])
				 net_err = 3;
		}
		else if(sum_sensor_v==3)
		{
			if(sensor_value[2] && sensor_value[3] && sensor_value[4])
				 net_err = -1.5-1;
			else if(sensor_value[3] && sensor_value[4] && sensor_value[5])
				 net_err = 1.5;
		}     		
	}	
	else
	{
		if(net_err<0)
			net_err=-5;
		else if(net_err>0)
			net_err=5;		
	}
	return net_err;
}
//
void control_pid_omni(double dr_angle,double upperlimit)
{
	/*__________________________CONTROL_PID_OMNI()___________________________
	Uses/Method:      when provided with direction of motion and the maximum 
                    speed it drives the omni wheel drive while staying in limits
                    set by lim11, lim12, lim21, lim22 and the value of icontrol
                    and keeps the value given to motor in check.
	Returns :         void
	Scope:            universal                                          */
	
	/*
	Parameters:-	dr_angle is the angle the robot has to follow;
	              upperlimit  is the max speed with which the motor can move.

	Use:- used to drive omni robot which holds its orientation using pid which utilises the value of kp,ki & kd;

	Notes:-
	-lim11,lim12,lim21,lim22 are limits which control speed of motor. these limits are slowly changed to accelerate the bot slowly.
	 change is made in drive() function.
	-if constant axis is to be maintained(i.e the bot will move towards a given point without effect of orientation), then
	 instead of passing angle which is to be followed as the parameter, pass (angle_to_be_followed - current_angle_of_robot)
	 as parameter.
	-icontrol is the parameter which limits the PID control value. If set too high then drastic oscillations will be produced.
	 if set too low then the robot can't maintain constant angle.
	-value_1, value_2, value_3 can be calculated and derived using simple trigonometric functions.
	-wheels are numbered along with their respective variables in the order shown below:
	-if angle greater than 2PI is sent, it'll hold its angle without moving(this is for user's convenience, that is if the user
	 want's robot to stop but still wants application of PID
	                                              3
	                                             ___
	                                           /     \
	                                          /       \
	                                         /         \
	                                         \         /
																			   2  \__ __ _/  1

	                                         ( operator )

																				   COM1- motor 3
																		 COM0 channel 1 - motor 2
																		 COM0 channel 2 - motor 1
	*/
	 if(count!=2)             //this is controlled in Gyroscope interrupt named get_rate(). This checks the permanently fixed bits.
	 {
		 lowerline();
		 lcd((char*)" Gyro removed!!");
		 while(count!=2)
		 {
			 ExtPrintbin(COM1,1,64);
			 ExtPrintbin(COM0,1,64);
			 ExtPrintbin(COM1,1,192);
			 ExtPrintbin(COM0,1,192);
		 }
		 lowerline();
		 lcd((char*)"                ");
	 }

	dummyl = upperlimit;
	dummyr = upperlimit;
	
	static double kp_copy;
	kp_copy=kp;              //Keep a copy of Kp as Kp is changed in the below program for better action
	
	difference=hold_angle-ang;
	
	if(abs(dr_angle)<(10*PI))      //Driving in a particular direction not asked to hold angle
	{
	  value_1=upperlimit * ((cos(dr_angle)*0.866) - (sin(dr_angle)*0.5));
	  value_2=upperlimit * ((cos(dr_angle)*0.866) + (sin(dr_angle)*0.5));
	  value_3=upperlimit * (sin(dr_angle));		
	}
	else //Hold angle command
	{
		value_1=0;
		value_2=0;
		value_3=0;
		dr_angle=0;
   /* if(difference<0.7 && difference>-0.7)
		   kp=5;
		else //if(difference<-0.1)		*/
			//kp=2;//1.2;//(tan(difference*Degree_to_Rad)*12.7);
		
		/*if(kp<0.9)
			kp=0.9;
		
		cls();
		lcd(kp);*/
    /*if(difference>-0.6 && difference<0.6)
        kp=6;
    else
        kp=0.9;			*/
	}
	
	//convention of motor changed later.
	value_1=0-value_1;
	value_2=0-value_2;
	value_3=0-value_3;

//-------------------------PID Algorithm-----------------------------------
	if(difference!= 0)	{
		//-----------------Proportional------------------------
		proportional = difference * kp;
		//-------------------Integral--------------------------
		integral += difference;
		integrald = integral * ki;
		//------------------Derivative-------------------------
		rate = prevposition - difference;
		derivative = rate * kd;
		//--------------------Control--------------------------
		control = proportional+derivative+integrald;
		integral /= 1.3;
		//--------------------PID Ends-------------------------

		//limit on control parameter: so that if angle changes more than a limit, the motors don't go hay-wire.
		if(control>icontrol)
			control=icontrol;
		else if(control<(0-icontrol))
				control=(0-icontrol);
		///////////////////////////////////////////////////////////////////////////////////////////////////////////

		control=0-control;     // Just inversion of values. Was getting inverted output. :p

		// LOCAL VARIABLES FOR USE IN THE FUNCTION ITSELF
		static double t_lim11,t_lim12,t_lim22,t_lim21;
		t_lim11=lim11;
		t_lim12=lim12;
		t_lim22=lim22;
		t_lim21=lim21;

		//----  LIMIT THE MAXIMUM VALUES BEING SENT TO THE DRIVER
		if((lim11+control)>128)
				 lim11=128-control;
		else if((lim11+control)<0)
				 lim11=0-control;

		if((lim12+control)>128)
				 lim12=128-control;
		else if((lim12+control)<0)
				 lim12=0-control;

		if((lim21-control)>255)
				 lim21=255+control;
		else if((lim21-control)<129)
				 lim21=129+control;

		if((lim22-control)>255)
				 lim22=255+control;
		else if((lim22-control)<129)
				 lim22=129+control;
		//________________________________________________ Check for limits on th output values _____________________________________________________
		txt1=(((192-value_1-control)<(lim22-control))?(lim22-control):(((192-value_1-control)>(lim21-control))?(lim21-control):(192-value_1-control)));
		txt2=(((64-value_2+control)<(lim12+control))?(lim12+control):(((64-value_2+control)>(lim11+control))?(lim11+control):(64-value_2+control)));
		txt3=(((64+value_3+control)>(lim11+control))?(lim11+control):(((64+value_3+control)<(lim12+control))?(lim12+control):(64+value_3+control)));
    
		//Still check if limits will not hamper the motion of other mototrs
		if((int)txt1<129)
			txt1=129;
		else if((int)txt1>255)
			txt1=255;

		if((int)txt2>127)
			txt2=127;
		else if((int)txt2<1)
			txt2=1;

		if((int)txt3>127)
			txt3=127;
		else if((int)txt3<1)
			txt3=1;

		//___ Send the Values____
		ExtPrintbin(COM0,1,txt2);    
		ExtPrintbin(COM1,1,txt3);
		ExtPrintbin(COM0,2,txt1);
	  
	 
		//RESTORE LIMIT VALUES BACK TO PREVIOUS ONES
    lim11=t_lim11;
		lim12=t_lim12;
		lim22=t_lim22;
		lim21=t_lim21;
		/////////////////////////////////////////////
	}
	else// if(difference == 0)
	{
		//last argument is simply for keeping the value in range of 0-128 and 128-255.. using ternary operator
		ExtPrintbin(COM0,1,(((64-value_2)>lim11)?lim11:(((64-value_2)<lim12)?lim12:(64-value_2))));
    ExtPrintbin(COM1,1,(((64+value_3)>lim11)?lim11:(((64+value_3)<lim12)?lim12:(64+value_3))));
    ExtPrintbin(COM0,2,(((192-value_1)>lim21)?lim21:(((192-value_1)<lim22)?lim22:(192-value_1))));	  
	}
	prevposition = difference;             //Store error for use with Kd term (i.e. Rate of change of error)
	kp=kp_copy;
	return;
}
//
void drive(double angle_of_motion_drive)
{
/*
Function:   drive()
Return:     void
Parameters: void
Notes:      This function drives 3 wheel omni robot using adc values from ps2. Put this in a loop and the result will be
            achieved.
            Caution, set high enough value of icontrol and set values of average values of adc's o get apt results!
*/

	get_angle();                       // gets angle of joystick in radians
   
	/*
	1. Change, but, only as fast as society can accept!
	2. You should not follow a leader who can't lead!
	3. You should not initiate a war, if u can't sustain it!
	4. Do not change things whose outcome you dont know; if u dare change it, make sure u have a backup plan!
	5. if a mistake happens, somebody must be responsible!
	                           - 5 Commandments of Aniruddha

	*/   
   
	static int flag_speed=0;//,flg;

	//static double tmp_x,tmp_y,tmp_heading;

	if((temp_four<401 && temp_four>399) || count_cycle2 < 400)//(((adc_r_h<(adc_r_h_avg+400))&&(adc_r_h>(adc_r_h_avg-400)))&&((adc_r_v<(adc_r_v_avg+400))&&(adc_r_v>(adc_r_v_avg-400))))//2042,2013,2077,2042
	{
		 if(flag_speed>0)
         flag_speed--;

		   txt1=64;
			 txt2=64;
			 txt3=192;

		   lim11=64+base_value;lim12=64-base_value;             //limits for channel 1 -> with stop value 64    80,48
	     lim21=192+base_value;lim22=192-base_value;           //limits for channel 2 -> with stop value 192   208,176

			if(turn_final==1)
			{
				if(abs(ang-hold_angle)<5)      ///keep it 3
						hold_angle += 0.2;
			  turn_yes=1;				
			}
			else if(turn_final==-1)
			{
				if(abs(ang-hold_angle)<5)        ///keep it 3
					hold_angle -= 0.2;//hold_angle-=ang_increment;
				turn_yes=1;
			}
			else if(turn_yes>=1)
			{
				turn_yes++;
				if(turn_yes>4)
					turn_yes=0;
				hold_angle=ang;
			}       
         
			if(abs(hold_angle-ang)>1)
			{
				err_count2=0;
				err_count1=0;
			}
      icontrol=7;
      //stay where ever you are. just maintain your orientaiton -> send first argument >10PI 
			control_pid_omni(40,10);
			icontrol=60;
   }
	 else if(count_cycle2>400)                // if anything except stop condition
	 {			
		   err_count1=0;
			 err_count2=0;
		   //flg=1;
			 if(turn_final==1)
			 {
					 hold_angle+=0.01;//ang_increment;
					 turn_yes=1;				 
			 }
			 else if(turn_final==-1)
			 {
						hold_angle-=0.01;//ang_decrement;
						turn_yes=1;					
			 }
			 else if(turn_yes)
			 {
					turn_yes=0;
					hold_angle=ang;
			 }


			 if(lim11>83)
					acc=0.007;
			 else
					acc=0.002;			 
			 
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

				static double kp_copy1;
				kp_copy1=kp;
				//kp=2.5 - (((lim11_lim-lim11)*1.7)/(lim11_lim-(64+base_value)));
				
				control_pid_omni(b_heading-angle_of_motion_drive/*-((hold_angle*PI)/180)*/,62);	    // to keep axis const.. b_heading-hold_angle
				
				kp=kp_copy1;
	 }
}
//
void get_angle()
{
	static double check,check2;
	/*
	  waitms(1);
	  adc_r_h=((unsigned int)((AD0DR0&0x80000000)?((AD0DR0&0x0000FFF0)>>4):adc_r_h_avg));
    waitms(1);
	  adc_l_v=((unsigned int)((AD0DR6&0x80000000)?((AD0DR4&0x0000FFF0)>>4):adc_l_v_avg));
	  waitms(1);
	  adc_r_v=((unsigned int)((AD0DR5&0x80000000)?((AD0DR5&0x0000FFF0)>>4):adc_r_v_avg));
	  waitms(1);
	  adc_l_h=((unsigned int)((AD0DR4&0x80000000)?((AD0DR6&0x0000FFF0)>>4):adc_l_h_avg));
	  //b_heading=-(atan2(((double)adc_r_v-adc_r_v_avg),-((double)adc_r_h-adc_r_h_avg)));  //2026,2062
	*/
	
	  
		if(temp_four==PI/2)
		{ kp=2.2;												     //Kp for Omni
	    kd=0.6; 	  										 //Kd for Omni
	    ki=0;			  										 //Ki for omni
	    icontrol=8;                     //Maximum value for control of Omni - in terms of maximum value sent to the motor
	    b_heading=((-18)*Degree_to_Rad);	 
		}
		
		else
    {
		kp=2;												     //Kp for Omni
	  kd=0.6; 	  										 //Kd for Omni
	  ki=0;			  										 //Ki for omni
	  icontrol=10;                     //Maximum value for control of Omni - in terms of maximum value sent to the motor
		}
		
		if (temp_four==(-(PI/2)))
		{
			b_heading=((-165)*Degree_to_Rad);	
		}
	
	
	  if((temp_four!=360)&&(temp_four!=400))
			b_heading = temp_four;
	
///////////////////////////////////////////////    THIS PART ADDED (21-7-18)(ORIGNALLY NOT PRESENT)  (USED JUST FOR 2018 ) (FOR GENERAL PURPOSE REMOVE THIS)        //////////////////////////////////////		
	
		else 
		{
			b_heading = 4*PI;
			if(temp_four==400)
			{
				err_count2=0;
				//err_count1=0;
			}
		}

		// snippet for stopping jerky ride!!
		if(abs(check2-b_heading)>1.5)
		{
			lim11=80,lim12=48;             //limits for channel 1 -> with stop value 64    80,48
	    lim21=208,lim22=176;           //limits for channel 2 -> with stop value 192   208,176
		}
		check2=check;
		check=b_heading;

		/*
	  if(abs(b_heading)<0.2618)
			b_heading=0;
		else if(b_heading>0.2618  &&  b_heading<0.7854)
			b_heading=0.5235;
		else if(b_heading>0.7854  &&  b_heading<1.309)
			b_heading=1.0472;
		else if(b_heading>1.309  &&  b_heading<1.8326)
			b_heading=1.5708;
		else if(b_heading>1.8326  &&  b_heading<2.3562)
			b_heading=2.0944;
		else if(b_heading>2.3562  &&  b_heading<2.8798)
			b_heading=2.618;
		else if(abs(b_heading)>2.8798)
			b_heading=3.1416;
		else if(b_heading<-0.7854  &&  b_heading>-1.309)
			b_heading=-1.0472;
		else if(b_heading<-1.309  &&  b_heading>-1.8326)
			b_heading=-1.5708;
		else if(b_heading<-1.8326  &&  b_heading>-2.3562)
			b_heading=-2.0944;
		else if(b_heading<-2.3562  &&  b_heading>-2.8798)
			b_heading=-2.618;
		*/
}

//----------------------------------------------------- IMU VALUES----------------------------------------------------------------------
void get_rate()
{
	reset(Port0_16);
	res[0]=SPI_Communicate(0x80);
	res[1]=SPI_Communicate(0x00);
	res[2]=SPI_Communicate(0x00);
	res[3]=SPI_Communicate(0x00);
	set(Port0_16);
	
	//----Uncomment only for Debugging or for caliberation
	count++;
	aniruddha = aniruddha+ans;  
	
	ans=(res[3]|(res[2]<<8)|(res[1]<<16)|(res[0]<<24));
	count = (ans & (0x7<<29))>>29;
	ans=((ans & 0x001FFFE0)>>5);
	
	if(ans>32767)
		ans=ans-(65536);                               
	
	static double ans1=0;  
		
	ans1 = (double)ans-85.9;//.29;//-75.9;//75.49999305555556;//-75.5;//76.4;//75.9;//76.937;//-80.19;79.5;80.19;79.54     //79.6																																																			//review-I ke liye -86;/*main value 
	
	if((ans1<=20)&&(ans1>=-20))
	{
		ans1=(ans1)/800;
	}
	else
	{
		ans1=(ans1)/80;
	}
	
	//1573598
	//if(count==2)
	  ang+=(ans1*0.0025002);//0.0025087);//0.0025477);//0.0025002);	
	//else
	//	aniruddha++;
	return;
}
//
//-------------------------------------------------FUNCTIONS FOR DEAD RECON--------------------------------------------------------------

void pos()            // this tiny one does all the heavy work!! whole dead recon is based on this

{
	a1=T1TC;
	//b1=T0TC;

	/*if(Pin2_13==1)
	{tickl=b1-prvvaluel;}
	else
	{tickl=prvvaluel-b1;}
  */
	if(Pin2_12==1)
	{tickr=a1-prvvaluer;}
	else
	{tickr=prvvaluer-a1;}
  
	//   |_|

	//templ+=tickl;
	//tempr+=tickr;
	
	//_______________________________________________________________________________
	//---------------COMMENTED SECTION: NOT MEANT TO BE UNCOMMENTED------------------
	//-------------------------------------------------------------------------------
	//dth=(tickl-tickr)*onetick;     //onetick = (PI*wheel_dia)/(1024*bot_dia);
	//radiansPerCount= 2*Pi*(wheelDiameter/trackWidth)/countsPerRevolution
	//deltaHeading= (rightCounts - leftCounts)* radiansPerCount/2

	//ddis=(((tickl*left_dia)+(tickr*right_dia))*factor);

	//ddis=(tickl+tickr)*factor;     //factor= (PI*wheel_dia)/2048;
	//distancePerCount = Pi * diameterWheel /countsPerRevolution
	//deltaD=istance = (leftCounts + rightCounts)/2*distancePerCount

	//dis+=ddis;
	
	//---------------COMMENTED SECTION: NOT MEANT TO BE UNCOMMENTED------------------
	//_______________________________________________________________________________
	
	//th=(ang*Degree_to_Rad);
	//th+=dth;

	//left here is used for the encoder which points towards wheel 3; right is for encoder which is horizontal..
	//left_ddis=tickl*dis_per_count_left;
	right_ddis=tickr*dis_per_count_right;
  
	y_dis+=right_ddis;
	//x_dis+=((left_ddis*cos(th))-(right_ddis*sin(th)));//((ddis)*(cos(th)));
  //y_dis+=((right_ddis*cos(th))+(left_ddis*sin(th)));//((ddis)*(sin(th)));

	//prvvaluel=b1;
	prvvaluer=a1;
	return;
}

//
void set_speed_limit()
{
	//FUNCTION TO SET SPEED IN MANUAL/AUTO MODE. PROVIDES FIVE DIFFERENT SPEED RANGES FOR NORMAL MOTION WHILE THREE SPEEDS FOR ROTATION
	if(ps_r1)
		{
			ps_r1=0;
			if(speed==0)
			{
			  lim11_lim=91; //80
		    lim12_lim=35;   //48
		    lim21_lim=219; //208
		    lim22_lim=165; //176
				speed++;
				////Update_display();
				//icontrol=18;
		  }
			else if(speed==1)
			{
			  lim11_lim=102; //80
		    lim12_lim=24;   //48
		    lim21_lim=230; //208
		    lim22_lim=154; //176
				speed++;
				//Update_display();
			  //icontrol=25;
		  }
			else if(speed==2)
			{
			  lim11_lim=113; //80
		    lim12_lim=13;   //48
		    lim21_lim=241; //208
		    lim22_lim=143; //176
				speed++;
				//Update_display();
			  //icontrol=30;
		  }
			else if(speed==3)
			{
			  lim11_lim=126; //80
		    lim12_lim=2;   //48
		    lim21_lim=253; //208
		    lim22_lim=130; //176
				speed++;
				//Update_display();
			  //icontrol=30;
		  }
		}
	else if(ps_r2)
	{
		ps_r2=0;
			if(speed==1)
			{
			  lim11_lim=75; //80
		    lim12_lim=53;   //48
		    lim21_lim=203; //208
		    lim22_lim=181; //176
				speed--;
				//Update_display();
			  //icontrol=18;
		  }
			else if(speed==2)
			{
			  lim11_lim=91; //80
		    lim12_lim=35;   //48
		    lim21_lim=219; //208
		    lim22_lim=165; //176
				speed--;
				//Update_display();
				//icontrol=18;
		  }
			else if(speed==3)
			{
			  lim11_lim=102; //80
		    lim12_lim=24;   //48
		    lim21_lim=230; //208
		    lim22_lim=154; //176
				speed--;
				//Update_display();
				//icontrol=25;
		  }
			else if(speed==4)
			{
			  lim11_lim=113; //80
		    lim12_lim=13;   //48
		    lim21_lim=241; //208
		    lim22_lim=143; //176
				speed--;
				//Update_display();
				//icontrol=30;
		  }
		}
}
//
void Ps2_val_update()
{
	/*
	updates the variables named after ps2 buttons, call it just as is to update all values.
	put Bluetooth UART in COM2
	QUE:- is interrupt to ATmega better for speed?
	*/
	  ps_up=ps_right=ps_left=ps_down=ps_square=ps_triangle=ps_cross=ps_select=ps_start=ps_circle=ps_r1=ps_r2=ps_l1=ps_l2=0;

	  int temp_four1=0;
		static int temp_four2,temp_one,temp_one2,temp_two,temp_two2,temp_three,temp_three2,temp_turn;
		static int go=1,go_one=1,go_two=1,go_three=1;
	  //___________________________________________________________________________________________________________________________
	  //----------------------------------- INPUT FROM COM2 STORED IN BYTES  ----------------------------------------------------------
	  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		ps2_val[0]=Inputbin(COM2);
		ps2_val[1]=Inputbin(COM2);
		ps2_val[2]=Inputbin(COM2);
	  ps2_val[3]=Inputbin(COM2);
    //___________________________________________________________________________________________________________________________
	  //------------------------ IF COMMUNICATION HAS STOPPED, THEN STOP THE BOT FROM MOVING   ----------------------------------------------------------
	  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	  if(safety!=0)
		{
			lowerline();
			lcd((char*)"Ps2 Disconnected");
			while(safety!=0)                     // SAFETY BECOMES ZERO IN INPUTBIN() COMMAND -> SAFETY INCREASES IN TIMER-2 INTERRUPT
			{
				manual_override=1;
				Inputbin(COM2);
				ExtPrintbin(COM1,1,64);
				ExtPrintbin(COM0,1,64);
				ExtPrintbin(COM1,1,192);
				ExtPrintbin(COM0,1,192);
			}
			lowerline();
			lcd((char*)"                ");
		}
    //___________________________________________________________________________________________________________________________
    //-----------------------Byte decoding section; numbering of bytes, seeking actually pressed buttons, etc ---------------------
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    for(int u=0;u<4;u++)
    {
			 for(int i=0;i<8;i++)
			 {
				 ip[u][i]=ps2_val[u]%2;
				 ps2_val[u]=ps2_val[u]/2;
			 }
		}

		if((ip[0][1]==0)&&(ip[0][0]==0))
			one=0;
		else if((ip[0][1]==0)&&(ip[0][0]==1))
		  two=0;
		else if((ip[0][1]==1)&&(ip[0][0]==0))
		  three=0;
		else if((ip[0][1]==1)&&(ip[0][0]==1))
		  four=0;


		if((ip[1][1]==0)&&(ip[1][0]==0))
			one=1;
		else if((ip[1][1]==0)&&(ip[1][0]==1))
		  two=1;
		else if((ip[1][1]==1)&&(ip[1][0]==0))
		  three=1;
		else if((ip[1][1]==1)&&(ip[1][0]==1))
		  four=1;


		if((ip[2][1]==0)&&(ip[2][0]==0))
			one=2;
		else if((ip[2][1]==0)&&(ip[2][0]==1))
		  two=2;
		else if((ip[2][1]==1)&&(ip[2][0]==0))
		  three=2;
		else if((ip[2][1]==1)&&(ip[2][0]==1))
		  four=2;


		if((ip[3][1]==0)&&(ip[3][0]==0))
			one=3;
		else if((ip[3][1]==0)&&(ip[3][0]==1))
		  two=3;
		else if((ip[3][1]==1)&&(ip[3][0]==0))
		  three=3;
		else if((ip[3][1]==1)&&(ip[3][0]==1))
		  four=3;


		//temp_four=0;
		temp_one= 0;
		temp_two= 0;
		temp_three= 0;

		// to get adc values for driving
		for(int i=7;i>=4;i--){
			  temp_four1= (temp_four1*2) + ip[four][i];
		}

		// to get turn command

		if((ip[four][2]==1)&&(ip[four][3]==0))
			turn_adc=-1;
		else if((ip[four][2]==0)&&(ip[four][3]==1))
		  turn_adc=1;
    else
			turn_adc=0;

		static int turn_adc1,turn_adc2,turn_adc3;
		
		if((temp_turn+turn_adc+turn_adc1+turn_adc2+turn_adc3) > 2)
			turn_final=1;
		else if((temp_turn+turn_adc+turn_adc1+turn_adc2+turn_adc3) < -2)
			turn_final=-1;
		else
			turn_final=0;

    
		//___________________________________________________________________________________________________________________________
		//---------------------------Byte verification section, comparision of two bytes to check elligibality of data--------------
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`
		temp_turn=turn_adc2;
		turn_adc2=turn_adc1;
		turn_adc1=turn_adc3;
		turn_adc3=turn_adc;

		go_one=0;
		go_two=0;
		go_three=0;
		for(int i=7;i>=0;i--)
		{
			temp_one= (temp_one*2) + ip[one][i];
			temp_two= (temp_two*2) + ip[two][i];
			temp_three= (temp_three*2) + ip[three][i];
		}

		if(temp_one2==temp_one)//&&(temp_four3==temp_four1))
		{
   			go_one=1;
		}
      
		if(temp_two2==temp_two)//&&(temp_four3==temp_four1))
		{
   			go_two=1;
		}
      
		if(temp_three2==temp_three)//&&(temp_four3==temp_four1))
		{
   			go_three=1;
		}
		go=0;
		//static int temp_four3=0;
		if(temp_four2==temp_four1)//)&&(temp_four3==temp_four1))
		{
   			go=1;
		}

		//temp_four3=temp_four2;
		temp_three2=temp_three;
		temp_two2=temp_two;
		temp_four2=temp_four1;
		temp_one2=temp_one;
    #if DED_RECON
         pos();
    #endif

		if(go==1)
		switch(temp_four1)
		{	                                                    //final values
			case 0: temp_four=((0*PI)/180); // 0	180   0         180
		            break;
      case 1: temp_four=((60*PI)/180); // 30 	 -150   60    -120
		            break;
			case 2: temp_four=((75*PI)/180); 	// 60  -120   75     -105
		            break;
			case 3: temp_four=(((90)*PI)/180); 	 //90  -90     90    -90
		            break;
			case 4: temp_four=((105*PI)/180); 	 //120  -60   105    -75
		            break;
			case 5: temp_four=((120*PI)/180); 	 //150  -30    120    -60
		            break;
			case 6: temp_four=((180*PI)/180); 	 //180  0      180       0
		            break;
			case 7: temp_four=((-60*PI)/180); 	 //-30  150   -60      120
		            break;
			case 8: temp_four=((-75*PI)/180); 	 //-60  120    -75      105
		            break;
			case 9: temp_four=(((-90)*PI)/180); 	 //-90  90       -90      90
		            break;
			case 10: temp_four=((-105*PI)/180); 	 //-120  60    -105     75
		            break;
			case 11: temp_four=((-120*PI)/180); 	 //-150  30    -120      60
								break;
			case 12: temp_four=360;
								break;
			case 13: temp_four=360;
								break;
			case 14: temp_four=360;
								break;
			case 15: temp_four=400;
								break;
		}

		// Portion to stop jerky motion due to sudden change in joystick direction.
		//static double temp_four_,temp_1four,temp_2four,temp_3four,temp_4four,temp_5four,temp_6four,temp_7four,temp_8four,temp_9four,temp_10four;
		
		//if(((temp_10four>0  &&  temp_four<0) ||  (temp_10four<0  &&  temp_four>0)) && temp_10four!=400 && temp_four!=400)// && temp_4four!=400
			                    //&& temp_3four!=400 && temp_2four!=400 && temp_1four!=400 && temp_four_!=400)// && temp_four!=400)
		{
			//count_cycle2=0;
		}
		/*temp_10four=temp_9four;
		temp_9four=temp_8four;
		temp_8four=temp_7four;
		temp_7four=temp_6four;
		temp_6four=temp_5four;
		temp_5four=temp_4four;
		temp_4four=temp_3four;
		temp_3four=temp_2four;
		temp_2four=temp_1four;
		temp_1four=temp_four_;
    temp_four_=temp_four;*/
    // the jerky motion section ends
				
		//cls();
		if(go_two>0)
		{
		if(ip[two][2]==0)
		{	ps_up=1;		 }//lcd("UP");}
		if(ip[two][3]==0)
		{	ps_right=1;		}// lcd("RIGHT");}
		if(ip[two][4]==0)
		{	ps_down=1;		}//lcd("DOWN");}
		if(ip[two][5]==0)
		{	ps_left=1;		}//lcd("LEFT");}
    	}

		if(go_three==1)
		{
		if(ip[three][2]==0)
		{	ps_cross=1;		}// lcd("Cross");}
		if(ip[three][3]==0)
		{	ps_square=1;	}//	lcd("Square");}
		if(ip[three][4]==0)
		{	ps_select=1;	}//	lcd("SELECT");}
		if(ip[three][7]==0)
		{  ps_start=1;  }//   lcd("START");}
	   }
		if(go_one==1)
		{
		if(ip[one][2]==0)
		  ps_l2=1;        //lcd("L2");
		if(ip[one][3]==0)
		  ps_r2=1;        //lcd("R2");
		if(ip[one][4]==0)
		  ps_l1=1;        //lcd("L1");
		if(ip[one][5]==0)
		  ps_r1=1;        //lcd("R1");
		if(ip[one][6]==0)
		  ps_triangle=1;  //lcd("Triangle");
		if(ip[one][7]==0)
		  ps_circle=1;    //lcd("Circle");
	  }

	}
//
void control_pid_line_follow(char Direction, int max_speed, double error_line)
{
	if(error_line!=0)
	{		
		//-----------------Proportional------------------------
		proportional_line_sense = kp_line_sense * error_line;
		//-------------------Integral--------------------------
		integral_line_sense += error_line;
		integrald_line_sense = integral_line_sense * ki_line_sense;
		//------------------Derivative-------------------------
		rate_line_sense = prev_err - error_line;
		derivative_line_sense = rate_line_sense * kd_line_sense;
		//--------------------Control--------------------------
		control_line_sense = proportional_line_sense+derivative_line_sense+integrald_line_sense;
		integral_line_sense /= 1.3;
		//--------------------PID Ends-------------------------		
		
		if(control_line_sense>icontrol_line_sense)
			  control_line_sense=icontrol_line_sense;
		else if(control_line_sense<(0-icontrol_line_sense))
				control_line_sense=(0-icontrol_line_sense);
				
		/*sum_sensor_h = Pin2_3+Pin0_9+Pin2_5+Pin2_4;
		if(sum_sensor_h==4)
			return;*/
				
		switch(Direction)
		{
			case FORWARD:   control_pid_omni(control_line_sense*Degree_to_Rad,max_speed);                //FORWARD depicts heading=0 degree
			                break;
			case BACKWARD:  control_pid_omni(PI - (control_line_sense*Degree_to_Rad),max_speed);        //FORWARD depicts heading=0 degree
			                break;			
			case RIGHT:     control_pid_omni(-PI/2+(control_line_sense*Degree_to_Rad),max_speed);				 //RIGHT depicts heading= -90 degree
			                break;
			case LEFT:      control_pid_omni(PI/2-(control_line_sense*Degree_to_Rad),max_speed);         //LEFT depicts heading=90 degree
			                break;			
		}
   
    return;		
	}
	else
	{
		switch(Direction)
		{
			case FORWARD: control_pid_omni(0,max_speed);						//FORWARD depicts heading=0 degree
			              break;
			case BACKWARD:control_pid_omni(PI,max_speed);						//FORWARD depicts heading=0 degree
			              break;
			case RIGHT:   control_pid_omni(-PI/2,max_speed); 				//RIGHT depicts heading= -90 degree
			              break;
			case LEFT:    control_pid_omni(PI/2,max_speed); 				//LEFT depicts heading=90 degree
			              break;
		}
		
		return;
		//After this, refer to control_pid_omni() for more detail
	}	
}
//
//==========================================================================================================================
//==========================================================================================================================
//        DESCRIPTION :: manual starts after the golden ball load and moves towards auto in zone two
//                       ->using laser encoder (before turning)
//                       ->sharp for setting the path and deccelration 
//                       ->laser for final position 
//        ALGORITHM :: 
//
//===========================================================================================================================
void golden_load_to_auto()
{
		
	lim11=80+40;   //-5; //126; //80
  lim12=48-40;   //+5; //2;   //48
	lim21=208+40;  //-5; //253; //208
	lim22=176-40;
	
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
 //============================================load to auto=============================
 
 // reset(P2_4);

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
//===============================================================================================================================================	
	// get all racks down.. if not already
	set(P2_1);
	set(P2_2);
	set(P2_3);
  set(P2_5);
	set(P2_6);
	
	//==============================================================Towards Auto=================================================================================
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
	next=0;
	
	y_dis = 3200; //new one
	T1TC=0;
	prvvaluer=0;
	
  hold_angle = 0;
	acc=0.009;
	icontrol=40;
	
	int i=0;
	count_cycle1=0;
	
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
		//if(count_cycle1>500)
		//reset(P2_1);
		if(y_dis<2500-Golden_offset || count_cycle1>500)                                    ///confirm it before run
			acc=0.009;
		if(y_dis<2000-Golden_offset || count_cycle1 > 1550)
		{
  	 next=1;
		  //reset(P2_1);
			/*
			while(1)                                  ///////////////////  THIS WHILE IS FOR BOT STOP USING ENCODER TICKS 
		       {
           control_pid_omni(4*PI,5);		        
				   }			 
	    */
		}
		//============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
		if((temp_four!=400) && (temp_four!=360))
		{
			hold_angle=0;
			manual_override=1;
		}
		//========================================== accelerate the robot by increasing the speed limit ======================================================
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
	 	*/
		//if(th<-4)
			th=0;
		//else if(th>5)
//			th=5;				
		control_pid_omni(((th*Degree_to_Rad)),60);                   /////TH->5
	}
	reset(P2_4);       //lock golden rack to stop any golden shuttles from getting in
	icontrol=60;
	count_cycle1=0;
	
	lim11=80+25;
	lim12=48-25;
  lim21=208+25;
	lim22=176-25;
	
//	if((Pin1_23||Pin1_20) && !pmv ) 		
	if(!manual_override)
	{
			reset(P2_3);	                     ////////////////    TO LIFT RACK BEFORE TRANSFER ZONE(AYUSH) 
	    reset(P2_1);
	}		
  //     pmv=1;	  
	
	//================================================ FIRST JUNCTION HAS BEEN DETECTED. NOW, TURN ==============================================================
	while(ang>-85 && !manual_override)
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
			hold_angle=ang;
			manual_override=1;
		}
		
		//================= Decrement the hold_angle to change robot orientation so that the shuttle loading mechanism faces the auto =============================
		hold_angle-=0.027;
		
		if(ang<hold_angle)
			hold_angle=ang;
		
		if(hold_angle<-87)
			hold_angle=-87;
	/*	
		//=========================== If line sensor detects line, then exit the loop. Debouncing is provided to reduce error =====================================
		sensor_val_update_v();
		
		if(sum_sensor_v>0 && count_cycle1>300)
		{
			i=100;
	    while(i--);
		  sensor_val_update_v();
		  
			if(sum_sensor_v>0)
			{
			  i=100;
	      while(i--);
		    sensor_val_update_v();
		    
				//if(sum_sensor_v>0);    //abhinavtechnologies11@gmail.com
					//next=1;
		  }
		}
		*/
		//============ Slow down the robot by removing driving power. Till the angle reaches a value, the robot just tries to maintain its orientation ===========		
		//if(ang<-10)
		   control_pid_omni((10-ang)*Degree_to_Rad,35);
		/*
		else
			control_pid_omni(40,10);*/
	}
	
	sensor_val_update_v();
	if(!manual_override)
	{
		hold_angle=-89;
		turned=0;
	}
////	set(P2_1);                   //////////////////////// RACK  ACTUATION JUST AFTER  ROTATION 	
	/*
	i=2000;
	while(i--)
	{
		control_pid_omni(-PI/2,9);
	}*/
		
	/*
	next=0;	
	manual_override=1;
	
	i=800;
	while(i-- && !manual_override)
		control_pid_omni((-100-ang)*Degree_to_Rad,8);
	
	icontrol=8;                          // This should reduce the wobble as the robot tries to hold its angle
		
	while(!next && !manual_override)
	{
		//============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
		if(count_cycle>7)
		{
			count_cycle=0;
		  Ps2_val_update();
	  }
		
		//============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
		if(temp_four!=400 && temp_four!=360)
		{
			manual_override=1;
		}
    
    //=========================== If line sensor detects line, then exit the loop. Debouncing is provided to reduce error =====================================
		sensor_val_update_v();
		
		if(sum_sensor_v>0)
		{
			i=100;
			while(i--);
		  sensor_val_update_v();
		
			if(sum_sensor_v>0)
			{
			  i=100;
			  while(i--);
		    sensor_val_update_v();
		
				if(sum_sensor_v>0)
					next=1;
		  }
		}
		
		control_pid_omni(85*Degree_to_Rad,3);      //to reduce the massive inertia, the speed is so less
	}
	
	lim11=80+10;
	lim12=48-10;
  lim21=208+10;
	lim22=176-10;
	
	//================================================= Stop the robot by giving a pulse in reverse direction ===================================================
	i=2700;
	next=0;
	while(i-- && !manual_override && !next)
	{
		control_pid_omni(-90*Degree_to_Rad,20);
		sensor_val_update_v();
		if(sum_sensor_v>0)
		{
			i=100;
			while(i--);
		  sensor_val_update_v();
		
			if(sum_sensor_v>0)
			{
			  i=100;
			  while(i--);
		    sensor_val_update_v();
		
				if(sum_sensor_v>0)
					next=1;
		  }
		}
	}
	
	sensor_val_update_v();
	
	//================= Find the line once we have overshot it. In ideal case, it is not required at all. But, for safety, we include it ========================
	next=0;
	while(!next && !manual_override)
	{
		control_pid_omni(-87*Degree_to_Rad,6);
	  sensor_val_update_v();
		if(sum_sensor_v)
			i++;
		
		if(i>200)
			next=1;
	}
	*/	
  //	
	icontrol=	60;

  cls();
  lcd((char*)"laser reference"); 
	
next=0;	
	y_dis = 0; //new one
	T1TC=0;
	prvvaluer=0;
/////////////////////////////////////////////////////   BOT STOP USING LASER ///////////////////////////////////   
///bool pmv=0;

while (!next && !manual_override)          ///////laser detect and manual override  
{ 
	// pos();
	 static int temp_count=0;
//============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
		if(count_cycle>7)
		{
			count_cycle=0;
		  Ps2_val_update();
	  }
		if((!Pin1_23||!Pin1_20)  && ang<-88.5 && ang>-89.5)
			temp_count++;
		
		if(temp_count>20)
		{
			next=1;
		}	
		control_pid_omni(120*Degree_to_Rad,21);    //5                   /////// bot moves twrds transfer zone      
		 //============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
		if((temp_four!=400) && (temp_four!=360))
		{	
				hold_angle = ang;
			manual_override=1;
		}
}
//==========================
        count_cycle1=600;
        BUTTON_ONE=1;				
//      actuate();
//==========================
next=0;
int sharp_value=0;
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
      hold_angle = ang;			
			manual_override=1;
		}	    
		
	sharp_value=(unsigned int)((AD0DR5 & 0x80000000)?((AD0DR5&0x0000FFF0)>>4):2048);//GetADC(Adc1);
	control_pid_omni(120*Degree_to_Rad,17);    //5  //20                 /////// bot moves twrds transfer zone
	if (sharp_value>2790)           //2700
	{
		next=1;
	}
}
/*
  	while(0)
	{
		cls();
		lcd(Pin1_23);
		//cls();
		//lcd(y_dis);
		control_pid_omni(PI/2,6);
	}

	y_dis = 0;//3100; //new one
	T1TC=0;
	prvvaluer=0;
  next=0;
  int temp_count_2=0;
  while(!next)//(!next && !manual_override)
	{
		//cls();
		//lcd(Pin1_23);
	//	lowerline();
	//	lcd(temp_count_2);
		   pos();
		
	//  if(count_cycle>7)
	//	{
	//		count_cycle=0;
	//	  Ps2_val_update();
	 // }
		
//============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
	//	if((temp_four!=400) && (temp_four!=360))
	//	{			
	//		manual_override=1;
	//	}	   
		
		if(Pin1_23==0)//((!Pin1_23) && ang<-88.5 && ang>-89.5)
			temp_count_2++;
		
		
		if((count_cycle>7)&&(Pin1_23==0))//(temp_count_2>20))
		{
			 //next=1;
			//break;
			cls();
			lcd((char *)"working");
		
		}			
		//break;//next=1;
		
	  control_pid_omni(90*Degree_to_Rad,6);
		
	}

	while(1)
	{
		cls();
		lcd((char *)"fuck");
		control_pid_omni(4*PI,10);
	}
	
	
	int z=500;
	while(z--)
	{
		 control_pid_omni(PI/2,10);
	}
	
//=================================================================================================================================
//=================================== ALIGN THE BOT USING ENCODER , LASER IN TZ1 ==================================================
/*                    **DESCRRIPTION::
/                              last position is decided by laser in vertical direction 
/                        encoder is used to decided the path and control speed (deacceraltion)
/                     **ALGORITHM USED::
/                                               ((final_dis - current_dis)/final_dis)
/                        speed  = base_value * e
/
*/
//=================================================================================================================================
//=================================================================================================================================
	
	y_dis = 0; //new one
	T1TC=0;
	prvvaluer=0;
	
//===========	
	int f_p_count=0;
	bool flag_p=0,switch_loop=1;
  double kp_to_auto_tz2 = 0;//3*Degree_to_Rad;	 //modified -. pmv
 
//===========	
	
  int proxy_cond=0,base_value=15,temp_sharp=0;
	double distance=500,heading_1=0,actual_final=2770;	
  next=0;
	cls();
	//lcd((char *)"1");
  while(!next && !manual_override) //&& !manual_override)          ///////laser detect and manual override  
   { 
	 /*
		  if(!FORWARD_PROXY)
		    f_p_count++;
			
			if (f_p_count>5)
			{
				flag_p=1;
			
			}
			*/
		 
	 /*
		 if ((sharp_value>2770)&&(switch_loop==0))
			 {
		    heading_1 = 90 +(20- 35.461*((actual_final-(double)sharp_value)/2000));//90 - (2450 - sharp_value)/10; //90 +(25- 35.461*((3870-(double)sharp_value)/2000));          //29.4   //3770
				base_value=8; 
				control_pid_omni(heading_1*Degree_to_Rad , base_value); 
				switch_loop=1; 
		   }	 
		 */
	 /*
		while(sharp_value<2800)
		{
			sharp_value=(unsigned int)((AD0DR5 & 0x80000000)?((AD0DR5&0x0000FFF0)>>4):2048);//GetADC(Adc1);
			control_pid_omni(30*Degree_to_Rad,25);
		}			
		 */
		 
		 
//	if(!Pin1_20)                             ////  TZ2
	   base_value=17;                        ////15->20   /isko chedna mat mc!!!! pmv's boli;bandook ki goli
 // else 
  //   base_value=10;		                     ////8->6
	/*
	if ((sharp_value>2765)||(sharp_value<2775))
		 base_value=20;
	else 
		base_value=15;
	*/	 
	if (switch_loop==1)	 
	{
		// base_value=20;
		 kp_to_auto_tz2 = (( sharp_value - 2770 )*2)*Degree_to_Rad ;
     if (kp_to_auto_tz2 > 0.35)//0.7)//2.09)
		 {
			// base_value=15;
			 kp_to_auto_tz2 = 0.35;
		 }
	 	 else if (kp_to_auto_tz2 < -0.35)
		 {
			// base_value=17;
			 kp_to_auto_tz2 = -0.35;
		 }
		 control_pid_omni(PI/2 + kp_to_auto_tz2,base_value);
	 }
	   pos();
	 sharp_value=(unsigned int)((AD0DR5 & 0x80000000)?((AD0DR5&0x0000FFF0)>>4):2048);//GetADC(Adc1);
	  /*
		 if(sharp_value<2770)
	
		if(sharp_value>2770)
		control_pid_omni(PI/2 + kp_to_auto_tz1,10);
		*/
	
		 /*
			 if (sharp_value>2770)
			 {
		    heading_1 = 90 +(21- 35.461*((actual_final-(double)sharp_value)/2000));//90 - (2450 - sharp_value)/10; //90 +(25- 35.461*((3870-(double)sharp_value)/2000));          //29.4   //3770
				base_value=10; 
		   }
			 else 
			 {
				 base_value=10;
				 heading_1=80;
			 }
			 */
				 /*
				 //  if(heading_1>140)
			   //		heading_1=140;
			  //	if(heading_1<50)
			  //	heading_1=50;
			// control_pid_omni(heading_1*Degree_to_Rad,base_value);

*/
//============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
		 if(count_cycle>7)
		   {
			   count_cycle=0;
		     Ps2_val_update();
	     }
		// cls();
	 //  lcd(Pin1_23);	 
		 static int temp_count=0;
		
		if((Pin1_20==1) && ((ang<-88.5) && (ang>-89.5)))
			 proxy_cond++;//temp_count++;
		
		//if(proxy_cond>9)     //20
	//		 next=1;
	   
	/*	if(proxy_cond>9)     //20
		{
      lowerline();
			lcd((char*)"Jindagi Fuck");
		 count_cycle=0;
			while(count_cycle<300)
			{control_pid_omni(heading_1*Degree_to_Rad,15); }     //6
		  next=1;
		}  
		
	  if (!FORWARD_PROXY)
		{
			reset(P2_4);
		}			
		*/
		
    if(proxy_cond==1)
		{
			next=1;
			//set(P2_4);
		}	
    
		//if (!Pin1_20)
			// ;
		
	  //((90 +(30- 29.4*((3770-(double)sharp_value)/2000)))*Degree_to_Rad,base_value);//(110*Degree_to_Rad,base_value);//(-PI/2,5);                       /////// bot moves twrds transfer zone      
//============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
		 if(count_cycle>7)
		   {
			   count_cycle=0;
		     Ps2_val_update();
	     }

		//============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
		 if((temp_four!=400) && (temp_four!=360))
		   {
hold_angle = ang;				 
			   manual_override=1;
		   }
		temp_sharp  = sharp_value;    
   }
//===========================================================================================================================
//------------------------------------------LOOP ENDS -------------------------------------------------------------------	 
//===========================================================================================================================
//=============================================================================================================================
//--------------------------------------JERK FOR STABELISIING THE BOT ----------------------------------------------------------
//===============================================================================================================================	 
	next=0;
	int sharp_fir=temp_sharp,count=0;
	while(!next && !manual_override)//(!next && !manual_override)
	{
		//============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
		/* 
		if(count_cycle>7)
		   {
			   count_cycle=0;
		     Ps2_val_update();
	     }

		 if((temp_four!=400) && (temp_four!=360))
		   {			
			   manual_override=1;
		   }	     
	 sharp_value=(unsigned int)((AD0DR5 & 0x80000000)?((AD0DR5&0x0000FFF0)>>4):2048);//GetADC(Adc1);
   */
		
		
		sharp_value=(unsigned int)((AD0DR5 & 0x80000000)?((AD0DR5&0x0000FFF0)>>4):2048);//GetADC(Adc1);
		/*
		kp_to_auto_tz1 = (( sharp_value - 3100 )*0.5)*Degree_to_Rad ;
		control_pid_omni(PI/2 + kp_to_auto_tz1,8);
		 */
		 /*
		 if (count<5)
		 {
			 sharp_fir += sharp_value;
		 }
		 
		 if (count==5)
		 {
			 sharp_fir /= 5;
			 count=0;
		 }
		 */
		 if (Pin1_20)
			 base_value=19;                    ///8
		 else 
			 base_value=6;                     //////////   6->4        
		 
		 kp_to_auto_tz2 = (( sharp_value - 2770)*2)*Degree_to_Rad ;
     if (kp_to_auto_tz2 > 0.35)
		 {
			 kp_to_auto_tz2 = 0.35;
		 }
	 	 else if (kp_to_auto_tz2 < -0.35)
		 {
			 kp_to_auto_tz2 = -0.35;
		 }
		 
		// if (!Pin1_20)
		 control_pid_omni(PI/2+kp_to_auto_tz2 ,base_value);
		// else 
			//  control_pid_omni(PI/2+7*Degree_to_Rad ,base_value);
		 /*
		 if (sharp_value>temp_sharp)
		 control_pid_omni(PI/2 + 1*Degree_to_Rad,8);
	   else if (sharp_value<temp_sharp)
			control_pid_omni(PI/2 - 10*Degree_to_Rad,8); 
		
		*/
		
	  //	 control_pid_omni(PI/2 - 30*Degree_to_Rad,6);		
		/*
		if (sharp_value<2970)
		{
			while(1)
			{
				control_pid_omni(0,10);
			}
		}
		*/
		
	  /*
	//	if (FORWARD_PROXY==0 )//&& (sharp_value<2750 || sharp_value>2800))
	//	{
	//		set(P2_1);
	//	  next = 1;                                                   
	//	}
    //if (REVERSE_PROXY==0)
     // next=1;			
		*/
    //  TAK'S CONDITIONS
		if (FORWARD_PROXY==0 && !Pin1_20)
		{
		  set(P2_1);                                                   
		}
    if (REVERSE_PROXY==0 && !Pin1_20)
      next=1; 			
	}
	reset(P2_1);		 
//===============================================================================================================================	 
	next=0;
	i = 2900;
while(i--)
{
  control_pid_omni(-PI/2,11);
}

while(!manual_override)
{
			 if(count_cycle>7)
		   {
			   count_cycle=0;
		     Ps2_val_update();
	     }

		//============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
		 if((temp_four!=400) && (temp_four!=360))
		   {	
         hold_angle = ang;				 
			   manual_override=1;
		   }
   	sharp_value=(unsigned int)((AD0DR5 & 0x80000000)?((AD0DR5&0x0000FFF0)>>4):2048);//GetADC(Adc1);
	  //set(P2_4);	 
	  //cls();
	  //lcd(sharp_value);
	  control_pid_omni(40,5);
}
set(P2_3);
/*
//-------------------------------------------------
// towards_victory_robocon()
//-------------------------------------------------
*/	
  return;	
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void without_turn()
{
	set(P2_2);
	set(P2_3);
	set(P2_4);
  set(P2_5);                                              //// PUSHING  PIN ALLOWS THE BALL TO GO IN 
	lim11=80+2;//+5;
	lim12=48-2;//-5;
  lim21=208+2;//+5;
	lim22=176-2;//-5;
		
	lim11_lim=80+40;   //-5; //126; //80
  lim12_lim=48-40;   //+5; //2;   //48
	lim21_lim=208+40;  //-5; //253; //208
	lim22_lim=176-40;
		
//======================================================= start to load (AYUSH)      =======================================================================================	
	//================================================= START TO LOAD (PMV)=======================================================
int rackproxy,temp=2000,proxy_count=0,rack_proxy =0;
double factor,base_speed=30; 
	int proxy_count_pmv = 0;
bool load_p=0,load_enc=0,decc=0,fuck_flag=0,pmv_proxy =0;
	acc=0.007;
  y_dis = 0; 
	T1TC=0;
	prvvaluer=0;
  next=0;
  manual_override=0;
	
	
	while(y_dis<150 && !manual_override)
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
		control_pid_omni(-170*Degree_to_Rad,30);
	}
	
	while(y_dis<2400 && !manual_override && rack_proxy<2)
		{
						if(Pin0_23 ==0)
						{
							fuck_flag = 1;
						}
						if(Pin0_23 == 1 && fuck_flag)
						{
							fuck_flag =0;
						rack_proxy++;
						}							
			pos();
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
			base_speed = 40*exp(-(y_dis/1600));    //30->40    1500->1650->1500
			
			if (base_speed<23)    //23 to 22 just to solve the rack conditions //15->14
				base_speed=23;
			
			control_pid_omni(-170*Degree_to_Rad,base_speed);
		}
			
  	next=0;
		
		lcd('3');	
	while(!next && !manual_override && rack_proxy<2)
		{
		
		
	if(Pin0_23 ==0)
	{
		fuck_flag = 1;
	}
	if(Pin0_23 == 1 && fuck_flag)
	{
		fuck_flag =0;
	rack_proxy++;
	}
	pos();
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

			static double y_dis_temp=0;
			
			if((y_dis - y_dis_temp)<0.1)
			  next=1;
			  				
      if(y_dis>2750)
        next=1;
			
			y_dis_temp=y_dis;

			control_pid_omni(-10*Degree_to_Rad,17);
		 }
			
		 next=0;
		 
		 lcd('4');
			while(!next && !manual_override && rack_proxy<2)
			{

      	if(Pin0_23 	==0)
	      {
		    fuck_flag = 1;
	      }
	      if(Pin0_23 == 1 && fuck_flag)
	      {
		    fuck_flag =0;
	      rack_proxy++;
	      }		
				pos();
				
			   control_pid_omni(-170*Degree_to_Rad,10);
		  	 
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
			   //=====================================================================================================
      	 
				 if(y_dis>3000)
           next=1;				 
			}	
	
			//////////////////////////for golden take the below part and comment or delete the above part
	lim11=80+2;   //-5; //126; //80
  lim12=48-2;   //+5; //2;   //48
	lim21=208+2;  //-5; //253; //208
	lim22=176-2;
   next = 0;  
			cls();
	
//zone detection paramters
	y_dis = 0;
	T1TC = 0;
	prvvaluer = 0;			
	int zone_select =0;			
//	reset(Port2_3);
  count_cycle1=0;		
	while(!next && !manual_override)
	{
		//pos();
		
								//============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
								if(count_cycle>7)
								{
								count_cycle=0;
								Ps2_val_update();
								}

								pos();

							 	//if(y_dis<2500 && count_cycle>500)							 ////// 	if(y_dis<2500 || count_cycle1>500)
								acc=0.007;
								
								//============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
								if((temp_four!=400) && (temp_four!=360))
								{
								hold_angle=0;
								manual_override=1;
								}

								//========================================== accelerate the robot by increasing the speed limit ======================================================
 
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

								th=-5; 
								
								if(y_dis<-500)
								{
									reset(P2_4);
								}
		            
                if (y_dis<-800)
								{
									 reset(P2_2);
									 reset(P2_3);
									 reset(P2_4);
								}
  								
								if(y_dis<-3100)
								{											
									if(REVERSE_PROXY)
									zone_select++;
							  }
								
								if(zone_select>20)
									next = 1;
		            
								control_pid_omni((th*Degree_to_Rad),42);                          /////35 for golden
		}
	
	next = 0;
	T1TC = 0;					
  prvvaluer = 0;
	y_dis = 0;
  th = 0;							
	int previous_value = y_dis;	
	proxy_count = 0;
					
	while(!next && !manual_override)
	{
			
		pos();
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
//				 if(y_dis<-2000)
//				 next = 1;
			   if(y_dis<BREAK_DISTANCE)       
			    {         
				        th = -5;
						base_value = 28;                                                                  //35 for golden
				       // base_value = 30;
							//	base_value = 40*exp(-1*(y_dis/(0.5*TOTAL_DISTANCE)));
                //if(base_value<21)
									//		base_value = 21;
			    }
	
				 if(y_dis>=BREAK_DISTANCE)
					{
					//	base_value = 15;
									base_value = 18*exp(-1*((y_dis-BREAK_DISTANCE)/(0.15*TOTAL_DISTANCE)));      //35 for golden
                if(base_value<11)
											base_value = 11;                                                         //15 for golden
						th= -5; 
					}

					if(y_dis>previous_value)
						next =1;
					
					if(y_dis<-3800)       //HIGH SPEED ON -4200
					{
						control_pid_omni(-170*Degree_to_Rad,20);
					}
					
					if(previous_value<y_dis)
						next = 1;
					
					previous_value = y_dis;
          
				//	if(!Pin1_23)
//					{
	////					set(P2_4);
			//		}
		control_pid_omni(th*Degree_to_Rad,base_value);
		if(y_dis<-5000)
			next = 1;
	
	}	
	  bool fuck_flag_returns = 0;
  	next = 0;
   base_value = 7;
	th = -10;
	count_cycle1=0;
	bool flag2 = 0;
	
  while(!next && !manual_override)
  {
	pos();

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
				 
				 if(!Pin1_23)                                  ///////////////  DETECTING THE laser falling edge
				 {
					// count_cycle1 = 0;
					 th = -4;
					 base_value = 3;
					 set(P2_4);
					 flag2 = 1;
				  // reset(P2_5);                                  ////////////// pushing11 the thread    reset was removed and the ball got in
				 }
				 if (Pin1_23)
				 {
					 count_cycle1=0;
				 }
         if(!FORWARD_PROXY)
				 {
					 th = -10;
					// set(Port0_7); 
				 }
				 
				 if(count_cycle1>100 && (flag2 ==1))                      ////////////   count cycle added now (paras)
					 reset(P2_5);                                 /////////////   pushing the thread   
			
         if(!REVERSE_PROXY)
				 base_value = 7;					 
				 
				 if(Pin1_20)                              ////////////  rising edge of golden shuttle 
				 {
					 reset(Port0_7);
					 next = 1;
				   set(P2_5);                                   //////////////  NO pushing  
				 }
	       control_pid_omni(th*Degree_to_Rad,base_value);				
	}


	int	i=2000;
	while(i--  && !manual_override)
  {
		control_pid_omni(-170*Degree_to_Rad,18);
	}
	next = 0;
	
	set(P2_5);
	
	while(!next && !manual_override)
	{ 
		pos();
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
		if(!Pin1_20)
		next = 1;
		
		control_pid_omni(-10*Degree_to_Rad,7);
	}

  th = -170;
	base_value = 7;
	next = 0;
  int count_22=0;	 
	count_cycle1=0;
	bool flag = 0;
	y_dis = 0;
	T1TC = 0;
	prvvaluer = 0;
	
	
	while(!next && !manual_override)
  {
			pos();
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

				 if(!REVERSE_PROXY)                                   
				 {
					 count_cycle1 = 0;
  					set(Port0_7);
					 flag = 1;
				 }
				 
				 if(count_cycle1>50 && y_dis >750 && (flag ==1))                      ////////////   count cycle added now (paras)
					 reset(P2_5);
				 
				 if(!FORWARD_PROXY)
				 {
					next = 1;
 		     }
				 
         control_pid_omni(-170*Degree_to_Rad,base_value);			 
	 }
	
	 without_turn_load_golden();

}

void without_turn_load_golden()
{
	next = 0;
	T1TC = 0;					
  prvvaluer = 0;
	y_dis = 0;
  th = 0;							
	int previous_value = y_dis;	
	int proxy_count = 0;
	//lcd('1');
		while(!next && !manual_override)
	  {
		pos();
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
			   
				    th = -170;
						base_value = 35;                                                                 
			   
					if(y_dis>800)
					{
						set(P2_2);
						set(P2_3);
						set(P2_4);
					}
				 
					control_pid_omni(th*Degree_to_Rad,base_value);
				 
					if(y_dis>GOLDEN_REVERSE_BREAK_POINT)
							next = 1;
	
	}	
		
	int zone_select =0;			
	count_cycle1 = 0;
	
	lim11=80+40;   //-5; //126; //80
  lim12=48-40;   //+5; //2;   //48
	lim21=208+40;  //-5; //253; //208
	lim22=176-40;
	
	double base_speed; 
	acc=0.007;
  y_dis = 0; 
	T1TC=0;
	prvvaluer=0;
  next=0;
  manual_override=0;
	int rack_proxy =0;
	bool fuck_flag = 0;
	
//	  lcd('2');		
		while(y_dis <(2500+ Golden_offset) && !manual_override)
		{
	    pos();
			
		  //============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
			if(count_cycle>7)
			{
				count_cycle=0;
				Ps2_val_update();
			}
		
			//============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
			if((temp_four!=400) && (temp_four!=360))
			{
			 hold_angle = ang;
			 manual_override=1;
			}
		
			base_speed = 40*exp(-(y_dis/1600));    //30->40    1500->1650->1500
			
			if (base_speed<19)    //23 to 22 just to solve the rack conditions //15->14
				base_speed=19;
			
			control_pid_omni(-162*Degree_to_Rad,base_speed);
		}
			
  //	lcd('3');
		
		while(!next && !manual_override && rack_proxy<3)
		{		
				if(Pin0_23 ==0)
						{
							fuck_flag = 1;
						}
				if(Pin0_23 == 1 && fuck_flag)
						{
							fuck_flag =0;
							rack_proxy++;
						}
				pos();
				//============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
				if(count_cycle>7)
						{
							count_cycle=0;
							Ps2_val_update();
						}

				//============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
				if((temp_four!=400) && (temp_four!=360))
					{
						hold_angle=ang;
						manual_override=1;
					}

				static double y_dis_temp=0;
				
				if((y_dis - y_dis_temp)<0)
					next=1;
									
				if(y_dis>2700)
					next=1;
				
				y_dis_temp=y_dis;

				control_pid_omni(3*Degree_to_Rad,23);  //-10
				
		}
			
		 next=0;
		 
		// lcd('4');
		  
		 while(!next && !manual_override && rack_proxy<3)
			{
				if(Pin0_23 ==0)
						{
							fuck_flag = 1;
						}
				if(Pin0_23 == 1 && fuck_flag )
						{
							fuck_flag =0;
							rack_proxy++;
						}
				pos();
				
  		  control_pid_omni(-170*Degree_to_Rad,4);
		  	 
				 //============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
		     if(count_cycle>7)
		     {
			       count_cycle=0;
		         Ps2_val_update();
	       }
	
     	   //============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
		     if((temp_four!=400) && (temp_four!=360))
		     {
		       	hold_angle=ang;
	          manual_override=1;
		     }
				 
				 if(rack_proxy == 3)
					 next = 1;
				 
				 //lcd('a');
				 //if(y_dis>3500)
           //next=1;				 
			
			}	
			
			without_turn_golden();
}

void without_turn_load_golden_aayush()                                ////// CHANGES (BY AYUSH)
{
	
	cls();
	lcd(123);
	
	// get all racks down.. if not already
	set(P2_1);
	set(P2_2);
	set(P2_3);
	set(P2_4);
	set(P2_5);
	set(P2_6);
	
	right_rack=0;
	centre_rack=0;
	left_rack=0;
		
	show_flag=4;
	
	// ==================================== Prepare for next loop ====================================================
	next=0;
	manual_override=0;
 
 
	lim11=80+5;               //these limits limit the speed of motors so that it doesn't beyond a range
	lim12=48-5;               //these limits limit the speed of motors so that it doesn't beyond a range
  lim21=208+5;              //these limits limit the speed of motors so that it doesn't beyond a range
	lim22=176-5;              //these limits limit the speed of motors so that it doesn't beyond a range
	
	lim11_lim=80+35;
  lim12_lim=48-35;
	lim21_lim=208+35;
	lim22_lim=176-35;
		
	icontrol=60;
		
	count_cycle1=0;
	
	y_dis = 0;
	T1TC=0;
	prvvaluer=0;


	while(y_dis<100 && !manual_override)  //////// THIS WHILE IS ADDED FOR CONTROLLING HEADING CHANGE at auto to rack start 
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
					control_pid_omni(-175*Degree_to_Rad,50);              //// SPEED=30 FOR 4sec
				}

			next=0;	
	while(!next && !manual_override)                       //////////////// IN THIS LOOP BOT MOVE TO DETECT START ZONE (proxy && encoder)
	{
		pos();
		if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		}
		
		 acc=0.009;
			//========================================== accelerate the robot by increasing the speed limit ======================================================
 
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

								th=-170; 
								
                if(y_dis<5000)                                                 //////distance to be checked
										control_pid_omni((-175*Degree_to_Rad),50);                   /////TH->5
									
				icontrol=60;
				count_cycle1=0;
							
								
				if(y_dis>5000)	
				{
				next=1; 
				}
	      
	}
	
		
	cls();
	lcd(1);	
	icontrol = 60;
	acc=0.007;
  next=0;   

  y_dis = 0;
	T1TC=0;
	prvvaluer=0;
	if(!manual_override)
		turned=1;
	
	//for proxy conditions
	int rack_proxy =0;
	bool fuck_flag = 0;
	
	lim11=80+24;               //these limits limit the speed of motors so that it doesn't beyond a range
	lim12=48-24;               //these limits limit the speed of motors so that it doesn't beyond a range
  lim21=208+24;              //these limits limit the speed of motors so that it doesn't beyond a range
	lim22=176-24;              //these limits limit the speed of motors so that it doesn't beyond a range
	
	count_cycle1 = 0;
	next = 0;
  int temp_speed=40;

	lim11=80+40;   //-5; //126; //80
  lim12=48-40;   //+5; //2;   //48
	lim21=208+40;  //-5; //253; //208
	lim22=176-40;
	
	
	//pmv's updates
	int proxy_count=0;
  double base_speed; 
	acc=0.007;
  y_dis = 0; 
	T1TC=0;
	prvvaluer=0;
  next=0;
  manual_override=0;
	rack_proxy =0;
	fuck_flag = 0;
	ang=0;
	
	
	cls(); 
	while(y_dis<150 && !manual_override )    //////   BOT STRTS TO MOVE TOWARDS RACK  FROM AUTO START ZONE
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
			hold_angle = ang;
			manual_override=1;
		}
    //=============================================================================================================================================	
		pos();
		control_pid_omni(-170*Degree_to_Rad,45);   //30
	}
	next = 0;
	rack_proxy=0;
	int temp_value = 0;
	bool psuedo_next = 0;
		while(!next && !psuedo_next && !manual_override)//(y_dis <5400 && !manual_override && rack_proxy<3)    //// just before entering rack   
		{
	    pos();
			
		  //============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
			if(count_cycle>7)
			{
				count_cycle=0;
				Ps2_val_update();
			}
		
			//============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
			if((temp_four!=400) && (temp_four!=360))
			{
				hold_angle = ang;
				//hold_angle=0;
				manual_override=1;
			}
		  if(y_dis<5400 && !manual_override )
			{
						base_speed = 22*exp(-(y_dis/1600));    //40>50
						th = -170;
												if (base_speed<14)    //23 to 22 just to solve the rack conditions //15->14
														base_speed=10;
			}
							
			if(y_dis>4500)
			{
				base_speed = 30;
				th = -10;
				
			}
			
	    if(Pin0_23 ==0)
	    {
		  fuck_flag = 1;
	    }
	    if(Pin0_23 == 1 && fuck_flag)
 	    {
		  fuck_flag =0;
	    rack_proxy++;
	    }
	
			if(rack_proxy==3)
					next = 1;
			
		  if(y_dis>5750)        //#backup
			{ 
			  next = 1;
			}
			
			if(temp_value<y_dis)       //velocity feedback;;
			{
				psuedo_next = 1;
			}
	
			control_pid_omni(th*Degree_to_Rad,base_speed);
	
			temp_value = y_dis;
		}
		while(psuedo_next)
		{
				pos();	
				base_value = 4;
				th = -170;	
												if(Pin0_23 ==0)
															{
																fuck_flag = 1;
															}
												if(Pin0_23 == 1 && fuck_flag)
															{
																fuck_flag =0;
																rack_proxy++;
															}
	                     control_pid_omni(th*Degree_to_Rad,base_speed);
			if(rack_proxy==3)
			{psuedo_next = 0;}
		}
		
		  next=0;
	  	count_cycle1=0;		
	    
/*
		while(count_cycle1<200 && !manual_override)	                      ////////////braking after proxy detected 3 times
	    { 
					control_pid_omni(-3*Degree_to_Rad,30);
						////============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
					if(count_cycle>7)
					{
						count_cycle=0;
						Ps2_val_update();
					}
				
					/////============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
					if((temp_four!=400) && (temp_four!=360))
					{
						hold_angle = ang;
						//hold_angle=0;
						manual_override=1;
					}
			}	      
*/
	without_turn_golden();

}


void without_turn_golden()
{
	lim11_lim=80+40;   //-5; //126; //80
  lim12_lim=48-40;   //+5; //2;   //48
	lim21_lim=208+40;  //-5; //253; //208
	lim22_lim=176-40;
	
//-----------------------------------------	
	
//======================================================= start to load (AYUSH)      =======================================================================================	
	//================================================= START TO LOAD (PMV)=======================================================
int rackproxy,temp=2000,proxy_count=0,rack_proxy =0;
double factor,base_speed=30; 
	int proxy_count_pmv = 0;
bool load_p=0,load_enc=0,decc=0,fuck_flag=0,pmv_proxy =0;
	acc=0.007;
  y_dis = 0; 
	T1TC=0;
	prvvaluer=0;
  next=0;
  manual_override=0;
	
			//////////////////////////for golden take the below part and comment or delete the above part
	lim11=80+2;   //-5; //126; //80
  lim12=48-2;   //+5; //2;   //48
	lim21=208+2;  //-5; //253; //208
	lim22=176-2;
//			cls();
	
//zone detection paramters
	y_dis = 0;
	T1TC = 0;
	prvvaluer = 0;			
	int zone_select =0;			
	
//  reset(P2_2);	
	while(!next && !manual_override)
	{
		//pos();
		
								//============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
								if(count_cycle>7)
								{
								count_cycle=0;
								Ps2_val_update();
								}

								pos();
								
								acc=0.007;
								
								//============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
								if((temp_four!=400) && (temp_four!=360))
								{
								hold_angle=0;
								manual_override=1;
								}

								//========================================== accelerate the robot by increasing the speed limit ======================================================
 
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

								th=-10; 
							
								if(y_dis<-3100)
								{											
								if(REVERSE_PROXY)
								zone_select++;
							  }
								if(zone_select >20)
									next = 1;
								if(y_dis<-500)
									reset(P2_4);
								
								if(y_dis<-800)
								{
									reset(P2_2);
									reset(P2_3);
									reset(P2_4);
								}
            		control_pid_omni((th*Degree_to_Rad),40);                          /////35 for golden
						}
  
	//initialisations based on next while loop
	next = 0;
	T1TC = 0;					
  prvvaluer = 0;
	y_dis = 0;
  th = 0;	
  //if(!manual_override)						
	//		{		
			//  reset(P2_3);
				//reset(P2_2);
//			}
		//int base_value = 40;
	int previous_value = y_dis;	
	proxy_count = 0;
						
	while(!next && !manual_override)
	{
		
		pos();
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
//				 if(y_dis<-2000)
	//				 next = 1;
			   if(y_dis<BREAK_DISTANCE_GOLDEN)       
			    {         
				        th = -10;
						base_value = 40;                                                                  //35 for golden
				      //  base_value = 30;
							//	base_value = 40*exp(-1*(y_dis/(0.5*TOTAL_DISTANCE)));
              //  if(base_value<21)
							//	base_value = 21;
			    }
	
				 if(y_dis>=BREAK_DISTANCE_GOLDEN)
					{
					//	base_value = 15;
									base_value = 35*exp(-1*((y_dis-BREAK_DISTANCE_GOLDEN)/(0.25*TOTAL_DISTANCE)));      //35 for golden
                if(base_value<14)
											base_value = 14;                                                         //15 for golden
			   
						th= -10; 
					}
			/*
					
					
      else
			{
				th = -170;
				if(y_dis > previous_value)
					next = 1;
				base_value = 20;
			}
			*/
		control_pid_omni(th*Degree_to_Rad,base_value);
		if(y_dis<-5000)
			next = 1;
	
	}	
   next = 0;
   base_value = 9;
	 th = -20;
	
  while(!next && !manual_override)
  {
	pos();
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
				 
         if(!FORWARD_PROXY)
				 {
					 th = -20;
					 base_value = 7;                                            //combo of 7& 9 for golden
				   set(P2_4);
				 }
				 if(!REVERSE_PROXY)
				 {  
					 th = -20;
					 base_value = 9;
					 reset(P2_4);
				 }
				 if(!Pin1_20)
				 {
					 th = -20;
					 next = 1;
				 }
	       control_pid_omni(th*Degree_to_Rad,base_value);
				
	}
  
  next = 0;
	int i = 4000;
	while(i-- && !manual_override)
	{
		control_pid_omni(-170*Degree_to_Rad,18);
	}
	th = -20;
  base_value = 7;
	while(!next && !manual_override)
  {
	pos();
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
	
				 if(!Pin1_20)
				 {
					 th = -20;
					 base_value = 7;
					 //reset(Port0_7);
					 set(P2_4);
					// next = 1;
				 }
				 
				 if(!REVERSE_PROXY)
				 {
					 next = 1;
				 }
         control_pid_omni(th*Degree_to_Rad,base_value);
				 
	}
	//i = 300;
	
	while(REVERSE_PROXY && !manual_override)
	{
		       control_pid_omni(-170*Degree_to_Rad,8);
	}
	
	//next while loop is to be removed if further more is to be applied
	while(!manual_override)
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
				 reset(P2_4);
//				if(!manual_override)						
	//		{		
		//	  set(P2_3);
			//	set(P2_2);
			//}
	 
	}
}



void towards_rack_golden_auto(void)
{
	// get all racks down.. if not already
	set(P2_1);
	set(P2_2);
	set(P2_3);
	set(P2_4);
	set(P2_5);
	set(P2_6);
	
	right_rack=0;
	centre_rack=0;
	left_rack=0;
	
	show_flag=4;
	
	// ==================================== Prepare for next loop ====================================================
	next=0;
	manual_override=0;
  		
	lim11=80+5;               //these limits limit the speed of motors so that it doesn't beyond a range
	lim12=48-5;               //these limits limit the speed of motors so that it doesn't beyond a range
  lim21=208+5;              //these limits limit the speed of motors so that it doesn't beyond a range
	lim22=176-5;              //these limits limit the speed of motors so that it doesn't beyond a range
	
	lim11_lim=80+35;
  lim12_lim=48-35;
	lim21_lim=208+35;
	lim22_lim=176-35;
		
	icontrol=60;
		
	count_cycle1=0;
	
	y_dis = 0;
	T1TC=0;
	prvvaluer=0;
		
	while(y_dis>-200 && !manual_override)
	{
		//================================================Update Ps2 Values every 7ms=====================================================
		if(count_cycle>7)
		{
			count_cycle=0;
			Ps2_val_update();
		}
				
		//================================================Check for any sign of manual override===========================================
		if((temp_four!=400 && temp_four!=360))
		{		 	
			manual_override=1;
		}		
				
		pos();
		
		control_pid_omni(0*Degree_to_Rad,10);
	}
	
	while(y_dis>-300 && !manual_override)
	{
		//================================================Update Ps2 Values every 7ms=====================================================
		if(count_cycle>7)
		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		//================================================Check for any sign of manual override===========================================
		if((temp_four!=400 && temp_four!=360))
		{		 	
			manual_override=1;
		}
		
		pos();
		control_pid_omni(-45*Degree_to_Rad,15);
	}
	
	icontrol = 60;
	acc=0.007;
	
	while(ang<-2 && !manual_override)
	{
		//================================================Update Ps2 Values every 7ms=====================================================
		if(count_cycle>7)
		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		//================================================Check for any sign of manual override===========================================
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
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
			
			
		hold_angle += 0.02;		
		
		if(hold_angle>0)
			hold_angle=0;
			
		if(ang>hold_angle)
			hold_angle=ang;
		
		control_pid_omni((177-ang)*Degree_to_Rad,30);
	}

//apply pmv's conditions from here i.e. comment below section if you want to apply pmv's condition	
//jump to 5181 for the conditions...

	y_dis = 0;
	T1TC=0;
	prvvaluer=0;
	if(!manual_override)
		turned=1;
	
	lim11=80+24;               //these limits limit the speed of motors so that it doesn't beyond a range
	lim12=48-24;               //these limits limit the speed of motors so that it doesn't beyond a range
  lim21=208+24;              //these limits limit the speed of motors so that it doesn't beyond a range
	lim22=176-24;              //these limits limit the speed of motors so that it doesn't beyond a range
	
	count_cycle1 = 0;
	
	while(y_dis<2000 && !manual_override)
	{
		//================================================Update Ps2 Values every 7ms=====================================================
		if(count_cycle>7)
		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		//================================================Check for any sign of manual override===========================================
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=0;
			manual_override=1;
		}
		
		if(count_cycle1>1000)
			next=1;
		
		pos();
		control_pid_omni(178*Degree_to_Rad,40);
	}
	
	if(manual_override)
	{
		cls();
		lcd((char*)"manual");
	}
while(1)
{
	pos();
	lcd(y_dis);
	control_pid_omni(40,5);
	cls();
}

return;
	//=========================================================== End of:- towards rack golden ==================================================================
}
//==============================================================================================================================================
void manual_start_to_load()
{
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>==	
//=======================FUNCTION FOR MAUAL TASK_1==================	
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>==	
//------->>>>DONT FORGET TO REFRESH THE ENCODER BEFORE CALLING<<<<<<<=	
//------------------------------------------------------------------  
manual_override =0;
 temp_tick_enc=0,p_temp_tick_enc=0,diff_tick=0,diff_max=0;
 target_speed = 20;
double base_value=50,heading_1,brake_normal=(-5000);
bool non_return=0,manual_slowing=0,slowing=0;
//int top_gate =0;
count_cycle=0;
heading_1=0;
	
//==============================================================================
//--------------------GO FOR BALL LOADING----------------------------------
/*
step_1: go towards rack with speed of 50 for 2 meter 
step_2: deccelerate for 1 meter 
step_3: load the balls while mainataining the speed of 10
step_4: exit on proxy condition with enocder backup
*/
//==============================================================================
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>STEP_1

lim11=80+20;//+5;
lim12=48-20;//-5;
lim21=208+20;//+5;
lim22=176-20;//-5;
	
lim11_lim=80+40;   //-5; //126; //80
lim12_lim=48-40;   //+5; //2;   //48
lim21_lim=208+40;  //-5; //253; //208
lim22_lim=176-40;
acc=0.008;                                       ///////////  0.005(orignal)
double temp_diff;


while(y_dis<80)
{	
	pos();
	control_pid_omni((-160)*Degree_to_Rad,30);                      ////// -168

		if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	//hold_angle=ang;
			manual_override=1;
		}
}

/*
lim11=80+40;//+5;
lim12=48-40;//-5;
lim21=208+40;//+5;
lim22=176-40;//-5;
*/


proxy_safety=0;
base_value=30;
heading_1=(-160*Degree_to_Rad);
while((y_dis<2100)&&(y_dis>=0) && !manual_override && !proxy_safety)                      ////////////////1850(ORIGNAL)
{   
		if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		manual_override=1;
		}
//--------ACCELERATION--------------------	
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
//------------------------------------	

///////////////  =================			
			temp_val = ((((double)4096-((unsigned int)((AD0DR5&0x80000000)?((AD0DR5&0x0000FFF0)>>4):temp_val)))/4096)*3000);
      temp_diff = (temp_val-2300);	  
		     
    if(temp_val>2260 && temp_val<2340) 				
           {
			     		heading_1= (-173)*Degree_to_Rad; 
  		     }	
					 
		else if(temp_val>2200 && temp_val<2280) 	
       {
			  temp_val = (temp_val-2300);                          /////// 850  assumed
			 heading_1 = ((-1)*(173*Degree_to_Rad) - (temp_val*0.03)*Degree_to_Rad); 	 
   		 }
			 
		else if(temp_val>2320 && temp_val<2500)
		{
		 temp_diff = (temp_val-2410);	  
		 heading_1 = ((-1)*(173*Degree_to_Rad) - (temp_val*0.003)*Degree_to_Rad); 	 
		}		
////////// .........................		

	//heading_1 -= ang*Degree_to_Rad;	
	
  if(heading_1 < -173*Degree_to_Rad)
		heading_1=(-173)*Degree_to_Rad;

	//heading_1=(-170)*Degree_to_Rad;  //extra line che. joieye tyare udai dejo
	
	if (!Pin0_23)
	{
		proxy_safety=1;
	}
 pos();

	   if(y_dis>500)
		 {acc=0.006;
			heading_1=(-170*Degree_to_Rad);
		  base_value=30;
		 } 
		 else 
		 {acc=0.006;
			heading_1=(-160*Degree_to_Rad); 
		  base_value=30; 
		 }

	control_pid_omni(heading_1,base_value);                      ////// -168
	
}

float temp_dis=0,brake_speed=55;
next=0;

while((y_dis>=0) && !manual_override && !next)
{

//temp_dis=y_dis-1550;
//brake_speed=brake_speed*exp(-(temp_dis/350));

	pos();
	  
	 if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	//hold_angle=ang;
			manual_override=1;
		}
		
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
		 heading_1 = ((-1)*(160*Degree_to_Rad) - (temp_val*0.003)*Degree_to_Rad); 	 
		}		
////////// .........................		
	
control_pid_omni((-90)*Degree_to_Rad,10);     //(-90,10)(ORIGNAL)
if (!Pin0_23)
	{
		next=1;
	}
	
} 



//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>STEP_2
/*
while(y_dis<2000)//&&(diff_tick>=0))
{
	pos();
	control_pid_omni(10*Degree_to_Rad,20);     //-15
}
*/
/*
int i=00;
while(i-- && !manual_override)
{
	control_pid_omni((-100)*Degree_to_Rad,20);
}
*/
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>STEP_3

next=0;
target_speed = 40;                               ///set the target speed here which you want to achieve 
count_cycle=0;
count_cycle1=0;
/*
while(!next && !manual_override && (y_dis>=0) && count_cycle1<800)
{
		if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		}
		
	if (!Pin0_23)
	{
		next=1;
	}
	
	pos();

	diff_tick = y_dis - temp_tick_enc;

	if (diff_tick > diff_max)
	{
		diff_max = diff_tick;
	}

//	heading_1  = (-1)*(175*Degree_to_Rad);
	if ((diff_max - target_speed)>0)
		 base_value -= (diff_max - target_speed)*0.7;
	else 
		 base_value -= (diff_max - target_speed)*1;
	 
	if (base_value>20)
	{
		base_value = 20;
	}
	else if (base_value < 12)
	{
		base_value = 12;
	}
//========================	

	///////////////  =================			
			temp_val = ((((double)4096-((unsigned int)((AD0DR5&0x80000000)?((AD0DR5&0x0000FFF0)>>4):temp_val)))/4096)*3000);
//      temp_diff = (temp_val-2300);	  
		     
    if(temp_val>2260 && temp_val<2340) 				
           {
			     		heading_1= (-175)*Degree_to_Rad; 
  		     }	
					 
		else if(temp_val>2200 && temp_val<2280) 	
       {
			  temp_val = (temp_val-2300);                          /////// 850  assumed
			 heading_1 = ((-1)*(175*Degree_to_Rad) - (temp_val*0.03)*Degree_to_Rad); 	 
   		 }
			 
		else if(temp_val>2320 && temp_val<2500)
		{
		 temp_diff = (temp_val-2410);	  
		 heading_1 = ((-1)*(175*Degree_to_Rad) - (temp_val*0.003)*Degree_to_Rad); 	 
		}		
////////// .........................		


	base_value =12;
	control_pid_omni(heading_1,base_value);
	
	temp_tick_enc = y_dis;

}
*/




//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>STEP_4
//cls();
//lcd((char *)"step_4");
next=0;
temp_tick_enc=0;
diff_tick=0;
y_dis=0;
prvvaluel=0;
prvvaluer=0;
T0TC=0;
T1TC=0;


while(!next && !manual_override)
{
			if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
			manual_override=1;
		}
	
	pos();
	diff_tick = y_dis - temp_tick_enc;
	control_pid_omni((-10)*Degree_to_Rad,20);                    /////// (-10,20)
	if (diff_tick < (0))
	{
		next=1;
	}
	temp_tick_enc = y_dis;

}

//=================================================================================
//------------------------return after loading the balls---------------------------
//==================================================================================
temp_tick_enc=0;
diff_tick=0;
lim11=80+5;//+5;
lim12=48-5;//-5;
lim21=208+5;//+5;
lim22=176-5;//-5;
	
lim11_lim=80+40;   //-5; //126; //80
lim12_lim=48-40;   //+5; //2;   //48
lim21_lim=208+40;  //-5; //253; //208
lim22_lim=176-40;
acc=0.0082;
bool non_return_tz1=0;
heading_1=((-1)*(10*Degree_to_Rad));                // -10
next=0;
target_speed = 25;
count_cycle2=0;
if(!manual_override)
set(P2_5);
bool lower_gate_2 =0;
 temp_dis=0,brake_speed=55;
int dist_temp=0; 
int trial_flag=0;
hold_angle=0;

while(!next && !manual_override)
{
	pos();
	diff_tick = ((-1)*y_dis) - temp_tick_enc;

//===================================================================	
		if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
			manual_override=1;
    }
//==========================================================================
//--------------------CONDITIONS FOR TASK TO BE PERFOMED -------------------	
	if (y_dis < (-1030) && !lower_gate_2)                     //turn the rack and actuate it 
	{
		if(!manual_override)
		{
		RACK_TURNED;
		reset(P2_3);
		reset(P2_4);
		}
		
	}
	
	if(ps_down)
	{
		manual_slowing=1;
	}
	
	if(y_dis< (-7200) && top_gate==0)
	{
		target_speed=10;
		lower_gate_2= 1;
		top_gate = 1;
		count_cycle2=0;
	 //  reset(P2_5);
	}

	
 	if((!FORWARD_PROXY) && (top_gate == 1) && (count_cycle2>400))                                   ////////////////Pin1_23
	{

		if(!manual_override)
		{
		//set(P2_4);
		count_cycle2 = 0;
		top_gate = 2;
		//reset(P2_5);
		dist_temp=y_dis;	
		tz2_actuation=y_dis;	
	  }
	
	}
	
	if ((top_gate==2)&&(y_dis<(dist_temp-50)))     
	{   
	set(P2_4);
	reset(P2_5);
	}
	
	
	if ((top_gate==2)&&(count_cycle2>170))
	{
		base_value = 10;
		target_speed = 10;
	}

  	if(y_dis<(dist_temp-590)&& top_gate>=2)     ////  590 (!!!!working ORIGNAL)
	  {
		   if(!manual_override)
		     {
		      set(P2_5);
	      	top_gate = 3;
	       }
		
				 
			   if(y_dis<(-9500)&& top_gate>=2)                                                               ///////// if(y_dis<(-9470)&& top_gate>=2)  /// 9470(orignal,working)
		     {
		 	    top_gate = 3;
	         if(!manual_override)
					 {set(P2_3);}                                                /////////////reset(P2_4);
		     }
				 
				 if(y_dis<(-10400)&& top_gate>=2)                                                               ///////// if(y_dis<(-9470)&& top_gate>=2)  /// 9470(orignal,working)
		     {
		 	    top_gate = 3;
					 
	         reset(P2_4);                                                /////////////reset(P2_4);
		     }
				 
		
		}	
		
	      //if(y_dis<(-9800)&& top_gate==3)
	      //{set(P2_3);}
		
		
		
	  if(y_dis<(-9300) && top_gate ==3)                    //////10200
	  { 
			base_value=11;
		  heading_1=(-4)*Degree_to_Rad;
		}
  if(y_dis<(-10900))
	{
	if(!manual_override)
	{reset(P2_3);}
	}	
		
	if (y_dis < (-11000))                   //11100 >> 10900 >> 10000
	{
	next=1;
	//reset(P2_3);
	}
		
//===========================================================================	
//--------ACCELERATION--------------------	
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
//------------------------------------		
	

	
//======SPEED CONTROL=====	
	if ((y_dis > brake_normal)&&(slowing==0))                        ////  550(new orignal)  >> 5600 >> 5800                  
	{ 
		base_value = 60;
	  brake_speed=base_value;
	 	heading_1=(-8)*Degree_to_Rad;
    
    if(y_dis>(-500))		
		{
		kp=1.5;
	  }
		else 
    kp=0.9;		 
		if(manual_slowing==1)
		{
			slowing=1;
		}
		
	}

	else 
	{
	
		if((diff_tick > 11)&&(non_return_tz1==0))
		{
			base_value=40;
			heading_1 = (-90)*Degree_to_Rad;
		}
		else
		{
			non_return_tz1=1;
			 if(ps_down)
       {base_value=7;}	
			 
      else
      {
			
			base_value=8;                     //////////10
			
			if(diff_tick<11)         ////////13
			{base_value+=1;}
			
      if(diff_tick>12)         ////////14
			{base_value-=1;}
			
       if(base_value>9)			
			 {base_value=8;}
		 }
			heading_1 = (-20)*Degree_to_Rad;
		
      		
		
		}

		
/*
		if (y_dis > (-7400)&&(non_return==0))                                 /// 6600(orignal)        // tick limit , use this reference in the equation 
		{  
     
		
		  
			
		}
		else  
		{		
			non_return=1;
			//heading_1  = (-1)*(23*Degree_to_Rad);                      /////// 19:02   -18>> -23
			if ((diff_max - target_speed)>0)
		      base_value -= (diff_max - target_speed)*0.7;
			else 
				  base_value -= (diff_max - target_speed)*1;

/////=========  			
	
			//===================== Get SHARP value and then smoothen it using a moving average filter. Set boundaries on the angle so obtained =======================
      		temp_val = ((((double)4096-((unsigned int)((AD0DR5&0x80000000)?((AD0DR5&0x0000FFF0)>>4):temp_val)))/4096)*3000);
      temp_diff = (temp_val-2300);	  
       
    if(temp_val>2260 && temp_val<2340) 				
      {
			heading_1= (-29)*Degree_to_Rad; 
  		}	
					 
		else if(temp_val>2200 && temp_val<2280) 	
       {
			 temp_val = (temp_val-2300);                          /////// 850  assumed
			 heading_1 = ((-1)*(29*Degree_to_Rad) - (temp_val*0.03)*Degree_to_Rad); 	 
   		 }
			 
		else if(temp_val>2320 && temp_val<2500)
		  {
		  temp_diff = (temp_val-2410);	  
		  heading_1 = ((-1)*(29*Degree_to_Rad) - (temp_val*0.003)*Degree_to_Rad); 	 
		  }		
			base_value=9;                                   ////////////// 7
		  heading_1=(-20)*Degree_to_Rad;
	 /////======   
		
		} 
		*/
	}
	
	if (base_value>52)
	{
		if (non_return==0)
		  base_value=52;
		else if (non_return==1)
			base_value = 8;                                   
	}
	
	else if (base_value < 3)            /// 8>>15 ,,,,,  7>>5>>4 
	{
		base_value = 3;                   /// 8>>15,,,,,   7>>5>>4
	}

	//========================	
	if(y_dis<-9600)
		base_value = 11;    //11
	
	control_pid_omni(heading_1,base_value);

	temp_tick_enc = (-1*y_dis);

}
	kp=2;												     //Kp for Omni
	kd=0.6; 	  										 //Kd for Omni
	ki=0;			  										 //Ki for omni

//----------------------------------------------------------------------------
//=============================call to the second function==========================
 manual_tz1_to_golden();
}
//=====================================================================================================================================
//------------------------------MANUAL TZ1 TO GOLDEN---------------------------------------------------------------------------------
//=====================================================================================================================================
void manual_tz1_to_golden()
{
	rule_flag=0;
	if(!manual_override)
	{reset(P2_4);
	set(P2_5);
	}
	 temp_tick_enc=0,p_temp_tick_enc=0,diff_tick=0,diff_max=0;
   target_speed = 29;
  double base_value=15,heading_1;
  bool non_return=0;
	//int	bottom_gate=0;
	int top_gate=0;
  count_cycle2=0;
  heading_1=(-175)*Degree_to_Rad;
	next=0;
	bool temp_flag=0;
  int fuck_flag=0;                                        ///////////  flag declARED NOW             
	int actuation_distance=0; 
		
	while(!next && !manual_override)
	{
		if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
	
	if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		
		}
//-----------------conditions for actuating of gate (upper and lower)--------------------		
		//if((y_dis > (-10400))&&(bottom_gate==0))           ///////  9700 >> 9600 >>   9100 >>9300 
     		 
 	  if(!FORWARD_PROXY && bottom_gate==0)
    {
    		set(P2_4);
    		count_cycle2=0;
        bottom_gate=1;  			 
   	}
		/*
		if((y_dis>(tz2_actuation-680))&&((bottom_gate==0)))            //// (700 ,  22-7-18)
		 { 
        set(P2_4);
    		count_cycle2=0;
        bottom_gate=1;  			 
		 }
		 
		 */
	  
		if(count_cycle2>1 && (bottom_gate==1))             //////   100 >>> 200   
		 {
			reset(P2_5);
			count_cycle2=0; 
			bottom_gate=2; 
		 }			
		
		 if(count_cycle2>850 && (bottom_gate==2))              /// 550                          //////   -8900 >> -8700 
		 {
			 next=1;
			 set(P2_5);
	 	 }
		 
		 if(y_dis<(-9700))
		 {base_value=5;}
		 //----------------------------------------------------------------------------------------		
	  pos();
	  diff_tick = temp_tick_enc - ((-1)*y_dis);
	  if (diff_tick > diff_max)
	   {
		   diff_max = diff_tick;
	   }
	  heading_1  = (-1)*(173*Degree_to_Rad);
	  if ((diff_max - target_speed)>0)
	  	 base_value -= (diff_max - target_speed)*0.7;
  	else 
	 	  base_value -= (diff_max - target_speed)*1;
	 
 
	  if (base_value>20)
	   {
		   base_value = 20;
     }
	  else if (base_value<8)    ////// 7>>8
	   {
		   base_value = 8;       /////// 7>>8
	   }
		 /////////////     speed control added (12-06-18 ++ 11:10) 
     /* 		
		 if(y_dis<(-10000))
		 {base_value=14;}
		  
		 else 
		 {base_value=10;}
		 */ 
		 //////+======
			 
//========================	
	  base_value = 10;                                  ////////////////// base_value = 12 
		control_pid_omni(heading_1,base_value);
 	  temp_tick_enc = ((-1)*y_dis);
		 
	}

///	limits changed here 

//-------------------------GO FOR GOLDEN BALL LOAD------------------------------------------------	
/*
//golden ball load as per preious conditions 
	int previous_value = y_dis;	
	next = 0;
	int proxy_count = 0;
		while(!next && !manual_override)
	  {
		pos();
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
			   
				    th = -170;
						base_value = 40;                                                                 
			   
					if(y_dis>-8100)
					{
						set(P2_5);
						set(P2_2);
						set(P2_3);
						set(P2_4);
					}
				 
					control_pid_omni(th*Degree_to_Rad,base_value);
				 
					if(y_dis>-4900)
							next = 1;
	}	
		
	int zone_select =0;			
	count_cycle1 = 0;
	
	lim11=80+40;   //-5; //126; //80
  lim12=48-40;   //+5; //2;   //48
	lim21=208+40;  //-5; //253; //208
	lim22=176-40;
	
	double base_speed; 
	acc=0.007;
  next=0;
	int rack_proxy =0;
	bool fuck_flag = 0;
			
		while((y_dis >(-1800)) && !manual_override)
		{
	    pos();
			
		  //============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
			if(count_cycle>7)
			{
				count_cycle=0;
				Ps2_val_update();
			}
		
			//============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
			if((temp_four!=400) && (temp_four!=360))
			{
			 hold_angle = ang;
			 manual_override=1;
			}
		
			base_speed = 40*exp(-((y_dis+4900)/3500));    //30->40    1500->1650->1500
			
			if (base_speed<19)    //23 to 22 just to solve the rack conditions //15->14
				base_speed=19;
			
			control_pid_omni(-162*Degree_to_Rad,base_speed);
		}
			
		while(!next && !manual_override && rack_proxy<2)
		{		
				if(Pin0_23 ==0)
						{
							fuck_flag = 1;
						}
				if(Pin0_23 == 1 && fuck_flag)
						{
							fuck_flag =0;
							rack_proxy++;
						}
				pos();
				//============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
				if(count_cycle>7)
						{
							count_cycle=0;
							Ps2_val_update();
						}

				//============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
				if((temp_four!=400) && (temp_four!=360))
					{
						hold_angle=ang;
						manual_override=1;
					}

				static double y_dis_temp=0;
				
				if((y_dis - y_dis_temp)<0)
					next=1;
									
				if(y_dis>-300)
					next=1;
				
				y_dis_temp=y_dis;

				control_pid_omni(3*Degree_to_Rad,23);  //-10
				
		}
			
		 next=0;
		 
		 while(!next && !manual_override && rack_proxy<2)
			{
				if(Pin0_23 ==0)
						{
							fuck_flag = 1;
						}
				if(Pin0_23 == 1 && fuck_flag )
						{
							fuck_flag =0;
							rack_proxy++;
						}
				pos();
				
  		  control_pid_omni(-170*Degree_to_Rad,6);
		  	 
				 //============================= Take input of Ps2 every 7 seconds. Optimal time obtained by trial and error method ========================================
		     if(count_cycle>7)
		     {
			       count_cycle=0;
		         Ps2_val_update();
	       }
	
     	   //============= If operator tries to override it, exit the loop and directly go to Ps2_drive() condition by skipping all the following loops ==============
		     if((temp_four!=400) && (temp_four!=360))
		     {
		       	hold_angle=ang;
	          manual_override=1;
		     }
				 
				 if(rack_proxy == 2)
					next = 1;
		
			}	
*/				
	
   //tak's golden ball load	
	//=================reset the variables to their initial values====================================	
	temp_tick_enc=0;diff_max=0;diff_tick=0;
	next=0;target_speed=12;
	float temp_dis=0 , brake_speed=0;
	
	lim11=80+20;//+5;
  lim12=48-20;//-5;
  lim21=208+20;//+5;
  lim22=176-20;//-5;
	
  lim11_lim=80+40;   //-5; //126; //80 
  lim12_lim=48-40;   //+5; //2;   //48
  lim21_lim=208+40;  //-5; //253; //208
  lim22_lim=176-40;
  acc=0.015;        /////   0.0075 >>>  0.008
	if(!manual_override)
	{
	RACK_NORMAL;
	set(P2_3);
  set(P2_4);
	}
		//heading_1 = (-145)*Degree_to_Rad;                               
	double temp_diff,temp_distance=0;
	pos();
  temp_distance=y_dis+200;
	
	
	
	while(!next && !manual_override)
	{
			if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		}
				
		pos();
	  diff_tick = temp_tick_enc - ((-1)*y_dis);
		
		if (diff_tick > diff_max)
	   {
		   diff_max = diff_tick;
	   }
     		 
		 if (y_dis < (START_BRAKE_LOAD_GOLDEN))                               ///// 2700 > 2600
		{
			base_value = 60;
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
//------------------------------------		
		
			pos();
		  ///if(y_dis<(temp_distance))
			 /// { heading_1=180*Degree_to_Rad;}
	
			 heading_1 = (-1)*(160*Degree_to_Rad); ////////////// 165 
////////// .........................		
	  }
			
		else 
		{
			heading_1 = ((-1)*160)*Degree_to_Rad;
			base_value = 11;
		}
		
		if(y_dis>START_BRAKE_LOAD_GOLDEN && !rule_flag)
		{     
     while(!rule_flag)	
     {
		 	if(count_cycle>7)    //////   ps2 value update    
 		   {
			 count_cycle=0;
			 Ps2_val_update();
		   }
		
		  if((temp_four!=400 && temp_four!=360))
		  {
		 	hold_angle=ang;
			manual_override=1;
		  }
	     
			if(ps_l1) 
			{rule_flag=1;}
		
     control_pid_omni(40,5);			
      			
		 }
			
		}
		
		
		
		
		
		/*
		else 
		{
		  if (y_dis < (STOP_BRAKE_LOAD_GOLDEN))                         /////  1600 >> 1500 >> 1400
			{
        pos();
				// 	heading_1 = (-1)*(17*Degree_to_Rad);       /////  -12 >> -17 >> -10   { 5-6-18>> 11:15  }
//		    base_value = 20;                           
       
      // base_value=60*exp(-(y_dis+3100)/600);  
        base_value=10;
	///////////////  =================			

			temp_val = ((((double)4096-((unsigned int)((AD0DR5&0x80000000)?((AD0DR5&0x0000FFF0)>>4):temp_val)))/4096)*3000);
      temp_diff = (temp_val-2300);	  
		     
    if(temp_val>2260 && temp_val<2340) 				
           {
			     		heading_1= (-165)*Degree_to_Rad; 
  		     }	
					 
		 else if(temp_val>2200 && temp_val<2280) 	
       {
			  temp_val = (temp_val-2300);                          /////// 850  assumed
			 heading_1 = ((-1)*(165*Degree_to_Rad) - (temp_val*0.03)*Degree_to_Rad); 	 
   		 }
			 
		 else if(temp_val>2320 && temp_val<2500)
		   {
		 temp_diff = (temp_val-2410);	  
		 heading_1 = ((-1)*(165*Degree_to_Rad) - (temp_val*0.003)*Degree_to_Rad); 	 
		   }		
			 heading_1 = (-1)*(160*Degree_to_Rad);

////////// .........................		

			}
			
			else 
			{
			 non_return=1;
			// heading_1  = (-1)*(145*Degree_to_Rad);        /////// { 5-6-18>> 11:15  }   
			 if ((diff_max - target_speed)>0)
		      base_value -= (diff_max - target_speed)*0.7;
			 else 
				  base_value -= (diff_max - target_speed)*1;	
      
      if (base_value>25)
			{
				base_value=25;              //7>>25
			}				
			else if (base_value<7)
			{
				base_value=8;
			}
			base_value=10;

///////////////  =================			
			temp_val = ((((double)4096-((unsigned int)((AD0DR5&0x80000000)?((AD0DR5&0x0000FFF0)>>4):temp_val)))/4096)*3000);
      temp_diff = (temp_val-2300);	  
		     
    if(temp_val>2260 && temp_val<2340) 				
           {
			     		heading_1= (-165)*Degree_to_Rad; 
  		     }	
					 
		else if(temp_val>2200 && temp_val<2280) 	
       {
			  temp_val = (temp_val-2300);                          /////// 850  assumed
			 heading_1 = ((-1)*(165*Degree_to_Rad) - (temp_val*0.03)*Degree_to_Rad); 	 
   		 }
			 
		else if(temp_val>2320 && temp_val<2500)
		{
		 temp_diff = (temp_val-2410);	  
		 heading_1 = ((-1)*(165*Degree_to_Rad) - (temp_val*0.003)*Degree_to_Rad); 	 
		}	
		heading_1 = (-1)*(173*Degree_to_Rad);
////////// .........................		
			}
		 //base_value=9;
		}*/
		control_pid_omni(heading_1,base_value);
		temp_tick_enc = ((-1)*y_dis);
//===================================   EXIT  =================		
		if ((y_dis>300)&&(!Pin0_23))
		{
			next=1;
		//	fuck_flag=1;
		}
//=============================================================		

	}

//---------------------------------------------------------------	
//==========================         BRAKE           ====================================	
//-------------------------------------------------------------
next=0;
temp_tick_enc=0;
diff_tick=0;
y_dis=0;
prvvaluel=0;
prvvaluer=0;
T0TC=0;
T1TC=0;
count_cycle2 = 0;

	
while(!next && !manual_override)
{

			if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		}
  
	pos();
	diff_tick = y_dis - temp_tick_enc;

	control_pid_omni((-5)*Degree_to_Rad,25);                          ////////////// -10 >> -17
			 
			 if(diff_tick < (0))//(count_cycle2>250) 										//(diff_tick < (0))
	     {
		    next=1;
	     }
	temp_tick_enc = y_dis;
}


//=================================================================================
//                            GO FOR GOLDEN BALL TRANSFER 
//=================================================================================
flag_TZ3 = 1;
bottom_gate=0;
top_gate=0;
temp_tick_enc=0;
diff_tick=0;
lim11=80+10;//+5;
lim12=48-10;//-5;
lim21=208+10;//+5;
lim22=176-10;//-5;
	
lim11_lim=80+40;   //-5; //126; //80
lim12_lim=48-40;   //+5; //2;   //48
lim21_lim=208+40;  //-5; //253; //208
lim22_lim=176-40;
acc=0.009;        ///0.0075 >>  0.009   
next=0;
target_speed = 60;                                               ///////////////////   25 >> 20 >> 
count_cycle2=0;
temp_dis=0;
brake_speed=0;
int top_gate_2 = 0;
float start_speed =20,temp_start_dis=0,no_reverse=0; 
bool init_condt=0;
next = 0;
heading_1=(-2)*Degree_to_Rad;                             ///////////////////  changed 5-6-18 2:08
bool non_return_tz3=0,slow=0;
int manual_slow=0;
double set_point=13,brake_start=(-5750);                       /// 13 , (-6250) ///  6050

while(!next && !manual_override)
{
	
	  pos(); 
	 diff_tick = ((-1)*y_dis) - temp_tick_enc;  
	 /* 	
	if (diff_tick > diff_max)
	   {
	    diff_max = diff_tick;
	   }
  */
	
	if((no_reverse==1)&&(diff_tick < 3))
		{
			base_value=6;
			heading_1 = (-10)*Degree_to_Rad;
		}
		if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		}

//==========================================================================
//--------------------CONDITIONS FOR TASK TO BE PERFOMED -------------------	
	  /*
		if (y_dis < (-200))
		{
			lim11=80+40;   //-5; //126; //80
      lim12=48-40;   //+5; //2;   //48
      lim21=208+40;  //-5; //253; //208
      lim22=176-40;
		}
		*/
		
		if ((ps_down)&&(manual_slow==0))
	{
	   manual_slow=1;
	}
		
		
		if(y_dis<(-12000))
		{
		 next=1;
		}	
		
	if (y_dis < (-1530) && !init_condt)                     //turn the rack and actuate it  (1830)
	{
		if(!manual_override)
		{
		RACK_TURNED;
		reset(P2_3);
		reset(P2_4);
	  } 
			//	heading_1 = (-17)*Degree_to_Rad; ///////////////////  changed 5-6-18 2:08  
    init_condt = 1;		
		//acc=0.003;
	}
	
	
  //actuations as per laser
	if((y_dis<(-8000)) && (top_gate_2==0))
	{
	top_gate_2 =1;
	}
		
  if((!FORWARD_PROXY) && (top_gate_2 ==1) )                      	//////if((Pin1_20==1) && (top_gate_2 ==1))            ////////// (Pin1_20==1)  
	{
		//set(P2_4);
		count_cycle2 = 0;
		top_gate_2 =3;
	  //reset(P2_5);
	  actuation_distance = y_dis;
	}
	
	if(y_dis<(actuation_distance-20) && top_gate_2==3)
	{
	 set(P2_4);
	 reset(P2_5);	
	}
	if(count_cycle2>10  && top_gate_2==3)
	 {
	  heading_1=(-5)*Degree_to_Rad;
		base_value=8;	  
	 } 
	
	 if((y_dis<(actuation_distance-610)) && (top_gate_2 ==3))              //////710 >> 560 >> 360                       //if( (count_cycle2 > 330)  && (top_gate_2 ==3))              //////// >> 300
	 {
		 set(P2_5);
     top_gate=4;
		 count_cycle2=0;
	 }  
	 
	 
   if(top_gate==4 && count_cycle2>400) 
	 {  base_value= 2;
	   heading_1=(-23)*Degree_to_Rad;
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
//------------------------------------		
	
	//////////======SPEED CONTROL=====	
	
	if ((y_dis > brake_start)&&(slow==0))                      /////   6200(ORIGNAL)
	{
		//base_value = 60;//50  >>> 55 
    brake_speed=60;	//50  >>> 55     
    
		
		
		if(y_dis>(-100))
		{
			base_value=30;
		 // heading_1=(-)*Degree_to_Rad;
		}
		
		else
		base_value=60;	
		if (manual_slow==1)
		{
			slow=1;
		}
		
	}
		
	else
	{  
   if((diff_tick < set_point)&&(non_return_tz3==0))
	 {
		 heading_1 = (-90)*Degree_to_Rad;
		 base_value = 40;
		 
	 }
	 
	 
	 else
	 {
		 
		 non_return_tz3=1;
		// base_value = 8;
		  if(ps_down)
   {base_value=6;}	
	 else{
		 
		 if(diff_tick<set_point + 1)               /////////////  
		 {base_value+=1;}
			
     if(diff_tick>set_point+3)                /////////////
		 {base_value-=1;}
			
     if(base_value>set_point-3)			          ////////////
		 {base_value=8;}
		
     if(manual_slow==0)
		 {			 
		 kp=2;
		 }
	 }
	    heading_1 = (-20)*Degree_to_Rad;
		
	}
}	 
		
	if (base_value>60)
	{
		if (non_return==0)
		  base_value=60;
		else if (non_return==1)
			base_value = 20;
	}
	else if (base_value < 10)         //////5 >> 10  {5>>7}   7>>9  
	{ 
	  base_value = 10;                //////5 >> 10  {5>>7}   7>>9
	}

	//========================	
	control_pid_omni(heading_1,base_value);
	temp_tick_enc = ((-1)*y_dis);
  
}

next=0;
kp=2;
while(!next && !manual_override)
{
			if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
			manual_override=1;
		}
	
	pos();
	diff_tick = y_dis - temp_tick_enc;
	control_pid_omni((-170)*Degree_to_Rad,27);                    /////// (-10,20)
	if (diff_tick < (0))
	{
		next=1;
		set(P2_5);
    set(P2_3);
		
	}
	temp_tick_enc = y_dis;

	
}
/*
////////////  added now for new braking in golden
next=1;
while(!next && manual_override)
{
	 if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		}
control_pid_omni(40,5);
}
////////////===================
*/
/*
next=1;
temp_tick_enc = y_dis;

int i=100;
while(i-- && !manual_override)
{
	control_pid_omni((-173)*Degree_to_Rad,20);
}	
*/
/*
next =1;

while(!next && !manual_override)
{
	   pos();
		if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		}
	
	//heading_1 = (-173)*Degree_to_Rad;
	//base_value=7;                                                               ////// 20 >>  40
	control_pid_omni(heading_1,base_value);
	
		if(temp_tick_enc < y_dis)
			 next=1;		

    temp_tick_enc = y_dis;
}
*/
int delay1=25000,delay2=10000;

while((delay1--) && !manual_override)
{
set(P2_3);

		 if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		}
control_pid_omni(40,5);	
	}
while((delay2--) && !manual_override)
{

	reset(P2_4);
		 if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		}
control_pid_omni(40,5);	
}

if(!manual_override)
{reset(P2_3);}




while(!manual_override)
{
	 if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		}
	control_pid_omni(40,5);
	
}

//if(manual_override)

}
//////////////
//////////////
//////////////
//////////////
//////////////

void golden_second_load()
{
	button_flag =1;
//	set(P2_4);
	//reset(P2_5);
lim11=80+20;//+5;
lim12=48-20;//-5;
lim21=208+20;//+5;
lim22=176-20;//-5;
	
lim11_lim=80+40;   //-5; //126; //80
lim12_lim=48-40;   //+5; //2;   //48
lim21_lim=208+40;  //-5; //253; //208
lim22_lim=176-40;
	
	

y_dis = 0;
prvvaluer = 0;
T1TC = 0;
next=0;
int dist_temp=0,proxy_flag=0;	
manual_override=0;

pos();

dist_temp=y_dis;



	 bool temp=0;
while(!next && !manual_override)	
{  pos();
	
	//  actuate();
	 if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		}
		
		if((!FORWARD_PROXY)&&(temp==0))
		{
			temp=1;
		set(P2_4);
	  reset(P2_5);
		dist_temp=y_dis;
			y_dis = 0;
    prvvaluer = 0;
    T1TC = 0;
  	proxy_flag=1;
		
		}
		
		
	control_pid_omni((-175)*Degree_to_Rad,13);		      //// 15

if((y_dis>(850))&&(temp==1))
{
	set(P2_5);
}
//if((y_dis>(1050))&&(temp==1))
{	
//next=1;
}
	

}

}






void golden_restart()
{
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>==	
//=======================FUNCTION FOR MAUAL TASK_1==================	
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>==	
//------->>>>DONT FORGET TO REFRESH THE ENCODER BEFORE CALLING<<<<<<<=	
//------------------------------------------------------------------  
manual_override =0;
 temp_tick_enc=0,p_temp_tick_enc=0,diff_tick=0,diff_max=0;
 target_speed = 20;
double base_value=50,heading_1;
bool non_return=0;
//int top_gate =0;
count_cycle=0;
heading_1=0;
	
//==============================================================================
//--------------------GO FOR BALL LOADING----------------------------------
/*
step_1: go towards rack with speed of 50 for 2 meter 
step_2: deccelerate for 1 meter 
step_3: load the balls while mainataining the speed of 10
step_4: exit on proxy condition with enocder backup
*/
//==============================================================================
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>STEP_1

lim11=80+20;//+5;
lim12=48-20;//-5;
lim21=208+20;//+5;
lim22=176-20;//-5;
	
lim11_lim=80+40;   //-5; //126; //80
lim12_lim=48-40;   //+5; //2;   //48
lim21_lim=208+40;  //-5; //253; //208
lim22_lim=176-40;
acc=0.008;                                       ///////////  0.005(orignal)
double temp_diff;


while(y_dis<80)
{	
	pos();
	control_pid_omni((-165)*Degree_to_Rad,60);                      ////// -168

		if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	//hold_angle=ang;
			manual_override=1;
		}

}


proxy_safety=0;

while((y_dis<2000)&&(y_dis>=0) && !manual_override && !proxy_safety)                      ////////////////1850(ORIGNAL)
{
		if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		manual_override=1;
		}
//--------ACCELERATION--------------------	
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
//------------------------------------	

///////////////  =================			
			temp_val = ((((double)4096-((unsigned int)((AD0DR5&0x80000000)?((AD0DR5&0x0000FFF0)>>4):temp_val)))/4096)*3000);
      temp_diff = (temp_val-2300);	  
		     
    if(temp_val>2260 && temp_val<2340) 				
           {
			     		heading_1= (-173)*Degree_to_Rad; 
  		     }	
					 
		else if(temp_val>2200 && temp_val<2280) 	
       {
			  temp_val = (temp_val-2300);                          /////// 850  assumed
			 heading_1 = ((-1)*(173*Degree_to_Rad) - (temp_val*0.03)*Degree_to_Rad); 	 
   		 }
			 
		else if(temp_val>2320 && temp_val<2500)
		{
		 temp_diff = (temp_val-2410);	  
		 heading_1 = ((-1)*(173*Degree_to_Rad) - (temp_val*0.003)*Degree_to_Rad); 	 
		}		
////////// .........................		

	//heading_1 -= ang*Degree_to_Rad;	
	
  if(heading_1 < -173*Degree_to_Rad)
		heading_1=(-173)*Degree_to_Rad;

	heading_1=(-163)*Degree_to_Rad;  //extra line che. joieye tyare udai dejo
	
	if (!Pin0_23)
	{
		proxy_safety=1;
	}

	pos();
	control_pid_omni(heading_1,60);                      ////// -168
	
}

float temp_dis=0,brake_speed=55;
next=0;

while((y_dis>=0) && !manual_override && !next)
{

//temp_dis=y_dis-1550;
//brake_speed=brake_speed*exp(-(temp_dis/350));

	pos();
	  
	 if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	//hold_angle=ang;
			manual_override=1;
		}
		
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
	
control_pid_omni((-90)*Degree_to_Rad,10);     //(-90,10)(ORIGNAL)
if (!Pin0_23)
	{
		next=1;
	}
	
} 



//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>STEP_2

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>STEP_3

next=0;
target_speed = 40;                               ///set the target speed here which you want to achieve 
count_cycle=0;
count_cycle1=0;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>STEP_4
//cls();
//lcd((char *)"step_4");
next=0;
temp_tick_enc=0;
diff_tick=0;
y_dis=0;
prvvaluel=0;
prvvaluer=0;
T0TC=0;
T1TC=0;


while(!next && !manual_override)
{
			if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
			manual_override=1;
		}
	
	pos();
	diff_tick = y_dis - temp_tick_enc;
	control_pid_omni((-10)*Degree_to_Rad,20);                    /////// (-10,20)
	if (diff_tick < (0))
	{
		next=1;
	}
	temp_tick_enc = y_dis;

}

////=================================================   GOLDEN BALL LOADING END   ======================================//////////	
////....................................................................................................................////////

//=================================================================================
//                            GO FOR GOLDEN BALL TRANSFER 
//=================================================================================
flag_TZ3 = 1;
bottom_gate=0;
top_gate=0;
temp_tick_enc=0;
diff_tick=0;
lim11=80+10;//+5;
lim12=48-10;//-5;
lim21=208+10;//+5;
lim22=176-10;//-5;
	
lim11_lim=80+40;   //-5; //126; //80
lim12_lim=48-40;   //+5; //2;   //48
lim21_lim=208+40;  //-5; //253; //208
lim22_lim=176-40;
acc=0.009;        ///0.0075 >>  0.009   
next=0;
target_speed = 60;                                               ///////////////////   25 >> 20 >> 
count_cycle2=0;
temp_dis=0;
brake_speed=0;
int top_gate_2 = 0;
float start_speed =20,temp_start_dis=0,no_reverse=0; 
bool init_condt=0;
next = 0;
heading_1=(-2)*Degree_to_Rad;                             ///////////////////  changed 5-6-18 2:08
bool non_return_tz3=0;
int actuation_distance=0;


while(!next && !manual_override)
{
	  pos(); 
	 diff_tick = ((-1)*y_dis) - temp_tick_enc;  
	 /* 	
	if (diff_tick > diff_max)
	   {
	    diff_max = diff_tick;
	   }
  */
	
	if((no_reverse==1)&&(diff_tick < 3))
		{
			base_value=6;
			heading_1 = (-10)*Degree_to_Rad;
		}
		if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		}

//==========================================================================
//--------------------CONDITIONS FOR TASK TO BE PERFOMED -------------------	
	  /*
		if (y_dis < (-200))
		{
			lim11=80+40;   //-5; //126; //80
      lim12=48-40;   //+5; //2;   //48
      lim21=208+40;  //-5; //253; //208
      lim22=176-40;
		}
		*/
		
		if(y_dis<(-11500))
		{
		 next=1;
		}	
		
	if (y_dis < (-1480) && !init_condt)                     //turn the rack and actuate it  (1830)
	{
		if(!manual_override)
		{
		RACK_TURNED;
		reset(P2_3);
		reset(P2_4);
	  } 
			//	heading_1 = (-17)*Degree_to_Rad; ///////////////////  changed 5-6-18 2:08  
    init_condt = 1;		
		//acc=0.003;
	}
	
	
  //actuations as per laser
	if((y_dis<(-7500)) && (top_gate_2==0))
	{
	top_gate_2 =1;
	}
		
  if((!FORWARD_PROXY) && (top_gate_2 ==1) )                      	//////if((Pin1_20==1) && (top_gate_2 ==1))            ////////// (Pin1_20==1)  
	{
		//set(P2_4);
		count_cycle2 = 0;
		top_gate_2 =3;
	 // reset(P2_5);
	  actuation_distance = y_dis;
	}
	
	if((y_dis<(actuation_distance-100)) && (top_gate_2 ==3))
	{
	set(P2_4);
	reset(P2_5);
	}
	
	if(count_cycle2>10  && top_gate_2==3)
	 {
	  heading_1=(-5)*Degree_to_Rad;
		base_value=8;	 
	 } 
	
	 if((y_dis<(actuation_distance-750)) && (top_gate_2 ==3))                                      //if( (count_cycle2 > 330)  && (top_gate_2 ==3))              //////// >> 300
	 {
		 set(P2_5);
     top_gate=4;
		 count_cycle2=0;
	 }  
	 
	 
   if(top_gate==4 && count_cycle2>400) 
	 {  base_value= 2;
	   heading_1=(-23)*Degree_to_Rad;
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
//------------------------------------		
	
	//////////======SPEED CONTROL=====	
	
	if (y_dis > (-5750))                      /////   6200(ORIGNAL)
	{
		//base_value = 60;//50  >>> 55 
    brake_speed=60;	//50  >>> 55     
    
		pos();
		
		if(y_dis>(-100))
		{
			base_value=30;
		 // heading_1=(-)*Degree_to_Rad;
		}
		
		else
		base_value=60;	
		
	}
	
	else
	{  
   if((diff_tick < 15)&&(non_return_tz3==0))
	 {
		 heading_1 = (-90)*Degree_to_Rad;
		 base_value = 40;
		 
	 }
	 else
	 {
		 non_return_tz3=1;
		 heading_1 = (-20)*Degree_to_Rad;
		 base_value = 10;
	 }
	
	}	
		
	if (base_value>60)
	{
		if (non_return==0)
		  base_value=60;
		else if (non_return==1)
			base_value = 20;
	}
	else if (base_value < 10)         //////5 >> 10  {5>>7}   7>>9  
	{ 
	  base_value = 10;                //////5 >> 10  {5>>7}   7>>9
	}

	//========================	
	control_pid_omni(heading_1,base_value);
	temp_tick_enc = ((-1)*y_dis);
  
}

next=0;

while(!next && !manual_override)
{
			if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
			manual_override=1;
		}
	
	pos();
	diff_tick = y_dis - temp_tick_enc;
	control_pid_omni((-170)*Degree_to_Rad,27);                    /////// (-10,20)
	if (diff_tick < (0))
	{
		next=1;
		reset(P2_4);
		set(P2_5);

	}
	temp_tick_enc = y_dis;

	
}
while(!manual_override)
{
	 if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		}
	control_pid_omni(40,5);
	
}

}	


void tz2_restart()
{

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>==	
//=======================FUNCTION FOR MAUAL TASK_1==================	
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>==	
//------->>>>DONT FORGET TO REFRESH THE ENCODER BEFORE CALLING<<<<<<<=	
//------------------------------------------------------------------  
manual_override =0;
 temp_tick_enc=0,p_temp_tick_enc=0,diff_tick=0,diff_max=0;
 target_speed = 20;
double base_value=50,heading_1;
bool non_return=0;
//int top_gate =0;
count_cycle=0;
heading_1=0;
	
//==============================================================================
//--------------------GO FOR BALL LOADING----------------------------------
/*
step_1: go towards rack with speed of 50 for 2 meter 
step_2: deccelerate for 1 meter 
step_3: load the balls while mainataining the speed of 10
step_4: exit on proxy condition with enocder backup
*/
//==============================================================================
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>STEP_1

lim11=80+20;//+5;
lim12=48-20;//-5;
lim21=208+20;//+5;
lim22=176-20;//-5;
	
lim11_lim=80+40;   //-5; //126; //80
lim12_lim=48-40;   //+5; //2;   //48
lim21_lim=208+40;  //-5; //253; //208
lim22_lim=176-40;
acc=0.008;                                       ///////////  0.005(orignal)
double temp_diff;


while(y_dis<80)
{	
	pos();
	control_pid_omni((-165)*Degree_to_Rad,60);                      ////// -168

		if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	//hold_angle=ang;
			manual_override=1;
		}

}
/*
lim11=80+40;//+5;
lim12=48-40;//-5;
lim21=208+40;//+5;
lim22=176-40;//-5;
*/


proxy_safety=0;

while((y_dis<2000)&&(y_dis>=0) && !manual_override && !proxy_safety)                      ////////////////1850(ORIGNAL)
{
		if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		manual_override=1;
		}
//--------ACCELERATION--------------------	
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
//------------------------------------	

///////////////  =================			
			temp_val = ((((double)4096-((unsigned int)((AD0DR5&0x80000000)?((AD0DR5&0x0000FFF0)>>4):temp_val)))/4096)*3000);
      temp_diff = (temp_val-2300);	  
		     
    if(temp_val>2260 && temp_val<2340) 				
           {
			     		heading_1= (-173)*Degree_to_Rad; 
  		     }	
					 
		else if(temp_val>2200 && temp_val<2280) 	
       {
			  temp_val = (temp_val-2300);                          /////// 850  assumed
			 heading_1 = ((-1)*(173*Degree_to_Rad) - (temp_val*0.03)*Degree_to_Rad); 	 
   		 }
			 
		else if(temp_val>2320 && temp_val<2500)
		{
		 temp_diff = (temp_val-2410);	  
		 heading_1 = ((-1)*(173*Degree_to_Rad) - (temp_val*0.003)*Degree_to_Rad); 	 
		}		
////////// .........................		

	//heading_1 -= ang*Degree_to_Rad;	
	
  if(heading_1 < -173*Degree_to_Rad)
		heading_1=(-173)*Degree_to_Rad;

	heading_1=(-163)*Degree_to_Rad;  //extra line che. joieye tyare udai dejo
	
	if (!Pin0_23)
	{
		proxy_safety=1;
	}

	pos();
	control_pid_omni(heading_1,60);                      ////// -168
	
}

float temp_dis=0,brake_speed=55;
next=0;

while((y_dis>=0) && !manual_override && !next)
{

//temp_dis=y_dis-1550;
//brake_speed=brake_speed*exp(-(temp_dis/350));

	pos();
	  
	 if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	//hold_angle=ang;
			manual_override=1;
		}
		
	
control_pid_omni((-90)*Degree_to_Rad,10);     //(-90,10)(ORIGNAL)
if (!Pin0_23)
	{
		next=1;
	}
	
}



//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>STEP_2
/*
while(y_dis<2000)//&&(diff_tick>=0))
{
	pos();
	control_pid_omni(10*Degree_to_Rad,20);     //-15
}
*/
/*
int i=00;
while(i-- && !manual_override)
{
	control_pid_omni((-100)*Degree_to_Rad,20);
}
*/
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>STEP_3

next=0;
target_speed = 40;                               ///set the target speed here which you want to achieve 
count_cycle=0;
count_cycle1=0;

while(!next && !manual_override && (y_dis>=0) && count_cycle1<800)
{
		if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		}
		
	if (!Pin0_23)
	{
		next=1;
	}
	
	pos();

	diff_tick = y_dis - temp_tick_enc;

	if (diff_tick > diff_max)
	{
		diff_max = diff_tick;
	}

//	heading_1  = (-1)*(175*Degree_to_Rad);
	if ((diff_max - target_speed)>0)
		 base_value -= (diff_max - target_speed)*0.7;
	else 
		 base_value -= (diff_max - target_speed)*1;
	 
	if (base_value>20)
	{
		base_value = 20;
	}
	else if (base_value < 12)
	{
		base_value = 12;
	}
//========================	

	///////////////  =================			
			temp_val = ((((double)4096-((unsigned int)((AD0DR5&0x80000000)?((AD0DR5&0x0000FFF0)>>4):temp_val)))/4096)*3000);
//      temp_diff = (temp_val-2300);	  
		     
    if(temp_val>2260 && temp_val<2340) 				
           {
			     		heading_1= (-175)*Degree_to_Rad; 
  		     }	
					 
		else if(temp_val>2200 && temp_val<2280) 	
       {
			  temp_val = (temp_val-2300);                          /////// 850  assumed
			 heading_1 = ((-1)*(175*Degree_to_Rad) - (temp_val*0.03)*Degree_to_Rad); 	 
   		 }
			 
		else if(temp_val>2320 && temp_val<2500)
		{
		 temp_diff = (temp_val-2410);	  
		 heading_1 = ((-1)*(175*Degree_to_Rad) - (temp_val*0.003)*Degree_to_Rad); 	 
		}		
////////// .........................		


	base_value =12;
	control_pid_omni(heading_1,base_value);
	
	temp_tick_enc = y_dis;

}





//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>STEP_4
//cls();
//lcd((char *)"step_4");
next=0;
temp_tick_enc=0;
diff_tick=0;
y_dis=0;
prvvaluel=0;
prvvaluer=0;
T0TC=0;
T1TC=0;


while(!next && !manual_override)
{
			if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
			manual_override=1;
		}
	
	pos();
	diff_tick = y_dis - temp_tick_enc;
	control_pid_omni((-10)*Degree_to_Rad,20);                    /////// (-10,20)
	if (diff_tick < (0))
	{
		next=1;
	}
	temp_tick_enc = y_dis;

}

//=================================================================================
//------------------------return after loading the balls---------------------------
//==================================================================================
temp_tick_enc=0;
diff_tick=0;
lim11=80+5;//+5;
lim12=48-5;//-5;
lim21=208+5;//+5;
lim22=176-5;//-5;
	
lim11_lim=80+40;   //-5; //126; //80
lim12_lim=48-40;   //+5; //2;   //48
lim21_lim=208+40;  //-5; //253; //208
lim22_lim=176-40;
acc=0.0082;
bool non_return_tz1=0;
heading_1=((-1)*(10*Degree_to_Rad));                // -10
next=0;
target_speed = 25;
count_cycle2=0;
if(!manual_override)
set(P2_5);
bool lower_gate_2 =0;
 temp_dis=0,brake_speed=55;
int dist_temp=0; 
int trial_flag=0;
hold_angle=0;

while(!next && !manual_override)
{
	pos();
	diff_tick = ((-1)*y_dis) - temp_tick_enc;

//===================================================================	
		if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
			manual_override=1;
    }
//==========================================================================
//--------------------CONDITIONS FOR TASK TO BE PERFOMED -------------------	
	if (y_dis < (-1030) && !lower_gate_2)                     //turn the rack and actuate it 
	{
		if(!manual_override)
		{
		RACK_TURNED;
		reset(P2_3);
		reset(P2_4);
		}
		
	}
	
	if(y_dis< (-7200) && top_gate==0)
	{
		target_speed=10;
		lower_gate_2= 1;
		top_gate = 1;
		count_cycle2=0;
	 //  reset(P2_5);
	}

	
 	if((!FORWARD_PROXY) && (top_gate == 1) && (count_cycle2>400))                                   ////////////////Pin1_23
	{

		if(!manual_override)
		{
		//set(P2_4);
		count_cycle2 = 0;
		top_gate = 2;
		//reset(P2_5);
		dist_temp=y_dis;	
		tz2_actuation=y_dis;	
	  }
	
	}
	
	if ((top_gate==2)&&(y_dis<(dist_temp-20)))
	{   
	//set(P2_4);
	//reset(P2_5);
	}
	
	
	if ((top_gate==2)&&(count_cycle2>170))
	{
		base_value = 10;
		target_speed = 10;
	}

  	if(y_dis<(dist_temp-590)&& top_gate>=2)     //// 9340
	  {
		   if(!manual_override)
		     {
		   //   set(P2_5);
	      	top_gate = 3;
	       }
		
				 if(y_dis<(-9470)&& top_gate>=2)  /// 9470(orignal,working) >> 9540>> (9690 {18-7-18 5:30 pm})
		     {
		 	    top_gate = 3;
	       // reset(P2_4);
		     }
		
		}	
		
	  if(y_dis<(-9300) && top_gate ==3)                    //////10200
	  { 
			base_value=11;
		  heading_1=(-4)*Degree_to_Rad;
		}

		
	if (y_dis < (-10000))                   //11100 >> 10900 >> 10000
	{
	next=1;
	}
		
//===========================================================================	
//--------ACCELERATION--------------------	
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
//------------------------------------		
	

	
//======SPEED CONTROL=====	
	if (y_dis > (-5400))                        ////  550(new orignal)  >> 5600 >> 5800                  
	{ 
		base_value = 60;
	  brake_speed=base_value;
	 	heading_1=(-8)*Degree_to_Rad;
    
    if(y_dis>(-500))		
		{
		kp=1.5;
	  }
		else 
    kp=0.9;		 
		
	}

	else 
	{
	
		if((diff_tick > 16)&&(non_return_tz1==0))
		{
			base_value=40;
			heading_1 = (-90)*Degree_to_Rad;
		}
		else
		{
			non_return_tz1=1;
			base_value=8;                     //////////10
			heading_1 = (-20)*Degree_to_Rad;
		}

		
/*
		if (y_dis > (-7400)&&(non_return==0))                                 /// 6600(orignal)        // tick limit , use this reference in the equation 
		{  
     
		
		  
			
		}
		else  
		{		
			non_return=1;
			//heading_1  = (-1)*(23*Degree_to_Rad);                      /////// 19:02   -18>> -23
			if ((diff_max - target_speed)>0)
		      base_value -= (diff_max - target_speed)*0.7;
			else 
				  base_value -= (diff_max - target_speed)*1;

/////=========  			
	
			//===================== Get SHARP value and then smoothen it using a moving average filter. Set boundaries on the angle so obtained =======================
      		temp_val = ((((double)4096-((unsigned int)((AD0DR5&0x80000000)?((AD0DR5&0x0000FFF0)>>4):temp_val)))/4096)*3000);
      temp_diff = (temp_val-2300);	  
       
    if(temp_val>2260 && temp_val<2340) 				
      {
			heading_1= (-29)*Degree_to_Rad; 
  		}	
					 
		else if(temp_val>2200 && temp_val<2280) 	
       {
			 temp_val = (temp_val-2300);                          /////// 850  assumed
			 heading_1 = ((-1)*(29*Degree_to_Rad) - (temp_val*0.03)*Degree_to_Rad); 	 
   		 }
			 
		else if(temp_val>2320 && temp_val<2500)
		  {
		  temp_diff = (temp_val-2410);	  
		  heading_1 = ((-1)*(29*Degree_to_Rad) - (temp_val*0.003)*Degree_to_Rad); 	 
		  }		
			base_value=9;                                   ////////////// 7
		  heading_1=(-20)*Degree_to_Rad;
	 /////======   
		
		} 
		*/
	}
	
	if (base_value>52)
	{
		if (non_return==0)
		  base_value=52;
		else if (non_return==1)
			base_value = 8;                                   
	}
	
	else if (base_value < 3)            /// 8>>15 ,,,,,  7>>5>>4 
	{
		base_value = 3;                   /// 8>>15,,,,,   7>>5>>4
	}

	//========================	
	if(y_dis<-9600)
		base_value = 11;    //11
	
	control_pid_omni(heading_1,base_value);

	temp_tick_enc = (-1*y_dis);

}
	kp=2;												     //Kp for Omni
	kd=0.6; 	  										 //Kd for Omni
	ki=0;			  										 //Ki for omni
manual_tz1_to_golden();

}

void golden_second_load_2()
{
	button_flag =1;
//	set(P2_4);
	//reset(P2_5);
lim11=80+20;//+5;
lim12=48-20;//-5;
lim21=208+20;//+5;
lim22=176-20;//-5;
	
lim11_lim=80+40;   //-5; //126; //80
lim12_lim=48-40;   //+5; //2;   //48
lim21_lim=208+40;  //-5; //253; //208
lim22_lim=176-40;
	
	

y_dis = 0;
prvvaluer = 0;
T1TC = 0;
next=0;
int dist_temp=0,proxy_flag=0;	
manual_override=0;

pos();

dist_temp=y_dis;
	
int i=2000;	

while(!next && !manual_override)	
{  pos();
	//  actuate();
	 if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		}
		
		if((!FORWARD_PROXY)&&(proxy_flag==0))
		{
		//set(P2_4);
	  //reset(P2_5);
		y_dis = 0;
    prvvaluer = 0;
     T1TC = 0;	
		//dist_temp=y_dis;
		proxy_flag=1;
			
		}
		
		
		if(proxy_flag && (y_dis<(-100)))
		{
		set(P2_4);
	  reset(P2_5);
		}
		
		/*
		if(ps_cross)
		{
		set(P2_4);
	  reset(P2_5);
		dist_temp=y_dis;
		}	
	  */
		

if(y_dis<(-650)&& proxy_flag==1)
{
	set(P2_5);
}
 
if(y_dis<(-880)&&proxy_flag==1)
{	
next=1;
set(P2_5);
//reset(P2_4);	
}	
control_pid_omni((-20)*Degree_to_Rad,19);

}
while(i-- && !manual_override )
{
 if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		}
	
control_pid_omni((-170)*Degree_to_Rad,9);
}

int delay1=25000,delay2=10000;

while((delay1--) && !manual_override)
{
set(P2_3);

		 if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		}
control_pid_omni(40,5);	
	}
while((delay2--) && !manual_override)
{

	reset(P2_4);
		 if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		}
control_pid_omni(40,5);	
}

reset(P2_3);




while(!manual_override)
{
	 if(count_cycle>7)    //////   ps2 value update    
 		{
			count_cycle=0;
			Ps2_val_update();
		}
		
		if((temp_four!=400 && temp_four!=360))
		{
		 	hold_angle=ang;
			manual_override=1;
		}
	control_pid_omni(40,5);
	
}




}












