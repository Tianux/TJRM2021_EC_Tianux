#include "main.h"

#define Feed_Off 0
#define Feed_On  1

#define Feed_stuck -1
#define Feed_stop 0
#define Feed_clearbomb 1
#define Feed_singleshoot 2
#define Feed_tripleshoot 3
#define Feed_tripleshoot_rh 4 //reach the heat limit under triple tap


/*PID parameter*/
PID_Type Feed_Speed_PID;
PID_Type Feed_Position_PID;

float Feed_PID_SP=12;//40.0;
float Feed_PID_SI=3.2;//3.2;
float Feed_PID_SD=0.1;//1;

float Feed_PID_PP=40;//40.0;
float Feed_PID_PI=6;//3.2;
float Feed_PID_PD=0;//1;

int Feed_speed_target;
int Feed_position_target;
float PIDOut_Position_Shoot,PIDOut_Speed_Shoot,PIDOut_Whole_Shoot;
int PIDOut_Feed;
int Feed_state; // for single shoot
	
int Feed_Mode;  //-1,stuck 0,feed_stop  1,clear_bomb 2,triple tap 3,signal shoot 4,reach the heat limit under triple tap
int Mode1_stop_flag = 0;

/*triple singal switch parameter*/
int Triple_Tap = 3;
int Signal_Shoot = 1;
int tri_sin_swi_flag = 1; //0 triple tap   1 signal shoot
int press_R_count = 0;
int mode_change_flag = 0;
int mode_change_delay = 0;
int mode_change_delay_init = 100;
/*stuck_parameter*/
int stuck_count=0;
int inver_rotate_delay=0;
int inver_rotate_delay_init=20;
/*RefereeInfo_parameter*/
float heat_limite=360;
int remain_bullet=0;
int old_shoot_seq = 0;

extern int16_t frictionState;
float heat_limit=200;
float rest_heat;//剩余热量
float rest_heat_h=100;//剩余热量上限（高于上限可以恢复射击） 参数可改
bool heat_limit_flag=false;
void FeedControlInit(void)
{
  PidFeedInit();
	
}

void StuckMoni(void)
{
	if((fabs(current_speed_207)<1&&PIDOut_Feed>8000))
	{
		stuck_count++;
	}
	if(stuck_count>300)    //0.6s
	{
		inver_rotate_delay=inver_rotate_delay_init;
		stuck_count=0;
	}
	
}
void StuckMoni_Speed(void)
{
	if((Feed_speed_target>100&&fabs(current_speed_207<5)))
	{
		stuck_count++;
	}
	if(stuck_count>300)     //0.6s
	{
		inver_rotate_delay=inver_rotate_delay_init;
		stuck_count=0;
	}
}

void FeedJudge()
{
  if(remoteState == NORMAL_REMOTE_STATE&&inver_rotate_delay<=0)
	{
		if(frictionState==Friction_ON&&RC_Ex_Ctl.rc.s1==2)
		{
		  
			Feed_Mode = Feed_clearbomb;
		}
		else
		{
			Feed_Mode = Feed_stop;
		}			
	}
	else if((remoteState == KEY_REMOTE_STATE||remoteState ==VIEW_STATE)&&inver_rotate_delay<=0)
	{
		if(frictionState==Friction_ON)
		{
			if(remain_bullet<=1)
			{
				Feed_Mode = Feed_stop;
			}
			else
			{
				if(RC_Ctl.mouse.press_r)//fullballet cancelled; not defined yet
				{
//						Feed_Mode = 1;
//					  Mode1_stop_flag = 1;
				}
				else if((RC_Ctl.mouse.press_r==0)&&(Mode1_stop_flag == 1))
				{
					Feed_Mode = Feed_stop;
					Mode1_stop_flag = 0;
				}
				else if((RC_Ctl.mouse.press_l)&&(tri_sin_swi_flag==0))//三连发
				{
					if(remain_bullet>3)
					{
						Feed_Mode = Feed_tripleshoot;
					}
					else 
					{
						Feed_Mode = Feed_tripleshoot_rh;
					}
					}
				else if((RC_Ctl.mouse.press_l)&&(tri_sin_swi_flag==1))//单发
				{
						Feed_Mode = Feed_singleshoot;
				}
				
			}
		}
	}
	else if(inver_rotate_delay>0)
	{
		Feed_Mode=Feed_stuck;
		inver_rotate_delay--;
	}
	
}

void PidFeedInit(void)
{
	PID_ControllerInit(&Feed_Speed_PID,50,50,9800,0.01);
	PID_SetGains(&Feed_Speed_PID,Feed_PID_SP,Feed_PID_SI,Feed_PID_SD);
	PID_ControllerInit(&Feed_Position_PID,50,50,9800,0.01);
	PID_SetGains(&Feed_Position_PID,Feed_PID_PP,Feed_PID_PI,Feed_PID_PD);
}

void TargetRenew(u8 flag)
{
	switch(flag)
	{
		case 0:
		{
			Feed_position_target=continuous_current_position_207;
			Feed_speed_target=current_speed_207;
		}break;
		case 1:
		{
			Feed_position_target=continuous_current_position_207;
		}break;
		case 2:
		{
			Feed_speed_target=current_speed_207;
		}break;
	}
}

void Shoot_Para_Moni(void)
{
	  if((RC_Ex_Ctl.key.v & KEY_PRESSED_OFFSET_R )==KEY_PRESSED_OFFSET_R)
	  {
				press_R_count++;
				if(mode_change_flag==0)
				{
					tri_sin_swi_flag = 1;
					mode_change_flag = 1;
					mode_change_delay = mode_change_delay_init;
				}
			  
				if(press_R_count>=300)
				{ 
					tri_sin_swi_flag = 0;
					press_R_count = 0;
				}	
		}
		else{
				if(mode_change_flag==1)
				{
					mode_change_delay--;
				}
				if(mode_change_delay<=0)
				{
					mode_change_flag=0;
					press_R_count=0;
				} 
		}
		
		
		remain_bullet=(heat_limite-ext_power_heat_data.shooter_id1_42mm_cooling_heat)/25.0f;
}

void Shoot_target_Cal(void)//right_mouseclick cancelled(fullbullet shooting)
{
	rest_heat=ext_game_robot_state.shooter_id1_42mm_cooling_limit-ext_power_heat_data.shooter_id1_42mm_cooling_heat;
	if(rest_heat>=rest_heat_h){
		heat_limit_flag=false;}
	else	
	{
		heat_limit_flag=true;		
	}


	
  if(Feed_Mode==Feed_stuck)
	{Feed_speed_target=-3000; PIDOut_Feed=PID_ControllerDriver(&Feed_Speed_PID,Feed_speed_target,current_speed_207);}
	else if(Feed_Mode==Feed_stop)
	{Feed_speed_target=0;PIDOut_Feed=PID_ControllerDriver(&Feed_Speed_PID,Feed_speed_target,current_speed_207);}
	else if(Feed_Mode==Feed_clearbomb)
	{Feed_speed_target=3000;PIDOut_Feed=PID_ControllerDriver(&Feed_Speed_PID,Feed_speed_target,current_speed_207);StuckMoni_Speed();} //3000,3300
	else if(Feed_Mode==Feed_singleshoot)
	{	if(RC_Ctl.mouse.press_l)
		{
			if(heat_limit_flag){Feed_position_target=continuous_current_position_207;}
			else Feed_position_target=continuous_current_position_207+Signal_Shoot*23;
		}
		ShootMove(Feed_position_target);
		StuckMoni();
	}
/*	{if(RC_Ctl.mouse.press_l){Feed_position_target=1*(continuous_current_position_207+Triple_Tap*40);}ShootMove(Feed_position_target);StuckMoni();}*/

}

void SingleShoot_Moni(void){
	if (shoot_seq > old_shoot_seq){
		Feed_state = Feed_Off;
		PIDOut_Feed = 0;
	}
	else{
		Feed_state = Feed_On;
	}
	old_shoot_seq = shoot_seq;
}

void ShootMove(int SetPos) 
{
	float NowPos = continuous_current_position_207;
	float NowSpeed=current_speed_207;
	

  PIDOut_Position_Shoot=PID_ControllerDriver(&Feed_Position_PID,SetPos,NowPos);
	PIDOut_Speed_Shoot=PID_ControllerDriver(&Feed_Speed_PID,PIDOut_Position_Shoot,NowSpeed);
	PIDOut_Feed = PIDOut_Speed_Shoot;	
}


static int u8count2 = 0;
void FeedControlLoop(void)
{
	Shoot_Para_Moni();
	FeedJudge();
	Shoot_target_Cal();
		u8count2++;
	if(u8count2 == 10){
					u8count2 = 0 ;
					UART_DMA_SEND(ext_power_heat_data.shooter_id1_42mm_cooling_heat);
					//ext_power_heat_data.shooter_heat0;speed_level
					}
	if (Feed_Mode == Feed_singleshoot){
		SingleShoot_Moni();		
	}

	 CAN2_Cmd_All((int16_t)-PIDOut_Whole_Yaw, (int16_t)PIDOut_Whole_Pit,(int16_t)PIDOut_Feed);
}


