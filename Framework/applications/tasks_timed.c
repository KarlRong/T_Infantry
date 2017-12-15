/**
  ******************************************************************************
  * File Name          : tasks_cmcontrol.c
  * Description        : 2ms定时任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
	* 2ms定时
	* 通过count可以获得500ms,1s等定时任务
	* 状态机切换，串口定时输出，看门狗等
  ******************************************************************************
  */
#include <tim.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <cmsis_os.h>
#include <task.h>
#include <usart.h>
#include "tasks_timed.h"
#include "pid_Regulator.h"
#include "drivers_uartrc_low.h"
#include "drivers_uartrc_user.h"
#include "tasks_remotecontrol.h"
#include "tasks_platemotor.h"
#include "application_motorcontrol.h"
#include "drivers_canmotor_low.h"
#include "drivers_canmotor_user.h"
#include "utilities_debug.h"
#include "rtos_semaphore.h"
#include "rtos_task.h"
#include "peripheral_define.h"
#include "drivers_platemotor.h"
#include "application_waveform.h"
#include "drivers_uartjudge_low.h"
#include "drivers_uartupper_user.h"
#include "utilities_minmax.h"
#include "drivers_ramp.h"
#include "peripheral_laser.h"
#include "drivers_uartrc_low.h"
#include "tasks_motor.h"//zy
#include "tasks_arm.h"
#include <stdbool.h>
#include "visualscope.h"

extern PID_Regulator_t CMRotatePID ; 
extern PID_Regulator_t CM1SpeedPID;
extern PID_Regulator_t CM2SpeedPID;
extern PID_Regulator_t CM3SpeedPID;
extern PID_Regulator_t CM4SpeedPID;



Shoot_State_e last_shoot_state = NOSHOOTING;
Shoot_State_e this_shoot_state = NOSHOOTING;
//uint32_t last_Encoder = 0;
//uint32_t this_Encoder = 0;
int flag = 0;

WorkState_e g_workState = PREPARE_STATE;
WorkState_e lastWorkState = PREPARE_STATE;
WorkState_e GetWorkState()
{
	return g_workState;
}
/*2ms定时任务*/

extern float ZGyroModuleAngle;
float ZGyroModuleAngleMAX;
float ZGyroModuleAngleMIN;
extern float yawRealAngle;
extern uint8_t g_isGYRO_Rested;
extern float pitchAngleTarget;
extern float pitchRealAngle;
extern float gYroZs;
extern float yawAngleTarget;
extern float yawRealAngle;

extern float PM1SpeedPID;

extern uint8_t JUDGE_STATE;

int mouse_click_left = 0;


extern uint8_t JUDGE_Received;
extern uint8_t JUDGE_State;

extern float PM1AngleTarget;
extern float PM2AngleTarget;
extern float PM1RealAngle;
extern float PM2RealAngle;


static uint32_t s_time_tick_2ms = 0;


extern RampGen_t frictionRamp ;
extern uint8_t bShoot;
uint16_t zyShootTimeCount=0;
uint8_t zyRuneMode=0;
uint16_t checkRecTime=300;
Location_Number_s pRunePosition[3];
uint16_t checkKeyTime=500;

uint8_t visualscopeCount = 0;
//uint8_t delay30ms_flag = 0;


void Timer_2ms_lTask(void const * argument)
{
	//RTOS提供，用来做2ms精确定时
	//与后面的vTaskDelayUntil()配合使用
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	//countwhile来获得不同定时任务
	static int s_countWhile = 0;

	
//static int shootwhile = 0;
//unsigned portBASE_TYPE StackResidue; //栈剩余
	while(1)  
	{       
		
		WorkStateFSM();//状态机
	  WorkStateSwitchProcess();//状态机动作

		//陀螺仪复位计时
    if(s_time_tick_2ms == 2000)
		{
			GYRO_RST();//给单轴陀螺仪将当前位置写零，注意需要一定的稳定时间
		}            //在从STOP切换到其他状态时，s_time_tick_2ms清零重加，会重新复位陀螺仪

		getJudgeState();
		
		if(checkRecTime<65534)
		{
			checkRecTime++;
		}
		if(checkKeyTime<65534)
		{
			checkKeyTime++;
		}

		if(visualscopeCount>=15)
		{
			visualscopeCount = 0;
		}
		else
		{
			visualscopeCount++ ;
		}
		
//		if(visualscopeCount==0)
//			//printf("%f\r\n", PM1RealAngle);
//		else if(visualscopeCount==14)
			VisualScope(&huart3, (int)PM1RealAngle, 0, 0, 0); 
		
		
		
		if(s_countWhile >= 2000)//150 1000
		{//定时1s,发送调试信息
			s_countWhile = 0;
			/*
//			IOPool_getNextRead(GMYAWRxIOPool, 0); 
//			float tempYaw = (IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle-100) * 360 / 8192.0f;
//			NORMALIZE_ANGLE180(tempYaw);
//			fw_printfln("YawAngle= %f,targetYaw:%f", tempYaw,yawAngleTarget);
////		fw_printfln("YawAngle= %f", tempYaw);
////			IOPool_getNextRead(GMPITCHRxIOPool, 0); 
//			float tempPitch = -(IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle - 6400) * 360 / 8192.0f;
//			NORMALIZE_ANGLE180(tempPitch);
//			fw_printfln("PitchAngle= %f,targetPitch:%f", tempPitch,pitchAngleTarget);
//			fw_printfln("PitchAngle= %f", tempPitch);
			//fw_printfln("ZGyroModuleAngle:  %f",ZGyroModuleAngle);
//			fw_printfln("YawAngle= %d", IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle);
//			fw_printfln("PitchAngle= %d", IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle);
			*****查看任务栈空间剩余示例*******
			//		StackResidue = uxTaskGetStackHighWaterMark( GMControlTaskHandle );
			//		fw_printfln("GM%ld",StackResidue);*/
			
//			fw_printfln("PM1AngelTarget is %f", PM1AngleTarget);
//			fw_printfln("PM2AngelTarget is %f", PM2AngleTarget);
			PM1AngleTarget += 360.0;
			//PM1AngleTarget += 360.0;
//			PM2AngleTarget += 360.0;
			
			if(JUDGE_State == OFFLINE)
			{
				fw_printfln("Judge not received");
			}
		}
		else
		{
			s_countWhile++;
		}
		
		vTaskDelayUntil( &xLastWakeTime, ( 2 / portTICK_RATE_MS ) );//这里进入阻塞态等待2ms
	}
}
	

void CMControlInit(void)
{
//底盘电机PID初始化，copy from官方开源程序
	//ShootMotorSpeedPID.Reset(&ShootMotorSpeedPID);
	CMRotatePID.Reset(&CMRotatePID);
	CM1SpeedPID.Reset(&CM1SpeedPID);
	CM2SpeedPID.Reset(&CM2SpeedPID);
	CM3SpeedPID.Reset(&CM3SpeedPID);
	CM4SpeedPID.Reset(&CM4SpeedPID);
}
/**********************************************************
*工作状态切换状态机
**********************************************************/

extern RemoteSwitch_t g_switch1; 
extern RC_Ctl_t RC_CtrlData; 
extern bool g_switchRead;


void WorkStateFSM(void)
{
	lastWorkState = g_workState;
	s_time_tick_2ms ++;
	
	switch(g_workState)
	{
		case PREPARE_STATE:
		{
			if(GetInputMode() == STOP )
			{
				g_workState = STOP_STATE;
			}
			else if(s_time_tick_2ms > PREPARE_TIME_TICK_MS)
			{
				LASER_ON();
				g_workState = NORMAL_STATE;
			}			
		}break;
		
		case NORMAL_STATE:     
		{
			if(GetInputMode() == STOP )
			{
				g_workState = STOP_STATE;
			}
		}break;
		
		case STOP_STATE:   
		{
			if(GetInputMode() != STOP )
			{
				g_workState = PREPARE_STATE;   
			}
		}break;

		default:
		{
			
		}
	}	
}


extern float gap_angle;
extern float pitchRealAngle;
	
void WorkStateSwitchProcess(void)
{
	if((lastWorkState != g_workState) && (g_workState == STOP_STATE))  
	{
		LASER_OFF();
		SetShootState(NOSHOOTING);
		SetFrictionWheelSpeed(1000);
		SetFrictionState(FRICTION_WHEEL_OFF);
		frictionRamp.ResetCounter(&frictionRamp);
	}
	//如果从其他模式切换到prapare模式，要将一系列参数初始化
	if((lastWorkState != g_workState) && (g_workState == PREPARE_STATE))  
	{
		//计数初始化
	  s_time_tick_2ms = 0;   
		yawAngleTarget = 0;
		pitchAngleTarget = 0;
		CMControlInit();
		RemoteTaskInit();
		armReset();
	}
}

void getJudgeState(void)
{
	static int s_count_judge = 0;
	if(JUDGE_Received==1)
	{
		s_count_judge = 0;
		JUDGE_State = ONLINE;
		JUDGE_Received = 0;
	}
	else
	{
		s_count_judge++;
		if(s_count_judge > 150)
		{//300ms
			JUDGE_State = OFFLINE;
		}
	}
}

