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

#include "utilities_minmax.h"
#include "drivers_ramp.h"
#include "peripheral_laser.h"
#include "drivers_uartrc_low.h"//zy
extern PID_Regulator_t CMRotatePID ; 
extern PID_Regulator_t CM1SpeedPID;
extern PID_Regulator_t CM2SpeedPID;
extern PID_Regulator_t CM3SpeedPID;
extern PID_Regulator_t CM4SpeedPID;
PID_Regulator_t ShootMotorPositionPID = SHOOT_MOTOR_POSITION_PID_DEFAULT;      //shoot motor
PID_Regulator_t ShootMotorSpeedPID = SHOOT_MOTOR_SPEED_PID_DEFAULT;


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
extern float pitchRealAngle;
extern float gYroZs;
extern float g_yawAngleTarget;
extern float yawRealAngle;

extern uint8_t JUDGE_STATE;

int mouse_click_left = 0;


int stuck = 0;	//卡弹标志位，未卡弹为false，卡弹为true

static int s_count_judge = 0;
extern uint8_t JUDGE_Received;
extern uint8_t JUDGE_State;

static uint32_t s_time_tick_2ms = 0;

FrictionWheelState_e friction_wheel_stateZY = FRICTION_WHEEL_OFF;
extern RampGen_t frictionRamp ;//张雁大符

void Timer_2ms_lTask(void const * argument)
{
	//RTOS提供，用来做2ms精确定时
	//与后面的vTaskDelayUntil()配合使用
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	//countwhile来获得不同定时任务
	static int s_countWhile = 0;

	ShootMotorPositionPID.ref = 0x0;
	ShootMotorPositionPID.fdb = 0x0;
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
		
		ShooterMControlLoop();       //发射机构控制任务
		
		
		if(s_countWhile >= 1000)
		{//定时1s,发送调试信息
			s_countWhile = 0;
//			IOPool_getNextRead(GMYAWRxIOPool, 0); 
//			float tempYaw = (IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle-1445) * 360 / 8192.0f;
//			NORMALIZE_ANGLE180(tempYaw);
//			fw_printfln("YawAngle= %f", tempYaw);
//			IOPool_getNextRead(GMPITCHRxIOPool, 0); 
//			float tempPitch = -(IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle - 5009) * 360 / 8192.0;
//			NORMALIZE_ANGLE180(tempPitch);
//			fw_printfln("PitchAngle= %f", tempPitch);
			//fw_printfln("ZGyroModuleAngle:  %f",ZGyroModuleAngle);
//			fw_printfln("YawAngle= %d", IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle);
//			fw_printfln("PitchAngle= %d", IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle);
			/*****查看任务栈空间剩余示例*******/
			//		StackResidue = uxTaskGetStackHighWaterMark( GMControlTaskHandle );
			//		fw_printfln("GM%ld",StackResidue);
			if(JUDGE_State == OFFLINE)
			{
				//fw_printfln("Judge not received");
			}
			else
			{
				//fw_printfln("Judge received");
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
	ShootMotorSpeedPID.Reset(&ShootMotorSpeedPID);
	CMRotatePID.Reset(&CMRotatePID);
	CM1SpeedPID.Reset(&CM1SpeedPID);
	CM2SpeedPID.Reset(&CM2SpeedPID);
	CM3SpeedPID.Reset(&CM3SpeedPID);
	CM4SpeedPID.Reset(&CM4SpeedPID);
}
/**********************************************************
*工作状态切换状态机
**********************************************************/

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
				g_workState = NORMAL_STATE;
			}			
		}break;
		case NORMAL_STATE:     
		{
			if(GetInputMode() == STOP )
			{
				g_workState = STOP_STATE;
			}
			//ZY
			else if(GetInputMode()==KEY_MOUSE_INPUT&&zyGetLeftPostion()==3)//&&(RC_CtrlData.rc.s1==3))
			{
				g_workState= RUNE_STATE;
			}
			//ZY
		}break;
		case STOP_STATE:   
		{
			if(GetInputMode() != STOP )
			{
				g_workState = PREPARE_STATE;   
			}
			else if(GetInputMode()==KEY_MOUSE_INPUT&&zyGetLeftPostion()==3)//&&(RC_CtrlData.rc.s1==3))
			{
				g_workState= RUNE_STATE;
			}
		}break;
		case RUNE_STATE:
		{
			if(zyGetLeftPostion()!=3)
			{
				g_workState = PREPARE_STATE;  
				//LASER_OFF();
				friction_wheel_stateZY = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
				SetShootState(NOSHOOTING);
			}
			else if(GetInputMode()==REMOTE_INPUT)
			{
				g_workState = PREPARE_STATE;
				if(friction_wheel_stateZY==FRICTION_WHEEL_ON||friction_wheel_stateZY==FRICTION_WHEEL_START_TURNNING)
				{
					//LASER_OFF();
					friction_wheel_stateZY = FRICTION_WHEEL_OFF;				  
					SetFrictionWheelSpeed(1000); 
					frictionRamp.ResetCounter(&frictionRamp);
					SetShootState(NOSHOOTING);
				}		
			}
			else if(GetInputMode() == STOP )
			{
				g_workState = STOP_STATE;
				if(friction_wheel_stateZY==FRICTION_WHEEL_ON||friction_wheel_stateZY==FRICTION_WHEEL_START_TURNNING)
				{
					//LASER_OFF();
					friction_wheel_stateZY = FRICTION_WHEEL_OFF;				  
					SetFrictionWheelSpeed(1000); 
					frictionRamp.ResetCounter(&frictionRamp);
					SetShootState(NOSHOOTING);
				}
			}
			else
			{
					switch(friction_wheel_stateZY)
					{
						case FRICTION_WHEEL_OFF:
						{
								SetShootState(NOSHOOTING);
								frictionRamp.ResetCounter(&frictionRamp);
								friction_wheel_stateZY = FRICTION_WHEEL_START_TURNNING;	 
								//LASER_ON(); 
						}break;
						case FRICTION_WHEEL_START_TURNNING:
						{
							
								/*摩擦轮转速修改 FRICTION_WHEEL_MAX_DUTY*/
								SetFrictionWheelSpeed(1000 + (FRICTION_WHEEL_MAX_DUTY-1000)*frictionRamp.Calc(&frictionRamp)); 
								if(frictionRamp.IsOverflow(&frictionRamp))
								{
									friction_wheel_stateZY = FRICTION_WHEEL_ON; 	
									//LASER_ON(); 
								}
								
						}
						break;
						case FRICTION_WHEEL_ON:
						{
							//SetShootState(on)				 
						} break;			
					}			
			}
		}break;
		default:
		{
			
		}
	}	
}

void WorkStateSwitchProcess(void)
{
	//如果从其他模式切换到prapare模式，要将一系列参数初始化
	if((lastWorkState != g_workState) && (g_workState == PREPARE_STATE))  
	{
		//计数初始化
	  s_time_tick_2ms = 0;   
		CMControlInit();
		RemoteTaskInit();
	}
}
  
void getJudgeState(void)
{
	if(JUDGE_Received==1)
	{
		s_count_judge = 0;
		JUDGE_State = ONLINE;
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
int32_t GetQuadEncoderDiff(void)
{
  int32_t cnt = 0;    
	cnt = __HAL_TIM_GET_COUNTER(&htim5) - 0x0;
	//fw_printfln("%x",cnt);
	 //__HAL_TIM_SET_COUNTER(&htim5, 0x7fff);
	return cnt;
}

int RotateAdd = 0;
int Stuck = 0;
int32_t last_fdb = 0x0;
int32_t this_fdb = 0;
void ShooterMControlLoop(void)	
{				      			
	if(GetShootState() == SHOOTING && GetInputMode()==KEY_MOUSE_INPUT && Stuck==0)
	{
		//ShootMotorPositionPID.ref = ShootMotorPositionPID.ref+OneShoot;//打一发弹编码器输出脉冲数
		ShootOneBullet();
	}

	//遥控器输入模式下，只要处于发射态，就一直转动
	if(GetShootState() == SHOOTING && GetInputMode() == REMOTE_INPUT && Stuck==0)
	{
		RotateAdd += 4;
		//fw_printfln("ref = %f",ShootMotorPositionPID.ref);
		if(RotateAdd>OneShoot)
		{
			//ShootMotorPositionPID.ref = ShootMotorPositionPID.ref+OneShoot;
			ShootOneBullet();
			RotateAdd = 0;
		}
	}
	else if(GetShootState() == NOSHOOTING && GetInputMode() == REMOTE_INPUT)
	{
		RotateAdd = 0;
	}

	if(GetFrictionState()==FRICTION_WHEEL_ON||friction_wheel_stateZY==FRICTION_WHEEL_ON)//拨盘转动前提条件：摩擦轮转动GetFrictionState()==FRICTION_WHEEL_ON,张雁加后面的条件
	{
		this_fdb = GetQuadEncoderDiff(); 
		//fw_printfln("this_fdb = %d",this_fdb);
		
		//卡弹处理
//		if(abs(ShootMotorPositionPID.ref-ShootMotorPositionPID.fdb)>100 && (abs(this_fdb-last_fdb)<5 || abs(this_fdb+65535-last_fdb)<5)) //认为卡弹
//		{//ShootMotorPositionPID.ref = ShootMotorPositionPID.ref - OneShoot;
//		}
	//	else
		//{
		if(this_fdb<last_fdb-10000 && getPlateMotorDir()==FORWARD)	//cnt寄存器溢出判断 正转
		{
			ShootMotorPositionPID.fdb = ShootMotorPositionPID.fdb+(65535+this_fdb-last_fdb);
		}
		else if(this_fdb>last_fdb+10000 && getPlateMotorDir()==REVERSE)	//cnt寄存器溢出判断 反转
		{
			ShootMotorPositionPID.fdb = ShootMotorPositionPID.fdb-(65535-this_fdb+last_fdb);
		}
		else
			ShootMotorPositionPID.fdb = ShootMotorPositionPID.fdb + this_fdb-last_fdb;
	//	}
		last_fdb = this_fdb;
		//fw_printfln("fdb = %f",ShootMotorPositionPID.fdb);
		ShootMotorPositionPID.Calc(&ShootMotorPositionPID);
//		ShootMotorSpeedPID.ref = ShootMotorPositionPID.output;
//		ShootMotorSpeedPID.fdb = 
		if(ShootMotorPositionPID.output<0) //反转
		{
			setPlateMotorDir(REVERSE);
			ShootMotorPositionPID.output = -ShootMotorPositionPID.output;
		}
		else
			setPlateMotorDir(FORWARD);
		
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ShootMotorPositionPID.output);
	}
	
	else
	{
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);//摩擦轮不转，立刻关闭拨盘
	}
}

