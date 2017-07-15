/**
  ******************************************************************************
  * File Name          : tasks_motor.c
  * Description        : 电机控制任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 云台、底盘电机控制任务
	* 处于阻塞态等待CAN接收任务释放信号量
	* 对CAN收到的数据进行PID计算，再将电流值发送到CAN
  ******************************************************************************
  */
#include "tasks_motor.h"
#include "drivers_canmotor_user.h"
#include "rtos_semaphore.h"
#include "drivers_uartrc_user.h"
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <usart.h>
#include "utilities_debug.h"
#include "tasks_upper.h"
#include "tasks_timed.h"
#include "tasks_remotecontrol.h"
#include "drivers_led_user.h"
#include "utilities_minmax.h"
#include "pid_regulator.h"
#include "application_motorcontrol.h"
#include "drivers_sonar_user.h"
#include "peripheral_define.h"
#include "drivers_uartupper_user.h"


//PID_INIT(Kp, Ki, Kd, KpMax, KiMax, KdMax, OutputMax)
//云台PID
#ifdef INFANTRY_5
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(15.0, 2.0, 1.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(5.3, -1.0, 0.5, 10000.0, 10000.0, 10000.0, 10000.0);//等幅振荡P37.3 I11.9 D3.75  原26.1 8.0 1.1
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(25.0, 0.0, 5.0, 10000.0, 10000.0, 10000.0, 3500.0);
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(40.0, 0.0, 20, 10000.0, 10000.0, 10000.0, 4000.0);
//手动标定0点
#define yaw_zero 614
#define pitch_zero 2924
#endif
#ifdef INFANTRY_4
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(4.0, 0, 2.5, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(5.3, -1.0, 0.5, 10000.0, 10000.0, 10000.0, 10000.0);//等幅振荡P37.3 I11.9 D3.75  原26.1 8.0 1.1
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(25.0, 0.0, 5.0, 10000.0, 10000.0, 10000.0, 3500.0);
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(40.0, 0.0, 20, 10000.0, 10000.0, 10000.0, 4000.0);
#define yaw_zero 1445//720
#define pitch_zero 5009 //5003
#endif
#ifdef INFANTRY_1
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(15.0, 0.0, 0.5, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(5.3, -1.0, 1.5, 10000.0, 10000.0, 10000.0, 10000.0);//等幅振荡P37.3 I11.9 D3.75  原26.1 8.0 1.1
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(35.0, 0.0, 2.0, 10000.0, 10000.0, 10000.0, 3500.0);
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(40.0, 0.0, 20, 10000.0, 10000.0, 10000.0, 4000.0);
#define yaw_zero 175//1040
#define pitch_zero 6000
#endif
//底盘速度PID
PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

extern uint8_t g_isGYRO_Rested;
//陀螺仪角速度
extern float gYroXs, gYroYs, gYroZs;
//外接单轴陀螺仪角度
extern float ZGyroModuleAngle;
float yawAngleTarget = 0.0;
float gap_angle = 0.0;

float pitchRealAngle = 0.0;
float pitchAngleTarget = 0.0;

//大神符、自瞄
extern Location_Number_s Location_Number[];
extern uint8_t CReceive;
extern uint8_t rune_flag;
static float yawAdd;
static float pitchAdd;
static uint8_t rune;
//扭腰
int twist_state = 0;
int twist_count = 0;
int twist =0;
float mm =0;
float nn =0;
int16_t twist_target = 0;

extern float zyYawTarget,zyPitchTarget;
float yawRealAngle = 0.0;//张雁调试大符

void CMGMControlTask(void const * argument){
	while(1)
	{
		//等待CAN接收回调函数信号量
		osSemaphoreWait(CMGMCanRefreshSemaphoreHandle, osWaitForever);
		
		UpdateFromManifold();
		ControlYaw();
		ControlPitch();
		
	  ControlRotate();
//		ChassisSpeedRef.rotate_ref = 0;//取消底盘跟随
		ControlCMFL();
		ControlCMFR();
		ControlCMBL();
		ControlCMBR();
	}//end of while
}



/*从妙算通信串口任务获得数据*/
void UpdateFromManifold()
{
	if(IOPool_hasNextRead(upperIOPool, 0))
	{
		IOPool_getNextRead(upperIOPool, 0);
		yawAdd = IOPool_pGetReadData(upperIOPool, 0)->yawAdd;
		pitchAdd = IOPool_pGetReadData(upperIOPool, 0)->pitchAdd;
		rune = IOPool_pGetReadData(upperIOPool, 0)->rune;
	}
}
/*Yaw电机*/
void ControlYaw(void)
{
	if(IOPool_hasNextRead(GMYAWRxIOPool, 0))
	{
		uint16_t yawZeroAngle = yaw_zero;
		//float yawRealAngle = 0.0;张雁改全局
		int16_t yawIntensity = 0;		
		
		/*从IOPool读编码器*/
		IOPool_getNextRead(GMYAWRxIOPool, 0); 
		yawRealAngle = (IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle - yawZeroAngle) * 360 / 8192.0f;
		NORMALIZE_ANGLE180(yawRealAngle);
		
		if(GetWorkState() == NORMAL_STATE) 
		{
			yawRealAngle = -ZGyroModuleAngle;//yawrealangle的值改为复位后陀螺仪的绝对值，进行yaw轴运动设定
			/*自瞄模式切换*/
			
//			if(GetShootMode() == AUTO) 
//			{
//				ChassisSpeedRef.rotate_ref = 0;
//				if((GetLocateState() == Located))
//				{
//				ChassisSpeedRef.rotate_ref = 0;
//				}
//				if((GetLocateState() == Locating) && (CReceive != 0))	
//				{
//				yawAngleTarget = yawRealAngle - yawAdd ;
//				fw_printfln("yawAdd-in control:%f",yawAdd );
//				CReceive--;
//				}
//				//大神符
//				else if((GetLocateState() == Located) && (rune_flag != 0))
//				{
//					if(GetRuneState() == AIMING)
//					{
//						fw_printfln("rune:%d", rune);
//						yawAngleTarget = Location_Number[rune - 1].yaw_position;
//						rune_flag--;
//					}
//				}
//			}//end of autoshoot
		}
		else if(GetWorkState()==RUNE_STATE)
		{
			//fw_printfln("Rune State:%f",yawAngleTarget);
			//yawAngleTarget=zyYawTartet;
			//yawRealAngle = -ZGyroModuleAngle;
		}
						
		yawIntensity = ProcessYawPID(yawAngleTarget, yawRealAngle, -gYroZs);
		setMotor(GMYAW, yawIntensity);
	}
}
/*Pitch电机*/
void ControlPitch(void)
{
	if(IOPool_hasNextRead(GMPITCHRxIOPool, 0))
	{
		uint16_t pitchZeroAngle = pitch_zero;
		int16_t pitchIntensity = 0;
		
		IOPool_getNextRead(GMPITCHRxIOPool, 0);
		pitchRealAngle = -(IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle - pitchZeroAngle) * 360 / 8192.0;
		NORMALIZE_ANGLE180(pitchRealAngle);
		
		//自瞄模式切换
//		if(GetShootMode() == AUTO) 
//		{
//			if((GetLocateState() == Locating) && (CReceive != 0))	
//			{
//				//fw_printfln("pitchAdd:%f",pitchAdd );
//				pitchAngleTarget = pitchRealAngle + pitchAdd ;
//				CReceive --;
//			}//大神符
//			else if((GetLocateState() == Located) && (rune_flag != 0))
//			{
//				if(GetRuneState() == AIMING)
//				{
//					pitchAngleTarget = Location_Number[rune - 1].pitch_position;
//					rune_flag--;
//				}
//			}
//		}
		if(GetWorkState()==RUNE_STATE)
		{
			//fw_printfln("Rune State:%d",1);
		}
		
		#ifdef INFANTRY_5
		MINMAX(pitchAngleTarget, -10.0f, 26.3f);
		#endif
		#ifdef INFANTRY_4
		MINMAX(pitchAngleTarget, -18.f, 30);
		#endif
		#ifdef INFANTRY_1
		MINMAX(pitchAngleTarget, -9.0f, 32);
		#endif
		
		pitchIntensity = ProcessPitchPID(pitchAngleTarget,pitchRealAngle,-gYroXs);
		setMotor(GMPITCH, pitchIntensity);
	}
}
/*底盘转动控制：跟随云台/扭腰等*/
void ControlRotate(void)
{
	gap_angle  = (IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle - yaw_zero) * 360 / 8192.0f;
    NORMALIZE_ANGLE180(gap_angle);	
	
	if(GetWorkState() == NORMAL_STATE) 
	{
		/*扭腰*/
		//试图用PID
		if (twist_state == 1)
		{
			CMRotatePID.output = 0; //一定角度之间进行扭腰
			twist = (twist_count / 600)%2 ;	
			if (twist == nn){
				CMRotatePID.output = -10;
				twist_count = twist_count + 1;
			}
			if (twist == (1-nn)){
				CMRotatePID.output = 10;
				twist_count = twist_count + 1;
			}
			 ChassisSpeedRef.rotate_ref = CMRotatePID.output;
		}				
		else
		{
			/*产生扭腰随机数*/  
			 srand(xTaskGetTickCount());
			 mm = (1.0f*rand()/RAND_MAX);//产生随机方向
			 nn = floor(2.0f*mm);
					
			/*底盘跟随编码器旋转PID计算*/		
			 CMRotatePID.ref = 0;
			 CMRotatePID.fdb = gap_angle;
			 CMRotatePID.Calc(&CMRotatePID);   
			 ChassisSpeedRef.rotate_ref = CMRotatePID.output;
		}
	}
}
/*底盘电机控制FL(ForwardLeft)FR BL BR*/
void ControlCMFL(void)
{		
	if(IOPool_hasNextRead(CMFLRxIOPool, 0))
	{
		IOPool_getNextRead(CMFLRxIOPool, 0);
		Motor820RRxMsg_t *pData = IOPool_pGetReadData(CMFLRxIOPool, 0);
		
		CM2SpeedPID.ref = - ChassisSpeedRef.forward_back_ref*0.075 
										 + ChassisSpeedRef.left_right_ref*0.075 
										 + ChassisSpeedRef.rotate_ref;
		CM2SpeedPID.ref = 160 * CM2SpeedPID.ref;
		CM2SpeedPID.fdb = pData->RotateSpeed;
		#ifdef INFANTRY_1
		CM2SpeedPID.ref = 1.2f * CM2SpeedPID.ref;
		#endif
		CM2SpeedPID.Calc(&CM2SpeedPID);
		
		setMotor(CMFR, CHASSIS_SPEED_ATTENUATION * CM2SpeedPID.output);
	}
}

void ControlCMFR(void)
{
	if(IOPool_hasNextRead(CMFRRxIOPool, 0))
	{
		IOPool_getNextRead(CMFRRxIOPool, 0);
		Motor820RRxMsg_t *pData = IOPool_pGetReadData(CMFRRxIOPool, 0);
		
		CM1SpeedPID.ref =  ChassisSpeedRef.forward_back_ref*0.075 
										 + ChassisSpeedRef.left_right_ref*0.075 
										 + ChassisSpeedRef.rotate_ref;	
		CM1SpeedPID.ref = 160 * CM1SpeedPID.ref;
		CM1SpeedPID.fdb = pData->RotateSpeed;
		#ifdef INFANTRY_1
		CM1SpeedPID.ref = 1.2f * CM1SpeedPID.ref;
		#endif
		CM1SpeedPID.Calc(&CM1SpeedPID);
		
		setMotor(CMFL, CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output);
	}
}
	
void ControlCMBL(void)
{
	if(IOPool_hasNextRead(CMBLRxIOPool, 0))
	{
		IOPool_getNextRead(CMBLRxIOPool, 0);
		Motor820RRxMsg_t *pData = IOPool_pGetReadData(CMBLRxIOPool, 0);
		
		CM3SpeedPID.ref =  ChassisSpeedRef.forward_back_ref*0.075 
										 - ChassisSpeedRef.left_right_ref*0.075 
										 + ChassisSpeedRef.rotate_ref;
		CM3SpeedPID.ref = 160 * CM3SpeedPID.ref;
		CM3SpeedPID.fdb = pData->RotateSpeed;
		#ifdef INFANTRY_1
		CM3SpeedPID.ref = 1.2f * CM3SpeedPID.ref;
		#endif
		CM3SpeedPID.Calc(&CM3SpeedPID);
		
		setMotor(CMBL, CHASSIS_SPEED_ATTENUATION * CM3SpeedPID.output);
	}
}

void ControlCMBR()
{
	if(IOPool_hasNextRead(CMBRRxIOPool, 0))
	{
		IOPool_getNextRead(CMBRRxIOPool, 0);
		Motor820RRxMsg_t *pData = IOPool_pGetReadData(CMBRRxIOPool, 0);
		
		CM4SpeedPID.ref = - ChassisSpeedRef.forward_back_ref*0.075 
										 - ChassisSpeedRef.left_right_ref*0.075 
										 + ChassisSpeedRef.rotate_ref;
		CM4SpeedPID.ref = 160 * CM4SpeedPID.ref;
		CM4SpeedPID.fdb = pData->RotateSpeed;
		#ifdef INFANTRY_1
		CM4SpeedPID.ref = 1.2f * CM4SpeedPID.ref;
		#endif
		CM4SpeedPID.Calc(&CM4SpeedPID);
		
		setMotor(CMBR, CHASSIS_SPEED_ATTENUATION * CM4SpeedPID.output);
	}
}

