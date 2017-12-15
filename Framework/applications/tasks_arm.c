/**
  ******************************************************************************
  * File Name          : tasks_arm.c
  * Description        : CAN2电机控制任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 取弹机械臂电机控制任务
	* 处于阻塞态等待CAN接收任务释放信号量
	* 对CAN收到的数据进行PID计算，再将电流值发送到CAN
  ******************************************************************************
  */
	
#include "tasks_arm.h"
#include "rtos_semaphore.h"
#include "pid_regulator.h"
#include "utilities_iopool.h"
#include "drivers_canmotor_user.h"
#include "application_motorcontrol.h"
#include "utilities_debug.h"

//PID_INIT(Kp, Ki, Kd, KpMax, KiMax, KdMax, OutputMax)
//机械臂电机PID

fw_PID_Regulator_t AM1LPositionPID = fw_PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t AM1RPositionPID = fw_PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t AM2LPositionPID = fw_PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t AM2RPositionPID = fw_PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t AM3LPositionPID = fw_PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t AM1LSpeedPID = fw_PID_INIT(10.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4000.0);
fw_PID_Regulator_t AM1RSpeedPID = fw_PID_INIT(10.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4000.0);
fw_PID_Regulator_t AM2LSpeedPID = fw_PID_INIT(10.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4000.0);
fw_PID_Regulator_t AM2RSpeedPID = fw_PID_INIT(10.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4000.0);
fw_PID_Regulator_t AM3LSpeedPID = fw_PID_INIT(10.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4000.0);

////待标定
//#define AM1L_zero 0
//#define AM1R_zero 0
//#define AM2L_zero 0
//#define AM2R_zero 0
//#define AM3L_zero 0

//机械臂电机目标物理角度值
float AM1LAngleTarget = 0.0;
float AM1RAngleTarget = 0.0;
float AM2LAngleTarget = 0.0;
float AM2RAngleTarget = 0.0;
float AM3LAngleTarget = 0.0;

//机械臂电机实际物理角度值
float AM1LRealAngle = 0.0;
float AM1RRealAngle = 0.0;
float AM2LRealAngle = 0.0;
float AM2RRealAngle = 0.0;
float AM3LRealAngle = 0.0;

//uint16_t AM1LRawAngle = 0;
//uint16_t AM1RRawAngle = 0;
//uint16_t AM2LRawAngle = 0;
//uint16_t AM2RRawAngle = 0;
//uint16_t AM3LRawAngle = 0;

//用于减小系统开销
static uint8_t s_AM1LCount = 0;
static uint8_t s_AM1RCount = 0;
static uint8_t s_AM2LCount = 0;
static uint8_t s_AM2RCount = 0;
static uint8_t s_AM3LCount = 0;

void Can2ControlTask(void const * argument)
{
	while(1)
	{
		osSemaphoreWait(Can2RefreshSemaphoreHandle, osWaitForever);
		
		ControlAM1L();
		ControlAM1R();
		ControlAM2L();
		ControlAM2R();
		ControlAM3L();
	}
}


uint8_t isAM1LFirstEnter = 1;
uint16_t AM1LThisAngle = 0;
uint16_t AM1LLastAngle = 0;
void ControlAM1L()
{
	if(IOPool_hasNextRead(AM1LRxIOPool, 0))
	{
		if(s_AM1LCount == 1)
		{		
			IOPool_getNextRead(AM1LRxIOPool, 0);
			AM1LThisAngle = IOPool_pGetReadData(AM1LRxIOPool, 0)->angle;
			
			if(isAM1LFirstEnter==1) {AM1LLastAngle = AM1LThisAngle;isAM1LFirstEnter = 0;return;}	//初始化时，记录下当前编码器的值
			
			if(AM1LThisAngle<=AM1LLastAngle)
			{
				if((AM1LLastAngle-AM1LThisAngle)>3000)//编码器上溢
					AM1LRealAngle = AM1LRealAngle + (AM1LThisAngle+8192-AM1LLastAngle) * 360 / 8192.0 / AM1Reduction;
				else//反转
					AM1LRealAngle = AM1LRealAngle - (AM1LLastAngle - AM1LThisAngle) * 360 / 8192.0 / AM1Reduction;
			}
			else
			{
				if((AM1LThisAngle-AM1LLastAngle)>3000)//编码器下溢
					AM1LRealAngle = AM1LRealAngle - (AM1LLastAngle+8192-AM1LThisAngle) *360 / 8192.0 / AM1Reduction;
				else//正转
					AM1LRealAngle = AM1LRealAngle + (AM1LThisAngle - AM1LLastAngle) * 360 / 8192.0 / AM1Reduction;
			}
			
				
			AM1LPositionPID.target = AM1LAngleTarget;
			AM1LPositionPID.feedback = AM1LRealAngle;
			AM1LPositionPID.Calc(&AM1LPositionPID);
			
			AM1LSpeedPID.target = AM1LPositionPID.output;
			AM1LSpeedPID.feedback = IOPool_pGetReadData(AM1LRxIOPool, 0)->RotateSpeed;
			AM1LSpeedPID.Calc(&AM1LSpeedPID);
			
			setMotor(AM1L, AM1LSpeedPID.output);
			s_AM1LCount = 0;
			AM1LLastAngle = AM1LThisAngle;
		}
		else
		{
			s_AM1LCount++;
		}
	}
}

uint8_t isAM1RFirstEnter = 1;
uint16_t AM1RThisAngle = 0;
uint16_t AM1RLastAngle = 0;
void ControlAM1R()
{
	if(IOPool_hasNextRead(AM1RRxIOPool, 0))
	{
		if(s_AM1RCount == 1)
		{
			IOPool_getNextRead(AM1RRxIOPool, 0);
			AM1RThisAngle = IOPool_pGetReadData(AM1RRxIOPool, 0)->angle;
			
			if(isAM1RFirstEnter==1) {AM1RLastAngle = AM1RThisAngle;isAM1RFirstEnter = 0;return;}	//初始化时，记录下当前编码器的值
			
			if(AM1RThisAngle<=AM1RLastAngle)
			{
				if((AM1RLastAngle-AM1RThisAngle)>3000)//编码器上溢
					AM1RRealAngle = AM1RRealAngle + (AM1RThisAngle+8192-AM1RLastAngle) * 360 / 8192.0 / AM1Reduction;
				else//反转
					AM1RRealAngle = AM1RRealAngle - (AM1RLastAngle - AM1RThisAngle) * 360 / 8192.0 / AM1Reduction;
			}
			else
			{
				if((AM1RThisAngle-AM1RLastAngle)>3000)//编码器下溢
					AM1RRealAngle = AM1RRealAngle - (AM1RLastAngle+8192-AM1RThisAngle) *360 / 8192.0 / AM1Reduction;
				else//正转
					AM1RRealAngle = AM1RRealAngle + (AM1RThisAngle - AM1RLastAngle) * 360 / 8192.0 / AM1Reduction;
			}
			
			
			AM1RPositionPID.target = AM1RAngleTarget;
			AM1RPositionPID.feedback = AM1RRealAngle;
			AM1RPositionPID.Calc(&AM1RPositionPID);
			
			AM1RSpeedPID.target = AM1RPositionPID.output;
			AM1RSpeedPID.feedback = IOPool_pGetReadData(AM1RRxIOPool, 0)->RotateSpeed;
			AM1RSpeedPID.Calc(&AM1RSpeedPID);
			
			setMotor(AM1R, AM1RSpeedPID.output);
			s_AM1RCount = 0;
			AM1RLastAngle = AM1RThisAngle;
		}
		else
		{
			s_AM1RCount++;
		}
	}
}

uint8_t isAM2LFirstEnter = 1;
uint16_t AM2LThisAngle = 0;
uint16_t AM2LLastAngle = 0;
void ControlAM2L()
{
	if(IOPool_hasNextRead(AM2LRxIOPool, 0))
	{
		if(s_AM2LCount == 1)
		{
			IOPool_getNextRead(AM2LRxIOPool, 0);
			AM2LThisAngle = IOPool_pGetReadData(AM2LRxIOPool, 0)->angle;
			
			if(isAM2LFirstEnter==1) {AM2LLastAngle = AM2LThisAngle;isAM2LFirstEnter = 0;return;}	//初始化时，记录下当前编码器的值
			
			if(AM2LThisAngle<=AM2LLastAngle)
			{
				if((AM2LLastAngle-AM2LThisAngle)>3000)//编码器上溢
					AM2LRealAngle = AM2LRealAngle + (AM2LThisAngle+8192-AM2LLastAngle) * 360 / 8192.0 / AM23Reduction;
				else//反转
					AM2LRealAngle = AM2LRealAngle - (AM2LLastAngle - AM2LThisAngle) * 360 / 8192.0 / AM23Reduction;
			}
			else
			{
				if((AM2LThisAngle-AM2LLastAngle)>3000)//编码器下溢
					AM2LRealAngle = AM2LRealAngle - (AM2LLastAngle+8192-AM2LThisAngle) *360 / 8192.0 / AM23Reduction;
				else//正转
					AM2LRealAngle = AM2LRealAngle + (AM2LThisAngle - AM2LLastAngle) * 360 / 8192.0 / AM23Reduction;
			}
			
			
			AM2LPositionPID.target = AM2LAngleTarget;
			AM2LPositionPID.feedback = AM2LRealAngle;
			AM2LPositionPID.Calc(&AM2LPositionPID);
			
			AM2LSpeedPID.target = AM2LPositionPID.output;
			AM2LSpeedPID.feedback = IOPool_pGetReadData(AM2LRxIOPool, 0)->RotateSpeed;
			AM2LSpeedPID.Calc(&AM2LSpeedPID);
			
			setMotor(AM2L, AM2LSpeedPID.output);
			s_AM2LCount = 0;
			AM2LLastAngle = AM2LThisAngle;
		}
		else
		{
			s_AM2LCount++;
		}
	}
}

uint8_t isAM2RFirstEnter = 1;
uint16_t AM2RThisAngle = 0;
uint16_t AM2RLastAngle = 0;
void ControlAM2R()
{
	if(IOPool_hasNextRead(AM2RRxIOPool, 0))
	{
		if(s_AM2RCount == 1)
		{
			IOPool_getNextRead(AM2RRxIOPool, 0);
			AM2RThisAngle = IOPool_pGetReadData(AM2RRxIOPool, 0)->angle;
			
			if(isAM2RFirstEnter==1) {AM2RLastAngle = AM2RThisAngle;isAM2RFirstEnter = 0;return;}	//初始化时，记录下当前编码器的值
			
			if(AM2RThisAngle<=AM2RLastAngle)
			{
				if((AM2RLastAngle-AM2RThisAngle)>3000)//编码器上溢
					AM2RRealAngle = AM2RRealAngle + (AM2RThisAngle+8192-AM2RLastAngle) * 360 / 8192.0 / AM23Reduction;
				else//反转
					AM2RRealAngle = AM2RRealAngle - (AM2RLastAngle - AM2RThisAngle) * 360 / 8192.0 / AM23Reduction;
			}
			else
			{
				if((AM2RThisAngle-AM2RLastAngle)>3000)//编码器下溢
					AM2RRealAngle = AM2RRealAngle - (AM2RLastAngle+8192-AM2RThisAngle) *360 / 8192.0 / AM23Reduction;
				else//正转
					AM2RRealAngle = AM2RRealAngle + (AM2RThisAngle - AM2RLastAngle) * 360 / 8192.0 / AM23Reduction;
			}
			
			
			AM2RPositionPID.target = AM2RAngleTarget;
			AM2RPositionPID.feedback = AM2RRealAngle;
			AM2RPositionPID.Calc(&AM2RPositionPID);
			
			AM2RSpeedPID.target = AM2RPositionPID.output;
			AM2RSpeedPID.feedback = IOPool_pGetReadData(AM2RRxIOPool, 0)->RotateSpeed;
			AM2RSpeedPID.Calc(&AM2RSpeedPID);
			
			setMotor(AM2R, AM2RSpeedPID.output);
			s_AM1RCount = 0;
			AM2RLastAngle = AM2RThisAngle;
		}
		else
		{
			s_AM2RCount++;
		}
	}
}

uint8_t isAM3LFirstEnter = 1;
uint16_t AM3LThisAngle = 0;
uint16_t AM3LLastAngle = 0;
void ControlAM3L()
{
	if(IOPool_hasNextRead(AM3LRxIOPool, 0))
	{
		if(s_AM3LCount == 1)
		{
			IOPool_getNextRead(AM3LRxIOPool, 0);
			AM3LThisAngle = IOPool_pGetReadData(AM3LRxIOPool, 0)->angle;
			
			if(isAM3LFirstEnter==1) {AM3LLastAngle = AM3LThisAngle;isAM3LFirstEnter = 0;return;}	//初始化时，记录下当前编码器的值
			
			if(AM3LThisAngle<=AM3LLastAngle)
			{
				if((AM3LLastAngle-AM3LThisAngle)>3000)//编码器上溢
					AM3LRealAngle = AM3LRealAngle + (AM3LThisAngle+8192-AM3LLastAngle) * 360 / 8192.0 / AM23Reduction;
				else//反转
					AM3LRealAngle = AM3LRealAngle - (AM3LLastAngle - AM3LThisAngle) * 360 / 8192.0 / AM23Reduction;
			}
			else
			{
				if((AM3LThisAngle-AM3LLastAngle)>3000)//编码器下溢
					AM3LRealAngle = AM3LRealAngle - (AM3LLastAngle+8192-AM3LThisAngle) *360 / 8192.0 / AM23Reduction;
				else//正转
					AM3LRealAngle = AM3LRealAngle + (AM3LThisAngle - AM3LLastAngle) * 360 / 8192.0 / AM23Reduction;
			}
			
			
			AM3LPositionPID.target = AM3LAngleTarget;
			AM3LPositionPID.feedback = AM3LRealAngle;
			AM3LPositionPID.Calc(&AM3LPositionPID);
			
			AM3LSpeedPID.target = AM3LPositionPID.output;
			AM3LSpeedPID.feedback = IOPool_pGetReadData(AM3LRxIOPool, 0)->RotateSpeed;
			AM3LSpeedPID.Calc(&AM3LSpeedPID);
			
			setMotor(AM3L, AM3LSpeedPID.output);
			s_AM3LCount = 0;
			AM3LLastAngle = AM3LThisAngle;
		}
		else
		{
			s_AM3LCount++;
		}
	}
}


void setAMAngle(MotorId id, float angle)
{
	switch(id){
		case AM1L:
			AM1LAngleTarget = angle;break;
		case AM1R:
			AM1RAngleTarget = angle;break;
		case AM2L:
			AM2LAngleTarget = angle;break;
		case AM2R:
			AM2RAngleTarget = angle;break;
		case AM3L:
			AM3LAngleTarget = angle;break;
		default:
			fw_Error_Handler();
	}
}

//一整套动作为下达取弹指令后，展开机械臂，进行取弹，取弹完毕后下达收回指令，机械臂收回
void getGolf()
{
	//待完善
	//思路：
	//目前两个电机的task都是用于跟踪的，即令反馈值跟踪上目标值
	//在遥控器任务和2ms定时任务中对目标值进行修改，从而做出指定的动作
	//取弹flag置位，当2ms任务检测到置位后，计数器开始计数，根据计数器的值来进行相应的取弹动作
}

void armReset()
{
	//待完善
	//思路：
	//取弹flag清零，回收flag置位，具体动作由2ms定时器任务完成，完成后flag清零
}
