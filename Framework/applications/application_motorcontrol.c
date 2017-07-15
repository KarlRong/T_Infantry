/**
  ******************************************************************************
  * File Name          : application_motorcontrol.c
  * Description        : 电机控制驱动函数
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 设定电机电流
	* 陀螺仪复位
  ******************************************************************************
  */
#include "application_motorcontrol.h"
#include "drivers_uartrc_user.h"
#include "can.h"
#include "peripheral_define.h"
#include "drivers_canmotor_user.h"
#include "rtos_semaphore.h"
#include "utilities_debug.h"
#include "tasks_timed.h"
#include "math.h"
#include <math.h>
#include <stdlib.h>
#include "tasks_motor.h"
#include "drivers_uartjudge_low.h"
#include "drivers_cmpower.h"

extern tGameInfo mytGameInfo;
extern uint8_t JUDGE_State;

void setMotor(MotorId motorId, int16_t Intensity){
	static int16_t CMFLIntensity = 0, CMFRIntensity = 0, CMBLIntensity = 0, CMBRIntensity = 0;
	static int8_t CMReady = 0;
	
	static int16_t GMYAWIntensity = 0, GMPITCHIntensity = 0;
	static int8_t GMReady = 0;
	
	switch(motorId)
	{
		case CMFL:
			if(CMReady & 0x1){CMReady = 0xF;}else{CMReady |= 0x1;}
			CMFLIntensity = Intensity;break;
		case CMFR:
			if(CMReady & 0x2){CMReady = 0xF;}else{CMReady |= 0x2;}
			CMFRIntensity = Intensity;break;
		case CMBL:
			if(CMReady & 0x4){CMReady = 0xF;}else{CMReady |= 0x4;}
			CMBLIntensity = Intensity;break;
		case CMBR:
			if(CMReady & 0x8){CMReady = 0xF;}else{CMReady |= 0x8;}
			CMBRIntensity = Intensity;break;
		
		case GMYAW:
			if(GMReady & 0x1){GMReady = 0x3;}else{GMReady |= 0x1;}
			GMYAWIntensity = Intensity;break;
		case GMPITCH:
			if(GMReady & 0x2){GMReady = 0x3;}else{GMReady |= 0x2;}
			GMPITCHIntensity = Intensity;break;
			
		default:
			fw_Error_Handler();
	}
	
	//底盘功率限制，80W，能量槽满60，低于0掉血
    RestrictPower(&CMFLIntensity, &CMFRIntensity, &CMBLIntensity, &CMBRIntensity);
	
	if(GetWorkState() == STOP_STATE)
	{
		CMFLIntensity = 0;
		CMFRIntensity = 0;
		CMBLIntensity = 0;
		CMBRIntensity = 0;
		GMYAWIntensity = 0;
		GMPITCHIntensity = 0;
	}

	if(CMReady == 0xF)
	{
		CanTxMsgTypeDef *pData = IOPool_pGetWriteData(CMTxIOPool);
		
		pData->StdId = CM_TXID;
  	pData->Data[0] = (uint8_t)(CMFLIntensity >> 8);
		pData->Data[1] = (uint8_t)CMFLIntensity;
		pData->Data[2] = (uint8_t)(CMFRIntensity >> 8);
		pData->Data[3] = (uint8_t)CMFRIntensity;
		pData->Data[4] = (uint8_t)(CMBLIntensity >> 8);
		pData->Data[5] = (uint8_t)CMBLIntensity;
		pData->Data[6] = (uint8_t)(CMBRIntensity >> 8);
		pData->Data[7] = (uint8_t)CMBRIntensity;
 	
		IOPool_getNextWrite(CMTxIOPool);
		
		TransmitCMGMCan();
		CMReady = 0;
    }
	
	if(GMReady == 0x3)
	{
		CanTxMsgTypeDef *pData = IOPool_pGetWriteData(GMTxIOPool);
		pData->StdId = GM_TXID;
		pData->Data[0] = (uint8_t)(GMYAWIntensity >> 8);
		pData->Data[1] = (uint8_t)GMYAWIntensity;
		pData->Data[2] = (uint8_t)(GMPITCHIntensity >> 8);
		pData->Data[3] = (uint8_t)GMPITCHIntensity;
		pData->Data[4] = 0;
		pData->Data[5] = 0;
		pData->Data[6] = 0;
		pData->Data[7] = 0;
		IOPool_getNextWrite(GMTxIOPool);
		GMReady = 0;

    TransmitCMGMCan();
	}
}
	

void GYRO_RST(void)
{
	CanTxMsgTypeDef *pData = IOPool_pGetWriteData(ZGYROTxIOPool);
	pData->StdId = ZGYRO_TXID;
	pData->Data[0] = 0x00;
	pData->Data[1] = 0x01;
	pData->Data[2] = 0x02;
	pData->Data[3] = 0x03;
	pData->Data[4] = 0x04;
	pData->Data[5] = 0x05;
	pData->Data[6] = 0x06;
	pData->Data[7] = 0x07;
	IOPool_getNextWrite(ZGYROTxIOPool);

	TransmitGYROCAN();
}
