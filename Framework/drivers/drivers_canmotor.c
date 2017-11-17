/**
  ******************************************************************************
  * File Name          : drivers_canmotor.c
  * Description        : 电机CAN总线驱动函数
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * CAN总线初始化
	* CAN接收处理
	* CAN发送任务
	* CAN1 CMGM电机
	* CAN2 单轴陀螺仪
  ******************************************************************************
  */
#include <cmsis_os.h>
#include "drivers_canmotor_low.h"
#include "drivers_canmotor_user.h"
#include "peripheral_define.h"
#include "utilities_debug.h"
#include "utilities_iopool.h"
#include "rtos_init.h"
#include "rtos_semaphore.h"


float ZGyroModuleAngle = 0.0f;

//RxIOPool
NaiveIOPoolDefine(CMFLRxIOPool, {0});
NaiveIOPoolDefine(CMFRRxIOPool, {0});
NaiveIOPoolDefine(CMBLRxIOPool, {0});
NaiveIOPoolDefine(CMBRRxIOPool, {0});

NaiveIOPoolDefine(GMPITCHRxIOPool, {0});
NaiveIOPoolDefine(GMYAWRxIOPool, {0});

//TxIOPool
#define DataPoolInit \
	{ \
		{CM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{CM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{CM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
NaiveIOPoolDefine(CMTxIOPool, DataPoolInit);
#undef DataPoolInit 
	
#define DataPoolInit \
	{ \
		{GM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{GM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{GM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
NaiveIOPoolDefine(GMTxIOPool, DataPoolInit);
#undef DataPoolInit 

#define DataPoolInit \
	{ \
		{ZGYRO_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{ZGYRO_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{ZGYRO_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
NaiveIOPoolDefine(ZGYROTxIOPool, DataPoolInit);
#undef DataPoolInit 

#define CanRxGetU16(canRxMsg, num) (((uint16_t)canRxMsg.Data[num * 2] << 8) | (uint16_t)canRxMsg.Data[num * 2 + 1])

uint8_t isRcanStarted_CMGM = 0, isRcanStarted_ZGYRO = 0;

CanRxMsgTypeDef CMGMCanRxMsg, ZGYROCanRxMsg;
/********************CAN接收******************************/
void InitCanReception()
{
	//CAN接收过滤器配置
	//http://www.eeworld.com.cn/mcu/article_2016122732674_3.html
	CMGMMOTOR_CAN.pRxMsg = &CMGMCanRxMsg;
	/*##-- Configure the CAN2 Filter ###########################################*/
	CAN_FilterConfTypeDef  sFilterConfig;
	sFilterConfig.FilterNumber = 0;//14 - 27//14
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;
  HAL_CAN_ConfigFilter(&CMGMMOTOR_CAN, &sFilterConfig);
	if(HAL_CAN_Receive_IT(&CMGMMOTOR_CAN, CAN_FIFO0) != HAL_OK){
		fw_Error_Handler(); 
	}
	isRcanStarted_CMGM = 1;
	
	ZGYRO_CAN.pRxMsg = &ZGYROCanRxMsg;
	/*##-- Configure the CAN2 Filter ###########################################*/
	CAN_FilterConfTypeDef sFilterConfig2;
	sFilterConfig2.FilterNumber = 14;//14 - 27//14
	sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig2.FilterIdHigh = 0x0000;
  sFilterConfig2.FilterIdLow = 0x0000;
  sFilterConfig2.FilterMaskIdHigh = 0x0000;
  sFilterConfig2.FilterMaskIdLow = 0x0000;
  sFilterConfig2.FilterFIFOAssignment = 0;
  sFilterConfig2.FilterActivation = ENABLE;
  sFilterConfig2.BankNumber = 14;
  HAL_CAN_ConfigFilter(&ZGYRO_CAN, &sFilterConfig2);
	if(HAL_CAN_Receive_IT(&ZGYRO_CAN, CAN_FIFO0) != HAL_OK){
		fw_Error_Handler(); 
	}
	isRcanStarted_ZGYRO = 1;
}
/*********************所有CAN接收终端****************************/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	if(hcan == &CMGMMOTOR_CAN){
		switch(CMGMCanRxMsg.StdId){
			case CMFL_RXID:
				//读取0 1字节为角度，2 3字节为速度
				IOPool_pGetWriteData(CMFLRxIOPool)->angle = CanRxGetU16(CMGMCanRxMsg, 0);
				IOPool_pGetWriteData(CMFLRxIOPool)->RotateSpeed = CanRxGetU16(CMGMCanRxMsg, 1);
				IOPool_getNextWrite(CMFLRxIOPool);
				break;
			case CMFR_RXID:
				IOPool_pGetWriteData(CMFRRxIOPool)->angle = CanRxGetU16(CMGMCanRxMsg, 0);
				IOPool_pGetWriteData(CMFRRxIOPool)->RotateSpeed = CanRxGetU16(CMGMCanRxMsg, 1);
				IOPool_getNextWrite(CMFRRxIOPool);
				break;
			case CMBL_RXID:
				IOPool_pGetWriteData(CMBLRxIOPool)->angle = CanRxGetU16(CMGMCanRxMsg, 0);
				IOPool_pGetWriteData(CMBLRxIOPool)->RotateSpeed = CanRxGetU16(CMGMCanRxMsg, 1);
				IOPool_getNextWrite(CMBLRxIOPool);
				break;
			case CMBR_RXID:
				IOPool_pGetWriteData(CMBRRxIOPool)->angle = CanRxGetU16(CMGMCanRxMsg, 0);
				IOPool_pGetWriteData(CMBRRxIOPool)->RotateSpeed = CanRxGetU16(CMGMCanRxMsg, 1);
				IOPool_getNextWrite(CMBRRxIOPool);
				break;
			case GMYAW_RXID:
				IOPool_pGetWriteData(GMYAWRxIOPool)->angle = CanRxGetU16(CMGMCanRxMsg, 0);
				IOPool_pGetWriteData(GMYAWRxIOPool)->realIntensity = CanRxGetU16(CMGMCanRxMsg, 1);
				IOPool_pGetWriteData(GMYAWRxIOPool)->giveIntensity = CanRxGetU16(CMGMCanRxMsg, 2);
				IOPool_getNextWrite(GMYAWRxIOPool);
				break;
			case GMPITCH_RXID:
				IOPool_pGetWriteData(GMPITCHRxIOPool)->angle = CanRxGetU16(CMGMCanRxMsg, 0);
				IOPool_pGetWriteData(GMPITCHRxIOPool)->realIntensity = CanRxGetU16(CMGMCanRxMsg, 1);
				IOPool_pGetWriteData(GMPITCHRxIOPool)->giveIntensity = CanRxGetU16(CMGMCanRxMsg, 2);
				IOPool_getNextWrite(GMPITCHRxIOPool);
				break;
			default:
			fw_Error_Handler();
		}
		//HAL CAN总线存在一定bug，使用自己的标志位
		if(HAL_CAN_Receive_IT(&CMGMMOTOR_CAN, CAN_FIFO0) != HAL_OK){
			fw_Warning();
			isRcanStarted_CMGM = 0;
		}else{
			isRcanStarted_CMGM = 1;
		}
		if(g_bInited == 1){
			//释放信号量交给控制任务
			osSemaphoreRelease(CMGMCanRefreshSemaphoreHandle);
		}
	}
	else if(hcan == &ZGYRO_CAN)
	{
		switch(ZGYROCanRxMsg.StdId)
		{
			case ZGYRO_RXID:
			 {
				//单轴陀螺仪没有datasheet，完全参照官方程序
				 //解算成角度值
				CanRxMsgTypeDef *msg = &ZGYROCanRxMsg;
				ZGyroModuleAngle = -0.01f*((int32_t)(msg->Data[0]<<24)|(int32_t)(msg->Data[1]<<16) | (int32_t)(msg->Data[2]<<8) | (int32_t)(msg->Data[3])); 
			 }
			 break;
			default:
			fw_Error_Handler();
		}
		if(HAL_CAN_Receive_IT(&ZGYRO_CAN, CAN_FIFO0) != HAL_OK)
		{
			//fw_Warning();
			isRcanStarted_ZGYRO = 0;
		}else{
			isRcanStarted_ZGYRO = 1;
		}
		if(g_bInited == 1){
			//这个信号量并没有人理0.o
			osSemaphoreRelease(ZGYROCanRefreshSemaphoreHandle);
		}
	}
}
/********************CAN发送*****************************/
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan)
{
	if(hcan == &CMGMMOTOR_CAN){
		osSemaphoreRelease(CMGMCanTransmitSemaphoreHandle);
	}
	else if(hcan == &ZGYRO_CAN)
	{
		osSemaphoreRelease(ZGYROCanTransmitSemaphoreHandle);
	}
}

void TransmitCMGMCan(void)
{
		if(IOPool_hasNextRead(CMTxIOPool, 0))
		{
			//使用信号量保护CAN发送资源，在发送中断中release
			osSemaphoreWait(CMGMCanTransmitSemaphoreHandle, osWaitForever);
			
			IOPool_getNextRead(CMTxIOPool, 0);
			CMGMMOTOR_CAN.pTxMsg = IOPool_pGetReadData(CMTxIOPool, 0);
			//使用临界区避免被抢占
			taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&CMGMMOTOR_CAN) != HAL_OK){
				fw_Warning();
			}
			taskEXIT_CRITICAL();
		}
		
		if(IOPool_hasNextRead(GMTxIOPool, 0))
		{
			osSemaphoreWait(CMGMCanTransmitSemaphoreHandle, osWaitForever);
			
			IOPool_getNextRead(GMTxIOPool, 0);
			CMGMMOTOR_CAN.pTxMsg = IOPool_pGetReadData(GMTxIOPool, 0);
			
			taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&CMGMMOTOR_CAN) != HAL_OK){
				fw_Warning();
			}
			taskEXIT_CRITICAL();
		}
}

void TransmitGYROCAN(void){
	if(IOPool_hasNextRead(ZGYROTxIOPool, 0))
	{
			osSemaphoreWait(ZGYROCanTransmitSemaphoreHandle, osWaitForever);
		
			IOPool_getNextRead(ZGYROTxIOPool, 0);
			ZGYRO_CAN.pTxMsg = IOPool_pGetReadData(ZGYROTxIOPool, 0);
		
			taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&ZGYRO_CAN) != HAL_OK)
			{
				fw_Warning();
			}
			taskEXIT_CRITICAL();
	}
}




/*
***********************************************************************************************
*Name          :EncoderProcess
*Input         :can message
*Return        :void
*Description   :对编码器数据进行处理得到速度
*               copy from官方程序
*								步兵未用到
*								值得参考
*
*
***********************************************************************************************
*/
/*
void EncoderProcess(volatile Encoder *v, Motor820RRxMsg_t * msg)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = msg->angle;
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -7500)    
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>7500)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}

	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);					
}

void GetEncoderBias(volatile Encoder *v, Motor820RRxMsg_t * msg)
{
	v->ecd_bias = msg->angle;  
	v->ecd_value = v->ecd_bias;
	v->last_raw_value = v->ecd_bias;
	v->temp_count++;
}
*/
