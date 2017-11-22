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

NaiveIOPoolDefine(AM1LRxIOPool, {0});
NaiveIOPoolDefine(AM1RRxIOPool, {0});
NaiveIOPoolDefine(AM2LRxIOPool, {0});
NaiveIOPoolDefine(AM2RRxIOPool, {0});
NaiveIOPoolDefine(AM3LRxIOPool, {0});

NaiveIOPoolDefine(PM1RxIOPool, {0});
NaiveIOPoolDefine(PM2RxIOPool, {0});

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
		{AM1_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{AM1_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{AM1_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
NaiveIOPoolDefine(AM1TxIOPool, DataPoolInit);
#undef DataPoolInit 
	
#define DataPoolInit \
	{ \
		{AM23_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{AM23_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{AM23_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
}
NaiveIOPoolDefine(AM23TxIOPool, DataPoolInit);
#undef DataPoolInit 

#define DataPoolInit \
	{ \
		{PM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{PM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{PM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
}
NaiveIOPoolDefine(PMTxIOPool, DataPoolInit);
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

uint8_t isRcanStarted_CAN1 = 0, isRcanStarted_CAN2 = 0;

CanRxMsgTypeDef Can1RxMsg, Can2RxMsg;
/********************CAN接收******************************/
void InitCanReception()
{
	//CAN接收过滤器配置
	//http://www.eeworld.com.cn/mcu/article_2016122732674_3.html
	hcan1.pRxMsg = &Can1RxMsg;
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
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	if(HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK){
		fw_Error_Handler(); 
	}
	isRcanStarted_CAN1 = 1;
	
	hcan2.pRxMsg = &Can2RxMsg;
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
  HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig2);
	if(HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0) != HAL_OK){
		fw_Error_Handler(); 
	}
	isRcanStarted_CAN2 = 1;
}
/*********************所有CAN接收终端****************************/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	if(hcan == &hcan1){
		switch(Can1RxMsg.StdId){
			case CMFL_RXID:
				//读取0 1字节为角度，2 3字节为速度
				IOPool_pGetWriteData(CMFLRxIOPool)->angle = CanRxGetU16(Can1RxMsg, 0);
				IOPool_pGetWriteData(CMFLRxIOPool)->RotateSpeed = CanRxGetU16(Can1RxMsg, 1);
				IOPool_getNextWrite(CMFLRxIOPool);
				break;
			case CMFR_RXID:
				IOPool_pGetWriteData(CMFRRxIOPool)->angle = CanRxGetU16(Can1RxMsg, 0);
				IOPool_pGetWriteData(CMFRRxIOPool)->RotateSpeed = CanRxGetU16(Can1RxMsg, 1);
				IOPool_getNextWrite(CMFRRxIOPool);
				break;
			case CMBL_RXID:
				IOPool_pGetWriteData(CMBLRxIOPool)->angle = CanRxGetU16(Can1RxMsg, 0);
				IOPool_pGetWriteData(CMBLRxIOPool)->RotateSpeed = CanRxGetU16(Can1RxMsg, 1);
				IOPool_getNextWrite(CMBLRxIOPool);
				break;
			case CMBR_RXID:
				IOPool_pGetWriteData(CMBRRxIOPool)->angle = CanRxGetU16(Can1RxMsg, 0);
				IOPool_pGetWriteData(CMBRRxIOPool)->RotateSpeed = CanRxGetU16(Can1RxMsg, 1);
				IOPool_getNextWrite(CMBRRxIOPool);
				break;
			case PM1_RXID:
				IOPool_pGetWriteData(PM1RxIOPool)->angle = CanRxGetU16(Can1RxMsg, 0);
				IOPool_pGetWriteData(PM1RxIOPool)->RotateSpeed = CanRxGetU16(Can1RxMsg, 1);
				IOPool_getNextWrite(PM1RxIOPool);
				break;
			case PM2_RXID:
				IOPool_pGetWriteData(PM2RxIOPool)->angle = CanRxGetU16(Can1RxMsg, 0);
				IOPool_pGetWriteData(PM2RxIOPool)->RotateSpeed = CanRxGetU16(Can1RxMsg, 1);
				IOPool_getNextWrite(PM2RxIOPool);
				break;
			case GMYAW_RXID:
				IOPool_pGetWriteData(GMYAWRxIOPool)->angle = CanRxGetU16(Can1RxMsg, 0);
				IOPool_pGetWriteData(GMYAWRxIOPool)->realIntensity = CanRxGetU16(Can1RxMsg, 1);
				IOPool_pGetWriteData(GMYAWRxIOPool)->giveIntensity = CanRxGetU16(Can1RxMsg, 2);
				IOPool_getNextWrite(GMYAWRxIOPool);
				break;
			case GMPITCH_RXID:
				IOPool_pGetWriteData(GMPITCHRxIOPool)->angle = CanRxGetU16(Can1RxMsg, 0);
				IOPool_pGetWriteData(GMPITCHRxIOPool)->realIntensity = CanRxGetU16(Can1RxMsg, 1);
				IOPool_pGetWriteData(GMPITCHRxIOPool)->giveIntensity = CanRxGetU16(Can1RxMsg, 2);
				IOPool_getNextWrite(GMPITCHRxIOPool);
				break;
			default:
			fw_Error_Handler();
		}
		//HAL CAN总线存在一定bug，使用自己的标志位
		if(HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK){
			fw_Warning();
			isRcanStarted_CAN1 = 0;
		}else{
			isRcanStarted_CAN1 = 1;
		}
		if(g_bInited == 1){
			//释放信号量交给控制任务
			osSemaphoreRelease(Can1RefreshSemaphoreHandle);
		}
	}
	else if(hcan == &hcan2)
	{
		switch(Can2RxMsg.StdId)
		{
			case AM1L_RXID:
				IOPool_pGetWriteData(AM1LRxIOPool)->angle = CanRxGetU16(Can2RxMsg, 0);
				IOPool_pGetWriteData(AM1LRxIOPool)->RotateSpeed = CanRxGetU16(Can2RxMsg, 1);
				IOPool_getNextWrite(AM1LRxIOPool);
				break;
			case AM1R_RXID:
				IOPool_pGetWriteData(AM1RRxIOPool)->angle = CanRxGetU16(Can2RxMsg, 0);
				IOPool_pGetWriteData(AM1RRxIOPool)->RotateSpeed = CanRxGetU16(Can2RxMsg, 1);
				IOPool_getNextWrite(AM1RRxIOPool);
				break;
			case AM2L_RXID:
				IOPool_pGetWriteData(AM2LRxIOPool)->angle = CanRxGetU16(Can2RxMsg, 0);
				IOPool_pGetWriteData(AM2LRxIOPool)->RotateSpeed = CanRxGetU16(Can2RxMsg, 1);
				IOPool_getNextWrite(AM2LRxIOPool);
				break;
			case AM2R_RXID:
				IOPool_pGetWriteData(AM2RRxIOPool)->angle = CanRxGetU16(Can2RxMsg, 0);
				IOPool_pGetWriteData(AM2RRxIOPool)->RotateSpeed = CanRxGetU16(Can2RxMsg, 1);
				IOPool_getNextWrite(AM2RRxIOPool);
				break;
			case AM3L_RXID:
				IOPool_pGetWriteData(AM3LRxIOPool)->angle = CanRxGetU16(Can2RxMsg, 0);
				IOPool_pGetWriteData(AM3LRxIOPool)->RotateSpeed = CanRxGetU16(Can2RxMsg, 1);
				IOPool_getNextWrite(AM3LRxIOPool);
				break;
			case ZGYRO_RXID:
			 {
				//单轴陀螺仪没有datasheet，完全参照官方程序
				 //解算成角度值
				CanRxMsgTypeDef *msg = &Can2RxMsg;
				ZGyroModuleAngle = -0.01f*((int32_t)(msg->Data[0]<<24)|(int32_t)(msg->Data[1]<<16) | (int32_t)(msg->Data[2]<<8) | (int32_t)(msg->Data[3])); 
			 }
			 break;
			default:
			fw_Error_Handler();
		}
		if(HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0) != HAL_OK)
		{
			//fw_Warning();
			isRcanStarted_CAN2 = 0;
		}else{
			isRcanStarted_CAN2 = 1;
		}
		if(g_bInited == 1){
			//这个信号量并没有人理0.o
			osSemaphoreRelease(Can2RefreshSemaphoreHandle);
		}
	}
}
/********************CAN发送*****************************/
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan)
{
	if(hcan == &hcan1){
		osSemaphoreRelease(Can1TransmitSemaphoreHandle);
	}
	else if(hcan == &hcan2)
	{
		osSemaphoreRelease(Can2TransmitSemaphoreHandle);
	}
}

void TransmitCAN1(void)
{
		if(IOPool_hasNextRead(CMTxIOPool, 0))
		{
			//使用信号量保护CAN发送资源，在发送中断中release
			osSemaphoreWait(Can1TransmitSemaphoreHandle, osWaitForever);
			
			IOPool_getNextRead(CMTxIOPool, 0);
			hcan1.pTxMsg = IOPool_pGetReadData(CMTxIOPool, 0);
			//使用临界区避免被抢占
			taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&hcan1) != HAL_OK){
				fw_Warning();
			}
			taskEXIT_CRITICAL();
		}
		
		if(IOPool_hasNextRead(GMTxIOPool, 0))
		{
			osSemaphoreWait(Can1TransmitSemaphoreHandle, osWaitForever);
			
			IOPool_getNextRead(GMTxIOPool, 0);
			hcan1.pTxMsg = IOPool_pGetReadData(GMTxIOPool, 0);
			
			taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&hcan1) != HAL_OK){
				fw_Warning();
			}
			taskEXIT_CRITICAL();
		}
		
		if(IOPool_hasNextRead(PMTxIOPool, 0))
		{
			osSemaphoreWait(Can1TransmitSemaphoreHandle, osWaitForever);
			
			IOPool_getNextRead(PMTxIOPool, 0);
			hcan1.pTxMsg = IOPool_pGetReadData(PMTxIOPool, 0);
			
			taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&hcan1) != HAL_OK){
				fw_Warning();
			}
			taskEXIT_CRITICAL();
		}
}

void TransmitCAN2(void){
	if(IOPool_hasNextRead(AM1TxIOPool, 0))
	{
			osSemaphoreWait(Can2TransmitSemaphoreHandle, osWaitForever);
			
			IOPool_getNextRead(AM1TxIOPool, 0);
			hcan2.pTxMsg = IOPool_pGetReadData(AM1TxIOPool, 0);
			
			taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&hcan2) != HAL_OK){
				fw_Warning();
			}
			taskEXIT_CRITICAL();
	}
	
	if(IOPool_hasNextRead(AM23TxIOPool, 0))
	{
			osSemaphoreWait(Can2TransmitSemaphoreHandle, osWaitForever);
			
			IOPool_getNextRead(AM23TxIOPool, 0);
			hcan2.pTxMsg = IOPool_pGetReadData(AM23TxIOPool, 0);
			
			taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&hcan2) != HAL_OK){
				fw_Warning();
			}
			taskEXIT_CRITICAL();
	}
	
	if(IOPool_hasNextRead(ZGYROTxIOPool, 0))
	{
			osSemaphoreWait(Can2TransmitSemaphoreHandle, osWaitForever);
		
			IOPool_getNextRead(ZGYROTxIOPool, 0);
			hcan2.pTxMsg = IOPool_pGetReadData(ZGYROTxIOPool, 0);
		
			taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&hcan2) != HAL_OK)
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
