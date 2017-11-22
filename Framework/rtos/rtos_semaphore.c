/**
  ******************************************************************************
  * File Name          : rtos_semaphore.c
  * Description        : 添加FreeRTOS信号量
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 添加信号量用于进程间同步

  ******************************************************************************
  */
#include <rtos_semaphore.h>
#include <rtos_init.h>

osSemaphoreId CMGMCanHaveTransmitSemaphoreHandle;
osSemaphoreId ZGYROCanHaveTransmitSemaphoreHandle;

osSemaphoreId Can1TransmitSemaphoreHandle;
osSemaphoreId Can2TransmitSemaphoreHandle;

osSemaphoreId motorCanReceiveSemaphoreHandle;

osSemaphoreId Can1RefreshSemaphoreHandle;
osSemaphoreId Can2RefreshSemaphoreHandle;

osSemaphoreId imurefreshGimbalSemaphoreHandle;

//osSemaphoreId imuSpiTxRxCpltSemaphoreHandle;
osSemaphoreId refreshMPU6500SemaphoreHandle;

xSemaphoreHandle xSemaphore_mfuart;
xSemaphoreHandle xSemaphore_rcuart;
xSemaphoreHandle motorCanTransmitSemaphore;
void rtos_AddSemaphores()
{
	osSemaphoreDef(Can1TransmitSemaphore);//CAN1发送信号量
	Can1TransmitSemaphoreHandle = osSemaphoreCreate(osSemaphore(Can1TransmitSemaphore), 1);
	
	osSemaphoreDef(Can2TransmitSemaphore);//CAN2发送信号量
	Can2TransmitSemaphoreHandle = osSemaphoreCreate(osSemaphore(Can2TransmitSemaphore), 1);

	osSemaphoreDef(Can1RefreshSemaphore);//CAN1接收信号量
	Can1RefreshSemaphoreHandle = osSemaphoreCreate(osSemaphore(Can1RefreshSemaphore), 1);
	
	osSemaphoreDef(Can2RefreshSemaphore);//CAN2接收信号量
	Can2RefreshSemaphoreHandle = osSemaphoreCreate(osSemaphore(Can2RefreshSemaphore), 1);
	
	osSemaphoreDef(imurefreshGimbalSemaphore);//IMU数据刷新信号量(有Release，无进程Take)
	imurefreshGimbalSemaphoreHandle = osSemaphoreCreate(osSemaphore(imurefreshGimbalSemaphore), 1);
	
//	osSemaphoreDef(imuSpiTxRxCpltSemaphore);
//	imuSpiTxRxCpltSemaphoreHandle = osSemaphoreCreate(osSemaphore(imuSpiTxRxCpltSemaphore), 1);
	
	osSemaphoreDef(refreshMPU6500Semaphore);//MPU6050数据刷新信号量：IO口外部中断Release，数据处理Task Take
	refreshMPU6500SemaphoreHandle = osSemaphoreCreate(osSemaphore(refreshMPU6500Semaphore), 1);
/**
******************************************************************************
* 提示：
* 这里示范了两种创建信号量的方式
* osSemaphoreDef(),osSemaphoreCreate()是CMSIS要求的RTOS统一接口
* vSemaphoreCreateBinary()是FreeRTOS提供的创建方式
* 两种方式本质相同，osSemaphoreCreate()实际上调用了xSemaphoreCreateBinary()
******************************************************************************
*/
	vSemaphoreCreateBinary(xSemaphore_mfuart);//mf(Manifold妙算)通信信号量，串口接收回掉函数Release，数据处理Task Take
	vSemaphoreCreateBinary(xSemaphore_rcuart);//rc(Remote Control遥控)同上
	
	motorCanTransmitSemaphore = xSemaphoreCreateCounting(10,0);//另一种信号量，未用 注意开启此功能需要在Cube中配置RTOS
}
