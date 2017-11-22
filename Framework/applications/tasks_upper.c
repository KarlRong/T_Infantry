/**
  ******************************************************************************
  * File Name          : tasks_upper.c
  * Description        : 与上位机(妙算)通信任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 通信串口数据获取
  ******************************************************************************
  */
#include <stdint.h>
#include <cmsis_os.h>
#include "tasks_upper.h"
#include "drivers_uartupper_user.h"
#include "drivers_uartupper_low.h"
#include "rtos_semaphore.h"
#include "drivers_flash.h"
#include "utilities_debug.h"


NaiveIOPoolDefine(upperIOPool, {0});

extern uint16_t yawAngle, pitchAngle;
int forPidDebug = 0;

extern float yawAngleTarget, pitchAngleTarget;
extern xSemaphoreHandle xSemaphore_mfuart;
extern xdata_ctrlUart ctrlData; 
extern uint16_t x;
uint8_t CReceive = 0;
uint8_t rune_flag = 0;
float yawAdd = 0;
float last_yawAdd = 0;
float yaw_speed = 0;


extern float pitchRealAngle;
extern float ZGyroModuleAngle;	//陀螺仪角度

void ManifoldUartTask(void const * argument){
	while(1){
			xSemaphoreTake(xSemaphore_mfuart, osWaitForever);
	//		fw_printfln("Ctrl");
			uint8_t *pData = IOPool_pGetReadData(ctrlUartIOPool, 0)->ch;

	 }
}

	
