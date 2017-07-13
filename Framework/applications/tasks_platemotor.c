/**
  ******************************************************************************
  * File Name          : tasks_platemotor.c
  * Description        : 拨盘电机控制任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
	* 定时循环
	* 编码器模式对编码器脉冲计数
	* PWM波控制速度
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
#include "application_motorcontrol.h"
#include "drivers_canmotor_low.h"
#include "drivers_canmotor_user.h"
#include "utilities_debug.h"
#include "rtos_semaphore.h"
#include "rtos_task.h"
#include "peripheral_define.h"
#include "drivers_platemotor.h"

extern PID_Regulator_t ShootMotorPositionPID;
void ShootOneBullet()
{
	ShootMotorPositionPID.ref = ShootMotorPositionPID.ref+OneShoot;
}


