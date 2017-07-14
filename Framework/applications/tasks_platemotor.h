/**
  ******************************************************************************
  * File Name          : tasks_platemotor.h
  * Description        : 拨盘电机任务
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
#ifndef FRAMEWORK_TASKS_PLATECONTROL_H
#define FRAMEWORK_TASKS_PLATECONTROL_H

#include "cmsis_os.h"
#include "tasks_timed.h"
#include "pid_regulator.h"

void ShootOneBullet(void);

#endif
