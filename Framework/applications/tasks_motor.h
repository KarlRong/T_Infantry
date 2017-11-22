/**
  ******************************************************************************
  * File Name          : tasks_motor.h
  * Description        : 电机控制任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
	* 主要为云台初始位置时的编码器位置
	* 以及其他可能存在的性能差异
  ******************************************************************************
  */
#ifndef TASKS_MOTOR_H
#define TASKS_MOTOR_H

#define PM1Reduction 19.0
#define PM2Reduction 19.0

void Can1ControlTask(void const * argument);
void ControlYaw(void);
void ControlPitch(void);
void ControlRotate(void);
void ControlCMFL(void);
void ControlCMFR(void);
void ControlCMBL(void);
void ControlCMBR(void);
void ControlPM1(void);
void ControlPM2(void);

void shootOneGolf(void);

#endif
