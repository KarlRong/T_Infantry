/**
  ******************************************************************************
  * File Name          : drivers_canmotor_low.h
  * Description        : ç”µæœºCANæ€»çº¿é©±åŠ¨å‡½æ•°
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * CANæ€»çº¿åº•å±‚å‡½æ•°
  ******************************************************************************
  */
#ifndef DRIVERS_CANMOTOR_LOW_H
#define DRIVERS_CANMOTOR_LOW_H

#include <can.h>
#include <stdint.h>
#include <cmsis_os.h>

void InitCanReception(void);

void CMGMCanTransmitTask(void const * argument);
//void ZGYROCanTransmitTask(void const * argument);//æ²¡ç”¨çš„

#define RATE_BUF_SIZE 6

//æ²¡ç”¨åˆ°
/*
typedef struct{
	int32_t raw_value;   									//±àÂëÆ÷²»¾­´¦ÀíµÄÔ­Ê¼Öµ
	int32_t last_raw_value;								//ÉÏÒ»´ÎµÄ±àÂëÆ÷Ô­Ê¼Öµ
	int32_t ecd_value;                       //¾­¹ı´¦ÀíºóÁ¬ĞøµÄ±àÂëÆ÷Öµ
	int32_t diff;													//Á½´Î±àÂëÆ÷Ö®¼äµÄ²îÖµ
	int32_t temp_count;                   //¼ÆÊıÓÃ
	uint8_t buf_count;								//ÂË²¨¸üĞÂbufÓÃ
	int32_t ecd_bias;											//³õÊ¼±àÂëÆ÷Öµ	
	int32_t ecd_raw_rate;									//Í¨¹ı±àÂëÆ÷¼ÆËãµÃµ½µÄËÙ¶ÈÔ­Ê¼Öµ
	int32_t rate_buf[RATE_BUF_SIZE];	//buf£¬for filter
	int32_t round_cnt;										//È¦Êı
	int32_t filter_rate;											//ËÙ¶È
	float ecd_angle;											//½Ç¶È
}Encoder;
*/


#endif
