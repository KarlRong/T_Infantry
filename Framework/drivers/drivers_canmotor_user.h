/**
  ******************************************************************************
  * File Name          : drivers_canmotor_user.h
  * Description        : 电机CAN总线驱动函数
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * CAN总线用户函数
  ******************************************************************************
  */
#ifndef DRIVERS_CANMOTOR_USER_H
#define DRIVERS_CANMOTOR_USER_H

#include <can.h>
#include "utilities_iopool.h"
#include "drivers_canmotor_low.h"

//RxID
//CM为底盘电机
#define CMFL_RXID 0x202u
#define CMFR_RXID 0x201u
#define CMBL_RXID 0x203u
#define CMBR_RXID 0x204u
//AM为机械臂电机
#define AM1L_RXID 0x205u
#define AM1R_RXID 0x206u
#define AM2L_RXID 0x201u
#define AM2R_RXID 0x202u
#define AM3L_RXID 0x203u
//GM为云台电机
#define GMYAW_RXID 0x209u
#define GMPITCH_RXID 0x20Au
//PM为推弹电机
#define PM1_RXID 0x205u
#define PM2_RXID 0x206u
//GYRO为单轴陀螺仪
#define ZGYRO_RXID   0x401u

//TxID
#define CM_TXID 0x200u	//CAN1
#define AM1_TXID 0x1FFu	//CAN2
#define GM_TXID 0x2FFu	//CAN1
#define AM23_TXID 0x200u//CAN2
#define PM_TXID 0x1FFu	//CAN1
#define ZGYRO_TXID   0x404u	//CAN2

//RxIOPool
typedef struct{
	uint16_t angle;
	int16_t realIntensity;
	int16_t giveIntensity;
}Motor6623RxMsg_t;
//6623--[0,1]Angle;[2,3]RealIntensity;[4,5]GiveIntensity;
IOPoolDeclare(GMPITCHRxIOPool, Motor6623RxMsg_t);
IOPoolDeclare(GMYAWRxIOPool, Motor6623RxMsg_t);

typedef struct{
	uint16_t angle;
	int16_t RotateSpeed;//RPM
}Motor820RRxMsg_t;

//820R--[0,1]Angle;[2,3]RotateSpeed;
IOPoolDeclare(CMFLRxIOPool, Motor820RRxMsg_t);
IOPoolDeclare(CMFRRxIOPool, Motor820RRxMsg_t);
IOPoolDeclare(CMBLRxIOPool, Motor820RRxMsg_t);
IOPoolDeclare(CMBRRxIOPool, Motor820RRxMsg_t);

IOPoolDeclare(AM1LRxIOPool, Motor820RRxMsg_t);
IOPoolDeclare(AM1RRxIOPool, Motor820RRxMsg_t);
IOPoolDeclare(AM2LRxIOPool, Motor820RRxMsg_t);
IOPoolDeclare(AM2RRxIOPool, Motor820RRxMsg_t);
IOPoolDeclare(AM3LRxIOPool, Motor820RRxMsg_t);

IOPoolDeclare(PM1RxIOPool, Motor820RRxMsg_t);
IOPoolDeclare(PM2RxIOPool, Motor820RRxMsg_t);

//TxIOPool
IOPoolDeclare(CMTxIOPool, CanTxMsgTypeDef);
IOPoolDeclare(GMTxIOPool, CanTxMsgTypeDef);
IOPoolDeclare(AM1TxIOPool, CanTxMsgTypeDef);
IOPoolDeclare(AM23TxIOPool, CanTxMsgTypeDef);
IOPoolDeclare(PMTxIOPool, CanTxMsgTypeDef);
IOPoolDeclare(ZGYROTxIOPool, CanTxMsgTypeDef);


void InitCanReception(void);
void TransmitCAN1(void);
void TransmitCAN2(void);

//void Set_CM_Speed(int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
//void CANReceiveMsgProcess_820R(Motor820RRxMsg_t * msg,volatile Encoder * CMxEncoder);
//void EncoderProcess(volatile Encoder *v, Motor820RRxMsg_t * msg);//未用到
//void GetEncoderBias(volatile Encoder *v, Motor820RRxMsg_t * msg);//未用到

#endif
