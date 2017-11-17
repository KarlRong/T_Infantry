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
#define CMFL_RXID 0x202u
#define CMFR_RXID 0x201u
#define CMBL_RXID 0x203u
#define CMBR_RXID 0x204u

#define GMYAW_RXID 0x205u
#define GMPITCH_RXID 0x206u

#define ZGYRO_RXID   0x401u

//TxID
#define CM_TXID 0x200u
#define GM_TXID 0x1FFu

#define ZGYRO_TXID   0x404u

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


//TxIOPool
IOPoolDeclare(CMTxIOPool, CanTxMsgTypeDef);
IOPoolDeclare(GMTxIOPool, CanTxMsgTypeDef);
IOPoolDeclare(ZGYROTxIOPool, CanTxMsgTypeDef);


void InitCanReception(void);
void TransmitCMGMCan(void);
void TransmitGYROCAN(void);

//void Set_CM_Speed(int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
//void CANReceiveMsgProcess_820R(Motor820RRxMsg_t * msg,volatile Encoder * CMxEncoder);
//void EncoderProcess(volatile Encoder *v, Motor820RRxMsg_t * msg);//未用到
//void GetEncoderBias(volatile Encoder *v, Motor820RRxMsg_t * msg);//未用到

#endif
