/**
  ******************************************************************************
  * File Name          : drivers_uart.c
  * Description        : 电机串口驱动函数
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 串口接收终端回调函数：遥控器
	* 妙算通讯
	* 裁判系统读取
  ******************************************************************************
  */
#include <usart.h>
#include "drivers_uart.h"
#include "drivers_uartrc_low.h"
#include "drivers_uartupper_low.h"
#include "drivers_uartjudge_low.h"
#include "peripheral_define.h"
#include "utilities_debug.h"

/********************所有串口接收中断****************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &RC_UART){
		//遥控器
		//fw_printfln("rc get!!!");
		rcUartRxCpltCallback();
		
	}else if(UartHandle == &MANIFOLD_UART){
		//妙算通信串口
		//自定义协议
		fw_printfln("manifold get!!!");
		manifoldUartRxCpltCallback();
//		if(HAL_UART_Receive_DMA(&MANIFOLD_UART, IOPool_pGetWriteData(ctrlUartIOPool)->ch, size_frame) != HAL_OK){
//		Error_Handler();
//		printf( "InitManifoldUart error" );
//} 
	}
	else if(UartHandle == &JUDGE_UART){
		//裁判系统读取采用单字节阻塞接收方式
		//比赛剩余时间
		//血量
		//底盘电压、电流
		
		
		//能量槽*****重要，超功率掉血
		//fw_printfln("judge get!!!");
		judgeUartRxCpltCallback();
	}
}   
