#ifndef _SOV_H_
#define _SOV_H_
#include "gpio.h"

#define SOV_ON()  HAL_GPIO_WritePin(SOV_GPIO_Port, SOV_Pin,GPIO_PIN_SET);\
										
#define SOV_OFF()  HAL_GPIO_WritePin(SOV_GPIO_Port, SOV_Pin,GPIO_PIN_RESET);\
											
#endif

