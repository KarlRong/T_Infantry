#include "peripheral_tim.h"
#include "cmsis_os.h"
#include "tim.h"
#include "peripheral_define.h"
#include "pwm_server_motor.h"
#include "drivers_uartrc_user.h"
#include "tasks_motor.h"
void UserTimerInit(void)
{
//	HAL_TIM_Encoder_Start(&PLATE_TIM, TIM_CHANNEL_ALL);
//	HAL_TIM_PWM_Start(&PLATE_MOTOR_TIM , TIM_CHANNEL_1);//²¦ÅÌµç»ú
	HAL_TIM_PWM_Start(&FRICTION_TIM, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&FRICTION_TIM, TIM_CHANNEL_2);
	pwm_server_motor_init(0);
#ifdef Infantry_3
	pwm_server_motor_set_angle(0,110.);
#endif
#ifdef Infantry_2
	pwm_server_motor_set_angle(0,180.);
#endif
  SetSlabState(CLOSE);
}
