#include "tasks_cmcontrol.h"
#include "pid_Regulator.h"
#include "drivers_uartrc_low.h"
#include "drivers_uartrc_user.h"
#include "tasks_remotecontrol.h"
#include "application_motorcontrol.h"
#include "tasks_cmcontrol.h"
#include "drivers_canmotor_low.h"
#include "drivers_canmotor_user.h"
#include "utilities_debug.h"
#include "rtos_semaphore.h"
#include "rtos_task.h"
#include "tim.h"
#include "stdint.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "task.h"
#include "usart.h"
#include "peripheral_define.h"


extern PID_Regulator_t CMRotatePID ; 
extern PID_Regulator_t CM1SpeedPID;
extern PID_Regulator_t CM2SpeedPID;
extern PID_Regulator_t CM3SpeedPID;
extern PID_Regulator_t CM4SpeedPID;
PID_Regulator_t ShootMotorPositionPID = SHOOT_MOTOR_POSITION_PID_DEFAULT;      //shoot motor
PID_Regulator_t ShootMotorSpeedPID = SHOOT_MOTOR_SPEED_PID_DEFAULT;

extern volatile Encoder CM1Encoder;
extern volatile Encoder CM2Encoder;
extern volatile Encoder CM3Encoder;
extern volatile Encoder CM4Encoder;
extern volatile Encoder GMYawEncoder;

WorkState_e workState = PREPARE_STATE;
WorkState_e lastWorkState = PREPARE_STATE;
WorkState_e GetWorkState()
{
	return workState;
}
/*2ms定时任务*/

extern float ZGyroModuleAngle;
float ZGyroModuleAngleMAX;
float ZGyroModuleAngleMIN;
extern float yawRealAngle;
extern uint8_t GYRO_RESETED;
extern float pitchRealAngle;
extern float gYroZs;
extern float yawAngleTarget;
extern float yawRealAngle;
//*********debug by ZY*********
typedef struct{
	uint16_t head;
	uint8_t id;
	uint8_t dlc;
	float dataPitch;
	float dataYaw;
	uint8_t checkSum;
}data_to_PC;


uint8_t data_send_to_PC[17];
data_to_PC my_data_to_PC;
void send_data_to_PC(UART_HandleTypeDef *huart,float zyPitch,float zyYaw,float zySpd)
{
//	my_data_to_PC.head=0xAAAA;
//	my_data_to_PC.id=0xF1;
//	my_data_to_PC.dlc=8;
//	my_data_to_PC.dataPitch=zyPitch;
//	my_data_to_PC.dataYaw=zyYaw;
//	
//	uint8_t * pTemp;
//	//uint8_t temp;
//	int i;
//	my_data_to_PC.checkSum=0;
//	pTemp = (uint8_t *)&(my_data_to_PC);  
//	for(i=0;i<12;i++)
//	{
//		 my_data_to_PC.checkSum+=pTemp[i];
//	}
	
	
	uint8_t * pTemp;
	int i;
	data_send_to_PC[0]=0xAA;
	data_send_to_PC[1]=0xAA;
	data_send_to_PC[2]=0xF1;
	data_send_to_PC[3]=12;
	pTemp=(uint8_t *)&zyPitch;
	for(i=0;i<4;i++)
	{
		 data_send_to_PC[4+i]=pTemp[3-i];
	}
	
	pTemp=(uint8_t *)&zyYaw;
	for(i=0;i<4;i++)
	{
		 data_send_to_PC[8+i]=pTemp[3-i];
	}
	
	pTemp=(uint8_t *)&zySpd;
	for(i=0;i<4;i++)
	{
		 data_send_to_PC[12+i]=pTemp[3-i];
	}
	
	data_send_to_PC[16]=0;
	for(i=0;i<16;i++)
	{
		 data_send_to_PC[16]+=data_send_to_PC[i];
	}
	
	HAL_UART_Transmit(huart,data_send_to_PC,17,1000);
}

//*********debug by ZY*********
extern int EncoderCnt;
void Timer_2ms_lTask(void const * argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	static int countwhile = 0;
	static int countwhile1 = 0;
	static int countwhile2 = 0;
	static int countwhile3 = 0;
//	unsigned portBASE_TYPE StackResidue; //栈剩余
	while(1)  {       //motor control frequency 2ms
//监控任务
//		SuperviseTask();    
		WorkStateFSM();
	  WorkStateSwitchProcess();
//1s循环
		if(countwhile >= 500){//定时 1S
//		countwhile = 0;
//			fw_printfln("ZGyroModuleAngle:  %f",ZGyroModuleAngle);
//			fw_printfln("YawAngle= %d", IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle);
//			fw_printfln("GMYawEncoder.ecd_angle:%f",GMYawEncoder.ecd_angle);
//			fw_printfln("PitAngle= %d", IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle);
//				fw_printfln("GMYAWEncoder.ecd_angle:%f",GMYawEncoder.ecd_angle );
//			fw_printfln("in CMcontrol_task");
//		StackResidue = uxTaskGetStackHighWaterMark( GMControlTaskHandle );
//		fw_printfln("GM%ld",StackResidue);
		}else{
			countwhile++;
		}
//陀螺仪复位计时
    if(countwhile1 > 2500){
			if(GYRO_RESETED == 0)GYRO_RST();//给单轴陀螺仪将当前位置写零
		}
		else{countwhile1++;}
		if(countwhile1 > 4000){
			GYRO_RESETED = 2;//正常遥控器或者键盘控制模式，底盘跟随模式
		}
		else{countwhile1++;}
//10ms循环
		if(countwhile2 >= 50){//定时 100MS
		countwhile2 = 0;
		//fw_printfln("yawRealAngle:%f",yawRealAngle);	
			
//		send_data_to_PC(&DEBUG_UART,pitchRealAngle,ZGyroModuleAngle, gYroZs);//发送数据到上位机看波形
			//printf("pitch:%f *** yaw:%f",pitchRealAngle,ZGyroModuleAngle);
//		HAL_UART_Transmit(&DEBUG_UART,txbuf,strlen((char *)txbuf),1000);
		}else{
			countwhile2++;
		}
		if(countwhile3 >=25 && GetShootState() == SHOOTING){
			countwhile3 = 0;
			//先检测是否卡弹，而后根据情况更新拨盘电机PID	
			if(EncoderCnt<3){		//产生卡弹
				//清空计数器
				//EncoderCnt = 0;			
				//卡弹之后直接反转
				HAL_GPIO_TogglePin(PM_Dir_Ctrl1_GPIO_Port,PM_Dir_Ctrl1_Pin);
				HAL_GPIO_TogglePin(PM_Dir_Ctrl2_GPIO_Port,PM_Dir_Ctrl2_Pin);
				ShootMotorSpeedPID.ref = PID_SHOOT_MOTOR_SPEED;
				fw_printfln("ref = %d",PID_SHOOT_MOTOR_SPEED);
				ShootMotorSpeedPID.fdb = EncoderCnt;
				fw_printfln("fdb = %d",EncoderCnt);
				EncoderCnt = 0;
				//ShootMotorSpeedPID.ref = PID_SHOOT_MOTOR_SPEED;
				ShootMotorSpeedPID.output = 1*(ShootMotorSpeedPID.ref-ShootMotorSpeedPID.fdb);
			}
			else{	//如果没有卡弹,更新PID
				ShootMotorSpeedPID.ref = PID_SHOOT_MOTOR_SPEED;
				fw_printfln("ref = %d",PID_SHOOT_MOTOR_SPEED);
				ShootMotorSpeedPID.fdb = EncoderCnt;
				fw_printfln("fdb = %d",EncoderCnt);
				EncoderCnt = 0;
				//ShootMotorSpeedPID.ref = PID_SHOOT_MOTOR_SPEED;
				ShootMotorSpeedPID.output = 1*(ShootMotorSpeedPID.ref-ShootMotorSpeedPID.fdb);
				//ShootMotorSpeedPID.Calc(&ShootMotorSpeedPID);	
			}
		}
		else countwhile3++;
		
		ShooterMControlLoop();       //发射机构控制任务
		
		vTaskDelayUntil( &xLastWakeTime, ( 2 / portTICK_RATE_MS ) );
	}
}
	

	static uint32_t time_tick_2ms = 0;
void CMControtLoopTaskInit(void)
{
	//计数初始化
	time_tick_2ms = 0;   //中断中的计数清零
  //监控任务初始化
	/*
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_RC));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_IMU));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_ZGYRO));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR1));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR2));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR3));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR4));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR5));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR6));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_DEADLOCK));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_NOCALI));
  */  
	ShootMotorSpeedPID.Reset(&ShootMotorSpeedPID);
	CMRotatePID.Reset(&CMRotatePID);
	CM1SpeedPID.Reset(&CM1SpeedPID);
	CM2SpeedPID.Reset(&CM2SpeedPID);
	CM3SpeedPID.Reset(&CM3SpeedPID);
	CM4SpeedPID.Reset(&CM4SpeedPID);
}
/**********************************************************
*工作状态切换状态机
**********************************************************/

void WorkStateFSM(void)
{
	lastWorkState = workState;
	time_tick_2ms ++;
	switch(workState)
	{
		case PREPARE_STATE:
		{
			if(GetInputMode() == STOP )//|| Is_Serious_Error())
			{
				workState = STOP_STATE;
			}
			else if(time_tick_2ms > PREPARE_TIME_TICK_MS)
			{
				workState = NORMAL_STATE;
			}			
		}break;
		case NORMAL_STATE:     
		{
			if(GetInputMode() == STOP )//|| Is_Serious_Error())
			{
				workState = STOP_STATE;
			}
			else if(!IsRemoteBeingAction()  && GetShootState() != SHOOTING) //||(Get_Lost_Error(LOST_ERROR_RC) == LOST_ERROR_RC
			{
				fw_printfln("进入STANDBY");
				workState = STANDBY_STATE;      
			}			
		}break;
		case STANDBY_STATE:  
		{
			if(GetInputMode() == STOP )//|| Is_Serious_Error())
			{
				workState = STOP_STATE;
			}
			else if(IsRemoteBeingAction() || (GetShootState()==SHOOTING) || GetFrictionState() == FRICTION_WHEEL_START_TURNNING)
			{
				workState = NORMAL_STATE;
			}				
		}break;
		case STOP_STATE:   
		{
			if(GetInputMode() != STOP )//&& !Is_Serious_Error())
			{
				workState = PREPARE_STATE;   
			}
		}break;
		default:
		{
			
		}
	}	
}
void WorkStateSwitchProcess(void)
{
	//如果从其他模式切换到prapare模式，要将一系列参数初始化
	if((lastWorkState != workState) && (workState == PREPARE_STATE))  
	{
		CMControtLoopTaskInit();
		RemoteTaskInit();
	}
}
//底盘控制任务
extern int16_t yawZeroAngle;
void CMControlLoop(void)
{  
	//底盘旋转量计算
	if(GetWorkState()==PREPARE_STATE) //启动阶段，底盘不旋转
	{
		ChassisSpeedRef.rotate_ref = 0;	 
	}
	else
	{
		 //底盘跟随编码器旋转PID计算
		 CMRotatePID.ref = 48;
		 CMRotatePID.fdb = GMYawEncoder.ecd_angle;
//		fw_printfln("%f",GMYawEncoder.ecd_angle );
		 CMRotatePID.Calc(&CMRotatePID);   
		 ChassisSpeedRef.rotate_ref = CMRotatePID.output;
		 ChassisSpeedRef.rotate_ref = 0;
	}
/*	if(Is_Lost_Error_Set(LOST_ERROR_RC))      //如果遥控器丢失，强制将速度设定值reset
	{
		ChassisSpeedRef.forward_back_ref = 0;
		ChassisSpeedRef.left_right_ref = 0;
	}
*/	
	CM1SpeedPID.ref =  -ChassisSpeedRef.forward_back_ref*0.075 + ChassisSpeedRef.left_right_ref*0.075 + ChassisSpeedRef.rotate_ref;
	CM2SpeedPID.ref = ChassisSpeedRef.forward_back_ref*0.075 + ChassisSpeedRef.left_right_ref*0.075 + ChassisSpeedRef.rotate_ref;
	CM3SpeedPID.ref = ChassisSpeedRef.forward_back_ref*0.075 - ChassisSpeedRef.left_right_ref*0.075 + ChassisSpeedRef.rotate_ref;
	CM4SpeedPID.ref = -ChassisSpeedRef.forward_back_ref*0.075 - ChassisSpeedRef.left_right_ref*0.075 + ChassisSpeedRef.rotate_ref;

	CM1SpeedPID.fdb = CM1Encoder.filter_rate;
	CM2SpeedPID.fdb = CM2Encoder.filter_rate;
	CM3SpeedPID.fdb = CM3Encoder.filter_rate;
	CM4SpeedPID.fdb = CM4Encoder.filter_rate;
	
	CM1SpeedPID.Calc(&CM1SpeedPID);
	CM2SpeedPID.Calc(&CM2SpeedPID);
	CM3SpeedPID.Calc(&CM3SpeedPID);

































CM4SpeedPID.Calc(&CM4SpeedPID);
	
	 if((GetWorkState() == STOP_STATE)  || GetWorkState() == CALI_STATE || GetWorkState() == PREPARE_STATE || GetEmergencyFlag() == EMERGENCY)   //||Is_Serious_Error()|| dead_lock_flag == 1紧急停车，编码器校准，无控制输入时都会使底盘控制停止
	 {
		 Set_CM_Speed(0,0,0,0);
	 }
	 else
	 {
		 Set_CM_Speed(CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output, CHASSIS_SPEED_ATTENUATION * CM2SpeedPID.output, CHASSIS_SPEED_ATTENUATION * CM3SpeedPID.output, CHASSIS_SPEED_ATTENUATION * CM4SpeedPID.output);		 
	 } 
	 
}
static char Encoder_Dir = 1;
  
int32_t GetQuadEncoderDiff(void)
{
  int32_t cnt = 0;    
	cnt = 0x7fff - __HAL_TIM_GET_COUNTER(&htim3);
//	fw_printfln("%x",cnt);
	 __HAL_TIM_SET_COUNTER(&htim3, 0x7fff);
    if(Encoder_Dir == 1)
	{
		return cnt;	
	}
	else
	{
		return -cnt;            
	}
}

//发射机构射击电机任务
int16_t pwm_ccr = 0;
//extern TIM_HandleTypeDef htim9;
uint16_t x = 0;
void ShooterMControlLoop(void)	
{				      
	if(GetShootState() == SHOOTING)
	{
//			
		int temp = (TIM4->CCR1 + ShootMotorSpeedPID.output);//
		//fw_printfln("PID_Output = %f",ShootMotorSpeedPID.output);
		fw_printfln("temp: %d",temp);
		ShootMotorSpeedPID.output = 0;
		
		if(temp>950) temp = 950;//700
		else if(temp<-1) temp = 0;
		TIM4->CCR1 = temp;//500500
		
	}
	else
	{
		ShootMotorSpeedPID.ref = 0;		
		TIM4->CCR1 = 0;
		//fw_printfln("NoShooting");
	}
	
//	ShootMotorSpeedPID.fdb = GetQuadEncoderDiff();   
//	ShootMotorSpeedPID.Calc(&ShootMotorSpeedPID);
//	fw_printfln("%d",(uint32_t)ShootMotorSpeedPID.output);
//	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, ShootMotorSpeedPID.output);
}

