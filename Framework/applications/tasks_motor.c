#include "tasks_motor.h"
#include "drivers_canmotor_user.h"
#include "rtos_semaphore.h"
#include "drivers_uartrc_user.h"
#include "utilities_debug.h"
#include "tasks_upper.h"
#include "tasks_cmcontrol.h"
#include "tasks_remotecontrol.h"
#include "drivers_led_user.h"
#include "utilities_minmax.h"
#include "pid_regulator.h"
#include "application_motorcontrol.h"
#include "drivers_sonar_user.h"
#include "usart.h"
#include "peripheral_define.h"
#include "drivers_uartupper_user.h"

#include "stdint.h"

//PID_INIT(Kp, Ki, Kd, KpMax, KiMax, KdMax, OutputMax)
#ifdef Infantry_1_Aim
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(15.0, 2.0, 1.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(5.3, -1.0, 0.5, 10000.0, 10000.0, 10000.0, 10000.0);//等幅振荡P37.3 I11.9 D3.75  原26.1 8.0 1.1
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(25.0, 0.0, 5.0, 10000.0, 10000.0, 10000.0, 3500.0);
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(40.0, 0.0, 20, 10000.0, 10000.0, 10000.0, 4000.0);
#define yaw_zero 1075
#define pitch_zero 3180
#endif
#ifdef Infantry_2
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(15.0, 2.0, 1.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(5.3, -1.0, 0.5, 10000.0, 10000.0, 10000.0, 10000.0);//等幅振荡P37.3 I11.9 D3.75  原26.1 8.0 1.1
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(25.0, 0.0, 5.0, 10000.0, 10000.0, 10000.0, 3500.0);
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(40.0, 0.0, 20, 10000.0, 10000.0, 10000.0, 4000.0);
#define yaw_zero 614
#define pitch_zero 2924
#endif
#ifdef Infantry_3
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(4.0, 0, 2.5, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(2.0, 0.5, 0.7, 10000.0, 10000.0, 10000.0, 10000.0);//等幅振荡P37.3 I11.9 D3.75  原26.1 8.0 1.1
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(25.0, 0.0, 5.0, 10000.0, 10000.0, 10000.0, 5900.0);
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(35.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 9800.0);
#define yaw_zero 720
#define pitch_zero 5003
#endif
#ifdef Infantry_1_Aim
PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT_old; 
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT_old;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT_old;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT_old;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT_old;
#else
PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
#endif
fw_PID_Regulator_t testPositionPID = fw_PID_INIT(6.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
fw_PID_Regulator_t testSpeedPID = fw_PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);//0.0, 0.00003

fw_PID_Regulator_t platePositionPID = fw_PID_INIT(40.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
fw_PID_Regulator_t plateSpeedPID = fw_PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);//0.0, 0.00003

extern float gYroXs, gYroYs, gYroZs;

extern PID_Regulator_t CMRotatePID;
extern volatile Encoder CM1Encoder;
extern volatile Encoder CM2Encoder;
extern volatile Encoder CM3Encoder;
extern volatile Encoder CM4Encoder;
extern volatile Encoder GMYawEncoder;
//extern int forPidDebug;

float pitchRealAngle = 0.0;
extern Location_Number_s Location_Number[];
//extern uint16_t pitchAngle, yawAngle;
//extern uint32_t flAngle, frAngle, blAngle, brAngle;
//extern uint16_t flSpeed, frSpeed, blSpeed, brSpeed;
extern uint8_t CReceive;
extern uint8_t rune_flag;
extern uint8_t GYRO_RESETED;
extern float ZGyroModuleAngle;
float yawAngleTarget = 0.0;
float pitchAngleTarget = 0.0;
int8_t flUpDown = 0, frUpDown = 0, blUpDown = 0, brUpDown = 0, allUpDown = 0;
int twist_state = 0;
int twist_count = 0;
int mm =0;

int16_t twist_target = 0;
void CMGMControlTask(void const * argument){
	static float yawAdd;
	static float pitchAdd;
	static uint8_t rune;
	while(1){
//  osSemaphoreWait(imurefreshGimbalSemaphoreHandle, osWaitForever);
		osSemaphoreWait(CMGMCanRefreshSemaphoreHandle, osWaitForever);
		if(IOPool_hasNextRead(upperIOPool, 0)){
		IOPool_getNextRead(upperIOPool, 0);
		yawAdd = IOPool_pGetReadData(upperIOPool, 0)->yawAdd;
		pitchAdd = IOPool_pGetReadData(upperIOPool, 0)->pitchAdd;
		rune = IOPool_pGetReadData(upperIOPool, 0)->rune;
		}
/*云台yaw轴*/
		if(IOPool_hasNextRead(GMYAWRxIOPool, 0)){
			uint16_t yawZeroAngle = yaw_zero;
			float yawRealAngle = 0.0;
			float gap_angle = 0.0;
			int16_t yawIntensity = 0;		
			IOPool_getNextRead(GMYAWRxIOPool, 0); 
			yawRealAngle = (IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle - yawZeroAngle) * 360 / 8192.0f;
			gap_angle  = (IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle - yawZeroAngle) * 360 / 8192.0f;
			
			
			
			NORMALIZE_ANGLE180(yawRealAngle);
			NORMALIZE_ANGLE180(gap_angle);	
		if(GYRO_RESETED == 2) {
				/*扭腰*/
			//试图用PID
				if (twist_state == 1){
//				CMRotatePID.ref = 0; //一定角度之间进行扭腰
//					twist_target = 30;
//					if (CMRotatePID.ref < twist_target){
//						twist_target = 30;
//			      CMRotatePID.ref = CMRotatePID.ref + 0.001f ;   //
//		        CMRotatePID.fdb = gap_angle;
//						fw_printfln("CMRotatePID.output:%f",CMRotatePID.output);
//					}
//			  	else{
//					  twist_target = -30;
//			      CMRotatePID.ref = CMRotatePID.ref - 0.001f ;   //
//		        CMRotatePID.fdb = gap_angle;
//				  }
//	         CMRotatePID.Calc(&CMRotatePID);   
//		       ChassisSpeedRef.rotate_ref = CMRotatePID.output;
//					fw_printfln("CMRotatePID.output:%f",CMRotatePID.output);
					
//					CMRotatePID.output = 0; //一定角度之间进行扭腰
//					twist_target = 20;
//					if (CMRotatePID.output < twist_target){
//						twist_target = 20;
//						CMRotatePID.output = CMRotatePID.output + 4;
//					}
//			  	else{
//					  twist_target = -20;
//						CMRotatePID.output = CMRotatePID.output - 4;
//				  }
//		       ChassisSpeedRef.rotate_ref = CMRotatePID.output;
					//fw_printfln("CMRotatePID.output:%f",CMRotatePID.output);
					
					CMRotatePID.output = 0; //一定角度之间进行扭腰
					mm = (twist_count / 600)%2 ;
					if (mm == 1){
						CMRotatePID.output = -10;
						twist_count = twist_count + 1;
					}
			  	if (mm == 0){
					  CMRotatePID.output = 10;
						twist_count = twist_count + 1;
				  }
					fw_printfln("mm:%d",mm);
		       ChassisSpeedRef.rotate_ref = CMRotatePID.output;
			}				
			else {
/******发送数据1  yaw角度*******/
				
/*底盘跟随编码器旋转PID计算*/
		 CMRotatePID.ref = 0;
		 CMRotatePID.fdb = yawRealAngle;
	   CMRotatePID.Calc(&CMRotatePID);   
		 ChassisSpeedRef.rotate_ref = CMRotatePID.output;
		//fw_printfln("CMRotatePID.output:%f",CMRotatePID.output);
//				ChassisSpeedRef.rotate_ref = 0;
//陀螺仪值获取
//		 yawRealAngle = -ZGyroModuleAngle;//此时底盘跟随已经设定完毕，yawrealangle的值改为复位后陀螺仪的绝对值，进行yaw轴运动设定
						
		//fw_printfln("GMYawEncoder.ecd_angle:%f",GMYawEncoder.ecd_angle);
			}
			yawRealAngle = -ZGyroModuleAngle;//此时底盘跟随已经设定完毕，yawrealangle的值改为复位后陀螺仪的绝对值，进行yaw轴运动设定
/*自瞄模式切换*/
			if(GetShootMode() == AUTO) {
				if((GetLocateState() == Locating) && (CReceive != 0))	{
				yawAngleTarget = yawRealAngle - yawAdd ;
				//fw_printfln("yawAdd:%f",yawAdd );
				CReceive--;
				}
//大神符
				else if((GetLocateState() == Located) && (rune_flag != 0)){
					if(GetRuneState() == AIMING){
						fw_printfln("rune:%d", rune);
						yawAngleTarget = Location_Number[rune - 1].yaw_position;
						rune_flag--;
					}
				}
		  }
		
						
//			MINMAX(yawAngleTarget, -45, 45);
// 进行yaw轴的控制pid
			
			yawIntensity = PID_PROCESS_Double(yawPositionPID,yawSpeedPID,yawAngleTarget,yawRealAngle,-gYroZs);

     //fw_printfln("yawRealAngle:%f",yawRealAngle);
			setMotor(GMYAW, yawIntensity);
		}
/*云台pitch轴*/
		if(IOPool_hasNextRead(GMPITCHRxIOPool, 0)){
			
			uint16_t pitchZeroAngle = pitch_zero;
			int16_t pitchIntensity = 0;
			
			IOPool_getNextRead(GMPITCHRxIOPool, 0);
			pitchRealAngle = -(IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle - pitchZeroAngle) * 360 / 8192.0;
			NORMALIZE_ANGLE180(pitchRealAngle);
/*******发送数据2 Pitch角度******/
//			fw_printfln("pitchRealAngle:%f",pitchRealAngle);
//自瞄模式切换
			if(GetShootMode() == AUTO) 
				{
				if((GetLocateState() == Locating) && (CReceive != 0))	{
			  //fw_printfln("pitchAdd:%f",pitchAdd );
			  pitchAngleTarget = pitchRealAngle + pitchAdd ;
				CReceive --;
				}
//大神符
				else if((GetLocateState() == Located) && (rune_flag != 0)){
					if(GetRuneState() == AIMING){
						pitchAngleTarget = Location_Number[rune - 1].pitch_position;
						rune_flag--;
					  }
				  }
		    }
			MINMAX(pitchAngleTarget, -25, 25);
			pitchIntensity = PID_PROCESS_Double(pitchPositionPID,pitchSpeedPID,pitchAngleTarget,pitchRealAngle,-gYroXs);
			//		fw_printfln("pitchIntensity:%d", pitchIntensity);
			setMotor(GMPITCH, pitchIntensity);
		}
//底盘电机 1 2 3 4	
//		ChassisSpeedRef.rotate_ref = 0;//取消底盘跟随

		if(IOPool_hasNextRead(CMFLRxIOPool, 0)){
			IOPool_getNextRead(CMFLRxIOPool, 0);
			Motor820RRxMsg_t *pData = IOPool_pGetReadData(CMFLRxIOPool, 0);
#ifdef Infantry_1_Aim
			CANReceiveMsgProcess_820R(pData, &CM2Encoder);
			CM2SpeedPID.ref =  ChassisSpeedRef.forward_back_ref*0.075 + ChassisSpeedRef.left_right_ref*0.075 + ChassisSpeedRef.rotate_ref;
			CM2SpeedPID.fdb = CM2Encoder.filter_rate;
#else
			CM2SpeedPID.ref =  -ChassisSpeedRef.forward_back_ref*0.075 + ChassisSpeedRef.left_right_ref*0.075 + ChassisSpeedRef.rotate_ref;
			CM2SpeedPID.ref = 160 * CM2SpeedPID.ref;
			CM2SpeedPID.fdb = pData->RotateSpeed;
#endif
		  CM2SpeedPID.Calc(&CM2SpeedPID);
		  setMotor(CMFR, CHASSIS_SPEED_ATTENUATION * CM2SpeedPID.output);
			
			//fw_printfln("GMYawEncoder.ecd_angle:%f",GMYawEncoder.ecd_angle);
		}
		  if(IOPool_hasNextRead(CMFRRxIOPool, 0)){
			IOPool_getNextRead(CMFRRxIOPool, 0);
			Motor820RRxMsg_t *pData = IOPool_pGetReadData(CMFRRxIOPool, 0);
			CANReceiveMsgProcess_820R(pData, &CM1Encoder);
#ifdef Infantry_1_Aim	
			CANReceiveMsgProcess_820R(pData, &CM1Encoder);				
		  CM1SpeedPID.ref =  -ChassisSpeedRef.forward_back_ref*0.075 + ChassisSpeedRef.left_right_ref*0.075 + ChassisSpeedRef.rotate_ref;
      CM1SpeedPID.fdb = CM1Encoder.filter_rate;
#else
			CM1SpeedPID.ref =  ChassisSpeedRef.forward_back_ref*0.075 + ChassisSpeedRef.left_right_ref*0.075 + ChassisSpeedRef.rotate_ref;	
			CM1SpeedPID.ref = 160 * CM1SpeedPID.ref;
			CM1SpeedPID.fdb = pData->RotateSpeed;
#endif
//		 fw_printfln("CM1SpeedPID.ref:%f",CM1SpeedPID.ref);
//		 fw_printfln("CM1Encoder.filter_rate:%d",CM1Encoder.filter_rate);
		  CM1SpeedPID.Calc(&CM1SpeedPID);
		  setMotor(CMFL, CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output);
		}
		if(IOPool_hasNextRead(CMBLRxIOPool, 0)){
			IOPool_getNextRead(CMBLRxIOPool, 0);
			Motor820RRxMsg_t *pData = IOPool_pGetReadData(CMBLRxIOPool, 0);
#ifdef Infantry_1_Aim
			CANReceiveMsgProcess_820R(pData, &CM3Encoder);
			CM3SpeedPID.ref =  ChassisSpeedRef.forward_back_ref*0.075 - ChassisSpeedRef.left_right_ref*0.075 + ChassisSpeedRef.rotate_ref;
			CM3SpeedPID.fdb = CM3Encoder.filter_rate;
#else		
			CM3SpeedPID.ref =  ChassisSpeedRef.forward_back_ref*0.075 - ChassisSpeedRef.left_right_ref*0.075 + ChassisSpeedRef.rotate_ref;
			CM3SpeedPID.ref = 160 * CM3SpeedPID.ref;
			CM3SpeedPID.fdb = pData->RotateSpeed;
#endif
		  CM3SpeedPID.Calc(&CM3SpeedPID);
		  setMotor(CMBL, CHASSIS_SPEED_ATTENUATION * CM3SpeedPID.output);
		}
		if(IOPool_hasNextRead(CMBRRxIOPool, 0)){
			IOPool_getNextRead(CMBRRxIOPool, 0);
			Motor820RRxMsg_t *pData = IOPool_pGetReadData(CMBRRxIOPool, 0);
#ifdef Infantry_1_Aim
			CANReceiveMsgProcess_820R(pData, &CM4Encoder);
			CM4SpeedPID.ref =  -ChassisSpeedRef.forward_back_ref*0.075 - ChassisSpeedRef.left_right_ref*0.075 + ChassisSpeedRef.rotate_ref;
			CM4SpeedPID.fdb = CM4Encoder.filter_rate;
#else
			CM4SpeedPID.ref =  -ChassisSpeedRef.forward_back_ref*0.075 - ChassisSpeedRef.left_right_ref*0.075 + ChassisSpeedRef.rotate_ref;
		  CM4SpeedPID.ref = 160 * CM4SpeedPID.ref;
#endif
			CM4SpeedPID.fdb = pData->RotateSpeed;
		  CM4SpeedPID.Calc(&CM4SpeedPID);
		  setMotor(CMBR, CHASSIS_SPEED_ATTENUATION * CM4SpeedPID.output);
		}
	}
}

}

