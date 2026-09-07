#ifndef __MOTOR_STRUCTURE_H
#define __MOTOR_STRUCTURE_H

#include "basic.h"
#include "snr8503x_dsp.h"
//-----------------------------------------------------------------------------------------------
typedef struct
{
		__IO  uint8_t   u8HallValueJudge;          
		__IO  uint8_t   u8HallValueRenew;     
		__IO  uint8_t   u8HallValueOld;       
		__IO  uint8_t   u8HallValueNext; 

		__IO  uint8_t		u8HALL_Run_Value;		
	
		__IO  uint8_t   u8HallValue60Degree; 
		__IO  uint8_t   u8HallValue60Degree_temp; 
	
		__IO  uint8_t   u8Direction;          	// direction
		__IO  uint8_t  	u8FlagEnMotorRun;				// Enable Motor run
		__IO  uint8_t  	u8FlagMotorIsRunning;		// Motor is running
		__IO  uint8_t		u8_ControlStep;  				// Motor control state
		__IO  uint16_t 	u16_TimCnt;							// Motor control state 1 delay cnt
	
		__IO  uint32_t 	u32Freerun_Det_cnt;
	
}BLDC_TypeDef;
extern BLDC_TypeDef    	BLDC;

//-----------------------------------------------------------------------------------------------
typedef struct
{
		__IO uint16_t 		u16VSP_filt_value;		// Speed command VSP filt value
	  __IO uint16_t 	  SpeedPI_Prc;						//SpeedPID loop time
}USER_TypeDef;
extern USER_TypeDef    	USER;

//-----------------------------------------------------------------------------------------------
typedef struct
{
	///***********  Motor control  *****/// 
	__IO  uint32_t 	u32closeloop_count;					// Enter Motor close-loop cnt
	__IO  uint32_t 	u32MotorRotorBlock_cnt;			// Motor Rotor Block cnt
	
	__IO  uint32_t 	u32MotorElePhaseValue[6];
	__IO  uint32_t 	u32MotorElePeriodValue;
	__IO  uint32_t 	u32MotorElePeriodValueTemp;
	__IO  uint32_t 	u32MotorEleRPM;
	__IO  uint32_t 	u32MotorEleRPM_Filt;
	__IO  uint32_t 	u32MotorEleRPM_Final;
	__IO  uint32_t 	u32MotorEleRPM_sum;
	
	__IO  int32_t 	i32_AdvanceAngleValue;
	
	__IO  uint32_t 	u32MotorRPMFilte_cnt;
	__IO  uint8_t 	u32MotorRAMP_OK_flag;
	
	__IO  uint32_t 	u32MotorRPM;
	__IO  uint32_t 	u32MotorRPM_sum;					/* 累加 */
	__IO  uint32_t 	u32MotorRPM_Filt;					/* 平均 */
	__IO  uint32_t 	u32MotorRPM_Final;				/* 最终 */
	__IO  uint32_t 	u32MotorRPMFinal;
	
	__IO  uint32_t 	u32MCPWM_TH_temp;
	
	__IO  uint16_t 	u16MotorShakeDelayProtectCnt;	

	__IO  uint8_t 	u8FlagDetectMotorFreerunAgainstwind;
	__IO  uint32_t 	u32MotorFreerunAgainstwindDelay;

	__IO  uint8_t 	u8FlagPhase_COMPENSATION;
	__IO  uint8_t 	u8PhaseChange_ON_COMPENSATION_flag;
	
	__IO  uint16_t 	u16Dis_Phase_COMPENSATION_cnt;	
	__IO  uint32_t 	u32Hall_Angle_Act_Value;
	
	__IO  uint32_t 	u32Cal_Hall_advance_Angle;
	__IO  uint32_t 	u32Cal_Hall_Angle_Act_Cmd;
	__IO  uint8_t 	u8Cal_Hall_Angle_OK_flag;

	__IO  uint16_t 	u16Phase_Compensation_cnt;
	__IO  uint8_t 	u8Read_Hall_CWCCW_flag;
	__IO  uint8_t 	u8Read_Hall_finish_flag;
	
} BLDC_CMP_TypeDef;
extern	BLDC_CMP_TypeDef	Control;

//-----------------------------------------------------------------------------------------------
typedef struct
{
    USER_TypeDef    	USER;
    BLDC_TypeDef    	BLDC;
		BLDC_CMP_TypeDef	Control;
} Motor_TypeDef;
extern Motor_TypeDef Motor;

extern stru_DiviComponents	HallAngle_Division;
extern stru_DiviComponents	Ele_Speed_Division;
//-----------------------------------------------------------------------------------------------
#endif

