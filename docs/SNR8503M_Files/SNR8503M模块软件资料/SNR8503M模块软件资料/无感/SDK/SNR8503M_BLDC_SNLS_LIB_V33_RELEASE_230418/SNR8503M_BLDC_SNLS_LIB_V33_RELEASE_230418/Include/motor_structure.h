#ifndef __MOTOR_STRUCTURE_H
#define __MOTOR_STRUCTURE_H

#include "basic.h"

//-----------------------------------------------------------------------------------------------
typedef struct
{
		__IO  uint8_t   u8Direction;          	// direction
		__IO  uint8_t  	u8FlagEnMotorRun;				// Enable Motor run
		__IO  uint8_t  	u8FlagMotorIsRunning;		// Motor is running
		__IO  uint8_t		u8_ControlStep;  				// Motor control state
		__IO  uint16_t 	u16_TimCnt;							// Motor control state 1 delay cnt
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
	///*********** COMP Sensorless Motor control  *****/// 
	__IO  uint32_t 	u32COMP_OUT_temp;						// Read COMP Value temp
//	__IO  uint8_t  	u8COMP_OUT_Value;						// COMP Value
	
	__IO  uint32_t 	u32phase_cnt;								// Motor phase time cnt, motor single step time 		
//	__IO  uint32_t 	u32phase_min;								// Motor Max speed
//	__IO  uint32_t 	u32phase_max;								// Motor Min speed
//	__IO  uint8_t 	u8phase_rise;								// Sensorless detect edge
//	__IO  uint32_t 	u32speed_cnt; 							// Motor commucation delay time counter 

	__IO  uint8_t		u8ControlStep;  						// Motor control status
	__IO  uint8_t		u8ramp_cnt;									// Motor ramp step cnt
	__IO  uint32_t 	u32RampTime; 								// Ramp-time cnt while Align

	__IO  uint32_t	u32ElectricalStepValue;			// Motor single step time
	__IO  uint32_t	u32ElectricalStepcnt;				// Motor single step time cnt
	__IO  uint32_t	u32ElectricalPeriod;				// Motor Electrical Period time record
	__IO  uint32_t	u32ElectricalPeriodcnt;			// Motor Electrical Period time cnt

	__IO  uint8_t 	u8FlagFreewheelAvoiding;		// Avoid motor-Freewheel flag
	__IO  uint32_t 	u32FreewheelAvoidcnt;				// Avoid motor-Freewheel delay time cnt
	__IO  uint32_t 	u32FreewheelAvoidDey;				// Avoid motor-Freewheel delay time
	
	__IO  uint32_t 	u32closeloop_count;					// Enter Motor close-loop cnt
	__IO  uint32_t 	u32MotorRotorBlock_cnt;			// Motor Rotor Block cnt
	
	__IO  uint32_t 	u32MotorElePhaseValue[6];
	__IO  uint32_t 	u32MotorElePeriodValue;
	__IO  uint32_t 	u32MotorRPM;
	__IO  uint32_t 	u32MotorRPMFilte_cnt;
	
	__IO  uint32_t 	u32MotorElePhaseCnt[7];
	__IO  uint32_t 	u32MotorElePhaseAvg;	
	__IO  uint32_t 	u32MotorElePhaseAvg_2;	
	__IO  uint32_t 	u32MotorElePhaseAvg_1_2;	
	__IO  uint32_t 	u32MotorElePhaseAvg_1_4;	
	__IO  uint32_t 	u32MotorElePhaseAvg_3_2;	
	__IO  uint8_t 	u8FlagGetMotorElePeriod;	
	__IO  uint16_t 	u16MotorShakeCnt;	
	__IO  uint16_t 	u16MotorShakeDelayProtectCnt;	

	//* Free-running detect *//
	__IO  uint8_t 	u8FlagDetectMotorFreerunning;
	__IO  uint8_t 	u8FlagDetectMotorFreerunTailwind;
	__IO  uint8_t 	u8FlagDetectMotorFreerunAgainstwind;
	__IO  uint8_t 	u8FlagDetectMotorFreerunnfinish;
	__IO  uint8_t 	u8Freerunstepchangecnt;
	__IO  uint8_t 	u8FreerunstepOld;
//	__IO  uint8_t 	u8FreerunstepNew;
	__IO  uint32_t 	u32DetectFreerunTimecnt;
//	__IO  uint32_t 	u32DetectFreerunstep_1_cnt;
//	__IO  uint32_t 	u32DetectFreerunstep_0_cnt;
//	__IO  uint32_t 	u32DetectFreerunstep_0_Mincnt;
	__IO  uint32_t 	u32speed_cnt_temp;
	__IO  uint32_t 	u32MotorFreerunAgainstwindDelay;
	
	// Charge
	__IO  uint32_t 	u32ChargeTime_count;
	
} BLDC_CMP_TypeDef;
extern	BLDC_CMP_TypeDef	BLDC_CMP;

//-----------------------------------------------------------------------------------------------
typedef struct
{
    USER_TypeDef    	USER;
    BLDC_TypeDef    	BLDC;
		BLDC_CMP_TypeDef	BLDC_CMP;
} Motor_TypeDef;
extern Motor_TypeDef Motor_1M1st;

////typedef struct
////{
////	__IO  uint8_t		u8CMP_Zero_OUT_Value;	
////	__IO  uint8_t		u8CMP_Zero_phase_rise;		
////	__IO  uint32_t	u32CMP_Zero_speed_cnt;		
////	__IO  uint32_t	u32CMP_Zero_phase_max;
////	__IO  uint32_t	u32CMP_Zero_phase_min;
////	
////	__IO  uint32_t	u32CMP_Freerunstep_0_Mincnt;
////	__IO  uint32_t	u32CMP_Freerunstep_0_cnt;
////	__IO  uint32_t	u32CMP_Freerunstep_1_cnt;
////	__IO  uint8_t		u8CMP_FreerunstepNew;
////} CMP_Zero_TypeDef;
////extern	CMP_Zero_TypeDef	CMP_ZeroCross_lib;

//-----------------------------------------------------------------------------------------------
#endif

