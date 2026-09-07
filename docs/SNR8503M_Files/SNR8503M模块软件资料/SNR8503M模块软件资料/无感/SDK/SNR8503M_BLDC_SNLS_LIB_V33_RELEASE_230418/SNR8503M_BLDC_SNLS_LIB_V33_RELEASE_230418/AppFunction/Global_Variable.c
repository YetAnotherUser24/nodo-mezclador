/*******************************************************************************
 *
 *******************************************************************************/
#include "MC_Parameter.h"
#include "Global_Variable.h"
#include "motor_structure.h"
#include "SpeedRamp.h"
#include "MC_Drive.h"
#include "Lib_Zero_detect.h"
#include "PID.h"

const char sVersion[10] = {1,2};        /* 程序版本 */

CMP_Zero_TypeDef	CMP_ZeroCross_lib;
Motor_TypeDef 		Motor_1M1st;
LMotor_TypeDef 		User_sys;

void Init_Parameter(void);
void variable_reset(void);

/**
  * @brief Initialization parameters
  * @retval None
  */
void Init_Parameter(void)
{
	// BLDC Structure  
	
	Motor_1M1st.BLDC.u8FlagEnMotorRun = 0;
	Motor_1M1st.BLDC.u8FlagMotorIsRunning = 0;
	Motor_1M1st.BLDC.u8_ControlStep = 0;  			
	Motor_1M1st.BLDC.u16_TimCnt = 0;
	
	// USER Structure
	Motor_1M1st.USER.u16VSP_filt_value = 0;											// Speed command VSP filt value
	
	// BLDC COMP Sensorless control
	Motor_1M1st.BLDC_CMP.u32COMP_OUT_temp = 0;						// Read COMP Value temp
	CMP_ZeroCross_lib.u8CMP_Zero_OUT_Value = 0;

	Motor_1M1st.BLDC_CMP.u32phase_cnt = 0;								// Motor phase time cnt, motor single step time 		
	CMP_ZeroCross_lib.u32CMP_Zero_phase_max = MAX_SPEED_CNT;
	CMP_ZeroCross_lib.u32CMP_Zero_phase_min = MIN_SPEED_CNT;
	CMP_ZeroCross_lib.u8CMP_Zero_phase_rise = 0;					// Sensorless detect edge
	CMP_ZeroCross_lib.u32CMP_Zero_speed_cnt = MAX_SPEED_CNT; 		// Motor commucation time counter 

	Motor_1M1st.BLDC_CMP.u8ControlStep = 0;  							// Motor control status
	Motor_1M1st.BLDC_CMP.u8ramp_cnt = 0;									// Motor ramp step cnt
	Motor_1M1st.BLDC_CMP.u32RampTime = 0; 								// Ramp-time cnt while Align

	Motor_1M1st.BLDC_CMP.u32ElectricalStepValue = MAX_SPEED_CNT;			// Motor single step time
	Motor_1M1st.BLDC_CMP.u32ElectricalStepcnt = 0;										// Motor single step time cnt
	Motor_1M1st.BLDC_CMP.u32ElectricalPeriod = (6*MAX_SPEED_CNT);			// Motor Electrical Period time record
	Motor_1M1st.BLDC_CMP.u32ElectricalPeriodcnt = 0;									// Motor Electrical Period time cnt

	Motor_1M1st.BLDC_CMP.u8FlagFreewheelAvoiding = 0;			// Avoid motor-Freewheel flag
	Motor_1M1st.BLDC_CMP.u32FreewheelAvoidcnt = 0;				// Avoid motor-Freewheel delay time cnt
	Motor_1M1st.BLDC_CMP.u32FreewheelAvoidDey = 0;				// Avoid motor-Freewheel delay time

	Motor_1M1st.BLDC_CMP.u32closeloop_count = 0;					// Enter Motor close-loop cnt
	
	Motor_1M1st.BLDC_CMP.u32MotorRotorBlock_cnt = 0;			// Motor Rotor Block cnt

	Motor_1M1st.BLDC_CMP.u32MotorRPMFilte_cnt = 0;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseValue[0] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseValue[1] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseValue[2] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseValue[3] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseValue[4] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseValue[5] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePeriodValue = 6*MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorRPM = 0;

	Motor_1M1st.BLDC_CMP.u32MotorElePhaseCnt[0] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseCnt[1] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseCnt[2] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseCnt[3] = MAX_SPEED_CNT;	
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseCnt[4] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseCnt[5] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseCnt[6] = MAX_SPEED_CNT;
	
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseAvg = 0;	
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseAvg_2 = 0;	
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseAvg_1_2 = 0;	
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseAvg_1_4 = 0;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseAvg_3_2 = 0;
	Motor_1M1st.BLDC_CMP.u8FlagGetMotorElePeriod = 0;	
	Motor_1M1st.BLDC_CMP.u16MotorShakeCnt = 0;	
	Motor_1M1st.BLDC_CMP.u16MotorShakeDelayProtectCnt = 0;	

	Motor_1M1st.BLDC_CMP.u8FlagDetectMotorFreerunning = 0;
	Motor_1M1st.BLDC_CMP.u8FlagDetectMotorFreerunTailwind = 0;
	Motor_1M1st.BLDC_CMP.u8FlagDetectMotorFreerunAgainstwind = 0;
	Motor_1M1st.BLDC_CMP.u8FlagDetectMotorFreerunnfinish = 0;
	Motor_1M1st.BLDC_CMP.u8Freerunstepchangecnt = 0;
	Motor_1M1st.BLDC_CMP.u8FreerunstepOld = 0;
	CMP_ZeroCross_lib.u8CMP_FreerunstepNew = 0;
	Motor_1M1st.BLDC_CMP.u32DetectFreerunTimecnt = 0;
	CMP_ZeroCross_lib.u32CMP_Freerunstep_1_cnt = 0;
	CMP_ZeroCross_lib.u32CMP_Freerunstep_0_cnt = 0;
	CMP_ZeroCross_lib.u32CMP_Freerunstep_0_Mincnt = MIN_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32speed_cnt_temp = 0;
	Motor_1M1st.BLDC_CMP.u32MotorFreerunAgainstwindDelay = 0;
	
	// Charge
	Motor_1M1st.BLDC_CMP.u32ChargeTime_count = 0;

	// User_sys TaskScheduler time cnt
	User_sys.struTaskScheduler.bTimeCnt1ms = 0;         /* 1mS计数器 */
	User_sys.struTaskScheduler.nTimeCnt10ms = 0;        /* 10mS计数器 */
	User_sys.struTaskScheduler.nTimeCnt100ms = 0;       /* 100mS计数器 */  
	User_sys.struTaskScheduler.nTimeCnt500ms = 0;       /* 500mS计数器 */
	
	// User_sys mBLDC_CtrProc
	User_sys.BLDC_SensorlessCtr.nSys_TimerPWM = 0;          	/* PWM周期计数Cnt */ 
	User_sys.BLDC_SensorlessCtr.nSpeed_ADC = 0;            	/* 速度开关ADC值 */
	User_sys.BLDC_SensorlessCtr.nBUS_Vol_ADC = 0;          	/* 母线电压ADC值 */
	User_sys.BLDC_SensorlessCtr.nMOS_NTC_ADC = 0;           	/* NTC温度ADC值 */	
	User_sys.BLDC_SensorlessCtr.nBAT_NTC_ADC = 0;           	/* 电池温度采样ADC */
	
	// User_sys BLDC_sensorlessCtr
	User_sys.BLDC_SensorlessCtr.communicate_Step = 0;   /* 当前换相电角度位置 */				
	User_sys.BLDC_SensorlessCtr.Hallstate = CMP_PhaseTab[User_sys.BLDC_SensorlessCtr.communicate_Step];		/* 当前换相霍尔状态 */
	
	User_sys.BLDC_SensorlessCtr.PeakCurrOffset = 0;   	/* 峰值电流偏置 */
	User_sys.BLDC_SensorlessCtr.peakBusCurrent = 0;   	/* 峰值电流 */	
	User_sys.BLDC_SensorlessCtr.peakBusCurrentIn = 0;   /* 限电流输入值 */		
	User_sys.BLDC_SensorlessCtr.u32Ibus_sum = 0;				/* 电流累加 */
	User_sys.BLDC_SensorlessCtr.u32Ibus_Filt = 0;				/* 平均电流 */
	User_sys.BLDC_SensorlessCtr.u32Ibus_Final = 0;			/* 最终电流 */
	User_sys.BLDC_SensorlessCtr.bIbus_Limit_Cur_Flag = 0;				/* 限电流标志位 */
	User_sys.BLDC_SensorlessCtr.u16CurLimt_PWMValue_Dec = 0;
	
	User_sys.BLDC_SensorlessCtr.PhaseA_offset = 0;    	/* A反电势ADC offset值 */
	User_sys.BLDC_SensorlessCtr.PhaseB_offset = 0;    	/* B反电势ADC offset值 */
	User_sys.BLDC_SensorlessCtr.PhaseC_offset = 0;    	/* C反电势ADC offset值 */
	User_sys.BLDC_SensorlessCtr.get_offset_flg = 0;			/* offset error */
	User_sys.BLDC_SensorlessCtr.sys_error_flg = 0;			/* sysstem error */
	
	User_sys.BLDC_SensorlessCtr.MotorBlockCnt = 0;			/* Motor rotor block cnt */
	
	RP.OutEn = 0;
	RP.State = 0;
	RP.Out = 0;
	RPValue.Dest = 0;
	RPValue.Act = 0;
	
	SPEED_Cmd.OutEn = 0;
	SPEED_Cmd.State = 0;
	SPEED_Cmd.Out = 0;
	SPEEDValue.Dest = 0;
	SPEEDValue.Act = 0;			
					
	InitPI();      //PI参数初始化
	Motor_1M1st.USER.SpeedPI_Prc = 0;
}

/**
  * @brief Reset parameters
  * @retval None
  */
void variable_reset(void)
{
	// BLDC Structure
	#if EN_IOSET_CWCCW
	Motor_1M1st.BLDC.u8Direction = CWCCW_bIOInput;  
	#else
	Motor_1M1st.BLDC.u8Direction = CW_CCW;  
	#endif
	   
	Motor_1M1st.BLDC.u8FlagMotorIsRunning = 0;
	
	// BLDC COMP Sensorless control
	Motor_1M1st.BLDC_CMP.u32phase_cnt = 0;								// Motor phase time cnt, motor single step time 		
	CMP_ZeroCross_lib.u8CMP_Zero_phase_rise = 0;					// Sensorless detect edge
	CMP_ZeroCross_lib.u32CMP_Zero_speed_cnt = MAX_SPEED_CNT; 		// Motor commucation time counter 

	Motor_1M1st.BLDC_CMP.u8ControlStep = 0;  							// Motor control status
	Motor_1M1st.BLDC_CMP.u8ramp_cnt = 0;									// Motor ramp step cnt
	Motor_1M1st.BLDC_CMP.u32RampTime = 0; 								// Ramp-time cnt while Align

	Motor_1M1st.BLDC_CMP.u32ElectricalStepValue = MAX_SPEED_CNT;					// Motor single step time
	Motor_1M1st.BLDC_CMP.u32ElectricalStepcnt = MAX_SPEED_CNT;						// Motor single step time cnt
	Motor_1M1st.BLDC_CMP.u32ElectricalPeriod = (6*MAX_SPEED_CNT);					// Motor Electrical Period time record
	Motor_1M1st.BLDC_CMP.u32ElectricalPeriodcnt = (6*MAX_SPEED_CNT);			// Motor Electrical Period time cnt

	Motor_1M1st.BLDC_CMP.u8FlagFreewheelAvoiding = 0;			// Avoid motor-Freewheel flag
	Motor_1M1st.BLDC_CMP.u32FreewheelAvoidcnt = 0;				// Avoid motor-Freewheel delay time cnt
	Motor_1M1st.BLDC_CMP.u32FreewheelAvoidDey = 0;				// Avoid motor-Freewheel delay time

	Motor_1M1st.BLDC_CMP.u32closeloop_count = 0;					// Enter Motor close-loop cnt
	
	Motor_1M1st.BLDC_CMP.u32MotorRotorBlock_cnt = 0;			// Motor Rotor Block cnt
	
	Motor_1M1st.BLDC_CMP.u32MotorRPMFilte_cnt = 0;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseValue[0] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseValue[1] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseValue[2] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseValue[3] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseValue[4] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseValue[5] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePeriodValue = 6*MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorRPM = 0;
	
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseCnt[0] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseCnt[1] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseCnt[2] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseCnt[3] = MAX_SPEED_CNT;	
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseCnt[4] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseCnt[5] = MAX_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseCnt[6] = MAX_SPEED_CNT;
	
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseAvg = 0;	
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseAvg_2 = 0;	
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseAvg_1_2 = 0;	
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseAvg_1_4 = 0;
	Motor_1M1st.BLDC_CMP.u32MotorElePhaseAvg_3_2 = 0;
	Motor_1M1st.BLDC_CMP.u8FlagGetMotorElePeriod = 0;	
	Motor_1M1st.BLDC_CMP.u16MotorShakeCnt = 0;	
	Motor_1M1st.BLDC_CMP.u16MotorShakeDelayProtectCnt = 0;	
	
	Motor_1M1st.BLDC_CMP.u8FlagDetectMotorFreerunning = 0;
	Motor_1M1st.BLDC_CMP.u8FlagDetectMotorFreerunTailwind = 0;
	Motor_1M1st.BLDC_CMP.u8FlagDetectMotorFreerunAgainstwind = 0;
	Motor_1M1st.BLDC_CMP.u8FlagDetectMotorFreerunnfinish = 0;
	Motor_1M1st.BLDC_CMP.u8Freerunstepchangecnt = 0;
	Motor_1M1st.BLDC_CMP.u8FreerunstepOld = 0;
	CMP_ZeroCross_lib.u8CMP_FreerunstepNew = 0;
	Motor_1M1st.BLDC_CMP.u32DetectFreerunTimecnt = 0;
	CMP_ZeroCross_lib.u32CMP_Freerunstep_1_cnt = 0;
	CMP_ZeroCross_lib.u32CMP_Freerunstep_0_cnt = 0;
	CMP_ZeroCross_lib.u32CMP_Freerunstep_0_Mincnt = MIN_SPEED_CNT;
	Motor_1M1st.BLDC_CMP.u32speed_cnt_temp = 0;
	Motor_1M1st.BLDC_CMP.u32MotorFreerunAgainstwindDelay = 0;
	
	// Charge
	Motor_1M1st.BLDC_CMP.u32ChargeTime_count = 0;
	
	// User_sys BLDC_sensorlessCtr
	User_sys.BLDC_SensorlessCtr.communicate_Step = 0;   /* 当前换相电角度位置 */
	User_sys.BLDC_SensorlessCtr.Hallstate = CMP_PhaseTab[User_sys.BLDC_SensorlessCtr.communicate_Step];		/* 当前换相霍尔状态 */
	
	User_sys.BLDC_SensorlessCtr.peakBusCurrent = 0;   			/* 峰值电流 */	
	User_sys.BLDC_SensorlessCtr.peakBusCurrentIn = 0;   		/* 限电流输入值 */		
	User_sys.BLDC_SensorlessCtr.u32Ibus_sum = 0;						/* 电流累加 */
	User_sys.BLDC_SensorlessCtr.u32Ibus_Filt = 0;						/* 平均电流 */
	User_sys.BLDC_SensorlessCtr.u32Ibus_Final = 0;					/* 最终电流 */
	User_sys.BLDC_SensorlessCtr.bIbus_Limit_Cur_Flag = 0;		/* 限电流标志位 */
	User_sys.BLDC_SensorlessCtr.u16CurLimt_PWMValue_Dec = 0;	
	
	User_sys.BLDC_SensorlessCtr.MotorBlockCnt = 0;			/* Motor rotor block cnt */
	
	RP.OutEn = 0;
	RP.State = 0;
	RP.Out = 0;
	RPValue.Dest = 0;
	RPValue.Act = 0;
	
	SPEED_Cmd.OutEn = 0;
	SPEED_Cmd.State = 0;
	SPEED_Cmd.Out = 0;
	SPEEDValue.Dest = 0;
	SPEEDValue.Act = 0;				
	
	InitPI();      //PI参数初始化
	Motor_1M1st.USER.SpeedPI_Prc = 0;
}



/* ------------------------------END OF FILE------------------------------------ */
