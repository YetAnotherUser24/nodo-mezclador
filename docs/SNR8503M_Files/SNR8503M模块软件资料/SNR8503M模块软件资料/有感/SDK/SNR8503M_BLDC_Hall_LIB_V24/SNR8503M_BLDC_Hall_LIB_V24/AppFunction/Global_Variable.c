/*******************************************************************************
 *
 *******************************************************************************/
#include "MC_Parameter.h"
#include "Global_Variable.h"
#include "motor_structure.h"
#include "SpeedRamp.h"
#include "MC_Drive.h"
#include "SNR_BLDC_HALL_V1p0.h"
#include "PID.h"

const char sVersion[10] = {1,2};        /* 程序版本 */

HALL_NEWVALUE_TypeDef	HALL_LIB;
Motor_TypeDef 				Motor;
LMotor_TypeDef 				User_sys;
stru_DiviComponents		HallAngle_Division;
stru_DiviComponents		Ele_Speed_Division;

unsigned char Read_hall_tab[6] = {0,0,0,0,0,0};

void Init_Parameter(void);
void variable_reset(void);

/**
  * @brief Initialization parameters
  * @retval None
  */
void Init_Parameter(void)
{
	static uint32_t	temp;
	
	// BLDC Structure  
	Motor.BLDC.u8FlagEnMotorRun = 0;
	Motor.BLDC.u8FlagMotorIsRunning = 0;
	Motor.BLDC.u8_ControlStep = 0;  			
	Motor.BLDC.u16_TimCnt = 0;
	
	// USER Structure
	Motor.USER.u16VSP_filt_value = 0;								// Speed command VSP filt value

	Motor.Control.u32closeloop_count = 0;					// Enter Motor close-loop cnt
	
	Motor.Control.u32MotorRotorBlock_cnt = 0;			// Motor Rotor Block cnt

	Motor.Control.u32MotorRPMFilte_cnt = 0;

	Motor.Control.u16MotorShakeDelayProtectCnt = 0;	

	// User_sys TaskScheduler time cnt
	User_sys.struTaskScheduler.bTimeCnt1ms = 0;         /* 1mS计数器 */
	User_sys.struTaskScheduler.nTimeCnt10ms = 0;        /* 10mS计数器 */
	User_sys.struTaskScheduler.nTimeCnt100ms = 0;       /* 100mS计数器 */  
	User_sys.struTaskScheduler.nTimeCnt500ms = 0;       /* 500mS计数器 */
	User_sys.struTaskScheduler.nMultiplex_cnt100ms = 0; /* 100ms延时SWD口复用*/
	
	// User_sys mBLDC_CtrProc
	User_sys.BLDC_SensorlessCtr.nSys_TimerPWM = 0;    	/* PWM周期计数Cnt */ 
	User_sys.BLDC_SensorlessCtr.nSpeed_ADC = 0;       	/* 速度开关ADC值 */
	User_sys.BLDC_SensorlessCtr.nBUS_Vol_ADC = 0;     	/* 母线电压ADC值 */
	User_sys.BLDC_SensorlessCtr.nMOS_NTC_ADC = 0;      	/* NTC温度ADC值 */	
	User_sys.BLDC_SensorlessCtr.nBAT_NTC_ADC = 0;     	/* 电池温度采样ADC */
	
	// User_sys BLDC_sensorlessCtr
	User_sys.BLDC_SensorlessCtr.communicate_Step = 0;   /* 当前换相电角度位置 */				
	
	User_sys.BLDC_SensorlessCtr.PeakCurrOffset = 0;   	/* 峰值电流偏置 */
	User_sys.BLDC_SensorlessCtr.peakBusCurrent = 0;   	/* 峰值电流 */	
	User_sys.BLDC_SensorlessCtr.peakBusCurrentIn = 0;   /* 限电流输入值 */		
	User_sys.BLDC_SensorlessCtr.u32Ibus_sum = 0;				/* 电流累加 */
	User_sys.BLDC_SensorlessCtr.u32Ibus_Filt = 0;				/* 平均电流 */
	User_sys.BLDC_SensorlessCtr.u32Ibus_Final = 0;			/* 最终电流 */
	User_sys.BLDC_SensorlessCtr.bIbus_Limit_Cur_Flag = 0;				/* 限电流标志位 */
	User_sys.BLDC_SensorlessCtr.u16CurLimt_PWMValue_Dec = 0;

	User_sys.BLDC_SensorlessCtr.get_offset_flg = 0;			/* offset error */
	User_sys.BLDC_SensorlessCtr.sys_error_flg = 0;			/* sysstem error */
	
	User_sys.BLDC_SensorlessCtr.MotorBlockCnt = 0;			/* Motor rotor block cnt */
	
	//temp = (MOTOR_SPEED_X/100/6)<<2;
	temp = (MOTOR_SPEED_X/100*6);
  Motor.Control.u32MotorRPM = 100;
	Motor.Control.u32MotorElePhaseValue[0] = temp;
  Motor.Control.u32MotorElePhaseValue[1] = temp;
  Motor.Control.u32MotorElePhaseValue[2] = temp;
  Motor.Control.u32MotorElePhaseValue[3] = temp;
  Motor.Control.u32MotorElePhaseValue[4] = temp;
  Motor.Control.u32MotorElePhaseValue[5] = temp;
  Motor.Control.u32MotorRPM_Filt = Motor.Control.u32MotorRPM;
  Motor.Control.u32MotorRPMFinal = Motor.Control.u32MotorRPM_Filt;
  Motor.Control.u32MotorRPM_sum = Motor.Control.u32MotorRPMFinal << 2;	
	
	Motor.Control.u32MotorElePeriodValueTemp = 0;
	Motor.Control.u32MotorElePeriodValue = 0;
	
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
	Motor.USER.SpeedPI_Prc = 0;
					
	Motor.Control.u8FlagPhase_COMPENSATION = 0;
	Motor.Control.u8PhaseChange_ON_COMPENSATION_flag = 0;
	
	Motor.Control.u16Dis_Phase_COMPENSATION_cnt = 0;
	Motor.Control.u16Phase_Compensation_cnt = 0;
	Motor.Control.u8Read_Hall_CWCCW_flag = 0;
	Motor.Control.u8Read_Hall_finish_flag = 0;
}

/**
  * @brief Reset parameters
  * @retval None
  */
void variable_reset(void)
{
	static uint32_t	temp;
	
	// BLDC Structure
	#if EN_IOSET_CWCCW
	Motor.BLDC.u8Direction = CWCCW_bIOInput;  
	#else
	Motor.BLDC.u8Direction = CW_CCW;  
	#endif
	   
	Motor.BLDC.u8FlagMotorIsRunning = 0;

	Motor.Control.u32closeloop_count = 0;					// Enter Motor close-loop cnt
	
	Motor.Control.u32MotorRotorBlock_cnt = 0;			// Motor Rotor Block cnt
	
	Motor.Control.u32MotorRPMFilte_cnt = 0;

	Motor.Control.u16MotorShakeDelayProtectCnt = 0;	

	// User_sys BLDC_sensorlessCtr
	User_sys.BLDC_SensorlessCtr.communicate_Step = 0;   /* 当前换相电角度位置 */
	
	User_sys.BLDC_SensorlessCtr.peakBusCurrent = 0;   			/* 峰值电流 */	
	User_sys.BLDC_SensorlessCtr.peakBusCurrentIn = 0;   		/* 限电流输入值 */		
	User_sys.BLDC_SensorlessCtr.u32Ibus_sum = 0;						/* 电流累加 */
	User_sys.BLDC_SensorlessCtr.u32Ibus_Filt = 0;						/* 平均电流 */
	User_sys.BLDC_SensorlessCtr.u32Ibus_Final = 0;					/* 最终电流 */
	User_sys.BLDC_SensorlessCtr.bIbus_Limit_Cur_Flag = 0;		/* 限电流标志位 */
	User_sys.BLDC_SensorlessCtr.u16CurLimt_PWMValue_Dec = 0;	
	
	User_sys.BLDC_SensorlessCtr.MotorBlockCnt = 0;					/* Motor rotor block cnt */

	//temp = (MOTOR_SPEED_X/100/6)<<2;
	temp = (MOTOR_SPEED_X/100*6);
  Motor.Control.u32MotorRPM = 100;
	Motor.Control.u32MotorElePhaseValue[0] = temp;
  Motor.Control.u32MotorElePhaseValue[1] = temp;
  Motor.Control.u32MotorElePhaseValue[2] = temp;
  Motor.Control.u32MotorElePhaseValue[3] = temp;
  Motor.Control.u32MotorElePhaseValue[4] = temp;
  Motor.Control.u32MotorElePhaseValue[5] = temp;
  Motor.Control.u32MotorRPM_Filt = Motor.Control.u32MotorRPM;
  Motor.Control.u32MotorRPMFinal = Motor.Control.u32MotorRPM_Filt;
  Motor.Control.u32MotorRPM_sum = Motor.Control.u32MotorRPMFinal << 2;	
	
	Motor.Control.u32MotorElePeriodValueTemp = 0;
	Motor.Control.u32MotorElePeriodValue = 0;

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
	
	InitPI();      								// PI参数初始化
	Motor.USER.SpeedPI_Prc = 0;
	
	Motor.Control.u8FlagPhase_COMPENSATION = 0;
	Motor.Control.u8PhaseChange_ON_COMPENSATION_flag = 0;
	
	Motor.Control.u16Dis_Phase_COMPENSATION_cnt = 0;
	Motor.Control.u16Phase_Compensation_cnt = 0;
	Motor.Control.u8Read_Hall_CWCCW_flag = 0;
	Motor.Control.u8Read_Hall_finish_flag = 0;
}
/* ------------------------------END OF FILE------------------------------------ */


