/*******************************************************************************
 * 版权所有 (C)2015, SNANER SEMICONDUCTOR Co.ltd
 *
 * 文件名称： Global_Variable.h
 * 文件标识：
 * 内容摘要： 全局变量声明文件
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Li
 * 完成日期： 2020年8月5日
 *
 * 修改记录1：
 * 修改日期：2020年8月5日
 * 版 本 号：V 1.0
 * 修 改 人：Li
 * 修改内容：创建
 *
 *******************************************************************************/

#ifndef __GLOBAL_VARIABLE__
#define __GLOBAL_VARIABLE__

#include "mc_type.h"
#include "basic.h"


#define SHORT_ERROR          BIT0  /* 短路故障 */
#define LOW_VOL_ERROR        BIT1  /* 低压故障 */
#define HIG_VOL_ERROR        BIT2  /* 过压故障 */
#define BLOCK_ERROR          BIT3  /* 堵转故障 */
#define DC_OFFSET_ERROR      BIT4  /* 直流偏置故障 */
#define MOS_OVER_ERROR       BIT5  /* MOS过温故障 */
#define MOS_LOW_ERROR        BIT6  /* MOS低温故障 */
#define BAT_OVER_ERROR       BIT7  /* 电池过温故障 */
#define BAT_LOW_ERROR        BIT8  /* 电池低温故障 */
#define OVER_LOAD_ERROR      BIT9  /* 过流故障 */
#define PHASE_DROP_ERROR     BIT10 /* 缺相故障 */
#define MOSFET_ERROR         BIT11 /* MOS自检故障 */

extern unsigned char Read_hall_tab[6];

typedef struct
{
    volatile u16 nSys_TimerPWM;   /* PWM周期计数Cnt */ 
  
    s16 nSpeed_ADC;       					/* 速度开关ADC值 */
		s16 nBUS_Vol_ADC;      					/* 母线电压ADC值 */
		s16 nMOS_NTC_ADC;      					/* NTC温度ADC值 */	
		s16 nBAT_NTC_ADC;      					/* 电池温度采样ADC */
	
    u8 	communicate_Step;   				/* 当前换相电角度位置 */
		u8 	Hallstate;   								/* 当前换相霍尔状态 */

    s16 PeakCurrOffset;   					/* 峰值电流偏置 */
    s16 peakBusCurrent;   					/* 峰值电流 */	
		u16 peakBusCurrentIn;						/* 限电流输入值 */	
		u32 u32Ibus_sum;								/* 电流累加 */
		u32 u32Ibus_Filt;								/* 平均电流 */
		u32 u32Ibus_Final;							/* 最终电流 */
		u8  bIbus_Limit_Cur_Flag;
		u16 u16CurLimt_PWMValue_Dec;
	
    s16 PhaseA_offset;    					/* A反电势ADC offset值 */
    s16 PhaseB_offset;    					/* B反电势ADC offset值 */
    s16 PhaseC_offset;    					/* C反电势ADC offset值 */
		u8 	get_offset_flg;							/* offset error */
		u16 sys_error_flg;							/* sysstem error */
	
		u16 MotorBlockCnt;							/* 堵转保护 */
	
		u32 u32SW_LimitCurrent_Value;		/* 限电流保护 */
	
} stru_BLDC_sensorlessCtr;

extern const char sVersion[10];                      /* 程序版本 */


typedef struct
{
	stru_TaskSchedulerDef 		struTaskScheduler;    	/* 任务调度结构体 */
	stru_BLDC_sensorlessCtr 	BLDC_SensorlessCtr; 		/* 换相控制结构体 */
}LMotor_TypeDef;
extern LMotor_TypeDef User_sys;

extern void DebugPWM_OutputFunction(void);

#endif

/* ********************** (C) COPYRIGHT SNANER SEMICONDUCTOR ******************** */
/* ------------------------------END OF FILE------------------------------------ */
