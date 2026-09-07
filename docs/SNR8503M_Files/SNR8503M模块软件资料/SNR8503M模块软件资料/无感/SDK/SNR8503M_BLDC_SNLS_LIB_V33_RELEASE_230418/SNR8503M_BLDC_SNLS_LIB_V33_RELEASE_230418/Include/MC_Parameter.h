/*******************************************************************************
 * 版权所有 (C)2019, SNANER SEMICONDUCTOR Co.ltd
 *
 * 文件名称： parameter.h
 * 文件标识：
 * 内容摘要： parameter config
 * 其它说明： 无
 * 当前版本： V1.0
 * 作    者： Li
 * 完成日期： 2020年8月18日
 *
 *******************************************************************************/
 
/*------------------------------prevent recursive inclusion -------------------*/ 
#ifndef __PARAMETER_H
#define __PARAMETER_H

#include "basic.h"
#include "hardware_config.h"

/* -------------------------------存储地址相关定义---------------------------- */ 
#define MEMORY_DATA_ADDR               0x7800

/* ---------------------------------功能相关定义------------------------------ */
#define ENABLE_FUNCTION                1
#define DISABLE_FUNCTION               0

/* --------------------------------- 硬件测试PWM输出功能 ------------------------------ */
#define DEBUG_PWM_OUTPUT               DISABLE_FUNCTION   //ENABLE_FUNCTION       /* PWM在调试的时候输出一定占空比 */

/* ----------------------PWM 频率及死区定义----------------------------------- */
#define MCU_MCLK                       (48000000LL)       /* PWM模块运行主频 */ 
#define PWM_MCLK                       ((u32)MCU_MCLK)    /* PWM模块运行主频 */
#define PWM_PRSC                       ((u8)0)            /* PWM模块运行预分频器 */
#define PWM_FREQ                       ((u16)16000)       /* PWM斩波频率 */

/* 电机控制PWM 周期计数器值 */
#define PWM_PERIOD                     ((u16)(PWM_MCLK/(u32)(2*PWM_FREQ*(PWM_PRSC+1))))
#define MAX_PWM_DUTY                   PWM_PERIOD							  /* 当PWM斩波频率为16K时，1500为最大占空比(全开) */
#define MIN_PWM_DUTY                   ((u16)(0.1*PWM_PERIOD)) 	/* 当PWM斩波频率为16K时，150为最小占空比 */
#define LIMT_PWM_DUTY                  ((u16)(PWM_PERIOD - MIN_PWM_DUTY))

/* -----------------------------Hardware Parameter---------------------------- */
#define ADC_SUPPLY_VOLTAGE             (3.6)              //单位: V  ADC基准电压，3.6或者2.4,大部分应用选择3.6
/*OPA采样匹配电阻 1K欧， 200:10=(200/(10.0+1.0)=18.18倍 */
/*相线最大采样电流值为   3.6/0.004/(200/(10.0+1.0)) = 44.5A*/
/*在实际项目中要注意合理设置最大采样电流值，一般按照3 倍过载来设计*/
#define AMPLIFICATION_GAIN             (18.18)          	//运放放大倍数
#define RSHUNT                         (0.004)            //单位: Ω  采样电阻阻值
#define VOLTAGE_SHUNT_RATIO            (1.0/(33.0+1.0)) 	//母线电压分压比    (下拉电阻/(上拉电阻+下拉电阻))

/* ------------------------------ADC校准相关参数设置---------------------------- */
#define CALIB_SAMPLES                  (512)    //ADC偏置校准次数，不可修改
#define OFFSET_THD                     (3500)   //ADC偏置误差阈值，不用修改
#define PHASE_OFFSET_MAX               (250)			
#define PHASE_OFFSET_MIN               (0)			

/* ----------------------------Current Lim SHORT-------------------------- */
#define SHORT_BUS_CURRENT              (u16)80 										  /* 短路电流 单位：A*/
#define SHORT_CURRENT_VOL              (SHORT_BUS_CURRENT * RSHUNT) /* 母线电流采样电压结果(最好在1.6V以内)*/
#define SHORT_CURRENT_DAC              (u16)((SHORT_BUS_CURRENT * RSHUNT * 256)/3)     /* 短路电压对应DAC值, DAC最大量程为3V */

/* ----------------------------Current Lim Parameter-------------------------- */
#define CURLIM_FUNCTION                0
#define POWLIM_FUNCTION                1
#define CUR_POW_SEL        						 CURLIM_FUNCTION                           /*限电流或限功率切换 0限电流 1限功率 */
#define MAX_BUS_CURRENT_SETTINT        (u16)18                                   /* 限电流 单位：A*/
#define CURRENT_ADC_PER_A              (RSHUNT * AMPLIFICATION_GAIN * 32752/3.6)  /* 每安电流ADC值 */ 
#define CURRENT_LIM_VALUE              (u16)(MAX_BUS_CURRENT_SETTINT * CURRENT_ADC_PER_A) /* 电流ADC值 */
#define IevgSum_Kp  Q15(0.05)
#define IevgSum_Ki  Q15(0.02)
#define IevgSum_Kc  Q15(0.5)						//0.5
#define POW_LIM_VALUE              		 (u32)(24*CURRENT_LIM_VALUE>>7) /* 功率ADC值  电压V*电流 */

#define BUS_CURRENT_FIRST              (u16)20 /* 一级限流保护 单位：A*/
#define BUS_CURRENT_SECOND             (u16)21 /* 二级限流保护 单位：A*/
#define CURRENT_ADC_PER_A              (RSHUNT * AMPLIFICATION_GAIN * 32752/3.6) /* 每安电流ADC值 */ 
#define OVER_CURRENT_FIRST_THD         (u16)(BUS_CURRENT_FIRST * CURRENT_ADC_PER_A)  /* 第一级限电流保护ADC值 */
#define OVER_CURRENT_SECOND_THD        (u16)(BUS_CURRENT_SECOND * CURRENT_ADC_PER_A) /* 第二级限电流保护ADC值 */
#define TIME_LIMIT_FIRST               1000  /* 一级限流保护时间 */
#define TIME_LIMIT_SECOND              200   /* 二级限流保护时间 */

/****************************MOS温度保护*************************************/
#define MOS_TEMP_UP_VOL              5     	/* MOS温度检测上拉电压，单位：V */   
#define MOS_TEMP_UP_RES              10    	/* MOS温度检测上拉电阻，单位：KΩ */  
#define MOS_TEMP_OVER_RES            1.0  	/* MOS过温时NTC阻值，95℃对应1.0,电压0.45V */ 
#define RSM_MOS_TEMP_OVER_RES        3.0  	/* MOS过温恢复NTC阻值，60℃对应3.0K，电压1.15V */ 
#define MOS_TEMP_OVER_THD            (u32)((MOS_TEMP_OVER_RES * MOS_TEMP_UP_VOL * 32752)/((MOS_TEMP_OVER_RES + MOS_TEMP_UP_RES) * 3.6))
#define RSM_MOS_TEMP_OVER_THD        (u32)((RSM_MOS_TEMP_OVER_RES * MOS_TEMP_UP_VOL * 32752)/((RSM_MOS_TEMP_OVER_RES + MOS_TEMP_UP_RES) * 3.6))
#define MOS_TEMP_OVER_TIME           500  	/* 单位：ms */  
#define RSM_MOS_TEMP_OVER_TIME       500  	/* 单位：ms */  

/* ---------------------------- Voltage Protect Parameter -------------------------- */
#define LOW_VOLATAGE_THD_1                5.5     	/* 第一段欠压, 电机工作中发生欠压保护 单位: V*/
#define LOW_VOLATAGE_THD_2                6.0   	  /* 第二段欠压, 电机待机状态中发生欠压保护 单位: V*/
#define RSM_LO_VOLATAGE_THD               10.0 		  /* 欠压恢复电压 单位: V*/
#define LOW_VOLATAGE_FIRST                (u16)(LOW_VOLATAGE_THD_1 * VOLTAGE_SHUNT_RATIO/3.6*32752)  
#define LOW_VOLATAGE_SECOND               (u16)(LOW_VOLATAGE_THD_2 * VOLTAGE_SHUNT_RATIO/3.6*32752)  
#define RSM_LO_VOLATAGE_ADC               (u16)(RSM_LO_VOLATAGE_THD * VOLTAGE_SHUNT_RATIO/3.6*32752)   
#define LV_PROTECT_TIME_SLOW              500    		/* 第一段欠压时间 	单位: ms*/
#define LV_PROTECT_TIME_FAST              50	    	/* 第二段欠压时间 	单位: ms*/
#define DIS_UV_PROTECT_TIME              	500    		/* 欠压恢复时间延时 单位: ms*/

#define OV_VOLTAGE_THD                    78   			/* 过压门槛 单位: V*/
#define RSM_OV_VOLTAGE_THD                70   			/* 过压恢复门槛 单位: V*/
#define OV_VOLTAGE_ADC                    (u16)(OV_VOLTAGE_THD * VOLTAGE_SHUNT_RATIO/3.6*32752)   
#define RSM_OV_VOLTAGE_ADC                (u16)(RSM_OV_VOLTAGE_THD * VOLTAGE_SHUNT_RATIO/3.6*32752) 
#define OV_PROTECT_TIME_FAST              10	    	/* 过压时间 单位: ms*/
#define DIS_OV_PROTECT_TIME_SLOW        	500    		/* 过压恢复时间延时 单位: ms*/

/* ---------------------------- 堵转保护 -------------------------- */
#define EN_MOTOR_BLOCK_DETECT        		(1)       	/* 堵转保护检测使能 */ 
#define MOTOR_BLOCK_DETECT_CNT     			(50)       	/* 堵转检测次数 */ 

/* ---------------------------- VSP Speed command Parameter 0~5V对应AD值0~1800 -------------------------- */
#define VSP_OFF_VALUE                		(200)       /* VSP关闭门槛(0.3V) 单位: AD值*/        
#define VSP_START_VALUE                	(300)       /* VSP启动门槛(0.5V) 单位: AD值*/          
#define VSP_MAX_VALUE                		(1800)      /* VSP最大值(3.2V) 单位: AD值*/      
#define VSP_DUTY_ACC_LOAD       				(1)         /* PWM DUTY每1ms增加的值,越大加速越快*/ 
#define VSP_DUTY_DEC_LOAD        				(1)         /* PWM DUTY每1ms减小的值,越大减速越快*/ 

/* ----------------------------direction check Parameter----------------------- */
#define EN_IOSET_CWCCW        	 			 (1)       	/* 读取CW-CCW IO设定CWCCW使能，如下配置则失效 */ 
#define CW                             (1)      	/* 电机转向：顺时针 */ 
#define CCW                            (0)      	/* 电机转向：逆时针*/	
#define CW_CCW                         CW      	  /* 电机转向设定，EN_IOSET_CWCCW=0时生效*/	

/* ----------------------------停机刹车功能----------------------- */
#define EN_BRAKE                  			0       	/* 电机停机刹车功能 */

/* ----------------------------顺风检测功能----------------------- */
#define EN_MOTOR_FREERUN_DETECT   			1       	/* 顺风检测功能开关 */
#define MOTOR_FREERUN_DETECT_CNT				4000			/* 最长检测时间 单位：TIMER1_TIMEBASE */

/* ----------------------------预驱自举电容预充电参数--------------------------- */
#define EN_PRE_CHARGE										1					/* 自举电容预充电功能 */
#define CHARGE_TIMECNT									16666			/* 共100ms 每相预充电时间，根据实际硬件参数修改  单位：TIMER1_TIMEBASE */

/* ---------------------------- Motor startup PWM DUTY -------------------------- */
#define MOTOR_STARTUP_PWMDUTY           ((u16)(0.1*PWM_PERIOD)) /* 当PWM斩波频率为16K时，150为启动占空比 */
#define STARTUP_DRAG_TIME         			100            			/* 启动时，启动duty强拖时间 单位：1ms */

/* ---------------------------- Motor startup parameter -------------------------- */
#define TIMER1_TIMEBASE									6										//建议不用修改，单位：us
#define TIMER1_TH_VALUE									(TIMER1_TIMEBASE*MCU_MCLK/1000000)
#define MAX_SPEED_CNT               		1000 								//Motor Minimum speed  单位： TIM1_TIMEBASE;  2500 
#define MIN_SPEED_CNT                		10									//Motor Maximum speed  单位： TIM1_TIMEBASE; 

/* ---------------------------- 相位补偿 -------------------------- */
#define EN_PHASE_COMP   				0       							/* 相位补偿功能开关 */
#define PHASE_COMP_LEAD_ANGLE   0.0										// 超前相位范围：0~2.5

/* ---------------------------- Motor Speed Close-loop--------------------------- */
#define EN_MOTOR_SPEED_CLOSELOOP				(0)											/* 转速闭环功能 */
#define MOTOR_POLES											2											  /* 电机极对数 10 */
#define MOTOR_SPEED_X										((60*1000000/MOTOR_POLES)/TIMER1_TIMEBASE)	/* 转速计算系数 */ 	// RPM = MOTOR_SPEED_X/T 
#define MOTOR_SPEED_MAX_RPM							50000										/* unit: RPM, 闭环最大目标转速 */
#define MOTOR_SPEED_MIN_RPM							200						   				/* unit: RPM, 闭环最小目标转速 */
#define SPEED_ACC_MS                   (float)(5.0)             //速度环爬坡加速度RPM/MS
#define SPEED_DEC_MS                   (float)(5.0)             //速度环爬坡减速度RPM/MS	
#define SPEED_PI_PRC                   (2)                 			//速度环预分频，此宏在时基更改时，需要手动调节
#define SSum_Kp  											  Q15(0.05)
#define SSum_Ki  												Q15(0.01)
#define SSum_Kc  												Q15(0.5)

/* ---------------------------- Motor IPD--------------------------- */
#define EN_MOTOR_ROTOR_DETECT						0
#define UART0_FUNCTION               	 DISABLE_FUNCTION         /* 电机控制UART0串口控制通信协议 */
#define UART0_BaudRate              	 9600     			 		      /* 电机控制UART0波特率 */


#endif  /* __PARAMETER_H */

/************************ (C) COPYRIGHT SNANER SEMICONDUCTOR **********************/
/* -----------------------------------END OF FILE------------------------------- */

