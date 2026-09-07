/*******************************************************************************
 * 版权所有 (C)2015, SNANER SEMICONDUCTOR Co.ltd
 *
 * 文件名称： hardware_config.h
 * 文件标识：
 * 内容摘要： 硬件相关文件
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Li
 * 完成日期： 2020年8月20日
 *
 * 修改记录1：
 * 修改日期： 2020年8月20日
 * 版 本 号： V 2.0
 * 修 改 人： Li
 * 修改内容： 创建
 *
 *******************************************************************************/

/*------------------------------prevent recursive inclusion -------------------*/
#ifndef __HARDWARE_CONFIG_H_
#define __HARDWARE_CONFIG_H_

#include "snr8503x.h"
#include "snr8503x_MCPWM.h"
#include "snr8503x_sys.h"
#include "snr8503x_adc.h"
#include "snr8503x_dac.h"
#include "snr8503x_MCPWM.h"
#include "snr8503x_Flash.h"
#include "snr8503x_gpio.h"
#include "snr8503x_dma.h"
#include "snr8503x_cmp.h"
#include "snr8503x_uart.h"
#include "snr8503x_hall.h"
#include "snr8503x_timer.h"
#include "snr8503x_NVR.h"
#include "snr8503x_iwdg.h"
#include "snr8503x_dsp.h"
#include "basic.h"

#include "delay.h"
#include "hardware_init.h"
#include "Global_Variable.h"
#include "MC_Parameter.h"

#define	setbit(x,y)				x |= (1<<y)
#define	clrbit(x,y)				x &= ~(1<<y)
#define	reversebit(x,y)		x ^= (1<<y)
#define	getbit(x,y)				((x) >> (y)&1)

/* ---------------------------- Hall Clock Division --------------------------- */
#define CLK_DIVISION 										(0) 				// Hall 时钟源分频, 数据范围: 0 ~ 3

#if(CLK_DIVISION == 0)
	#define HALL_CLK_K										48					// Hall 时钟频率: 48MHz
#elif(CLK_DIVISION == 1)
	#define HALL_CLK_K										24					// Hall 时钟频率: 24MHz
#elif(CLK_DIVISION == 2)
	#define HALL_CLK_K										12					// Hall 时钟频率: 12MHz
#elif(CLK_DIVISION == 3)
	#define HALL_CLK_K										6						// Hall 时钟频率: 6MHz
#endif

/* --------------------------------ADC通道号定义------------------------------ */
#define ADC_CHANNEL_OPA0              ADC_CHANNEL_0
#define ADC_CHANNEL_OPA1              ADC_CHANNEL_8

/* ADC相电流采样时序，硬件相关 ------------------------------------------------ */
/* Porting Application Notice 注意采样序列 ------------------------------------ */
#define ADC_BUS_CUR_CHANNEL            ADC_CHANNEL_OPA0    /* 母线电流 */
#define ADC_PEAK_CUR_CHANNEL           ADC_CHANNEL_OPA0

#define ADC_BUS_VOL_CHANNEL            ADC_CHANNEL_4       /* 母线电压 */
#define ADC_SPEED_CHANNEL              ADC_CHANNEL_5       /* 速度信号 */

#define ADC_MOS_TEMP_CHANNEL           ADC_CHANNEL_9       /* MOS温度 */

#define ADC_OPA_OUT_CHANNEL            ADC_CHANNEL_5      /* 运放输出采样 */

#define BEMF_CH_A                      ADC_CHANNEL_1        /* ADC_Ch1 */
#define BEMF_CH_B                      ADC_CHANNEL_2        /* ADC_Ch2 */
#define BEMF_CH_C                      ADC_CHANNEL_3        /* ADC_Ch3 */

#define ADC_STATE_RESET()              {ADC_CFG |= BIT11;}  /* ADC0 状态机复位,用以极限情况下确定ADC工作状态 */
#define ADC_SOFTWARE_TRIG_ONLY()       {ADC_CFG = 0;}       /* ADC设置为仅软件触发 */

// CMP0 Sensorless Detect
#define CMP0_BEMF_CH_U                 CMP0_SELP_CMP0_IP2   /* ADC_Ch1 */
#define CMP0_BEMF_CH_V                 CMP0_SELP_CMP0_IP1   /* ADC_Ch2 */
#define CMP0_BEMF_CH_W                 CMP0_SELP_CMP0_IP3   /* ADC_Ch3 */

/* -----------------------------驱动极性相关定义 ------------------------------- */
#define HN_HIGH__LN_HIGH                   0          /* 预驱预动极性设置 上管高电平有效，下管高电平有效 */
#define HN_HIGH__LN_LOW                    1          /* 预驱预动极性设置 上管高电平有效，下管低电平有效 */
#define	PRE_DRIVER_POLARITY								 HN_HIGH__LN_HIGH
#if ((PRE_DRIVER_POLARITY == HN_HIGH__LN_HIGH)) 
#define DRIVER_POLARITY                0x0000      		// 取 MCPWM_IO01 、 MCPWIO23 配置里面CHxN、CHxP的极性的配置
#else
#define DRIVER_POLARITY                0x0101      // 取 MCPWM_IO01 、 MCPWIO23 配置里面CHxN、CHxP的极性的配置
#endif

#define DEADTIME_NS                    ((u16)1500)                              /* 死区时间 */
#define DEADTIME                       (u16)(((unsigned long long)PWM_MCLK * (unsigned long long)DEADTIME_NS)/1000000000uL)

/* ------------------------------PGA操作相关定义 ------------------------------ */
#define PGA_GAIN_20                    0                   /* 反馈电阻200:10 */
#define PGA_GAIN_9P5                   1                   /* 反馈电阻190:20 */
#define PGA_GAIN_6                     2                   /* 反馈电阻180:30 */
#define PGA_GAIN_4P25                  3                   /* 反馈电阻170:40 */
                                                                                  
#define OPA_GIAN                      (PGA_GAIN_20)

/* ---------------------------CMP比较器设置相关定义 --------------------------- */
#define CMP1_SELN_SET                  CMP1_SELN_DAC            /* 比较器1负端选择DAC */
#define CMP1_SELP_SET                  CMP1_SELP_CMP1_IP0       /* 比较器1正端选择运放1输出结果 */

#define CMP0_SELN_SET                  CMP0_SELN_HALL_MID       /* 比较器0负端选择HALL MID */
//#define CMP0_SELP_SET                  CMP0_SELP_CMP0_IP1       /* 比较器0正端选择IP0 */
#define CMP_IP_A                       CMP0_SELP_CMP0_IP2        /* A相反电势电压比较器通道 */
#define CMP_IP_B                       CMP0_SELP_CMP0_IP1        /* B相反电势电压比较器通道 */
#define CMP_IP_C                       CMP0_SELP_CMP0_IP3        /* C相反电势电压比较器通道 */

#define CMP0_IE_ENABLE()                {CMP_IE |= BIT0;}   /* 比较器0关闭使能 */
#define CMP0_IE_DISABLE()               {CMP_IE &= ~BIT0;}  /* 比较器0关闭使能 */
#define CMP0_IE_STATE()                 (CMP_IE & BIT0)     /* 判断当前比较器0状态 */

//#define CMP0_OUT_LEVEL()                (CMP_DATA & BIT0)   /* 比较器0输出电平原始信号状态 */
#define CMP0_OUT_LEVEL()                (CMP_DATA & BIT8)   /* 比较器0输出电平经过滤波信号状态 */

#define CMP0_POL_STATE()                (CMP_CFG & BIT0)    /* 当前比较器0输出极性选择 */ 

/* ------------------------------编译器选项------------------------------------ */
#define RUN_IN_RAM_FUNC  __attribute__ ((used, section ("ram3functions")))

/* ------------------------------硬件IO控制接口-------------------------------- */
/** FG_IO interface */
#define GPIO_FG                     	 	(GPIO1)
#define GPIO_PIN_FG                	   	(GPIO_Pin_3)
#define FG_OFF                         	{GPIO_FG->PDO &= (~GPIO_PIN_FG);} // 0
#define FG_ON                          	{GPIO_FG->PDO |= (GPIO_PIN_FG);}  // 1
#define FG_ONOFF                       	{GPIO_FG->PDO ^= (GPIO_PIN_FG);} 

/** CWCCW_IO interface */
#define GPIO_CWCCW                     	(GPIO0)
#define GPIO_PIN_CWCCW                 	(GPIO_Pin_9)

#define GPIO_Debug                   		(GPIO0)
#define GPIO_PIN_Debug              		(GPIO_Pin_9)
#define Debug_OFF                     	{GPIO_Debug->PDO &= (~GPIO_PIN_Debug);} // 0
#define Debug_ON                       	{GPIO_Debug->PDO |= (GPIO_PIN_Debug);}  // 1
#define Debug_ONOFF                   	{GPIO_Debug->PDO ^= (GPIO_PIN_Debug);} 


/** LED_IO interface */
#define GPIO_LED                     	 (GPIO0)
#define GPIO_PIN_LED                	 (GPIO_Pin_0)
#define LED_ON                         {GPIO_LED->PDO &= (~GPIO_PIN_LED);} // 0
#define LED_OFF                        {GPIO_LED->PDO |= (GPIO_PIN_LED);}  // 1
#define LED_ONOFF                      {GPIO_LED->PDO ^= (GPIO_PIN_LED);} 


/* ----------------------------------------------------------------------------- */
/* ----------------------------------------------------------------------------- */
/* ----------------------------------------------------------------------------- */
// 电机电转速换算电周期 & 单步时间
#define MOTOR_e_RPM              	 			120000     			 		   					/* 电机电转速: 12WeRPM */
#define MOTOR_e_K												((60*1000000)>>2)*HALL_CLK_K		/* 因超出数据类型范围, 代入计算前先右移两位, 避免数据溢出 */
#define MOTOR_e_PERIOD									(MOTOR_e_K/MOTOR_e_RPM)					/* 电周期 */
#define MOTOR_e_SINGLE_STEP							(MOTOR_e_PERIOD/6)							/* 单步时间 */

#define MOTOR_1K_e_RPM									(1000)					/* 电转速/Min: 1K Hz */
#define MOTOR_2K_e_RPM									(2000)					/* 电转速: 2K */
#define MOTOR_3K_e_RPM									(3000)					/* 电转速: 3K */
#define MOTOR_4K_e_RPM									(4000)					/* 电转速: 4K */
#define MOTOR_5K_e_RPM									(5000)					/* 电转速: 5K */
#define MOTOR_6K_e_RPM									(6000)					/* 电转速: 6K */
#define MOTOR_7K_e_RPM									(7000)					/* 电转速: 7K */
#define MOTOR_8K_e_RPM									(8000)					/* 电转速: 8K */
#define MOTOR_9K_e_RPM									(9000)					/* 电转速: 9K */
#define MOTOR_1W_e_RPM									(10000)					/* 电转速: 1W */
#define MOTOR_2W_e_RPM									(20000)					/* 电转速: 2W */
#define MOTOR_3W_e_RPM									(30000)					/* 电转速: 3W */
#define MOTOR_4W_e_RPM									(40000)					/* 电转速: 4W */
#define MOTOR_5W_e_RPM									(50000)					/* 电转速: 5W */
#define MOTOR_6W_e_RPM									(60000)					/* 电转速: 6W */
#define MOTOR_7W_e_RPM									(70000)					/* 电转速: 7W */
#define MOTOR_8W_e_RPM									(80000)					/* 电转速: 8W */
#define MOTOR_9W_e_RPM									(90000)					/* 电转速: 9W */
#define MOTOR_10W_e_RPM									(100000)				/* 电转速: 10W */
#define MOTOR_11W_e_RPM									(110000)				/* 电转速: 11W */
#define MOTOR_12W_e_RPM									(120000)				/* 电转速: 12W */
/* ----------------------------------------------------------------------------- */
/* ----------------------------------------------------------------------------- */
/* ----------------------------------------------------------------------------- */

/* ---------------------------- Hall Phase Compensation  --------------------------- */
#define EN_HALL_PHASE_COMPENSATION  		(0)																// 是否开启霍尔相位补偿

#define EN_PHASE_COMPENSATION_SPEED  		(u32)(MOTOR_2W_e_RPM)							// 启动相位补偿的转速
#define DIS_PHASE_COMPENSATION_SPEED  	(u32)(MOTOR_1W_e_RPM)							// 解除相位补偿的转速

#define START_SPEED_COMPENSATION				EN_PHASE_COMPENSATION_SPEED				// 线性方程最小输入值
#define MAX_SPEED_COMPENSATION					MOTOR_3W_e_RPM										// 线性方程最大输入值

#define EN_PHASE_COMPENSATION						EN_PHASE_COMPENSATION_SPEED				// 开启相位补偿的条件
#define DIS_PHASE_COMPENSATION					DIS_PHASE_COMPENSATION_SPEED			// 解除相位补偿的条件

#define MIN_ADVANCE_ANGLE								15																// 数据范围: 15 ~ 45, 实际效果需观察相电压波形调整
#define MAX_ADVANCE_ANGLE								45																// 数据范围: 15 ~ 45, 实际效果需观察相电压波形调整

#define LIMIT_MIN_ADVANCE_ANGLE					15																// 限制最小补偿角, 固定值: 15
#define LIMIT_MAX_ADVANCE_ANGLE					45																// 限制最小补偿角, 固定值: 45

//#define LED_DELAY_TIME                 60000       /* 松扳机后延时关机时间 */ 

//#define LOCK_POWER_ON()                {GPIO0_PDO |= BIT1;}         
//#define LOCK_POWER_OFF()               {GPIO0_PDO &= ~BIT1;}

//#define LED_OFF_TIME                   500         /*LED报警闪烁时间设置*/
//#define LED_ON_TIME                    500
//#define LED_PERIOD_DELAY               1000        /*LED报警闪烁周期延时时间设置*/

//#define LED_ON()                       {GPIO0_PDO &= ~BIT3;}
//#define LED_OFF()                      {GPIO0_PDO |= BIT3;}

//#define KEYIN_FILTER_TIME              30 /* 按键输入滤波时间 */

//#define SWITCH_INPUT_MASK              (GPIO1_PDI & BIT13)
//#define DIRECT_INPUT_MASK              (GPIO1_PDI & BIT4)

//#define SWITCH_INPUT                   (BIT0)  /* 开关 */
//#define DIRECT_INPUT                   (BIT1)  /* 方向 */

#endif  /* __HARDWARE_CONFIG_H_ */

 
/************************ (C) COPYRIGHT SNANER SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */
 
 
