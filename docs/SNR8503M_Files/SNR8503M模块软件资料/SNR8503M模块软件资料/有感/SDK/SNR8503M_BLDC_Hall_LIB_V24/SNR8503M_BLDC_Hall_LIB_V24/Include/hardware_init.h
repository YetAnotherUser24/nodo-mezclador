/*******************************************************************************
 * 版权所有 (C)2015, SNANER SEMICONDUCTOR Co.ltd
 *
 * 文件名称： hardware_init.h
 * 文件标识：
 * 内容摘要： 硬件初始化头文件定义
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Li
 * 完成日期： 2022/5/30

 *******************************************************************************/
#ifndef __HARDWARE_INIT_H
#define __HARDWARE_INIT_H

#include "basic.h"

void PGA_init(void);
void DAC_init(void);
void CMP_init(void);
void MCPWM_init(void);
void UTimer_init(uint32_t Clk_Division);
void GPIO_init(void);
void HALL_Perip_Init(uint32_t Clk_Division);
void UART1_init(void);
void ADC_init(void);
void get_ic_temp_init(void);
void Clock_Init(void);
void Hardware_init(void);
void HALL_init(void); 
void SoftDelay(u32 cnt);
void BLDC_init(void);
void ICP_Loader(void);

#endif

/************************ (C) COPYRIGHT SNANER SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */

