/*******************************************************************************
 * 版权所有 (C)2015, SNANER SEMICONDUCTOR Co.ltd
 *
 * 文件名称： snr8503x_dsp.h
 * 文件标识：
 * 内容摘要： DSP驱动头文件
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： 
 * 完成日期： 
 *
 * 修改记录1：
 * 修改日期：
 * 版 本 号：V 1.0
 * 修 改 人：
 * 修改内容：创建
 *
 * 修改记录2：
 * 修改日期：
 * 版 本 号：
 * 修 改 人：
 * 修改内容：
 *
 *******************************************************************************/
 
#ifndef __snr8503x_DSP_H
#define __snr8503x_DSP_H

/* Includes ------------------------------------------------------------------*/
#include "snr8503x.h"
#include "snr8503x_sys.h"
#include "basic.h"

typedef struct
{
	  s32 Dividend;     /* 被除数 */
	  s32 Divisor;      /* 除数   */
    s32 Quotient;     /* 商     */
    s32 Remainder;    /* 余数   */
} stru_DiviComponents;/* 除法运算结构体 */

void DSP_Cmd(FuncState state);                          		/* DSP使能和关闭 */
int32_t DSP_CalcDivision(stru_DiviComponents *stru_Divi);  	/* DSP除法运算 */
uint32_t DSP_GetSqrt(u32 Data); /*DSP取得开方根 sprt(Data)*/

#endif 

/************************ (C) COPYRIGHT SNANER SEMICONDUCTOR *****END OF FILE****/
