/*******************************************************************************
 * 版权所有 (C)2019, SNANER SEMICONDUCTOR Co.ltd
 *
 * 文件名称： UartCtrl.h
 * 文件标识：
 * 内容摘要： 电位器调速
 * 其它说明： 无
 * 当前版本： V1.0
 * 作    者： HuangMG
 * 完成日期： 2021年5月30日
 *
 *******************************************************************************/
#ifndef __URCTRL_H
#define __URCTRL_H

#include "basic.h"

//-----------------------------------------------------------------------------
#define UART0_BUF_SIZE 256
//-----------------------------------------------------------------------------
typedef struct
{
  u8 buffer[UART0_BUF_SIZE];
  u16 write_pt;
  u16 read_pt;
  u16 cnt;
}_UART0_STRUCT;


extern _UART0_STRUCT rxd_comm0;

extern volatile u8 UART_Flag;

extern void UART0_init(void);
extern void UartDealRX(void);
extern void UartDealTX(void);


#endif
