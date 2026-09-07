/*******************************************************************************
 * 版权所有 (C)2021, SNANER SEMICONDUCTOR Co.ltd
 *
 * 文件名称： SNR85_03x_flash.h
 * 文件标识：
 * 内容摘要： flash外设驱动程序
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者：   Li
 * 完成日期： 2021年10月14日
 *
 *
 *******************************************************************************/
#ifndef __snr8503x_FLASH__
#define __snr8503x_FLASH__
#include "snr8503x.h"

extern volatile u32 erase_flag;
extern volatile u32 progm_flag;
void EraseSector(u32 adr);
void ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf);
void Read_More_Flash(u32 adr, u32 sz, u32 *buf);

#endif

/************************ (C) COPYRIGHT SNANER SEMICONDUCTOR *****END OF FILE****/
