/*******************************************************************************
 * 版权所有 (C)2015, SNANER SEMICONDUCTOR Co.ltd
 *
 * 文件名称： MC_type.c
 * 文件标识：
 * 内容摘要： 电机控制相关结构体声明
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Li Li
 * 完成日期： 2020年8月16日
 *
 * 修改记录1：
 * 修改日期：2020年8月16日
 * 版 本 号：V 1.0
 * 修 改 人：Li Li
 * 修改内容：创建
 *
 *******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_TYPE_H
#define __MC_TYPE_H

/* Includes ------------------------------------------------------------------*/
#include "basic.h"
#include "MC_type.h"

/* Exported types ------------------------------------------------------------*/
/*! Macro converting a signed fractional [-1,1) number into a fixed point 16-bit
 * number in the format Q1.15. */
#define FRAC16(x)           ((s16) ((x) < (S16_MAX) ? ((x) >= S16_MIN ? (x)*0x8000 : 0x8000) : 0x7fff))

/*! Macro converting a signed fractional [-1,1) number into a fixed point 32-bit
 * number in the format Q1.31. */
#define FRAC32(x)           ((s32) ((x) < (S32_MAX) ? ((x) >= S32_MIN ? (x)*0x80000000 : 0x80000000) : 0x7fffffff))
#define ABS(X)              ( ( (X) >= 0 ) ? (X) : -(X) )
#define sat(x,ll,ul)        ( (x) > (ul) ) ? (ul) : ( ( (x) < (ll) ) ? (ll) : (x) )


typedef struct
{
    u8  bTimeCnt1ms;         /* 1mS计数器 */
    u16 nTimeCnt10ms;        /* 10mS计数器 */
    u16 nTimeCnt100ms;       /* 100mS计数器 */  
    u16 nTimeCnt500ms;       /* 500mS计数器 */

	  u16 nMultiplex_cnt100ms; /* 100ms延时SWD口复用*/
    const char* sVersion;    /* 程序版本号 */
} stru_TaskSchedulerDef;


/* Exported constants --------------------------------------------------------*/
#define U8_MAX     ((u8)255)
#define S8_MAX     ((s8)127)
#define S8_MIN     ((s8)-128)
#define U16_MAX    ((u16)65535u)
#define S16_MAX    ((s16)32767)
#define S16_MIN    ((s16)-32768)
#define U32_MAX    ((u32)4294967295uL)
#define S32_MAX    ((s32)2147483647)
#define S32_MIN    ((s32)-2147483648)

#define S16P80     ((s16)26213)
#define S16P65     ((s16)25000)
#define S16P50     ((s16)13000)
#define S16P95     ((s16)30000)

/**
* Not initialized pointer
*/
#define MC_NULL    (uint16_t)(0x0000u)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __MC_TYPE_H */
/************************ (C) COPYRIGHT SNANER SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */

