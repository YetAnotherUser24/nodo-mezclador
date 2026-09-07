#ifndef __MOSTEST_H
#define __MOSTEST_H

#include "basic.h"
void MosTest(void);

extern void GPIO_init(void);
extern void MCPWM_init(void);

/* ==============================MOS管自检测-定义=========================== */

/* MCPWM_N通过FD2203驱动上桥     MCPWM_P通过FD2203驱动下桥 */

#define MOSTEST_VALUE1   (u16)(-1000)
#define MOSTEST_VALUE2   200

#define MCPWM_P          0
#define MCPWM_N          (u16)(-350)

#define ADC_DELAY1 MOSTEST_VALUE2
#define ADC_DELAY2 MOSTEST_VALUE2
#define ADC_DELAY3 MCPWM_P

typedef struct
{
	volatile u8  check_step;
  volatile s32 Bus_current[5];
	volatile u16 PWM_Setting;
	volatile u16 Sample_Point;
	volatile u8  PWM_cnt;
  volatile u8  PWM_delay;
	volatile u8  MosTest_start;
	volatile u8  MOSfail_flg;
}stru_MOS_test;

extern stru_MOS_test MOS_Selftest;           /* BLDC过程控制结构体 */
				
void MOS_Check_Step(unsigned char MOS_Check_step_cnt);														
										
#endif



