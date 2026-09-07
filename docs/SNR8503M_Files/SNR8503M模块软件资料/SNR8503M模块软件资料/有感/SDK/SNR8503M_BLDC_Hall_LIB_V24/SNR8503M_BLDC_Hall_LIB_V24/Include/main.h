
#ifndef __MAIN_H_
#define __MAIN_H_

#include "hardware_config.h"

void Hardware_init(void);
void sys_init(void);
void Task_Scheduler(void);

void Flash_write(void);
//void Clear_error(void);

extern void MosTest(void);
extern void Init_Parameter(void);
extern void variable_reset(void);
extern u16 procHander(u16 hallbarADvalue);
extern void Phase_COMPENSATION_Cal(void);
extern void VSP_Control_Motor(void);
extern void Vol_protect(void);
extern void Motor_Block_Protect(void);
extern void Current_Limit(void);
extern void OVER_current_protect(void);
extern void MOS_TEMP_protect(void);
extern void BAT_TEMP_protect(void);

#define TASK_SCHEDU_1MS                   (2)                                      /* 任务调度1ms计数时长 */
#define TASK_SCHEDU_10MS                  (20)                                     /* 任务调度10ms计数时长 */
#define TASK_SCHEDU_100MS                 (200)                                    /* 任务调度100ms计数时长 */
#define TASK_SCHEDU_500MS                 (1000)                                   /* 任务调度500ms计数时长 */

/*******************************************************************************
 函数名称：    void SetTimeOut_Counter(u16 hTimeout)
 功能描述：    设置等待函数等待时间
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Li Li          创建
 *******************************************************************************/
#define SetTimeOut_Counter(SetTimeLeftCnt, hTimeout)  {SetTimeLeftCnt = hTimeout;}


#endif  /* __MAIN_H_ */


