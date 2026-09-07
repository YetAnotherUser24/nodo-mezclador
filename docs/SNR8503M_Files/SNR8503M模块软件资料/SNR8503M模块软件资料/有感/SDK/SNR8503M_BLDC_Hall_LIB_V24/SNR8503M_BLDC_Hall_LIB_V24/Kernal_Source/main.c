/*******************************************************************************
 * 版权所有 (C)2015, SNANER SEMICONDUCTOR Co.ltd
 *
 * 文件名称： Main.c
 * 文件标识：
 * 内容摘要： 工程主代码
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Li
 * 完成日期： 2020年8月5日
 *
 * 修改记录1：
 * 修改日期： 2020年8月16日
 * 版 本 号： V 1.0
 * 修 改 人： Li
 * 修改内容： 创建
 *
 *******************************************************************************/
#include "main.h"
#include "PID.h"
#include "MC_Drive.h"
#include "SpeedRamp.h"
#include "motor_structure.h"
#include "M1_StateMachine.h"
#include "UR_Ctrl.h"


/*******************************************************************************
 函数名称：    int main(void)
 功能描述：    主程序入口
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/8/5      V1.0             Li          创建
 *******************************************************************************/

int main(void)
{
  SoftDelay(20000);      /* 延时等待硬件初始化稳定 20000--2.68ms */  //  
  SoftDelay(20000);      /* 延时等待硬件初始化稳定 20000--2.68ms */  //  	
	
	Hardware_init();			 /* 硬件初始化 */
	
	Init_Parameter();
	
	InitPI();
	sys_init();            /* 系统初始化 */

	MosTest();
	LED_ON
	FG_ON
	
	////电位器门限低值，门限高值，输出最大值，输入最大值
	InitNormalization(VSP_START_VALUE,VSP_MAX_VALUE,MIN_PWM_DUTY,MAX_PWM_DUTY,&RP);		// 速度开环
	InitNormalization(VSP_START_VALUE,VSP_MAX_VALUE,MOTOR_SPEED_MIN_RPM,MOTOR_SPEED_MAX_RPM,&SPEED_Cmd);		// 速度闭环

	#if EN_HALL_PHASE_COMPENSATION		
		InitNormalization(START_SPEED_COMPENSATION,MAX_SPEED_COMPENSATION,MIN_ADVANCE_ANGLE,MAX_ADVANCE_ANGLE,&Speed_PHASE_COMPENSATION_Cmd);		// 相位补偿
	#endif
	
	// read CWCCW
	CWCCW_PowerOn();	
	
	// 启动前获取Hall信号
	hall_info = HALL_GetFilterValue();    	/* 取得HALL值，滤波结果*/
	Motor.BLDC.u8HallValueRenew = hall_info;
	// 根据正反转, 获取下一步hall信号
	if(Motor.BLDC.u8Direction == CCW)
	{
		Motor.BLDC.u8HallValueRenew = (0x07) & (~Motor.BLDC.u8HallValueRenew);
	}
	Motor.BLDC.u8HallValueOld = Motor.BLDC.u8HallValueRenew;
	
	for (;;)
	{
			Task_Scheduler();
	}
}


/*******************************************************************************
 函数名称：    void Task_Scheduler(void)
 功能描述：    按时间片任务调度函数
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2022/5/30      V1.0                     创建
 *******************************************************************************/
void Task_Scheduler(void)
{
    if(User_sys.struTaskScheduler.bTimeCnt1ms >= TASK_SCHEDU_1MS)
    {   
			/* 1毫秒事件，任务调度 */
			User_sys.struTaskScheduler.bTimeCnt1ms = 0;
			
			// Speed command Input
			Motor.USER.u16VSP_filt_value = procHander((u16) User_sys.BLDC_SensorlessCtr.nSpeed_ADC);		/* Speed command */
			VSP_Control_Motor();		/* Speed command to RPM */ 		
			
			// 霍尔相位补偿
			Phase_COMPENSATION_Cal();
			
			// 电压保护检测
			Vol_protect();											
			
			// 堵转检测
			Motor_Block_Protect();
			
			// 限最大电流
			Current_Limit();
							
			/* 分级限流保护 */
			OVER_current_protect(); 
			
			// MOS 温度保护		
			MOS_TEMP_protect();

			/*  Statemachine */
			sM1_STATE[e1M1_MainState]();
    }  

    if( User_sys.struTaskScheduler.nTimeCnt10ms >= TASK_SCHEDU_10MS )
    {   /* 10毫秒事件，任务调度 */
        User_sys.struTaskScheduler.nTimeCnt10ms = 0;
			
				/* 电机LED指示 */
				Motor_LED_DISP();
			
				/* 电机转向设定 */
				CWCCW_ReadIO();			
		}
		
    if( User_sys.struTaskScheduler.nTimeCnt100ms >= TASK_SCHEDU_100MS )
		{
			User_sys.struTaskScheduler.nTimeCnt100ms = 0;
			//Usart_SendHalfWord(User_sys.BLDC_SensorlessCtr.nMOS_NTC_ADC);
			//UART0_SendByte(User_sys.BLDC_SensorlessCtr.nMOS_NTC_ADC);
		}			
		#if (UART0_FUNCTION == ENABLE_FUNCTION)
		UartDealRX();
		UartDealTX();
		#endif
}

/************************ (C) COPYRIGHT SNANER SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */



