/*******************************************************************************
 * 文件名称： interrupt.c
 * 文件标识：
 * 内容摘要： 中断服务程序文件
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Li
 * 完成日期： 2021年11月15日
 *
 *******************************************************************************/
#include "hardware_config.h"
#include "MC_Parameter.h"
#include "MC_Drive.h"
#include "motor_structure.h"
#include "M1_StateMachine.h"
#include "MosTest.h"
#include "UR_Ctrl.h"

void Digital_Value_Getting(void);
/*******************************************************************************
 函数名称：    void Digital_Value_Getting(void)
 功能描述：    ADC值获取
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2022/5/26      V1.0           Li
 *******************************************************************************/
void Digital_Value_Getting(void)
{
		/* ADC值读取 */	
		User_sys.BLDC_SensorlessCtr.peakBusCurrent = (s16)(ADC_DAT0 - User_sys.BLDC_SensorlessCtr.PeakCurrOffset); //电流值读取  
			
		if(ADC_DAT1 & BIT15)		/* VBus */
		{
		  User_sys.BLDC_SensorlessCtr.nBUS_Vol_ADC = 0;
		}
    else
		{
      User_sys.BLDC_SensorlessCtr.nBUS_Vol_ADC = (User_sys.BLDC_SensorlessCtr.nBUS_Vol_ADC + ADC_DAT1)/2;		
		}		
		
		if(ADC_DAT2 & BIT15)		/* Speed command */
		{
			User_sys.BLDC_SensorlessCtr.nSpeed_ADC = 0;
		}
		else
		{
			User_sys.BLDC_SensorlessCtr.nSpeed_ADC = (ADC_DAT2 >> 4);	
		}
		
		if(ADC_DAT3 & BIT15)		/* MOS NTC */
		{
		  User_sys.BLDC_SensorlessCtr.nMOS_NTC_ADC = 0;
		}
		else
		{
		  User_sys.BLDC_SensorlessCtr.nMOS_NTC_ADC = (User_sys.BLDC_SensorlessCtr.nMOS_NTC_ADC + ADC_DAT3)/2;
		}		
}

/*******************************************************************************
 函数名称：    void ADC_IRQHandler(void)
 功能描述：    ADC0中断处理函数
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/15      V1.0        Li        创建
 *******************************************************************************/
void ADC_IRQHandler(void)
{
    ADC_IF |= BIT1 | BIT0;

    User_sys.BLDC_SensorlessCtr.nSys_TimerPWM++;        /* PWM时基计数器 */

		/* ADC值读取 */	
		Digital_Value_Getting();	
	
		if(MOS_Selftest.MosTest_start == 1)
		{
		  MOS_Selftest.PWM_cnt++;
			MOS_Selftest.PWM_delay++;
		}  
}


/*******************************************************************************
 函数名称：    void HALL_IRQHandler(void)
 功能描述：    HALL中断处理函数
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/15      V1.0        Li        创建
 *******************************************************************************/
void HALL_IRQHandler(void)
{
	static unsigned char i = 0;
	static unsigned char j = 0;

	static unsigned char hall_index = 0;
	
	if (HALL_GetIRQFlag(HALL_CAPTURE_EVENT))  /* HALL信号变化捕获事件 */
	{
    HALL_ClearIRQFlag(HALL_CAPTURE_EVENT);	/* 清除HALL捕获中断标志位 */
		
		hall_info = HALL_GetFilterValue();    					/* 取得HALL值，滤波结果 */
		hall_cnt = HALL_WIDCount();		      						/* 取得AHLL发生捕获时HALL计数器值 */
		Motor.BLDC.u8HallValueRenew = hall_info;	/* 存储hall信号 */
		
		if(i >= 5){	i = 0;	}
		else			{	i++;		}		
		Motor.Control.u32MotorElePhaseValue[i] = hall_cnt;
		
		if((Motor.BLDC.u8FlagMotorIsRunning == 1) && (Motor.Control.u8Read_Hall_finish_flag == 0))
		{			
			// 读实际霍尔变换时序
			if((Motor.BLDC.u8HallValueRenew == 5) || (Motor.BLDC.u8HallValueRenew == 0))	// 120度 或 60度
			{
				hall_index = 0;
				Read_hall_tab[hall_index] = Motor.BLDC.u8HallValueRenew;
				hall_index = 1;
			}
			else
			{
				Read_hall_tab[hall_index] = Motor.BLDC.u8HallValueRenew;
				if(hall_index < 5)
				{
					hall_index++;
				}
				else
				{
					hall_index = 0;
				}
			}
		}
		
		// 计算电气周期获取转速数据
		Motor.Control.u32MotorElePeriodValue = 0;
		for(j=0;j<6;j++)
		{
			Motor.Control.u32MotorElePeriodValue += Motor.Control.u32MotorElePhaseValue[j];				// 电气周期
		}
		Motor.Control.u32MotorElePeriodValueTemp = Motor.Control.u32MotorElePeriodValue;

		// 反转, 驱动信号取反
		if(Motor.BLDC.u8Direction == CCW)
		{
			Motor.BLDC.u8HallValueRenew = (0x07) & (~Motor.BLDC.u8HallValueRenew);
		}
		
		if(Motor.BLDC.u8FlagMotorIsRunning == 1)//
		{
			// 判断hall信号是否正确, 并进行换相处理
			BLDC_Sensor_Judge(Motor.BLDC.u8HallValueRenew);
		
			if(Motor.Control.u8Read_Hall_finish_flag == 0)
			{
				if(Motor.Control.u16Phase_Compensation_cnt > 30)
				{
					#if (Hall_Degree_60_or_120 == Degree60)
						if((Read_hall_tab[1] == 7) && (Read_hall_tab[5] == 1))	// CW
						{
							Motor.Control.u8Read_Hall_CWCCW_flag = CW;
							Motor.Control.u8Read_Hall_finish_flag = 1;
						}
						else if((Read_hall_tab[1] == 1) && (Read_hall_tab[5] == 7))	// CCW
						{
							Motor.Control.u8Read_Hall_CWCCW_flag = CCW;
							Motor.Control.u8Read_Hall_finish_flag = 1;
						}
						else		// 霍尔错误, 不能进行超前换相
						{
							Motor.Control.u16Phase_Compensation_cnt = 0;
							Motor.Control.u8Read_Hall_finish_flag = 0;
						}					
					#else
						if((Read_hall_tab[1] == 4) && (Read_hall_tab[5] == 1))	// CW
						{
							Motor.Control.u8Read_Hall_CWCCW_flag = CW;
							Motor.Control.u8Read_Hall_finish_flag = 1;
						}
						else if((Read_hall_tab[1] == 1) && (Read_hall_tab[5] == 4))	// CCW
						{
							Motor.Control.u8Read_Hall_CWCCW_flag = CCW;
							Motor.Control.u8Read_Hall_finish_flag = 1;
						}
						else		// 霍尔错误, 不能进行超前换相
						{
							Motor.Control.u16Phase_Compensation_cnt = 0;
							Motor.Control.u8Read_Hall_finish_flag = 0;
						}
					#endif
				}
			}
		}
	
		if(Motor.Control.u16Phase_Compensation_cnt > 50)	// 等待转速数据稳定
		{
			// 设置超前换相处理 
			#if EN_HALL_PHASE_COMPENSATION
				if(Motor.Control.u8Cal_Hall_Angle_OK_flag == 1)
				{				
					// 使用硬件除法器计算
//					HallAngle_Division.Dividend = Motor.Control.u32Cal_Hall_Angle_Act_Cmd * Motor.Control.u32MotorElePeriodValueTemp;		// 被除数
//					HallAngle_Division.Divisor = 360;		// 除数, 一个电气周期360度
//					Motor.Control.i32_AdvanceAngleValue = DSP_CalcDivision(&HallAngle_Division);
					Motor.Control.i32_AdvanceAngleValue = Motor.Control.u32Cal_Hall_Angle_Act_Cmd * Motor.Control.u32MotorElePeriodValueTemp / 360;
					
					TIM_Timer_Set_TH_Value(TIMER1, Motor.Control.i32_AdvanceAngleValue);
					TIM_Timer_ClearCnt(TIMER1);					// 清除TIM1计数值
					TIM_TimerCmd(TIMER1, ENABLE); 			// 计算时间进行换相
				}
			#endif
			/*--------------- 以上为超前换相配置 ----------------*/
		}
	
		Motor.BLDC.u32Freerun_Det_cnt = 0;				/* 顺风检测延时计数 */
		if((Motor.BLDC.u8FlagEnMotorRun == 1) && (e1M1_RunSubState == M1_RunState_Freerun_Det))
    {
			Motor.BLDC.u8FlagMotorIsRunning = 1;
			
			e1M1_RunSubState = M1_RunState_Spin;
		
			// 获取Hall信号
			hall_info = HALL_GetFilterValue();    /* 取得HALL值，滤波结果*/
			Motor.BLDC.u8HallValueRenew = hall_info;
			// 根据正反转, 获取下一步hall信号
			if(Motor.BLDC.u8Direction == CCW)
			{
				Motor.BLDC.u8HallValueRenew = (0x07) & (~Motor.BLDC.u8HallValueRenew);
			}
			// 更新hall信号
			Motor.BLDC.u8HallValueOld = Motor.BLDC.u8HallValueRenew;
			
			BLDC_Sensor_control(Motor.BLDC.u8HallValueRenew);
			MCPWM_UPDATE = 0;
			MCPWM_TH00 = 	- MIN_PWM_DUTY;
			MCPWM_TH01 = 		MIN_PWM_DUTY;
			MCPWM_TH10 = 	-	MIN_PWM_DUTY;
			MCPWM_TH11 = 		MIN_PWM_DUTY;
			MCPWM_TH20 = 	-	MIN_PWM_DUTY;
			MCPWM_TH21 = 		MIN_PWM_DUTY;
			MCPWM_UPDATE = 0xff;
			PWMOutputs(MCPWM0,ENABLE);
			
			Motor.Control.u32MCPWM_TH_temp = MIN_PWM_DUTY;
		}
	}
	
	// 堵转检测
	if(HALL_GetIRQFlag(HALL_OVERFLOW_EVENT)) 		/* HALL计数器溢出事件*/
	{
    HALL_ClearIRQFlag(HALL_OVERFLOW_EVENT);		/* 清除HALL溢出中断标志位*/
		
		if(Motor.BLDC.u8FlagMotorIsRunning == 1)
		{
			Motor.Control.u16MotorShakeDelayProtectCnt++;
		}
	}
}

/*******************************************************************************
 函数名称：    void TIMER1_IRQHandler(void)
 功能描述：    TIMER1中断处理函数
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 *******************************************************************************/
void TIMER1_IRQHandler(void)
{
	if(UTIMER1_IF & TIM_IRQ_IF_ZC)   /* UTIMER1过零中断 */
	{
		UTIMER1_IF = TIM_IRQ_IF_ZC;    /* 清除中断标志位 */
	}

	#if EN_HALL_PHASE_COMPENSATION
			if(Motor.Control.u8Read_Hall_CWCCW_flag == CW)
			{
				Motor.BLDC.u8HallValueNext = Motor_step_tab_cw[Motor.BLDC.u8HallValueOld];
			}
			else
			{			
				Motor.BLDC.u8HallValueNext = Motor_step_tab_ccw[Motor.BLDC.u8HallValueOld];
			}
	
		// 超前换相
		BLDC_Sensor_control(Motor.BLDC.u8HallValueNext);
		Motor.Control.u8PhaseChange_ON_COMPENSATION_flag = 1;

		TIM_TimerCmd(TIMER1, DISABLE); 
	#endif
}

/*******************************************************************************
 函数名称：    void TIMER0_IRQHandler(void)
 功能描述：    TIMER0中断处理函数
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 *******************************************************************************/
void TIMER0_IRQHandler(void)
{
	 if(UTIMER0_IF & TIM_IRQ_IF_ZC)   /* UTIMER0过零中断*/
	 {
			UTIMER0_IF = TIM_IRQ_IF_ZC;    /* 清除中断标志位*/
				 
			/* 时基500us */
			User_sys.struTaskScheduler.bTimeCnt1ms++;
			User_sys.struTaskScheduler.nTimeCnt10ms ++;
			User_sys.struTaskScheduler.nTimeCnt100ms ++;
			User_sys.struTaskScheduler.nTimeCnt500ms++;
			  
			ICP_Loader();
	 }
}

/*******************************************************************************
 函数名称：    void CMP_IRQHandler(void)
 功能描述：    比较器中断处理函数
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/15      V1.0        Li        创建
 *******************************************************************************/
void CMP_IRQHandler(void)
{
	volatile u8 t_i;
	u8 CMP_fail_cnt = 0;
  //CMP_IF = BIT0 | BIT1;
	if(CMP_IF & BIT1)
	{
		CMP_IF = BIT1;
		CMP_fail_cnt = 0;
		for(t_i = 0; t_i < 4; t_i++)
		{
			if(CMP_DATA & BIT1)
			{
				CMP_fail_cnt++;
			}
		}
		if(CMP_fail_cnt > 2)
		{
			Motor.BLDC.u8FlagMotorIsRunning = 0;			// Motor stop
			e1M1_MainState = M1_MainState_Fault;						// MainState Fault
			e1M1_RunSubState = M1_RunState_Calib;						// RunState Calib
			User_sys.BLDC_SensorlessCtr.sys_error_flg |= SHORT_ERROR;				
			#if EN_BRAKE
					BLDC_Brake();
			#else
					BLDC_Stop();			
			#endif
		}
	}	
}

/*******************************************************************************
 函数名称：    void SysTick_Handler(void)
 功能描述：    系统滴答定时中断
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/15      V1.0        Li        创建
 *******************************************************************************/
void SysTick_Handler(void)
{
	
}

/*******************************************************************************
 函数名称：    void MCPWM0_IRQHandler(void)
 功能描述：    MCPWM0中断处理函数
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/15      V1.0        Li        创建
 *******************************************************************************/
void MCPWM0_IRQHandler(void)
{
  if(MCPWM_EIF & FAIL1_IF)
  {	 
		Motor.BLDC.u8FlagMotorIsRunning = 0;			// Motor stop
		e1M1_MainState = M1_MainState_Fault;						// MainState Fault
		e1M1_RunSubState = M1_RunState_Calib;						// RunState Calib
		User_sys.BLDC_SensorlessCtr.sys_error_flg |= SHORT_ERROR;				
		#if EN_BRAKE
				BLDC_Brake();
		#else
				BLDC_Stop();			
		#endif
     MCPWM_EIF = FAIL1_IF;
  }  
}


/*******************************************************************************
 函数名称：    void DMA_IRQHandler(void)
 功能描述：    DMA中断处理函数
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/15      V1.0        Li        创建
 *******************************************************************************/
void DMA_IRQHandler(void)
{
  DMA_IF = 0x0f;
}



/*******************************************************************************
 函数名称：    void MCPWM1_IRQHandler(void)
 功能描述：    MCPWM1中断处理函数
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/15      V1.0        Li        创建
 *******************************************************************************/
void MCPWM1_IRQHandler(void)
{
	
}


/*******************************************************************************
 函数名称：    void SW_IRQHandler(void)
 功能描述：    软件中断处理函数
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/15      V1.0        Li        创建
 *******************************************************************************/
void SW_IRQHandler(void)
{

}

/*******************************************************************************
 函数名称：    void UART_IRQHandler(void)
 功能描述：    UART中断处理函数
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/15      V1.0        Li        创建
 *******************************************************************************/
void UART_IRQHandler(void)
{
	#if (UART0_FUNCTION == ENABLE_FUNCTION)
 	if (UART_IF & UART_IF_SendOver) //发送完成中断
	{
		UART_IF = UART_IF_SendOver;
		UART_Flag = 0;
	}
	if (UART_IF & UART_IF_RcvOver) //接收完成中断
	{
		UART_IF = UART_IF_RcvOver;
  // Rx, move data from UART FIFO to buffer
    rxd_comm0.buffer[rxd_comm0.write_pt] = UART0->BUFF;
    rxd_comm0.write_pt = (rxd_comm0.write_pt + 1) % UART0_BUF_SIZE;
    rxd_comm0.cnt++;	
	} 
	#endif	
}

/*******************************************************************************
 函数名称：    void WAKE_IRQHandler(void)
 功能描述：    休眠唤醒中断处理函数
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/15      V1.0        Li        创建
 *******************************************************************************/
void WAKE_IRQHandler(void)
{
    while(1);
}

/*******************************************************************************
 函数名称：    void GPIO_IRQHandler(void)
 功能描述：    GPIO中断处理函数
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/15      V1.0        Li        创建
 *******************************************************************************/
void GPIO_IRQHandler(void)
{
	if(EXTI_IF & GPIO_P05_EXTI_IF) /* 判断是否为P0.5外部中断*/
	{
    EXTI_IF = GPIO_P05_EXTI_IF;  /* 清除P0.5外部中断标志位*/		
	}
}

/*******************************************************************************
 函数名称：    void I2C_IRQHandler(void)
 功能描述：    I2C中断处理函数
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/15      V1.0        Li        创建
 *******************************************************************************/
void I2C_IRQHandler(void)
{

}

/*******************************************************************************
 函数名称：    void SPI_IRQHandler(void)
 功能描述：    SPI中断处理函数
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/15      V1.0        Li        创建
 *******************************************************************************/
void SPI_IRQHandler(void)
{

}

/************************ (C) COPYRIGHT SNANER SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */


