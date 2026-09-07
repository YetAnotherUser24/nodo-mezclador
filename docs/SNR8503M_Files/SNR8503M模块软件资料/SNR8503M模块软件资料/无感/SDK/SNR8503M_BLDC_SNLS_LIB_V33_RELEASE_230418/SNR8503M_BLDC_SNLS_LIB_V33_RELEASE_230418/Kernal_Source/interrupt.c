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
#include "COMP_Sensorless.h"
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

		if(ADC_DAT5 & BIT15)		/* Speed command */
		{
			User_sys.BLDC_SensorlessCtr.nSpeed_ADC = 0;
		}
		else
		{
			User_sys.BLDC_SensorlessCtr.nSpeed_ADC = (ADC_DAT5 >> 4);	
		}
			
		if(ADC_DAT4 & BIT15)		/* VBus */
		{
		  User_sys.BLDC_SensorlessCtr.nBUS_Vol_ADC = 0;
		}
    else
		{
      User_sys.BLDC_SensorlessCtr.nBUS_Vol_ADC = (User_sys.BLDC_SensorlessCtr.nBUS_Vol_ADC + ADC_DAT4)/2;		
		}
		
		if(ADC_DAT6 & BIT15)		/* MOS NTC */
		{
		  User_sys.BLDC_SensorlessCtr.nMOS_NTC_ADC = 0;
		}
		else
		{
		  User_sys.BLDC_SensorlessCtr.nMOS_NTC_ADC = (User_sys.BLDC_SensorlessCtr.nMOS_NTC_ADC + ADC_DAT6)/2;
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

//		/* ADC值读取 */	
		Digital_Value_Getting();	
	
		if(MOS_Selftest.MosTest_start == 1)
		{
		  MOS_Selftest.PWM_cnt++;
			MOS_Selftest.PWM_delay++;
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
	if(UTIMER1_IF & TIM_IRQ_IF_ZC)   /* UTIMER1过零中断*/
	{
		UTIMER1_IF = TIM_IRQ_IF_ZC;    /* 清除中断标志位*/

		////	/** COMP5 Sensorless mode */
		if((e1M1_RunSubState == M1_RunState_Freerun_Det) || (e1M1_RunSubState == M1_RunState_Spin))
		{
			{
				switch(Motor_1M1st.BLDC_CMP.u8ControlStep)
				{
					case MOTOR_READY:		
							if(Motor_1M1st.BLDC_CMP.u8FlagDetectMotorFreerunning == 1)		// Motor free-running detect
							{
								Motor_Freerun_Detect();
							}
							else
							{
								if(Motor_1M1st.BLDC.u8FlagEnMotorRun == 1)
								{
									Motor_1M1st.BLDC_CMP.u8ControlStep = MOTOR_ROTOR_DETECT;								
									Motor_1M1st.BLDC_CMP.u8ramp_cnt = 0;					// Drag Motor's step cnt
									Motor_1M1st.BLDC_CMP.u32closeloop_count = 0;	// Speed closp-loop cnt
									
									Motor_1M1st.BLDC_CMP.u8FlagDetectMotorFreerunning = 0;
								}
							}
						break;
						
					case MOTOR_ROTOR_DETECT:
							#if(EN_MOTOR_ROTOR_DETECT)
								Motor_Rotor_Detect();
							#else
								Motor_1M1st.BLDC_CMP.u8ControlStep = MOTOR_ROTOR_ALIGN;
								Motor_1M1st.BLDC_CMP.u32ChargeTime_count = 0;
							#endif	
						break;
					
					case MOTOR_ROTOR_ALIGN:
							if(Motor_1M1st.BLDC_CMP.u32ChargeTime_count < CHARGE_TIMECNT)
							{
								#if	EN_PRE_CHARGE
									Motor_1M1st.BLDC_CMP.u32ChargeTime_count++;
									BLDC_Brake();
								#else
									Motor_1M1st.BLDC_CMP.u32ChargeTime_count = CHARGE_TIMECNT;
								#endif
							}
							else
							{
								CMP_SNS_Motor_Align();
							}
						break;

					case MOTOR_RUN:
							CMP_SNS_Motor_AutoCommutation();
						break;	

					default:
	//						Motor_1M1st.BLDC_CMP.u8ControlStep = 0;
	//						Motor_1M1st.BLDC.u32FlagMotorIsRunning = 0;
						break;				
				}
			}		
		}
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
			Motor_1M1st.BLDC.u8FlagMotorIsRunning = 0;			// Motor stop
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
		Motor_1M1st.BLDC.u8FlagMotorIsRunning = 0;			// Motor stop
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
	HALL_INFO = HALL_CAPTURE_EVENT;//清除标志位
	HALL_INFO = HALL_OVERFLOW_EVENT;//清除标志位
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


