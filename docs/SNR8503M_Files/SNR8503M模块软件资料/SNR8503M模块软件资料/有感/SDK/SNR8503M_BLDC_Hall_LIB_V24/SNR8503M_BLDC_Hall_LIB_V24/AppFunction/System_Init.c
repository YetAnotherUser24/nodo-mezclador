/*******************************************************************************
 * 文件名称： System_init.c
 * 文件标识：
 * 内容摘要： 定时相关函数
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： 
 * 完成日期： 
 *******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "hardware_config.h"
#include "MC_parameter.h"
#include "M1_StateMachine.h"
#include "motor_structure.h"

void Multiplex_SWD(void);

void CurrentOffsetCalibration(void);
void DC_offset_protect(void);

volatile u16 ADC_VOL_test;
volatile u16 OPA_OFFSET_VOL_test;
volatile u16 test_OPA_DAC;
volatile u16 test_short_DAC,test_sys_afe_dac;

/*******************************************************************************
 函数名称：    sys_init(void)
 功能描述：    系统变量初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/8/5      V1.0           Li Li          创建
 *******************************************************************************/
void sys_init(void)
{	
//	  u8 i;
    while(User_sys.BLDC_SensorlessCtr.nSys_TimerPWM < 200){;} /* 延时150ms */

    User_sys.struTaskScheduler.sVersion = &sVersion[0];/* 初始化版本号 */
  
		CurrentOffsetCalibration();			// Get Current Offset value, BEMF_A、B、C Offset value
		DC_offset_protect();						// Judge DC offset state
			
#if DEBUG_PWM_OUTPUT 
    DebugPWM_OutputFunction(); /* 调试的时候输出25%的PWM波形 */
#endif
}

/*******************************************************************************
 函数名称：    void Multiplex_SWD(void)
 功能描述：    SWD复用
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/9/20      V1.0           Li Li          创建
 *******************************************************************************/
void Multiplex_SWD(void)
{		
		SYS_PROTECT = 0x7a83;
		SYS_IO_CFG &= ~BIT6;         /* SWD口用作正常GPIO用 */			
		SYS_PROTECT = 0;
}

/*******************************************************************************
 函数名称：    void Multiplex_nRST(void)
 功能描述：    nRST复用IO
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/9/20      V1.0           Li Li          创建
 *******************************************************************************/
void Multiplex_nRST(void)
{
		/*P0.0 NRST*/
		SYS_PROTECT = 0x7a83;
		SYS_IO_CFG |= BIT5;										 		 /*NRST复用为P0.0*/
		SYS_PROTECT = 0;
}

/*******************************************************************************
 函数名称：    void Multiplex_nRST(void)
 功能描述：    nRST恢复
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/9/20      V1.0           Li Li          创建
 *******************************************************************************/
void SET_nRST(void)
{
		/*P0.0 NRST*/
		SYS_PROTECT = 0x7a83;
		SYS_IO_CFG &= ~BIT5;										 		 /*P0.0恢复为NRST*/
		SYS_PROTECT = 0;
}

/*******************************************************************************
 函数名称：    void DebugPWM_OutputFunction(void)
 功能描述：    PWM输出功能调试   输出25%占空比
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2017/11/5      V1.0           Li Li          创建
 *******************************************************************************/
void DebugPWM_OutputFunction(void)
{
    MCPWM_TH00 = -(PWM_PERIOD >> 2);
    MCPWM_TH01 = (PWM_PERIOD >> 2);
    MCPWM_TH10 = -(PWM_PERIOD >> 2);
    MCPWM_TH11 = (PWM_PERIOD >> 2);
    MCPWM_TH20 = -(PWM_PERIOD >> 2);
    MCPWM_TH21 = (PWM_PERIOD >> 2);

    PWMOutputs(MCPWM0,ENABLE);
	
		Motor.Control.u32MCPWM_TH_temp = (PWM_PERIOD >> 2);
	
    while(1)
    {
    }
}

void CurrentOffsetCalibration(void)
{
    volatile u32 t_dlay;
    volatile u16 t_cnt;
    volatile s32 t_offset1, t_offset2, t_offset3, t_offset4;

    __disable_irq();

    ADC_SOFTWARE_TRIG_ONLY();			// ADC 使用软件触发
    ADC_STATE_RESET();

//    MCPWM_TH00 = 0;
//    MCPWM_TH01 = 0;

//    MCPWM_TH10 = 0;
//    MCPWM_TH11 = 0;

//    MCPWM_TH20 = 0;
//    MCPWM_TH21 = 0;
	
//  	Motor.Control.u32MCPWM_TH_temp = 0;
//     for (t_dlay = 0; t_dlay < 0xffff; t_dlay++)
//        ;
//    PWMOutputs(MCPWM0,ENABLE);
//    for (t_dlay = 0; t_dlay < 0x2ffff; t_dlay++)
//        ;

    t_offset1 = 0;
    t_offset2 = 0;
    t_offset3 = 0;
    t_offset4 = 0;

    ADC_CHN0 = ADC_PEAK_CUR_CHANNEL | (ADC_PEAK_CUR_CHANNEL << 4) | ( ADC_PEAK_CUR_CHANNEL << 8) | (ADC_PEAK_CUR_CHANNEL << 12);

    ADC_IF = 0x1f;

    ADC_SWT = 0x00005AA5;

    for (t_cnt = 0; t_cnt < CALIB_SAMPLES; t_cnt++)		// 采多次数据校准
    {
        for (t_dlay = 0; t_dlay < 0x1ff; t_dlay++);
        ADC_IF |= BIT1 | BIT0;
        t_offset1 += (s16)((ADC_DAT0));
        t_offset2 += (s16)((ADC_DAT1));
        t_offset3 += (s16)((ADC_DAT2));
        t_offset4 += (s16)((ADC_DAT3));

        /* Clear the ADC0 JEOC pending flag */
        ADC_SWT = 0x00005AA5;
    }
 
    PWMOutputs(MCPWM0,DISABLE);
		
    User_sys.BLDC_SensorlessCtr.PeakCurrOffset = (s16)(t_offset1 >> 9);		// 做平均
                          
    ADC_init();
    MCPWM_init();
    __enable_irq();
}





/*******************************************************************************
 函数名称：    void DC_offset_protect(void)
 功能描述：    直流偏置保护
 *******************************************************************************/
void DC_offset_protect(void)
{
	// Current offset error
	if ((User_sys.BLDC_SensorlessCtr.PeakCurrOffset  > OFFSET_THD) || (User_sys.BLDC_SensorlessCtr.PeakCurrOffset  < -OFFSET_THD))
	{
			User_sys.BLDC_SensorlessCtr.get_offset_flg = 1;
	}
	
	// system offset error
	if(User_sys.BLDC_SensorlessCtr.get_offset_flg)
	{
		e1M1_MainState = M1_MainState_Fault;						// MainState Fault
		e1M1_RunSubState = M1_RunState_Calib;						// RunState Calib					
		User_sys.BLDC_SensorlessCtr.sys_error_flg |= DC_OFFSET_ERROR;
	}
}

/************************ (C) COPYRIGHT SNANER SEMICONDUCTOR **********************/
/* ------------------------------END OF FILE------------------------------------ */
