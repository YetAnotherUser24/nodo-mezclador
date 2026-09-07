/*******************************************************************************
 * 版权所有 (C)2021, SNANER SEMICONDUCTOR Co.ltd
 *
 * 文件名称： snr8503x_MCPWM.c
 * 文件标识：
 * 内容摘要： MCPWM外设驱动程序
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者：   Li
 * 完成日期： 2021年10月14日
 *
 *******************************************************************************/
#include "snr8503x_MCPWM.h"
#include "snr8503x_sys.h"
#include "snr8503x.h"
#include "string.h"

/*******************************************************************************
 函数名称：    void PWMOutputs(MCPWM_REG_TypeDef *MCPWM, FuncState t_state)
 功能描述：    MCPWM CH0/CH1/CH2 信号输出使能
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/10/14   V1.0             Li              创建
 *******************************************************************************/
void PWMOutputs(MCPWM_REG_TypeDef *MCPWMx, FuncState t_state)
{
    MCPWMx->PWM_PRT = 0x0000DEAD;
    if (t_state == ENABLE)
    {
        MCPWMx->PWM_FAIL012 |= MCPWM_MOE_ENABLE_MASK;
    }
    else
    {
        MCPWMx->PWM_FAIL012 &= MCPWM_MOE_DISABLE_MASK;
    }
    MCPWMx->PWM_PRT = 0x0000CAFE;
}

/*******************************************************************************
 函数名称：    void PWMOutputs_CH3(MCPWM_REG_TypeDef *MCPWM, FuncState t_state)
 功能描述：    MCPWM CH3 信号输出使能
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/10/14   V1.0             Li              创建
 *******************************************************************************/
void PWMOutputs_CH3(MCPWM_REG_TypeDef *MCPWMx, FuncState t_state)
{
    MCPWMx->PWM_PRT = 0x0000DEAD;
    if (t_state == ENABLE)
    {
        MCPWMx->PWM_FAIL3 |= MCPWM_MOE_ENABLE_MASK;
    }
    else
    {
        MCPWMx->PWM_FAIL3 &= MCPWM_MOE_DISABLE_MASK;
    }
    MCPWMx->PWM_PRT = 0x0000CAFE;
}

/*******************************************************************************
 函数名称：    void PWM_CH3_Outputs(MCPWM_REG_TypeDef *MCPWM, FuncState t_state)
 功能描述：    MCPWM CH3通道信号输出
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/10/14   V1.0             Li              创建
 *******************************************************************************/
void PWM_CH3_Outputs(MCPWM_REG_TypeDef *MCPWMx, FuncState t_state)
{
    MCPWMx->PWM_PRT = 0x0000DEAD;

    if (t_state == ENABLE)
    {
        MCPWMx->PWM_FAIL3 |= MCPWM_MOE_ENABLE_MASK;
    }
    else
    {
        MCPWMx->PWM_FAIL3 &= MCPWM_MOE_DISABLE_MASK;
    }
    MCPWMx->PWM_PRT = 0x0000CAFE;
}

/*******************************************************************************
 函数名称：    void MCPWM_StructInit(MCPWM_InitTypeDef* MCPWM_InitStruct)
 功能描述：    MCPWM结构体初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/10/14   V1.0             Li              创建
 *******************************************************************************/
void MCPWM_StructInit(MCPWM_InitTypeDef *MCPWM_InitStruct)
{

    memset(MCPWM_InitStruct, 0, sizeof(MCPWM_InitTypeDef));
}

/*******************************************************************************
 函数名称：    void MCPWM_Init(MCPWM_REG_TypeDef *MCPWM, MCPWM_InitTypeDef* MCPWM_InitStruct)
 功能描述：    MCPWM初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/10/14   V1.0             Li              创建
 *******************************************************************************/
void MCPWM_Init(MCPWM_REG_TypeDef *MCPWMx, MCPWM_InitTypeDef *MCPWM_InitStruct)
{
    SYS_ModuleClockCmd(SYS_Module_MCPWM, ENABLE);

    MCPWMx->PWM_PRT = 0x0000DEAD; /*enter password to unlock write protection */
    MCPWMx->PWM_TCLK = MCPWM_InitStruct->CLK_DIV | (MCPWM_InitStruct->MCLK_EN << 2) | (MCPWM_InitStruct->TMR2_TimeBase_Sel << 4) | (MCPWM_InitStruct->TMR3_TimeBase_Sel << 5) | 
	                    (MCPWM_InitStruct->MCPWM_Cnt0_EN << 6) | (MCPWM_InitStruct->MCPWM_Cnt1_EN << 7) | (MCPWM_InitStruct->TimeBase0_Trig_Enable << 8) | (MCPWM_InitStruct->TimeBase1_Trig_Enable << 9);
	
    MCPWMx->PWM_TH0 = MCPWM_InitStruct->TimeBase0_PERIOD;
    MCPWMx->PWM_TH1 = MCPWM_InitStruct->TimeBase1_PERIOD;

    MCPWMx->PWM_TMR0 = MCPWM_InitStruct->TriggerPoint0;
    MCPWMx->PWM_TMR1 = MCPWM_InitStruct->TriggerPoint1;
    MCPWMx->PWM_TMR2 = MCPWM_InitStruct->TriggerPoint2;
    MCPWMx->PWM_TMR3 = MCPWM_InitStruct->TriggerPoint3;

    MCPWMx->PWM_FLT = (MCPWM_InitStruct->CMP_BKIN_Filter << 8) | MCPWM_InitStruct->GPIO_BKIN_Filter;

    MCPWMx->PWM_IO01 = MCPWM_InitStruct->CH0N_Polarity_INV | (MCPWM_InitStruct->CH0P_Polarity_INV << 1) | (MCPWM_InitStruct->Switch_CH0N_CH0P << 6) | (MCPWM_InitStruct->MCPWM_WorkModeCH0 << 7) | 
	                     (MCPWM_InitStruct->CH1N_Polarity_INV << 8) | (MCPWM_InitStruct->CH1P_Polarity_INV << 9) | (MCPWM_InitStruct->Switch_CH1N_CH1P << 14) | (MCPWM_InitStruct->MCPWM_WorkModeCH1 << 15);

    MCPWMx->PWM_IO23 = MCPWM_InitStruct->CH2N_Polarity_INV | (MCPWM_InitStruct->CH2P_Polarity_INV << 1) | (MCPWM_InitStruct->Switch_CH2N_CH2P << 6) | (MCPWM_InitStruct->MCPWM_WorkModeCH2 << 7) | 
	                     (MCPWM_InitStruct->CH3N_Polarity_INV << 8) | (MCPWM_InitStruct->CH3P_Polarity_INV << 9) | (MCPWM_InitStruct->Switch_CH3N_CH3P << 14) | (MCPWM_InitStruct->MCPWM_WorkModeCH3 << 15);

    MCPWMx->PWM_FAIL012 = MCPWM_InitStruct->FAIL0_Signal_Sel | (MCPWM_InitStruct->FAIL0_Polarity << 2) | (MCPWM_InitStruct->FAIL0_INPUT_EN << 4) | 
		                     (MCPWM_InitStruct->FAIL1_INPUT_EN << 5) | (MCPWM_InitStruct->FAIL1_Signal_Sel << 1) | (MCPWM_InitStruct->FAIL1_Polarity << 3) | (MCPWM_InitStruct->DebugMode_PWM_out << 7) | 
												 (MCPWM_InitStruct->CH0N_default_output << 8) | (MCPWM_InitStruct->CH0P_default_output << 9) | (MCPWM_InitStruct->CH1N_default_output << 10) | 
												 (MCPWM_InitStruct->CH1P_default_output << 11) | (MCPWM_InitStruct->CH2N_default_output << 12) | (MCPWM_InitStruct->CH2P_default_output << 13);

    MCPWMx->PWM_FAIL3 = MCPWM_InitStruct->FAIL2_Signal_Sel | (MCPWM_InitStruct->FAIL2_Polarity << 2) | (MCPWM_InitStruct->FAIL2_INPUT_EN << 4) | (MCPWM_InitStruct->FAIL3_INPUT_EN << 5) |
                   		(MCPWM_InitStruct->FAIL3_Signal_Sel << 1) | (MCPWM_InitStruct->FAIL3_Polarity << 3) | (MCPWM_InitStruct->DebugMode_PWM_out << 7) | (MCPWM_InitStruct->CH3P_default_output << 8) | 
											(MCPWM_InitStruct->CH3N_default_output << 9);

    MCPWMx->PWM_SDCFG = MCPWM_InitStruct->MCPWM_UpdateT0Interval | (MCPWM_InitStruct->MCPWM_Base0T0_UpdateEN << 4) | (MCPWM_InitStruct->MCPWM_Base0T1_UpdateEN << 5) | (MCPWM_InitStruct->MCPWM_Auto0_ERR_EN << 6) | 
		                    (MCPWM_InitStruct->MCPWM_UpdateT1Interval) << 8 | (MCPWM_InitStruct->MCPWM_Base1T0_UpdateEN << 12) | (MCPWM_InitStruct->MCPWM_Base1T1_UpdateEN << 13) | (MCPWM_InitStruct->MCPWM_Auto1_ERR_EN << 14);

    MCPWMx->PWM_DTH0 = MCPWM_InitStruct->DeadTimeCH0123N;
    MCPWMx->PWM_DTH1 = MCPWM_InitStruct->DeadTimeCH0123P;

    MCPWMx->PWM_AUEN = MCPWM_InitStruct->AUEN;

    MCPWMx->PWM_EVT0 = MCPWM_InitStruct->TimeBase_TrigEvt0;
    MCPWMx->PWM_EVT1 = MCPWM_InitStruct->TimeBase_TrigEvt1;

    MCPWMx->PWM_CNT0 = MCPWM_InitStruct->TimeBase0Init_CNT;
    MCPWMx->PWM_CNT1 = MCPWM_InitStruct->TimeBase1Init_CNT;

    MCPWMx->PWM_IE0 = MCPWM_InitStruct->CNT0_T0_Update_INT_EN | (MCPWM_InitStruct->CNT0_T1_Update_INT_EN << 1) | (MCPWM_InitStruct->CNT0_Update_TH00_EN << 2) | (MCPWM_InitStruct->CNT0_Update_TH01_EN << 3) |
                 		(MCPWM_InitStruct->CNT0_Update_TH10_EN << 4) | (MCPWM_InitStruct->CNT0_Update_TH11_EN << 5) | (MCPWM_InitStruct->CNT0_Update_TH20_EN << 6) | (MCPWM_InitStruct->CNT0_Update_TH21_EN << 7) |
										(MCPWM_InitStruct->CNT0_Update_TH30_EN << 8) | (MCPWM_InitStruct->CNT0_Update_TH31_EN << 9) | (MCPWM_InitStruct->CNT0_TMR0_Match_INT_EN << 10) | 
										(MCPWM_InitStruct->CNT0_TMR1_Match_INT_EN << 11) | (MCPWM_InitStruct->CNT0_TMR2_Match_INT_EN << 12) | (MCPWM_InitStruct->CNT0_TMR3_Match_INT_EN << 13) | (MCPWM_InitStruct->CNT0_Update_EN << 14);
										
    MCPWMx->PWM_IE1 = MCPWM_InitStruct->CNT1_T0_Update_INT_EN | (MCPWM_InitStruct->CNT1_T1_Update_INT_EN << 1) | (MCPWM_InitStruct->CNT1_Update_TH30_EN << 8) | (MCPWM_InitStruct->CNT1_Update_TH31_EN << 9) | 
		                 (MCPWM_InitStruct->CNT1_TMR2_Match_INT_EN << 12) | (MCPWM_InitStruct->CNT1_TMR3_Match_INT_EN << 13) | (MCPWM_InitStruct->CNT1_Update_EN << 14);

    MCPWMx->PWM_EIE = (MCPWM_InitStruct->FAIL0_INT_EN << 4) | (MCPWM_InitStruct->FAIL1_INT_EN << 5) | (MCPWM_InitStruct->FAIL2_INT_EN << 6) | (MCPWM_InitStruct->FAIL3_INT_EN << 7);

    MCPWMx->PWM_RE = (MCPWM_InitStruct->TMR0_DMA_RE) | (MCPWM_InitStruct->TMR1_DMA_RE << 1) | (MCPWM_InitStruct->TMR2_DMA_RE << 2) | (MCPWM_InitStruct->TMR3_DMA_RE << 3) | (MCPWM_InitStruct->TR0_T0_DMA_RE << 4) | 
		                 (MCPWM_InitStruct->TR0_T1_DMA_RE << 5) | (MCPWM_InitStruct->TR1_T0_DMA_RE << 6) | (MCPWM_InitStruct->TR1_T1_DMA_RE << 7);

    MCPWMx->PWM_PP = MCPWM_InitStruct->IO_PPE;

    MCPWMx->PWM_UPDATE = 0xffff; /* write whatever value to trigger update */

    MCPWMx->PWM_IF0 = 0xffff;
    MCPWMx->PWM_IF1 = 0xffff;
    MCPWMx->PWM_EIF = 0xffff;

    MCPWMx->PWM_PRT = 0x0000CAFE; /* write any value other than 0xDEAD to enable write protection */
}

/*******************************************************************************
 函数名称：    void Predrive_Circuit(void)
 功能描述：    MCPWM通道连接预驱时交换
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：    芯片内部带预驱时使用
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/10/14   V1.0             Li              创建
 *******************************************************************************/
void MCPWM_SwapFunction(void)
{
    MCPWM_PRT = 0x0000DEAD; /* 解除MCPWM 寄存器写保护*/
    MCPWM_SWAP = 0x67;      /* 开启芯片内部MCPWM通道转换顺序*/
    MCPWM_PRT = 0x0000CAFE; /* 开启MCPWM 寄存器写保护*/
}

/************************ (C) COPYRIGHT SNANER SEMICONDUCTOR *****END OF FILE****/
