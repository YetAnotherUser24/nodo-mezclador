/*******************************************************************************
 * 版权所有 (C)2015, SNANER SEMICONDUCTOR Co.ltd
 *
 * 文件名称： hardware_init.c
 * 文件标识：
 * 内容摘要： 硬件初始化代码
 * 其它说明： 无
 * 当前版本： V 1.0
 * 作    者： Li 
 * 完成日期： 2021年11月15日
 *
 *******************************************************************************/
#include "hardware_config.h"
#include "hardware_init.h"
#include "basic.h"
#include "MC_Parameter.h"
#include "UR_Ctrl.h"
#include "main.h"

void Multiplex_nRST(void);
void Multiplex_SWD(void);
/*******************************************************************************
 函数名称：    void Hardware_init(void)
 功能描述：    硬件部分初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/15      V1.0        Li          创建
 *******************************************************************************/
void Hardware_init(void)
{
    __disable_irq();                  /* 关闭中断 中断总开关 */
		SYS_WR_PROTECT = 0x7a83;          /* 使能系统寄存器写操作 */  
    IWDG_DISABLE();                   /* 关闭独立看门狗使能*/
    FLASH_CFG |= 0x00080000;          /* FLASH 预取加速使能*/	
    delay_init(48);                   /* 延时函数初始化主频48MHz*/
		delay_ms(100);
	
		#if (UART0_FUNCTION == ENABLE_FUNCTION)
			UART0_init();										/* 串口初始化UART0*/
		#endif
	
    GPIO_init();                      /* GPIO初始化 */	
		HALL_Perip_Init(CLK_DIVISION);  	/* hall初始化*/
		PGA_init();                       /* PGA 初始化 */
	  ADC_init();                       /* ADC初始化 */
	  DAC_init();                       /* DAC 初始化 */
	  CMP_init();                       /* 比较器初始化 */	  
    UTimer_init(CLK_DIVISION);        /* UTimer初始化 */
    MCPWM_init();                     /* MCPWM初始化 */	
		BLDC_init();											/* BLDC硬件初始化 */
	
		DSP_Cmd(ENABLE);
	
		SoftDelay(500000);                /* 延时等待硬件初始化稳定 1000--134us*/

		NVIC_SetPriority(HALL_IRQn, 0);		/* 配置HALL中断优先级 */
		NVIC_EnableIRQ(HALL_IRQn);     		/* 使能HALL中断 */
				
    NVIC_SetPriority(ADC_IRQn, 1);    /* ADC中断优先级配置*/
    NVIC_EnableIRQ(ADC_IRQn);         /* 使能ADC中断*/	
	
    NVIC_SetPriority(TIMER0_IRQn, 2); /* TIMER0中断优先级配置*/
    NVIC_EnableIRQ(TIMER0_IRQn);      /* 使能TIMER0中断*/	

    NVIC_SetPriority(TIMER1_IRQn, 0); /* TIMER1中断优先级配置*/
    NVIC_EnableIRQ(TIMER1_IRQn);      /* 使能TIMER1中断*/		

    NVIC_SetPriority(MCPWM0_IRQn, 2); /* MCPWM0中断优先级配置*/
    NVIC_EnableIRQ(MCPWM0_IRQn);      /* 使能MCPWM0中断*/		

    NVIC_SetPriority(CMP_IRQn, 0);    /* CMP中断优先级配置*/	
		NVIC_EnableIRQ(CMP_IRQn);         /* 使能CMP中断*/	

		#if (UART0_FUNCTION == ENABLE_FUNCTION)
    NVIC_SetPriority(UART_IRQn, 1);   /* UART_IRQn外部中断优先级设置为1*/
    NVIC_EnableIRQ(UART_IRQn);        /* 使能UART_IRQn外部中断*/
		#endif

    delay_us(100);                    /* 延时等待硬件初始化稳定 */
    __enable_irq();                   /* 开启总中断 */
}

/*******************************************************************************
 函数名称：    void Clock_Init(void)  
 功能描述：    时钟配置
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/15      V1.0        Li          创建
 *******************************************************************************/
void Clock_Init(void)
{
	  delay_init(96);            /* 延时函数初始化主频96MHz*/
    SYS_WR_PROTECT = 0x7a83;   /* 解除系统寄存器写保护 */
    SYS_AFE_REG0 |= BIT15;     /* BIT15:PLLPDN */
    SoftDelay(100);            /* 延时100us,等待PLL稳定*/
    SYS_CLK_CFG |= 0x000001ff; /* BIT8:0: CLK_HS,1:PLL  | BIT[7:0]CLK_DIV  | 1ff对应96M时钟 */
	  SYS_SoftResetModule(0xfff);/* 复位所有外设寄存器 */
}

/*******************************************************************************
 函数名称：    void SystemInit(void)
 功能描述：    硬件系统初始化，调用时钟初始化函数
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/15      V1.0        Li        创建
 *******************************************************************************/
void SystemInit(void)
{
    Clock_Init(); /* 时钟初始化 */
}

/*******************************************************************************
 函数名称：    void GPIO_init(void)
 功能描述：    GPIO硬件初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/11/15      V1.0        Li        创建
 *******************************************************************************/
void GPIO_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructInit(&GPIO_InitStruct);
	
    /* MCPWM P0.10~P1.15 */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; /*输出模式*/
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIO0, &GPIO_InitStruct);
    /* P0.10~P1.15 复用成MCPWM功能 */
	  GPIO_PinAFConfig(GPIO0, GPIO_PinSource_10, AF3_MCPWM);
    GPIO_PinAFConfig(GPIO0, GPIO_PinSource_11, AF3_MCPWM);
    GPIO_PinAFConfig(GPIO0, GPIO_PinSource_12, AF3_MCPWM);
    GPIO_PinAFConfig(GPIO0, GPIO_PinSource_13, AF3_MCPWM);
    GPIO_PinAFConfig(GPIO0, GPIO_PinSource_14, AF3_MCPWM);
    GPIO_PinAFConfig(GPIO0, GPIO_PinSource_15, AF3_MCPWM);

	  //GPIO_P0.4/P0.5/P0.6 设置为 HALL_IN0/HALL_IN1 模拟通道
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
		GPIO_Init(GPIO0, &GPIO_InitStruct);
		GPIO_PinAFConfig(GPIO0, GPIO_PinSource_4, AF2_HALL);
		GPIO_PinAFConfig(GPIO0, GPIO_PinSource_5, AF2_HALL);
		GPIO_PinAFConfig(GPIO0, GPIO_PinSource_6, AF2_HALL);
	
  	/* P1.3 FG */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; /*输出模式*/
    GPIO_InitStruct.GPIO_Pin = GPIO_PIN_FG;    
    GPIO_Init(GPIO_FG, &GPIO_InitStruct);	

			/* P0.9 CW-CCW */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;  	/*GPIO输入模式*/
    GPIO_InitStruct.GPIO_Pin = GPIO_PIN_CWCCW;   
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; 	
    GPIO_Init(GPIO_CWCCW, &GPIO_InitStruct);	
		
		/*P0.0 LED/NRST*/
		Multiplex_nRST();
	  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; /*输出模式*/
    GPIO_InitStruct.GPIO_Pin = GPIO_PIN_LED;     
    GPIO_Init(GPIO_LED, &GPIO_InitStruct);		
		GPIO_WriteBit(GPIO_LED, GPIO_PIN_LED, Bit_SET);
}

/*******************************************************************************
 函数名称：    void HALL_Init(void)
 功能描述：    GPIO硬件初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2018/11/5      V1.0           Howlet Li          创建
 *******************************************************************************/
void HALL_Perip_Init(uint32_t Clk_Division)
{
  HALL_InitTypeDef HALL_InitStruct;
  HALL_StructInit(&HALL_InitStruct);
	
  HALL_InitStruct.HALL_Ena = ENABLE;             	/* 模块使能 */  
	
  HALL_InitStruct.ClockDivision = Clk_Division; 	/* 设置Hall模块时钟分频系数 */
  HALL_InitStruct.CountTH = 9600000;             	/* Hall模块计数模值，计数超过模值会产生超时中断 */
  HALL_InitStruct.FilterLen = 63;               	/* Hall信号数字滤波长度 256个时钟周期 */
	
  HALL_InitStruct.Filter75_Ena = ENABLE;         	/* Hall信号滤波方式，7判5模式或者全1有效模式 */
	
  HALL_InitStruct.Capture_IRQ_Ena = ENABLE;      	/* 捕捉中断使能 */
  HALL_InitStruct.OverFlow_IRQ_Ena = ENABLE;     	/* 超时中断使能 */
  HALL_InitStruct.softIE = DISABLE;              	/* 软件中断使能 */
  HALL_Init(&HALL_InitStruct);
}

/*******************************************************************************
 函数名称：    void ADC0_init(void)
 功能描述：    ADC0硬件初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Li Li          创建
 *******************************************************************************/
void ADC_init(void)
{
    ADC_InitTypeDef ADC_InitStructure;

    ADC_StructInit(&ADC_InitStructure);                       /*ADC初始化结构体*/
    ADC_InitStructure.Align = ADC_LEFT_ALIGN;                 /* ADC数据输出左对齐*/
    ADC_InitStructure.Trigger_Mode = ADC_1SEG_TRG;            /* 设置ADC转换模式为4段式采样 */
    ADC_InitStructure.FirSeg_Ch = 4;                          /* 第一段共采样8个通道 */
    ADC_InitStructure.SecSeg_Ch = 0;                          /* 第二段共采样0个通道 */
    ADC_InitStructure.ThrSeg_Ch = 0;                          /* 第三段共采样0个通道 */
    ADC_InitStructure.FouSeg_Ch = 0;                          /* 第四段共采样0个通道 */
    ADC_InitStructure.Trigger_Cnt = 0;                        /* 单段触发模式下触发一次采样所需要的事件数:
																																0~15 0表示需要一次触发，15表示需要16次 */
    ADC_InitStructure.Trigger_En = ADC_HARDWARE_T0_TRG;    	  /* 开启T0和T1硬件触发事件 */
    ADC_InitStructure.SEL_En = ADC_MCPWM_SEL;                 /* MCPWM触发ADC采样 */
    ADC_InitStructure.IE = ADC_EOS0_IRQ_EN;                   /* 第一段扫描结束中断*/
    ADC_InitStructure.ADC_SAMP_CLK = 8;                      	/* 设置采样时间为20个ADC时钟周期 范围 4--35*/
    ADC_InitStructure.IE = ADC_EOS0_IRQ_EN;                   /* 第一段、第二段、超阈值扫描结束中断 */
    ADC_Init(ADC, &ADC_InitStructure);										
		
    ADC_CHN0 = ADC_PEAK_CUR_CHANNEL  | (ADC_BUS_VOL_CHANNEL << 4) | ( ADC_SPEED_CHANNEL << 8) | (ADC_MOS_TEMP_CHANNEL << 12);
		
    ADC_IF = 0xff;                                            /* 清中断标志位	*/
    ADC_CFG |= BIT11;                                         /* 状态机复位控制信号	*/
}

/*******************************************************************************
 函数名称：    void UTimer_init(uint32_t Clk_Division)
 功能描述：    UTimer硬件初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Li Li          创建
 *******************************************************************************/
void UTimer_init(uint32_t Clk_Division)
{
    TIM_TimerInitTypeDef TIM_InitStruct;
	
    TIM_TimerStrutInit(&TIM_InitStruct); 				/* Timer结构体变量初始化 */
    TIM_InitStruct.EN = ENABLE;                 /* 模块使能 */
    TIM_InitStruct.CH1_MODE = TIMER_OPMode_CAP; /* 设置Timer CH1为捕获模式*/
    TIM_InitStruct.CH1_FE_CAP_EN = DISABLE;    
    TIM_InitStruct.CH1_RE_CAP_EN = DISABLE;   
    TIM_InitStruct.SRC1 = TIM_SRC0_1;           /* 捕获模式通道1信号来源,Timer通道1*/
    TIM_InitStruct.CAP1_CLR_EN = DISABLE;       /* 当发生CAP1捕获事件时，清零Timer计数器*/
    TIM_InitStruct.TH = 24000;                  /* 设置计数器计数模值 */
    TIM_InitStruct.FLT = 0;                     /* 通道 0/1 信号滤波宽度选择，0-255 */
    TIM_InitStruct.CLK_DIV = TIM_Clk_Div1;      /* 设置Timer模块数据分频系数 */
    TIM_InitStruct.IE = TIM_IRQ_IE_ZC;          /* 使能Timer模块CH1比较中断 */
    TIM_TimerInit(TIMER0, &TIM_InitStruct);

		// TIMER1 用于做进角(超前换相)调节, 
    TIM_TimerStrutInit(&TIM_InitStruct);        /* Timer结构体变量初始化 */
    TIM_InitStruct.EN = DISABLE;                /* 模块使能 */
    TIM_InitStruct.CH0_MODE = TIMER_OPMode_CAP; /* 设置Timer CH0为捕获模式*/
    TIM_InitStruct.CH0_FE_CAP_EN = DISABLE;     
    TIM_InitStruct.CH0_RE_CAP_EN = DISABLE;     
    TIM_InitStruct.SRC0 = TIM_SRC1_0;           /* 捕获模式通道1信号来源,Timer通道0*/
    TIM_InitStruct.CAP0_CLR_EN = DISABLE;       /* 当发生CAP0捕获事件时，清零Timer计数器*/
    TIM_InitStruct.TH = 48000;     							/* 设置计数器计数模值*/
    TIM_InitStruct.FLT = 0;                     /* 通道 0/1 信号滤波宽度选择，0-255 */
    TIM_InitStruct.CLK_DIV = Clk_Division; 	//与Hall模块的分频保持一致     /* 设置Timer模块数据分频系数 */
    TIM_InitStruct.IE = TIM_IRQ_IE_ZC;          /* 使能Timer模块CH0比较中断 */
    TIM_TimerInit(TIMER1, &TIM_InitStruct);
}

/*******************************************************************************
 函数名称：    void MCPWM_init(void)
 功能描述：    MCPWM_init硬件初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2021/8/25      V1.0           Li              创建
 *******************************************************************************/
void MCPWM_init(void)
{
    MCPWM_InitTypeDef MCPWM_InitStructure;
    MCPWM_StructInit(&MCPWM_InitStructure);

    MCPWM_InitStructure.CLK_DIV = 0;            /* MCPWM时钟分频设置 */
    MCPWM_InitStructure.MCLK_EN = ENABLE;       /* 模块时钟开启 */
    MCPWM_InitStructure.MCPWM_Cnt0_EN = ENABLE; /* 时基0主计数器开始计数使能开关 */
    MCPWM_InitStructure.MCPWM_WorkModeCH0 =	CENTRAL_PWM_MODE;	
    MCPWM_InitStructure.MCPWM_WorkModeCH1 = CENTRAL_PWM_MODE; /* 通道工作模式设置，中心对齐或边沿对齐 */
    MCPWM_InitStructure.MCPWM_WorkModeCH2 = CENTRAL_PWM_MODE;	
	
    /* 自动更新使能寄存器 MCPWM_TH00 自动加载使能 MCPWM_TMR0 自动加载使能 MCPWM_0TH 自动加载使能 MCPWM_0CNT 自动加载使能*/
    MCPWM_InitStructure.AUEN = TH00_AUEN | TH01_AUEN | TH10_AUEN | TH11_AUEN |
                               TH20_AUEN | TH21_AUEN | TMR0_AUEN | TMR1_AUEN |
                               TH0_AUEN;
	
    MCPWM_InitStructure.GPIO_BKIN_Filter = 12;                	/* 急停事件(来自IO口信号)数字滤波器时间设置 */
    MCPWM_InitStructure.CMP_BKIN_Filter = 12;                 	/* 急停事件(来自比较器信号)数字滤波器时间设置 */	
	
    MCPWM_InitStructure.TimeBase0_PERIOD = PWM_PERIOD;          /* 时期0周期设置 */
    MCPWM_InitStructure.TriggerPoint0 = (u16)( -100 ); 					//10 - PWM_PERIOD /* MCPWM_TMR0 ADC触发事件T0 设置 */
    MCPWM_InitStructure.TriggerPoint1 = (u16)(300); 						/* MCPWM_TMR1 ADC触发事件T1 设置 */

    MCPWM_InitStructure.DeadTimeCH0123N = DEADTIME;
    MCPWM_InitStructure.DeadTimeCH0123P = DEADTIME;             /* 死区时间设置 */

		#if ((PRE_DRIVER_POLARITY == HN_HIGH__LN_HIGH)) 
    MCPWM_InitStructure.CH0P_Polarity_INV = DISABLE;            /* CH0P通道输出极性设置 | 正常输出或取反输出 */
    MCPWM_InitStructure.CH0N_Polarity_INV = DISABLE;            /* CH0N通道输出极性设置 | 正常输出或取反输出*/
    MCPWM_InitStructure.CH1P_Polarity_INV = DISABLE;
    MCPWM_InitStructure.CH1N_Polarity_INV = DISABLE;
    MCPWM_InitStructure.CH2P_Polarity_INV = DISABLE;
    MCPWM_InitStructure.CH2N_Polarity_INV = DISABLE;

    /* 默认电平设置 默认电平输出不受MCPWM_IO01和MCPWM_IO23的 BIT0、BIT1、BIT8、BIT9、BIT6、BIT14
                                                     通道交换和极性控制的影响，直接控制通道输出 */
    MCPWM_InitStructure.CH0P_default_output = LOW_LEVEL;
    MCPWM_InitStructure.CH0N_default_output = LOW_LEVEL;
    MCPWM_InitStructure.CH1P_default_output = LOW_LEVEL;      /* CH1P对应引脚在空闲状态输出低电平 */
    MCPWM_InitStructure.CH1N_default_output = LOW_LEVEL;     /* CH1N对应引脚在空闲状态输出高电平 */
    MCPWM_InitStructure.CH2P_default_output = LOW_LEVEL;
    MCPWM_InitStructure.CH2N_default_output = LOW_LEVEL;	
		#else
    MCPWM_InitStructure.CH0P_Polarity_INV = DISABLE;            /* CH0P通道输出极性设置 | 正常输出或取反输出 */
    MCPWM_InitStructure.CH0N_Polarity_INV = ENABLE;            /* CH0N通道输出极性设置 | 正常输出或取反输出*/
    MCPWM_InitStructure.CH1P_Polarity_INV = DISABLE;
    MCPWM_InitStructure.CH1N_Polarity_INV = ENABLE;
    MCPWM_InitStructure.CH2P_Polarity_INV = DISABLE;
    MCPWM_InitStructure.CH2N_Polarity_INV = ENABLE;

    /* 默认电平设置 默认电平输出不受MCPWM_IO01和MCPWM_IO23的 BIT0、BIT1、BIT8、BIT9、BIT6、BIT14
                                                     通道交换和极性控制的影响，直接控制通道输出 */
    MCPWM_InitStructure.CH0P_default_output = LOW_LEVEL;
    MCPWM_InitStructure.CH0N_default_output = HIGH_LEVEL;
    MCPWM_InitStructure.CH1P_default_output = LOW_LEVEL;      /* CH1P对应引脚在空闲状态输出低电平 */
    MCPWM_InitStructure.CH1N_default_output = HIGH_LEVEL;     /* CH1N对应引脚在空闲状态输出高电平 */
    MCPWM_InitStructure.CH2P_default_output = LOW_LEVEL;
    MCPWM_InitStructure.CH2N_default_output = HIGH_LEVEL;	
		#endif
	
		
    MCPWM_InitStructure.Switch_CH0N_CH0P = DISABLE;           /* 通道交换选择设置 | CH0P和CH0N是否选择信号交换 */
    MCPWM_InitStructure.Switch_CH1N_CH1P = DISABLE;           /* 通道交换选择设置 */
    MCPWM_InitStructure.Switch_CH2N_CH2P = DISABLE;           /* 通道交换选择设置 */
		
    MCPWM_InitStructure.DebugMode_PWM_out = ENABLE;           /* 在接上仿真器debug程序时，暂停MCU运行时，选择各PWM通道正常输出调制信号
                                                                 还是输出默认电平，保护功率器件 ENABLE:正常输出 DISABLE:输出默认电平 */

    MCPWM_InitStructure.MCPWM_Base0T0_UpdateEN = ENABLE;      /* MCPWM T0事件更新使能 */
    MCPWM_InitStructure.MCPWM_Base0T1_UpdateEN = DISABLE;     /* MCPWM T1事件更新禁止 */
		
    MCPWM_InitStructure.CNT0_T0_Update_INT_EN = DISABLE;      /* T0更新事件 中断使能或关闭 */		
	
    MCPWM_InitStructure.FAIL0_INT_EN = DISABLE;               /* FAIL0事件 中断使能或关闭 */
    MCPWM_InitStructure.FAIL0_INPUT_EN = DISABLE;             /* FAIL0通道急停功能打开或关闭 */
    MCPWM_InitStructure.FAIL0_Signal_Sel = FAIL_SEL_CMP;      /* FAIL0事件信号选择，比较器或IO口 */
    MCPWM_InitStructure.FAIL0_Polarity = HIGH_LEVEL_VALID;    /* FAIL0事件极性选择，高有效或低有效 */

    MCPWM_InitStructure.FAIL1_INT_EN = ENABLE;               /* FAIL1事件 中断使能或关闭 */
    MCPWM_InitStructure.FAIL1_INPUT_EN = ENABLE;             /* FAIL1通道急停功能打开或关闭 */
    MCPWM_InitStructure.FAIL1_Signal_Sel = FAIL_SEL_CMP;      /* FAIL1事件信号选择，比较器或IO口 */
    MCPWM_InitStructure.FAIL1_Polarity = HIGH_LEVEL_VALID;    /* FAIL1事件极性选择，高有效或低有效 */

    MCPWM_PRT = 0x0000DEAD; 																	/* enter password to unlock write protection */
		//MCPWM_SWAP = 0x67;			// 是否切换PWM Output输出脚
		MCPWM_PRT = 0x0000CAFE;
		
    MCPWM_Init(MCPWM0, &MCPWM_InitStructure);                 /* MCPWM0 模块初始化 */
}


/*******************************************************************************
 函数名称：    void DAC_Init(void)
 功能描述：    DAC初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 *******************************************************************************/
void DAC_init(void)
{
	  DAC_InitTypeDef DAC_InitStruct;
    DAC_StructInit(&DAC_InitStruct);
    DAC_InitStruct.DACOUT_EN = DISABLE; /*关闭DAC通过IO（P0.0）输出*/

    DAC_Init(&DAC_InitStruct);
    DAC_OutputValue(SHORT_CURRENT_DAC);    
}
/*******************************************************************************
 函数名称：    void CMP_Init(void)
 功能描述：    比较器初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2016/3/15      V1.0           Li Li          创建
 *******************************************************************************/
void CMP_init(void)
{
    CMP_InitTypeDef CMP_InitStruct;
    CMP_StructInit(&CMP_InitStruct);
		
    SYS_AnalogModuleClockCmd(SYS_AnalogModule_CMP0,DISABLE); 	/* 比较器0开关 */
    SYS_AnalogModuleClockCmd(SYS_AnalogModule_CMP1,ENABLE);  /* 比较器1开关 */
	
		//CMP_InitStruct.CMP_HYS = ENABLE;               /* 比较器关闭回差20MV */ 	  
    CMP_InitStruct.FIL_CLK10_DIV16 = 5;           /* 即滤波宽度=tclk 周期*16*CMP_FltCnt */
    CMP_InitStruct.CLK10_EN = ENABLE;              /* 时钟使能 */
	
    CMP_InitStruct.CMP0_SELN = CMP0_SELN_DAC;
    CMP_InitStruct.CMP0_SELP = CMP0_SELP_CMP0_IP0; 		 /* CMP0_P CMP0_IP0母线 */
    CMP_InitStruct.CMP0_IN_EN = DISABLE;            /* 比较器信号输入使能 */
    CMP_InitStruct.CMP0_IE = DISABLE;              /* 比较器0信号中断使能 */
		
    CMP_InitStruct.CMP1_SELN = CMP1_SELN_DAC;      /* CMP1_N 内部DAC 输出 CMP1_SELN_DAC */
    CMP_InitStruct.CMP1_SELP = CMP1_SELP_CMP1_IP0; /* CMP1_P CMP1_IP0母线*/
    CMP_InitStruct.CMP1_IN_EN = ENABLE;            /* 比较器信号输入使能 */
    CMP_InitStruct.CMP1_IE = ENABLE;              /* 比较器1信号中断使能 */ 

    CMP_Init(&CMP_InitStruct);
}

/*******************************************************************************
 函数名称：    void PGA_Init(void)
 功能描述：    PGA初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 *******************************************************************************/
void PGA_init(void)
{
    SYS_AnalogModuleClockCmd(SYS_AnalogModule_OPA, ENABLE);		/* OPA使能 */
	
    SYS_WR_PROTECT = 0x7A83; 

		SYS_AFE_REG0 |= OPA_GIAN; 			/* OPA增益设置 */
	
	  SYS_WR_PROTECT = 0;
}

/*******************************************************************************
 函数名称：    void HALL_Init(void)
 功能描述：    GPIO硬件初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 *******************************************************************************/
void HALL_init(void)
{
    HALL_InitTypeDef HALL_InitStruct;
    HALL_StructInit(&HALL_InitStruct);
	
    HALL_InitStruct.FilterLen = 512;                /* Hall信号数字滤波长度 512个时钟周期 */
    HALL_InitStruct.ClockDivision = HALL_CLK_DIV1;  /* 设置Hall模块时钟分频系数 */
    HALL_InitStruct.Filter75_Ena = DISABLE;         /* Hall信号滤波方式，7判5模式或者全1有效模式 */
    HALL_InitStruct.HALL_Ena = ENABLE;              /* 模块使能 */
    HALL_InitStruct.Capture_IRQ_Ena = ENABLE;       /* 捕捉中断使能 */
    HALL_InitStruct.OverFlow_IRQ_Ena = ENABLE;      /* 超时中断使能 */
    HALL_InitStruct.CountTH = 9600000;              /* Hall模块计数模值，计数超过模值会产生超时中断 */

    HALL_Init(&HALL_InitStruct);
}

/*******************************************************************************
 函数名称：    void SoftDelay(void)
 功能描述：    软件延时函数
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 *******************************************************************************/
void SoftDelay(u32 cnt)
{
    volatile u32 t_cnt;

    for (t_cnt = 0; t_cnt < cnt; t_cnt++)
    {
        __nop();
    }
}


