/*******************************************************************************
 * 版权所有 (C)2019, SNANER SEMICONDUCTOR Co.ltd
 *
 * 文件名称： UartCtrl.c
 * 文件标识：
 * 内容摘要： 电机控制UART串口控制通信协议
 * 其它说明： 
 * 当前版本： V1.0
 * 作    者：   Li
 * 完成日期： 2022年5月30日
 *
 *******************************************************************************/
#include "UR_Ctrl.h"
#include "hardware_config.h"
#include "hardware_init.h"
#include "basic.h"
#include "MC_Parameter.h"

_UART0_STRUCT rxd_comm0;

volatile u8 UART_Flag = 1; //UART发送完成标志位
u8 UartResponceFlag = 0;	//UART接收完成标志位

/*******************************************************************************
 函数名称：    void UART1_init_buffer (void)
 功能描述：    UART0缓存初始化
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Li                  创建
 *******************************************************************************/
void UART0_init_buffer(void)
{
  rxd_comm0.read_pt = 0;
  rxd_comm0.write_pt = 0;
  rxd_comm0.cnt = 0;
}

/*******************************************************************************
 函数名称：    void UART0_init(void)
 功能描述：    UART0寄存器配置
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Li                  创建
 *******************************************************************************/
void UART0_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
	UART_InitTypeDef UART_InitStruct;
  GPIO_StructInit(&GPIO_InitStruct); //初始化结构体
	
  /* P1.7-RX0, P1.6-TX0  UART0 */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_6;
  GPIO_Init(GPIO1, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_7;
  GPIO_Init(GPIO1, &GPIO_InitStruct);
		
  GPIO_PinAFConfig(GPIO1, GPIO_PinSource_7, AF4_UART); //P1.7复用为UART_RX
  GPIO_PinAFConfig(GPIO1, GPIO_PinSource_6, AF4_UART); //P1.6复用为UART_TX

  UART_StructInit(&UART_InitStruct);
  UART_InitStruct.BaudRate     = UART0_BaudRate;      /* 设置波特率 */
  UART_InitStruct.WordLength   = UART_WORDLENGTH_8b;  /* 发送数据长度8位 */
  UART_InitStruct.StopBits     = UART_STOPBITS_1b;    /* 停止位长度1位 */
  UART_InitStruct.FirstSend    = UART_FIRSTSEND_LSB;  /* 先发送LSB */
  UART_InitStruct.ParityMode   = UART_Parity_NO;      /* 无奇偶校验 */
  UART_InitStruct.RXD_INV      = DISABLE;             /* RXD电平正常输出*/            
  UART_InitStruct.TXD_INV      = DISABLE;             /* TXD电平正常输出*/
	/*使能接收和发送完成中断*/
	UART_InitStruct.IRQEna       = UART_IRQEna_SendOver | UART_IRQEna_RcvOver; 
  UART_Init(UART0, &UART_InitStruct);
	
	UART0_init_buffer();	
}

/*******************************************************************************
 函数名称：    void UART0_SendByte(uint8_t ch)
 功能描述：    发送一个字节
 输入参数：    uint8_t ch
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Li                  创建
 *******************************************************************************/
void UART0_SendByte(uint8_t ch)
{
   UART_Flag = 1;
   UART0->BUFF = ch;
	 while(UART_Flag);
}

/*******************************************************************************
 函数名称：    void UART0_SendArray(uint8_t *array, uint16_t num)
 功能描述：    发送8位的数组
 输入参数：    uint8_t *array,uint16_t num
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Li                  创建
 *******************************************************************************/
void UART0_SendArray(uint8_t *array, uint16_t num)
{
  uint8_t i;
	UART_Flag = 1;
	for(i=0; i<num; i++)
  {
	    /* 发送一个字节数据到UART */
		   UART_Flag = 1;
	     UART0->BUFF = array[i];	
		   while(UART_Flag);
  }
}

/*******************************************************************************
 函数名称：    void Usart_SendString(char *str)
 功能描述：    发送字符串
 输入参数：    char *str
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Li                  创建
 *******************************************************************************/
void Usart_SendString(char *str)
{
	unsigned int k=0;
  do 
  {
		   UART_Flag = 1;
       UART0->BUFF = *(str + k);
       k++;
		   while(UART_Flag);//判断发送是否完成
  } while(*(str + k)!='\0');
}

/*******************************************************************************
 函数名称：    void Usart_SendHalfWord( uint16_t ch)
 功能描述：    发送一个16位数
 输入参数：    uint16_t ch
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Li                  创建
 *******************************************************************************/
void Usart_SendHalfWord( uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* 取出高八位 */
	temp_h = (ch&0XFF00)>>8;
	/* 取出低八位 */
	temp_l = ch&0XFF;
	
	/* 发送高八位 */
	UART_Flag = 1;
	UART0_SendByte(temp_h);	
	while(UART_Flag);
	
	/* 发送低八位 */
	UART_Flag = 1;
	UART0_SendByte(temp_l);	
	while(UART_Flag);	
}

/*******************************************************************************
 函数名称：    int fputc(int ch, FILE *f)
 功能描述：    重定向c库函数printf到串口，重定向后可使用printf函数
 输入参数：    int ch, FILE *f
 输出参数：    无
 返 回 值：    无
 其它说明：
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Li                   创建
 *******************************************************************************/
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到串口 */
		UART_SendData(UART0, (uint8_t) ch);	
//		/* 等待发送完毕 */
		while(UART_Flag);		
	
		return (ch);
}

/*******************************************************************************
 函数名称：    void UartDealRX(void)
 功能描述：    接收UART数据并校验
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
						// AA	         XX XX                     XX XX              XX XX             CKM              55
					  //头码   设定电机电气周期(Hz)    设定电机限功率(mW)       预留           校验和(低8位)		   结束码

 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Li                  创建
 *******************************************************************************/
#define Rnum	9
u8 Uart_R_DATA[Rnum];
void UartDealRX(void)
{
	u16 SpeedCMD;
	u16 PowerCMD;
  u8 tmp,i;
	u16 checkdate = 0x00;
	
  if(!rxd_comm0.cnt)
    return;

  tmp = rxd_comm0.buffer[rxd_comm0.read_pt];		//记录头码read pt地址
  if(tmp != 0xAA)            										//头码
  {
    NVIC_DisableIRQ(UART_IRQn);
    rxd_comm0.cnt--;  													//丢弃无效头码
    NVIC_EnableIRQ(UART_IRQn);
    rxd_comm0.read_pt = (rxd_comm0.read_pt + 1) % UART0_BUF_SIZE;
    return;
  }
  else if(rxd_comm0.cnt >= Rnum)    								//接收一组数据
  {
		for(i=0; i<Rnum; i++)
		{
			Uart_R_DATA[i] = rxd_comm0.buffer[rxd_comm0.read_pt];
			rxd_comm0.read_pt = (rxd_comm0.read_pt + 1) % UART0_BUF_SIZE;
			rxd_comm0.cnt--;
		} 
		if ((Uart_R_DATA[0]==0xAA) && (Uart_R_DATA[Rnum-1]==0x55))
		{
			for(i=0; i<(Rnum-2); i++)
			{
				checkdate += Uart_R_DATA[i];
			}
			if ((checkdate&0xFF) == Uart_R_DATA[Rnum-2])
			{
				UartResponceFlag = 1;										//接收完成，准备发送回复
				
				//获取需设定的转速数据
				SpeedCMD = (Uart_R_DATA[1]<<8) | Uart_R_DATA[2];
				PowerCMD = (Uart_R_DATA[3]<<8) | Uart_R_DATA[4];
				
				//转速设定
				if (SpeedCMD > 0)
				{
					//调速指令，速度参考设定定SPEED_SET单位Hz
					//struAppCommData.wSpeedValue = App2CoreFreqTrans(User2AppFreqTrans(SpeedCMD));	
					
					//功率指令，功率限制设定
					//struMotorSpeed.wPowerLimitValue = PowerCMD;
					
//					if((struFOC_CtrProc.bMC_RunFlg == 0) && (stru_Faults.R == 0))
//					{
//							struFOC_CtrProc.bMC_RunFlg = 1;
//					}				
					
					//有错误状态，则清除错误
					if (User_sys.BLDC_SensorlessCtr.sys_error_flg != 0)
					{
						User_sys.BLDC_SensorlessCtr.sys_error_flg = 0;	
					}
				  
				}
				else
				{
					//struAppCommData.wSpeedValue = 0;																//关机
					//struFOC_CtrProc.bMC_RunFlg = 0;
				}				
			}
		}	
  }
}

/*******************************************************************************
 函数名称：    void UartDealTX(void)
 功能描述：    处理串口并回传数据
 输入参数：    无
 输出参数：    无
 返 回 值：    无
 其它说明：
						// AA	        XX               XX 	XX                    XX XX               XX XX          XX XX                XX XX           CKM             55
					  //头码   回传电机状态    回传电机当前电气周期(Hz)     回传电机当前限功率(mW)      预留      回传电机错误代码      回传母线电压     校验和(低8位)		 结束码

 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2015/11/5      V1.0           Li                  创建
 *******************************************************************************/
#define Tnum	16
u8 Uart_T_DATA[Tnum];
void UartDealTX(void)
{
//	u16 j=0;
//	u16 checkdate = 0x00;
//	u16 SpeedValue;
	
	if(UartResponceFlag == 1)						
	{
		//TX发送数据
		//SpeedValue= Core2AppFreqTrans(struMotorSpeed.wSpeedfbk)/App2UserFreq;									

//		Uart_T_DATA[0]     = 0xAA;
//		Uart_T_DATA[1]     = struFOC_CtrProc.eSysState;                   				//电机状态
//		Uart_T_DATA[2]     = SpeedValue>>8;																			  //电机当前转动的电气周期 高8位(Hz) 					
//		Uart_T_DATA[3]     = (u8)SpeedValue;																			//电机当前转动的电气周期 低8位(Hz)		
//		Uart_T_DATA[4]     = struPower.wPowerValue>>8;          									//电机当前功率值 高8位(mW)
//		Uart_T_DATA[5]     = (u8)struPower.wPowerValue;         									//电机当前功率值 低8位(mW)
//		Uart_T_DATA[6]     = 0;               																		//预留
//		Uart_T_DATA[7]     = 0;																										//预留
//		Uart_T_DATA[8]     = stru_Faults.R>>8;					  												//电机错误代码 高8位
//		Uart_T_DATA[9]     = (u8)stru_Faults.R;																		//电机错误代码 低8位
//		Uart_T_DATA[10]     = Core2AppVdcTrans(struFOC_CurrLoop.nBusVoltage)>>8;  //实际母线电压=Uart_T_DATA[10]/256 高8位(V)
//		Uart_T_DATA[11]     = (u8)Core2AppVdcTrans(struFOC_CurrLoop.nBusVoltage); //实际母线电压=Uart_T_DATA[11]/256 低8位(V)
//		Uart_T_DATA[12]     = struPower.wPowerIbus>>8;  													//实际母线电流=Uart_T_DATA[12]/256 高8位(A)
//		Uart_T_DATA[13]     = (u8)struPower.wPowerIbus; 													//实际母线电流=Uart_T_DATA[13]/256 低8位(A)		
//		
//		for( j = 0; j < (Tnum-2); j++ )
//		{
//			checkdate += Uart_T_DATA[j];
//		}
//		Uart_T_DATA[Tnum-2]     = checkdate;
//		Uart_T_DATA[Tnum-1]     = 0x55;
//		UART0_SendArray(Uart_T_DATA, Tnum);	
		UartResponceFlag = 0;
	}		
}



