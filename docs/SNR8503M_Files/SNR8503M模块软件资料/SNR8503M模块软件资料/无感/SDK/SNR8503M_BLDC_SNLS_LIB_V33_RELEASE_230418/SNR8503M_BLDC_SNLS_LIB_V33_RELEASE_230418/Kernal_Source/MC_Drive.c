/*-------------------- Includes -----------------------*/
#include "MC_Drive.h"
#include "Global_Variable.h"
#include "motor_structure.h"
#include "M1_StateMachine.h"
#include "SpeedRamp.h"
#include "MC_Parameter.h"
#include "PID.h"
#include "commonfunc.h"

u8 CWCCW_bIOInput = 0;	
/*------------------- Private variables ---------------*/
void BLDC_Communication(unsigned char Hall_value);
void BLDC_COMP_Input_Only(unsigned char Hall_value);
void VSP_Control_Motor(void);
void BLDC_Stop(void);
void BLDC_Brake(void);

/*------------------ Private functions ----------------*/
void BLDC_Communication(unsigned char Hall_value)
{
    MCPWM_PRT = 0x0000DEAD;
		SYS_WR_PROTECT = 0x7a83;
		SYS_AFE_REG1 &= 0xf8f3;																				//只操作比较器0，比较器1作过流保护不动
    switch(Hall_value) 
		{
			case VH_WL_HPWM_LON:
					// A相悬空，B相PWM C相ON ；检测下降沿过零事件
					// A float | B PWM | C ON
					MCPWM_IO01 = DRIVER_POLARITY | (0x040c);		
					MCPWM_IO23 = DRIVER_POLARITY | 0x1c;			
			
					SYS_AFE_REG1 |= (CMP0_SELN_SET << 2) | (CMP_IP_A << 8); //只操作比较器0，比较器1作过流保护不动
			break;
				
				
			case VH_UL_HPWM_LON: 
					// A相ON，B相PWM C相悬空 ；检测上升沿过零事件
					// A ON | B PWM | C float
					MCPWM_IO01 = DRIVER_POLARITY | 0x041c;      	// 
					MCPWM_IO23 = DRIVER_POLARITY | (0x0c);
			
					SYS_AFE_REG1 |= (CMP0_SELN_SET << 2) | (CMP_IP_C << 8);
			break;
				
			
			case WH_UL_HPWM_LON: 
					// A相ON，B相悬空 C相PWM ；检测下降沿过零事件
					// A ON | B float | C PWM
					MCPWM_IO01 = DRIVER_POLARITY | (0x0c1c);		// 互补模式
					MCPWM_IO23 = DRIVER_POLARITY | 0x04;
			
					SYS_AFE_REG1 |= (CMP0_SELN_SET << 2) | (CMP_IP_B << 8);  
			break;
				
			
			case WH_VL_HPWM_LON: 
					// A相悬空，B相ON C相PWM  ；检测上升沿过零事件
					// A float | B ON | C PWM
					MCPWM_IO01 = DRIVER_POLARITY | (0x1c0c);		// 互补模式
					MCPWM_IO23 = DRIVER_POLARITY | 0x04;	
			
					SYS_AFE_REG1 |= (CMP0_SELN_SET << 2) | (CMP_IP_A << 8);
			break;
				
			
			case UH_VL_HPWM_LON: 
					// A相PWM，B相ON C相悬空  ；检测下降沿过零事件
					// A PWM | B on | C float
					MCPWM_IO01 = DRIVER_POLARITY | 0x1c04;			// 互补模式
					MCPWM_IO23 = DRIVER_POLARITY | (0x0c);	
			
					SYS_AFE_REG1 |= (CMP0_SELN_SET << 2) | (CMP_IP_C << 8);			
			break;
					
				
			case UH_WL_HPWM_LON: 
					// A相PWM，B相悬空 C ON；检测上升沿过零事件
					// A pwm | B float | C ON
					MCPWM_IO01 = DRIVER_POLARITY | (0x0c04);		// 互补模式
					MCPWM_IO23 = DRIVER_POLARITY | 0x1c;
			
					SYS_AFE_REG1 |= (CMP0_SELN_SET << 2) | (CMP_IP_B << 8);  
			break;
		}
		MCPWM_PRT = 0x0000CAFE;
		SYS_WR_PROTECT = 0;
}

void BLDC_COMP_Input_Only(unsigned char Hall_value)
{
		SYS_WR_PROTECT = 0x7a83;
		SYS_AFE_REG1 &= 0xf8f3;																				//只操作比较器0，比较器1作过流保护不动
    switch(Hall_value) 
		{
			case VH_WL_HPWM_LON:
					// A相悬空，B相PWM C相ON ；检测下降沿过零事件
					// A float | B PWM | C ON
					SYS_AFE_REG1 |= (CMP0_SELN_SET << 2) | (CMP_IP_A << 8);	//只操作比较器0，比较器1作过流保护不动
			break;
				
				
			case VH_UL_HPWM_LON: 
					// A相ON，B相PWM C相悬空 ；检测上升沿过零事件
					// A ON | B PWM | C float
					SYS_AFE_REG1 |= (CMP0_SELN_SET << 2) | (CMP_IP_C << 8);
			break;
				
			
			case WH_UL_HPWM_LON: 
					// A相ON，B相悬空 C相PWM ；检测下降沿过零事件
					// A ON | B float | C PWM
					SYS_AFE_REG1 |= (CMP0_SELN_SET << 2) | (CMP_IP_B << 8);  
			break;
				
			
			case WH_VL_HPWM_LON: 
					// A相悬空，B相ON C相PWM  ；检测上升沿过零事件
					// A float | B ON | C PWM
					SYS_AFE_REG1 |= (CMP0_SELN_SET << 2) | (CMP_IP_A << 8);
			break;
				
			
			case UH_VL_HPWM_LON: 
					// A相PWM，B相ON C相悬空  ；检测下降沿过零事件
					// A PWM | B on | C float
					SYS_AFE_REG1 |= (CMP0_SELN_SET << 2) | (CMP_IP_C << 8);			
			break;
					
				
			case UH_WL_HPWM_LON: 
					// A相PWM，B相悬空 C ON；检测上升沿过零事件
					// A pwm | B float | C ON
					SYS_AFE_REG1 |= (CMP0_SELN_SET << 2) | (CMP_IP_B << 8);  
			break;
		}
		SYS_WR_PROTECT = 0;
}

void VSP_Control_Motor(void)
{
		static  uint8_t i;
		static  s32 RefDUTY;
		static  s32 DesirdDuty;
	
		if(Motor_1M1st.USER.u16VSP_filt_value < VSP_OFF_VALUE)					//电位器未按下
		{
				Motor_1M1st.BLDC.u8_ControlStep = 0;

			#if EN_BRAKE
					BLDC_Brake();
			#else
					BLDC_Stop();			
			#endif

				Motor_1M1st.BLDC.u16_TimCnt = 0;
				Motor_1M1st.BLDC.u8FlagEnMotorRun = 0;		// Motor stop	
				Motor_1M1st.BLDC.u8FlagMotorIsRunning = 0;
			
				//e1M1_RunSubState = M1_RunState_Calib;
		}
		else if(Motor_1M1st.USER.u16VSP_filt_value > VSP_START_VALUE)
		{
			if(Motor_1M1st.BLDC.u16_TimCnt < 3)
			{
				Motor_1M1st.BLDC.u16_TimCnt++;
				Motor_1M1st.BLDC.u8FlagEnMotorRun = 1; 		// Enable Motor Start
			}
			else
			{
				Motor_1M1st.BLDC.u16_TimCnt = 3;
			}
		}
		
		// Motor speed control 
		if(Motor_1M1st.BLDC.u8FlagMotorIsRunning == 1)
		{			
			if (Motor_1M1st.BLDC_CMP.u32MotorRPMFilte_cnt < STARTUP_DRAG_TIME)
			{
				Motor_1M1st.BLDC_CMP.u32MotorRPMFilte_cnt++;
				DesirdDuty = MOTOR_STARTUP_PWMDUTY;
			}
			else
			{
				#if EN_MOTOR_SPEED_CLOSELOOP	// 闭环调速
					CalcNormalization(Motor_1M1st.USER.u16VSP_filt_value,&SPEED_Cmd);		/* 调速参数归一化处理 */
					SPEEDValue.Dest = CalcGraudNormalizationData(SPEED_Cmd.Out,MOTOR_SPEED_MIN_RPM,SPEED_ACC_MS,SPEED_DEC_MS,1);	/*缓冲调速输出	，加减速曲线*/
				#else	
				// 开环VSP
				CalcNormalization(Motor_1M1st.USER.u16VSP_filt_value,&RP);					/* 调速参数归一化处理 */	
				RPValue.Dest = CalcGraudNormalizationData(RP.Out,MIN_PWM_DUTY,VSP_DUTY_ACC_LOAD,VSP_DUTY_DEC_LOAD,1);	/*缓冲PWM占空比输出，加减速曲线*/
				#endif
				
				//计算转速RPM
				Motor_1M1st.BLDC_CMP.u32MotorElePeriodValue = 0;
				for(i=0;i<6;i++)
				{
					Motor_1M1st.BLDC_CMP.u32MotorElePeriodValue += Motor_1M1st.BLDC_CMP.u32MotorElePhaseValue[i];				// 电气周期
				}				
				Motor_1M1st.BLDC_CMP.u32MotorRPM = MOTOR_SPEED_X/Motor_1M1st.BLDC_CMP.u32MotorElePeriodValue;

				//调速
				#if EN_MOTOR_SPEED_CLOSELOOP	// 速度闭环
					if(++Motor_1M1st.USER.SpeedPI_Prc >= SPEED_PI_PRC)  //速度环调节
					{
						Motor_1M1st.USER.SpeedPI_Prc = 0;
						PIParmS.qInMeas = Motor_1M1st.BLDC_CMP.u32MotorRPM;         //
						PIParmS.qInRef = SPEEDValue.Dest; 
						CalcPI(&PIParmS); 
					}
					RefDUTY = PIParmS.qOut;
					DesirdDuty = PiCurrentLimitPWMDuty(RefDUTY,1);  			//经过电流限制后的PWM占空比值	
				#else	// 开环调速
					RefDUTY = RPValue.Dest;
					DesirdDuty = PiCurrentLimitPWMDuty(RefDUTY,1);  			//经过电流限制后的PWM占空比值	
				#endif	
			}

			if (DesirdDuty > MAX_PWM_DUTY)
			{
				DesirdDuty = MAX_PWM_DUTY;
			}
			
			if(DesirdDuty < MIN_PWM_DUTY)
			{
				DesirdDuty = MIN_PWM_DUTY;
			}
			MCPWM_TH00 = 	-	DesirdDuty;
			MCPWM_TH01 = 		DesirdDuty;
			MCPWM_TH10 = 	-	DesirdDuty;
			MCPWM_TH11 = 		DesirdDuty;
			MCPWM_TH20 = 	-	DesirdDuty;
			MCPWM_TH21 = 		DesirdDuty;			
		}
		else
		{
			PiCurrentLimitPWMDuty(0,0);  
			#if EN_MOTOR_SPEED_CLOSELOOP	// 闭环调速			
				CalcGraudNormalizationData(0,MOTOR_SPEED_MIN_RPM,0,0,0);		
			#else													// VSP开环	
				CalcGraudNormalizationData(0,MIN_PWM_DUTY,0,0,0);		
			#endif
		}
}


//////////////////////////////////////////////////////////////////////////

//函数名称: PiCurrentLimitPWMDuty(u16 Duty_In,u8 newstate)

//函数功能说名：峰值电流限制PI调节后装载Duty值

//输入参数: 无

//输出参数: 无

//调用函数: 无

//完成时间:2022-02-28

//作者: Danny

///////////////////////////////////////////////////////////////

u16 PiCurrentLimitPWMDuty(u16 Duty_In,u8 newstate)
{
    static u32 ReturnValue;
		static u32 DutyValueIn;	
		static u8  DataAccelerate = 20;  
		DutyValueIn = Duty_In;
		if(newstate)
		{
			if(User_sys.BLDC_SensorlessCtr.u32Ibus_Final > User_sys.BLDC_SensorlessCtr.u32SW_LimitCurrent_Value) //均值电流环限制
			{
				PIParmIevg.qInMeas = User_sys.BLDC_SensorlessCtr.u32Ibus_Final;         
				PIParmIevg.qInRef = User_sys.BLDC_SensorlessCtr.u32SW_LimitCurrent_Value; 
				CalcPI(&PIParmIevg);  
				ReturnValue = PIParmIevg.qOut;
				RPValue.Act = PIParmIevg.qOut;
			}
			else
			{    
				ReturnValue  = Adj(ReturnValue,DutyValueIn,DataAccelerate);
				PIParmIevg.qdSum = ReturnValue << 15;
				PIParmIevg.qOut = ReturnValue; 
			}					
		}
		else
		{
			ReturnValue = MIN_PWM_DUTY;
		}
		return(ReturnValue);
}

/*******************************************************************************
 函数名称：    void Motor_LED_Error(void)
 功能描述：    电机LED错误
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/9/20      V1.0           Li Li          创建
 *******************************************************************************/
void Motor_LED_Error(void)
{
	static u8 Err100ms_CNT = 0;	
	static u8 MotorErrTiems = 0;
	static u8 MotorErrStep = 0;
	static u8 MotorErrCNT = 0;
	
	Err100ms_CNT++;
	if (Err100ms_CNT > 20)																													//200ms
	{
		Err100ms_CNT = 0;
		
		//错误指示
		if(User_sys.BLDC_SensorlessCtr.sys_error_flg == SHORT_ERROR)									/* 短路故障 */
		{
			MotorErrTiems = 2;																													//1个脉冲，停2秒
		}
		else if (User_sys.BLDC_SensorlessCtr.sys_error_flg == LOW_VOL_ERROR)					/* 低压故障 */
		{
			MotorErrTiems = 4;																													//2个脉冲，停2秒
		}
		else if (User_sys.BLDC_SensorlessCtr.sys_error_flg == HIG_VOL_ERROR)					/* 过压故障 */
		{
			MotorErrTiems = 6;																													//3个脉冲，停2秒
		}
		else if (User_sys.BLDC_SensorlessCtr.sys_error_flg == BLOCK_ERROR)						/* 堵转故障 */
		{
			MotorErrTiems = 8;																													//4个脉冲，停2秒		
		}
		else if (User_sys.BLDC_SensorlessCtr.sys_error_flg == DC_OFFSET_ERROR)				/* 直流偏置故障 */
		{
			MotorErrTiems = 10;																													//5个脉冲，停2秒						
		}
		else if (User_sys.BLDC_SensorlessCtr.sys_error_flg == MOS_OVER_ERROR)					/* MOS过温故障 */
		{
			MotorErrTiems = 12;																													//6个脉冲，停2秒						
		}			
		else if (User_sys.BLDC_SensorlessCtr.sys_error_flg == MOS_LOW_ERROR)					/* MOS低温故障 */
		{
			MotorErrTiems = 14;																													//7个脉冲，停2秒
		}
		else if (User_sys.BLDC_SensorlessCtr.sys_error_flg == BAT_OVER_ERROR)					 /* 电池过温故障 */
		{
			MotorErrTiems = 16;																													//8个脉冲，停2秒
		}		
		else if (User_sys.BLDC_SensorlessCtr.sys_error_flg == BAT_LOW_ERROR)					 /* 电池低温故障 */
		{
			MotorErrTiems = 18;																													//9个脉冲，停2秒
		}	
		else if (User_sys.BLDC_SensorlessCtr.sys_error_flg == OVER_LOAD_ERROR)				/* 过流故障 */
		{
			MotorErrTiems = 20;																													//10个脉冲，停2秒
		}	
		else if (User_sys.BLDC_SensorlessCtr.sys_error_flg == PHASE_DROP_ERROR)				/* 缺相故障 */
		{
			MotorErrTiems = 22;																													//11个脉冲，停2秒
		}		
		else if (User_sys.BLDC_SensorlessCtr.sys_error_flg == MOSFET_ERROR)						/* MOS自检故障 */
		{
			MotorErrTiems = 24;																													//12个脉冲，停2秒
		}		
		
		switch(MotorErrStep)
		{
			case 0:
				LED_ONOFF;
			  FG_ONOFF;
				MotorErrCNT++;
				if (MotorErrCNT >= MotorErrTiems)				//共计次数
				{
					MotorErrCNT = 0;
					MotorErrStep = 1;
				}								
				break;
				
			case 1:
				LED_OFF;		
				FG_ON;
				MotorErrCNT++;
				if (MotorErrCNT >= 10)										//2s
				{
					MotorErrCNT = 0;
					MotorErrStep = 0;
				}
				break;	
				
			default:
				break;
		}			
	}	
}

/*******************************************************************************
 函数名称：    void Motor_LED_DISP(void)
 功能描述：    电机LED显示
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/9/20      V1.0           Li Li          创建
 *******************************************************************************/
void Motor_LED_DISP(void)
{
	static u8 cnt_led=0;
	if (Motor_1M1st.BLDC.u8FlagEnMotorRun == 0)		// Motor stop	
	{
		if (User_sys.BLDC_SensorlessCtr.sys_error_flg == 0)		//no err
		{		
			cnt_led++;
			if (cnt_led >= 50)
			{
				cnt_led = 0;
				LED_ONOFF;
			}		
			FG_OFF;
		}
		else 
		{
			Motor_LED_Error();
		}
	}
	else																										// Motor run
	{
		if (User_sys.BLDC_SensorlessCtr.sys_error_flg == 0)		//no err
		{
			LED_ON;
		}
		else
		{
			Motor_LED_Error();
		}
	}
}

/*******************************************************************************
 函数名称：    void CWCCW_ReadIO(void)
 功能描述：    电机转向设定
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/9/20      V1.0           Li Li          创建
 *******************************************************************************/
void CWCCW_PowerOn(void)
{
	CWCCW_bIOInput = GPIO_ReadInputDataBit(GPIO_CWCCW, GPIO_PIN_CWCCW);
}

/*******************************************************************************
 函数名称：    void CWCCW_ReadIO(void)
 功能描述：    电机转向设定
 修改日期      版本号          修改人            修改内容
 -----------------------------------------------------------------------------
 2020/9/20      V1.0           Li Li          创建
 *******************************************************************************/
void CWCCW_ReadIO(void)
{
	static u8 CWCCW_IOInputCNT1 = 0;
	static u8 CWCCW_IOInputCNT2 = 0;

	if (GPIO_ReadInputDataBit(GPIO_CWCCW, GPIO_PIN_CWCCW) == 0)	
	{
		CWCCW_IOInputCNT2 = 0;
		if (CWCCW_bIOInput == 1)
		{
			CWCCW_IOInputCNT1++;
			if (CWCCW_IOInputCNT1 > 5)
			{
				CWCCW_IOInputCNT1 = 0;
				CWCCW_bIOInput = 0;
			}		
		}		
	}
	else
	{
		CWCCW_IOInputCNT1 = 0;
		if (CWCCW_bIOInput == 0)
		{
			CWCCW_bIOInput = 0;
			CWCCW_IOInputCNT2++;
			if (CWCCW_IOInputCNT2 > 5)
			{
				CWCCW_IOInputCNT2 = 0;
				CWCCW_bIOInput = 1;
			}
		}
	}		
}
			

void BLDC_Stop(void)
{
		MCPWM_TH00 = -0;
		MCPWM_TH01 =  0;
		MCPWM_TH10 = -0;
		MCPWM_TH11 =  0;
		MCPWM_TH20 = -0;
		MCPWM_TH21 =  0; 
		PWMOutputs(MCPWM0,DISABLE);
}

void BLDC_Brake(void)
{
	MCPWM_TH00 = -0;
  MCPWM_TH01 =  0;
  MCPWM_TH10 = -0;
  MCPWM_TH11 =  0;
  MCPWM_TH20 = -0;
  MCPWM_TH21 =  0; 
 
  //开三相下桥,关三相上桥
  MCPWM_PRT = 0x0000DEAD;
  MCPWM_IO01 = DRIVER_POLARITY | 0x1C1C;
  MCPWM_IO23 = DRIVER_POLARITY | 0x1C;
  MCPWM_PRT = 0x0000CAFE;

  PWMOutputs(MCPWM0,ENABLE);
}






