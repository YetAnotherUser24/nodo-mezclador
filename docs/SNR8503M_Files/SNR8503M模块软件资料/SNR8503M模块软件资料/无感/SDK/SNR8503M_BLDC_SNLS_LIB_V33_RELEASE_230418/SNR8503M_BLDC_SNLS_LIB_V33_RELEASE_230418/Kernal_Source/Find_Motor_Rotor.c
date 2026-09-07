#include "basic.h"
#include "global_variable.h"
#include "MC_Parameter.h"
#include "motor_structure.h"
#include "COMP_Sensorless.h"
#include "MC_Drive.h"
#include "M1_StateMachine.h"
#include "Find_Motor_Rotor.h"

#define CHANNLLOW       4  //TIME1 TIME2-CCMR1,CCMR2
#define CHANNLHIGH      5  //TIME1 TIME2-CCMR1,CCMR2
#define PWM1ENBLE       6  //TIME1 TIME2-CCMR1,CCMR2

/***********************************************************************
* 函数名称: ToPwmValue
* 功能描述: 占空比更新函数
* 输出参数: 无
* 返回参数: 无
************************************************************************/
void ToPwmValue(uint16_t PwmDuty)
{
	  //TIM1->CCR123预装载使能
	  //只在更新事件发生时才写入此寄存器
		MCPWM_TH00 = -PwmDuty;
		MCPWM_TH01 =  PwmDuty;
		MCPWM_TH10 = -PwmDuty;
		MCPWM_TH11 =  PwmDuty;
		MCPWM_TH20 = -PwmDuty;
		MCPWM_TH21 =  PwmDuty; 
}

/***********************************************************************
* 函数名称: turn_off
* 功能描述: 关闭MOS驱动输出
* 输出参数: 无
* 返回参数: 无
************************************************************************/
void turn_off(void)
{
		MCPWM_TH00 = -0;
		MCPWM_TH01 =  0;
		MCPWM_TH10 = -0;
		MCPWM_TH11 =  0;
		MCPWM_TH20 = -0;
		MCPWM_TH21 =  0; 
		PWMOutputs(MCPWM0,DISABLE);
}

/***********************************************************************
* 函数名称: MOS_Electronic_Brake
* 功能描述: MOS电子刹车
* 输出参数: 无
* 返回参数: 无
************************************************************************/
void MOS_Electronic_Brake(uint8_t FunInside_over_cur_flag)
{
    if(FunInside_over_cur_flag != 0)
    {
				turn_off();
    }
    else
    {
				MCPWM_TH00 = -0;
				MCPWM_TH01 =  0;
				MCPWM_TH10 = -0;
				MCPWM_TH11 =  0;
				MCPWM_TH20 = -0;
				MCPWM_TH21 =  0; 
			
				/* 上桥关闭,下桥打开 */
				//开三相下桥,关三相上桥
				MCPWM_PRT = 0x0000DEAD;
				MCPWM_IO01 = DRIVER_POLARITY | 0x0808;
				MCPWM_IO23 = DRIVER_POLARITY | 0x08;
				MCPWM_PRT = 0x0000CAFE;
    }
}

//正常通道单片机PWM输出 UVW
const uint16_t NoHallSystem_CCMR_CCER[28] =
{
  (CC1_Out_Pwm1|CC2_Out_Effect_level),		CC3_Out_NoEffect_level,		(CC1_HS_EN|CC2_LS_EN|CC3_LS_EN), 		0, 		//AB  0
	
  (CC1_Out_Pwm1|CC2_Out_NoEffect_level),	CC3_Out_Effect_level, 		(CC1_HS_EN|CC2_LS_EN|CC3_LS_EN), 		0, 		//AC  1
	
  (CC1_Out_NoEffect_level|CC2_Out_Pwm1),	CC3_Out_Effect_level, 		(CC1_LS_EN|CC2_HS_EN|CC3_LS_EN), 		0, 		//BC  2
	
  (CC1_Out_Effect_level|CC2_Out_Pwm1),		CC3_Out_NoEffect_level, 	(CC1_LS_EN|CC2_HS_EN|CC3_LS_EN), 		0, 		//BA  3
	
  (CC1_Out_Effect_level|CC2_Out_NoEffect_level),		CC3_Out_Pwm1, 	(CC1_LS_EN|CC2_LS_EN|CC3_HS_EN), 		0, 		//CA  4
	
  (CC1_Out_NoEffect_level|CC2_Out_Effect_level),		CC3_Out_Pwm1, 	(CC1_LS_EN|CC2_LS_EN|CC3_HS_EN), 		0, 		//CB  5
	
	(CC1_Out_Pwm1|CC2_Out_Effect_level),							CC3_Out_Pwm1, 	(CC1_HS_EN|CC2_LS_EN|CC3_HS_EN), 		0, 		//AC B AC(定位) 6  
};


/***********************************************************************
* 函数名称: Motor_Rotor_Detect
* 功能描述: 电机转子检测状态
* 输出参数: 无
* 返回参数: ReturnValue
************************************************************************/
void Motor_Rotor_Detect(void)
{
	static uint8_t  Find_Motor_Location_Pre_delay_Flag=0;
	static uint16_t Find_Motor_Location_Pre_delay_cnt=0;

	if(Find_Motor_Location_Pre_delay_Flag == 0)
	{
			MOS_Electronic_Brake(BLDC_HITPULSE.bOver_Cur_Flag);		// 刹车

			Find_Motor_Location_Pre_delay_cnt++;
			if(Find_Motor_Location_Pre_delay_cnt > 1250)	//1250*20us = 25ms
			{
					Find_Motor_Location_Pre_delay_cnt = 0;
					Find_Motor_Location_Pre_delay_Flag = 1;
			}
	}
	else if(Find_Motor_Location_Pre_delay_Flag == 1)
	{	
			Find_Motor_Rotor_Position();		// 寻找转子位置

			if(BLDC_HITPULSE.bHit_Pulse_Error_Use_Location_Flag == 1)
			{
				 BLDC_HITPULSE.bFind_Motor_Location_Successful_Flag = 1;   
				
				 Motor_1st.BLDC_CMP.u8ControlStep = MOTOR_ROTOR_ALIGN;
				
				 Find_Motor_Location_Pre_delay_cnt = 0;
				 Find_Motor_Location_Pre_delay_Flag = 0;
				 BLDC_HITPULSE.bFind_Motor_Location_Successful_Flag = 0;
				 BLDC_HITPULSE.bHit_Pulse_Error_Use_Location_Flag = 0;		
			}
	}
}


/***********************************************************************
* 函数名称: Hid_Pulse_Logic
* 功能描述: 打脉冲的逻辑
* 输出参数: 无
* 返回参数: 无
************************************************************************/
void Hid_Pulse_Logic(uint8_t temp)
{
		static uint8_t inside_temp = 0;
	
	  inside_temp = temp;
			
		switch(inside_temp)
		{
			case 0://打AB脉冲
                         BLDC_NoHallSystem_Change_Phanse_Logic(BLDC_HITPULSE.bOver_Cur_Flag,0);   
						 break;
			
			case 1://打BA脉冲
                         BLDC_NoHallSystem_Change_Phanse_Logic(BLDC_HITPULSE.bOver_Cur_Flag,3);
						 break;
			
			case 2://打AC脉冲
                         BLDC_NoHallSystem_Change_Phanse_Logic(BLDC_HITPULSE.bOver_Cur_Flag,1);
						 break;
			
			case 3://打CA脉冲
                         BLDC_NoHallSystem_Change_Phanse_Logic(BLDC_HITPULSE.bOver_Cur_Flag,4);
						 break;
			
			case 4://打BC脉冲
                         BLDC_NoHallSystem_Change_Phanse_Logic(BLDC_HITPULSE.bOver_Cur_Flag,2);
						 break;
			
			case 5://打CB脉冲
                         BLDC_NoHallSystem_Change_Phanse_Logic(BLDC_HITPULSE.bOver_Cur_Flag,5);
						 break;
		}

		ToPwmValue(HITPULSE_DUTY);		//打脉冲占空比 10 20 30  不同电机需要占空比不一样 
}

/***********************************************************************
* 函数名称: sort
* 功能描述: 选择排序法对一维数组排序
* 输出参数: 无
* 返回参数: 无
************************************************************************/
void sort(uint16_t array[],uint8_t m)
{
		static uint8_t  i=0,j=0,k=0;
	  static uint16_t temp=0;
	
	  for(i=0; i<m-1; i++)
    {
				k = i;	//k记录当前擂台i的位置
			
			  for(j=i+1; j<m; j++)
			  {
					  if(array[j] < array[k])//从小到大排序:小于号 从大到小排序:大于号
						{
								k = j; //当前获胜者作为擂主,k跟踪记录其位置						  
						}								
				}	
				
				/* 擂主交换位置 */
				temp = array[k];
				array[k] = array[i];
				array[i] = temp;			
		}
}

/***********************************************************************
* 函数名称: Find_FirstMax_SecondMax_Value
* 功能描述: 找最大值、次最大值,并记录其位置
* 输出参数: 无
* 返回参数: 无
************************************************************************/
void Find_FirstMax_SecondMax_Value(uint16_t array[],uint8_t m)
{
		static uint8_t  i=0,k=0;
	  static uint16_t FirstMax_Value=0;
	  static uint16_t SecondMax_Value=0;  
	
	  /* 找最大值,并记录其位置 */
	  FirstMax_Value = 0;
	  for(i=0; i<m; i++)
    {
			  if(array[i] > FirstMax_Value)
				{
					 FirstMax_Value = array[i];
           k = i;		//记录当前最大值的位置					
				}	
		}
		BLDC_HITPULSE.u8First_number = k;//把最大值的位置输出
		
		/* 找次最大值,并记录其位置 */
		SecondMax_Value = 0;
	  for(i=0;i<m;i++)
    {   
			  if(i != BLDC_HITPULSE.u8First_number)
				{
						if(array[i] > SecondMax_Value)
						{
							 SecondMax_Value = array[i];
							 k = i;	//记录当前次最大值的位置					
						}	
				}  
		}	
		BLDC_HITPULSE.u8Second_number = k;//把次最大值的位置输出
}

/***********************************************************************
* 函数名称: Hit_pulse_find_position_if_successful_function
* 功能描述: 确认高频注入法找位置是否正确
* 输出参数: 无
* 返回参数: 无
************************************************************************/
void Hit_pulse_find_position_if_successful_function(void)
{
		switch(BLDC_HITPULSE.u8First_number)
		{
			 case 0:	// AB
						if(BLDC_HITPULSE.u8Second_number==2 || BLDC_HITPULSE.u8Second_number==5)	//电流邻近最大值是AC或者CB
						{
								BLDC_HITPULSE.bhit_pulse_successful_flag = 1;
						}
						else
						{
								BLDC_HITPULSE.bhit_pulse_successful_flag = 0;
						}
						break;
						
			 case 1:	//BA
						if(BLDC_HITPULSE.u8Second_number == 3 || BLDC_HITPULSE.u8Second_number == 4)	//电流邻近最大值是CA或者BC
						{	
								BLDC_HITPULSE.bhit_pulse_successful_flag = 1;
						}
						else
						{
								BLDC_HITPULSE.bhit_pulse_successful_flag = 0;
						}
						break;
						
			 case 2:	//AC
						if(BLDC_HITPULSE.u8Second_number==4||BLDC_HITPULSE.u8Second_number==0)	//电流邻近最大值是BC或者AB
						{
								BLDC_HITPULSE.bhit_pulse_successful_flag = 1;
						}
						else
						{
								BLDC_HITPULSE.bhit_pulse_successful_flag = 0;
						}
						break;
						
			 case 3:	//CA
						if(BLDC_HITPULSE.u8Second_number==5||BLDC_HITPULSE.u8Second_number==1)	//电流邻近最大值是CB或者BA
						{
								BLDC_HITPULSE.bhit_pulse_successful_flag = 1;
						}
						else
						{
								BLDC_HITPULSE.bhit_pulse_successful_flag = 0;
						}
						break;
						
			 case 4:	//BC
						if(BLDC_HITPULSE.u8Second_number==1||BLDC_HITPULSE.u8Second_number==2)	//电流邻近最大值是BA或者AC
						{
								BLDC_HITPULSE.bhit_pulse_successful_flag = 1;
						}
						else
						{
								BLDC_HITPULSE.bhit_pulse_successful_flag = 0;
						}
						break;
						
			 case 5:	//CB
						if(BLDC_HITPULSE.u8Second_number==0||BLDC_HITPULSE.u8Second_number==3)	//电流邻近最大值是AB或者CA
						{
								BLDC_HITPULSE.bhit_pulse_successful_flag = 1;
						}
						else
						{
								BLDC_HITPULSE.bhit_pulse_successful_flag = 0;
						}
						break;								
		}
}

/***********************************************************************
* 函数名称: Sure_CutIn_OpenLoop_MOS_Number
* 功能描述: 高频注入法确定位置后,确认位置的序号
* 输出参数: 无
* 返回参数: 无
************************************************************************/
void Sure_CutIn_OpenLoop_MOS_Number(uint8_t temp)
{
	  static uint8_t inside_temp=0;
	
	  inside_temp = temp;
	
		switch(inside_temp)
		{
				case 0:	//AB脉冲最大
							BLDC_HITPULSE.u8Motor_Drag_number = 0;
							break;
				
				case 1:	//BA脉冲最大
							BLDC_HITPULSE.u8Motor_Drag_number = 3;
							break;
				
				case 2:	//AC脉冲最大
							BLDC_HITPULSE.u8Motor_Drag_number = 1;
							break;
				
				case 3:	//CA脉冲最大
							BLDC_HITPULSE.u8Motor_Drag_number = 4;
							break;
				
				case 4:	//BC脉冲最大
							BLDC_HITPULSE.u8Motor_Drag_number = 2;
							break;
				
				case 5:	//CB脉冲最大
							BLDC_HITPULSE.u8Motor_Drag_number = 5;
							break;   
		}
}

/***********************************************************************
* 函数名称: Find_Motor_Rotor_Position
* 功能描述: 高频注入法,找转子位置
* 输出参数: 无
* 返回参数: 无
************************************************************************/
void Find_Motor_Rotor_Position(void)
{
		#define my_define_hit_pulse_number 9  
		static uint16_t current_ad_buff[my_define_hit_pulse_number] = {0};	// 高频脉冲电流数据存储
    static uint16_t hit_pulse_cur_sum_value = 0;												// 电流数据累加和
    static uint16_t hit_pulse_cur[6] = {0};															// 六步开通电流数据存储
	  static uint8_t  hit_pulse_step_flag = 0;
	  static uint8_t  storage_hit_pulse_cur_step_flag = 0;
	  static uint8_t  cur_i = 0;																					// 数组计数
	  static uint8_t  hit_pulse_number = 0;																// 开通步数计数
		static uint16_t hit_next_pulse_delay_cnt = 0;												// 延时
		static uint8_t  hit_next_pulse_delay_flag = 0; 	
    static uint8_t  hit_pulse_error_cnt = 0;														// 检测错误次数累计
			
		if((hit_pulse_step_flag == 0) && (storage_hit_pulse_cur_step_flag == 0))
		{	  
			  // 打脉冲阶段			  
			  /* 每打下一个脉冲之前先延时一段时间 */
			  if(hit_next_pulse_delay_flag == 0)
				{
						hit_next_pulse_delay_cnt++;
					  if(hit_next_pulse_delay_cnt > 500)	
						{
								hit_next_pulse_delay_cnt = 0;
							  hit_next_pulse_delay_flag = 1;
						}
						
						/* 延时期间要刹车,防止意外转子转动,找位置失败 */
						MOS_Electronic_Brake(BLDC_HITPULSE.bOver_Cur_Flag);
				}
				else
				{
						Hid_Pulse_Logic(hit_pulse_number);
						hit_pulse_step_flag = 1;
					
					  cur_i = 0;
						BLDC_HITPULSE.bAD_Cur_Flag = 0;			
					  hit_next_pulse_delay_flag = 0;				  
				}	  
		}
		else if(hit_pulse_step_flag == 1 && storage_hit_pulse_cur_step_flag == 0)
		{
			 //存储打脉冲的电流值阶段
			 if(cur_i < my_define_hit_pulse_number)
			 {
					if(BLDC_HITPULSE.bAD_Cur_Flag == 1)
					{
							BLDC_HITPULSE.bAD_Cur_Flag = 0;
						
							current_ad_buff[cur_i] = BLDC_HITPULSE.u16AD_Value_Cur;
							cur_i++;
					}
			 }
			 else
			 {
					/* 每打完一次脉冲马上关闭PWM输出 (不能刹车,否则电流过大,影响采AD,延时期间再刹车) */
					turn_off();
				 
					storage_hit_pulse_cur_step_flag = 1;					 
					cur_i = 0;
					BLDC_HITPULSE.bAD_Cur_Flag = 0;
			 }			
		}
		else if(hit_pulse_step_flag == 1 && storage_hit_pulse_cur_step_flag == 1)
		{			
        /* 做数字虑波处理 */
			  sort(current_ad_buff,my_define_hit_pulse_number);		// 有可能最后面的并不一定就是最大的电流值,需要排序
//			  for(cur_i = my_define_hit_pulse_number - 2; cur_i < my_define_hit_pulse_number; cur_i++)		// 选择最后两个脉冲电流作为最大电流值
			  for(cur_i = 1; cur_i < my_define_hit_pulse_number; cur_i++)		// 选择最后两个脉冲电流作为最大电流值
        {
						hit_pulse_cur_sum_value += current_ad_buff[cur_i];
				}
 							
				/* 存储这个电流值 */
				hit_pulse_cur[hit_pulse_number] = hit_pulse_cur_sum_value;		
			  hit_pulse_cur_sum_value = 0;		// 必须清零
						
				hit_pulse_number++;							// 下一项打脉冲
        if(hit_pulse_number >= 6)
				{											
					  /* 找最大、次最大值,确定电机转子位置 */
					  Find_FirstMax_SecondMax_Value(hit_pulse_cur,6);
					
					  /* 判断这两个位置是否正确 */
					  Hit_pulse_find_position_if_successful_function();
                    
            /* 确定当前位置的序号 */						
					  Sure_CutIn_OpenLoop_MOS_Number(BLDC_HITPULSE.u8First_number);
						Motor_1st.BLDC_CMP.u8step_Index = BLDC_HITPULSE.u8Motor_Drag_number;	// 开通相序
						if(Motor_1st.BLDC.u8Direction == ECW)
						{								
							Motor_1st.BLDC_CMP.u8step_Index = Motor_step_tab_cw[Motor_1st.BLDC_CMP.u8step_Index];
						}
						else
						{
							Motor_1st.BLDC_CMP.u8step_Index = Motor_step_tab_ccw[Motor_1st.BLDC_CMP.u8step_Index];
						}
						
						if(BLDC_HITPULSE.bhit_pulse_successful_flag == 1)
						{
							  /* 高频注入法找位置正确 */															
							  BLDC_HITPULSE.bFind_Motor_Location_Successful_Flag = 1;	//找位置完成						
							  BLDC_HITPULSE.bhit_pulse_successful_flag = 0;                
                hit_pulse_error_cnt = 0;	

								Motor_1st.BLDC_CMP.u8ControlStep = MOTOR_ROTOR_ALIGN;
							
								BLDC_HITPULSE.bFind_Motor_Location_Successful_Flag = 0;
								BLDC_HITPULSE.bHit_Pulse_Error_Use_Location_Flag = 0;		
						}
						else
            {
								/* 高频注入法找位置错误 */
							  /* 重新打脉冲 */
							
								turn_off();

								hit_pulse_error_cnt++;
								if(hit_pulse_error_cnt >= 3)	//1 3
								{
									hit_pulse_error_cnt = 0;
									 
									BLDC_HITPULSE.bHit_Pulse_Error_Use_Location_Flag = 1;                    
								}
								else
								{
									//小于三次重新打脉冲
								}
						}							
            									
						hit_pulse_number = 0;	
				}

				hit_pulse_step_flag = 0;
				storage_hit_pulse_cur_step_flag = 0;
		}	
}

