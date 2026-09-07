#ifndef _Find_Motor_Rotor_H_
#define _Find_Motor_Rotor_H_


#include "main.h"

//CC1通道输出比较模式配置(TIM1->CCMR1寄存器里配置) 
//有效电平:高   
//无效电平:低
#define CC1_Out_Pwm1 	          (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1)//PWM模式1 
#define CC1_Out_Effect_level    (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0)//强制为有效电平
#define CC1_Out_NoEffect_level  (TIM_CCMR1_OC1M_2)//强制为无效电平

//CC2通道输出比较模式配置(TIM1->CCMR1寄存器里配置) 
#define CC2_Out_Pwm1            (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1)//PWM模式1  
#define CC2_Out_Effect_level    (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_0)//强制为有效电平
#define CC2_Out_NoEffect_level  (TIM_CCMR1_OC2M_2)//强制为无效电平

//CC3通道输出比较模式配置(TIM1->CCMR2寄存器里配置) 
#define CC3_Out_Pwm1 	          (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1)//PWM模式1  
#define CC3_Out_Effect_level    (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_0)//强制为有效电平
#define CC3_Out_NoEffect_level  (TIM_CCMR2_OC3M_2)//强制为无效电平

//CC123通道极性配置(TIM1->CCER寄存器里配置)
//0:高有效 1：低有效
#define CC1_POL_HS  0		                      //(高有效)
#define CC1_POL_LS  0 //TIM_CCER_CC1NP	          //(低有效)
#define CC1_POL     (CC1_POL_HS | CC1_POL_LS) //上下桥极性叠加

#define CC2_POL_HS  0               					//(高有效)
#define CC2_POL_LS  0 //TIM_CCER_CC2NP  					//(低有效)
#define CC2_POL     (CC2_POL_HS | CC2_POL_LS) //上下桥极性叠加

#define CC3_POL_HS  0               					//(高有效)
#define CC3_POL_LS  0 //TIM_CCER_CC3NP  					//(低有效)
#define CC3_POL     (CC3_POL_HS | CC3_POL_LS) //上下桥极性叠加

//CC123通道使能配置(TIM1->CCER寄存器里配置)
#define CC1_HS_EN   (TIM_CCER_CC1E|CC1_POL)   //CC1高边使能
#define CC1_LS_EN   (TIM_CCER_CC1NE|CC1_POL)  //CC1低边使能		 

#define CC2_HS_EN 	 (TIM_CCER_CC2E|CC2_POL)  //CC2高边使能   
#define CC2_LS_EN    (TIM_CCER_CC2NE|CC2_POL) //CC2低边使能	  

#define CC3_HS_EN 	 (TIM_CCER_CC3E|CC3_POL)  //CC3高边使能     
#define CC3_LS_EN 	 (TIM_CCER_CC3NE|CC3_POL) //CC3低边使能	 

void ToPwmValue(uint16_t PwmDuty);
void turn_off(void);
void MOS_Electronic_Brake(uint8_t FunInside_over_cur_flag);
void Motor_Rotor_Detect(void);
void Hid_Pulse_Logic(uint8_t temp);
void Find_Motor_Rotor_Position(void);
void sort(uint16_t array[],uint8_t m);
void Sure_CutIn_OpenLoop_MOS_Number(uint8_t temp);
void Hit_pulse_find_position_if_successful_function(void);
void Find_FirstMax_SecondMax_Value(uint16_t array[],uint8_t m);

extern void BLDC_NoHallSystem_Change_Phanse_Logic(uint8_t FunInside_over_cur_flag,uint8_t NoHallNumber);

#endif
