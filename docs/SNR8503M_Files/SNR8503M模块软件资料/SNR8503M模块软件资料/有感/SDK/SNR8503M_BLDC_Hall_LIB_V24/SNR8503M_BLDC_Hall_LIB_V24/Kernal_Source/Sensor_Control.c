/*-------------------- Includes -----------------------*/
#include "MC_Drive.h"
#include "Global_Variable.h"
#include "motor_structure.h"
#include "M1_StateMachine.h"
#include "SpeedRamp.h"
#include "MC_Parameter.h"
#include "SNR_BLDC_HALL_V1p0.h"

#if (Hall_Degree_60_or_120 == Degree60)
															 //0 1 2 3 4 5 6 7				
uint8_t Motor_step_tab_cw[8] 	= {4,0,8,1,6,8,7,3};		// 0、4、6、7、3、1
uint8_t Motor_step_tab_ccw[8] = {1,3,8,7,0,8,4,6};		// 0、1、3、7、6、4

#else
															 //0 1 2 3 4 5 6 7
uint8_t Motor_step_tab_cw[8]  = {0,5,3,1,6,4,2,0};
uint8_t Motor_step_tab_ccw[8] = {0,3,6,2,5,1,4,0};

#endif

/*------------------- Private variables ---------------*/
void BLDC_Sensor_control(unsigned char Hall_value);
void BLDC_Sensor_Judge(unsigned char Hall_value);

/*------------------ Private functions ----------------*/
void BLDC_Sensor_control(unsigned char Hall_value)
{
    MCPWM_PRT = 0x0000DEAD;
		SYS_WR_PROTECT = 0x7a83;

	#if (Hall_Degree_60_or_120 == Degree60)
    switch(Hall_value) 
		{
			case 5: 
					// A相PWM，B相悬空 C ON；检测上升沿过零事件
					// A pwm | B float | C ON
					MCPWM_IO01 = DRIVER_POLARITY | (0x0c04);		// 
					MCPWM_IO23 = DRIVER_POLARITY | 0x1c;	
					FG_ONOFF;
			break;
			
			
			case 7:
					// A相悬空，B相PWM C相ON ；检测下降沿过零事件
					// A float | B PWM | C ON
					MCPWM_IO01 = DRIVER_POLARITY | (0x040c);		
					MCPWM_IO23 = DRIVER_POLARITY | 0x1c;			
			break;
				
				
			case 6: 
					// A相ON，B相PWM C相悬空 ；检测上升沿过零事件
					// A ON | B PWM | C float
					MCPWM_IO01 = DRIVER_POLARITY | 0x041c;      	// 
					MCPWM_IO23 = DRIVER_POLARITY | (0x0c);
			break;
				
			
			case 2: 
					// A相ON，B相悬空 C相PWM ；检测下降沿过零事件
					// A ON | B float | C PWM
					MCPWM_IO01 = DRIVER_POLARITY | (0x0c1c);		// 
					MCPWM_IO23 = DRIVER_POLARITY | 0x04;
					FG_ONOFF;
			break;
				
			
			case 0: 
					// A相悬空，B相ON C相PWM  ；检测上升沿过零事件
					// A float | B ON | C PWM
					MCPWM_IO01 = DRIVER_POLARITY | (0x1c0c);		// 
					MCPWM_IO23 = DRIVER_POLARITY | 0x04;	
			break;
				
			
			case 1: 
					// A相PWM，B相ON C相悬空  ；检测下降沿过零事件
					// A PWM | B on | C float
					MCPWM_IO01 = DRIVER_POLARITY | 0x1c04;			// 
					MCPWM_IO23 = DRIVER_POLARITY | (0x0c);		
			break;
		}
	
	#else
    switch(Hall_value) 
		{
			case 5: 
					// A相PWM，B相悬空 C ON；检测上升沿过零事件
					// A pwm | B float | C ON
					MCPWM_IO01 = DRIVER_POLARITY | (0x0c04);		// 
					MCPWM_IO23 = DRIVER_POLARITY | 0x1c;	
					FG_ONOFF;
			break;
			
			
			case 4:
					// A相悬空，B相PWM C相ON ；检测下降沿过零事件
					// A float | B PWM | C ON
					MCPWM_IO01 = DRIVER_POLARITY | (0x040c);		
					MCPWM_IO23 = DRIVER_POLARITY | 0x1c;			
			break;
				
				
			case 6: 
					// A相ON，B相PWM C相悬空 ；检测上升沿过零事件
					// A ON | B PWM | C float
					MCPWM_IO01 = DRIVER_POLARITY | 0x041c;      	// 
					MCPWM_IO23 = DRIVER_POLARITY | (0x0c);
			break;
				
			
			case 2: 
					// A相ON，B相悬空 C相PWM ；检测下降沿过零事件
					// A ON | B float | C PWM
					MCPWM_IO01 = DRIVER_POLARITY | (0x0c1c);		// 
					MCPWM_IO23 = DRIVER_POLARITY | 0x04;
					FG_ONOFF;
			break;
				
			
			case 3: 
					// A相悬空，B相ON C相PWM  ；检测上升沿过零事件
					// A float | B ON | C PWM
					MCPWM_IO01 = DRIVER_POLARITY | (0x1c0c);		// 
					MCPWM_IO23 = DRIVER_POLARITY | 0x04;	
			break;
				
			
			case 1: 
					// A相PWM，B相ON C相悬空  ；检测下降沿过零事件
					// A PWM | B on | C float
					MCPWM_IO01 = DRIVER_POLARITY | 0x1c04;			// 
					MCPWM_IO23 = DRIVER_POLARITY | (0x0c);		
			break;
		}
		#endif
		
		MCPWM_PRT = 0x0000CAFE;
		SYS_WR_PROTECT = 0;
}

void BLDC_Sensor_Judge(unsigned char Hall_value)
{
	if(Hall_value != Motor.BLDC.u8HallValueOld)
	{
			if(Motor.Control.u16Phase_Compensation_cnt < 200)
			{
				if(Motor.Control.u32MotorRAMP_OK_flag == 1)	// 启动完成后开始计数
				{
					Motor.Control.u16Phase_Compensation_cnt++;
				}
			}
			
			if(Motor.Control.u8PhaseChange_ON_COMPENSATION_flag == 1)
			{
				Motor.Control.u8PhaseChange_ON_COMPENSATION_flag = 0;
			}
			else
			{
				// 换相
				HALL_LIB.u8HALL_NEW_Value 	= Motor.BLDC.u8HallValueRenew;
				Motor.BLDC.u8HALL_Run_Value = HALL_Update(&HALL_LIB);
				BLDC_Sensor_control(Motor.BLDC.u8HALL_Run_Value);
			}
			// 更新hall信号	
			Motor.BLDC.u8HallValueOld = Motor.BLDC.u8HALL_Run_Value;
	}
}




