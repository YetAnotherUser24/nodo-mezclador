
#ifndef __MC_DRIVE_H
#define __MC_DRIVE_H

#include "main.h"

extern u8 CWCCW_bIOInput;	
extern void BLDC_Communication(unsigned char Hall_value);
extern void BLDC_COMP_Input_Only(unsigned char Hall_value);
extern void BLDC_Stop(void);
extern void BLDC_Brake(void);
extern void Motor_LED_DISP(void);
extern void CWCCW_PowerOn(void);
extern void CWCCW_ReadIO(void);
extern u16 PiCurrentLimitPWMDuty(u16 Duty_In,u8 newstate);

#endif
