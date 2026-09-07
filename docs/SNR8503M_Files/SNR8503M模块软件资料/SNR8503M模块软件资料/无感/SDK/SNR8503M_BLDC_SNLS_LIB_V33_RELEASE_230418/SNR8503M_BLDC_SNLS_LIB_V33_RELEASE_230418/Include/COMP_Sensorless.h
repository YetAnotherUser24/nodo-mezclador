#ifndef __COMP_Sensorless_H
#define __COMP_Sensorless_H

#include "motor_structure.h"
#include "Lib_Zero_detect.h"

/***********电机运动状态标识*************/
typedef enum {
		MOTOR_READY 					= 0, 
		MOTOR_ROTOR_DETECT 		= 1,
		MOTOR_ROTOR_ALIGN 		= 2,
		MOTOR_RUN 						= 3
}MOTOR_RUN_STATUS;

extern	void COMP_Motor_Zero_Detect(CMP_Zero_TypeDef *Motor);
extern	void Freerun_Zero_Detect(CMP_Zero_TypeDef *Motor);
extern	void CMP_SNS_Motor_AutoCommutation(void);
extern	void CMP_SNS_Motor_Align(void);
extern	void Motor_Freerun_Detect(void);
extern	void INIT_FreerunDetect(void);
#endif

