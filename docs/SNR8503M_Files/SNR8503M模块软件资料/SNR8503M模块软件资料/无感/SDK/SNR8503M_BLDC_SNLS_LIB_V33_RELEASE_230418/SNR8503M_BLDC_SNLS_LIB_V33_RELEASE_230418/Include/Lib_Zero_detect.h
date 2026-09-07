/*------------------------------prevent recursive inclusion -------------------*/ 
#ifndef __Lib_Zero_detect_H
#define __Lib_Zero_detect_H

#include "basic.h"

//*---------------------------------------------------------*//
//	该结构体不允许做任何改动, 否则会失控
//*---------------------------------------------------------*//
typedef struct
{
	__IO  uint8_t		u8CMP_Zero_OUT_Value;	
	__IO  uint8_t		u8CMP_Zero_phase_rise;		
	__IO  uint32_t	u32CMP_Zero_speed_cnt;		
	__IO  uint32_t	u32CMP_Zero_phase_max;
	__IO  uint32_t	u32CMP_Zero_phase_min;
	
	__IO  uint32_t	u32CMP_Freerunstep_0_Mincnt;
	__IO  uint32_t	u32CMP_Freerunstep_0_cnt;
	__IO  uint32_t	u32CMP_Freerunstep_1_cnt;
	__IO  uint8_t		u8CMP_FreerunstepNew;
} CMP_Zero_TypeDef;
extern	CMP_Zero_TypeDef	CMP_ZeroCross_lib;
//*---------------------------------------------------------*//
//	该结构体不允许做任何改动, 否则会失控
//*---------------------------------------------------------*//
#endif

//-----------------------------------------------------------------------------------------------


