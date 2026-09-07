
#ifndef __SpeedRamp_H
#define __SpeedRamp_H

#include "main.h"

typedef struct
{
	u32 In;             //输入值
	
	u32 MaxIn;          //最大值有效值
	u32 MinIn;          //最小值有效值		
	
	u32 MinOut;					// 
	u32 MaxOut;         //对应允许最大输出值
	
	u32 Out;            //输出值
	u8  OutEn;          //输出使能标志
  u8  State;          //输入状态
	float Kslope;       //对应斜率
}
NormalizationType; //参数归一化的相关变量
extern NormalizationType RP;
extern NormalizationType SPEED_Cmd;
extern NormalizationType Duty_PHASE_COMPENSATION_Cmd;
extern NormalizationType Speed_PHASE_COMPENSATION_Cmd;

typedef struct
{
	s32 Dest;		//参考目标量
	s32 Act;		//实际使用量
	s32 Max;		//最大限制值
	s32 Min;		//最小限制值
	s32 Inc;		//加速步距
	s32 Dec;		//加速步距
}LoopCMP_T; 
extern LoopCMP_T RPValue;
extern LoopCMP_T SPEEDValue;

extern u16 CalcGraudNormalizationData(u16 data,u16 Minout,u16 Accelerate,u16 Decelerate,u8 newstate);
extern void InitNormalization(u32 Lowdata,u32 highdata,u32 minout,u32 maxout,NormalizationType *u);
extern void CalcNormalization(u32 value,NormalizationType *u);

#endif


