/*-------------------- Includes -----------------------*/
#include "SpeedRamp.h"
#include "Global_Variable.h"
#include "motor_structure.h"

NormalizationType 		RP;
LoopCMP_T 						RPValue;

NormalizationType 		SPEED_Cmd;
LoopCMP_T 						SPEEDValue;

NormalizationType 		Duty_PHASE_COMPENSATION_Cmd;
NormalizationType 		Speed_PHASE_COMPENSATION_Cmd;

/****************************************************************
	函数名：InitNormalization
	描述：电位器归一化参数初始化
	输入：无
	输出：无
****************************************************************/
void InitNormalization(u32 Lowdata,u32 highdata,u32 minout,u32 maxout,NormalizationType *u)
{
	u->MinIn = Lowdata; 			//最低门限
	u->MaxIn = highdata; 			
	u->MinOut = minout;  			
	u->MaxOut = maxout;  			//最大输出值 
	u->Kslope = ((float)u->MaxOut)/(u->MaxIn - u->MinIn);  //斜率	= 0.75
	u->State = 0;
	u->OutEn = 0;
	u->Out = 0;
}

//////////////////////////////////////////////////////////////////////////

//函数名称:u16 CalcGraudNormalizationData(u16 data,u16 Minout,u16 Accelerate,u16 Decelerate,u8 newstate)

//函数功能说明:缓冲调速输出

//输入参数: 

//输出参数: 无

//调用函数: 无

//完成时间:2022-02-28

//作者: Li

///////////////////////////////////////////////////////////////
u16 CalcGraudNormalizationData(u16 data,u16 Minout,u16 Accelerate,u16 Decelerate,u8 newstate)
{
		static u16 DataReturn;	
	
		if(newstate == ENABLE)
		{
			if(data <= Minout)
			{
				data = Minout; 
			}
			if(((DataReturn > data)&&(DataReturn < (data + Decelerate)))
					 ||((DataReturn < data)&&(DataReturn > (data - Accelerate))))
			{
				DataReturn = data;
			}
					
			else if(DataReturn > data)
			{
				DataReturn -= Decelerate;
			}
			else if(DataReturn < data)
			{
				DataReturn += Accelerate;
			}
			else
			{
				DataReturn = data;
			}			
		}
		else
		{
			DataReturn = Minout;
		}

	  return(DataReturn);
}


/****************************************************************
	函数名：CalcNormalization
	描述：电位器归一化计算
	输入：无
	输出：无
****************************************************************/
void CalcNormalization(u32 value,NormalizationType *u)
{
	u->In = value;

	if(u->In < (u->MinIn - 15))		//无效区
	{
		u->State |= 1;
		u->Out = 0;
		
		if(u->State == 3)
		{
			u->State &= 1;
			u->OutEn = 0;
		}
	}
	else if((u->In > u->MinIn))	//有效区
	{
		u->State |= 2;
		u->Out = u->Kslope*(u->In - u->MinIn);		//线性归一化
		
		if(u->Out > u->MaxOut)
		{
			u->Out = u->MaxOut;	
		}
		if(u->Out < u->MinOut)
		{
			u->Out = u->MinOut;	
		}		
		
		if(u->State == 3)
		{
			u->State &= 2;
			u->OutEn = 1;
		}
	}
	else				//回差区状态不变
	{

	}
}

