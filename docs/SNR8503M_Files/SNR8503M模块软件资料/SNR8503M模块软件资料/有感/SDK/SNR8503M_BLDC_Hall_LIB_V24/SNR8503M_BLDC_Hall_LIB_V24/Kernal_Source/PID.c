/*-------------------- Includes -----------------------*/
#include "PID.h"
#include "global_variable.h"
#include "MC_Parameter.h"
#include "motor_structure.h"
#include "M1_StateMachine.h"
#include "MC_Drive.h"
#include "SpeedRamp.h"

/*------------------- Private variables ---------------*/
tPIParm PIParmIevg;                //均值电流调节PI结构体变量 
tPIParm PIParmS;                   //速度调节PI结构体变量

//////////////////////////////////////////////////////////////////////////

//函数名称:  InitPI()初始化PI

//函数功能说明: 初始化输入输出配置变量

//输入参数：

//输出参数：

//调用函数：无

//完成时间:2022-02-28

//作者: Danny

///////////////////////////////////////////////////////////////
void InitPI(void)
{
	PIParmIevg.qOutMax    = MAX_PWM_DUTY;
	PIParmIevg.qOutMin    = MIN_PWM_DUTY;
	PIParmIevg.qKp        = IevgSum_Kp ;
	PIParmIevg.qKi        = IevgSum_Ki ;
	PIParmIevg.qKc        = IevgSum_Kc ;
	PIParmIevg.qdSum      = 0;
	PIParmIevg.qInMeas    = 0;
	PIParmIevg.qOut       = MIN_PWM_DUTY;
	
	PIParmS.qOutMax    = MAX_PWM_DUTY;
	PIParmS.qOutMin    = MIN_PWM_DUTY;
	PIParmS.qKp        = SSum_Kp ;
	PIParmS.qKi        = SSum_Ki ;
	PIParmS.qKc        = SSum_Kc ;
	PIParmS.qdSum      = 0;
	PIParmS.qInMeas    = 0;
	PIParmS.qOut       = MIN_PWM_DUTY;
}

//////////////////////////////////////////////////////////////////////////

//函数名称:  CalcPI( tPIParm *pParm)PI环调节

//函数功能说明: 用于电机速度环的闭环控制

//输入参数：pParm->qInMeas参考输入 pParm->qInRef测量输入

//输出参数：PIParmI.qOut PI调节输出

//调用函数：无

//完成时间:2022-02-28

//作者: Danny

///////////////////////////////////////////////////////////////

void CalcPI( tPIParm *pParm)
{
	signed int   Error;
	signed int   Exc;
	signed long  U;
	Error =  pParm->qInRef - pParm->qInMeas;
	U =  pParm->qdSum + pParm-> qKp * Error;
	U  >>= 15;
	if( U  >  pParm-> qOutMax)
	{
		pParm->qOut = pParm-> qOutMax;
	}
	else if( U  <  pParm-> qOutMin)
	{
		pParm->qOut = pParm-> qOutMin;
	}
	else
	{
		pParm->qOut  =  U;
	}
	Exc = U - pParm->qOut;
	pParm->qdSum +=  pParm-> qKi* Error - pParm-> qKc * Exc;
}




