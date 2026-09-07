#ifndef PID_h
#define PID_h
#include "commonfunc.h"

typedef struct {
	  signed   int   qdSum;
	  signed   int   qKp;
	  signed   int   qKi;
	  signed   int   qKc;
	  signed   int   qOutMax;
	  signed   int   qOutMin;
	  signed   int   qInRef;
	  signed   int   qInMeas;
	  signed   int   qOut;
} tPIParm;
	
extern tPIParm PIParmIevg;                //均值电流调节PI结构体变量 
extern tPIParm PIParmS;                   //速度调节PI结构体变量

extern void InitPI(void);              //PI初始化程序
extern void CalcPI( tPIParm *pParm);   //PI运算程序


#endif
