#ifndef __Commonfunc_h
#define __Commonfunc_h

#include "basic.h"

typedef  int8_t   s8;
typedef  int16_t  s16;
typedef  int32_t  s32;

typedef  uint8_t  u8;
typedef  uint16_t u16;
typedef  uint32_t u32;

typedef union
{
  struct
	{
		 u8 data0;
		 u8 data1;
		 u8 data2;
		 u8 data3;
	}byte;
	struct
	{
		 u16 data0;
		 u16 data1;
	}halfword;
	u32  word;
}Word_Type;

typedef union
{
  struct
	{
		 u8 data0;
		 u8 data1;
	}byte;
	u16  halfword;
}HalfWord_Type;

#define MAX_U16_DATA  (u16)65535
//----------------取非负值-------------------//
#define ABS(X)         ( ( (X) >= 0 ) ? (X) : -(X) ) 
//----------------舍去负值-------------------//
#define ABL(X)         ( ( (X) >= 0 ) ? (X) : 0 )
//----------------参数限幅-------------------//
#define Limit(x,LT,HT) ( (x) > (HT) ) ? (HT) : ( ( (x) < (LT) ) ? (LT) : (x) )
//----------------取最大值-------------------//
#define Max(x,y,z)     ( (x) >= (y) ) ? ( ( (x) >= (z) ) ? (x) : (z) ) : ( ( (y) >= (z) ) ? (y) : (z) )
//----------------取最小值-------------------//
#define Min(x,y,z)     ( (x) <= (y) ) ? ( ( (x) <= (z) ) ? (x) : (z) ) : ( ( (y) <= (z) ) ? (y) : (z) )
//----------------x 以y为目标，以最大为z步进趋近，不达到z则直接设为y-------------------//
#define Adj(x,y,z)     ( (x) < (y - z) ) ? (x + z) : ( ( (x) > (y + z) ) ? (x - z) : (y) )

#define Q15(Float_Value)	\
			((Float_Value < 0.0) ? (s16)(32768 * (Float_Value) - 0.5) \
			: (s16)(32767 * (Float_Value) + 0.5))

#endif
