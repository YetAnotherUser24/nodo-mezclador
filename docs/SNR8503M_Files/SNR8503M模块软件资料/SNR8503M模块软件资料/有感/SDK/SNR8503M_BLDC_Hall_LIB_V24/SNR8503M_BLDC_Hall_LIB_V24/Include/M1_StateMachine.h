/****************************************************************************************
 * @file       StateMachine.h
 * @author   Motor TEAM
 * @brief    This file provides the ITR functions and test samples.
 *
 * @attention
 *
 * THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
 * CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
 * TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
 * HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
 * CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
 *
 * <H2><CENTER>&COPY; COPYRIGHT MINDMOTION </CENTER></H2>
 */

/** Define to prevent recursive inclusion */
#include  "stdint.h"

/* 1 ------------------------------------------------------------------------
 宏函数定义
//-------------------------------------------------------------------------- */


/* 1 end------------------------------------------------------------------ */

/* 2 ------------------------------------------------------------------------
 变量定义
-------------------------------------------------------------------------- */
#ifndef _0_MotorTask_C
#define __VAR_DEFINE  extern
#else
#define __VAR_DEFINE
#endif


#undef __VAR_DEFINE

/* 2 end------------------------------------------------------------------ */

/* 3 ------------------------------------------------------------------------
 函数定义
-------------------------------------------------------------------------- */
#ifndef _0_MotorTask_C
#define __FUN_DEFINE  extern
#else
#define __FUN_DEFINE
#endif

__FUN_DEFINE void MotorTask(void);  // 电机任务1ms

#undef __FUN_DEFINE
/* 3 end------------------------------------------------------------------ */


/*! @brief Pointer to function */
typedef void (*pfcn_void_void)(void);

/*! @brief device fault typedef */
typedef uint16_t mcdef_fault_t;

/*! @brief States of machine enumeration */
typedef enum _m1m1_app_mainstate_t
{
    M1_MainState_Fault = 0,
    M1_MainState_Init  = 1,
    M1_MainState_Stop  = 2,
    M1_MainState_Run   = 3,
} m1m1_app_mainstate_t; /* Main States */


/*! @brief States of machine enumeration */
typedef enum _1m1_run_substate_t
{
    M1_RunState_Calib     	= 0,
    M1_RunState_Ready     	= 1,
    M1_RunState_Freerun_Det	= 2,
		M1_RunState_Align     	= 3,
    M1_RunState_Startup   	= 4,
    M1_RunState_Spin      	= 5,
    M1_RunState_Brake     	= 6,
} m1m1_run_substate_t; /* Run sub-states */


typedef enum _m1m1_run_bldcstate_t
{
    M1_BldcState_Init             = 0,
    M1_BldcState_FreeWhell        = 1,
    M1_BldcState_ZeroCrossDet     = 2,
    M1_BldcState_30DegreeDelay    = 3,
} m1m1_run_bldcstate_t; /* Run sub-states */


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MC_FAULT_I_DCBUS_OVER    0  /* OverCurrent fault flag */
#define MC_FAULT_U_DCBUS_UNDER   1  /* Undervoltage fault flag */
#define MC_FAULT_U_DCBUS_OVER    2  /* Overvoltage fault flag */
#define MC_FAULT_LOAD_OVER       3  /* Overload fault flag */
#define MC_FAULT_SPEED_OVER      4  /* Over speed fault flag */
#define MC_FAULT_ROTOR_BLOCKED   5  /* Blocked rotor fault flag */
#define MC_FAULT_PHASELESS       6 	/* Phasee less fault flag */

/* Sets the fault bit defined by faultid in the faults variable */
#define MC_FAULT_SET(faults, faultid) (faults |= ((mcdef_fault_t)1 << faultid))

/* Clears the fault bit defined by faultid in the faults variable */
#define MC_FAULT_CLEAR(faults, faultid) (faults &= ~((mcdef_fault_t)1 << faultid))

/* Check the fault bit defined by faultid in the faults variable, returns 1 or 0 */
#define MC_FAULT_CHECK(faults, faultid) ((faults & ((mcdef_fault_t)1 << faultid)) >> faultid)

/* Clears all fault bits in the faults variable */
#define MC_FAULT_CLEAR_ALL(faults) (faults = 0)

extern m1m1_run_substate_t   e1M1_RunSubState;
extern m1m1_app_mainstate_t  e1M1_MainState;                 /* Main State */

//extern m1_run_bldcstate_t  eM1_BldcState;
/*! @brief Application sub-state function field - fast */
extern const pfcn_void_void sM1_STATE[4];


