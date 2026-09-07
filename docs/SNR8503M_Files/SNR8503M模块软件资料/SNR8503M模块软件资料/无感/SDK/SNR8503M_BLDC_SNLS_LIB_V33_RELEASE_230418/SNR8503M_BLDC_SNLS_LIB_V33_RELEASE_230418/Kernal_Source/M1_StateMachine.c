#include "M1_StateMachine.h"
#include "basic.h"
#include "global_variable.h"
#include "absacc.h"
#include "MC_Parameter.h"
#include "motor_structure.h"
#include "COMP_Sensorless.h"
#include "MC_Drive.h"
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void M1M1_Fault(void);
void M1M1_Init (void);
void M1M1_Stop (void);
void M1M1_Run  (void);

static void M1M1_SwitchFaultStop(void);
static void M1M1_SwitchInitStop(void);
static void M1M1_SwitchStopRun(void);
static void M1M1_SwitchRunStop(void);

static void M1M1_RunCalib (void);
static void M1M1_RunReady (void);
static void M1M1_RunAlign (void);
static void M1M1_RunStartup (void);
static void M1M1_RunSpin (void);
static void M1M1_RunFreerun_Det (void);
static void M1M1_RunBrake (void);
static void M1M1_SwitchRunCalibReady(void);
static void M1M1_SwitchRunReadyFreerun_Det(void);
static void M1M1_SwitchRunReadyAlign(void);
static void M1M1_SwitchRunAlignStartup(void);
static void M1M1_SwitchRunStartupSpin(void);

extern uint8_t Motor_step_tab_cw[6];
extern uint8_t Motor_step_tab_ccw[6];
extern uint8_t phase_rise_tab_cw[6];
extern uint8_t phase_rise_tab_ccw[6];

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief M1 structure */
m1m1_app_mainstate_t  e1M1_MainState = M1_MainState_Init;                 /* Main State */
m1m1_run_substate_t   e1M1_RunSubState = M1_RunState_Calib;								/* Run Sub State */

/*! @brief Application sub-state function field  */
const pfcn_void_void sM1_STATE[4] = {M1M1_Fault, M1M1_Init, M1M1_Stop, M1M1_Run};

/*! @brief Application sub-state function field  */
static const pfcn_void_void s_M1_STATE_RUN_TABLE[7] = {  	M1M1_RunCalib, 				M1M1_RunReady,
																													M1M1_RunFreerun_Det,	M1M1_RunAlign, 
																													M1M1_RunStartup,			M1M1_RunSpin,   
																													M1M1_RunBrake
                                                      };

/*******************************************************************************
* Code
******************************************************************************/

/*!
 * @brief Fault state called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
void M1M1_Fault(void)
{
		#if EN_BRAKE
				BLDC_Brake();
		#else
				BLDC_Stop();			
		#endif
		if(Motor_1M1st.BLDC.u8FlagEnMotorRun == 0)		// Speed command = 0; 解除非电压、过温保护、直流偏置故障、MOS自检故障、缺相故障
		{
			User_sys.BLDC_SensorlessCtr.sys_error_flg &= ~(SHORT_ERROR | BLOCK_ERROR | OVER_LOAD_ERROR);
		}				
		if(User_sys.BLDC_SensorlessCtr.sys_error_flg == 0)
		{
			M1M1_SwitchFaultStop();		
		}
}

/*!
 * @brief State initialization routine called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1M1_Init(void)
{
    variable_reset();				// Reset motor's variable
    M1M1_SwitchInitStop();
}


/*!
 * @brief Stop state routine called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1M1_Stop(void)
{	
		variable_reset();					// Reset motor's variable
    if( Motor_1M1st.BLDC.u8FlagEnMotorRun == 1)
    {
        M1M1_SwitchStopRun();
    }
}

/*!
 * @brief Run state routine called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1M1_Run(void)
{
    s_M1_STATE_RUN_TABLE[e1M1_RunSubState]();			// Motor sub-state control
}


/*!
 * @brief Switch from Fault to Stop state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1M1_SwitchFaultStop(void)
{
    /* Type the code to do when going from the FAULT to the INIT state */
    e1M1_MainState = M1_MainState_Stop;
}


/*!
 * @brief Switch from Init to Stop state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1M1_SwitchInitStop(void)
{
    /* type the code to do when going from the INIT to the FAULT state */
    e1M1_MainState = M1_MainState_Stop;
}

/*!
 * @brief Switch from Stop to Run state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1M1_SwitchStopRun(void)
{
    /* type the code to do when going from the STOP to the RUN state */

    e1M1_RunSubState = M1_RunState_Calib;
    e1M1_MainState = M1_MainState_Run;
}


/*!
 * @brief Switch from Run to Stop state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1M1_SwitchRunStop(void)
{
    /* type the code to do when going from the RUN to the STOP state */
	
		#if EN_BRAKE
				BLDC_Brake();
		#else
				BLDC_Stop();			
		#endif
	
    e1M1_MainState  =   M1_MainState_Stop ;
    e1M1_RunSubState = M1_RunState_Calib;
}

/*!
 * @brief Calibration process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1M1_RunCalib (void)
{
    M1M1_SwitchRunCalibReady();
}

/*!
 * @brief Ready state called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1M1_RunReady (void)
{
	#if	EN_BRAKE
		M1M1_SwitchRunReadyAlign();
	#else
		#if EN_MOTOR_FREERUN_DETECT
			INIT_FreerunDetect();
			M1M1_SwitchRunReadyFreerun_Det();
		#else
			M1M1_SwitchRunReadyAlign();
		#endif
	#endif
}

/*!
 * @brief Alignment process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1M1_RunAlign (void)
{
		/** Initialize Motor startup PWM duty */
    M1M1_SwitchRunAlignStartup();
}

/*!
 * @brief Start-up process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1M1_RunStartup (void)
{
		M1M1_SwitchRunStartupSpin();
}

/*!
 * @brief Spin state called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1M1_RunSpin (void)
{
		if(Motor_1M1st.BLDC.u8FlagEnMotorRun == 0)
    {
        M1M1_SwitchRunStop();
    }
}


/*!
 * @brief Free-wheel process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1M1_RunFreerun_Det (void)
{
	if(Motor_1M1st.BLDC.u8FlagEnMotorRun == 0)
	{
			M1M1_SwitchRunStop();
	}
}


/*!
 * @brief Brake process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1M1_RunBrake (void)
{
		BLDC_Brake();
	
		// 刹车延时
		if(Motor_1M1st.BLDC_CMP.u8FlagDetectMotorFreerunAgainstwind == 1)
		{
			if(Motor_1M1st.BLDC_CMP.u32MotorFreerunAgainstwindDelay < 100)
			{
				Motor_1M1st.BLDC_CMP.u32MotorFreerunAgainstwindDelay++;
			}
			else
			{
				Motor_1M1st.BLDC_CMP.u32MotorFreerunAgainstwindDelay = 0;
				Motor_1M1st.BLDC_CMP.u8FlagDetectMotorFreerunAgainstwind = 0;
				
				e1M1_RunSubState = M1_RunState_Calib;				
			}
		}
}


/*!
 * @brief Switch from Calib to Ready state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1M1_SwitchRunCalibReady(void)
{
    /* Type the code to do when going from the RUN CALIB to the RUN READY sub-state */
    e1M1_RunSubState = M1_RunState_Ready;
}

/*!
 * @brief Switch from Ready to Freerun detect state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1M1_SwitchRunReadyFreerun_Det(void)
{
    /* Type the code to do when going from the RUN kM1_RunState_Ready to the RUN kM1_RunState_Freerun_Det sub-state */
    e1M1_RunSubState = M1_RunState_Freerun_Det;
}

/*!
 * @brief Switch from Ready to Align state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1M1_SwitchRunReadyAlign(void)
{
    /* Type the code to do when going from the RUN kM1_RunState_Ready to the RUN kM1_RunState_Align sub-state */
    e1M1_RunSubState = M1_RunState_Align;
}

/*!
 * @brief Switch from Align to Startup state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1M1_SwitchRunAlignStartup(void)
{
    /* Type the code to do when going from the RUN kM1_RunState_Align to the RUN kM1_RunState_Startup sub-state */
    e1M1_RunSubState = M1_RunState_Startup;
}

/*!
 * @brief Switch from Startup to Spin state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1M1_SwitchRunStartupSpin(void)
{
    /* Type the code to do when going from the RUN kM1_RunState_Startup to the RUN kM1_RunState_Spin sub-state */
    e1M1_RunSubState = M1_RunState_Spin;
}



