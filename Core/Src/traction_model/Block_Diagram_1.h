/*
 * File: Block_Diagram_1.h
 *
 * Code generated for Simulink model 'Block_Diagram_1'.
 *
 * Model version                  : 2.0
 * Simulink Coder version         : 26.2 (R2026b) 22-May-2026
 * C/C++ source code generated on : Sun Nov 22 22:34:14 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef Block_Diagram_1_h_
#define Block_Diagram_1_h_
#ifndef Block_Diagram_1_COMMON_INCLUDES_
#define Block_Diagram_1_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* Block_Diagram_1_COMMON_INCLUDES_ */

#include "Block_Diagram_1_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Invariant block signals (default storage) */
typedef struct {
  const real_T Sum10;                  /* '<Root>/Sum10' */
} ConstB_Block_Diagram_1_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T wheel_fl_rads;                /* '<Root>/wheel_fl_rads' */
  real_T wheel_fr_rads;                /* '<Root>/wheel_fr_rads' */
  real_T wheel_rl_rads;                /* '<Root>/wheel_rl_rads' */
  real_T wheel_rr_rads;                /* '<Root>/wheel_rr_rads' */
  real_T driver_nm;                    /* '<Root>/driver_nm' */
} ExtU_Block_Diagram_1_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T CurrentSlipRatio;             /* '<Root>/Current Slip Ratio' */
  real_T CurrentSlipSpeed;             /* '<Root>/Current Slip Speed' */
  real_T Torque_Command;               /* '<Root>/Torque_Command' */
  real_T Torque_CommandNm;             /* '<Root>/Torque_Command [Nm]' */
} ExtY_Block_Diagram_1_T;

/* Real-time Model Data Structure */
struct tag_RTM_Block_Diagram_1_T {
  const char_T * volatile errorStatus;
};

/* External inputs (root inport signals with default storage) */
extern ExtU_Block_Diagram_1_T Block_Diagram_1_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_Block_Diagram_1_T Block_Diagram_1_Y;
extern const ConstB_Block_Diagram_1_T Block_Diagram_1_ConstB;/* constant block i/o */

/* Model entry point functions */
extern void Block_Diagram_1_initialize(void);
extern void Block_Diagram_1_step(void);
extern void Block_Diagram_1_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Block_Diagram_1_T *const Block_Diagram_1_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Block_Diagram_1'
 */
#endif                                 /* Block_Diagram_1_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
