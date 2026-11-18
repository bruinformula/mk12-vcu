/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Block_Diagram_1.h
 *
 * Code generated for Simulink model 'Block_Diagram_1'.
 *
 * Model version                  : 1.17
 * Simulink Coder version         : 26.1 (R2026a) 20-Nov-2025
 * C/C++ source code generated on : Tue Nov 17 14:35:57 2026
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
  const real_T RLLoadedEffectiveTireRadius;
                                 /* '<Root>/RL Loaded//Effective Tire Radius' */
  const real_T Sum;                    /* '<Root>/Sum' */
  const real_T Gain;                   /* '<Root>/Gain' */
  const real_T FrontLoadedEffectiveTireRad;
                              /* '<Root>/Front Loaded//Effective Tire Radius' */
  const real_T Divide;                 /* '<Root>/Divide' */
  const real_T Add;                    /* '<Root>/Add' */
  const real_T RRLoadedEffectiveTireRadius;
                                 /* '<Root>/RR Loaded//Effective Tire Radius' */
  const real_T Divide1;                /* '<Root>/Divide1' */
  const real_T Add1;                   /* '<Root>/Add1' */
  const real_T Max;                    /* '<Root>/Max' */
  const real_T RLLoadedEffectiveTireRadi_l;
                                /* '<Root>/RL Loaded//Effective Tire Radius1' */
  const real_T Add2;                   /* '<Root>/Add2' */
  const real_T RRLoadedEffectiveTireRadi_c;
                                /* '<Root>/RR Loaded//Effective Tire Radius1' */
  const real_T Add3;                   /* '<Root>/Add3' */
  const real_T Max1;                   /* '<Root>/Max1' */
  const real_T Sum9;                   /* '<Root>/Sum9' */
  const real_T Sum10;                  /* '<Root>/Sum10' */
  const real_T Divide2;                /* '<Root>/Divide2' */
  const real_T Sum11[3];               /* '<Root>/Sum11' */
  const real_T u220NmClamp;            /* '<Root>/0-220Nm Clamp' */
} ConstB_Block_Diagram_1_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T In3;                          /* '<Root>/In3' */
  real_T In4;                          /* '<Root>/In4' */
  real_T In1;                          /* '<Root>/In1' */
} ExtU_Block_Diagram_1_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T CurrentSlipRatio;             /* '<Root>/Current Slip Ratio' */
  real_T CurrentSlipSpeed;             /* '<Root>/Current Slip Speed' */
  real_T Torque_Command;               /* '<Root>/Torque_Command' */
  real_T Torque_CommandNm[3];          /* '<Root>/Torque_Command [Nm]' */
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
