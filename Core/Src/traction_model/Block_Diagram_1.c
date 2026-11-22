/*
 * File: Block_Diagram_1.c
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

#include "Block_Diagram_1.h"
#include <math.h>
#include "rtwtypes.h"

/* External inputs (root inport signals with default storage) */
ExtU_Block_Diagram_1_T Block_Diagram_1_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_Block_Diagram_1_T Block_Diagram_1_Y;

/* Real-time model */
static RT_MODEL_Block_Diagram_1_T Block_Diagram_1_M_;
RT_MODEL_Block_Diagram_1_T *const Block_Diagram_1_M = &Block_Diagram_1_M_;

/* Model step function */
void Block_Diagram_1_step(void)
{
  real_T rtb_Add3;
  real_T rtb_Max1_tmp;
  real_T rtb_Max1_tmp_0;
  real_T rtb_u220NmClamp;
  real_T tmp;
  real_T u0;

  /* Gain: '<Root>/Front Loaded//Effective Tire Radius' incorporates:
   *  Gain: '<Root>/Gain'
   *  Inport: '<Root>/wheel_fl_rads'
   *  Inport: '<Root>/wheel_fr_rads'
   *  Sum: '<Root>/Sum'
   */
  rtb_Add3 = (Block_Diagram_1_U.wheel_fl_rads + Block_Diagram_1_U.wheel_fr_rads)
    * 0.5 * 0.2;

  /* Saturate: '<Root>/Front_Speed_Min' */
  if (rtb_Add3 <= 0.5) {
    rtb_u220NmClamp = 0.5;
  } else {
    rtb_u220NmClamp = rtb_Add3;
  }

  /* End of Saturate: '<Root>/Front_Speed_Min' */

  /* Gain: '<Root>/RL Loaded//Effective Tire Radius' incorporates:
   *  Gain: '<Root>/RL Loaded//Effective Tire Radius1'
   *  Inport: '<Root>/wheel_rl_rads'
   */
  rtb_Max1_tmp = 0.2 * Block_Diagram_1_U.wheel_rl_rads;

  /* Gain: '<Root>/RR Loaded//Effective Tire Radius' incorporates:
   *  Gain: '<Root>/RR Loaded//Effective Tire Radius1'
   *  Inport: '<Root>/wheel_rr_rads'
   */
  rtb_Max1_tmp_0 = 0.2 * Block_Diagram_1_U.wheel_rr_rads;

  /* MinMax: '<Root>/Max1' incorporates:
   *  Gain: '<Root>/RL Loaded//Effective Tire Radius'
   *  Gain: '<Root>/RR Loaded//Effective Tire Radius'
   *  MinMax: '<Root>/Max'
   *  Product: '<Root>/Divide'
   *  Product: '<Root>/Divide1'
   *  Sum: '<Root>/Add'
   *  Sum: '<Root>/Add1'
   */
  Block_Diagram_1_Y.CurrentSlipRatio = fmax(rtb_Max1_tmp / rtb_u220NmClamp - 1.0,
    rtb_Max1_tmp_0 / rtb_u220NmClamp - 1.0);

  /* Saturate: '<Root>/0-220Nm Clamp' incorporates:
   *  Inport: '<Root>/driver_nm'
   */
  if (Block_Diagram_1_U.driver_nm > 220.0) {
    rtb_u220NmClamp = 220.0;
  } else if (Block_Diagram_1_U.driver_nm < 0.0) {
    rtb_u220NmClamp = 0.0;
  } else {
    rtb_u220NmClamp = Block_Diagram_1_U.driver_nm;
  }

  /* End of Saturate: '<Root>/0-220Nm Clamp' */

  /* Gain: '<Root>/P Gain' incorporates:
   *  Constant: '<Root>/Target Slip Ratio'
   *  Sum: '<Root>/Sum1'
   */
  u0 = (0.136 - Block_Diagram_1_Y.CurrentSlipRatio) * -5.0;

  /* Saturate: '<Root>/0-1 Clamp' */
  if (u0 > 1.0) {
    tmp = 1.0;
  } else if (u0 < 0.0) {
    tmp = 0.0;
  } else {
    tmp = u0;
  }

  /* Outport: '<Root>/Torque_Command' incorporates:
   *  Constant: '<Root>/Constant2'
   *  MinMax: '<Root>/Min'
   *  Product: '<Root>/Product'
   *  Saturate: '<Root>/0-1 Clamp'
   *  Sum: '<Root>/Sum2'
   */
  Block_Diagram_1_Y.Torque_Command = fmin(rtb_u220NmClamp, (1.0 - tmp) *
    rtb_u220NmClamp);

  /* Product: '<Root>/Divide2' incorporates:
   *  Constant: '<Root>/Start Transition Speed'
   *  Sum: '<Root>/Sum9'
   */
  rtb_u220NmClamp = (rtb_Add3 - 1.0) / Block_Diagram_1_ConstB.Sum10;

  /* Saturate: '<Root>/Transition_Clamp' */
  if (rtb_u220NmClamp > 1.0) {
    rtb_u220NmClamp = 1.0;
  } else if (rtb_u220NmClamp < 0.0) {
    rtb_u220NmClamp = 0.0;
  }

  /* End of Saturate: '<Root>/Transition_Clamp' */

  /* MinMax: '<Root>/Max1' incorporates:
   *  Sum: '<Root>/Add2'
   *  Sum: '<Root>/Add3'
   */
  Block_Diagram_1_Y.CurrentSlipSpeed = fmax(rtb_Max1_tmp - rtb_Add3,
    rtb_Max1_tmp_0 - rtb_Add3);

  /* Sum: '<Root>/Sum12' incorporates:
   *  Constant: '<Root>/Constant6'
   *  Constant: '<Root>/Target SS'
   *  Gain: '<Root>/SR P Gain'
   *  Gain: '<Root>/SS P Gain'
   *  Product: '<Root>/Divide3'
   *  Product: '<Root>/Divide4'
   *  Sum: '<Root>/Sum11'
   *  Sum: '<Root>/Sum7'
   */
  u0 = -(1.0 - Block_Diagram_1_Y.CurrentSlipSpeed) * (1.0 - rtb_u220NmClamp) +
    u0 * rtb_u220NmClamp;

  /* Saturate: '<Root>/Clamp (0-1)' */
  if (u0 > 1.0) {
    u0 = 1.0;
  } else if (u0 < 0.0) {
    u0 = 0.0;
  }

  /* Outport: '<Root>/Torque_Command [Nm]' incorporates:
   *  Constant: '<Root>/Constant7'
   *  Inport: '<Root>/driver_nm'
   *  MinMax: '<Root>/Min2'
   *  Product: '<Root>/Divide5'
   *  Saturate: '<Root>/Clamp (0-1)'
   *  Sum: '<Root>/Sum13'
   */
  Block_Diagram_1_Y.Torque_CommandNm = fmin(Block_Diagram_1_U.driver_nm, (1.0 -
    u0) * Block_Diagram_1_U.driver_nm);
}

/* Model initialize function */
void Block_Diagram_1_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void Block_Diagram_1_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
