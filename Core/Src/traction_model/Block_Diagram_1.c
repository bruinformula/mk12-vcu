/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Block_Diagram_1.c
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
  real_T rtb_Product;
  real_T u0;

  /* Gain: '<Root>/SR P Gain' incorporates:
   *  Constant: '<Root>/Target SR'
   *  Inport: '<Root>/In3'
   *  Sum: '<Root>/Sum6'
   */
  rtb_Product = (0.136 - Block_Diagram_1_U.In3) * -5.0;

  /* Sum: '<Root>/Sum12' incorporates:
   *  Constant: '<Root>/Target SS'
   *  Gain: '<Root>/SS P Gain'
   *  Inport: '<Root>/In4'
   *  Product: '<Root>/Divide3'
   *  Product: '<Root>/Divide4'
   *  Sum: '<Root>/Sum7'
   */
  u0 = -(1.0 - Block_Diagram_1_U.In4) * Block_Diagram_1_ConstB.Sum11[0] +
    rtb_Product;

  /* Saturate: '<Root>/Clamp (0-1)' */
  if (u0 > 1.0) {
    u0 = 1.0;
  } else if (u0 < 0.0) {
    u0 = 0.0;
  }

  /* Outport: '<Root>/Torque_Command [Nm]' incorporates:
   *  Constant: '<Root>/Constant7'
   *  Constant: '<Root>/Torque_DriverDemand [Nm]'
   *  MinMax: '<Root>/Min2'
   *  Product: '<Root>/Divide5'
   *  Saturate: '<Root>/Clamp (0-1)'
   *  Sum: '<Root>/Sum13'
   */
  Block_Diagram_1_Y.Torque_CommandNm[0] = fmin(220.0, (1.0 - u0) * 220.0);

  /* Sum: '<Root>/Sum12' incorporates:
   *  Constant: '<Root>/Target SS'
   *  Gain: '<Root>/SS P Gain'
   *  Inport: '<Root>/In4'
   *  Product: '<Root>/Divide3'
   *  Product: '<Root>/Divide4'
   *  Sum: '<Root>/Sum7'
   */
  u0 = -(1.0 - Block_Diagram_1_U.In4) * Block_Diagram_1_ConstB.Sum11[1] +
    rtb_Product * Block_Diagram_1_ConstB.Divide2;

  /* Saturate: '<Root>/Clamp (0-1)' */
  if (u0 > 1.0) {
    u0 = 1.0;
  } else if (u0 < 0.0) {
    u0 = 0.0;
  }

  /* Outport: '<Root>/Torque_Command [Nm]' incorporates:
   *  Constant: '<Root>/Constant7'
   *  Constant: '<Root>/Torque_DriverDemand [Nm]'
   *  MinMax: '<Root>/Min2'
   *  Product: '<Root>/Divide5'
   *  Saturate: '<Root>/Clamp (0-1)'
   *  Sum: '<Root>/Sum13'
   */
  Block_Diagram_1_Y.Torque_CommandNm[1] = fmin(220.0, (1.0 - u0) * 220.0);

  /* Sum: '<Root>/Sum12' incorporates:
   *  Constant: '<Root>/Constant5'
   *  Constant: '<Root>/Target SS'
   *  Gain: '<Root>/SS P Gain'
   *  Inport: '<Root>/In4'
   *  Product: '<Root>/Divide3'
   *  Product: '<Root>/Divide4'
   *  Sum: '<Root>/Sum7'
   */
  u0 = -(1.0 - Block_Diagram_1_U.In4) * Block_Diagram_1_ConstB.Sum11[2] +
    rtb_Product * 0.0;

  /* Saturate: '<Root>/Clamp (0-1)' */
  if (u0 > 1.0) {
    u0 = 1.0;
  } else if (u0 < 0.0) {
    u0 = 0.0;
  }

  /* Outport: '<Root>/Torque_Command [Nm]' incorporates:
   *  Constant: '<Root>/Constant7'
   *  Constant: '<Root>/Torque_DriverDemand [Nm]'
   *  MinMax: '<Root>/Min2'
   *  Product: '<Root>/Divide5'
   *  Saturate: '<Root>/Clamp (0-1)'
   *  Sum: '<Root>/Sum13'
   */
  Block_Diagram_1_Y.Torque_CommandNm[2] = fmin(220.0, (1.0 - u0) * 220.0);

  /* Gain: '<Root>/P Gain' incorporates:
   *  Constant: '<Root>/Target Slip Ratio'
   *  Inport: '<Root>/In1'
   *  Sum: '<Root>/Sum1'
   */
  u0 = (0.136 - Block_Diagram_1_U.In1) * -5.0;

  /* Saturate: '<Root>/0-1 Clamp' */
  if (u0 > 1.0) {
    u0 = 1.0;
  } else if (u0 < 1.0) {
    u0 = 1.0;
  }

  /* Outport: '<Root>/Torque_Command' incorporates:
   *  Constant: '<Root>/Constant2'
   *  MinMax: '<Root>/Min'
   *  Product: '<Root>/Product'
   *  Saturate: '<Root>/0-1 Clamp'
   *  Sum: '<Root>/Sum2'
   */
  Block_Diagram_1_Y.Torque_Command = fmin(Block_Diagram_1_ConstB.u220NmClamp,
    (1.0 - u0) * Block_Diagram_1_ConstB.u220NmClamp);
}

/* Model initialize function */
void Block_Diagram_1_initialize(void)
{
  /* ConstCode for Outport: '<Root>/Current Slip Ratio' */
  Block_Diagram_1_Y.CurrentSlipRatio = Block_Diagram_1_ConstB.Max;

  /* ConstCode for Outport: '<Root>/Current Slip Speed' */
  Block_Diagram_1_Y.CurrentSlipSpeed = Block_Diagram_1_ConstB.Max1;
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
