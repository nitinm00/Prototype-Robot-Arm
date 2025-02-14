/*
 * ArmReverseKinematics_data.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "ArmReverseKinematics".
 *
 * Model version              : 3.1
 * Simulink Coder version : 23.2 (R2023b) 01-Aug-2023
 * C source code generated on : Wed May 15 02:42:36 2024
 *
 * Target selection: rsim.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "ArmReverseKinematics.h"

/* Block parameters (default storage) */
P rtP = {
  /* Computed Parameter: fromWS_Signal1_Time0
   * Referenced by: '<S8>/fromWS_Signal 1'
   */
  { 0.0, 1.0, 2.0, 3.0, 4.0 },

  /* Computed Parameter: fromWS_Signal1_Data0
   * Referenced by: '<S8>/fromWS_Signal 1'
   */
  { 2.0, 2.0, 1.5, 1.5, 2.0 },

  /* Computed Parameter: FromWorkspace_Time0
   * Referenced by: '<S8>/From Workspace'
   */
  { 0.0, 1.0, 2.0, 3.0, 4.0 },

  /* Computed Parameter: FromWorkspace_Data0
   * Referenced by: '<S8>/From Workspace'
   */
  { 0.5, 1.0, 1.0, 0.5, 0.5 },

  /* Computed Parameter: FromWorkspace1_Time0
   * Referenced by: '<S8>/From Workspace1'
   */
  { 0.0, 1.0, 2.0, 3.0, 4.0 },

  /* Computed Parameter: FromWorkspace1_Data0
   * Referenced by: '<S8>/From Workspace1'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: [0 0 0 1 1 1]
   * Referenced by: '<S7>/weight'
   */
  { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 },

  /* Expression: [0 0 0 0]
   * Referenced by: '<S7>/init'
   */
  { 0.0, 0.0, 0.0, 0.0 }
};
