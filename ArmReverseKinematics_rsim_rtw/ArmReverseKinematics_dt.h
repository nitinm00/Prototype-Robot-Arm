/*
 * ArmReverseKinematics_dt.h
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

/* data type size table */
static uint_T rtDataTypeSizes[] = {
  sizeof(real_T),
  sizeof(real32_T),
  sizeof(int8_T),
  sizeof(uint8_T),
  sizeof(int16_T),
  sizeof(uint16_T),
  sizeof(int32_T),
  sizeof(uint32_T),
  sizeof(boolean_T),
  sizeof(fcn_call_T),
  sizeof(int_T),
  sizeof(pointer_T),
  sizeof(action_T),
  2*sizeof(uint32_T),
  sizeof(int32_T),
  sizeof(int64_T),
  sizeof(uint64_T),
  sizeof(int32_T),
  sizeof(robotics_slcore_internal_bloc_T),
  sizeof(robotics_slmanip_internal_blo_T),
  sizeof(robotics_slmanip_internal_b_e_T),
  sizeof(uint64_T),
  sizeof(int64_T),
  sizeof(uint_T),
  sizeof(char_T),
  sizeof(uchar_T),
  sizeof(time_T)
};

/* data type name table */
static const char_T * rtDataTypeNames[] = {
  "real_T",
  "real32_T",
  "int8_T",
  "uint8_T",
  "int16_T",
  "uint16_T",
  "int32_T",
  "uint32_T",
  "boolean_T",
  "fcn_call_T",
  "int_T",
  "pointer_T",
  "action_T",
  "timer_uint32_pair_T",
  "physical_connection",
  "int64_T",
  "uint64_T",
  "struct_j29BDD3GtugYMsepf4x9iH",
  "robotics_slcore_internal_bloc_T",
  "robotics_slmanip_internal_blo_T",
  "robotics_slmanip_internal_b_e_T",
  "uint64_T",
  "int64_T",
  "uint_T",
  "char_T",
  "uchar_T",
  "time_T"
};

/* data type transitions for block I/O structure */
static DataTypeTransition rtBTransitions[] = {
  { (char_T *)(&rtB.TmpSignalConversionAtCoordinate[0]), 0, 0, 33 }
  ,

  { (char_T *)(&rtDW.obj), 20, 0, 1 },

  { (char_T *)(&rtDW.obj_l), 19, 0, 1 },

  { (char_T *)(&rtDW.INPUT_4_1_1_Discrete), 0, 0, 11 },

  { (char_T *)(&rtDW.fromWS_Signal1_PWORK.TimePtr), 11, 0, 17 },

  { (char_T *)(&rtDW.state), 7, 0, 1261 },

  { (char_T *)(&rtDW.obj_ln), 18, 0, 2 },

  { (char_T *)(&rtDW.fromWS_Signal1_IWORK.PrevIndex), 10, 0, 5 },

  { (char_T *)(&rtDW.OUTPUT_1_0_FirstOutput), 8, 0, 17 }
};

/* data type transition table for block I/O structure */
static DataTypeTransitionTable rtBTransTable = {
  9U,
  rtBTransitions
};

/* data type transitions for Parameters structure */
static DataTypeTransition rtPTransitions[] = {
  { (char_T *)(&rtP.fromWS_Signal1_Time0[0]), 0, 0, 40 }
};

/* data type transition table for Parameters structure */
static DataTypeTransitionTable rtPTransTable = {
  1U,
  rtPTransitions
};

/* [EOF] ArmReverseKinematics_dt.h */
