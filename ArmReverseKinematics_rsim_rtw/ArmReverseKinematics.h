/*
 * ArmReverseKinematics.h
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

#ifndef RTW_HEADER_ArmReverseKinematics_h_
#define RTW_HEADER_ArmReverseKinematics_h_
#ifndef ArmReverseKinematics_COMMON_INCLUDES_
#define ArmReverseKinematics_COMMON_INCLUDES_
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "rsim.h"
#include "rt_logging.h"
#include "dt_info.h"
#include "collisioncodegen_api.hpp"
#include "coder_posix_time.h"
#include "nesl_rtw.h"
#include "ArmReverseKinematics_aa99d991_1_gateway.h"
#endif                               /* ArmReverseKinematics_COMMON_INCLUDES_ */

#include "ArmReverseKinematics_types.h"
#include <math.h>
#include "rt_nonfinite.h"
#include <stddef.h>
#include "rtGetNaN.h"
#include "rt_defines.h"
#include "rtGetInf.h"
#define MODEL_NAME                     ArmReverseKinematics
#define NSAMPLE_TIMES                  (2)                       /* Number of sample times */
#define NINPUTS                        (0)                       /* Number of model inputs */
#define NOUTPUTS                       (0)                       /* Number of model outputs */
#define NBLOCKIO                       (8)                       /* Number of data output port signals */
#define NUM_ZC_EVENTS                  (0)                       /* Number of zero-crossing events */
#ifndef NCSTATES
#define NCSTATES                       (8)                       /* Number of continuous states */
#elif NCSTATES != 8
# error Invalid specification of NCSTATES defined in compiler command
#endif

#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm)         (NULL)
#endif

#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T TmpSignalConversionAtCoordinate[3];
  real_T weight[6];                    /* '<S7>/weight' */
  real_T init[4];                      /* '<S7>/init' */
  real_T INPUT_4_1_1[4];               /* '<S19>/INPUT_4_1_1' */
  real_T INPUT_1_1_1[4];               /* '<S19>/INPUT_1_1_1' */
  real_T INPUT_2_1_1[4];               /* '<S19>/INPUT_2_1_1' */
  real_T INPUT_3_1_1[4];               /* '<S19>/INPUT_3_1_1' */
  real_T MATLABSystem[4];              /* '<S25>/MATLAB System' */
} B;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  robotics_slmanip_internal_b_e_T obj; /* '<S25>/MATLAB System' */
  robotics_slmanip_internal_blo_T obj_l;/* '<S20>/MATLAB System' */
  real_T INPUT_4_1_1_Discrete;         /* '<S19>/INPUT_4_1_1' */
  real_T INPUT_4_1_1_FirstOutput;      /* '<S19>/INPUT_4_1_1' */
  real_T INPUT_1_1_1_Discrete;         /* '<S19>/INPUT_1_1_1' */
  real_T INPUT_1_1_1_FirstOutput;      /* '<S19>/INPUT_1_1_1' */
  real_T INPUT_2_1_1_Discrete;         /* '<S19>/INPUT_2_1_1' */
  real_T INPUT_2_1_1_FirstOutput;      /* '<S19>/INPUT_2_1_1' */
  real_T INPUT_3_1_1_Discrete;         /* '<S19>/INPUT_3_1_1' */
  real_T INPUT_3_1_1_FirstOutput;      /* '<S19>/INPUT_3_1_1' */
  real_T OUTPUT_1_0_Discrete;          /* '<S19>/OUTPUT_1_0' */
  real_T STATE_1_Discrete;             /* '<S19>/STATE_1' */
  real_T freq;                         /* '<S25>/MATLAB System' */
  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } fromWS_Signal1_PWORK;              /* '<S8>/fromWS_Signal 1' */

  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } FromWorkspace_PWORK;               /* '<S8>/From Workspace' */

  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } FromWorkspace1_PWORK;              /* '<S8>/From Workspace1' */

  void* OUTPUT_1_0_Simulator;          /* '<S19>/OUTPUT_1_0' */
  void* OUTPUT_1_0_SimData;            /* '<S19>/OUTPUT_1_0' */
  void* OUTPUT_1_0_DiagMgr;            /* '<S19>/OUTPUT_1_0' */
  void* OUTPUT_1_0_ZcLogger;           /* '<S19>/OUTPUT_1_0' */
  void* OUTPUT_1_0_TsInfo;             /* '<S19>/OUTPUT_1_0' */
  struct {
    void *LoggedData;
  } Scope_PWORK;                       /* '<Root>/Scope' */

  void* SINK_1_RtwLogger;              /* '<S19>/SINK_1' */
  void* SINK_1_RtwLogBuffer;           /* '<S19>/SINK_1' */
  void* SINK_1_RtwLogFcnManager;       /* '<S19>/SINK_1' */
  void* STATE_1_Simulator;             /* '<S19>/STATE_1' */
  void* STATE_1_SimData;               /* '<S19>/STATE_1' */
  void* STATE_1_DiagMgr;               /* '<S19>/STATE_1' */
  void* STATE_1_ZcLogger;              /* '<S19>/STATE_1' */
  void* STATE_1_TsInfo;                /* '<S19>/STATE_1' */
  uint32_T state;                      /* '<S25>/MATLAB System' */
  uint32_T state_l[2];                 /* '<S25>/MATLAB System' */
  uint32_T state_i[625];               /* '<S25>/MATLAB System' */
  uint32_T method;                     /* '<S25>/MATLAB System' */
  uint32_T method_d;                   /* '<S25>/MATLAB System' */
  uint32_T state_k[2];                 /* '<S25>/MATLAB System' */
  uint32_T state_h;                    /* '<S20>/MATLAB System' */
  uint32_T state_n[2];                 /* '<S20>/MATLAB System' */
  uint32_T state_j[625];               /* '<S20>/MATLAB System' */
  uint32_T method_e;                   /* '<S20>/MATLAB System' */
  robotics_slcore_internal_bloc_T obj_ln;
                               /* '<S7>/Coordinate Transformation Conversion' */
  robotics_slcore_internal_bloc_T obj_p;
                               /* '<S2>/Coordinate Transformation Conversion' */
  struct {
    int_T PrevIndex;
  } fromWS_Signal1_IWORK;              /* '<S8>/fromWS_Signal 1' */

  struct {
    int_T PrevIndex;
  } FromWorkspace_IWORK;               /* '<S8>/From Workspace' */

  struct {
    int_T PrevIndex;
  } FromWorkspace1_IWORK;              /* '<S8>/From Workspace1' */

  int_T OUTPUT_1_0_Modes;              /* '<S19>/OUTPUT_1_0' */
  int_T STATE_1_Modes;                 /* '<S19>/STATE_1' */
  boolean_T OUTPUT_1_0_FirstOutput;    /* '<S19>/OUTPUT_1_0' */
  boolean_T STATE_1_FirstOutput;       /* '<S19>/STATE_1' */
  boolean_T objisempty;                /* '<S25>/MATLAB System' */
  boolean_T state_not_empty;           /* '<S25>/MATLAB System' */
  boolean_T state_not_empty_k;         /* '<S25>/MATLAB System' */
  boolean_T state_not_empty_d;         /* '<S25>/MATLAB System' */
  boolean_T method_not_empty;          /* '<S25>/MATLAB System' */
  boolean_T freq_not_empty;            /* '<S25>/MATLAB System' */
  boolean_T method_not_empty_m;        /* '<S25>/MATLAB System' */
  boolean_T state_not_empty_j;         /* '<S25>/MATLAB System' */
  boolean_T objisempty_p;      /* '<S7>/Coordinate Transformation Conversion' */
  boolean_T objisempty_n;              /* '<S20>/MATLAB System' */
  boolean_T state_not_empty_p;         /* '<S20>/MATLAB System' */
  boolean_T state_not_empty_kp;        /* '<S20>/MATLAB System' */
  boolean_T state_not_empty_h;         /* '<S20>/MATLAB System' */
  boolean_T method_not_empty_k;        /* '<S20>/MATLAB System' */
  boolean_T objisempty_j;      /* '<S2>/Coordinate Transformation Conversion' */
} DW;

/* Continuous states (default storage) */
typedef struct {
  real_T ArmReverseKinematicsSimulink_PS[2];/* '<S19>/INPUT_4_1_1' */
  real_T ArmReverseKinematicsSimulink__b[2];/* '<S19>/INPUT_1_1_1' */
  real_T ArmReverseKinematicsSimulink__l[2];/* '<S19>/INPUT_2_1_1' */
  real_T ArmReverseKinematicsSimulink__p[2];/* '<S19>/INPUT_3_1_1' */
} X;

/* State derivatives (default storage) */
typedef struct {
  real_T ArmReverseKinematicsSimulink_PS[2];/* '<S19>/INPUT_4_1_1' */
  real_T ArmReverseKinematicsSimulink__b[2];/* '<S19>/INPUT_1_1_1' */
  real_T ArmReverseKinematicsSimulink__l[2];/* '<S19>/INPUT_2_1_1' */
  real_T ArmReverseKinematicsSimulink__p[2];/* '<S19>/INPUT_3_1_1' */
} XDot;

/* State disabled  */
typedef struct {
  boolean_T ArmReverseKinematicsSimulink_PS[2];/* '<S19>/INPUT_4_1_1' */
  boolean_T ArmReverseKinematicsSimulink__b[2];/* '<S19>/INPUT_1_1_1' */
  boolean_T ArmReverseKinematicsSimulink__l[2];/* '<S19>/INPUT_2_1_1' */
  boolean_T ArmReverseKinematicsSimulink__p[2];/* '<S19>/INPUT_3_1_1' */
} XDis;

/* Continuous State Absolute Tolerance  */
typedef struct {
  real_T ArmReverseKinematicsSimulink_PS[2];/* '<S19>/INPUT_4_1_1' */
  real_T ArmReverseKinematicsSimulink__b[2];/* '<S19>/INPUT_1_1_1' */
  real_T ArmReverseKinematicsSimulink__l[2];/* '<S19>/INPUT_2_1_1' */
  real_T ArmReverseKinematicsSimulink__p[2];/* '<S19>/INPUT_3_1_1' */
} CStateAbsTol;

/* Continuous State Perturb Min  */
typedef struct {
  real_T ArmReverseKinematicsSimulink_PS[2];/* '<S19>/INPUT_4_1_1' */
  real_T ArmReverseKinematicsSimulink__b[2];/* '<S19>/INPUT_1_1_1' */
  real_T ArmReverseKinematicsSimulink__l[2];/* '<S19>/INPUT_2_1_1' */
  real_T ArmReverseKinematicsSimulink__p[2];/* '<S19>/INPUT_3_1_1' */
} CXPtMin;

/* Continuous State Perturb Max  */
typedef struct {
  real_T ArmReverseKinematicsSimulink_PS[2];/* '<S19>/INPUT_4_1_1' */
  real_T ArmReverseKinematicsSimulink__b[2];/* '<S19>/INPUT_1_1_1' */
  real_T ArmReverseKinematicsSimulink__l[2];/* '<S19>/INPUT_2_1_1' */
  real_T ArmReverseKinematicsSimulink__p[2];/* '<S19>/INPUT_3_1_1' */
} CXPtMax;

/* Parameters (default storage) */
struct P_ {
  real_T fromWS_Signal1_Time0[5];    /* Computed Parameter: fromWS_Signal1_Time0
                                      * Referenced by: '<S8>/fromWS_Signal 1'
                                      */
  real_T fromWS_Signal1_Data0[5];    /* Computed Parameter: fromWS_Signal1_Data0
                                      * Referenced by: '<S8>/fromWS_Signal 1'
                                      */
  real_T FromWorkspace_Time0[5];      /* Computed Parameter: FromWorkspace_Time0
                                       * Referenced by: '<S8>/From Workspace'
                                       */
  real_T FromWorkspace_Data0[5];      /* Computed Parameter: FromWorkspace_Data0
                                       * Referenced by: '<S8>/From Workspace'
                                       */
  real_T FromWorkspace1_Time0[5];    /* Computed Parameter: FromWorkspace1_Time0
                                      * Referenced by: '<S8>/From Workspace1'
                                      */
  real_T FromWorkspace1_Data0[5];    /* Computed Parameter: FromWorkspace1_Data0
                                      * Referenced by: '<S8>/From Workspace1'
                                      */
  real_T weight_Value[6];              /* Expression: [0 0 0 1 1 1]
                                        * Referenced by: '<S7>/weight'
                                        */
  real_T init_Value[4];                /* Expression: [0 0 0 0]
                                        * Referenced by: '<S7>/init'
                                        */
};

/* External data declarations for dependent source files */
extern const char_T *RT_MEMORY_ALLOCATION_ERROR;
extern B rtB;                          /* block i/o */
extern X rtX;                          /* states (continuous) */
extern DW rtDW;                        /* states (dwork) */
extern P rtP;                          /* parameters */

/* Simulation Structure */
extern SimStruct *const rtS;

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
 * '<Root>' : 'ArmReverseKinematics'
 * '<S1>'   : 'ArmReverseKinematics/Arm'
 * '<S2>'   : 'ArmReverseKinematics/Foward Kinematics'
 * '<S3>'   : 'ArmReverseKinematics/PS-Simulink Converter'
 * '<S4>'   : 'ArmReverseKinematics/PS-Simulink Converter1'
 * '<S5>'   : 'ArmReverseKinematics/PS-Simulink Converter2'
 * '<S6>'   : 'ArmReverseKinematics/PS-Simulink Converter3'
 * '<S7>'   : 'ArmReverseKinematics/Reverse Kinematics'
 * '<S8>'   : 'ArmReverseKinematics/Signal Editor'
 * '<S9>'   : 'ArmReverseKinematics/Simulink-PS Converter'
 * '<S10>'  : 'ArmReverseKinematics/Simulink-PS Converter1'
 * '<S11>'  : 'ArmReverseKinematics/Simulink-PS Converter2'
 * '<S12>'  : 'ArmReverseKinematics/Simulink-PS Converter3'
 * '<S13>'  : 'ArmReverseKinematics/Arm/Base'
 * '<S14>'  : 'ArmReverseKinematics/Arm/Claw'
 * '<S15>'  : 'ArmReverseKinematics/Arm/Link 1'
 * '<S16>'  : 'ArmReverseKinematics/Arm/Link 2'
 * '<S17>'  : 'ArmReverseKinematics/Arm/Link 3'
 * '<S18>'  : 'ArmReverseKinematics/Arm/Solver Configuration'
 * '<S19>'  : 'ArmReverseKinematics/Arm/Solver Configuration/EVAL_KEY'
 * '<S20>'  : 'ArmReverseKinematics/Foward Kinematics/Get Transform'
 * '<S21>'  : 'ArmReverseKinematics/PS-Simulink Converter/EVAL_KEY'
 * '<S22>'  : 'ArmReverseKinematics/PS-Simulink Converter1/EVAL_KEY'
 * '<S23>'  : 'ArmReverseKinematics/PS-Simulink Converter2/EVAL_KEY'
 * '<S24>'  : 'ArmReverseKinematics/PS-Simulink Converter3/EVAL_KEY'
 * '<S25>'  : 'ArmReverseKinematics/Reverse Kinematics/Inverse Kinematics'
 * '<S26>'  : 'ArmReverseKinematics/Simulink-PS Converter/EVAL_KEY'
 * '<S27>'  : 'ArmReverseKinematics/Simulink-PS Converter1/EVAL_KEY'
 * '<S28>'  : 'ArmReverseKinematics/Simulink-PS Converter2/EVAL_KEY'
 * '<S29>'  : 'ArmReverseKinematics/Simulink-PS Converter3/EVAL_KEY'
 */

/* user code (bottom of header file) */
extern const int_T gblNumToFiles;
extern const int_T gblNumFrFiles;
extern const int_T gblNumFrWksBlocks;
extern rtInportTUtable *gblInportTUtables;
extern const char *gblInportFileName;
extern const int_T gblNumRootInportBlks;
extern const int_T gblNumModelInputs;
extern const int_T gblInportDataTypeIdx[];
extern const int_T gblInportDims[];
extern const int_T gblInportComplex[];
extern const int_T gblInportInterpoFlag[];
extern const int_T gblInportContinuous[];

#endif                                 /* RTW_HEADER_ArmReverseKinematics_h_ */
