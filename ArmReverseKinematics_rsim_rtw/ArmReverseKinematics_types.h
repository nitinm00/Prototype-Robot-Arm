/*
 * ArmReverseKinematics_types.h
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

#ifndef RTW_HEADER_ArmReverseKinematics_types_h_
#define RTW_HEADER_ArmReverseKinematics_types_h_
#include "rtwtypes.h"
#ifndef struct_tag_2PsGMppoK4e2vdwpogf6iH
#define struct_tag_2PsGMppoK4e2vdwpogf6iH

struct tag_2PsGMppoK4e2vdwpogf6iH
{
  int32_T isInitialized;
};

#endif                                 /* struct_tag_2PsGMppoK4e2vdwpogf6iH */

#ifndef typedef_robotics_slcore_internal_bloc_T
#define typedef_robotics_slcore_internal_bloc_T

typedef struct tag_2PsGMppoK4e2vdwpogf6iH robotics_slcore_internal_bloc_T;

#endif                             /* typedef_robotics_slcore_internal_bloc_T */

#ifndef struct_tag_1rRlHW44Di0Fe0KbG1JvIG
#define struct_tag_1rRlHW44Di0Fe0KbG1JvIG

struct tag_1rRlHW44Di0Fe0KbG1JvIG
{
  void *CollisionPrimitive;
};

#endif                                 /* struct_tag_1rRlHW44Di0Fe0KbG1JvIG */

#ifndef typedef_h_robotics_manip_internal_Col_T
#define typedef_h_robotics_manip_internal_Col_T

typedef struct tag_1rRlHW44Di0Fe0KbG1JvIG h_robotics_manip_internal_Col_T;

#endif                             /* typedef_h_robotics_manip_internal_Col_T */

#ifndef struct_tag_sdAmwXbnJnEmimT0NaJRtAD
#define struct_tag_sdAmwXbnJnEmimT0NaJRtAD

struct tag_sdAmwXbnJnEmimT0NaJRtAD
{
  real_T tv_sec;
  real_T tv_nsec;
};

#endif                                 /* struct_tag_sdAmwXbnJnEmimT0NaJRtAD */

#ifndef typedef_sdAmwXbnJnEmimT0NaJRtAD_ArmRe_T
#define typedef_sdAmwXbnJnEmimT0NaJRtAD_ArmRe_T

typedef struct tag_sdAmwXbnJnEmimT0NaJRtAD sdAmwXbnJnEmimT0NaJRtAD_ArmRe_T;

#endif                             /* typedef_sdAmwXbnJnEmimT0NaJRtAD_ArmRe_T */

/* Custom Type definition for MATLABSystem: '<S25>/MATLAB System' */
#include "coder_posix_time.h"
#ifndef struct_tag_1FpmCQNe36RDLjratTWCgF
#define struct_tag_1FpmCQNe36RDLjratTWCgF

struct tag_1FpmCQNe36RDLjratTWCgF
{
  int32_T __dummy;
};

#endif                                 /* struct_tag_1FpmCQNe36RDLjratTWCgF */

#ifndef typedef_f_robotics_manip_internal_Fas_T
#define typedef_f_robotics_manip_internal_Fas_T

typedef struct tag_1FpmCQNe36RDLjratTWCgF f_robotics_manip_internal_Fas_T;

#endif                             /* typedef_f_robotics_manip_internal_Fas_T */

#ifndef struct_tag_9VaLdcnhzQxC5h4iXVOCU
#define struct_tag_9VaLdcnhzQxC5h4iXVOCU

struct tag_9VaLdcnhzQxC5h4iXVOCU
{
  sdAmwXbnJnEmimT0NaJRtAD_ArmRe_T StartTime;
};

#endif                                 /* struct_tag_9VaLdcnhzQxC5h4iXVOCU */

#ifndef typedef_f_robotics_core_internal_Syst_T
#define typedef_f_robotics_core_internal_Syst_T

typedef struct tag_9VaLdcnhzQxC5h4iXVOCU f_robotics_core_internal_Syst_T;

#endif                             /* typedef_f_robotics_core_internal_Syst_T */

#ifndef struct_tag_I7lxy6BEal0s7MBxygd9JE
#define struct_tag_I7lxy6BEal0s7MBxygd9JE

struct tag_I7lxy6BEal0s7MBxygd9JE
{
  real_T f1[16];
};

#endif                                 /* struct_tag_I7lxy6BEal0s7MBxygd9JE */

#ifndef typedef_f_cell_wrap_ArmReverseKinemat_T
#define typedef_f_cell_wrap_ArmReverseKinemat_T

typedef struct tag_I7lxy6BEal0s7MBxygd9JE f_cell_wrap_ArmReverseKinemat_T;

#endif                             /* typedef_f_cell_wrap_ArmReverseKinemat_T */

#ifndef struct_emxArray_char_T
#define struct_emxArray_char_T

struct emxArray_char_T
{
  char_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /* struct_emxArray_char_T */

#ifndef typedef_emxArray_char_T_ArmReverseKin_T
#define typedef_emxArray_char_T_ArmReverseKin_T

typedef struct emxArray_char_T emxArray_char_T_ArmReverseKin_T;

#endif                             /* typedef_emxArray_char_T_ArmReverseKin_T */

#ifndef struct_emxArray_tag_1rRlHW44Di0Fe0KbG1
#define struct_emxArray_tag_1rRlHW44Di0Fe0KbG1

struct emxArray_tag_1rRlHW44Di0Fe0KbG1
{
  h_robotics_manip_internal_Col_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                              /* struct_emxArray_tag_1rRlHW44Di0Fe0KbG1 */

#ifndef typedef_emxArray_h_robotics_manip_int_T
#define typedef_emxArray_h_robotics_manip_int_T

typedef struct emxArray_tag_1rRlHW44Di0Fe0KbG1 emxArray_h_robotics_manip_int_T;

#endif                             /* typedef_emxArray_h_robotics_manip_int_T */

#ifndef struct_tag_fg3hltRT1BVNp1FuRifnyD
#define struct_tag_fg3hltRT1BVNp1FuRifnyD

struct tag_fg3hltRT1BVNp1FuRifnyD
{
  emxArray_char_T_ArmReverseKin_T *Type;
  real_T PositionNumber;
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
  real_T JointAxisInternal[3];
};

#endif                                 /* struct_tag_fg3hltRT1BVNp1FuRifnyD */

#ifndef typedef_c_rigidBodyJoint_ArmReverseKi_T
#define typedef_c_rigidBodyJoint_ArmReverseKi_T

typedef struct tag_fg3hltRT1BVNp1FuRifnyD c_rigidBodyJoint_ArmReverseKi_T;

#endif                             /* typedef_c_rigidBodyJoint_ArmReverseKi_T */

#ifndef struct_tag_EF7TXrGttbvnV740WJJROG
#define struct_tag_EF7TXrGttbvnV740WJJROG

struct tag_EF7TXrGttbvnV740WJJROG
{
  boolean_T matlabCodegenIsDeleted;
  emxArray_h_robotics_manip_int_T *CollisionGeometries;
  real_T MaxElements;
  real_T Size;
};

#endif                                 /* struct_tag_EF7TXrGttbvnV740WJJROG */

#ifndef typedef_i_robotics_manip_internal_Col_T
#define typedef_i_robotics_manip_internal_Col_T

typedef struct tag_EF7TXrGttbvnV740WJJROG i_robotics_manip_internal_Col_T;

#endif                             /* typedef_i_robotics_manip_internal_Col_T */

#ifndef struct_tag_8rdsta0EwtxYbgt3GUOUsC
#define struct_tag_8rdsta0EwtxYbgt3GUOUsC

struct tag_8rdsta0EwtxYbgt3GUOUsC
{
  boolean_T matlabCodegenIsDeleted;
  emxArray_char_T_ArmReverseKin_T *NameInternal;
  c_rigidBodyJoint_ArmReverseKi_T JointInternal;
  real_T ParentIndex;
  i_robotics_manip_internal_Col_T CollisionsInternal;
};

#endif                                 /* struct_tag_8rdsta0EwtxYbgt3GUOUsC */

#ifndef typedef_k_robotics_manip_internal_Rig_T
#define typedef_k_robotics_manip_internal_Rig_T

typedef struct tag_8rdsta0EwtxYbgt3GUOUsC k_robotics_manip_internal_Rig_T;

#endif                             /* typedef_k_robotics_manip_internal_Rig_T */

#ifndef struct_tag_SoxYOMtZTbXBwiJkNX5z2E
#define struct_tag_SoxYOMtZTbXBwiJkNX5z2E

struct tag_SoxYOMtZTbXBwiJkNX5z2E
{
  boolean_T matlabCodegenIsDeleted;
  real_T NumBodies;
  k_robotics_manip_internal_Rig_T Base;
  k_robotics_manip_internal_Rig_T *Bodies[5];
  real_T PositionNumber;
  k_robotics_manip_internal_Rig_T _pobj0[10];
};

#endif                                 /* struct_tag_SoxYOMtZTbXBwiJkNX5z2E */

#ifndef typedef_l_robotics_manip_internal_Rig_T
#define typedef_l_robotics_manip_internal_Rig_T

typedef struct tag_SoxYOMtZTbXBwiJkNX5z2E l_robotics_manip_internal_Rig_T;

#endif                             /* typedef_l_robotics_manip_internal_Rig_T */

#ifndef struct_tag_rBkD54uM3T058mLh9ks4qH
#define struct_tag_rBkD54uM3T058mLh9ks4qH

struct tag_rBkD54uM3T058mLh9ks4qH
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  l_robotics_manip_internal_Rig_T TreeInternal;
};

#endif                                 /* struct_tag_rBkD54uM3T058mLh9ks4qH */

#ifndef typedef_robotics_slmanip_internal_blo_T
#define typedef_robotics_slmanip_internal_blo_T

typedef struct tag_rBkD54uM3T058mLh9ks4qH robotics_slmanip_internal_blo_T;

#endif                             /* typedef_robotics_slmanip_internal_blo_T */

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /* struct_emxArray_real_T */

#ifndef typedef_emxArray_real_T_ArmReverseKin_T
#define typedef_emxArray_real_T_ArmReverseKin_T

typedef struct emxArray_real_T emxArray_real_T_ArmReverseKin_T;

#endif                             /* typedef_emxArray_real_T_ArmReverseKin_T */

#ifndef struct_tag_EhjQFBatIylmRskzCQ6c4C
#define struct_tag_EhjQFBatIylmRskzCQ6c4C

struct tag_EhjQFBatIylmRskzCQ6c4C
{
  emxArray_char_T_ArmReverseKin_T *Type;
  real_T VelocityNumber;
  real_T PositionNumber;
  emxArray_real_T_ArmReverseKin_T *MotionSubspace;
  boolean_T InTree;
  real_T JointToParentTransform[16];
  real_T ChildToJointTransform[16];
  emxArray_char_T_ArmReverseKin_T *NameInternal;
  emxArray_real_T_ArmReverseKin_T *PositionLimitsInternal;
  emxArray_real_T_ArmReverseKin_T *HomePositionInternal;
  real_T JointAxisInternal[3];
};

#endif                                 /* struct_tag_EhjQFBatIylmRskzCQ6c4C */

#ifndef typedef_c_rigidBodyJoint_ArmReverse_e_T
#define typedef_c_rigidBodyJoint_ArmReverse_e_T

typedef struct tag_EhjQFBatIylmRskzCQ6c4C c_rigidBodyJoint_ArmReverse_e_T;

#endif                             /* typedef_c_rigidBodyJoint_ArmReverse_e_T */

#ifndef struct_emxArray_int8_T
#define struct_emxArray_int8_T

struct emxArray_int8_T
{
  int8_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /* struct_emxArray_int8_T */

#ifndef typedef_emxArray_int8_T_ArmReverseKin_T
#define typedef_emxArray_int8_T_ArmReverseKin_T

typedef struct emxArray_int8_T emxArray_int8_T_ArmReverseKin_T;

#endif                             /* typedef_emxArray_int8_T_ArmReverseKin_T */

#ifndef struct_emxArray_tag_I7lxy6BEal0s7MBxyg
#define struct_emxArray_tag_I7lxy6BEal0s7MBxyg

struct emxArray_tag_I7lxy6BEal0s7MBxyg
{
  f_cell_wrap_ArmReverseKinemat_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                              /* struct_emxArray_tag_I7lxy6BEal0s7MBxyg */

#ifndef typedef_emxArray_f_cell_wrap_ArmRever_T
#define typedef_emxArray_f_cell_wrap_ArmRever_T

typedef struct emxArray_tag_I7lxy6BEal0s7MBxyg emxArray_f_cell_wrap_ArmRever_T;

#endif                             /* typedef_emxArray_f_cell_wrap_ArmRever_T */

#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T

struct emxArray_boolean_T
{
  boolean_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /* struct_emxArray_boolean_T */

#ifndef typedef_emxArray_boolean_T_ArmReverse_T
#define typedef_emxArray_boolean_T_ArmReverse_T

typedef struct emxArray_boolean_T emxArray_boolean_T_ArmReverse_T;

#endif                             /* typedef_emxArray_boolean_T_ArmReverse_T */

#ifndef struct_emxArray_uint32_T
#define struct_emxArray_uint32_T

struct emxArray_uint32_T
{
  uint32_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /* struct_emxArray_uint32_T */

#ifndef typedef_emxArray_uint32_T_ArmReverseK_T
#define typedef_emxArray_uint32_T_ArmReverseK_T

typedef struct emxArray_uint32_T emxArray_uint32_T_ArmReverseK_T;

#endif                             /* typedef_emxArray_uint32_T_ArmReverseK_T */

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T

struct emxArray_int32_T
{
  int32_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /* struct_emxArray_int32_T */

#ifndef typedef_emxArray_int32_T_ArmReverseKi_T
#define typedef_emxArray_int32_T_ArmReverseKi_T

typedef struct emxArray_int32_T emxArray_int32_T_ArmReverseKi_T;

#endif                             /* typedef_emxArray_int32_T_ArmReverseKi_T */

#ifndef struct_tag_JNuYD2xDDDy8J51XZ4QDzC
#define struct_tag_JNuYD2xDDDy8J51XZ4QDzC

struct tag_JNuYD2xDDDy8J51XZ4QDzC
{
  void *CollisionPrimitive;
  real_T LocalPose[16];
  real_T WorldPose[16];
  real_T MeshScale[3];
};

#endif                                 /* struct_tag_JNuYD2xDDDy8J51XZ4QDzC */

#ifndef typedef_m_robotics_manip_internal_Col_T
#define typedef_m_robotics_manip_internal_Col_T

typedef struct tag_JNuYD2xDDDy8J51XZ4QDzC m_robotics_manip_internal_Col_T;

#endif                             /* typedef_m_robotics_manip_internal_Col_T */

#ifndef struct_emxArray_tag_JNuYD2xDDDy8J51XZ4
#define struct_emxArray_tag_JNuYD2xDDDy8J51XZ4

struct emxArray_tag_JNuYD2xDDDy8J51XZ4
{
  m_robotics_manip_internal_Col_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                              /* struct_emxArray_tag_JNuYD2xDDDy8J51XZ4 */

#ifndef typedef_emxArray_m_robotics_manip_int_T
#define typedef_emxArray_m_robotics_manip_int_T

typedef struct emxArray_tag_JNuYD2xDDDy8J51XZ4 emxArray_m_robotics_manip_int_T;

#endif                             /* typedef_emxArray_m_robotics_manip_int_T */

#ifndef struct_tag_Ig0XvOQen7UWJ6cHF5czXG
#define struct_tag_Ig0XvOQen7UWJ6cHF5czXG

struct tag_Ig0XvOQen7UWJ6cHF5czXG
{
  boolean_T matlabCodegenIsDeleted;
  emxArray_m_robotics_manip_int_T *CollisionGeometries;
  real_T MaxElements;
  real_T Size;
};

#endif                                 /* struct_tag_Ig0XvOQen7UWJ6cHF5czXG */

#ifndef typedef_n_robotics_manip_internal_Col_T
#define typedef_n_robotics_manip_internal_Col_T

typedef struct tag_Ig0XvOQen7UWJ6cHF5czXG n_robotics_manip_internal_Col_T;

#endif                             /* typedef_n_robotics_manip_internal_Col_T */

#ifndef struct_tag_6MCr9vJQxeu6xsRdsDW1JH
#define struct_tag_6MCr9vJQxeu6xsRdsDW1JH

struct tag_6MCr9vJQxeu6xsRdsDW1JH
{
  boolean_T matlabCodegenIsDeleted;
  real_T Index;
  emxArray_char_T_ArmReverseKin_T *NameInternal;
  c_rigidBodyJoint_ArmReverse_e_T *JointInternal;
  real_T ParentIndex;
  real_T MassInternal;
  real_T CenterOfMassInternal[3];
  real_T InertiaInternal[9];
  real_T SpatialInertia[36];
  n_robotics_manip_internal_Col_T *CollisionsInternal;
};

#endif                                 /* struct_tag_6MCr9vJQxeu6xsRdsDW1JH */

#ifndef typedef_v_robotics_manip_internal_Rig_T
#define typedef_v_robotics_manip_internal_Rig_T

typedef struct tag_6MCr9vJQxeu6xsRdsDW1JH v_robotics_manip_internal_Rig_T;

#endif                             /* typedef_v_robotics_manip_internal_Rig_T */

#ifndef struct_tag_71LhgmcHkiMAxvnIRVCf1C
#define struct_tag_71LhgmcHkiMAxvnIRVCf1C

struct tag_71LhgmcHkiMAxvnIRVCf1C
{
  boolean_T matlabCodegenIsDeleted;
  real_T NumBodies;
  v_robotics_manip_internal_Rig_T Base;
  real_T Gravity[3];
  v_robotics_manip_internal_Rig_T *Bodies[5];
  n_robotics_manip_internal_Col_T _pobj0[11];
  c_rigidBodyJoint_ArmReverse_e_T _pobj1[11];
  v_robotics_manip_internal_Rig_T _pobj2[10];
};

#endif                                 /* struct_tag_71LhgmcHkiMAxvnIRVCf1C */

#ifndef typedef_w_robotics_manip_internal_Rig_T
#define typedef_w_robotics_manip_internal_Rig_T

typedef struct tag_71LhgmcHkiMAxvnIRVCf1C w_robotics_manip_internal_Rig_T;

#endif                             /* typedef_w_robotics_manip_internal_Rig_T */

#ifndef struct_tag_e5aoQj1Nuks1V2RvvPGbaD
#define struct_tag_e5aoQj1Nuks1V2RvvPGbaD

struct tag_e5aoQj1Nuks1V2RvvPGbaD
{
  boolean_T matlabCodegenIsDeleted;
  real_T NumBodies;
  v_robotics_manip_internal_Rig_T Base;
  f_robotics_manip_internal_Fas_T FastVisualizationHelper;
  v_robotics_manip_internal_Rig_T *Bodies[5];
  real_T NumNonFixedBodies;
  real_T PositionNumber;
  real_T VelocityNumber;
  real_T PositionDoFMap[10];
  real_T VelocityDoFMap[10];
  v_robotics_manip_internal_Rig_T _pobj0[5];
  n_robotics_manip_internal_Col_T _pobj1[6];
  c_rigidBodyJoint_ArmReverse_e_T _pobj2[6];
};

#endif                                 /* struct_tag_e5aoQj1Nuks1V2RvvPGbaD */

#ifndef typedef_x_robotics_manip_internal_Rig_T
#define typedef_x_robotics_manip_internal_Rig_T

typedef struct tag_e5aoQj1Nuks1V2RvvPGbaD x_robotics_manip_internal_Rig_T;

#endif                             /* typedef_x_robotics_manip_internal_Rig_T */

#ifndef struct_tag_viGbUejBOXL7koWc4Bn6HF
#define struct_tag_viGbUejBOXL7koWc4Bn6HF

struct tag_viGbUejBOXL7koWc4Bn6HF
{
  boolean_T matlabCodegenIsDeleted;
  x_robotics_manip_internal_Rig_T *Robot;
  real_T WeightMatrix[36];
  emxArray_real_T_ArmReverseKin_T *Limits;
  real_T Tform[16];
  emxArray_real_T_ArmReverseKin_T *ErrTemp;
  real_T CostTemp;
  emxArray_real_T_ArmReverseKin_T *GradTemp;
  real_T BodyIndex;
  real_T KinematicModel;
};

#endif                                 /* struct_tag_viGbUejBOXL7koWc4Bn6HF */

#ifndef typedef_f_robotics_manip_internal_IKE_T
#define typedef_f_robotics_manip_internal_IKE_T

typedef struct tag_viGbUejBOXL7koWc4Bn6HF f_robotics_manip_internal_IKE_T;

#endif                             /* typedef_f_robotics_manip_internal_IKE_T */

#ifndef struct_tag_O5H2AKf48qgYZhUrMigySC
#define struct_tag_O5H2AKf48qgYZhUrMigySC

struct tag_O5H2AKf48qgYZhUrMigySC
{
  boolean_T matlabCodegenIsDeleted;
  char_T Name[18];
  boolean_T ConstraintsOn;
  real_T SolutionTolerance;
  boolean_T RandomRestart;
  f_robotics_manip_internal_IKE_T *ExtraArgs;
  real_T MaxNumIteration;
  real_T MaxTime;
  real_T SeedInternal[4];
  real_T MaxTimeInternal;
  real_T MaxNumIterationInternal;
  real_T StepTolerance;
  f_robotics_core_internal_Syst_T TimeObj;
  real_T GradientTolerance;
  real_T ErrorChangeTolerance;
  real_T DampingBias;
  boolean_T UseErrorDamping;
  f_robotics_core_internal_Syst_T TimeObjInternal;
};

#endif                                 /* struct_tag_O5H2AKf48qgYZhUrMigySC */

#ifndef typedef_h_robotics_core_internal_Erro_T
#define typedef_h_robotics_core_internal_Erro_T

typedef struct tag_O5H2AKf48qgYZhUrMigySC h_robotics_core_internal_Erro_T;

#endif                             /* typedef_h_robotics_core_internal_Erro_T */

#ifndef struct_tag_9bfhTP7pswDaZyNa8kpFVG
#define struct_tag_9bfhTP7pswDaZyNa8kpFVG

struct tag_9bfhTP7pswDaZyNa8kpFVG
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  real_T RigidBodyTreeKinematicModel;
  h_robotics_core_internal_Erro_T *Solver;
  emxArray_real_T_ArmReverseKin_T *Limits;
  x_robotics_manip_internal_Rig_T *RigidBodyTreeInternal;
  f_robotics_manip_internal_IKE_T _pobj0;
  c_rigidBodyJoint_ArmReverse_e_T _pobj1[10];
  v_robotics_manip_internal_Rig_T _pobj2[5];
  n_robotics_manip_internal_Col_T _pobj3[11];
  x_robotics_manip_internal_Rig_T _pobj4;
  h_robotics_core_internal_Erro_T _pobj5;
};

#endif                                 /* struct_tag_9bfhTP7pswDaZyNa8kpFVG */

#ifndef typedef_b_inverseKinematics_ArmRevers_T
#define typedef_b_inverseKinematics_ArmRevers_T

typedef struct tag_9bfhTP7pswDaZyNa8kpFVG b_inverseKinematics_ArmRevers_T;

#endif                             /* typedef_b_inverseKinematics_ArmRevers_T */

#ifndef struct_tag_AhPQdH8ya1iZBBOLTScGaC
#define struct_tag_AhPQdH8ya1iZBBOLTScGaC

struct tag_AhPQdH8ya1iZBBOLTScGaC
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  w_robotics_manip_internal_Rig_T TreeInternal;
  b_inverseKinematics_ArmRevers_T IKInternal;
};

#endif                                 /* struct_tag_AhPQdH8ya1iZBBOLTScGaC */

#ifndef typedef_robotics_slmanip_internal_b_e_T
#define typedef_robotics_slmanip_internal_b_e_T

typedef struct tag_AhPQdH8ya1iZBBOLTScGaC robotics_slmanip_internal_b_e_T;

#endif                             /* typedef_robotics_slmanip_internal_b_e_T */

#ifndef typedef_c_robotics_core_internal_NLPS_T
#define typedef_c_robotics_core_internal_NLPS_T

typedef int32_T c_robotics_core_internal_NLPS_T;

#endif                             /* typedef_c_robotics_core_internal_NLPS_T */

#ifndef robotics_core_internal_NLPSolverExitFlags_constants
#define robotics_core_internal_NLPSolverExitFlags_constants

/* enum robotics_core_internal_NLPSolverExitFlags */
#define LocalMinimumFound              (1)
#define IterationLimitExceeded         (2)
#define TimeLimitExceeded              (3)
#define StepSizeBelowMinimum           (4)
#define ChangeInErrorBelowMinimum      (5)
#define SearchDirectionInvalid         (6)
#define HessianNotPositiveSemidefinite (7)
#define TrustRegionRadiusBelowMinimum  (8)
#endif                 /* robotics_core_internal_NLPSolverExitFlags_constants */

#ifndef SS_UINT64
#define SS_UINT64                      21
#endif

#ifndef SS_INT64
#define SS_INT64                       22
#endif

/* Parameters (default storage) */
typedef struct P_ P;

#endif                            /* RTW_HEADER_ArmReverseKinematics_types_h_ */
