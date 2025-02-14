/*
 * ArmReverseKinematics.c
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
#include "ArmReverseKinematics_types.h"
#include "rtwtypes.h"
#include <string.h>
#include <math.h>
#include <emmintrin.h>
#include <stddef.h>
#include "ArmReverseKinematics_private.h"
#include "rt_nonfinite.h"
#include "coder_posix_time.h"
#include <stdlib.h>
#include "rt_defines.h"
#include "ArmReverseKinematics_dt.h"

/* user code (top of parameter file) */
const int_T gblNumToFiles = 0;
const int_T gblNumFrFiles = 0;
const int_T gblNumFrWksBlocks = 3;
const char *gblSlvrJacPatternFileName =
  "ArmReverseKinematics_rsim_rtw\\ArmReverseKinematics_Jpattern.mat";

/* Root inports information  */
const int_T gblNumRootInportBlks = 0;
const int_T gblNumModelInputs = 0;
extern rtInportTUtable *gblInportTUtables;
extern const char *gblInportFileName;
const int_T gblInportDataTypeIdx[] = { -1 };

const int_T gblInportDims[] = { -1 } ;

const int_T gblInportComplex[] = { -1 };

const int_T gblInportInterpoFlag[] = { -1 };

const int_T gblInportContinuous[] = { -1 };

#include "simstruc.h"
#include "fixedpoint.h"

/* Block signals (default storage) */
B rtB;

/* Continuous states */
X rtX;

/* Block states (default storage) */
DW rtDW;

/* Parent Simstruct */
static SimStruct model_S;
SimStruct *const rtS = &model_S;

/* Forward declaration for local functions */
static void ArmReverseKinema_emxInit_char_T(emxArray_char_T_ArmReverseKin_T
  **pEmxArray, int32_T numDimensions);
static void emxInitStruct_v_robotics_manip_(v_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInit_m_robotics_manip_intern(emxArray_m_robotics_manip_int_T
  **pEmxArray, int32_T numDimensions);
static void emxInitStruct_n_robotics_manip_(n_robotics_manip_internal_Col_T
  *pStruct);
static void emxInitMatrix_n_robotics_manip_(n_robotics_manip_internal_Col_T
  pMatrix[11]);
static void ArmReverseKinema_emxInit_real_T(emxArray_real_T_ArmReverseKin_T
  **pEmxArray, int32_T numDimensions);
static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_ArmReverse_e_T
  *pStruct);
static void emxInitMatrix_c_rigidBodyJoint(c_rigidBodyJoint_ArmReverse_e_T
  pMatrix[11]);
static void emxInitMatrix_v_robotics_manip_(v_robotics_manip_internal_Rig_T
  pMatrix[10]);
static void emxInitStruct_w_robotics_manip_(w_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T
  *pStruct);
static void emxInitMatrix_c_rigidBodyJoint1(c_rigidBodyJoint_ArmReverse_e_T
  pMatrix[10]);
static void emxInitMatrix_v_robotics_mani_e(v_robotics_manip_internal_Rig_T
  pMatrix[5]);
static void emxInitMatrix_n_robotics_mani_e(n_robotics_manip_internal_Col_T
  pMatrix[6]);
static void emxInitMatrix_c_rigidBodyJoint2(c_rigidBodyJoint_ArmReverse_e_T
  pMatrix[6]);
static void emxInitStruct_x_robotics_manip_(x_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_b_inverseKinemati(b_inverseKinematics_ArmRevers_T
  *pStruct);
static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_b_e_T
  *pStruct);
static void ArmReve_genrand_uint32_vector_e(uint32_T mt[625], uint32_T u[2]);
static boolean_T ArmReverseKinema_is_valid_state(const uint32_T mt[625]);
static void ArmReverseKinematics_rand_e(real_T r[5]);
static void ArmRev_emxEnsureCapacity_char_T(emxArray_char_T_ArmReverseKin_T
  *emxArray, int32_T oldNumel);
static void ArmReverseKinema_emxFree_char_T(emxArray_char_T_ArmReverseKin_T
  **pEmxArray);
static void ArmRev_emxEnsureCapacity_real_T(emxArray_real_T_ArmReverseKin_T
  *emxArray, int32_T oldNumel);
static void emxEnsureCapacity_m_robotics_ma(emxArray_m_robotics_manip_int_T
  *emxArray, int32_T oldNumel);
static void emxFree_m_robotics_manip_intern(emxArray_m_robotics_manip_int_T
  **pEmxArray);
static n_robotics_manip_internal_Col_T *Arm_CollisionSet_CollisionSet_e
  (n_robotics_manip_internal_Col_T *obj, real_T maxElements);
static v_robotics_manip_internal_Rig_T *ArmRever_RigidBody_RigidBody_ev
  (v_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_ArmReverse_e_T *iobj_1);
static v_robotics_manip_internal_Rig_T *ArmReve_RigidBody_RigidBody_evf
  (v_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_ArmReverse_e_T *iobj_1);
static v_robotics_manip_internal_Rig_T *ArmRev_RigidBody_RigidBody_evfm
  (v_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_ArmReverse_e_T *iobj_1);
static v_robotics_manip_internal_Rig_T *ArmRe_RigidBody_RigidBody_evfmf
  (v_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_ArmReverse_e_T *iobj_1);
static v_robotics_manip_internal_Rig_T *ArmR_RigidBody_RigidBody_evfmfk
  (v_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_ArmReverse_e_T *iobj_1);
static v_robotics_manip_internal_Rig_T *Arm_RigidBody_RigidBody_evfmfk1
  (v_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_ArmReverse_e_T *iobj_1);
static v_robotics_manip_internal_Rig_T *Ar_RigidBody_RigidBody_evfmfk1h
  (v_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_ArmReverse_e_T *iobj_1);
static v_robotics_manip_internal_Rig_T *A_RigidBody_RigidBody_evfmfk1h2
  (v_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_ArmReverse_e_T *iobj_1);
static c_rigidBodyJoint_ArmReverse_e_T *A_rigidBodyJoint_rigidBodyJoint
  (c_rigidBodyJoint_ArmReverse_e_T *obj, const emxArray_char_T_ArmReverseKin_T
   *jname);
static boolean_T ArmReverseKinematics_strcmp(const
  emxArray_char_T_ArmReverseKin_T *a, const emxArray_char_T_ArmReverseKin_T *b);
static real_T RigidBodyTree_findBodyIndexByNa(x_robotics_manip_internal_Rig_T
  *obj, const emxArray_char_T_ArmReverseKin_T *bodyname);
static void ArmReverseKinema_emxFree_real_T(emxArray_real_T_ArmReverseKin_T
  **pEmxArray);
static v_robotics_manip_internal_Rig_T *ArmReverseKinema_RigidBody_copy
  (v_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_ArmReverse_e_T *iobj_1, v_robotics_manip_internal_Rig_T
   *iobj_2);
static void ArmRevers_RigidBodyTree_addBody(x_robotics_manip_internal_Rig_T *obj,
  v_robotics_manip_internal_Rig_T *bodyin, const emxArray_char_T_ArmReverseKin_T
  *parentName, n_robotics_manip_internal_Col_T *iobj_0,
  c_rigidBodyJoint_ArmReverse_e_T *iobj_1, v_robotics_manip_internal_Rig_T
  *iobj_2);
static void ArmRev_SystemProp_setProperties(b_inverseKinematics_ArmRevers_T *obj,
  w_robotics_manip_internal_Rig_T *varargin_2, c_rigidBodyJoint_ArmReverse_e_T
  *iobj_0, v_robotics_manip_internal_Rig_T *iobj_1,
  n_robotics_manip_internal_Col_T *iobj_2, x_robotics_manip_internal_Rig_T
  *iobj_3, h_robotics_core_internal_Erro_T *iobj_4);
static void ArmReverseKi_SystemCore_setup_e(robotics_slmanip_internal_b_e_T *obj);
static void emxInitStruct_c_rigidBodyJoint1(c_rigidBodyJoint_ArmReverseKi_T
  *pStruct);
static void emxInit_h_robotics_manip_intern(emxArray_h_robotics_manip_int_T
  **pEmxArray, int32_T numDimensions);
static void emxInitStruct_i_robotics_manip_(i_robotics_manip_internal_Col_T
  *pStruct);
static void emxInitStruct_k_robotics_manip_(k_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitMatrix_k_robotics_manip_(k_robotics_manip_internal_Rig_T
  pMatrix[10]);
static void emxInitStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct);
static void emxInitStruct_robotics_slmani_e(robotics_slmanip_internal_blo_T
  *pStruct);
static void ArmReverseKinematics_rand(real_T r[5]);
static void emxEnsureCapacity_h_robotics_ma(emxArray_h_robotics_manip_int_T
  *emxArray, int32_T oldNumel);
static void emxFree_h_robotics_manip_intern(emxArray_h_robotics_manip_int_T
  **pEmxArray);
static i_robotics_manip_internal_Col_T *ArmRe_CollisionSet_CollisionSet
  (i_robotics_manip_internal_Col_T *obj);
static void RigidBodyTree_defaultInitialize(l_robotics_manip_internal_Rig_T *obj,
  k_robotics_manip_internal_Rig_T *iobj_0);
static k_robotics_manip_internal_Rig_T *ArmReverseK_RigidBody_RigidBody
  (k_robotics_manip_internal_Rig_T *obj);
static k_robotics_manip_internal_Rig_T *ArmRevers_RigidBody_RigidBody_e
  (k_robotics_manip_internal_Rig_T *obj);
static void ArmReverseKine_SystemCore_setup(robotics_slmanip_internal_blo_T *obj);
static void RigidBodyTree_get_JointPosition(x_robotics_manip_internal_Rig_T *obj,
  emxArray_real_T_ArmReverseKin_T *limits);
static void ArmReverseKinema_emxInit_int8_T(emxArray_int8_T_ArmReverseKin_T
  **pEmxArray, int32_T numDimensions);
static void ArmRev_emxEnsureCapacity_int8_T(emxArray_int8_T_ArmReverseKin_T
  *emxArray, int32_T oldNumel);
static void ArmReverseKinema_emxFree_int8_T(emxArray_int8_T_ArmReverseKin_T
  **pEmxArray);
static void ArmReverseKi_binary_expand_op_4(boolean_T in1[4], const real_T in2[4],
  const emxArray_real_T_ArmReverseKin_T *in3);
static void ArmReverseKi_binary_expand_op_3(boolean_T in1[4], const real_T in2[4],
  const emxArray_real_T_ArmReverseKin_T *in3);
static void ArmReverseKinematics_eml_find(const boolean_T x[4], int32_T i_data[],
  int32_T *i_size);
static void ArmReverseKinematics_tic(real_T *tstart_tv_sec, real_T
  *tstart_tv_nsec);
static void A_RigidBodyTree_ancestorIndices(x_robotics_manip_internal_Rig_T *obj,
  v_robotics_manip_internal_Rig_T *body, emxArray_real_T_ArmReverseKin_T
  *indices);
static void RigidBodyTree_kinematicPathInte(x_robotics_manip_internal_Rig_T *obj,
  v_robotics_manip_internal_Rig_T *body1, v_robotics_manip_internal_Rig_T *body2,
  emxArray_real_T_ArmReverseKin_T *indices);
static void rigidBodyJoint_get_JointAxis_e(const c_rigidBodyJoint_ArmReverse_e_T
  *obj, real_T ax[3]);
static void ArmReverseKinematics_cat(real_T varargin_1, real_T varargin_2,
  real_T varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6,
  real_T varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9]);
static void ArmReverseKinematics_mtimes(const real_T A[36], const
  emxArray_real_T_ArmReverseKin_T *B_0, emxArray_real_T_ArmReverseKin_T *C);
static void RigidBodyTree_efficientFKAndJac(x_robotics_manip_internal_Rig_T *obj,
  const real_T qv[4], real_T bid1, real_T T_data[], int32_T T_size[2],
  emxArray_real_T_ArmReverseKin_T *Jac);
static creal_T ArmReverseKinematics_sqrt(const creal_T x);
static real_T ArmReverseKinematics_xnrm2(int32_T n, const real_T x[9], int32_T
  ix0);
static real_T ArmReverseKinematics_xdotc(int32_T n, const real_T x[9], int32_T
  ix0, const real_T y[9], int32_T iy0);
static void ArmReverseKinematics_xaxpy(int32_T n, real_T a, int32_T ix0, const
  real_T y[9], int32_T iy0, real_T b_y[9]);
static real_T ArmReverseKinematics_xnrm2_e(const real_T x[3], int32_T ix0);
static void ArmReverseKinematics_xaxpy_evf(int32_T n, real_T a, const real_T x[9],
  int32_T ix0, real_T y[3], int32_T iy0);
static void ArmReverseKinematics_xaxpy_ev(int32_T n, real_T a, const real_T x[3],
  int32_T ix0, const real_T y[9], int32_T iy0, real_T b_y[9]);
static void ArmReverseKinematics_xswap(const real_T x[9], int32_T ix0, int32_T
  iy0, real_T b_x[9]);
static void ArmReverseKinematics_xrotg(real_T a, real_T b, real_T *b_a, real_T
  *b_b, real_T *c, real_T *s);
static void ArmReverseKinematics_xrot(const real_T x[9], int32_T ix0, int32_T
  iy0, real_T c, real_T s, real_T b_x[9]);
static void ArmReverseKinematics_svd(const real_T A[9], real_T U[9], real_T s[3],
  real_T V[9]);
static void ArmReverseK_IKHelpers_poseError(const real_T Td[16], const real_T
  T_data[], const int32_T T_size[2], real_T errorvec[6]);
static void ArmReverseKinematics_mtimes_e(const real_T A[6], const
  emxArray_real_T_ArmReverseKin_T *B_1, emxArray_real_T_ArmReverseKin_T *C);
static void ArmReverseKin_emxInit_boolean_T(emxArray_boolean_T_ArmReverse_T
  **pEmxArray, int32_T numDimensions);
static real_T ArmReverseKinematics_norm_e(const real_T x[6]);
static void ArmReverseKinematics_minus_e(emxArray_real_T_ArmReverseKin_T *in1,
  const emxArray_real_T_ArmReverseKin_T *in2);
static void Arm_emxEnsureCapacity_boolean_T(emxArray_boolean_T_ArmReverse_T
  *emxArray, int32_T oldNumel);
static real_T ArmReverseKinematics_toc(real_T tstart_tv_sec, real_T
  tstart_tv_nsec);
static void ArmReverseKinematics_mldivide(const real_T A[16], const
  emxArray_real_T_ArmReverseKin_T *B_2, real_T Y_data[], int32_T *Y_size);
static void ArmReverseKi_binary_expand_op_1(real_T in1_data[], int32_T *in1_size,
  const emxArray_real_T_ArmReverseKin_T *in2, real_T in3, const real_T in4[16],
  const emxArray_real_T_ArmReverseKin_T *in5);
static void ArmReverseKinematics_expand_max(const
  emxArray_real_T_ArmReverseKin_T *a, const real_T b[4], real_T c[4]);
static void ArmReverseKinematics_expand_min(const
  emxArray_real_T_ArmReverseKin_T *a, const real_T b[4], real_T c[4]);
static void ArmReverseKin_emxFree_boolean_T(emxArray_boolean_T_ArmReverse_T
  **pEmxArray);
static void ErrorDampedLevenbergMarquardt_s(h_robotics_core_internal_Erro_T *obj,
  real_T xSol[4], c_robotics_core_internal_NLPS_T *exitFlag, real_T *en, real_T *
  iter);
static boolean_T ArmReverseKinematics_any(const emxArray_boolean_T_ArmReverse_T *
  x);
static void ArmReverseKinematics_randn(const real_T varargin_1[2],
  emxArray_real_T_ArmReverseKin_T *r);
static void ArmReverseKinematics_minus(emxArray_real_T_ArmReverseKin_T *in1,
  const emxArray_real_T_ArmReverseKin_T *in2);
static void ArmReverseKinematics_plus(emxArray_real_T_ArmReverseKin_T *in1,
  const emxArray_real_T_ArmReverseKin_T *in2);
static void ArmReverseKinematics_rand_ev(real_T varargin_1,
  emxArray_real_T_ArmReverseKin_T *r);
static void ArmReverseKine_binary_expand_op(emxArray_real_T_ArmReverseKin_T *in1,
  const emxArray_real_T_ArmReverseKin_T *in2, const
  emxArray_real_T_ArmReverseKin_T *in3);
static void ArmRev_NLPSolverInterface_solve(h_robotics_core_internal_Erro_T *obj,
  const real_T seed[4], real_T xSol[4], real_T *solutionInfo_Iterations, real_T *
  solutionInfo_RRAttempts, real_T *solutionInfo_Error, real_T
  *solutionInfo_ExitFlag, char_T solutionInfo_Status_data[], int32_T
  solutionInfo_Status_size[2]);
static void ArmReverseKinem_emxInit_int32_T(emxArray_int32_T_ArmReverseKi_T
  **pEmxArray, int32_T numDimensions);
static void ArmRe_emxEnsureCapacity_int32_T(emxArray_int32_T_ArmReverseKi_T
  *emxArray, int32_T oldNumel);
static void ArmReverseKinem_emxFree_int32_T(emxArray_int32_T_ArmReverseKi_T
  **pEmxArray);
static void ArmReverseKine_emxInit_uint32_T(emxArray_uint32_T_ArmReverseK_T
  **pEmxArray, int32_T numDimensions);
static void ArmR_emxEnsureCapacity_uint32_T(emxArray_uint32_T_ArmReverseK_T
  *emxArray, int32_T oldNumel);
static void ArmReverseKine_emxFree_uint32_T(emxArray_uint32_T_ArmReverseK_T
  **pEmxArray);
static void ArmRe_inverseKinematics_solve_e(b_inverseKinematics_ArmRevers_T *obj,
  real_T initialGuess[4], real_T *solutionInfo_Iterations, real_T
  *solutionInfo_NumRandomRestarts, real_T *solutionInfo_PoseErrorNorm, real_T
  *solutionInfo_ExitFlag, char_T solutionInfo_Status_data[], int32_T
  solutionInfo_Status_size[2]);
static void ArmR_inverseKinematics_stepImpl(b_inverseKinematics_ArmRevers_T *obj,
  const real_T tform[16], const real_T weights[6], const real_T initialGuess[4],
  real_T QSol[4]);
static void ArmReverseK_emxInit_f_cell_wrap(emxArray_f_cell_wrap_ArmRever_T
  **pEmxArray, int32_T numDimensions);
static void A_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_ArmRever_T
  *emxArray, int32_T oldNumel);
static void Ar_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_ArmReverseKi_T *obj, real_T ax[3]);
static void ArmReverseK_emxFree_f_cell_wrap(emxArray_f_cell_wrap_ArmRever_T
  **pEmxArray);
static void emxFreeStruct_v_robotics_manip_(v_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_n_robotics_manip_(n_robotics_manip_internal_Col_T
  *pStruct);
static void emxFreeMatrix_n_robotics_manip_(n_robotics_manip_internal_Col_T
  pMatrix[11]);
static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_ArmReverse_e_T
  *pStruct);
static void emxFreeMatrix_c_rigidBodyJoint(c_rigidBodyJoint_ArmReverse_e_T
  pMatrix[11]);
static void emxFreeMatrix_v_robotics_manip_(v_robotics_manip_internal_Rig_T
  pMatrix[10]);
static void emxFreeStruct_w_robotics_manip_(w_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T
  *pStruct);
static void emxFreeMatrix_c_rigidBodyJoint1(c_rigidBodyJoint_ArmReverse_e_T
  pMatrix[10]);
static void emxFreeMatrix_v_robotics_mani_e(v_robotics_manip_internal_Rig_T
  pMatrix[5]);
static void emxFreeMatrix_n_robotics_mani_e(n_robotics_manip_internal_Col_T
  pMatrix[6]);
static void emxFreeMatrix_c_rigidBodyJoint2(c_rigidBodyJoint_ArmReverse_e_T
  pMatrix[6]);
static void emxFreeStruct_x_robotics_manip_(x_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_b_inverseKinemati(b_inverseKinematics_ArmRevers_T
  *pStruct);
static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_b_e_T
  *pStruct);
static void emxFreeStruct_c_rigidBodyJoint1(c_rigidBodyJoint_ArmReverseKi_T
  *pStruct);
static void emxFreeStruct_i_robotics_manip_(i_robotics_manip_internal_Col_T
  *pStruct);
static void emxFreeStruct_k_robotics_manip_(k_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeMatrix_k_robotics_manip_(k_robotics_manip_internal_Rig_T
  pMatrix[10]);
static void emxFreeStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct);
static void emxFreeStruct_robotics_slmani_e(robotics_slmanip_internal_blo_T
  *pStruct);
int32_T div_s32(int32_T numerator, int32_T denominator)
{
  int32_T quotient;
  if (denominator == 0) {
    quotient = numerator >= 0 ? MAX_int32_T : MIN_int32_T;

    /* Divide by zero handler */
  } else {
    uint32_T tempAbsQuotient;
    tempAbsQuotient = (numerator < 0 ? ~(uint32_T)numerator + 1U : (uint32_T)
                       numerator) / (denominator < 0 ? ~(uint32_T)denominator +
      1U : (uint32_T)denominator);
    quotient = (numerator < 0) != (denominator < 0) ? -(int32_T)tempAbsQuotient :
      (int32_T)tempAbsQuotient;
  }

  return quotient;
}

static void ArmReverseKinema_emxInit_char_T(emxArray_char_T_ArmReverseKin_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_char_T_ArmReverseKin_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_char_T_ArmReverseKin_T *)malloc(sizeof
    (emxArray_char_T_ArmReverseKin_T));
  emxArray = *pEmxArray;
  emxArray->data = (char_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * (uint32_T)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void emxInitStruct_v_robotics_manip_(v_robotics_manip_internal_Rig_T
  *pStruct)
{
  ArmReverseKinema_emxInit_char_T(&pStruct->NameInternal, 2);
}

static void emxInit_m_robotics_manip_intern(emxArray_m_robotics_manip_int_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_m_robotics_manip_int_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_m_robotics_manip_int_T *)malloc(sizeof
    (emxArray_m_robotics_manip_int_T));
  emxArray = *pEmxArray;
  emxArray->data = (m_robotics_manip_internal_Col_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * (uint32_T)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void emxInitStruct_n_robotics_manip_(n_robotics_manip_internal_Col_T
  *pStruct)
{
  emxInit_m_robotics_manip_intern(&pStruct->CollisionGeometries, 2);
}

static void emxInitMatrix_n_robotics_manip_(n_robotics_manip_internal_Col_T
  pMatrix[11])
{
  int32_T i;
  for (i = 0; i < 11; i++) {
    emxInitStruct_n_robotics_manip_(&pMatrix[i]);
  }
}

static void ArmReverseKinema_emxInit_real_T(emxArray_real_T_ArmReverseKin_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_real_T_ArmReverseKin_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_real_T_ArmReverseKin_T *)malloc(sizeof
    (emxArray_real_T_ArmReverseKin_T));
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * (uint32_T)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void emxInitStruct_c_rigidBodyJoint(c_rigidBodyJoint_ArmReverse_e_T
  *pStruct)
{
  ArmReverseKinema_emxInit_char_T(&pStruct->Type, 2);
  ArmReverseKinema_emxInit_real_T(&pStruct->MotionSubspace, 2);
  ArmReverseKinema_emxInit_char_T(&pStruct->NameInternal, 2);
  ArmReverseKinema_emxInit_real_T(&pStruct->PositionLimitsInternal, 2);
  ArmReverseKinema_emxInit_real_T(&pStruct->HomePositionInternal, 1);
}

static void emxInitMatrix_c_rigidBodyJoint(c_rigidBodyJoint_ArmReverse_e_T
  pMatrix[11])
{
  int32_T i;
  for (i = 0; i < 11; i++) {
    emxInitStruct_c_rigidBodyJoint(&pMatrix[i]);
  }
}

static void emxInitMatrix_v_robotics_manip_(v_robotics_manip_internal_Rig_T
  pMatrix[10])
{
  int32_T i;
  for (i = 0; i < 10; i++) {
    emxInitStruct_v_robotics_manip_(&pMatrix[i]);
  }
}

static void emxInitStruct_w_robotics_manip_(w_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_v_robotics_manip_(&pStruct->Base);
  emxInitMatrix_n_robotics_manip_(pStruct->_pobj0);
  emxInitMatrix_c_rigidBodyJoint(pStruct->_pobj1);
  emxInitMatrix_v_robotics_manip_(pStruct->_pobj2);
}

static void emxInitStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T
  *pStruct)
{
  ArmReverseKinema_emxInit_real_T(&pStruct->Limits, 2);
  ArmReverseKinema_emxInit_real_T(&pStruct->ErrTemp, 1);
  ArmReverseKinema_emxInit_real_T(&pStruct->GradTemp, 1);
}

static void emxInitMatrix_c_rigidBodyJoint1(c_rigidBodyJoint_ArmReverse_e_T
  pMatrix[10])
{
  int32_T i;
  for (i = 0; i < 10; i++) {
    emxInitStruct_c_rigidBodyJoint(&pMatrix[i]);
  }
}

static void emxInitMatrix_v_robotics_mani_e(v_robotics_manip_internal_Rig_T
  pMatrix[5])
{
  int32_T i;
  for (i = 0; i < 5; i++) {
    emxInitStruct_v_robotics_manip_(&pMatrix[i]);
  }
}

static void emxInitMatrix_n_robotics_mani_e(n_robotics_manip_internal_Col_T
  pMatrix[6])
{
  int32_T i;
  for (i = 0; i < 6; i++) {
    emxInitStruct_n_robotics_manip_(&pMatrix[i]);
  }
}

static void emxInitMatrix_c_rigidBodyJoint2(c_rigidBodyJoint_ArmReverse_e_T
  pMatrix[6])
{
  int32_T i;
  for (i = 0; i < 6; i++) {
    emxInitStruct_c_rigidBodyJoint(&pMatrix[i]);
  }
}

static void emxInitStruct_x_robotics_manip_(x_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_v_robotics_manip_(&pStruct->Base);
  emxInitMatrix_v_robotics_mani_e(pStruct->_pobj0);
  emxInitMatrix_n_robotics_mani_e(pStruct->_pobj1);
  emxInitMatrix_c_rigidBodyJoint2(pStruct->_pobj2);
}

static void emxInitStruct_b_inverseKinemati(b_inverseKinematics_ArmRevers_T
  *pStruct)
{
  ArmReverseKinema_emxInit_real_T(&pStruct->Limits, 2);
  emxInitStruct_f_robotics_manip_(&pStruct->_pobj0);
  emxInitMatrix_c_rigidBodyJoint1(pStruct->_pobj1);
  emxInitMatrix_v_robotics_mani_e(pStruct->_pobj2);
  emxInitMatrix_n_robotics_manip_(pStruct->_pobj3);
  emxInitStruct_x_robotics_manip_(&pStruct->_pobj4);
}

static void emxInitStruct_robotics_slmanip_(robotics_slmanip_internal_b_e_T
  *pStruct)
{
  emxInitStruct_w_robotics_manip_(&pStruct->TreeInternal);
  emxInitStruct_b_inverseKinemati(&pStruct->IKInternal);
}

static void ArmReve_genrand_uint32_vector_e(uint32_T mt[625], uint32_T u[2])
{
  int32_T b_j;
  int32_T b_kk;
  for (b_j = 0; b_j < 2; b_j++) {
    uint32_T mti;
    uint32_T y;
    mti = mt[624] + 1U;
    if (mt[624] + 1U >= 625U) {
      for (b_kk = 0; b_kk < 227; b_kk++) {
        y = (mt[b_kk + 1] & 2147483647U) | (mt[b_kk] & 2147483648U);
        if ((y & 1U) == 0U) {
          mti = y >> 1U;
        } else {
          mti = y >> 1U ^ 2567483615U;
        }

        mt[b_kk] = mt[b_kk + 397] ^ mti;
      }

      for (b_kk = 0; b_kk < 396; b_kk++) {
        y = (mt[b_kk + 227] & 2147483648U) | (mt[b_kk + 228] & 2147483647U);
        if ((y & 1U) == 0U) {
          mti = y >> 1U;
        } else {
          mti = y >> 1U ^ 2567483615U;
        }

        mt[b_kk + 227] = mt[b_kk] ^ mti;
      }

      y = (mt[623] & 2147483648U) | (mt[0] & 2147483647U);
      if ((y & 1U) == 0U) {
        mti = y >> 1U;
      } else {
        mti = y >> 1U ^ 2567483615U;
      }

      mt[623] = mt[396] ^ mti;
      mti = 1U;
    }

    y = mt[(int32_T)mti - 1];
    mt[624] = mti;
    y ^= y >> 11U;
    y ^= y << 7U & 2636928640U;
    y ^= y << 15U & 4022730752U;
    u[b_j] = y >> 18U ^ y;
  }
}

static boolean_T ArmReverseKinema_is_valid_state(const uint32_T mt[625])
{
  boolean_T isvalid;
  if ((mt[624] >= 1U) && (mt[624] < 625U)) {
    int32_T k;
    boolean_T exitg1;
    isvalid = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k + 1 < 625)) {
      if (mt[k] == 0U) {
        k++;
      } else {
        isvalid = true;
        exitg1 = true;
      }
    }
  } else {
    isvalid = false;
  }

  return isvalid;
}

static void ArmReverseKinematics_rand_e(real_T r[5])
{
  real_T b_r;
  int32_T b_k;
  int32_T exitg1;
  uint32_T b_u[2];
  for (b_k = 0; b_k < 5; b_k++) {
    /* ========================= COPYRIGHT NOTICE ============================ */
    /*  This is a uniform (0,1) pseudorandom number generator based on:        */
    /*                                                                         */
    /*  A C-program for MT19937, with initialization improved 2002/1/26.       */
    /*  Coded by Takuji Nishimura and Makoto Matsumoto.                        */
    /*                                                                         */
    /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      */
    /*  All rights reserved.                                                   */
    /*                                                                         */
    /*  Redistribution and use in source and binary forms, with or without     */
    /*  modification, are permitted provided that the following conditions     */
    /*  are met:                                                               */
    /*                                                                         */
    /*    1. Redistributions of source code must retain the above copyright    */
    /*       notice, this list of conditions and the following disclaimer.     */
    /*                                                                         */
    /*    2. Redistributions in binary form must reproduce the above copyright */
    /*       notice, this list of conditions and the following disclaimer      */
    /*       in the documentation and/or other materials provided with the     */
    /*       distribution.                                                     */
    /*                                                                         */
    /*    3. The names of its contributors may not be used to endorse or       */
    /*       promote products derived from this software without specific      */
    /*       prior written permission.                                         */
    /*                                                                         */
    /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
    /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
    /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
    /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  */
    /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
    /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
    /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
    /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
    /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
    /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
    /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */
    /*                                                                         */
    /* =============================   END   ================================= */
    do {
      exitg1 = 0;
      ArmReve_genrand_uint32_vector_e(rtDW.state_i, b_u);
      b_r = ((real_T)(b_u[0] >> 5U) * 6.7108864E+7 + (real_T)(b_u[1] >> 6U)) *
        1.1102230246251565E-16;
      if (b_r == 0.0) {
        if (!ArmReverseKinema_is_valid_state(rtDW.state_i)) {
          rtDW.state_i[0] = 5489U;
          rtDW.state_i[624] = 624U;
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    r[b_k] = b_r;
  }
}

static void ArmRev_emxEnsureCapacity_char_T(emxArray_char_T_ArmReverseKin_T
  *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = malloc((uint32_T)i * sizeof(char_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void ArmReverseKinema_emxFree_char_T(emxArray_char_T_ArmReverseKin_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T_ArmReverseKin_T *)NULL) {
    if (((*pEmxArray)->data != (char_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_char_T_ArmReverseKin_T *)NULL;
  }
}

static void ArmRev_emxEnsureCapacity_real_T(emxArray_real_T_ArmReverseKin_T
  *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = malloc((uint32_T)i * sizeof(real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void emxEnsureCapacity_m_robotics_ma(emxArray_m_robotics_manip_int_T
  *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = malloc((uint32_T)i * sizeof(m_robotics_manip_internal_Col_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(m_robotics_manip_internal_Col_T)
             * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (m_robotics_manip_internal_Col_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void emxFree_m_robotics_manip_intern(emxArray_m_robotics_manip_int_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_m_robotics_manip_int_T *)NULL) {
    if (((*pEmxArray)->data != (m_robotics_manip_internal_Col_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_m_robotics_manip_int_T *)NULL;
  }
}

static n_robotics_manip_internal_Col_T *Arm_CollisionSet_CollisionSet_e
  (n_robotics_manip_internal_Col_T *obj, real_T maxElements)
{
  static const void *GeometryInternal = NULL;
  emxArray_m_robotics_manip_int_T *e;
  n_robotics_manip_internal_Col_T *b_obj;
  real_T c;
  int32_T b_i;
  int32_T d;
  int8_T localPose[16];
  static const int8_T tmp[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  int32_T i;
  obj->Size = 0.0;
  b_obj = obj;
  obj->MaxElements = maxElements;
  emxInit_m_robotics_manip_intern(&e, 2);
  i = e->size[0] * e->size[1];
  e->size[1] = (int32_T)obj->MaxElements;
  emxEnsureCapacity_m_robotics_ma(e, i);
  i = obj->CollisionGeometries->size[0] * obj->CollisionGeometries->size[1];
  obj->CollisionGeometries->size[0] = 1;
  obj->CollisionGeometries->size[1] = e->size[1];
  emxFree_m_robotics_manip_intern(&e);
  emxEnsureCapacity_m_robotics_ma(obj->CollisionGeometries, i);
  for (i = 0; i < 16; i++) {
    localPose[i] = tmp[i];
  }

  c = obj->MaxElements;
  d = (int32_T)c - 1;
  for (b_i = 0; b_i <= d; b_i++) {
    obj->CollisionGeometries->data[b_i].CollisionPrimitive = (void *)
      GeometryInternal;
    for (i = 0; i < 16; i++) {
      obj->CollisionGeometries->data[b_i].LocalPose[i] = localPose[i];
    }

    for (i = 0; i < 16; i++) {
      obj->CollisionGeometries->data[b_i].WorldPose[i] = localPose[i];
    }

    obj->CollisionGeometries->data[b_i].MeshScale[0] = 1.0;
    obj->CollisionGeometries->data[b_i].MeshScale[1] = 1.0;
    obj->CollisionGeometries->data[b_i].MeshScale[2] = 1.0;
  }

  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static v_robotics_manip_internal_Rig_T *ArmRever_RigidBody_RigidBody_ev
  (v_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_ArmReverse_e_T *iobj_1)
{
  emxArray_char_T_ArmReverseKin_T *switch_expression;
  v_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T a_0[9];
  char_T a[8];
  int8_T msubspace_data[36];
  int8_T b_I[9];
  int8_T tmp[6];
  int8_T tmp_0;
  boolean_T result;
  static const char_T tmp_1[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1' };

  static const int8_T tmp_2[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_3[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1', '_', 'j', 'n', 't' };

  static const char_T tmp_4[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  ArmRev_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_1[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    tmp_0 = tmp_2[b_kstr];
    iobj_1->JointToParentTransform[b_kstr] = tmp_0;
    iobj_1->ChildToJointTransform[b_kstr] = tmp_0;
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  ArmRev_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_4[b_kstr];
  }

  ArmReverseKinema_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    a[b_kstr] = tmp_5[b_kstr];
  }

  result = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_6[b_kstr];
    }

    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  ArmReverseKinema_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj->MassInternal = 1.0;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    b_I[b_kstr] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = b_I[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    msubspace_data[b_kstr] = 0;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    msubspace_data[b_kstr + 6 * b_kstr] = 1;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = msubspace_data[b_kstr];
  }

  obj->CollisionsInternal = Arm_CollisionSet_CollisionSet_e(iobj_0, 0.0);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static v_robotics_manip_internal_Rig_T *ArmReve_RigidBody_RigidBody_evf
  (v_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_ArmReverse_e_T *iobj_1)
{
  emxArray_char_T_ArmReverseKin_T *switch_expression;
  v_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T a_0[9];
  char_T a[8];
  int8_T msubspace_data[36];
  int8_T b_I[9];
  int8_T tmp[6];
  int8_T tmp_0;
  boolean_T result;
  static const char_T tmp_1[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '2' };

  static const int8_T tmp_2[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_3[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '2', '_', 'j', 'n', 't' };

  static const char_T tmp_4[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  ArmRev_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_1[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    tmp_0 = tmp_2[b_kstr];
    iobj_1->JointToParentTransform[b_kstr] = tmp_0;
    iobj_1->ChildToJointTransform[b_kstr] = tmp_0;
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  ArmRev_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_4[b_kstr];
  }

  ArmReverseKinema_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    a[b_kstr] = tmp_5[b_kstr];
  }

  result = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_6[b_kstr];
    }

    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  ArmReverseKinema_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj->MassInternal = 1.0;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    b_I[b_kstr] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = b_I[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    msubspace_data[b_kstr] = 0;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    msubspace_data[b_kstr + 6 * b_kstr] = 1;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = msubspace_data[b_kstr];
  }

  obj->CollisionsInternal = Arm_CollisionSet_CollisionSet_e(iobj_0, 0.0);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static v_robotics_manip_internal_Rig_T *ArmRev_RigidBody_RigidBody_evfm
  (v_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_ArmReverse_e_T *iobj_1)
{
  emxArray_char_T_ArmReverseKin_T *switch_expression;
  v_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T a_0[9];
  char_T a[8];
  int8_T msubspace_data[36];
  int8_T b_I[9];
  int8_T tmp[6];
  int8_T tmp_0;
  boolean_T result;
  static const char_T tmp_1[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '3' };

  static const int8_T tmp_2[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_3[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '3', '_', 'j', 'n', 't' };

  static const char_T tmp_4[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  ArmRev_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_1[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    tmp_0 = tmp_2[b_kstr];
    iobj_1->JointToParentTransform[b_kstr] = tmp_0;
    iobj_1->ChildToJointTransform[b_kstr] = tmp_0;
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  ArmRev_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_4[b_kstr];
  }

  ArmReverseKinema_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    a[b_kstr] = tmp_5[b_kstr];
  }

  result = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_6[b_kstr];
    }

    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  ArmReverseKinema_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj->MassInternal = 1.0;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    b_I[b_kstr] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = b_I[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    msubspace_data[b_kstr] = 0;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    msubspace_data[b_kstr + 6 * b_kstr] = 1;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = msubspace_data[b_kstr];
  }

  obj->CollisionsInternal = Arm_CollisionSet_CollisionSet_e(iobj_0, 0.0);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static v_robotics_manip_internal_Rig_T *ArmRe_RigidBody_RigidBody_evfmf
  (v_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_ArmReverse_e_T *iobj_1)
{
  emxArray_char_T_ArmReverseKin_T *switch_expression;
  v_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T a_0[9];
  char_T a[8];
  int8_T msubspace_data[36];
  int8_T b_I[9];
  int8_T tmp[6];
  int8_T tmp_0;
  boolean_T result;
  static const char_T tmp_1[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '4' };

  static const int8_T tmp_2[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_3[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '4', '_', 'j', 'n', 't' };

  static const char_T tmp_4[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  ArmRev_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_1[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    tmp_0 = tmp_2[b_kstr];
    iobj_1->JointToParentTransform[b_kstr] = tmp_0;
    iobj_1->ChildToJointTransform[b_kstr] = tmp_0;
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  ArmRev_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_4[b_kstr];
  }

  ArmReverseKinema_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    a[b_kstr] = tmp_5[b_kstr];
  }

  result = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_6[b_kstr];
    }

    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  ArmReverseKinema_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj->MassInternal = 1.0;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    b_I[b_kstr] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = b_I[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    msubspace_data[b_kstr] = 0;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    msubspace_data[b_kstr + 6 * b_kstr] = 1;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = msubspace_data[b_kstr];
  }

  obj->CollisionsInternal = Arm_CollisionSet_CollisionSet_e(iobj_0, 0.0);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static v_robotics_manip_internal_Rig_T *ArmR_RigidBody_RigidBody_evfmfk
  (v_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_ArmReverse_e_T *iobj_1)
{
  emxArray_char_T_ArmReverseKin_T *switch_expression;
  v_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T a_0[9];
  char_T a[8];
  int8_T msubspace_data[36];
  int8_T b_I[9];
  int8_T tmp[6];
  int8_T tmp_0;
  boolean_T result;
  static const char_T tmp_1[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '5' };

  static const int8_T tmp_2[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_3[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '5', '_', 'j', 'n', 't' };

  static const char_T tmp_4[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 10;
  ArmRev_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_1[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    tmp_0 = tmp_2[b_kstr];
    iobj_1->JointToParentTransform[b_kstr] = tmp_0;
    iobj_1->ChildToJointTransform[b_kstr] = tmp_0;
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 14;
  ArmRev_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 14; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_4[b_kstr];
  }

  ArmReverseKinema_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    a[b_kstr] = tmp_5[b_kstr];
  }

  result = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_6[b_kstr];
    }

    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  ArmReverseKinema_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj->MassInternal = 1.0;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    b_I[b_kstr] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = b_I[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    msubspace_data[b_kstr] = 0;
  }

  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    msubspace_data[b_kstr + 6 * b_kstr] = 1;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = msubspace_data[b_kstr];
  }

  obj->CollisionsInternal = Arm_CollisionSet_CollisionSet_e(iobj_0, 0.0);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static v_robotics_manip_internal_Rig_T *Arm_RigidBody_RigidBody_evfmfk1
  (v_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_ArmReverse_e_T *iobj_1)
{
  emxArray_char_T_ArmReverseKin_T *switch_expression;
  v_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T a_0[9];
  char_T a[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  int8_T tmp_0;
  boolean_T result;
  static const char_T tmp_1[5] = { 'B', 'o', 'd', 'y', '3' };

  static const real_T tmp_2[9] = { 0.00083333333333333371, 0.0, 0.0, 0.0,
    0.067333333333333356, 0.0, 0.0, 0.0, 0.066833333333333356 };

  static const real_T tmp_3[36] = { 0.00083333333333333371, 0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0, 0.067333333333333356, 0.0, -0.0, 0.0, -0.10000000000000002, 0.0,
    0.0, 0.066833333333333356, 0.0, 0.10000000000000002, 0.0, 0.0, -0.0, 0.0,
    0.20000000000000004, 0.0, 0.0, 0.0, 0.0, 0.10000000000000002, 0.0,
    0.20000000000000004, 0.0, -0.0, -0.10000000000000002, 0.0, 0.0, 0.0,
    0.20000000000000004 };

  static const int8_T tmp_4[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_5[6] = { 'J', 'o', 'i', 'n', 't', '3' };

  static const char_T tmp_6[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_7[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_8[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_9[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_a[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_1[b_kstr];
  }

  obj->ParentIndex = 2.0;
  obj->MassInternal = 0.20000000000000004;
  obj->CenterOfMassInternal[0] = 0.5;
  obj->CenterOfMassInternal[1] = -0.0;
  obj->CenterOfMassInternal[2] = -0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_3[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    tmp_0 = tmp_4[b_kstr];
    iobj_1->JointToParentTransform[b_kstr] = tmp_0;
    iobj_1->ChildToJointTransform[b_kstr] = tmp_0;
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 6;
  ArmRev_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_5[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 8;
  ArmRev_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_6[b_kstr];
  }

  ArmReverseKinema_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    a[b_kstr] = tmp_6[b_kstr];
  }

  result = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_7[b_kstr];
    }

    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  ArmReverseKinema_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->JointToParentTransform[b_kstr] = tmp_8[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->ChildToJointTransform[b_kstr] = tmp_9[b_kstr];
  }

  b_kstr = obj->JointInternal->MotionSubspace->size[0] * obj->
    JointInternal->MotionSubspace->size[1];
  obj->JointInternal->MotionSubspace->size[0] = 6;
  obj->JointInternal->MotionSubspace->size[1] = 1;
  ArmRev_emxEnsureCapacity_real_T(obj->JointInternal->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal->MotionSubspace->data[b_kstr] = tmp_a[b_kstr];
  }

  obj->JointInternal->InTree = true;
  b_kstr = obj->JointInternal->PositionLimitsInternal->size[0] *
    obj->JointInternal->PositionLimitsInternal->size[1];
  obj->JointInternal->PositionLimitsInternal->size[0] = 1;
  obj->JointInternal->PositionLimitsInternal->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(obj->JointInternal->PositionLimitsInternal,
    b_kstr);
  obj->JointInternal->PositionLimitsInternal->data[0] = -1.5707963267948966;
  obj->JointInternal->PositionLimitsInternal->data[obj->
    JointInternal->PositionLimitsInternal->size[0]] = 1.5707963267948966;
  obj->JointInternal->JointAxisInternal[0] = 0.0;
  obj->JointInternal->JointAxisInternal[1] = 0.0;
  obj->JointInternal->JointAxisInternal[2] = 1.0;
  b_kstr = obj->JointInternal->HomePositionInternal->size[0];
  obj->JointInternal->HomePositionInternal->size[0] = 1;
  ArmRev_emxEnsureCapacity_real_T(obj->JointInternal->HomePositionInternal,
    b_kstr);
  obj->JointInternal->HomePositionInternal->data[0] = 0.0;
  obj->CollisionsInternal = Arm_CollisionSet_CollisionSet_e(iobj_0, 0.0);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static v_robotics_manip_internal_Rig_T *Ar_RigidBody_RigidBody_evfmfk1h
  (v_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_ArmReverse_e_T *iobj_1)
{
  emxArray_char_T_ArmReverseKin_T *switch_expression;
  v_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T a_0[9];
  char_T a[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  int8_T tmp_0;
  boolean_T result;
  static const char_T tmp_1[5] = { 'B', 'o', 'd', 'y', '4' };

  static const real_T tmp_2[9] = { 0.00083333333333333371, 0.0, 0.0, 0.0,
    0.067333333333333356, 0.0, 0.0, 0.0, 0.066833333333333356 };

  static const real_T tmp_3[36] = { 0.00083333333333333371, 0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0, 0.067333333333333356, 0.0, -0.0, 0.0, -0.10000000000000002, 0.0,
    0.0, 0.066833333333333356, 0.0, 0.10000000000000002, 0.0, 0.0, -0.0, 0.0,
    0.20000000000000004, 0.0, 0.0, 0.0, 0.0, 0.10000000000000002, 0.0,
    0.20000000000000004, 0.0, -0.0, -0.10000000000000002, 0.0, 0.0, 0.0,
    0.20000000000000004 };

  static const int8_T tmp_4[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_5[6] = { 'J', 'o', 'i', 'n', 't', '4' };

  static const char_T tmp_6[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_7[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_8[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_9[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_a[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_1[b_kstr];
  }

  obj->ParentIndex = 3.0;
  obj->MassInternal = 0.20000000000000004;
  obj->CenterOfMassInternal[0] = 0.5;
  obj->CenterOfMassInternal[1] = -0.0;
  obj->CenterOfMassInternal[2] = -0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_3[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    tmp_0 = tmp_4[b_kstr];
    iobj_1->JointToParentTransform[b_kstr] = tmp_0;
    iobj_1->ChildToJointTransform[b_kstr] = tmp_0;
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 6;
  ArmRev_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_5[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 8;
  ArmRev_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_6[b_kstr];
  }

  ArmReverseKinema_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    a[b_kstr] = tmp_6[b_kstr];
  }

  result = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_7[b_kstr];
    }

    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  ArmReverseKinema_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->JointToParentTransform[b_kstr] = tmp_8[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->ChildToJointTransform[b_kstr] = tmp_9[b_kstr];
  }

  b_kstr = obj->JointInternal->MotionSubspace->size[0] * obj->
    JointInternal->MotionSubspace->size[1];
  obj->JointInternal->MotionSubspace->size[0] = 6;
  obj->JointInternal->MotionSubspace->size[1] = 1;
  ArmRev_emxEnsureCapacity_real_T(obj->JointInternal->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal->MotionSubspace->data[b_kstr] = tmp_a[b_kstr];
  }

  obj->JointInternal->InTree = true;
  b_kstr = obj->JointInternal->PositionLimitsInternal->size[0] *
    obj->JointInternal->PositionLimitsInternal->size[1];
  obj->JointInternal->PositionLimitsInternal->size[0] = 1;
  obj->JointInternal->PositionLimitsInternal->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(obj->JointInternal->PositionLimitsInternal,
    b_kstr);
  obj->JointInternal->PositionLimitsInternal->data[0] = -1.5707963267948966;
  obj->JointInternal->PositionLimitsInternal->data[obj->
    JointInternal->PositionLimitsInternal->size[0]] = 1.5707963267948966;
  obj->JointInternal->JointAxisInternal[0] = 0.0;
  obj->JointInternal->JointAxisInternal[1] = 0.0;
  obj->JointInternal->JointAxisInternal[2] = 1.0;
  b_kstr = obj->JointInternal->HomePositionInternal->size[0];
  obj->JointInternal->HomePositionInternal->size[0] = 1;
  ArmRev_emxEnsureCapacity_real_T(obj->JointInternal->HomePositionInternal,
    b_kstr);
  obj->JointInternal->HomePositionInternal->data[0] = 0.0;
  obj->CollisionsInternal = Arm_CollisionSet_CollisionSet_e(iobj_0, 0.0);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static v_robotics_manip_internal_Rig_T *A_RigidBody_RigidBody_evfmfk1h2
  (v_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_ArmReverse_e_T *iobj_1)
{
  emxArray_char_T_ArmReverseKin_T *switch_expression;
  v_robotics_manip_internal_Rig_T *b_obj;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T a_0[9];
  char_T a[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  int8_T tmp_0;
  boolean_T result;
  static const char_T tmp_1[5] = { 'B', 'o', 'd', 'y', '5' };

  static const real_T tmp_2[9] = { 0.00016666666666666674, 0.0, 0.0, 0.0,
    0.000666666666666667, 0.0, 0.0, 0.0, 0.00056666666666666692 };

  static const real_T tmp_3[36] = { 0.00016666666666666674, 0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0, 0.000666666666666667, 0.0, -0.0, 0.0, -0.004000000000000001, 0.0,
    0.0, 0.00056666666666666692, 0.0, 0.004000000000000001, 0.0, 0.0, -0.0, 0.0,
    0.040000000000000008, 0.0, 0.0, 0.0, 0.0, 0.004000000000000001, 0.0,
    0.040000000000000008, 0.0, -0.0, -0.004000000000000001, 0.0, 0.0, 0.0,
    0.040000000000000008 };

  static const int8_T tmp_4[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_5[6] = { 'J', 'o', 'i', 'n', 't', '5' };

  static const char_T tmp_6[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_7[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_8[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_9[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_a[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp_1[b_kstr];
  }

  obj->ParentIndex = 4.0;
  obj->MassInternal = 0.040000000000000008;
  obj->CenterOfMassInternal[0] = 0.1;
  obj->CenterOfMassInternal[1] = -0.0;
  obj->CenterOfMassInternal[2] = -0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->InertiaInternal[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->SpatialInertia[b_kstr] = tmp_3[b_kstr];
  }

  iobj_1->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    tmp_0 = tmp_4[b_kstr];
    iobj_1->JointToParentTransform[b_kstr] = tmp_0;
    iobj_1->ChildToJointTransform[b_kstr] = tmp_0;
  }

  b_kstr = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 6;
  ArmRev_emxEnsureCapacity_char_T(iobj_1->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->NameInternal->data[b_kstr] = tmp_5[b_kstr];
  }

  b_kstr = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(iobj_1->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_1->Type->data[b_kstr] = tmp_6[b_kstr];
  }

  ArmReverseKinema_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_1->Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_1->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    a[b_kstr] = tmp_7[b_kstr];
  }

  result = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_8[b_kstr];
    }

    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  ArmReverseKinema_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    iobj_1->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    iobj_1->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  ArmRev_emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_kstr);
  iobj_1->HomePositionInternal->data[0] = 0.0;
  obj->JointInternal = iobj_1;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->JointToParentTransform[b_kstr] = tmp_9[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal->ChildToJointTransform[b_kstr] = tmp_a[b_kstr];
  }

  b_kstr = obj->JointInternal->MotionSubspace->size[0] * obj->
    JointInternal->MotionSubspace->size[1];
  obj->JointInternal->MotionSubspace->size[0] = 6;
  obj->JointInternal->MotionSubspace->size[1] = 1;
  ArmRev_emxEnsureCapacity_real_T(obj->JointInternal->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->JointInternal->MotionSubspace->data[b_kstr] = 0.0;
  }

  obj->JointInternal->InTree = true;
  b_kstr = obj->JointInternal->PositionLimitsInternal->size[0] *
    obj->JointInternal->PositionLimitsInternal->size[1];
  obj->JointInternal->PositionLimitsInternal->size[0] = 1;
  obj->JointInternal->PositionLimitsInternal->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(obj->JointInternal->PositionLimitsInternal,
    b_kstr);
  obj->JointInternal->PositionLimitsInternal->data[0] = 0.0;
  obj->JointInternal->PositionLimitsInternal->data[obj->
    JointInternal->PositionLimitsInternal->size[0]] = 0.0;
  obj->JointInternal->JointAxisInternal[0] = 0.0;
  obj->JointInternal->JointAxisInternal[1] = 0.0;
  obj->JointInternal->JointAxisInternal[2] = 0.0;
  b_kstr = obj->JointInternal->HomePositionInternal->size[0];
  obj->JointInternal->HomePositionInternal->size[0] = 1;
  ArmRev_emxEnsureCapacity_real_T(obj->JointInternal->HomePositionInternal,
    b_kstr);
  obj->JointInternal->HomePositionInternal->data[0] = 0.0;
  obj->CollisionsInternal = Arm_CollisionSet_CollisionSet_e(iobj_0, 0.0);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static c_rigidBodyJoint_ArmReverse_e_T *A_rigidBodyJoint_rigidBodyJoint
  (c_rigidBodyJoint_ArmReverse_e_T *obj, const emxArray_char_T_ArmReverseKin_T
   *jname)
{
  c_rigidBodyJoint_ArmReverse_e_T *b_obj;
  emxArray_char_T_ArmReverseKin_T *switch_expression;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T a_0[9];
  char_T a[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  int8_T tmp_0;
  boolean_T result;
  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_2[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_3[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_4[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  obj->InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    tmp_0 = tmp_1[b_kstr];
    obj->JointToParentTransform[b_kstr] = tmp_0;
    obj->ChildToJointTransform[b_kstr] = tmp_0;
  }

  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = jname->size[1];
  ArmRev_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  loop_ub = jname->size[1] - 1;
  for (b_kstr = 0; b_kstr <= loop_ub; b_kstr++) {
    obj->NameInternal->data[b_kstr] = jname->data[b_kstr];
  }

  b_kstr = obj->Type->size[0] * obj->Type->size[1];
  obj->Type->size[0] = 1;
  obj->Type->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(obj->Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->Type->data[b_kstr] = tmp_2[b_kstr];
  }

  ArmReverseKinema_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    a[b_kstr] = tmp_3[b_kstr];
  }

  result = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_4[b_kstr];
    }

    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  ArmReverseKinema_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    obj->VelocityNumber = 0.0;
    obj->PositionNumber = 0.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->MotionSubspace->size[0] * obj->MotionSubspace->size[1];
  obj->MotionSubspace->size[0] = 6;
  obj->MotionSubspace->size[1] = 1;
  ArmRev_emxEnsureCapacity_real_T(obj->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->MotionSubspace->data[b_kstr] = msubspace_data[b_kstr];
  }

  b_kstr = obj->PositionLimitsInternal->size[0] * obj->
    PositionLimitsInternal->size[1];
  obj->PositionLimitsInternal->size[0] = 1;
  obj->PositionLimitsInternal->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(obj->PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->PositionLimitsInternal->data[b_kstr] = poslim_data[b_kstr];
  }

  b_kstr = obj->HomePositionInternal->size[0];
  obj->HomePositionInternal->size[0] = 1;
  ArmRev_emxEnsureCapacity_real_T(obj->HomePositionInternal, b_kstr);
  obj->HomePositionInternal->data[0] = 0.0;
  return b_obj;
}

static boolean_T ArmReverseKinematics_strcmp(const
  emxArray_char_T_ArmReverseKin_T *a, const emxArray_char_T_ArmReverseKin_T *b)
{
  boolean_T b_bool;
  boolean_T d;
  b_bool = false;
  d = (a->size[1] == 0);
  if (d && (b->size[1] == 0)) {
    b_bool = true;
  } else if (a->size[1] != b->size[1]) {
  } else {
    int32_T b_kstr;
    b_kstr = 1;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 <= b->size[1] - 1) {
        if (a->data[b_kstr - 1] != b->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

static real_T RigidBodyTree_findBodyIndexByNa(x_robotics_manip_internal_Rig_T
  *obj, const emxArray_char_T_ArmReverseKin_T *bodyname)
{
  emxArray_char_T_ArmReverseKin_T *bname;
  v_robotics_manip_internal_Rig_T *obj_0;
  real_T b;
  real_T bid;
  int32_T b_i;
  int32_T i;
  int32_T loop_ub;
  boolean_T exitg1;
  bid = -1.0;
  ArmReverseKinema_emxInit_char_T(&bname, 2);
  i = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj->Base.NameInternal->size[1];
  ArmRev_emxEnsureCapacity_char_T(bname, i);
  loop_ub = obj->Base.NameInternal->size[1];
  for (i = 0; i < loop_ub; i++) {
    bname->data[i] = obj->Base.NameInternal->data[i];
  }

  if (ArmReverseKinematics_strcmp(bname, bodyname)) {
    bid = 0.0;
  } else {
    b = obj->NumBodies;
    b_i = 0;
    exitg1 = false;
    while ((!exitg1) && (b_i <= (int32_T)b - 1)) {
      obj_0 = obj->Bodies[b_i];
      i = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = obj_0->NameInternal->size[1];
      ArmRev_emxEnsureCapacity_char_T(bname, i);
      loop_ub = obj_0->NameInternal->size[1];
      for (i = 0; i < loop_ub; i++) {
        bname->data[i] = obj_0->NameInternal->data[i];
      }

      if (ArmReverseKinematics_strcmp(bname, bodyname)) {
        bid = (real_T)b_i + 1.0;
        exitg1 = true;
      } else {
        b_i++;
      }
    }
  }

  ArmReverseKinema_emxFree_char_T(&bname);
  return bid;
}

static void ArmReverseKinema_emxFree_real_T(emxArray_real_T_ArmReverseKin_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T_ArmReverseKin_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T_ArmReverseKin_T *)NULL;
  }
}

static v_robotics_manip_internal_Rig_T *ArmReverseKinema_RigidBody_copy
  (v_robotics_manip_internal_Rig_T *obj, n_robotics_manip_internal_Col_T *iobj_0,
   c_rigidBodyJoint_ArmReverse_e_T *iobj_1, v_robotics_manip_internal_Rig_T
   *iobj_2)
{
  void *copyGeometryInternal;
  c_rigidBodyJoint_ArmReverse_e_T *obj_0;
  emxArray_char_T_ArmReverseKin_T *jname;
  emxArray_char_T_ArmReverseKin_T *jtype;
  emxArray_real_T_ArmReverseKin_T *obj_2;
  m_robotics_manip_internal_Col_T tmp;
  n_robotics_manip_internal_Col_T *newObj;
  n_robotics_manip_internal_Col_T *obj_1;
  v_robotics_manip_internal_Rig_T *newbody;
  real_T obj_5[36];
  real_T obj_3[16];
  real_T poslim_data[12];
  real_T obj_4[9];
  real_T obj_idx_1;
  real_T obj_idx_2;
  real_T y;
  int32_T b_k;
  int32_T minnanb;
  int32_T u1;
  char_T b_b_0[9];
  char_T b_vstr[9];
  char_T partial_match_data[9];
  char_T b_b[8];
  char_T vstr[8];
  char_T b_b_1[5];
  char_T c_vstr[5];
  int8_T msubspace_data[36];
  int8_T b_I[9];
  int8_T tmp_0[6];
  int8_T tmp_1;
  boolean_T b_bool;
  boolean_T matched;
  static const int8_T tmp_2[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_3[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_4[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_5[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_6[128] = { '\x00', '\x01', '\x02', '\x03', '\x04',
    '\x05', '\x06', '\a', '\b', '\t', '\n', '\v', '\f', '\r', '\x0e', '\x0f',
    '\x10', '\x11', '\x12', '\x13', '\x14', '\x15', '\x16', '\x17', '\x18',
    '\x19', '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ', '!', '\"', '#',
    '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/', '0', '1', '2',
    '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?', '@', 'a',
    'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p',
    'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[', '\\', ']', '^', '_',
    '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
    'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '{', '|', '}',
    '~', '\x7f' };

  int32_T exitg1;
  int32_T partial_match_size_idx_1;
  boolean_T guard1;
  boolean_T guard11;
  boolean_T guard2;
  boolean_T guard3;
  ArmReverseKinema_emxInit_char_T(&jtype, 2);
  u1 = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = obj->NameInternal->size[1];
  ArmRev_emxEnsureCapacity_char_T(jtype, u1);
  minnanb = obj->NameInternal->size[1];
  for (u1 = 0; u1 < minnanb; u1++) {
    jtype->data[u1] = obj->NameInternal->data[u1];
  }

  newbody = iobj_2;
  u1 = iobj_2->NameInternal->size[0] * iobj_2->NameInternal->size[1];
  iobj_2->NameInternal->size[0] = 1;
  iobj_2->NameInternal->size[1] = jtype->size[1];
  ArmRev_emxEnsureCapacity_char_T(iobj_2->NameInternal, u1);
  minnanb = jtype->size[1] - 1;
  for (u1 = 0; u1 <= minnanb; u1++) {
    iobj_2->NameInternal->data[u1] = jtype->data[u1];
  }

  ArmReverseKinema_emxInit_char_T(&jname, 2);
  u1 = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = jtype->size[1] + 4;
  ArmRev_emxEnsureCapacity_char_T(jname, u1);
  minnanb = jtype->size[1];
  if (minnanb - 1 >= 0) {
    memcpy(&jname->data[0], &jtype->data[0], (uint32_T)minnanb * sizeof(char_T));
  }

  jname->data[jtype->size[1]] = '_';
  jname->data[jtype->size[1] + 1] = 'j';
  jname->data[jtype->size[1] + 2] = 'n';
  jname->data[jtype->size[1] + 3] = 't';
  iobj_2->JointInternal = A_rigidBodyJoint_rigidBodyJoint(&iobj_1[0], jname);
  iobj_2->Index = -1.0;
  iobj_2->ParentIndex = -1.0;
  iobj_2->MassInternal = 1.0;
  iobj_2->CenterOfMassInternal[0] = 0.0;
  iobj_2->CenterOfMassInternal[1] = 0.0;
  iobj_2->CenterOfMassInternal[2] = 0.0;
  for (u1 = 0; u1 < 9; u1++) {
    b_I[u1] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (u1 = 0; u1 < 9; u1++) {
    iobj_2->InertiaInternal[u1] = b_I[u1];
  }

  for (u1 = 0; u1 < 36; u1++) {
    msubspace_data[u1] = 0;
  }

  for (b_k = 0; b_k < 6; b_k++) {
    msubspace_data[b_k + 6 * b_k] = 1;
  }

  for (u1 = 0; u1 < 36; u1++) {
    iobj_2->SpatialInertia[u1] = msubspace_data[u1];
  }

  iobj_2->CollisionsInternal = Arm_CollisionSet_CollisionSet_e(&iobj_0[0], 0.0);
  iobj_2->matlabCodegenIsDeleted = false;
  obj_0 = obj->JointInternal;
  u1 = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = obj_0->Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(jtype, u1);
  minnanb = obj_0->Type->size[1];
  for (u1 = 0; u1 < minnanb; u1++) {
    jtype->data[u1] = obj_0->Type->data[u1];
  }

  u1 = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = obj_0->NameInternal->size[1];
  ArmRev_emxEnsureCapacity_char_T(jname, u1);
  minnanb = obj_0->NameInternal->size[1];
  for (u1 = 0; u1 < minnanb; u1++) {
    jname->data[u1] = obj_0->NameInternal->data[u1];
  }

  iobj_1[1].InTree = false;
  for (u1 = 0; u1 < 16; u1++) {
    tmp_1 = tmp_2[u1];
    iobj_1[1].JointToParentTransform[u1] = tmp_1;
    iobj_1[1].ChildToJointTransform[u1] = tmp_1;
  }

  u1 = iobj_1[1].NameInternal->size[0] * iobj_1[1].NameInternal->size[1];
  iobj_1[1].NameInternal->size[0] = 1;
  iobj_1[1].NameInternal->size[1] = jname->size[1];
  ArmRev_emxEnsureCapacity_char_T(iobj_1[1].NameInternal, u1);
  minnanb = jname->size[1] - 1;
  for (u1 = 0; u1 <= minnanb; u1++) {
    iobj_1[1].NameInternal->data[u1] = jname->data[u1];
  }

  ArmReverseKinema_emxFree_char_T(&jname);
  partial_match_size_idx_1 = 8;
  b_k = 0;
  matched = false;
  for (u1 = 0; u1 < 8; u1++) {
    partial_match_data[u1] = ' ';
    vstr[u1] = tmp_3[u1];
  }

  guard1 = false;
  guard2 = false;
  guard3 = false;
  if (jtype->size[1] <= 8) {
    for (u1 = 0; u1 < 8; u1++) {
      b_b[u1] = tmp_3[u1];
    }

    b_bool = false;
    minnanb = jtype->size[1];
    u1 = jtype->size[1];
    guard11 = false;
    if (u1 <= minnanb) {
      if (minnanb > u1) {
        minnanb = u1;
      }

      guard11 = true;
    } else if (jtype->size[1] == 8) {
      minnanb = 8;
      guard11 = true;
    }

    if (guard11) {
      u1 = 1;
      do {
        exitg1 = 0;
        if (u1 - 1 <= minnanb - 1) {
          if (tmp_6[(uint8_T)jtype->data[u1 - 1] & 127] != tmp_6[(int32_T)b_b[u1
              - 1]]) {
            exitg1 = 1;
          } else {
            u1++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      if (jtype->size[1] == 8) {
        b_k = 1;
        partial_match_size_idx_1 = 8;
        for (u1 = 0; u1 < 8; u1++) {
          b_b_0[u1] = vstr[u1];
        }
      } else {
        partial_match_size_idx_1 = 8;
        for (u1 = 0; u1 < 8; u1++) {
          partial_match_data[u1] = vstr[u1];
        }

        matched = true;
        b_k = 1;
        guard3 = true;
      }
    } else {
      guard3 = true;
    }
  } else {
    guard3 = true;
  }

  if (guard3) {
    for (u1 = 0; u1 < 9; u1++) {
      b_vstr[u1] = tmp_4[u1];
    }

    if (jtype->size[1] <= 9) {
      for (u1 = 0; u1 < 9; u1++) {
        b_b_0[u1] = tmp_4[u1];
      }

      b_bool = false;
      minnanb = jtype->size[1];
      u1 = jtype->size[1];
      guard11 = false;
      if (u1 <= minnanb) {
        if (minnanb > u1) {
          minnanb = u1;
        }

        guard11 = true;
      } else if (jtype->size[1] == 9) {
        minnanb = 9;
        guard11 = true;
      }

      if (guard11) {
        u1 = 1;
        do {
          exitg1 = 0;
          if (u1 - 1 <= minnanb - 1) {
            if (tmp_6[(uint8_T)jtype->data[u1 - 1] & 127] != tmp_6[(int32_T)
                b_b_0[u1 - 1]]) {
              exitg1 = 1;
            } else {
              u1++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (b_bool) {
        if (jtype->size[1] == 9) {
          b_k = 1;
          partial_match_size_idx_1 = 9;
          for (u1 = 0; u1 < 9; u1++) {
            b_b_0[u1] = b_vstr[u1];
          }
        } else {
          if (!matched) {
            partial_match_size_idx_1 = 9;
            for (u1 = 0; u1 < 9; u1++) {
              partial_match_data[u1] = b_vstr[u1];
            }
          }

          matched = true;
          b_k++;
          guard2 = true;
        }
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }
  }

  if (guard2) {
    for (u1 = 0; u1 < 5; u1++) {
      c_vstr[u1] = tmp_5[u1];
    }

    if (jtype->size[1] <= 5) {
      for (u1 = 0; u1 < 5; u1++) {
        b_b_1[u1] = tmp_5[u1];
      }

      b_bool = false;
      minnanb = jtype->size[1];
      u1 = jtype->size[1];
      guard11 = false;
      if (u1 <= minnanb) {
        if (minnanb > u1) {
          minnanb = u1;
        }

        guard11 = true;
      } else if (jtype->size[1] == 5) {
        minnanb = 5;
        guard11 = true;
      }

      if (guard11) {
        u1 = 1;
        do {
          exitg1 = 0;
          if (u1 - 1 <= minnanb - 1) {
            if (tmp_6[(uint8_T)jtype->data[u1 - 1] & 127] != tmp_6[(int32_T)
                b_b_1[u1 - 1]]) {
              exitg1 = 1;
            } else {
              u1++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (b_bool) {
        if (jtype->size[1] == 5) {
          b_k = 1;
          partial_match_size_idx_1 = 5;
          for (u1 = 0; u1 < 5; u1++) {
            b_b_0[u1] = c_vstr[u1];
          }
        } else {
          if (!matched) {
            partial_match_size_idx_1 = 5;
            for (u1 = 0; u1 < 5; u1++) {
              partial_match_data[u1] = c_vstr[u1];
            }
          }

          b_k++;
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    if (b_k == 0) {
      partial_match_size_idx_1 = 8;
      for (u1 = 0; u1 < 8; u1++) {
        b_b_0[u1] = ' ';
      }
    } else {
      memcpy(&b_b_0[0], &partial_match_data[0], (uint32_T)
             partial_match_size_idx_1 * sizeof(char_T));
    }
  }

  if ((b_k == 0) || (jtype->size[1] == 0)) {
    partial_match_size_idx_1 = 8;
    for (u1 = 0; u1 < 8; u1++) {
      partial_match_data[u1] = ' ';
    }
  } else {
    memcpy(&partial_match_data[0], &b_b_0[0], (uint32_T)partial_match_size_idx_1
           * sizeof(char_T));
  }

  u1 = iobj_1[1].Type->size[0] * iobj_1[1].Type->size[1];
  iobj_1[1].Type->size[0] = 1;
  iobj_1[1].Type->size[1] = partial_match_size_idx_1;
  ArmRev_emxEnsureCapacity_char_T(iobj_1[1].Type, u1);
  minnanb = partial_match_size_idx_1 - 1;
  for (u1 = 0; u1 <= minnanb; u1++) {
    iobj_1[1].Type->data[u1] = partial_match_data[u1];
  }

  u1 = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = iobj_1[1].Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(jtype, u1);
  minnanb = iobj_1[1].Type->size[1];
  for (u1 = 0; u1 < minnanb; u1++) {
    jtype->data[u1] = iobj_1[1].Type->data[u1];
  }

  matched = false;
  if (jtype->size[1] != 8) {
  } else {
    u1 = 1;
    do {
      exitg1 = 0;
      if (u1 - 1 < 8) {
        if (vstr[u1 - 1] != jtype->data[u1 - 1]) {
          exitg1 = 1;
        } else {
          u1++;
        }
      } else {
        matched = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (matched) {
    y = 0.0;
  } else {
    for (u1 = 0; u1 < 9; u1++) {
      b_vstr[u1] = tmp_4[u1];
    }

    if (jtype->size[1] != 9) {
    } else {
      u1 = 1;
      do {
        exitg1 = 0;
        if (u1 - 1 < 9) {
          if (b_vstr[u1 - 1] != jtype->data[u1 - 1]) {
            exitg1 = 1;
          } else {
            u1++;
          }
        } else {
          matched = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (matched) {
      y = 1.0;
    } else {
      y = -1.0;
    }
  }

  switch ((int32_T)y) {
   case 0:
    tmp_0[0] = 0;
    tmp_0[1] = 0;
    tmp_0[2] = 1;
    tmp_0[3] = 0;
    tmp_0[4] = 0;
    tmp_0[5] = 0;
    for (u1 = 0; u1 < 6; u1++) {
      msubspace_data[u1] = tmp_0[u1];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1[1].VelocityNumber = 1.0;
    iobj_1[1].PositionNumber = 1.0;
    iobj_1[1].JointAxisInternal[0] = 0.0;
    iobj_1[1].JointAxisInternal[1] = 0.0;
    iobj_1[1].JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp_0[0] = 0;
    tmp_0[1] = 0;
    tmp_0[2] = 0;
    tmp_0[3] = 0;
    tmp_0[4] = 0;
    tmp_0[5] = 1;
    for (u1 = 0; u1 < 6; u1++) {
      msubspace_data[u1] = tmp_0[u1];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1[1].VelocityNumber = 1.0;
    iobj_1[1].PositionNumber = 1.0;
    iobj_1[1].JointAxisInternal[0] = 0.0;
    iobj_1[1].JointAxisInternal[1] = 0.0;
    iobj_1[1].JointAxisInternal[2] = 1.0;
    break;

   default:
    for (u1 = 0; u1 < 6; u1++) {
      msubspace_data[u1] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1[1].VelocityNumber = 0.0;
    iobj_1[1].PositionNumber = 0.0;
    iobj_1[1].JointAxisInternal[0] = 0.0;
    iobj_1[1].JointAxisInternal[1] = 0.0;
    iobj_1[1].JointAxisInternal[2] = 0.0;
    break;
  }

  u1 = iobj_1[1].MotionSubspace->size[0] * iobj_1[1].MotionSubspace->size[1];
  iobj_1[1].MotionSubspace->size[0] = 6;
  iobj_1[1].MotionSubspace->size[1] = 1;
  ArmRev_emxEnsureCapacity_real_T(iobj_1[1].MotionSubspace, u1);
  for (u1 = 0; u1 < 6; u1++) {
    iobj_1[1].MotionSubspace->data[u1] = msubspace_data[u1];
  }

  u1 = iobj_1[1].PositionLimitsInternal->size[0] * iobj_1[1].
    PositionLimitsInternal->size[1];
  iobj_1[1].PositionLimitsInternal->size[0] = 1;
  iobj_1[1].PositionLimitsInternal->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(iobj_1[1].PositionLimitsInternal, u1);
  for (u1 = 0; u1 < 2; u1++) {
    iobj_1[1].PositionLimitsInternal->data[u1] = poslim_data[u1];
  }

  u1 = iobj_1[1].HomePositionInternal->size[0];
  iobj_1[1].HomePositionInternal->size[0] = 1;
  ArmRev_emxEnsureCapacity_real_T(iobj_1[1].HomePositionInternal, u1);
  iobj_1[1].HomePositionInternal->data[0] = 0.0;
  u1 = jtype->size[0] * jtype->size[1];
  jtype->size[1] = obj_0->NameInternal->size[1];
  ArmRev_emxEnsureCapacity_char_T(jtype, u1);
  if (jtype->size[1] != 0) {
    u1 = jtype->size[0] * jtype->size[1];
    jtype->size[0] = 1;
    jtype->size[1] = obj_0->NameInternal->size[1];
    ArmRev_emxEnsureCapacity_char_T(jtype, u1);
    minnanb = obj_0->NameInternal->size[1];
    for (u1 = 0; u1 < minnanb; u1++) {
      jtype->data[u1] = obj_0->NameInternal->data[u1];
    }

    if (!iobj_1[1].InTree) {
      u1 = iobj_1[1].NameInternal->size[0] * iobj_1[1].NameInternal->size[1];
      iobj_1[1].NameInternal->size[0] = 1;
      iobj_1[1].NameInternal->size[1] = jtype->size[1];
      ArmRev_emxEnsureCapacity_char_T(iobj_1[1].NameInternal, u1);
      minnanb = jtype->size[1] - 1;
      for (u1 = 0; u1 <= minnanb; u1++) {
        iobj_1[1].NameInternal->data[u1] = jtype->data[u1];
      }
    }
  }

  ArmReverseKinema_emxFree_char_T(&jtype);
  b_k = obj_0->PositionLimitsInternal->size[0] << 1;
  u1 = iobj_1[1].PositionLimitsInternal->size[0] * iobj_1[1].
    PositionLimitsInternal->size[1];
  iobj_1[1].PositionLimitsInternal->size[0] = obj_0->
    PositionLimitsInternal->size[0];
  iobj_1[1].PositionLimitsInternal->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(iobj_1[1].PositionLimitsInternal, u1);
  ArmReverseKinema_emxInit_real_T(&obj_2, 1);
  u1 = obj_2->size[0];
  obj_2->size[0] = b_k;
  ArmRev_emxEnsureCapacity_real_T(obj_2, u1);
  for (u1 = 0; u1 < b_k; u1++) {
    obj_2->data[u1] = obj_0->PositionLimitsInternal->data[u1];
  }

  minnanb = obj_2->size[0];
  for (u1 = 0; u1 < minnanb; u1++) {
    iobj_1[1].PositionLimitsInternal->data[u1] = obj_2->data[u1];
  }

  u1 = obj_2->size[0];
  obj_2->size[0] = obj_0->HomePositionInternal->size[0];
  ArmRev_emxEnsureCapacity_real_T(obj_2, u1);
  minnanb = obj_0->HomePositionInternal->size[0];
  for (u1 = 0; u1 < minnanb; u1++) {
    obj_2->data[u1] = obj_0->HomePositionInternal->data[u1];
  }

  u1 = iobj_1[1].HomePositionInternal->size[0];
  iobj_1[1].HomePositionInternal->size[0] = obj_2->size[0];
  ArmRev_emxEnsureCapacity_real_T(iobj_1[1].HomePositionInternal, u1);
  minnanb = obj_2->size[0];
  for (u1 = 0; u1 < minnanb; u1++) {
    iobj_1[1].HomePositionInternal->data[u1] = obj_2->data[u1];
  }

  y = obj_0->JointAxisInternal[0];
  obj_idx_1 = obj_0->JointAxisInternal[1];
  obj_idx_2 = obj_0->JointAxisInternal[2];
  iobj_1[1].JointAxisInternal[0] = y;
  iobj_1[1].JointAxisInternal[1] = obj_idx_1;
  iobj_1[1].JointAxisInternal[2] = obj_idx_2;
  b_k = 6 * obj_0->MotionSubspace->size[1];
  u1 = iobj_1[1].MotionSubspace->size[0] * iobj_1[1].MotionSubspace->size[1];
  iobj_1[1].MotionSubspace->size[0] = 6;
  iobj_1[1].MotionSubspace->size[1] = obj_0->MotionSubspace->size[1];
  ArmRev_emxEnsureCapacity_real_T(iobj_1[1].MotionSubspace, u1);
  u1 = obj_2->size[0];
  obj_2->size[0] = b_k;
  ArmRev_emxEnsureCapacity_real_T(obj_2, u1);
  for (u1 = 0; u1 < b_k; u1++) {
    obj_2->data[u1] = obj_0->MotionSubspace->data[u1];
  }

  minnanb = obj_2->size[0];
  for (u1 = 0; u1 < minnanb; u1++) {
    iobj_1[1].MotionSubspace->data[u1] = obj_2->data[u1];
  }

  ArmReverseKinema_emxFree_real_T(&obj_2);
  for (u1 = 0; u1 < 16; u1++) {
    obj_3[u1] = obj_0->JointToParentTransform[u1];
  }

  for (u1 = 0; u1 < 16; u1++) {
    iobj_1[1].JointToParentTransform[u1] = obj_3[u1];
  }

  for (u1 = 0; u1 < 16; u1++) {
    obj_3[u1] = obj_0->ChildToJointTransform[u1];
  }

  for (u1 = 0; u1 < 16; u1++) {
    iobj_1[1].ChildToJointTransform[u1] = obj_3[u1];
  }

  iobj_2->JointInternal = &iobj_1[1];
  iobj_2->MassInternal = obj->MassInternal;
  y = obj->CenterOfMassInternal[0];
  obj_idx_1 = obj->CenterOfMassInternal[1];
  obj_idx_2 = obj->CenterOfMassInternal[2];
  iobj_2->CenterOfMassInternal[0] = y;
  iobj_2->CenterOfMassInternal[1] = obj_idx_1;
  iobj_2->CenterOfMassInternal[2] = obj_idx_2;
  for (u1 = 0; u1 < 9; u1++) {
    obj_4[u1] = obj->InertiaInternal[u1];
  }

  for (u1 = 0; u1 < 9; u1++) {
    iobj_2->InertiaInternal[u1] = obj_4[u1];
  }

  for (u1 = 0; u1 < 36; u1++) {
    obj_5[u1] = obj->SpatialInertia[u1];
  }

  for (u1 = 0; u1 < 36; u1++) {
    iobj_2->SpatialInertia[u1] = obj_5[u1];
  }

  obj_1 = obj->CollisionsInternal;
  newObj = Arm_CollisionSet_CollisionSet_e(&iobj_0[1], obj_1->MaxElements);
  newObj->Size = obj_1->Size;
  y = obj_1->Size;
  minnanb = (int32_T)y - 1;
  for (b_k = 0; b_k <= minnanb; b_k++) {
    tmp = obj_1->CollisionGeometries->data[b_k];
    copyGeometryInternal = collisioncodegen_copyGeometry(tmp.CollisionPrimitive);
    newObj->CollisionGeometries->data[b_k].CollisionPrimitive =
      copyGeometryInternal;
    memcpy(&obj_3[0], &tmp.LocalPose[0], sizeof(real_T) << 4U);
    for (u1 = 0; u1 < 16; u1++) {
      newObj->CollisionGeometries->data[b_k].LocalPose[u1] = obj_3[u1];
    }

    memcpy(&obj_3[0], &tmp.WorldPose[0], sizeof(real_T) << 4U);
    for (u1 = 0; u1 < 16; u1++) {
      newObj->CollisionGeometries->data[b_k].WorldPose[u1] = obj_3[u1];
    }

    newObj->CollisionGeometries->data[b_k].MeshScale[0] = tmp.MeshScale[0];
    newObj->CollisionGeometries->data[b_k].MeshScale[1] = tmp.MeshScale[1];
    newObj->CollisionGeometries->data[b_k].MeshScale[2] = tmp.MeshScale[2];
  }

  iobj_2->CollisionsInternal = newObj;
  return newbody;
}

static void ArmRevers_RigidBodyTree_addBody(x_robotics_manip_internal_Rig_T *obj,
  v_robotics_manip_internal_Rig_T *bodyin, const emxArray_char_T_ArmReverseKin_T
  *parentName, n_robotics_manip_internal_Col_T *iobj_0,
  c_rigidBodyJoint_ArmReverse_e_T *iobj_1, v_robotics_manip_internal_Rig_T
  *iobj_2)
{
  c_rigidBodyJoint_ArmReverse_e_T *jnt;
  emxArray_char_T_ArmReverseKin_T *bname;
  v_robotics_manip_internal_Rig_T *body;
  real_T b_index;
  real_T pid;
  int32_T b_kstr;
  int32_T loop_ub;
  char_T b[5];
  boolean_T b_bool;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T exitg1;
  ArmReverseKinema_emxInit_char_T(&bname, 2);
  b_kstr = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = bodyin->NameInternal->size[1];
  ArmRev_emxEnsureCapacity_char_T(bname, b_kstr);
  loop_ub = bodyin->NameInternal->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    bname->data[b_kstr] = bodyin->NameInternal->data[b_kstr];
  }

  RigidBodyTree_findBodyIndexByNa(obj, bname);
  pid = RigidBodyTree_findBodyIndexByNa(obj, parentName);
  b_index = obj->NumBodies + 1.0;
  body = ArmReverseKinema_RigidBody_copy(bodyin, &iobj_0[0], &iobj_1[0], iobj_2);
  obj->Bodies[(int32_T)b_index - 1] = body;
  body->Index = b_index;
  body->ParentIndex = pid;
  body->JointInternal->InTree = true;
  obj->NumBodies++;
  jnt = body->JointInternal;
  b_kstr = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = jnt->Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(bname, b_kstr);
  loop_ub = jnt->Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    bname->data[b_kstr] = jnt->Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    b[b_kstr] = tmp[b_kstr];
  }

  b_bool = false;
  if (bname->size[1] != 5) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 5) {
        if (bname->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  ArmReverseKinema_emxFree_char_T(&bname);
  if (!b_bool) {
    obj->NumNonFixedBodies++;
    jnt = body->JointInternal;
    b_kstr = (int32_T)body->Index - 1;
    obj->PositionDoFMap[b_kstr] = obj->PositionNumber + 1.0;
    obj->PositionDoFMap[b_kstr + 5] = obj->PositionNumber + jnt->PositionNumber;
    jnt = body->JointInternal;
    b_kstr = (int32_T)body->Index - 1;
    obj->VelocityDoFMap[b_kstr] = obj->VelocityNumber + 1.0;
    obj->VelocityDoFMap[b_kstr + 5] = obj->VelocityNumber + jnt->VelocityNumber;
  } else {
    b_kstr = (int32_T)body->Index;
    obj->PositionDoFMap[b_kstr - 1] = 0.0;
    obj->PositionDoFMap[b_kstr + 4] = -1.0;
    b_kstr = (int32_T)body->Index;
    obj->VelocityDoFMap[b_kstr - 1] = 0.0;
    obj->VelocityDoFMap[b_kstr + 4] = -1.0;
  }

  jnt = body->JointInternal;
  obj->PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  obj->VelocityNumber += jnt->VelocityNumber;
}

static void ArmRev_SystemProp_setProperties(b_inverseKinematics_ArmRevers_T *obj,
  w_robotics_manip_internal_Rig_T *varargin_2, c_rigidBodyJoint_ArmReverse_e_T
  *iobj_0, v_robotics_manip_internal_Rig_T *iobj_1,
  n_robotics_manip_internal_Col_T *iobj_2, x_robotics_manip_internal_Rig_T
  *iobj_3, h_robotics_core_internal_Erro_T *iobj_4)
{
  void *copyGeometryInternal;
  emxArray_char_T_ArmReverseKin_T *bname;
  emxArray_char_T_ArmReverseKin_T *switch_expression_0;
  h_robotics_core_internal_Erro_T *obj_0;
  m_robotics_manip_internal_Col_T tmp;
  n_robotics_manip_internal_Col_T *newObj;
  n_robotics_manip_internal_Col_T *obj_1;
  v_robotics_manip_internal_Rig_T *body;
  v_robotics_manip_internal_Rig_T *parent;
  real_T poslim_data[12];
  real_T b;
  real_T bid;
  int32_T loop_ub;
  int32_T ret;
  char_T a[18];
  char_T switch_expression[18];
  char_T a_1[9];
  char_T a_0[8];
  int8_T msubspace_data[36];
  int8_T b_I[9];
  int8_T tmp_0[6];
  boolean_T result;
  static const int8_T tmp_1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_2[8] = { 'b', 'a', 's', 'e', '_', 'j', 'n', 't' };

  static const char_T tmp_3[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_4[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_5[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const sdAmwXbnJnEmimT0NaJRtAD_ArmRe_T tmp_6 = { 0.0,/* tv_sec */
    0.0                                /* tv_nsec */
  };

  static const char_T tmp_7[18] = { 'L', 'e', 'v', 'e', 'n', 'b', 'e', 'r', 'g',
    'M', 'a', 'r', 'q', 'u', 'a', 'r', 'd', 't' };

  real_T unusedExpr[5];
  real_T *tmp_8;
  int32_T exitg1;
  int32_T i;
  boolean_T exitg2;
  ArmReverseKinematics_rand_e(unusedExpr);
  i = iobj_3->Base.NameInternal->size[0] * iobj_3->Base.NameInternal->size[1];
  iobj_3->Base.NameInternal->size[0] = 1;
  iobj_3->Base.NameInternal->size[1] = 4;
  ArmRev_emxEnsureCapacity_char_T(iobj_3->Base.NameInternal, i);
  iobj_3->Base.NameInternal->data[0] = 'b';
  iobj_3->Base.NameInternal->data[1] = 'a';
  iobj_3->Base.NameInternal->data[2] = 's';
  iobj_3->Base.NameInternal->data[3] = 'e';
  iobj_3->_pobj2[0].InTree = false;
  for (i = 0; i < 16; i++) {
    iobj_3->_pobj2[0].JointToParentTransform[i] = tmp_1[i];
  }

  for (i = 0; i < 16; i++) {
    iobj_3->_pobj2[0].ChildToJointTransform[i] = tmp_1[i];
  }

  i = iobj_3->_pobj2[0].NameInternal->size[0] * iobj_3->_pobj2[0]
    .NameInternal->size[1];
  iobj_3->_pobj2[0].NameInternal->size[0] = 1;
  iobj_3->_pobj2[0].NameInternal->size[1] = 8;
  ArmRev_emxEnsureCapacity_char_T(iobj_3->_pobj2[0].NameInternal, i);
  for (i = 0; i < 8; i++) {
    iobj_3->_pobj2[0].NameInternal->data[i] = tmp_2[i];
  }

  i = iobj_3->_pobj2[0].Type->size[0] * iobj_3->_pobj2[0].Type->size[1];
  iobj_3->_pobj2[0].Type->size[0] = 1;
  iobj_3->_pobj2[0].Type->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(iobj_3->_pobj2[0].Type, i);
  for (i = 0; i < 5; i++) {
    iobj_3->_pobj2[0].Type->data[i] = tmp_3[i];
  }

  ArmReverseKinema_emxInit_char_T(&switch_expression_0, 2);
  i = switch_expression_0->size[0] * switch_expression_0->size[1];
  switch_expression_0->size[0] = 1;
  switch_expression_0->size[1] = iobj_3->_pobj2[0].Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression_0, i);
  loop_ub = iobj_3->_pobj2[0].Type->size[1];
  for (i = 0; i < loop_ub; i++) {
    switch_expression_0->data[i] = iobj_3->_pobj2[0].Type->data[i];
  }

  for (i = 0; i < 8; i++) {
    a_0[i] = tmp_4[i];
  }

  result = false;
  if (switch_expression_0->size[1] != 8) {
  } else {
    ret = 1;
    do {
      exitg1 = 0;
      if (ret - 1 < 8) {
        if (a_0[ret - 1] != switch_expression_0->data[ret - 1]) {
          exitg1 = 1;
        } else {
          ret++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    bid = 0.0;
  } else {
    for (i = 0; i < 9; i++) {
      a_1[i] = tmp_5[i];
    }

    if (switch_expression_0->size[1] != 9) {
    } else {
      ret = 1;
      do {
        exitg1 = 0;
        if (ret - 1 < 9) {
          if (a_1[ret - 1] != switch_expression_0->data[ret - 1]) {
            exitg1 = 1;
          } else {
            ret++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      bid = 1.0;
    } else {
      bid = -1.0;
    }
  }

  switch ((int32_T)bid) {
   case 0:
    tmp_0[0] = 0;
    tmp_0[1] = 0;
    tmp_0[2] = 1;
    tmp_0[3] = 0;
    tmp_0[4] = 0;
    tmp_0[5] = 0;
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = tmp_0[i];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_3->_pobj2[0].VelocityNumber = 1.0;
    iobj_3->_pobj2[0].PositionNumber = 1.0;
    iobj_3->_pobj2[0].JointAxisInternal[0] = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[1] = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp_0[0] = 0;
    tmp_0[1] = 0;
    tmp_0[2] = 0;
    tmp_0[3] = 0;
    tmp_0[4] = 0;
    tmp_0[5] = 1;
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = tmp_0[i];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_3->_pobj2[0].VelocityNumber = 1.0;
    iobj_3->_pobj2[0].PositionNumber = 1.0;
    iobj_3->_pobj2[0].JointAxisInternal[0] = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[1] = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[2] = 1.0;
    break;

   default:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_3->_pobj2[0].VelocityNumber = 0.0;
    iobj_3->_pobj2[0].PositionNumber = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[0] = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[1] = 0.0;
    iobj_3->_pobj2[0].JointAxisInternal[2] = 0.0;
    break;
  }

  i = iobj_3->_pobj2[0].MotionSubspace->size[0] * iobj_3->_pobj2[0].
    MotionSubspace->size[1];
  iobj_3->_pobj2[0].MotionSubspace->size[0] = 6;
  iobj_3->_pobj2[0].MotionSubspace->size[1] = 1;
  ArmRev_emxEnsureCapacity_real_T(iobj_3->_pobj2[0].MotionSubspace, i);
  for (i = 0; i < 6; i++) {
    iobj_3->_pobj2[0].MotionSubspace->data[i] = msubspace_data[i];
  }

  i = iobj_3->_pobj2[0].PositionLimitsInternal->size[0] * iobj_3->_pobj2[0].
    PositionLimitsInternal->size[1];
  iobj_3->_pobj2[0].PositionLimitsInternal->size[0] = 1;
  iobj_3->_pobj2[0].PositionLimitsInternal->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(iobj_3->_pobj2[0].PositionLimitsInternal, i);
  for (i = 0; i < 2; i++) {
    iobj_3->_pobj2[0].PositionLimitsInternal->data[i] = poslim_data[i];
  }

  i = iobj_3->_pobj2[0].HomePositionInternal->size[0];
  iobj_3->_pobj2[0].HomePositionInternal->size[0] = 1;
  ArmRev_emxEnsureCapacity_real_T(iobj_3->_pobj2[0].HomePositionInternal, i);
  for (i = 0; i < 1; i++) {
    iobj_3->_pobj2[0].HomePositionInternal->data[0] = 0.0;
  }

  iobj_3->Base.JointInternal = &iobj_3->_pobj2[0];
  iobj_3->Base.Index = -1.0;
  iobj_3->Base.ParentIndex = -1.0;
  iobj_3->Base.MassInternal = 1.0;
  iobj_3->Base.CenterOfMassInternal[0] = 0.0;
  iobj_3->Base.CenterOfMassInternal[1] = 0.0;
  iobj_3->Base.CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    b_I[i] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (i = 0; i < 9; i++) {
    iobj_3->Base.InertiaInternal[i] = b_I[i];
  }

  for (i = 0; i < 36; i++) {
    msubspace_data[i] = 0;
  }

  for (ret = 0; ret < 6; ret++) {
    msubspace_data[ret + 6 * ret] = 1;
  }

  for (i = 0; i < 36; i++) {
    iobj_3->Base.SpatialInertia[i] = msubspace_data[i];
  }

  iobj_3->Base.CollisionsInternal = Arm_CollisionSet_CollisionSet_e
    (&iobj_3->_pobj1[0], 0.0);
  iobj_3->Base.matlabCodegenIsDeleted = false;
  iobj_3->Base.Index = 0.0;
  iobj_3->Bodies[0] = ArmRever_RigidBody_RigidBody_ev(&(&(&iobj_3->_pobj0[0])[0])
    [0], &(&(&iobj_3->_pobj1[1])[0])[0], &(&(&iobj_3->_pobj2[1])[0])[0]);
  iobj_3->Bodies[1] = ArmReve_RigidBody_RigidBody_evf(&(&(&iobj_3->_pobj0[0])[0])
    [1], &(&(&iobj_3->_pobj1[1])[0])[1], &(&(&iobj_3->_pobj2[1])[0])[1]);
  iobj_3->Bodies[2] = ArmRev_RigidBody_RigidBody_evfm(&(&(&iobj_3->_pobj0[0])[0])
    [2], &(&(&iobj_3->_pobj1[1])[0])[2], &(&(&iobj_3->_pobj2[1])[0])[2]);
  iobj_3->Bodies[3] = ArmRe_RigidBody_RigidBody_evfmf(&(&(&iobj_3->_pobj0[0])[0])
    [3], &(&(&iobj_3->_pobj1[1])[0])[3], &(&(&iobj_3->_pobj2[1])[0])[3]);
  iobj_3->Bodies[4] = ArmR_RigidBody_RigidBody_evfmfk(&(&(&iobj_3->_pobj0[0])[0])
    [4], &(&(&iobj_3->_pobj1[1])[0])[4], &(&(&iobj_3->_pobj2[1])[0])[4]);
  iobj_3->NumBodies = 0.0;
  iobj_3->NumNonFixedBodies = 0.0;
  iobj_3->PositionNumber = 0.0;
  iobj_3->VelocityNumber = 0.0;
  ArmReverseKinematics_rand_e(unusedExpr);
  for (i = 0; i < 5; i++) {
    iobj_3->PositionDoFMap[i] = 0.0;
  }

  for (i = 0; i < 5; i++) {
    iobj_3->PositionDoFMap[i + 5] = -1.0;
  }

  for (i = 0; i < 5; i++) {
    iobj_3->VelocityDoFMap[i] = 0.0;
  }

  for (i = 0; i < 5; i++) {
    iobj_3->VelocityDoFMap[i + 5] = -1.0;
  }

  iobj_3->matlabCodegenIsDeleted = false;
  i = switch_expression_0->size[0] * switch_expression_0->size[1];
  switch_expression_0->size[0] = 1;
  switch_expression_0->size[1] = varargin_2->Base.NameInternal->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression_0, i);
  loop_ub = varargin_2->Base.NameInternal->size[1];
  for (i = 0; i < loop_ub; i++) {
    switch_expression_0->data[i] = varargin_2->Base.NameInternal->data[i];
  }

  bid = -1.0;
  ArmReverseKinema_emxInit_char_T(&bname, 2);
  i = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = iobj_3->Base.NameInternal->size[1];
  ArmRev_emxEnsureCapacity_char_T(bname, i);
  loop_ub = iobj_3->Base.NameInternal->size[1];
  for (i = 0; i < loop_ub; i++) {
    bname->data[i] = iobj_3->Base.NameInternal->data[i];
  }

  if (ArmReverseKinematics_strcmp(bname, switch_expression_0)) {
    bid = 0.0;
  } else {
    b = iobj_3->NumBodies;
    ret = 0;
    exitg2 = false;
    while ((!exitg2) && (ret <= (int32_T)b - 1)) {
      body = iobj_3->Bodies[ret];
      i = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = body->NameInternal->size[1];
      ArmRev_emxEnsureCapacity_char_T(bname, i);
      loop_ub = body->NameInternal->size[1];
      for (i = 0; i < loop_ub; i++) {
        bname->data[i] = body->NameInternal->data[i];
      }

      if (ArmReverseKinematics_strcmp(bname, switch_expression_0)) {
        bid = (real_T)ret + 1.0;
        exitg2 = true;
      } else {
        ret++;
      }
    }
  }

  if ((!(bid == 0.0)) && (bid < 0.0)) {
    i = iobj_3->Base.NameInternal->size[0] * iobj_3->Base.NameInternal->size[1];
    iobj_3->Base.NameInternal->size[0] = 1;
    iobj_3->Base.NameInternal->size[1] = switch_expression_0->size[1];
    ArmRev_emxEnsureCapacity_char_T(iobj_3->Base.NameInternal, i);
    loop_ub = switch_expression_0->size[1] - 1;
    for (i = 0; i <= loop_ub; i++) {
      iobj_3->Base.NameInternal->data[i] = switch_expression_0->data[i];
    }
  }

  ArmReverseKinema_emxFree_char_T(&switch_expression_0);
  obj_1 = varargin_2->Base.CollisionsInternal;
  newObj = Arm_CollisionSet_CollisionSet_e(&(&(&(&(&iobj_2[0])[0])[0])[0])[0],
    obj_1->MaxElements);
  newObj->Size = obj_1->Size;
  b = obj_1->Size;
  loop_ub = (int32_T)b - 1;
  for (ret = 0; ret <= loop_ub; ret++) {
    tmp = obj_1->CollisionGeometries->data[ret];
    copyGeometryInternal = collisioncodegen_copyGeometry(tmp.CollisionPrimitive);
    newObj->CollisionGeometries->data[ret].CollisionPrimitive =
      copyGeometryInternal;
    tmp_8 = &tmp.LocalPose[0];
    for (i = 0; i < 16; i++) {
      newObj->CollisionGeometries->data[ret].LocalPose[i] = tmp_8[i];
    }

    tmp_8 = &tmp.WorldPose[0];
    for (i = 0; i < 16; i++) {
      newObj->CollisionGeometries->data[ret].WorldPose[i] = tmp_8[i];
    }

    newObj->CollisionGeometries->data[ret].MeshScale[0] = tmp.MeshScale[0];
    newObj->CollisionGeometries->data[ret].MeshScale[1] = tmp.MeshScale[1];
    newObj->CollisionGeometries->data[ret].MeshScale[2] = tmp.MeshScale[2];
  }

  iobj_3->Base.CollisionsInternal = newObj;
  if (varargin_2->NumBodies >= 1.0) {
    body = varargin_2->Bodies[0];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2->Bodies[(int32_T)bid - 1];
    } else {
      parent = &varargin_2->Base;
    }

    i = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    ArmRev_emxEnsureCapacity_char_T(bname, i);
    loop_ub = parent->NameInternal->size[1];
    for (i = 0; i < loop_ub; i++) {
      bname->data[i] = parent->NameInternal->data[i];
    }

    ArmRevers_RigidBodyTree_addBody(iobj_3, body, bname, &(&(&(&(&iobj_2[0])[0])
      [0])[0])[1], &(&(&(&(&iobj_0[0])[0])[0])[0])[0], &(&(&(&(&iobj_1[0])[0])[0])
      [0])[0]);
  }

  if (varargin_2->NumBodies >= 2.0) {
    body = varargin_2->Bodies[1];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2->Bodies[(int32_T)bid - 1];
    } else {
      parent = &varargin_2->Base;
    }

    i = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    ArmRev_emxEnsureCapacity_char_T(bname, i);
    loop_ub = parent->NameInternal->size[1];
    for (i = 0; i < loop_ub; i++) {
      bname->data[i] = parent->NameInternal->data[i];
    }

    ArmRevers_RigidBodyTree_addBody(iobj_3, body, bname, &(&(&(&(&iobj_2[0])[0])
      [0])[0])[3], &(&(&(&(&iobj_0[0])[0])[0])[0])[2], &(&(&(&(&iobj_1[0])[0])[0])
      [0])[1]);
  }

  if (varargin_2->NumBodies >= 3.0) {
    body = varargin_2->Bodies[2];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2->Bodies[(int32_T)bid - 1];
    } else {
      parent = &varargin_2->Base;
    }

    i = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    ArmRev_emxEnsureCapacity_char_T(bname, i);
    loop_ub = parent->NameInternal->size[1];
    for (i = 0; i < loop_ub; i++) {
      bname->data[i] = parent->NameInternal->data[i];
    }

    ArmRevers_RigidBodyTree_addBody(iobj_3, body, bname, &(&(&(&(&iobj_2[0])[0])
      [0])[0])[5], &(&(&(&(&iobj_0[0])[0])[0])[0])[4], &(&(&(&(&iobj_1[0])[0])[0])
      [0])[2]);
  }

  if (varargin_2->NumBodies >= 4.0) {
    body = varargin_2->Bodies[3];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2->Bodies[(int32_T)bid - 1];
    } else {
      parent = &varargin_2->Base;
    }

    i = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    ArmRev_emxEnsureCapacity_char_T(bname, i);
    loop_ub = parent->NameInternal->size[1];
    for (i = 0; i < loop_ub; i++) {
      bname->data[i] = parent->NameInternal->data[i];
    }

    ArmRevers_RigidBodyTree_addBody(iobj_3, body, bname, &(&(&(&(&iobj_2[0])[0])
      [0])[0])[7], &(&(&(&(&iobj_0[0])[0])[0])[0])[6], &(&(&(&(&iobj_1[0])[0])[0])
      [0])[3]);
  }

  if (varargin_2->NumBodies >= 5.0) {
    body = varargin_2->Bodies[4];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2->Bodies[(int32_T)bid - 1];
    } else {
      parent = &varargin_2->Base;
    }

    i = bname->size[0] * bname->size[1];
    bname->size[0] = 1;
    bname->size[1] = parent->NameInternal->size[1];
    ArmRev_emxEnsureCapacity_char_T(bname, i);
    loop_ub = parent->NameInternal->size[1];
    for (i = 0; i < loop_ub; i++) {
      bname->data[i] = parent->NameInternal->data[i];
    }

    ArmRevers_RigidBodyTree_addBody(iobj_3, body, bname, &(&(&(&(&iobj_2[0])[0])
      [0])[0])[9], &(&(&(&(&iobj_0[0])[0])[0])[0])[8], &(&(&(&(&iobj_1[0])[0])[0])
      [0])[4]);
  }

  ArmReverseKinema_emxFree_char_T(&bname);
  obj->RigidBodyTreeInternal = iobj_3;
  obj->RigidBodyTreeKinematicModel = 0.0;
  iobj_4->MaxNumIteration = 1500.0;
  iobj_4->MaxTime = 10.0;
  iobj_4->SolutionTolerance = 1.0E-6;
  iobj_4->ConstraintsOn = true;
  iobj_4->RandomRestart = true;
  iobj_4->StepTolerance = 1.0E-12;
  iobj_4->GradientTolerance = 5.0E-9;
  iobj_4->ErrorChangeTolerance = 1.0E-12;
  iobj_4->DampingBias = 0.0025;
  iobj_4->UseErrorDamping = true;
  for (i = 0; i < 18; i++) {
    iobj_4->Name[i] = tmp_7[i];
  }

  iobj_4->TimeObj.StartTime = tmp_6;
  iobj_4->TimeObjInternal.StartTime = tmp_6;
  iobj_4->matlabCodegenIsDeleted = false;
  obj->Solver = iobj_4;
  obj_0 = obj->Solver;
  bid = obj_0->ErrorChangeTolerance;
  b = obj_0->DampingBias;
  result = obj_0->UseErrorDamping;
  for (i = 0; i < 18; i++) {
    switch_expression[i] = obj->Solver->Name[i];
  }

  for (i = 0; i < 18; i++) {
    a[i] = tmp_7[i];
  }

  ret = memcmp(&a[0], &switch_expression[0], 18);
  if (ret == 0) {
    bid = 0.001;
    b = 0.0025;
    result = true;
  }

  obj_0 = obj->Solver;
  obj_0->MaxNumIteration = 1500.0;
  obj_0->MaxTime = 10.0;
  obj_0->GradientTolerance = 0.001;
  obj_0->SolutionTolerance = 0.01;
  obj_0->ConstraintsOn = true;
  obj_0->RandomRestart = false;
  obj_0->StepTolerance = 1.0E-6;
  obj_0->ErrorChangeTolerance = bid;
  obj_0->DampingBias = b;
  obj_0->UseErrorDamping = result;
}

static void ArmReverseKi_SystemCore_setup_e(robotics_slmanip_internal_b_e_T *obj)
{
  emxArray_char_T_ArmReverseKin_T *jname;
  real_T poslim_data[12];
  int32_T b_kstr;
  int32_T loop_ub;
  char_T a_0[9];
  char_T a[8];
  int8_T msubspace_data[36];
  int8_T tmp[6];
  boolean_T result;
  static const char_T tmp_0[5] = { 'B', 'o', 'd', 'y', '1' };

  static const real_T tmp_1[9] = { 0.051705379090332015, 0.0, 0.0, 0.0,
    0.051705379090332015, 0.0, 0.0, 0.0, 0.098174770424681035 };

  static const real_T tmp_2[36] = { 0.051705379090332015, 0.0, 0.0, 0.0,
    -0.039269908169872414, -0.0, 0.0, 0.051705379090332015, 0.0,
    0.039269908169872414, 0.0, 0.0, 0.0, 0.0, 0.098174770424681035, 0.0, -0.0,
    0.0, 0.0, 0.039269908169872414, 0.0, 0.78539816339744828, 0.0, 0.0,
    -0.039269908169872414, 0.0, -0.0, 0.0, 0.78539816339744828, 0.0, -0.0, 0.0,
    0.0, 0.0, 0.0, 0.78539816339744828 };

  static const int8_T tmp_3[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_4[6] = { 'J', 'o', 'i', 'n', 't', '1' };

  static const char_T tmp_5[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_6[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_7[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_8[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_9[36] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const char_T tmp_a[5] = { 'B', 'o', 'd', 'y', '2' };

  static const real_T tmp_b[9] = { 0.00083333333333333371, 0.0, 0.0, 0.0,
    0.067333333333333356, 0.0, 0.0, 0.0, 0.066833333333333356 };

  static const real_T tmp_c[36] = { 0.00083333333333333371, 0.0, 0.0, 0.0, 0.0,
    -0.0, 0.0, 0.067333333333333356, 0.0, -0.0, 0.0, -0.10000000000000002, 0.0,
    0.0, 0.066833333333333356, 0.0, 0.10000000000000002, 0.0, 0.0, -0.0, 0.0,
    0.20000000000000004, 0.0, 0.0, 0.0, 0.0, 0.10000000000000002, 0.0,
    0.20000000000000004, 0.0, -0.0, -0.10000000000000002, 0.0, 0.0, 0.0,
    0.20000000000000004 };

  static const char_T tmp_d[6] = { 'J', 'o', 'i', 'n', 't', '2' };

  static const real_T tmp_e[16] = { 6.123233995736766E-17, 0.0, 1.0, 0.0,
    -1.2246467991473532E-16, -1.0, 7.498798913309288E-33, 0.0, 1.0,
    -1.2246467991473532E-16, -6.123233995736766E-17, 0.0, 0.0, 0.0, 0.1, 1.0 };

  real_T unusedExpr[5];
  int32_T exitg1;
  obj->isInitialized = 1;
  ArmReverseKinematics_rand_e(unusedExpr);
  obj->TreeInternal.NumBodies = 5.0;
  obj->TreeInternal.Bodies[0] = ArmRever_RigidBody_RigidBody_ev
    (&(&obj->TreeInternal._pobj2[0])[0], &(&obj->TreeInternal._pobj0[0])[0],
     &(&obj->TreeInternal._pobj1[0])[0]);
  obj->TreeInternal.Bodies[1] = ArmReve_RigidBody_RigidBody_evf
    (&(&obj->TreeInternal._pobj2[0])[1], &(&obj->TreeInternal._pobj0[0])[1],
     &(&obj->TreeInternal._pobj1[0])[1]);
  obj->TreeInternal.Bodies[2] = ArmRev_RigidBody_RigidBody_evfm
    (&(&obj->TreeInternal._pobj2[0])[2], &(&obj->TreeInternal._pobj0[0])[2],
     &(&obj->TreeInternal._pobj1[0])[2]);
  obj->TreeInternal.Bodies[3] = ArmRe_RigidBody_RigidBody_evfmf
    (&(&obj->TreeInternal._pobj2[0])[3], &(&obj->TreeInternal._pobj0[0])[3],
     &(&obj->TreeInternal._pobj1[0])[3]);
  obj->TreeInternal.Bodies[4] = ArmR_RigidBody_RigidBody_evfmfk
    (&(&obj->TreeInternal._pobj2[0])[4], &(&obj->TreeInternal._pobj0[0])[4],
     &(&obj->TreeInternal._pobj1[0])[4]);
  b_kstr = obj->TreeInternal._pobj2[5].NameInternal->size[0] *
    obj->TreeInternal._pobj2[5].NameInternal->size[1];
  obj->TreeInternal._pobj2[5].NameInternal->size[0] = 1;
  obj->TreeInternal._pobj2[5].NameInternal->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(obj->TreeInternal._pobj2[5].NameInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->TreeInternal._pobj2[5].NameInternal->data[b_kstr] = tmp_0[b_kstr];
  }

  obj->TreeInternal._pobj2[5].ParentIndex = 0.0;
  obj->TreeInternal._pobj2[5].MassInternal = 0.78539816339744828;
  obj->TreeInternal._pobj2[5].CenterOfMassInternal[0] = -0.0;
  obj->TreeInternal._pobj2[5].CenterOfMassInternal[1] = -0.0;
  obj->TreeInternal._pobj2[5].CenterOfMassInternal[2] = 0.05;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->TreeInternal._pobj2[5].InertiaInternal[b_kstr] = tmp_1[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->TreeInternal._pobj2[5].SpatialInertia[b_kstr] = tmp_2[b_kstr];
  }

  obj->TreeInternal._pobj1[5].InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->TreeInternal._pobj1[5].JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->TreeInternal._pobj1[5].ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = obj->TreeInternal._pobj1[5].NameInternal->size[0] *
    obj->TreeInternal._pobj1[5].NameInternal->size[1];
  obj->TreeInternal._pobj1[5].NameInternal->size[0] = 1;
  obj->TreeInternal._pobj1[5].NameInternal->size[1] = 6;
  ArmRev_emxEnsureCapacity_char_T(obj->TreeInternal._pobj1[5].NameInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->TreeInternal._pobj1[5].NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = obj->TreeInternal._pobj1[5].Type->size[0] * obj->TreeInternal._pobj1
    [5].Type->size[1];
  obj->TreeInternal._pobj1[5].Type->size[0] = 1;
  obj->TreeInternal._pobj1[5].Type->size[1] = 8;
  ArmRev_emxEnsureCapacity_char_T(obj->TreeInternal._pobj1[5].Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->TreeInternal._pobj1[5].Type->data[b_kstr] = tmp_5[b_kstr];
  }

  ArmReverseKinema_emxInit_char_T(&jname, 2);
  b_kstr = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = obj->TreeInternal._pobj1[5].Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(jname, b_kstr);
  loop_ub = obj->TreeInternal._pobj1[5].Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    jname->data[b_kstr] = obj->TreeInternal._pobj1[5].Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    a[b_kstr] = tmp_5[b_kstr];
  }

  result = false;
  if (jname->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != jname->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_6[b_kstr];
    }

    if (jname->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != jname->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    obj->TreeInternal._pobj1[5].VelocityNumber = 1.0;
    obj->TreeInternal._pobj1[5].PositionNumber = 1.0;
    obj->TreeInternal._pobj1[5].JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj1[5].JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj1[5].JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    obj->TreeInternal._pobj1[5].VelocityNumber = 1.0;
    obj->TreeInternal._pobj1[5].PositionNumber = 1.0;
    obj->TreeInternal._pobj1[5].JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj1[5].JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj1[5].JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    obj->TreeInternal._pobj1[5].VelocityNumber = 0.0;
    obj->TreeInternal._pobj1[5].PositionNumber = 0.0;
    obj->TreeInternal._pobj1[5].JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj1[5].JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj1[5].JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->TreeInternal._pobj1[5].MotionSubspace->size[0] *
    obj->TreeInternal._pobj1[5].MotionSubspace->size[1];
  obj->TreeInternal._pobj1[5].MotionSubspace->size[0] = 6;
  obj->TreeInternal._pobj1[5].MotionSubspace->size[1] = 1;
  ArmRev_emxEnsureCapacity_real_T(obj->TreeInternal._pobj1[5].MotionSubspace,
    b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->TreeInternal._pobj1[5].MotionSubspace->data[b_kstr] =
      msubspace_data[b_kstr];
  }

  b_kstr = obj->TreeInternal._pobj1[5].PositionLimitsInternal->size[0] *
    obj->TreeInternal._pobj1[5].PositionLimitsInternal->size[1];
  obj->TreeInternal._pobj1[5].PositionLimitsInternal->size[0] = 1;
  obj->TreeInternal._pobj1[5].PositionLimitsInternal->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(obj->TreeInternal._pobj1[5].
    PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->TreeInternal._pobj1[5].PositionLimitsInternal->data[b_kstr] =
      poslim_data[b_kstr];
  }

  b_kstr = obj->TreeInternal._pobj1[5].HomePositionInternal->size[0];
  obj->TreeInternal._pobj1[5].HomePositionInternal->size[0] = 1;
  ArmRev_emxEnsureCapacity_real_T(obj->TreeInternal._pobj1[5].
    HomePositionInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    obj->TreeInternal._pobj1[5].HomePositionInternal->data[0] = 0.0;
  }

  obj->TreeInternal._pobj2[5].JointInternal = &obj->TreeInternal._pobj1[5];
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->TreeInternal._pobj2[5].JointInternal->JointToParentTransform[b_kstr] =
      tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->TreeInternal._pobj2[5].JointInternal->ChildToJointTransform[b_kstr] =
      tmp_8[b_kstr];
  }

  b_kstr = obj->TreeInternal._pobj2[5].JointInternal->MotionSubspace->size[0] *
    obj->TreeInternal._pobj2[5].JointInternal->MotionSubspace->size[1];
  obj->TreeInternal._pobj2[5].JointInternal->MotionSubspace->size[0] = 6;
  obj->TreeInternal._pobj2[5].JointInternal->MotionSubspace->size[1] = 1;
  ArmRev_emxEnsureCapacity_real_T(obj->TreeInternal._pobj2[5]
    .JointInternal->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->TreeInternal._pobj2[5].JointInternal->MotionSubspace->data[b_kstr] =
      tmp_9[b_kstr];
  }

  obj->TreeInternal._pobj2[5].JointInternal->InTree = true;
  b_kstr = obj->TreeInternal._pobj2[5].JointInternal->
    PositionLimitsInternal->size[0] * obj->TreeInternal._pobj2[5].
    JointInternal->PositionLimitsInternal->size[1];
  obj->TreeInternal._pobj2[5].JointInternal->PositionLimitsInternal->size[0] = 1;
  obj->TreeInternal._pobj2[5].JointInternal->PositionLimitsInternal->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(obj->TreeInternal._pobj2[5]
    .JointInternal->PositionLimitsInternal, b_kstr);
  obj->TreeInternal._pobj2[5].JointInternal->PositionLimitsInternal->data[0] =
    -3.1415926535897931;
  obj->TreeInternal._pobj2[5].JointInternal->PositionLimitsInternal->data
    [obj->TreeInternal._pobj2[5].JointInternal->PositionLimitsInternal->size[0]]
    = 3.1415926535897931;
  obj->TreeInternal._pobj2[5].JointInternal->JointAxisInternal[0] = 0.0;
  obj->TreeInternal._pobj2[5].JointInternal->JointAxisInternal[1] = 0.0;
  obj->TreeInternal._pobj2[5].JointInternal->JointAxisInternal[2] = 1.0;
  b_kstr = obj->TreeInternal._pobj2[5].JointInternal->HomePositionInternal->
    size[0];
  obj->TreeInternal._pobj2[5].JointInternal->HomePositionInternal->size[0] = 1;
  ArmRev_emxEnsureCapacity_real_T(obj->TreeInternal._pobj2[5]
    .JointInternal->HomePositionInternal, b_kstr);
  obj->TreeInternal._pobj2[5].JointInternal->HomePositionInternal->data[0] = 0.0;
  obj->TreeInternal._pobj2[5].CollisionsInternal =
    Arm_CollisionSet_CollisionSet_e(&obj->TreeInternal._pobj0[5], 0.0);
  obj->TreeInternal._pobj2[5].matlabCodegenIsDeleted = false;
  obj->TreeInternal.Bodies[0] = &obj->TreeInternal._pobj2[5];
  obj->TreeInternal.Bodies[0]->Index = 1.0;
  b_kstr = obj->TreeInternal._pobj2[6].NameInternal->size[0] *
    obj->TreeInternal._pobj2[6].NameInternal->size[1];
  obj->TreeInternal._pobj2[6].NameInternal->size[0] = 1;
  obj->TreeInternal._pobj2[6].NameInternal->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(obj->TreeInternal._pobj2[6].NameInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->TreeInternal._pobj2[6].NameInternal->data[b_kstr] = tmp_a[b_kstr];
  }

  obj->TreeInternal._pobj2[6].ParentIndex = 1.0;
  obj->TreeInternal._pobj2[6].MassInternal = 0.20000000000000004;
  obj->TreeInternal._pobj2[6].CenterOfMassInternal[0] = 0.5;
  obj->TreeInternal._pobj2[6].CenterOfMassInternal[1] = -0.0;
  obj->TreeInternal._pobj2[6].CenterOfMassInternal[2] = -0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->TreeInternal._pobj2[6].InertiaInternal[b_kstr] = tmp_b[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->TreeInternal._pobj2[6].SpatialInertia[b_kstr] = tmp_c[b_kstr];
  }

  obj->TreeInternal._pobj1[6].InTree = false;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->TreeInternal._pobj1[6].JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->TreeInternal._pobj1[6].ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = obj->TreeInternal._pobj1[6].NameInternal->size[0] *
    obj->TreeInternal._pobj1[6].NameInternal->size[1];
  obj->TreeInternal._pobj1[6].NameInternal->size[0] = 1;
  obj->TreeInternal._pobj1[6].NameInternal->size[1] = 6;
  ArmRev_emxEnsureCapacity_char_T(obj->TreeInternal._pobj1[6].NameInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->TreeInternal._pobj1[6].NameInternal->data[b_kstr] = tmp_d[b_kstr];
  }

  b_kstr = obj->TreeInternal._pobj1[6].Type->size[0] * obj->TreeInternal._pobj1
    [6].Type->size[1];
  obj->TreeInternal._pobj1[6].Type->size[0] = 1;
  obj->TreeInternal._pobj1[6].Type->size[1] = 8;
  ArmRev_emxEnsureCapacity_char_T(obj->TreeInternal._pobj1[6].Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->TreeInternal._pobj1[6].Type->data[b_kstr] = tmp_5[b_kstr];
  }

  b_kstr = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = obj->TreeInternal._pobj1[6].Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(jname, b_kstr);
  loop_ub = obj->TreeInternal._pobj1[6].Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    jname->data[b_kstr] = obj->TreeInternal._pobj1[6].Type->data[b_kstr];
  }

  result = false;
  if (jname->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != jname->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_6[b_kstr];
    }

    if (jname->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != jname->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  switch (b_kstr) {
   case 0:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 1;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 0;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    obj->TreeInternal._pobj1[6].VelocityNumber = 1.0;
    obj->TreeInternal._pobj1[6].PositionNumber = 1.0;
    obj->TreeInternal._pobj1[6].JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj1[6].JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj1[6].JointAxisInternal[2] = 1.0;
    break;

   case 1:
    tmp[0] = 0;
    tmp[1] = 0;
    tmp[2] = 0;
    tmp[3] = 0;
    tmp[4] = 0;
    tmp[5] = 1;
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = tmp[b_kstr];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    obj->TreeInternal._pobj1[6].VelocityNumber = 1.0;
    obj->TreeInternal._pobj1[6].PositionNumber = 1.0;
    obj->TreeInternal._pobj1[6].JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj1[6].JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj1[6].JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_kstr = 0; b_kstr < 6; b_kstr++) {
      msubspace_data[b_kstr] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    obj->TreeInternal._pobj1[6].VelocityNumber = 0.0;
    obj->TreeInternal._pobj1[6].PositionNumber = 0.0;
    obj->TreeInternal._pobj1[6].JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj1[6].JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj1[6].JointAxisInternal[2] = 0.0;
    break;
  }

  b_kstr = obj->TreeInternal._pobj1[6].MotionSubspace->size[0] *
    obj->TreeInternal._pobj1[6].MotionSubspace->size[1];
  obj->TreeInternal._pobj1[6].MotionSubspace->size[0] = 6;
  obj->TreeInternal._pobj1[6].MotionSubspace->size[1] = 1;
  ArmRev_emxEnsureCapacity_real_T(obj->TreeInternal._pobj1[6].MotionSubspace,
    b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->TreeInternal._pobj1[6].MotionSubspace->data[b_kstr] =
      msubspace_data[b_kstr];
  }

  b_kstr = obj->TreeInternal._pobj1[6].PositionLimitsInternal->size[0] *
    obj->TreeInternal._pobj1[6].PositionLimitsInternal->size[1];
  obj->TreeInternal._pobj1[6].PositionLimitsInternal->size[0] = 1;
  obj->TreeInternal._pobj1[6].PositionLimitsInternal->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(obj->TreeInternal._pobj1[6].
    PositionLimitsInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 2; b_kstr++) {
    obj->TreeInternal._pobj1[6].PositionLimitsInternal->data[b_kstr] =
      poslim_data[b_kstr];
  }

  b_kstr = obj->TreeInternal._pobj1[6].HomePositionInternal->size[0];
  obj->TreeInternal._pobj1[6].HomePositionInternal->size[0] = 1;
  ArmRev_emxEnsureCapacity_real_T(obj->TreeInternal._pobj1[6].
    HomePositionInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 1; b_kstr++) {
    obj->TreeInternal._pobj1[6].HomePositionInternal->data[0] = 0.0;
  }

  obj->TreeInternal._pobj2[6].JointInternal = &obj->TreeInternal._pobj1[6];
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->TreeInternal._pobj2[6].JointInternal->JointToParentTransform[b_kstr] =
      tmp_e[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->TreeInternal._pobj2[6].JointInternal->ChildToJointTransform[b_kstr] =
      tmp_8[b_kstr];
  }

  b_kstr = obj->TreeInternal._pobj2[6].JointInternal->MotionSubspace->size[0] *
    obj->TreeInternal._pobj2[6].JointInternal->MotionSubspace->size[1];
  obj->TreeInternal._pobj2[6].JointInternal->MotionSubspace->size[0] = 6;
  obj->TreeInternal._pobj2[6].JointInternal->MotionSubspace->size[1] = 1;
  ArmRev_emxEnsureCapacity_real_T(obj->TreeInternal._pobj2[6]
    .JointInternal->MotionSubspace, b_kstr);
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    obj->TreeInternal._pobj2[6].JointInternal->MotionSubspace->data[b_kstr] =
      tmp_9[b_kstr];
  }

  obj->TreeInternal._pobj2[6].JointInternal->InTree = true;
  b_kstr = obj->TreeInternal._pobj2[6].JointInternal->
    PositionLimitsInternal->size[0] * obj->TreeInternal._pobj2[6].
    JointInternal->PositionLimitsInternal->size[1];
  obj->TreeInternal._pobj2[6].JointInternal->PositionLimitsInternal->size[0] = 1;
  obj->TreeInternal._pobj2[6].JointInternal->PositionLimitsInternal->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(obj->TreeInternal._pobj2[6]
    .JointInternal->PositionLimitsInternal, b_kstr);
  obj->TreeInternal._pobj2[6].JointInternal->PositionLimitsInternal->data[0] =
    -1.5707963267948966;
  obj->TreeInternal._pobj2[6].JointInternal->PositionLimitsInternal->data
    [obj->TreeInternal._pobj2[6].JointInternal->PositionLimitsInternal->size[0]]
    = 1.5707963267948966;
  obj->TreeInternal._pobj2[6].JointInternal->JointAxisInternal[0] = 0.0;
  obj->TreeInternal._pobj2[6].JointInternal->JointAxisInternal[1] = 0.0;
  obj->TreeInternal._pobj2[6].JointInternal->JointAxisInternal[2] = 1.0;
  b_kstr = obj->TreeInternal._pobj2[6].JointInternal->HomePositionInternal->
    size[0];
  obj->TreeInternal._pobj2[6].JointInternal->HomePositionInternal->size[0] = 1;
  ArmRev_emxEnsureCapacity_real_T(obj->TreeInternal._pobj2[6]
    .JointInternal->HomePositionInternal, b_kstr);
  obj->TreeInternal._pobj2[6].JointInternal->HomePositionInternal->data[0] = 0.0;
  obj->TreeInternal._pobj2[6].CollisionsInternal =
    Arm_CollisionSet_CollisionSet_e(&obj->TreeInternal._pobj0[6], 0.0);
  obj->TreeInternal._pobj2[6].matlabCodegenIsDeleted = false;
  obj->TreeInternal.Bodies[1] = &obj->TreeInternal._pobj2[6];
  obj->TreeInternal.Bodies[1]->Index = 2.0;
  obj->TreeInternal.Bodies[2] = Arm_RigidBody_RigidBody_evfmfk1
    (&obj->TreeInternal._pobj2[7], &obj->TreeInternal._pobj0[7],
     &obj->TreeInternal._pobj1[7]);
  obj->TreeInternal.Bodies[2]->Index = 3.0;
  obj->TreeInternal.Bodies[3] = Ar_RigidBody_RigidBody_evfmfk1h
    (&obj->TreeInternal._pobj2[8], &obj->TreeInternal._pobj0[8],
     &obj->TreeInternal._pobj1[8]);
  obj->TreeInternal.Bodies[3]->Index = 4.0;
  obj->TreeInternal.Bodies[4] = A_RigidBody_RigidBody_evfmfk1h2
    (&obj->TreeInternal._pobj2[9], &obj->TreeInternal._pobj0[9],
     &obj->TreeInternal._pobj1[9]);
  obj->TreeInternal.Bodies[4]->Index = 5.0;
  obj->TreeInternal.Gravity[0] = 0.0;
  obj->TreeInternal.Gravity[1] = 0.0;
  obj->TreeInternal.Gravity[2] = -9.80665;
  b_kstr = obj->TreeInternal.Base.NameInternal->size[0] *
    obj->TreeInternal.Base.NameInternal->size[1];
  obj->TreeInternal.Base.NameInternal->size[0] = 1;
  obj->TreeInternal.Base.NameInternal->size[1] = 4;
  ArmRev_emxEnsureCapacity_char_T(obj->TreeInternal.Base.NameInternal, b_kstr);
  obj->TreeInternal.Base.NameInternal->data[0] = 'B';
  obj->TreeInternal.Base.NameInternal->data[1] = 'a';
  obj->TreeInternal.Base.NameInternal->data[2] = 's';
  obj->TreeInternal.Base.NameInternal->data[3] = 'e';
  obj->TreeInternal.Base.ParentIndex = -1.0;
  obj->TreeInternal.Base.MassInternal = 0.0;
  obj->TreeInternal.Base.CenterOfMassInternal[0] = 0.0;
  obj->TreeInternal.Base.CenterOfMassInternal[1] = 0.0;
  obj->TreeInternal.Base.CenterOfMassInternal[2] = 0.0;
  for (b_kstr = 0; b_kstr < 9; b_kstr++) {
    obj->TreeInternal.Base.InertiaInternal[b_kstr] = 0.0;
  }

  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    obj->TreeInternal.Base.SpatialInertia[b_kstr] = 0.0;
  }

  b_kstr = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = obj->TreeInternal.Base.NameInternal->size[1] + 4;
  ArmRev_emxEnsureCapacity_char_T(jname, b_kstr);
  loop_ub = obj->TreeInternal.Base.NameInternal->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    jname->data[b_kstr] = obj->TreeInternal.Base.NameInternal->data[b_kstr];
  }

  jname->data[obj->TreeInternal.Base.NameInternal->size[1]] = '_';
  jname->data[obj->TreeInternal.Base.NameInternal->size[1] + 1] = 'j';
  jname->data[obj->TreeInternal.Base.NameInternal->size[1] + 2] = 'n';
  jname->data[obj->TreeInternal.Base.NameInternal->size[1] + 3] = 't';
  obj->TreeInternal.Base.JointInternal = A_rigidBodyJoint_rigidBodyJoint
    (&obj->TreeInternal._pobj1[10], jname);
  ArmReverseKinema_emxFree_char_T(&jname);
  obj->TreeInternal.Base.CollisionsInternal = Arm_CollisionSet_CollisionSet_e
    (&obj->TreeInternal._pobj0[10], 0.0);
  obj->TreeInternal.Base.matlabCodegenIsDeleted = false;
  obj->TreeInternal.Base.Index = 0.0;
  obj->TreeInternal.matlabCodegenIsDeleted = false;
  obj->IKInternal.isInitialized = 0;
  ArmRev_SystemProp_setProperties(&obj->IKInternal, &obj->TreeInternal,
    &obj->IKInternal._pobj1[0], &obj->IKInternal._pobj2[0],
    &obj->IKInternal._pobj3[0], &obj->IKInternal._pobj4, &obj->IKInternal._pobj5);
  obj->IKInternal.RigidBodyTreeKinematicModel = 0.0;
  obj->IKInternal.matlabCodegenIsDeleted = false;
}

static void emxInitStruct_c_rigidBodyJoint1(c_rigidBodyJoint_ArmReverseKi_T
  *pStruct)
{
  ArmReverseKinema_emxInit_char_T(&pStruct->Type, 2);
}

static void emxInit_h_robotics_manip_intern(emxArray_h_robotics_manip_int_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_h_robotics_manip_int_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_h_robotics_manip_int_T *)malloc(sizeof
    (emxArray_h_robotics_manip_int_T));
  emxArray = *pEmxArray;
  emxArray->data = (h_robotics_manip_internal_Col_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * (uint32_T)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void emxInitStruct_i_robotics_manip_(i_robotics_manip_internal_Col_T
  *pStruct)
{
  emxInit_h_robotics_manip_intern(&pStruct->CollisionGeometries, 2);
}

static void emxInitStruct_k_robotics_manip_(k_robotics_manip_internal_Rig_T
  *pStruct)
{
  ArmReverseKinema_emxInit_char_T(&pStruct->NameInternal, 2);
  emxInitStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
  emxInitStruct_i_robotics_manip_(&pStruct->CollisionsInternal);
}

static void emxInitMatrix_k_robotics_manip_(k_robotics_manip_internal_Rig_T
  pMatrix[10])
{
  int32_T i;
  for (i = 0; i < 10; i++) {
    emxInitStruct_k_robotics_manip_(&pMatrix[i]);
  }
}

static void emxInitStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxInitStruct_k_robotics_manip_(&pStruct->Base);
  emxInitMatrix_k_robotics_manip_(pStruct->_pobj0);
}

static void emxInitStruct_robotics_slmani_e(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxInitStruct_l_robotics_manip_(&pStruct->TreeInternal);
}

static void ArmReverseKinematics_rand(real_T r[5])
{
  int32_T b_k;
  int32_T b_kk;
  int32_T k;
  uint32_T b_u[2];
  for (b_k = 0; b_k < 5; b_k++) {
    real_T b_r;

    /* ========================= COPYRIGHT NOTICE ============================ */
    /*  This is a uniform (0,1) pseudorandom number generator based on:        */
    /*                                                                         */
    /*  A C-program for MT19937, with initialization improved 2002/1/26.       */
    /*  Coded by Takuji Nishimura and Makoto Matsumoto.                        */
    /*                                                                         */
    /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      */
    /*  All rights reserved.                                                   */
    /*                                                                         */
    /*  Redistribution and use in source and binary forms, with or without     */
    /*  modification, are permitted provided that the following conditions     */
    /*  are met:                                                               */
    /*                                                                         */
    /*    1. Redistributions of source code must retain the above copyright    */
    /*       notice, this list of conditions and the following disclaimer.     */
    /*                                                                         */
    /*    2. Redistributions in binary form must reproduce the above copyright */
    /*       notice, this list of conditions and the following disclaimer      */
    /*       in the documentation and/or other materials provided with the     */
    /*       distribution.                                                     */
    /*                                                                         */
    /*    3. The names of its contributors may not be used to endorse or       */
    /*       promote products derived from this software without specific      */
    /*       prior written permission.                                         */
    /*                                                                         */
    /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
    /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
    /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
    /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  */
    /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
    /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
    /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
    /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
    /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
    /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
    /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */
    /*                                                                         */
    /* =============================   END   ================================= */
    int32_T exitg1;
    do {
      exitg1 = 0;
      for (k = 0; k < 2; k++) {
        uint32_T mti;
        uint32_T y;
        mti = rtDW.state_j[624] + 1U;
        if (rtDW.state_j[624] + 1U >= 625U) {
          for (b_kk = 0; b_kk < 227; b_kk++) {
            y = (rtDW.state_j[b_kk + 1] & 2147483647U) | (rtDW.state_j[b_kk] &
              2147483648U);
            if ((y & 1U) == 0U) {
              mti = y >> 1U;
            } else {
              mti = y >> 1U ^ 2567483615U;
            }

            rtDW.state_j[b_kk] = rtDW.state_j[b_kk + 397] ^ mti;
          }

          for (b_kk = 0; b_kk < 396; b_kk++) {
            y = (rtDW.state_j[b_kk + 227] & 2147483648U) | (rtDW.state_j[b_kk +
              228] & 2147483647U);
            if ((y & 1U) == 0U) {
              mti = y >> 1U;
            } else {
              mti = y >> 1U ^ 2567483615U;
            }

            rtDW.state_j[b_kk + 227] = rtDW.state_j[b_kk] ^ mti;
          }

          y = (rtDW.state_j[623] & 2147483648U) | (rtDW.state_j[0] & 2147483647U);
          if ((y & 1U) == 0U) {
            mti = y >> 1U;
          } else {
            mti = y >> 1U ^ 2567483615U;
          }

          rtDW.state_j[623] = rtDW.state_j[396] ^ mti;
          mti = 1U;
        }

        y = rtDW.state_j[(int32_T)mti - 1];
        rtDW.state_j[624] = mti;
        y ^= y >> 11U;
        y ^= y << 7U & 2636928640U;
        y ^= y << 15U & 4022730752U;
        b_u[k] = y >> 18U ^ y;
      }

      b_r = ((real_T)(b_u[0] >> 5U) * 6.7108864E+7 + (real_T)(b_u[1] >> 6U)) *
        1.1102230246251565E-16;
      if (b_r == 0.0) {
        boolean_T b_isvalid;
        if ((rtDW.state_j[624] >= 1U) && (rtDW.state_j[624] < 625U)) {
          boolean_T exitg2;
          b_isvalid = false;
          k = 0;
          exitg2 = false;
          while ((!exitg2) && (k + 1 < 625)) {
            if (rtDW.state_j[k] == 0U) {
              k++;
            } else {
              b_isvalid = true;
              exitg2 = true;
            }
          }
        } else {
          b_isvalid = false;
        }

        if (!b_isvalid) {
          rtDW.state_j[0] = 5489U;
          rtDW.state_j[624] = 624U;
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    r[b_k] = b_r;
  }
}

static void emxEnsureCapacity_h_robotics_ma(emxArray_h_robotics_manip_int_T
  *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = malloc((uint32_T)i * sizeof(h_robotics_manip_internal_Col_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(h_robotics_manip_internal_Col_T)
             * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (h_robotics_manip_internal_Col_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void emxFree_h_robotics_manip_intern(emxArray_h_robotics_manip_int_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_h_robotics_manip_int_T *)NULL) {
    if (((*pEmxArray)->data != (h_robotics_manip_internal_Col_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_h_robotics_manip_int_T *)NULL;
  }
}

static i_robotics_manip_internal_Col_T *ArmRe_CollisionSet_CollisionSet
  (i_robotics_manip_internal_Col_T *obj)
{
  static const void *GeometryInternal = NULL;
  emxArray_h_robotics_manip_int_T *e;
  i_robotics_manip_internal_Col_T *b_obj;
  real_T c;
  int32_T b_i;
  int32_T d;
  obj->Size = 0.0;
  b_obj = obj;
  obj->MaxElements = 0.0;
  emxInit_h_robotics_manip_intern(&e, 2);
  b_i = e->size[0] * e->size[1];
  e->size[1] = (int32_T)obj->MaxElements;
  emxEnsureCapacity_h_robotics_ma(e, b_i);
  b_i = obj->CollisionGeometries->size[0] * obj->CollisionGeometries->size[1];
  obj->CollisionGeometries->size[0] = 1;
  obj->CollisionGeometries->size[1] = e->size[1];
  emxFree_h_robotics_manip_intern(&e);
  emxEnsureCapacity_h_robotics_ma(obj->CollisionGeometries, b_i);
  c = obj->MaxElements;
  d = (int32_T)c - 1;
  for (b_i = 0; b_i <= d; b_i++) {
    obj->CollisionGeometries->data[b_i].CollisionPrimitive = (void *)
      GeometryInternal;
  }

  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static void RigidBodyTree_defaultInitialize(l_robotics_manip_internal_Rig_T *obj,
  k_robotics_manip_internal_Rig_T *iobj_0)
{
  emxArray_char_T_ArmReverseKin_T *switch_expression;
  int32_T b_kstr;
  int32_T loop_ub;
  char_T a_0[9];
  char_T a[8];
  boolean_T result;
  static const char_T tmp[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '1' };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const char_T tmp_3[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '2' };

  static const char_T tmp_4[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '3' };

  static const char_T tmp_5[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '4' };

  static const char_T tmp_6[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '5' };

  int32_T exitg1;
  b_kstr = iobj_0[0].NameInternal->size[0] * iobj_0[0].NameInternal->size[1];
  iobj_0[0].NameInternal->size[0] = 1;
  iobj_0[0].NameInternal->size[1] = 10;
  ArmRev_emxEnsureCapacity_char_T(iobj_0[0].NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    iobj_0[0].NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  b_kstr = iobj_0[0].JointInternal.Type->size[0] * iobj_0[0].
    JointInternal.Type->size[1];
  iobj_0[0].JointInternal.Type->size[0] = 1;
  iobj_0[0].JointInternal.Type->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(iobj_0[0].JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_0[0].JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  ArmReverseKinema_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_0[0].JointInternal.Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_0[0].JointInternal.Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_0[0].JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    a[b_kstr] = tmp_1[b_kstr];
  }

  result = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_2[b_kstr];
    }

    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  switch (b_kstr) {
   case 0:
    iobj_0[0].JointInternal.PositionNumber = 1.0;
    iobj_0[0].JointInternal.JointAxisInternal[0] = 0.0;
    iobj_0[0].JointInternal.JointAxisInternal[1] = 0.0;
    iobj_0[0].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    iobj_0[0].JointInternal.PositionNumber = 1.0;
    iobj_0[0].JointInternal.JointAxisInternal[0] = 0.0;
    iobj_0[0].JointInternal.JointAxisInternal[1] = 0.0;
    iobj_0[0].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    iobj_0[0].JointInternal.PositionNumber = 0.0;
    iobj_0[0].JointInternal.JointAxisInternal[0] = 0.0;
    iobj_0[0].JointInternal.JointAxisInternal[1] = 0.0;
    iobj_0[0].JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  iobj_0[0].ParentIndex = -1.0;
  ArmRe_CollisionSet_CollisionSet(&iobj_0[0].CollisionsInternal);
  iobj_0[0].matlabCodegenIsDeleted = false;
  obj->Bodies[0] = &iobj_0[0];
  b_kstr = iobj_0[1].NameInternal->size[0] * iobj_0[1].NameInternal->size[1];
  iobj_0[1].NameInternal->size[0] = 1;
  iobj_0[1].NameInternal->size[1] = 10;
  ArmRev_emxEnsureCapacity_char_T(iobj_0[1].NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    iobj_0[1].NameInternal->data[b_kstr] = tmp_3[b_kstr];
  }

  b_kstr = iobj_0[1].JointInternal.Type->size[0] * iobj_0[1].
    JointInternal.Type->size[1];
  iobj_0[1].JointInternal.Type->size[0] = 1;
  iobj_0[1].JointInternal.Type->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(iobj_0[1].JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_0[1].JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_0[1].JointInternal.Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_0[1].JointInternal.Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_0[1].JointInternal.Type->data[b_kstr];
  }

  result = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_2[b_kstr];
    }

    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  switch (b_kstr) {
   case 0:
    iobj_0[1].JointInternal.PositionNumber = 1.0;
    iobj_0[1].JointInternal.JointAxisInternal[0] = 0.0;
    iobj_0[1].JointInternal.JointAxisInternal[1] = 0.0;
    iobj_0[1].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    iobj_0[1].JointInternal.PositionNumber = 1.0;
    iobj_0[1].JointInternal.JointAxisInternal[0] = 0.0;
    iobj_0[1].JointInternal.JointAxisInternal[1] = 0.0;
    iobj_0[1].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    iobj_0[1].JointInternal.PositionNumber = 0.0;
    iobj_0[1].JointInternal.JointAxisInternal[0] = 0.0;
    iobj_0[1].JointInternal.JointAxisInternal[1] = 0.0;
    iobj_0[1].JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  iobj_0[1].ParentIndex = -1.0;
  ArmRe_CollisionSet_CollisionSet(&iobj_0[1].CollisionsInternal);
  iobj_0[1].matlabCodegenIsDeleted = false;
  obj->Bodies[1] = &iobj_0[1];
  b_kstr = iobj_0[2].NameInternal->size[0] * iobj_0[2].NameInternal->size[1];
  iobj_0[2].NameInternal->size[0] = 1;
  iobj_0[2].NameInternal->size[1] = 10;
  ArmRev_emxEnsureCapacity_char_T(iobj_0[2].NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    iobj_0[2].NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  b_kstr = iobj_0[2].JointInternal.Type->size[0] * iobj_0[2].
    JointInternal.Type->size[1];
  iobj_0[2].JointInternal.Type->size[0] = 1;
  iobj_0[2].JointInternal.Type->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(iobj_0[2].JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_0[2].JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_0[2].JointInternal.Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_0[2].JointInternal.Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_0[2].JointInternal.Type->data[b_kstr];
  }

  result = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_2[b_kstr];
    }

    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  switch (b_kstr) {
   case 0:
    iobj_0[2].JointInternal.PositionNumber = 1.0;
    iobj_0[2].JointInternal.JointAxisInternal[0] = 0.0;
    iobj_0[2].JointInternal.JointAxisInternal[1] = 0.0;
    iobj_0[2].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    iobj_0[2].JointInternal.PositionNumber = 1.0;
    iobj_0[2].JointInternal.JointAxisInternal[0] = 0.0;
    iobj_0[2].JointInternal.JointAxisInternal[1] = 0.0;
    iobj_0[2].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    iobj_0[2].JointInternal.PositionNumber = 0.0;
    iobj_0[2].JointInternal.JointAxisInternal[0] = 0.0;
    iobj_0[2].JointInternal.JointAxisInternal[1] = 0.0;
    iobj_0[2].JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  iobj_0[2].ParentIndex = -1.0;
  ArmRe_CollisionSet_CollisionSet(&iobj_0[2].CollisionsInternal);
  iobj_0[2].matlabCodegenIsDeleted = false;
  obj->Bodies[2] = &iobj_0[2];
  b_kstr = iobj_0[3].NameInternal->size[0] * iobj_0[3].NameInternal->size[1];
  iobj_0[3].NameInternal->size[0] = 1;
  iobj_0[3].NameInternal->size[1] = 10;
  ArmRev_emxEnsureCapacity_char_T(iobj_0[3].NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    iobj_0[3].NameInternal->data[b_kstr] = tmp_5[b_kstr];
  }

  b_kstr = iobj_0[3].JointInternal.Type->size[0] * iobj_0[3].
    JointInternal.Type->size[1];
  iobj_0[3].JointInternal.Type->size[0] = 1;
  iobj_0[3].JointInternal.Type->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(iobj_0[3].JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_0[3].JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_0[3].JointInternal.Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_0[3].JointInternal.Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_0[3].JointInternal.Type->data[b_kstr];
  }

  result = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_2[b_kstr];
    }

    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  switch (b_kstr) {
   case 0:
    iobj_0[3].JointInternal.PositionNumber = 1.0;
    iobj_0[3].JointInternal.JointAxisInternal[0] = 0.0;
    iobj_0[3].JointInternal.JointAxisInternal[1] = 0.0;
    iobj_0[3].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    iobj_0[3].JointInternal.PositionNumber = 1.0;
    iobj_0[3].JointInternal.JointAxisInternal[0] = 0.0;
    iobj_0[3].JointInternal.JointAxisInternal[1] = 0.0;
    iobj_0[3].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    iobj_0[3].JointInternal.PositionNumber = 0.0;
    iobj_0[3].JointInternal.JointAxisInternal[0] = 0.0;
    iobj_0[3].JointInternal.JointAxisInternal[1] = 0.0;
    iobj_0[3].JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  iobj_0[3].ParentIndex = -1.0;
  ArmRe_CollisionSet_CollisionSet(&iobj_0[3].CollisionsInternal);
  iobj_0[3].matlabCodegenIsDeleted = false;
  obj->Bodies[3] = &iobj_0[3];
  b_kstr = iobj_0[4].NameInternal->size[0] * iobj_0[4].NameInternal->size[1];
  iobj_0[4].NameInternal->size[0] = 1;
  iobj_0[4].NameInternal->size[1] = 10;
  ArmRev_emxEnsureCapacity_char_T(iobj_0[4].NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 10; b_kstr++) {
    iobj_0[4].NameInternal->data[b_kstr] = tmp_6[b_kstr];
  }

  b_kstr = iobj_0[4].JointInternal.Type->size[0] * iobj_0[4].
    JointInternal.Type->size[1];
  iobj_0[4].JointInternal.Type->size[0] = 1;
  iobj_0[4].JointInternal.Type->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(iobj_0[4].JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    iobj_0[4].JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_0[4].JointInternal.Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = iobj_0[4].JointInternal.Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = iobj_0[4].JointInternal.Type->data[b_kstr];
  }

  result = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_2[b_kstr];
    }

    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  ArmReverseKinema_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    iobj_0[4].JointInternal.PositionNumber = 1.0;
    iobj_0[4].JointInternal.JointAxisInternal[0] = 0.0;
    iobj_0[4].JointInternal.JointAxisInternal[1] = 0.0;
    iobj_0[4].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    iobj_0[4].JointInternal.PositionNumber = 1.0;
    iobj_0[4].JointInternal.JointAxisInternal[0] = 0.0;
    iobj_0[4].JointInternal.JointAxisInternal[1] = 0.0;
    iobj_0[4].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    iobj_0[4].JointInternal.PositionNumber = 0.0;
    iobj_0[4].JointInternal.JointAxisInternal[0] = 0.0;
    iobj_0[4].JointInternal.JointAxisInternal[1] = 0.0;
    iobj_0[4].JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  iobj_0[4].ParentIndex = -1.0;
  ArmRe_CollisionSet_CollisionSet(&iobj_0[4].CollisionsInternal);
  iobj_0[4].matlabCodegenIsDeleted = false;
  obj->Bodies[4] = &iobj_0[4];
}

static k_robotics_manip_internal_Rig_T *ArmReverseK_RigidBody_RigidBody
  (k_robotics_manip_internal_Rig_T *obj)
{
  emxArray_char_T_ArmReverseKin_T *switch_expression;
  k_robotics_manip_internal_Rig_T *b_obj;
  int32_T b_kstr;
  int32_T loop_ub;
  char_T a_0[9];
  char_T a[8];
  boolean_T result;
  static const char_T tmp[5] = { 'B', 'o', 'd', 'y', '4' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 3.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 8;
  ArmRev_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  ArmReverseKinema_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    a[b_kstr] = tmp_0[b_kstr];
  }

  result = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_1[b_kstr];
    }

    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  ArmReverseKinema_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_3[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 1.0;
  ArmRe_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static k_robotics_manip_internal_Rig_T *ArmRevers_RigidBody_RigidBody_e
  (k_robotics_manip_internal_Rig_T *obj)
{
  emxArray_char_T_ArmReverseKin_T *switch_expression;
  k_robotics_manip_internal_Rig_T *b_obj;
  int32_T b_kstr;
  int32_T loop_ub;
  char_T a_0[9];
  char_T a[8];
  boolean_T result;
  static const char_T tmp[5] = { 'B', 'o', 'd', 'y', '5' };

  static const char_T tmp_0[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_1[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_2[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_3[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_4[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T exitg1;
  b_obj = obj;
  b_kstr = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  obj->NameInternal->size[0] = 1;
  obj->NameInternal->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(obj->NameInternal, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->ParentIndex = 4.0;
  b_kstr = obj->JointInternal.Type->size[0] * obj->JointInternal.Type->size[1];
  obj->JointInternal.Type->size[0] = 1;
  obj->JointInternal.Type->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(obj->JointInternal.Type, b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  ArmReverseKinema_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->JointInternal.Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->JointInternal.Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    a[b_kstr] = tmp_1[b_kstr];
  }

  result = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_2[b_kstr];
    }

    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  ArmReverseKinema_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->JointInternal.PositionNumber = 1.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->JointInternal.PositionNumber = 0.0;
    obj->JointInternal.JointAxisInternal[0] = 0.0;
    obj->JointInternal.JointAxisInternal[1] = 0.0;
    obj->JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.JointToParentTransform[b_kstr] = tmp_3[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->JointInternal.ChildToJointTransform[b_kstr] = tmp_4[b_kstr];
  }

  obj->JointInternal.JointAxisInternal[0] = 0.0;
  obj->JointInternal.JointAxisInternal[1] = 0.0;
  obj->JointInternal.JointAxisInternal[2] = 0.0;
  ArmRe_CollisionSet_CollisionSet(&obj->CollisionsInternal);
  obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

static void ArmReverseKine_SystemCore_setup(robotics_slmanip_internal_blo_T *obj)
{
  emxArray_char_T_ArmReverseKin_T *switch_expression;
  int32_T b_kstr;
  int32_T loop_ub;
  char_T a_0[9];
  char_T a[8];
  boolean_T result;
  static const char_T tmp[5] = { 'B', 'o', 'd', 'y', '1' };

  static const char_T tmp_0[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_1[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  static const real_T tmp_2[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const real_T tmp_3[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  static const char_T tmp_4[5] = { 'B', 'o', 'd', 'y', '2' };

  static const real_T tmp_5[16] = { 6.123233995736766E-17, 0.0, 1.0, 0.0,
    -1.2246467991473532E-16, -1.0, 7.498798913309288E-33, 0.0, 1.0,
    -1.2246467991473532E-16, -6.123233995736766E-17, 0.0, 0.0, 0.0, 0.1, 1.0 };

  static const char_T tmp_6[5] = { 'B', 'o', 'd', 'y', '3' };

  static const real_T tmp_7[16] = { 1.0, 0.0, -0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0 };

  static const char_T tmp_8[5] = { 'f', 'i', 'x', 'e', 'd' };

  real_T unusedExpr[5];
  int32_T exitg1;
  obj->isInitialized = 1;
  ArmReverseKinematics_rand(unusedExpr);
  obj->TreeInternal.NumBodies = 5.0;
  RigidBodyTree_defaultInitialize(&obj->TreeInternal, &obj->TreeInternal._pobj0
    [0]);
  b_kstr = obj->TreeInternal._pobj0[5].NameInternal->size[0] *
    obj->TreeInternal._pobj0[5].NameInternal->size[1];
  obj->TreeInternal._pobj0[5].NameInternal->size[0] = 1;
  obj->TreeInternal._pobj0[5].NameInternal->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(obj->TreeInternal._pobj0[5].NameInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->TreeInternal._pobj0[5].NameInternal->data[b_kstr] = tmp[b_kstr];
  }

  obj->TreeInternal._pobj0[5].ParentIndex = 0.0;
  b_kstr = obj->TreeInternal._pobj0[5].JointInternal.Type->size[0] *
    obj->TreeInternal._pobj0[5].JointInternal.Type->size[1];
  obj->TreeInternal._pobj0[5].JointInternal.Type->size[0] = 1;
  obj->TreeInternal._pobj0[5].JointInternal.Type->size[1] = 8;
  ArmRev_emxEnsureCapacity_char_T(obj->TreeInternal._pobj0[5].JointInternal.Type,
    b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->TreeInternal._pobj0[5].JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  ArmReverseKinema_emxInit_char_T(&switch_expression, 2);
  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->TreeInternal._pobj0[5]
    .JointInternal.Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->TreeInternal._pobj0[5].JointInternal.Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->TreeInternal._pobj0[5].
      JointInternal.Type->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    a[b_kstr] = tmp_0[b_kstr];
  }

  result = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_1[b_kstr];
    }

    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  switch (b_kstr) {
   case 0:
    obj->TreeInternal._pobj0[5].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[5].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[5].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[5].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->TreeInternal._pobj0[5].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[5].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[5].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[5].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->TreeInternal._pobj0[5].JointInternal.PositionNumber = 0.0;
    obj->TreeInternal._pobj0[5].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[5].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[5].JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->TreeInternal._pobj0[5].JointInternal.JointToParentTransform[b_kstr] =
      tmp_2[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->TreeInternal._pobj0[5].JointInternal.ChildToJointTransform[b_kstr] =
      tmp_3[b_kstr];
  }

  obj->TreeInternal._pobj0[5].JointInternal.JointAxisInternal[0] = 0.0;
  obj->TreeInternal._pobj0[5].JointInternal.JointAxisInternal[1] = 0.0;
  obj->TreeInternal._pobj0[5].JointInternal.JointAxisInternal[2] = 1.0;
  ArmRe_CollisionSet_CollisionSet(&obj->TreeInternal._pobj0[5].
    CollisionsInternal);
  obj->TreeInternal._pobj0[5].matlabCodegenIsDeleted = false;
  obj->TreeInternal.Bodies[0] = &obj->TreeInternal._pobj0[5];
  b_kstr = obj->TreeInternal._pobj0[6].NameInternal->size[0] *
    obj->TreeInternal._pobj0[6].NameInternal->size[1];
  obj->TreeInternal._pobj0[6].NameInternal->size[0] = 1;
  obj->TreeInternal._pobj0[6].NameInternal->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(obj->TreeInternal._pobj0[6].NameInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->TreeInternal._pobj0[6].NameInternal->data[b_kstr] = tmp_4[b_kstr];
  }

  obj->TreeInternal._pobj0[6].ParentIndex = 1.0;
  b_kstr = obj->TreeInternal._pobj0[6].JointInternal.Type->size[0] *
    obj->TreeInternal._pobj0[6].JointInternal.Type->size[1];
  obj->TreeInternal._pobj0[6].JointInternal.Type->size[0] = 1;
  obj->TreeInternal._pobj0[6].JointInternal.Type->size[1] = 8;
  ArmRev_emxEnsureCapacity_char_T(obj->TreeInternal._pobj0[6].JointInternal.Type,
    b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->TreeInternal._pobj0[6].JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->TreeInternal._pobj0[6]
    .JointInternal.Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->TreeInternal._pobj0[6].JointInternal.Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->TreeInternal._pobj0[6].
      JointInternal.Type->data[b_kstr];
  }

  result = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_1[b_kstr];
    }

    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  switch (b_kstr) {
   case 0:
    obj->TreeInternal._pobj0[6].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[6].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[6].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[6].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->TreeInternal._pobj0[6].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[6].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[6].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[6].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->TreeInternal._pobj0[6].JointInternal.PositionNumber = 0.0;
    obj->TreeInternal._pobj0[6].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[6].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[6].JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->TreeInternal._pobj0[6].JointInternal.JointToParentTransform[b_kstr] =
      tmp_5[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->TreeInternal._pobj0[6].JointInternal.ChildToJointTransform[b_kstr] =
      tmp_3[b_kstr];
  }

  obj->TreeInternal._pobj0[6].JointInternal.JointAxisInternal[0] = 0.0;
  obj->TreeInternal._pobj0[6].JointInternal.JointAxisInternal[1] = 0.0;
  obj->TreeInternal._pobj0[6].JointInternal.JointAxisInternal[2] = 1.0;
  ArmRe_CollisionSet_CollisionSet(&obj->TreeInternal._pobj0[6].
    CollisionsInternal);
  obj->TreeInternal._pobj0[6].matlabCodegenIsDeleted = false;
  obj->TreeInternal.Bodies[1] = &obj->TreeInternal._pobj0[6];
  b_kstr = obj->TreeInternal._pobj0[7].NameInternal->size[0] *
    obj->TreeInternal._pobj0[7].NameInternal->size[1];
  obj->TreeInternal._pobj0[7].NameInternal->size[0] = 1;
  obj->TreeInternal._pobj0[7].NameInternal->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(obj->TreeInternal._pobj0[7].NameInternal,
    b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->TreeInternal._pobj0[7].NameInternal->data[b_kstr] = tmp_6[b_kstr];
  }

  obj->TreeInternal._pobj0[7].ParentIndex = 2.0;
  b_kstr = obj->TreeInternal._pobj0[7].JointInternal.Type->size[0] *
    obj->TreeInternal._pobj0[7].JointInternal.Type->size[1];
  obj->TreeInternal._pobj0[7].JointInternal.Type->size[0] = 1;
  obj->TreeInternal._pobj0[7].JointInternal.Type->size[1] = 8;
  ArmRev_emxEnsureCapacity_char_T(obj->TreeInternal._pobj0[7].JointInternal.Type,
    b_kstr);
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    obj->TreeInternal._pobj0[7].JointInternal.Type->data[b_kstr] = tmp_0[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->TreeInternal._pobj0[7]
    .JointInternal.Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->TreeInternal._pobj0[7].JointInternal.Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->TreeInternal._pobj0[7].
      JointInternal.Type->data[b_kstr];
  }

  result = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_1[b_kstr];
    }

    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  switch (b_kstr) {
   case 0:
    obj->TreeInternal._pobj0[7].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[7].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[7].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[7].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->TreeInternal._pobj0[7].JointInternal.PositionNumber = 1.0;
    obj->TreeInternal._pobj0[7].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[7].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[7].JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->TreeInternal._pobj0[7].JointInternal.PositionNumber = 0.0;
    obj->TreeInternal._pobj0[7].JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal._pobj0[7].JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal._pobj0[7].JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->TreeInternal._pobj0[7].JointInternal.JointToParentTransform[b_kstr] =
      tmp_7[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    obj->TreeInternal._pobj0[7].JointInternal.ChildToJointTransform[b_kstr] =
      tmp_3[b_kstr];
  }

  obj->TreeInternal._pobj0[7].JointInternal.JointAxisInternal[0] = 0.0;
  obj->TreeInternal._pobj0[7].JointInternal.JointAxisInternal[1] = 0.0;
  obj->TreeInternal._pobj0[7].JointInternal.JointAxisInternal[2] = 1.0;
  ArmRe_CollisionSet_CollisionSet(&obj->TreeInternal._pobj0[7].
    CollisionsInternal);
  obj->TreeInternal._pobj0[7].matlabCodegenIsDeleted = false;
  obj->TreeInternal.Bodies[2] = &obj->TreeInternal._pobj0[7];
  obj->TreeInternal.Bodies[3] = ArmReverseK_RigidBody_RigidBody
    (&obj->TreeInternal._pobj0[8]);
  obj->TreeInternal.Bodies[4] = ArmRevers_RigidBody_RigidBody_e
    (&obj->TreeInternal._pobj0[9]);
  obj->TreeInternal.PositionNumber = 4.0;
  b_kstr = obj->TreeInternal.Base.NameInternal->size[0] *
    obj->TreeInternal.Base.NameInternal->size[1];
  obj->TreeInternal.Base.NameInternal->size[0] = 1;
  obj->TreeInternal.Base.NameInternal->size[1] = 4;
  ArmRev_emxEnsureCapacity_char_T(obj->TreeInternal.Base.NameInternal, b_kstr);
  obj->TreeInternal.Base.NameInternal->data[0] = 'B';
  obj->TreeInternal.Base.NameInternal->data[1] = 'a';
  obj->TreeInternal.Base.NameInternal->data[2] = 's';
  obj->TreeInternal.Base.NameInternal->data[3] = 'e';
  obj->TreeInternal.Base.ParentIndex = -1.0;
  b_kstr = obj->TreeInternal.Base.JointInternal.Type->size[0] *
    obj->TreeInternal.Base.JointInternal.Type->size[1];
  obj->TreeInternal.Base.JointInternal.Type->size[0] = 1;
  obj->TreeInternal.Base.JointInternal.Type->size[1] = 5;
  ArmRev_emxEnsureCapacity_char_T(obj->TreeInternal.Base.JointInternal.Type,
    b_kstr);
  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    obj->TreeInternal.Base.JointInternal.Type->data[b_kstr] = tmp_8[b_kstr];
  }

  b_kstr = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->TreeInternal.Base.JointInternal.Type->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, b_kstr);
  loop_ub = obj->TreeInternal.Base.JointInternal.Type->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    switch_expression->data[b_kstr] = obj->
      TreeInternal.Base.JointInternal.Type->data[b_kstr];
  }

  result = false;
  if (switch_expression->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (a[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    b_kstr = 0;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      a_0[b_kstr] = tmp_1[b_kstr];
    }

    if (switch_expression->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (a_0[b_kstr - 1] != switch_expression->data[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      b_kstr = 1;
    } else {
      b_kstr = -1;
    }
  }

  ArmReverseKinema_emxFree_char_T(&switch_expression);
  switch (b_kstr) {
   case 0:
    obj->TreeInternal.Base.JointInternal.PositionNumber = 1.0;
    obj->TreeInternal.Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal.Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal.Base.JointInternal.JointAxisInternal[2] = 1.0;
    break;

   case 1:
    obj->TreeInternal.Base.JointInternal.PositionNumber = 1.0;
    obj->TreeInternal.Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal.Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal.Base.JointInternal.JointAxisInternal[2] = 1.0;
    break;

   default:
    obj->TreeInternal.Base.JointInternal.PositionNumber = 0.0;
    obj->TreeInternal.Base.JointInternal.JointAxisInternal[0] = 0.0;
    obj->TreeInternal.Base.JointInternal.JointAxisInternal[1] = 0.0;
    obj->TreeInternal.Base.JointInternal.JointAxisInternal[2] = 0.0;
    break;
  }

  ArmRe_CollisionSet_CollisionSet(&obj->TreeInternal.Base.CollisionsInternal);
  obj->TreeInternal.Base.matlabCodegenIsDeleted = false;
  obj->TreeInternal.matlabCodegenIsDeleted = false;
}

static void RigidBodyTree_get_JointPosition(x_robotics_manip_internal_Rig_T *obj,
  emxArray_real_T_ArmReverseKin_T *limits)
{
  c_rigidBodyJoint_ArmReverse_e_T *obj_0;
  emxArray_char_T_ArmReverseKin_T *a;
  emxArray_real_T_ArmReverseKin_T *h;
  v_robotics_manip_internal_Rig_T *body;
  real_T k;
  real_T pnum;
  int32_T b_kstr;
  int32_T kstr;
  int32_T limits_0;
  int32_T loop_ub;
  char_T b[5];
  boolean_T b_bool;
  static const char_T tmp[5] = { 'f', 'i', 'x', 'e', 'd' };

  int32_T c_tmp;
  int32_T exitg1;
  int32_T i;
  i = limits->size[0] * limits->size[1];
  limits->size[0] = (int32_T)obj->PositionNumber;
  limits->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(limits, i);
  loop_ub = (int32_T)obj->PositionNumber << 1;
  if (loop_ub - 1 >= 0) {
    memset(&limits->data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
  }

  k = 1.0;
  pnum = obj->NumBodies;
  c_tmp = (int32_T)pnum - 1;
  if ((int32_T)pnum - 1 >= 0) {
    for (i = 0; i < 5; i++) {
      b[i] = tmp[i];
    }
  }

  ArmReverseKinema_emxInit_real_T(&h, 2);
  ArmReverseKinema_emxInit_char_T(&a, 2);
  for (limits_0 = 0; limits_0 <= c_tmp; limits_0++) {
    body = obj->Bodies[limits_0];
    i = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = body->JointInternal->Type->size[1];
    ArmRev_emxEnsureCapacity_char_T(a, i);
    loop_ub = body->JointInternal->Type->size[1];
    for (i = 0; i < loop_ub; i++) {
      a->data[i] = body->JointInternal->Type->data[i];
    }

    b_bool = false;
    if (a->size[1] != 5) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 5) {
          if (a->data[b_kstr - 1] != b[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (!b_bool) {
      pnum = body->JointInternal->PositionNumber;
      pnum += k;
      if (k > pnum - 1.0) {
        b_kstr = 0;
        kstr = 0;
      } else {
        b_kstr = (int32_T)k - 1;
        kstr = (int32_T)(pnum - 1.0);
      }

      obj_0 = body->JointInternal;
      i = h->size[0] * h->size[1];
      h->size[0] = obj_0->PositionLimitsInternal->size[0];
      h->size[1] = 2;
      ArmRev_emxEnsureCapacity_real_T(h, i);
      loop_ub = obj_0->PositionLimitsInternal->size[0] << 1;
      for (i = 0; i < loop_ub; i++) {
        h->data[i] = obj_0->PositionLimitsInternal->data[i];
      }

      kstr -= b_kstr;
      for (i = 0; i < 2; i++) {
        for (loop_ub = 0; loop_ub < kstr; loop_ub++) {
          limits->data[(b_kstr + loop_ub) + limits->size[0] * i] = h->data
            [h->size[0] * i + loop_ub];
        }
      }

      k = pnum;
    }
  }

  ArmReverseKinema_emxFree_char_T(&a);
  ArmReverseKinema_emxFree_real_T(&h);
}

static void ArmReverseKinema_emxInit_int8_T(emxArray_int8_T_ArmReverseKin_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_int8_T_ArmReverseKin_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_int8_T_ArmReverseKin_T *)malloc(sizeof
    (emxArray_int8_T_ArmReverseKin_T));
  emxArray = *pEmxArray;
  emxArray->data = (int8_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * (uint32_T)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void ArmRev_emxEnsureCapacity_int8_T(emxArray_int8_T_ArmReverseKin_T
  *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = malloc((uint32_T)i * sizeof(int8_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(int8_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (int8_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void ArmReverseKinema_emxFree_int8_T(emxArray_int8_T_ArmReverseKin_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_int8_T_ArmReverseKin_T *)NULL) {
    if (((*pEmxArray)->data != (int8_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_int8_T_ArmReverseKin_T *)NULL;
  }
}

static void ArmReverseKi_binary_expand_op_4(boolean_T in1[4], const real_T in2[4],
  const emxArray_real_T_ArmReverseKin_T *in3)
{
  int32_T stride_0_0;
  stride_0_0 = (in3->size[0] != 1);
  in1[0] = (in2[0] <= in3->data[in3->size[0]] + 4.4408920985006262E-16);
  in1[1] = (in2[1] <= in3->data[stride_0_0 + in3->size[0]] +
            4.4408920985006262E-16);
  in1[2] = (in2[2] <= in3->data[(stride_0_0 << 1) + in3->size[0]] +
            4.4408920985006262E-16);
  in1[3] = (in2[3] <= in3->data[3 * stride_0_0 + in3->size[0]] +
            4.4408920985006262E-16);
}

static void ArmReverseKi_binary_expand_op_3(boolean_T in1[4], const real_T in2[4],
  const emxArray_real_T_ArmReverseKin_T *in3)
{
  int32_T stride_0_0;
  stride_0_0 = (in3->size[0] != 1);
  in1[0] = (in2[0] >= in3->data[0] - 4.4408920985006262E-16);
  in1[1] = (in2[1] >= in3->data[stride_0_0] - 4.4408920985006262E-16);
  in1[2] = (in2[2] >= in3->data[stride_0_0 << 1] - 4.4408920985006262E-16);
  in1[3] = (in2[3] >= in3->data[3 * stride_0_0] - 4.4408920985006262E-16);
}

static void ArmReverseKinematics_eml_find(const boolean_T x[4], int32_T i_data[],
  int32_T *i_size)
{
  int32_T b_ii;
  int32_T idx;
  boolean_T exitg1;
  idx = 0;
  b_ii = 1;
  exitg1 = false;
  while ((!exitg1) && (b_ii - 1 < 4)) {
    if (x[b_ii - 1]) {
      idx++;
      i_data[idx - 1] = b_ii;
      if (idx >= 4) {
        exitg1 = true;
      } else {
        b_ii++;
      }
    } else {
      b_ii++;
    }
  }

  if (idx < 1) {
    *i_size = 0;
  } else {
    *i_size = idx;
  }
}

static void ArmReverseKinematics_tic(real_T *tstart_tv_sec, real_T
  *tstart_tv_nsec)
{
  coderTimespec b_timespec;
  if (!rtDW.method_not_empty) {
    rtDW.method_not_empty = true;
    coderInitTimeFunctions(&rtDW.freq);
  }

  coderTimeClockGettimeMonotonic(&b_timespec, rtDW.freq);
  *tstart_tv_sec = b_timespec.tv_sec;
  *tstart_tv_nsec = b_timespec.tv_nsec;
}

static void A_RigidBodyTree_ancestorIndices(x_robotics_manip_internal_Rig_T *obj,
  v_robotics_manip_internal_Rig_T *body, emxArray_real_T_ArmReverseKin_T
  *indices)
{
  real_T i;
  int32_T loop_ub;
  loop_ub = indices->size[0] * indices->size[1];
  indices->size[0] = 1;
  indices->size[1] = (int32_T)(obj->NumBodies + 1.0);
  ArmRev_emxEnsureCapacity_real_T(indices, loop_ub);
  loop_ub = (int32_T)(obj->NumBodies + 1.0);
  if (loop_ub - 1 >= 0) {
    memset(&indices->data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
  }

  i = 2.0;
  indices->data[0] = body->Index;
  while (body->ParentIndex > 0.0) {
    body = obj->Bodies[(int32_T)body->ParentIndex - 1];
    indices->data[(int32_T)i - 1] = body->Index;
    i++;
  }

  if (body->Index > 0.0) {
    indices->data[(int32_T)i - 1] = body->ParentIndex;
    i++;
  }

  loop_ub = indices->size[0] * indices->size[1];
  indices->size[0] = 1;
  indices->size[1] = (int32_T)(i - 1.0);
  ArmRev_emxEnsureCapacity_real_T(indices, loop_ub);
}

static void RigidBodyTree_kinematicPathInte(x_robotics_manip_internal_Rig_T *obj,
  v_robotics_manip_internal_Rig_T *body1, v_robotics_manip_internal_Rig_T *body2,
  emxArray_real_T_ArmReverseKin_T *indices)
{
  emxArray_real_T_ArmReverseKin_T *ancestorIndices1;
  emxArray_real_T_ArmReverseKin_T *ancestorIndices2;
  int32_T b_i;
  int32_T d;
  int32_T d_tmp;
  int32_T h;
  int32_T loop_ub;
  int32_T minPathLength;
  int32_T tmp;
  boolean_T exitg1;
  ArmReverseKinema_emxInit_real_T(&ancestorIndices1, 2);
  A_RigidBodyTree_ancestorIndices(obj, body1, ancestorIndices1);
  ArmReverseKinema_emxInit_real_T(&ancestorIndices2, 2);
  A_RigidBodyTree_ancestorIndices(obj, body2, ancestorIndices2);
  minPathLength = (int32_T)fmin(ancestorIndices1->size[1],
    ancestorIndices2->size[1]);
  b_i = 0;
  exitg1 = false;
  while ((!exitg1) && (b_i <= minPathLength - 2)) {
    if (ancestorIndices1->data[(ancestorIndices1->size[1] - b_i) - 2] !=
        ancestorIndices2->data[(ancestorIndices2->size[1] - b_i) - 2]) {
      minPathLength = b_i + 1;
      exitg1 = true;
    } else {
      b_i++;
    }
  }

  d_tmp = ancestorIndices1->size[1] - minPathLength;
  if (d_tmp < 1) {
    b_i = 0;
  } else {
    b_i = d_tmp;
  }

  d = ancestorIndices2->size[1] - minPathLength;
  if (d < 1) {
    d = 1;
    h = 1;
    minPathLength = 0;
  } else {
    h = -1;
    minPathLength = 1;
  }

  tmp = indices->size[0] * indices->size[1];
  indices->size[0] = 1;
  loop_ub = div_s32(minPathLength - d, h);
  indices->size[1] = (loop_ub + b_i) + 2;
  ArmRev_emxEnsureCapacity_real_T(indices, tmp);
  if (b_i - 1 >= 0) {
    memcpy(&indices->data[0], &ancestorIndices1->data[0], (uint32_T)b_i * sizeof
           (real_T));
  }

  indices->data[b_i] = ancestorIndices1->data[d_tmp];
  ArmReverseKinema_emxFree_real_T(&ancestorIndices1);
  for (minPathLength = 0; minPathLength <= loop_ub; minPathLength++) {
    indices->data[(minPathLength + b_i) + 1] = ancestorIndices2->data[(h *
      minPathLength + d) - 1];
  }

  ArmReverseKinema_emxFree_real_T(&ancestorIndices2);
}

static void rigidBodyJoint_get_JointAxis_e(const c_rigidBodyJoint_ArmReverse_e_T
  *obj, real_T ax[3])
{
  int32_T b_kstr;
  char_T b_0[9];
  char_T b[8];
  boolean_T b_bool;
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  boolean_T guard1;
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp[b_kstr];
  }

  b_bool = false;
  if (obj->Type->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (obj->Type->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (b_bool) {
    guard1 = true;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_0[b_kstr];
    }

    if (obj->Type->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (obj->Type->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      guard1 = true;
    } else {
      ax[0] = (rtNaN);
      ax[1] = (rtNaN);
      ax[2] = (rtNaN);
    }
  }

  if (guard1) {
    ax[0] = obj->JointAxisInternal[0];
    ax[1] = obj->JointAxisInternal[1];
    ax[2] = obj->JointAxisInternal[2];
  }
}

static void ArmReverseKinematics_cat(real_T varargin_1, real_T varargin_2,
  real_T varargin_3, real_T varargin_4, real_T varargin_5, real_T varargin_6,
  real_T varargin_7, real_T varargin_8, real_T varargin_9, real_T y[9])
{
  y[0] = varargin_1;
  y[1] = varargin_2;
  y[2] = varargin_3;
  y[3] = varargin_4;
  y[4] = varargin_5;
  y[5] = varargin_6;
  y[6] = varargin_7;
  y[7] = varargin_8;
  y[8] = varargin_9;
}

static void ArmReverseKinematics_mtimes(const real_T A[36], const
  emxArray_real_T_ArmReverseKin_T *B_0, emxArray_real_T_ArmReverseKin_T *C)
{
  real_T s;
  int32_T b_i;
  int32_T b_j;
  int32_T b_k;
  int32_T coffset_tmp;
  int32_T n;
  n = B_0->size[1] - 1;
  b_j = C->size[0] * C->size[1];
  C->size[0] = 6;
  C->size[1] = B_0->size[1];
  ArmRev_emxEnsureCapacity_real_T(C, b_j);
  for (b_j = 0; b_j <= n; b_j++) {
    coffset_tmp = b_j * 6 - 1;
    for (b_i = 0; b_i < 6; b_i++) {
      s = 0.0;
      for (b_k = 0; b_k < 6; b_k++) {
        s += A[b_k * 6 + b_i] * B_0->data[(coffset_tmp + b_k) + 1];
      }

      C->data[(coffset_tmp + b_i) + 1] = s;
    }
  }
}

static void RigidBodyTree_efficientFKAndJac(x_robotics_manip_internal_Rig_T *obj,
  const real_T qv[4], real_T bid1, real_T T_data[], int32_T T_size[2],
  emxArray_real_T_ArmReverseKin_T *Jac)
{
  __m128d tmp_1;
  c_rigidBodyJoint_ArmReverse_e_T *joint;
  emxArray_char_T_ArmReverseKin_T *a;
  emxArray_real_T_ArmReverseKin_T *b;
  emxArray_real_T_ArmReverseKin_T *kinematicPathIndices;
  emxArray_real_T_ArmReverseKin_T *tmp;
  v_robotics_manip_internal_Rig_T *body1;
  v_robotics_manip_internal_Rig_T *nextBody;
  real_T Tj_1[36];
  real_T T1[16];
  real_T T1j[16];
  real_T Tc2p[16];
  real_T Tj[16];
  real_T Tj_0[16];
  real_T b_1[16];
  real_T R[9];
  real_T R_0[9];
  real_T R_1[9];
  real_T R_2[9];
  real_T tempR[9];
  real_T tmp_0[9];
  real_T result_data[4];
  real_T v[3];
  real_T qidx_idx_0;
  real_T qidx_idx_1;
  real_T tempR_tmp;
  real_T tempR_tmp_0;
  real_T tempR_tmp_1;
  real_T theta;
  int32_T Jac_0;
  int32_T c;
  int32_T f;
  int32_T g;
  int32_T i;
  int32_T jointSign;
  int32_T k;
  int32_T loop_ub;
  char_T a_0[8];
  char_T b_0[5];
  boolean_T b_bool;
  boolean_T nextBodyIsParent;
  static const char_T tmp_2[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_3[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  emxArray_real_T_ArmReverseKin_T *Jac_1;
  real_T qidx_idx_0_tmp;
  int32_T exitg1;
  ArmReverseKinema_emxInit_real_T(&tmp, 2);
  if (bid1 >= 0.0) {
    if (bid1 == 0.0) {
      body1 = &obj->Base;
    } else {
      body1 = obj->Bodies[(int32_T)bid1 - 1];
    }

    ArmReverseKinema_emxInit_real_T(&kinematicPathIndices, 2);
    RigidBodyTree_kinematicPathInte(obj, body1, &obj->Base, kinematicPathIndices);
    memset(&T1[0], 0, sizeof(real_T) << 4U);
    T1[0] = 1.0;
    T1[5] = 1.0;
    T1[10] = 1.0;
    T1[15] = 1.0;
    loop_ub = Jac->size[0] * Jac->size[1];
    Jac->size[0] = 6;
    Jac->size[1] = (int32_T)obj->PositionNumber;
    ArmRev_emxEnsureCapacity_real_T(Jac, loop_ub);
    loop_ub = 6 * (int32_T)obj->PositionNumber;
    if (loop_ub - 1 >= 0) {
      memset(&Jac->data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
    }

    c = kinematicPathIndices->size[1] - 2;
    if (kinematicPathIndices->size[1] - 2 >= 0) {
      for (i = 0; i < 5; i++) {
        b_0[i] = tmp_2[i];
      }
    }

    ArmReverseKinema_emxInit_real_T(&b, 2);
    ArmReverseKinema_emxInit_char_T(&a, 2);
    for (Jac_0 = 0; Jac_0 <= c; Jac_0++) {
      qidx_idx_0 = kinematicPathIndices->data[Jac_0];
      if (qidx_idx_0 != 0.0) {
        body1 = obj->Bodies[(int32_T)qidx_idx_0 - 1];
      } else {
        body1 = &obj->Base;
      }

      qidx_idx_0 = kinematicPathIndices->data[Jac_0 + 1];
      if (qidx_idx_0 != 0.0) {
        nextBody = obj->Bodies[(int32_T)qidx_idx_0 - 1];
      } else {
        nextBody = &obj->Base;
      }

      nextBodyIsParent = (nextBody->Index == body1->ParentIndex);
      if (nextBodyIsParent) {
        nextBody = body1;
        jointSign = 1;
      } else {
        jointSign = -1;
      }

      joint = nextBody->JointInternal;
      loop_ub = a->size[0] * a->size[1];
      a->size[0] = 1;
      a->size[1] = joint->Type->size[1];
      ArmRev_emxEnsureCapacity_char_T(a, loop_ub);
      loop_ub = joint->Type->size[1];
      for (i = 0; i < loop_ub; i++) {
        a->data[i] = joint->Type->data[i];
      }

      b_bool = false;
      if (a->size[1] != 5) {
      } else {
        k = 1;
        do {
          exitg1 = 0;
          if (k - 1 < 5) {
            if (a->data[k - 1] != b_0[k - 1]) {
              exitg1 = 1;
            } else {
              k++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (b_bool) {
        for (i = 0; i < 16; i++) {
          Tj[i] = joint->JointToParentTransform[i];
        }

        loop_ub = a->size[0] * a->size[1];
        a->size[0] = 1;
        a->size[1] = joint->Type->size[1];
        ArmRev_emxEnsureCapacity_char_T(a, loop_ub);
        loop_ub = joint->Type->size[1];
        for (i = 0; i < loop_ub; i++) {
          a->data[i] = joint->Type->data[i];
        }

        b_bool = false;
        if (a->size[1] != 5) {
        } else {
          k = 1;
          do {
            exitg1 = 0;
            if (k - 1 < 5) {
              if (b_0[k - 1] != a->data[k - 1]) {
                exitg1 = 1;
              } else {
                k++;
              }
            } else {
              b_bool = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (b_bool) {
          qidx_idx_0 = 0.0;
        } else {
          for (i = 0; i < 8; i++) {
            a_0[i] = tmp_3[i];
          }

          if (a->size[1] != 8) {
          } else {
            k = 1;
            do {
              exitg1 = 0;
              if (k - 1 < 8) {
                if (a_0[k - 1] != a->data[k - 1]) {
                  exitg1 = 1;
                } else {
                  k++;
                }
              } else {
                b_bool = true;
                exitg1 = 1;
              }
            } while (exitg1 == 0);
          }

          if (b_bool) {
            qidx_idx_0 = 1.0;
          } else {
            qidx_idx_0 = -1.0;
          }
        }

        switch ((int32_T)qidx_idx_0) {
         case 0:
          memset(&T1j[0], 0, sizeof(real_T) << 4U);
          T1j[0] = 1.0;
          T1j[5] = 1.0;
          T1j[10] = 1.0;
          T1j[15] = 1.0;
          break;

         case 1:
          rigidBodyJoint_get_JointAxis_e(joint, v);
          qidx_idx_1 = v[0];
          qidx_idx_0_tmp = v[1];
          tempR_tmp = v[2];
          qidx_idx_0 = 1.0 / sqrt((qidx_idx_1 * qidx_idx_1 + qidx_idx_0_tmp *
            qidx_idx_0_tmp) + tempR_tmp * tempR_tmp);
          v[0] = qidx_idx_1 * qidx_idx_0;
          v[1] = qidx_idx_0_tmp * qidx_idx_0;
          v[2] = tempR_tmp * qidx_idx_0;
          qidx_idx_1 = v[0] * v[1] * 0.0;
          qidx_idx_0_tmp = v[0] * v[2] * 0.0;
          tempR_tmp = v[1] * v[2] * 0.0;
          ArmReverseKinematics_cat(v[0] * v[0] * 0.0 + 1.0, qidx_idx_1 - v[2] *
            0.0, qidx_idx_0_tmp + v[1] * 0.0, qidx_idx_1 + v[2] * 0.0, v[1] * v
            [1] * 0.0 + 1.0, tempR_tmp - v[0] * 0.0, qidx_idx_0_tmp - v[1] * 0.0,
            tempR_tmp + v[0] * 0.0, v[2] * v[2] * 0.0 + 1.0, tempR);
          for (g = 0; g < 3; g++) {
            R[g] = tempR[g * 3];
            R[g + 3] = tempR[g * 3 + 1];
            R[g + 6] = tempR[g * 3 + 2];
          }

          memset(&T1j[0], 0, sizeof(real_T) << 4U);
          for (i = 0; i < 3; i++) {
            g = i << 2;
            T1j[g] = R[3 * i];
            T1j[g + 1] = R[3 * i + 1];
            T1j[g + 2] = R[3 * i + 2];
          }

          T1j[15] = 1.0;
          break;

         default:
          rigidBodyJoint_get_JointAxis_e(joint, v);
          memset(&tempR[0], 0, 9U * sizeof(real_T));
          tempR[0] = 1.0;
          tempR[4] = 1.0;
          tempR[8] = 1.0;
          for (i = 0; i < 3; i++) {
            g = i << 2;
            T1j[g] = tempR[3 * i];
            T1j[g + 1] = tempR[3 * i + 1];
            T1j[g + 2] = tempR[3 * i + 2];
            T1j[i + 12] = v[i] * 0.0;
          }

          T1j[3] = 0.0;
          T1j[7] = 0.0;
          T1j[11] = 0.0;
          T1j[15] = 1.0;
          break;
        }

        for (i = 0; i < 16; i++) {
          b_1[i] = joint->ChildToJointTransform[i];
        }

        for (i = 0; i < 4; i++) {
          qidx_idx_0_tmp = Tj[i + 4];
          tempR_tmp = Tj[i];
          tempR_tmp_0 = Tj[i + 8];
          tempR_tmp_1 = Tj[i + 12];
          for (loop_ub = 0; loop_ub < 4; loop_ub++) {
            f = loop_ub << 2;
            Tj_0[i + f] = ((T1j[f + 1] * qidx_idx_0_tmp + T1j[f] * tempR_tmp) +
                           T1j[f + 2] * tempR_tmp_0) + T1j[f + 3] * tempR_tmp_1;
          }

          qidx_idx_0_tmp = Tj_0[i + 4];
          tempR_tmp = Tj_0[i];
          tempR_tmp_0 = Tj_0[i + 8];
          tempR_tmp_1 = Tj_0[i + 12];
          for (loop_ub = 0; loop_ub < 4; loop_ub++) {
            g = loop_ub << 2;
            Tc2p[i + g] = ((b_1[g + 1] * qidx_idx_0_tmp + b_1[g] * tempR_tmp) +
                           b_1[g + 2] * tempR_tmp_0) + b_1[g + 3] * tempR_tmp_1;
          }
        }
      } else {
        i = (int32_T)nextBody->Index;
        qidx_idx_0 = obj->PositionDoFMap[i - 1];
        qidx_idx_1 = obj->PositionDoFMap[i + 4];
        if (qidx_idx_0 > qidx_idx_1) {
          g = 0;
          f = 0;
        } else {
          g = (int32_T)qidx_idx_0 - 1;
          f = (int32_T)qidx_idx_1;
        }

        for (i = 0; i < 16; i++) {
          Tj[i] = joint->JointToParentTransform[i];
        }

        loop_ub = a->size[0] * a->size[1];
        a->size[0] = 1;
        a->size[1] = joint->Type->size[1];
        ArmRev_emxEnsureCapacity_char_T(a, loop_ub);
        loop_ub = joint->Type->size[1];
        for (i = 0; i < loop_ub; i++) {
          a->data[i] = joint->Type->data[i];
        }

        if (a->size[1] != 5) {
        } else {
          k = 1;
          do {
            exitg1 = 0;
            if (k - 1 < 5) {
              if (b_0[k - 1] != a->data[k - 1]) {
                exitg1 = 1;
              } else {
                k++;
              }
            } else {
              b_bool = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (b_bool) {
          qidx_idx_0 = 0.0;
        } else {
          for (i = 0; i < 8; i++) {
            a_0[i] = tmp_3[i];
          }

          if (a->size[1] != 8) {
          } else {
            k = 1;
            do {
              exitg1 = 0;
              if (k - 1 < 8) {
                if (a_0[k - 1] != a->data[k - 1]) {
                  exitg1 = 1;
                } else {
                  k++;
                }
              } else {
                b_bool = true;
                exitg1 = 1;
              }
            } while (exitg1 == 0);
          }

          if (b_bool) {
            qidx_idx_0 = 1.0;
          } else {
            qidx_idx_0 = -1.0;
          }
        }

        switch ((int32_T)qidx_idx_0) {
         case 0:
          memset(&T1j[0], 0, sizeof(real_T) << 4U);
          T1j[0] = 1.0;
          T1j[5] = 1.0;
          T1j[10] = 1.0;
          T1j[15] = 1.0;
          break;

         case 1:
          rigidBodyJoint_get_JointAxis_e(joint, v);
          result_data[0] = v[0];
          result_data[1] = v[1];
          result_data[2] = v[2];
          if ((f - g != 0) - 1 >= 0) {
            result_data[3] = qv[g];
          }

          qidx_idx_1 = result_data[0];
          theta = result_data[1];
          qidx_idx_0_tmp = result_data[2];
          qidx_idx_0 = 1.0 / sqrt((qidx_idx_1 * qidx_idx_1 + theta * theta) +
            qidx_idx_0_tmp * qidx_idx_0_tmp);
          v[0] = qidx_idx_1 * qidx_idx_0;
          v[1] = theta * qidx_idx_0;
          v[2] = qidx_idx_0_tmp * qidx_idx_0;
          theta = result_data[3];
          qidx_idx_0 = cos(theta);
          theta = sin(theta);
          qidx_idx_1 = v[0] * v[1] * (1.0 - qidx_idx_0);
          qidx_idx_0_tmp = v[2] * theta;
          tempR_tmp = v[0] * v[2] * (1.0 - qidx_idx_0);
          tempR_tmp_0 = v[1] * theta;
          tempR_tmp_1 = v[1] * v[2] * (1.0 - qidx_idx_0);
          theta *= v[0];
          ArmReverseKinematics_cat(v[0] * v[0] * (1.0 - qidx_idx_0) + qidx_idx_0,
            qidx_idx_1 - qidx_idx_0_tmp, tempR_tmp + tempR_tmp_0, qidx_idx_1 +
            qidx_idx_0_tmp, v[1] * v[1] * (1.0 - qidx_idx_0) + qidx_idx_0,
            tempR_tmp_1 - theta, tempR_tmp - tempR_tmp_0, tempR_tmp_1 + theta,
            v[2] * v[2] * (1.0 - qidx_idx_0) + qidx_idx_0, tempR);
          for (g = 0; g < 3; g++) {
            R[g] = tempR[g * 3];
            R[g + 3] = tempR[g * 3 + 1];
            R[g + 6] = tempR[g * 3 + 2];
          }

          memset(&T1j[0], 0, sizeof(real_T) << 4U);
          for (i = 0; i < 3; i++) {
            g = i << 2;
            T1j[g] = R[3 * i];
            T1j[g + 1] = R[3 * i + 1];
            T1j[g + 2] = R[3 * i + 2];
          }

          T1j[15] = 1.0;
          break;

         default:
          rigidBodyJoint_get_JointAxis_e(joint, v);
          memset(&tempR[0], 0, 9U * sizeof(real_T));
          tempR[0] = 1.0;
          tempR[4] = 1.0;
          tempR[8] = 1.0;
          qidx_idx_0 = qv[g];
          for (i = 0; i < 3; i++) {
            g = i << 2;
            T1j[g] = tempR[3 * i];
            T1j[g + 1] = tempR[3 * i + 1];
            T1j[g + 2] = tempR[3 * i + 2];
            T1j[i + 12] = v[i] * qidx_idx_0;
          }

          T1j[3] = 0.0;
          T1j[7] = 0.0;
          T1j[11] = 0.0;
          T1j[15] = 1.0;
          break;
        }

        for (i = 0; i < 16; i++) {
          b_1[i] = joint->ChildToJointTransform[i];
        }

        for (i = 0; i < 4; i++) {
          qidx_idx_0_tmp = Tj[i + 4];
          tempR_tmp = Tj[i];
          tempR_tmp_0 = Tj[i + 8];
          tempR_tmp_1 = Tj[i + 12];
          for (loop_ub = 0; loop_ub < 4; loop_ub++) {
            f = loop_ub << 2;
            Tj_0[i + f] = ((T1j[f + 1] * qidx_idx_0_tmp + T1j[f] * tempR_tmp) +
                           T1j[f + 2] * tempR_tmp_0) + T1j[f + 3] * tempR_tmp_1;
          }

          qidx_idx_0_tmp = Tj_0[i + 4];
          tempR_tmp = Tj_0[i];
          tempR_tmp_0 = Tj_0[i + 8];
          tempR_tmp_1 = Tj_0[i + 12];
          for (loop_ub = 0; loop_ub < 4; loop_ub++) {
            g = loop_ub << 2;
            Tc2p[i + g] = ((b_1[g + 1] * qidx_idx_0_tmp + b_1[g] * tempR_tmp) +
                           b_1[g + 2] * tempR_tmp_0) + b_1[g + 3] * tempR_tmp_1;
          }
        }

        i = (int32_T)nextBody->Index;
        qidx_idx_0 = obj->VelocityDoFMap[i - 1];
        qidx_idx_1 = obj->VelocityDoFMap[i + 4];
        if (nextBodyIsParent) {
          for (i = 0; i < 16; i++) {
            Tj[i] = joint->ChildToJointTransform[i];
          }
        } else {
          for (i = 0; i < 16; i++) {
            T1j[i] = joint->JointToParentTransform[i];
          }

          for (i = 0; i < 3; i++) {
            R[3 * i] = T1j[i];
            R[3 * i + 1] = T1j[i + 4];
            R[3 * i + 2] = T1j[i + 8];
          }

          for (i = 0; i <= 6; i += 2) {
            tmp_1 = _mm_loadu_pd(&R[i]);
            _mm_storeu_pd(&R_1[i], _mm_mul_pd(tmp_1, _mm_set1_pd(-1.0)));
          }

          for (i = 8; i < 9; i++) {
            R_1[i] = -R[i];
          }

          qidx_idx_0_tmp = T1j[13];
          tempR_tmp = T1j[12];
          tempR_tmp_0 = T1j[14];
          for (i = 0; i < 3; i++) {
            f = i << 2;
            Tj[f] = R[3 * i];
            Tj[f + 1] = R[3 * i + 1];
            Tj[f + 2] = R[3 * i + 2];
            Tj[i + 12] = (R_1[i + 3] * qidx_idx_0_tmp + R_1[i] * tempR_tmp) +
              R_1[i + 6] * tempR_tmp_0;
          }

          Tj[3] = 0.0;
          Tj[7] = 0.0;
          Tj[11] = 0.0;
          Tj[15] = 1.0;
        }

        for (i = 0; i < 4; i++) {
          qidx_idx_0_tmp = Tj[i + 4];
          tempR_tmp = Tj[i];
          tempR_tmp_0 = Tj[i + 8];
          tempR_tmp_1 = Tj[i + 12];
          for (loop_ub = 0; loop_ub < 4; loop_ub++) {
            g = loop_ub << 2;
            T1j[i + g] = ((T1[g + 1] * qidx_idx_0_tmp + T1[g] * tempR_tmp) +
                          T1[g + 2] * tempR_tmp_0) + T1[g + 3] * tempR_tmp_1;
          }
        }

        for (i = 0; i < 3; i++) {
          R[3 * i] = T1j[i];
          R[3 * i + 1] = T1j[i + 4];
          R[3 * i + 2] = T1j[i + 8];
        }

        for (i = 0; i <= 6; i += 2) {
          tmp_1 = _mm_loadu_pd(&R[i]);
          _mm_storeu_pd(&R_2[i], _mm_mul_pd(tmp_1, _mm_set1_pd(-1.0)));
        }

        for (i = 8; i < 9; i++) {
          R_2[i] = -R[i];
        }

        qidx_idx_0_tmp = T1j[13];
        tempR_tmp = T1j[12];
        tempR_tmp_0 = T1j[14];
        for (i = 0; i < 3; i++) {
          f = i << 2;
          Tj[f] = R[3 * i];
          Tj[f + 1] = R[3 * i + 1];
          Tj[f + 2] = R[3 * i + 2];
          Tj[i + 12] = (R_2[i + 3] * qidx_idx_0_tmp + R_2[i] * tempR_tmp) +
            R_2[i + 6] * tempR_tmp_0;
        }

        Tj[3] = 0.0;
        Tj[7] = 0.0;
        Tj[11] = 0.0;
        Tj[15] = 1.0;
        loop_ub = b->size[0] * b->size[1];
        b->size[0] = 6;
        b->size[1] = joint->MotionSubspace->size[1];
        ArmRev_emxEnsureCapacity_real_T(b, loop_ub);
        loop_ub = 6 * joint->MotionSubspace->size[1];
        for (i = 0; i < loop_ub; i++) {
          b->data[i] = joint->MotionSubspace->data[i];
        }

        if (qidx_idx_0 > qidx_idx_1) {
          g = 0;
          k = 0;
        } else {
          g = (int32_T)qidx_idx_0 - 1;
          k = (int32_T)qidx_idx_1;
        }

        tempR[0] = 0.0;
        tempR[3] = -Tj[14];
        tempR[6] = Tj[13];
        tempR[1] = Tj[14];
        tempR[4] = 0.0;
        tempR[7] = -Tj[12];
        tempR[2] = -Tj[13];
        tempR[5] = Tj[12];
        tempR[8] = 0.0;
        for (i = 0; i < 3; i++) {
          qidx_idx_0 = tempR[i + 3];
          qidx_idx_1 = tempR[i];
          qidx_idx_0_tmp = tempR[i + 6];
          for (loop_ub = 0; loop_ub < 3; loop_ub++) {
            f = loop_ub << 2;
            tmp_0[i + 3 * loop_ub] = (Tj[f + 1] * qidx_idx_0 + Tj[f] *
              qidx_idx_1) + Tj[f + 2] * qidx_idx_0_tmp;
            Tj_1[loop_ub + 6 * i] = Tj[(i << 2) + loop_ub];
            Tj_1[loop_ub + 6 * (i + 3)] = 0.0;
          }
        }

        for (i = 0; i < 3; i++) {
          Tj_1[6 * i + 3] = tmp_0[3 * i];
          f = i << 2;
          loop_ub = (i + 3) * 6;
          Tj_1[loop_ub + 3] = Tj[f];
          Tj_1[6 * i + 4] = tmp_0[3 * i + 1];
          Tj_1[loop_ub + 4] = Tj[f + 1];
          Tj_1[6 * i + 5] = tmp_0[3 * i + 2];
          Tj_1[loop_ub + 5] = Tj[f + 2];
        }

        ArmReverseKinematics_mtimes(Tj_1, b, tmp);
        k -= g;
        for (i = 0; i < k; i++) {
          for (loop_ub = 0; loop_ub <= 4; loop_ub += 2) {
            tmp_1 = _mm_loadu_pd(&tmp->data[6 * i + loop_ub]);
            _mm_storeu_pd(&Jac->data[loop_ub + 6 * (g + i)], _mm_mul_pd(tmp_1,
              _mm_set1_pd(jointSign)));
          }
        }
      }

      if (nextBodyIsParent) {
        for (i = 0; i < 4; i++) {
          qidx_idx_0 = Tc2p[i + 4];
          qidx_idx_1 = Tc2p[i];
          qidx_idx_0_tmp = Tc2p[i + 8];
          tempR_tmp = Tc2p[i + 12];
          for (loop_ub = 0; loop_ub < 4; loop_ub++) {
            g = loop_ub << 2;
            Tj[i + g] = ((T1[g + 1] * qidx_idx_0 + T1[g] * qidx_idx_1) + T1[g +
                         2] * qidx_idx_0_tmp) + T1[g + 3] * tempR_tmp;
          }
        }

        memcpy(&T1[0], &Tj[0], sizeof(real_T) << 4U);
      } else {
        for (i = 0; i < 3; i++) {
          R[3 * i] = Tc2p[i];
          R[3 * i + 1] = Tc2p[i + 4];
          R[3 * i + 2] = Tc2p[i + 8];
        }

        for (i = 0; i <= 6; i += 2) {
          tmp_1 = _mm_loadu_pd(&R[i]);
          _mm_storeu_pd(&R_0[i], _mm_mul_pd(tmp_1, _mm_set1_pd(-1.0)));
        }

        for (i = 8; i < 9; i++) {
          R_0[i] = -R[i];
        }

        qidx_idx_0 = Tc2p[13];
        qidx_idx_1 = Tc2p[12];
        qidx_idx_0_tmp = Tc2p[14];
        for (i = 0; i < 3; i++) {
          jointSign = i << 2;
          Tc2p[jointSign] = R[3 * i];
          Tc2p[jointSign + 1] = R[3 * i + 1];
          Tc2p[jointSign + 2] = R[3 * i + 2];
          Tc2p[i + 12] = (R_0[i + 3] * qidx_idx_0 + R_0[i] * qidx_idx_1) + R_0[i
            + 6] * qidx_idx_0_tmp;
        }

        Tc2p[3] = 0.0;
        Tc2p[7] = 0.0;
        Tc2p[11] = 0.0;
        Tc2p[15] = 1.0;
        for (i = 0; i < 4; i++) {
          qidx_idx_0 = Tc2p[i + 4];
          qidx_idx_1 = Tc2p[i];
          qidx_idx_0_tmp = Tc2p[i + 8];
          tempR_tmp = Tc2p[i + 12];
          for (loop_ub = 0; loop_ub < 4; loop_ub++) {
            jointSign = loop_ub << 2;
            Tj[i + jointSign] = ((T1[jointSign + 1] * qidx_idx_0 + T1[jointSign]
                                  * qidx_idx_1) + T1[jointSign + 2] *
                                 qidx_idx_0_tmp) + T1[jointSign + 3] * tempR_tmp;
          }
        }

        memcpy(&T1[0], &Tj[0], sizeof(real_T) << 4U);
      }
    }

    ArmReverseKinema_emxFree_char_T(&a);
    ArmReverseKinema_emxFree_real_T(&b);
    ArmReverseKinema_emxFree_real_T(&kinematicPathIndices);
    for (i = 0; i < 3; i++) {
      Jac_0 = i << 2;
      qidx_idx_0 = T1[Jac_0];
      Tj_1[6 * i] = qidx_idx_0;
      c = (i + 3) * 6;
      Tj_1[c] = 0.0;
      Tj_1[6 * i + 3] = 0.0;
      Tj_1[c + 3] = qidx_idx_0;
      qidx_idx_0 = T1[Jac_0 + 1];
      Tj_1[6 * i + 1] = qidx_idx_0;
      Tj_1[c + 1] = 0.0;
      Tj_1[6 * i + 4] = 0.0;
      Tj_1[c + 4] = qidx_idx_0;
      qidx_idx_0 = T1[Jac_0 + 2];
      Tj_1[6 * i + 2] = qidx_idx_0;
      Tj_1[c + 2] = 0.0;
      Tj_1[6 * i + 5] = 0.0;
      Tj_1[c + 5] = qidx_idx_0;
    }

    ArmReverseKinema_emxInit_real_T(&Jac_1, 2);
    loop_ub = Jac_1->size[0] * Jac_1->size[1];
    Jac_1->size[0] = 6;
    Jac_1->size[1] = Jac->size[1];
    ArmRev_emxEnsureCapacity_real_T(Jac_1, loop_ub);
    loop_ub = Jac->size[0] * Jac->size[1] - 1;
    if (loop_ub >= 0) {
      memcpy(&Jac_1->data[0], &Jac->data[0], (uint32_T)(loop_ub + 1) * sizeof
             (real_T));
    }

    ArmReverseKinematics_mtimes(Tj_1, Jac_1, Jac);
    ArmReverseKinema_emxFree_real_T(&Jac_1);
    T_size[0] = 4;
    T_size[1] = 4;
    memcpy(&T_data[0], &T1[0], sizeof(real_T) << 4U);
  } else {
    T_size[0] = 0;
    T_size[1] = 0;
    Jac->size[0] = 6;
    Jac->size[1] = 0;
  }

  ArmReverseKinema_emxFree_real_T(&tmp);
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T a;
  real_T b;
  real_T y;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = sqrt(a * a + 1.0) * b;
  } else if (a > b) {
    b /= a;
    y = sqrt(b * b + 1.0) * a;
  } else if (rtIsNaN(b)) {
    y = (rtNaN);
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

static creal_T ArmReverseKinematics_sqrt(const creal_T x)
{
  creal_T b_x;
  real_T absxr;
  real_T xr;
  xr = x.re;
  if (x.im == 0.0) {
    if (x.re < 0.0) {
      absxr = 0.0;
      xr = sqrt(-x.re);
    } else {
      absxr = sqrt(x.re);
      xr = 0.0;
    }
  } else if (x.re == 0.0) {
    if (x.im < 0.0) {
      absxr = sqrt(-x.im / 2.0);
      xr = -absxr;
    } else {
      absxr = sqrt(x.im / 2.0);
      xr = absxr;
    }
  } else if (rtIsNaN(x.re)) {
    absxr = (rtNaN);
  } else if (rtIsNaN(x.im)) {
    absxr = (rtNaN);
    xr = (rtNaN);
  } else if (rtIsInf(x.im)) {
    absxr = fabs(x.im);
    xr = x.im;
  } else if (rtIsInf(x.re)) {
    if (x.re < 0.0) {
      absxr = 0.0;
      xr = x.im * -x.re;
    } else {
      absxr = x.re;
      xr = 0.0;
    }
  } else {
    absxr = fabs(x.re);
    xr = fabs(x.im);
    if ((absxr > 4.4942328371557893E+307) || (xr > 4.4942328371557893E+307)) {
      absxr *= 0.5;
      xr = rt_hypotd_snf(absxr, xr * 0.5);
      if (xr > absxr) {
        absxr = sqrt(absxr / xr + 1.0) * sqrt(xr);
      } else {
        absxr = sqrt(xr) * 1.4142135623730951;
      }
    } else {
      absxr = sqrt((rt_hypotd_snf(absxr, xr) + absxr) * 0.5);
    }

    if (x.re > 0.0) {
      xr = x.im / absxr * 0.5;
    } else {
      if (x.im < 0.0) {
        xr = -absxr;
      } else {
        xr = absxr;
      }

      absxr = x.im / xr * 0.5;
    }
  }

  b_x.re = absxr;
  b_x.im = xr;
  return b_x;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp, tmp_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

static real_T ArmReverseKinematics_xnrm2(int32_T n, const real_T x[9], int32_T
  ix0)
{
  real_T scale;
  real_T y;
  int32_T k;
  int32_T kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = ix0 + n;
  for (k = ix0; k < kend; k++) {
    real_T absxk;
    absxk = fabs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

static real_T ArmReverseKinematics_xdotc(int32_T n, const real_T x[9], int32_T
  ix0, const real_T y[9], int32_T iy0)
{
  real_T d;
  int32_T b;
  int32_T k;
  d = 0.0;
  b = (uint8_T)n;
  for (k = 0; k < b; k++) {
    d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
  }

  return d;
}

static void ArmReverseKinematics_xaxpy(int32_T n, real_T a, int32_T ix0, const
  real_T y[9], int32_T iy0, real_T b_y[9])
{
  int32_T k;
  memcpy(&b_y[0], &y[0], 9U * sizeof(real_T));
  if (!(a == 0.0)) {
    for (k = 0; k < n; k++) {
      int32_T b_y_tmp;
      b_y_tmp = (iy0 + k) - 1;
      b_y[b_y_tmp] += b_y[(ix0 + k) - 1] * a;
    }
  }
}

static real_T ArmReverseKinematics_xnrm2_e(const real_T x[3], int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T k;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = ix0; k <= ix0 + 1; k++) {
    real_T absxk;
    absxk = fabs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

static void ArmReverseKinematics_xaxpy_evf(int32_T n, real_T a, const real_T x[9],
  int32_T ix0, real_T y[3], int32_T iy0)
{
  int32_T k;
  if (!(a == 0.0)) {
    int32_T scalarLB;
    int32_T tmp_1;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (k = 0; k <= vectorUB; k += 2) {
      __m128d tmp;
      __m128d tmp_0;
      tmp = _mm_loadu_pd(&x[(ix0 + k) - 1]);
      tmp_1 = (iy0 + k) - 1;
      tmp_0 = _mm_loadu_pd(&y[tmp_1]);
      _mm_storeu_pd(&y[tmp_1], _mm_add_pd(_mm_mul_pd(tmp, _mm_set1_pd(a)), tmp_0));
    }

    for (k = scalarLB; k < n; k++) {
      tmp_1 = (iy0 + k) - 1;
      y[tmp_1] += x[(ix0 + k) - 1] * a;
    }
  }
}

static void ArmReverseKinematics_xaxpy_ev(int32_T n, real_T a, const real_T x[3],
  int32_T ix0, const real_T y[9], int32_T iy0, real_T b_y[9])
{
  int32_T k;
  memcpy(&b_y[0], &y[0], 9U * sizeof(real_T));
  if (!(a == 0.0)) {
    int32_T b_y_tmp;
    int32_T scalarLB;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (k = 0; k <= vectorUB; k += 2) {
      __m128d tmp;
      __m128d tmp_0;
      tmp = _mm_loadu_pd(&x[(ix0 + k) - 1]);
      b_y_tmp = (iy0 + k) - 1;
      tmp_0 = _mm_loadu_pd(&b_y[b_y_tmp]);
      _mm_storeu_pd(&b_y[b_y_tmp], _mm_add_pd(_mm_mul_pd(tmp, _mm_set1_pd(a)),
        tmp_0));
    }

    for (k = scalarLB; k < n; k++) {
      b_y_tmp = (iy0 + k) - 1;
      b_y[b_y_tmp] += x[(ix0 + k) - 1] * a;
    }
  }
}

static void ArmReverseKinematics_xswap(const real_T x[9], int32_T ix0, int32_T
  iy0, real_T b_x[9])
{
  real_T temp;
  memcpy(&b_x[0], &x[0], 9U * sizeof(real_T));
  temp = b_x[ix0 - 1];
  b_x[ix0 - 1] = b_x[iy0 - 1];
  b_x[iy0 - 1] = temp;
  temp = b_x[ix0];
  b_x[ix0] = b_x[iy0];
  b_x[iy0] = temp;
  temp = b_x[ix0 + 1];
  b_x[ix0 + 1] = b_x[iy0 + 1];
  b_x[iy0 + 1] = temp;
}

static void ArmReverseKinematics_xrotg(real_T a, real_T b, real_T *b_a, real_T
  *b_b, real_T *c, real_T *s)
{
  real_T absa;
  real_T absb;
  real_T roe;
  real_T scale;
  roe = b;
  absa = fabs(a);
  absb = fabs(b);
  if (absa > absb) {
    roe = a;
  }

  scale = absa + absb;
  if (scale == 0.0) {
    *s = 0.0;
    *c = 1.0;
    *b_a = 0.0;
    *b_b = 0.0;
  } else {
    real_T ads;
    real_T bds;
    ads = absa / scale;
    bds = absb / scale;
    *b_a = sqrt(ads * ads + bds * bds) * scale;
    if (roe < 0.0) {
      *b_a = -*b_a;
    }

    *c = a / *b_a;
    *s = b / *b_a;
    if (absa > absb) {
      *b_b = *s;
    } else if (*c != 0.0) {
      *b_b = 1.0 / *c;
    } else {
      *b_b = 1.0;
    }
  }
}

static void ArmReverseKinematics_xrot(const real_T x[9], int32_T ix0, int32_T
  iy0, real_T c, real_T s, real_T b_x[9])
{
  real_T temp;
  real_T temp_tmp;
  memcpy(&b_x[0], &x[0], 9U * sizeof(real_T));
  temp = b_x[iy0 - 1];
  temp_tmp = b_x[ix0 - 1];
  b_x[iy0 - 1] = temp * c - temp_tmp * s;
  b_x[ix0 - 1] = temp_tmp * c + temp * s;
  temp = b_x[ix0] * c + b_x[iy0] * s;
  b_x[iy0] = b_x[iy0] * c - b_x[ix0] * s;
  b_x[ix0] = temp;
  temp = b_x[iy0 + 1];
  temp_tmp = b_x[ix0 + 1];
  b_x[iy0 + 1] = temp * c - temp_tmp * s;
  b_x[ix0 + 1] = temp_tmp * c + temp * s;
}

static void ArmReverseKinematics_svd(const real_T A[9], real_T U[9], real_T s[3],
  real_T V[9])
{
  __m128d tmp;
  real_T A_0[9];
  real_T A_1[9];
  real_T e[3];
  real_T s_0[3];
  real_T work[3];
  real_T emm1;
  real_T nrm;
  real_T rt;
  real_T shift;
  real_T smm1;
  real_T sqds;
  real_T ztest;
  real_T ztest0;
  real_T ztest0_tmp;
  int32_T b;
  int32_T colqp1;
  int32_T m;
  int32_T qjj;
  int32_T qp1;
  int32_T qq;
  int32_T qq_tmp;
  int32_T scalarLB;
  int32_T vectorUB;
  boolean_T apply_transform;
  boolean_T exitg1;
  s_0[0] = 0.0;
  e[0] = 0.0;
  work[0] = 0.0;
  s_0[1] = 0.0;
  e[1] = 0.0;
  work[1] = 0.0;
  s_0[2] = 0.0;
  e[2] = 0.0;
  work[2] = 0.0;
  for (m = 0; m < 9; m++) {
    A_0[m] = A[m];
    U[m] = 0.0;
    V[m] = 0.0;
  }

  for (m = 0; m < 2; m++) {
    qp1 = m + 2;
    qq_tmp = m * 3 + m;
    qq = qq_tmp + 1;
    apply_transform = false;
    nrm = ArmReverseKinematics_xnrm2(3 - m, A_0, qq_tmp + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (A_0[qq_tmp] < 0.0) {
        ztest = -nrm;
        s_0[m] = -nrm;
      } else {
        ztest = nrm;
        s_0[m] = nrm;
      }

      if (fabs(ztest) >= 1.0020841800044864E-292) {
        nrm = 1.0 / ztest;
        b = qq_tmp - m;
        scalarLB = (((((b - qq_tmp) + 3) / 2) << 1) + qq_tmp) + 1;
        vectorUB = scalarLB - 2;
        for (qjj = qq; qjj <= vectorUB; qjj += 2) {
          tmp = _mm_loadu_pd(&A_0[qjj - 1]);
          _mm_storeu_pd(&A_0[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
        }

        for (qjj = scalarLB; qjj <= b + 3; qjj++) {
          A_0[qjj - 1] *= nrm;
        }
      } else {
        b = qq_tmp - m;
        scalarLB = (((((b - qq_tmp) + 3) / 2) << 1) + qq_tmp) + 1;
        vectorUB = scalarLB - 2;
        for (qjj = qq; qjj <= vectorUB; qjj += 2) {
          tmp = _mm_loadu_pd(&A_0[qjj - 1]);
          _mm_storeu_pd(&A_0[qjj - 1], _mm_div_pd(tmp, _mm_set1_pd(s_0[m])));
        }

        for (qjj = scalarLB; qjj <= b + 3; qjj++) {
          A_0[qjj - 1] /= s_0[m];
        }
      }

      A_0[qq_tmp]++;
      s_0[m] = -s_0[m];
    } else {
      s_0[m] = 0.0;
    }

    for (b = qp1; b < 4; b++) {
      qjj = ((b - 1) * 3 + m) + 1;
      if (apply_transform) {
        memcpy(&A_1[0], &A_0[0], 9U * sizeof(real_T));
        ArmReverseKinematics_xaxpy(3 - m, -(ArmReverseKinematics_xdotc(3 - m,
          A_0, qq_tmp + 1, A_0, qjj) / A_0[qq_tmp]), qq_tmp + 1, A_1, qjj, A_0);
      }

      e[b - 1] = A_0[qjj - 1];
    }

    memcpy(&U[(m * 3 + (m + 1)) + -1], &A_0[(m * 3 + (m + 1)) + -1], (uint32_T)(
            -(m + 1) + 4) * sizeof(real_T));
    if (m + 1 <= 1) {
      nrm = ArmReverseKinematics_xnrm2_e(e, 2);
      if (nrm == 0.0) {
        e[0] = 0.0;
      } else {
        if (e[1] < 0.0) {
          rt = -nrm;
          e[0] = -nrm;
        } else {
          rt = nrm;
          e[0] = nrm;
        }

        if (fabs(rt) >= 1.0020841800044864E-292) {
          nrm = 1.0 / rt;
          scalarLB = ((((2 - m) / 2) << 1) + m) + 2;
          vectorUB = scalarLB - 2;
          for (qjj = qp1; qjj <= vectorUB; qjj += 2) {
            tmp = _mm_loadu_pd(&e[qjj - 1]);
            _mm_storeu_pd(&e[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
          }

          for (qjj = scalarLB; qjj < 4; qjj++) {
            e[qjj - 1] *= nrm;
          }
        } else {
          scalarLB = ((((2 - m) / 2) << 1) + m) + 2;
          vectorUB = scalarLB - 2;
          for (qjj = qp1; qjj <= vectorUB; qjj += 2) {
            tmp = _mm_loadu_pd(&e[qjj - 1]);
            _mm_storeu_pd(&e[qjj - 1], _mm_div_pd(tmp, _mm_set1_pd(rt)));
          }

          for (qjj = scalarLB; qjj < 4; qjj++) {
            e[qjj - 1] /= rt;
          }
        }

        e[1]++;
        e[0] = -e[0];
        for (qq = qp1; qq < 4; qq++) {
          work[qq - 1] = 0.0;
        }

        for (qq = qp1; qq < 4; qq++) {
          ArmReverseKinematics_xaxpy_evf(2, e[qq - 1], A_0, 3 * (qq - 1) + 2,
            work, 2);
        }

        for (qq = qp1; qq < 4; qq++) {
          memcpy(&A_1[0], &A_0[0], 9U * sizeof(real_T));
          ArmReverseKinematics_xaxpy_ev(2, -e[qq - 1] / e[1], work, 2, A_1, (qq
            - 1) * 3 + 2, A_0);
        }
      }

      for (colqp1 = qp1; colqp1 < 4; colqp1++) {
        V[colqp1 - 1] = e[colqp1 - 1];
      }
    }
  }

  m = 2;
  s_0[2] = A_0[8];
  e[1] = A_0[7];
  e[2] = 0.0;
  U[6] = 0.0;
  U[7] = 0.0;
  U[8] = 1.0;
  for (colqp1 = 1; colqp1 >= 0; colqp1--) {
    qq = 3 * colqp1 + colqp1;
    if (s_0[colqp1] != 0.0) {
      for (b = colqp1 + 2; b < 4; b++) {
        qjj = ((b - 1) * 3 + colqp1) + 1;
        memcpy(&A_0[0], &U[0], 9U * sizeof(real_T));
        ArmReverseKinematics_xaxpy(3 - colqp1, -(ArmReverseKinematics_xdotc(3 -
          colqp1, U, qq + 1, U, qjj) / U[qq]), qq + 1, A_0, qjj, U);
      }

      for (qp1 = colqp1 + 1; qp1 < 4; qp1++) {
        qq_tmp = (3 * colqp1 + qp1) - 1;
        U[qq_tmp] = -U[qq_tmp];
      }

      U[qq]++;
      if (colqp1 - 1 >= 0) {
        U[3 * colqp1] = 0.0;
      }
    } else {
      U[3 * colqp1] = 0.0;
      U[3 * colqp1 + 1] = 0.0;
      U[3 * colqp1 + 2] = 0.0;
      U[qq] = 1.0;
    }
  }

  for (colqp1 = 2; colqp1 >= 0; colqp1--) {
    if ((colqp1 + 1 <= 1) && (e[0] != 0.0)) {
      memcpy(&A_0[0], &V[0], 9U * sizeof(real_T));
      ArmReverseKinematics_xaxpy(2, -(ArmReverseKinematics_xdotc(2, V, 2, V, 5) /
        V[1]), 2, A_0, 5, V);
      memcpy(&A_0[0], &V[0], 9U * sizeof(real_T));
      ArmReverseKinematics_xaxpy(2, -(ArmReverseKinematics_xdotc(2, V, 2, V, 8) /
        V[1]), 2, A_0, 8, V);
    }

    V[3 * colqp1] = 0.0;
    V[3 * colqp1 + 1] = 0.0;
    V[3 * colqp1 + 2] = 0.0;
    V[colqp1 + 3 * colqp1] = 1.0;
  }

  for (qp1 = 0; qp1 < 3; qp1++) {
    ztest = s_0[qp1];
    if (ztest != 0.0) {
      rt = fabs(ztest);
      nrm = ztest / rt;
      s_0[qp1] = rt;
      if (qp1 + 1 < 3) {
        e[qp1] /= nrm;
      }

      qq = 3 * qp1;
      scalarLB = qq + 3;
      vectorUB = qq + 1;
      for (qjj = qq + 1; qjj <= vectorUB; qjj += 2) {
        tmp = _mm_loadu_pd(&U[qjj - 1]);
        _mm_storeu_pd(&U[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
      }

      for (qjj = scalarLB; qjj <= qq + 3; qjj++) {
        U[qjj - 1] *= nrm;
      }
    }

    if (qp1 + 1 < 3) {
      emm1 = e[qp1];
      if (emm1 != 0.0) {
        rt = fabs(emm1);
        nrm = rt / emm1;
        e[qp1] = rt;
        s_0[qp1 + 1] *= nrm;
        colqp1 = (qp1 + 1) * 3;
        scalarLB = colqp1 + 3;
        vectorUB = colqp1 + 1;
        for (qjj = colqp1 + 1; qjj <= vectorUB; qjj += 2) {
          tmp = _mm_loadu_pd(&V[qjj - 1]);
          _mm_storeu_pd(&V[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(nrm)));
        }

        for (qjj = scalarLB; qjj <= colqp1 + 3; qjj++) {
          V[qjj - 1] *= nrm;
        }
      }
    }
  }

  rt = 0.0;
  nrm = fmax(fmax(fmax(0.0, fmax(fabs(s_0[0]), fabs(e[0]))), fmax(fabs(s_0[1]),
    fabs(e[1]))), fmax(fabs(s_0[2]), fabs(e[2])));
  while ((m + 1 > 0) && (!(rt >= 75.0))) {
    colqp1 = m;
    qp1 = m;
    exitg1 = false;
    while ((!exitg1) && (qp1 > -1)) {
      colqp1 = qp1;
      if (qp1 == 0) {
        exitg1 = true;
      } else {
        ztest0 = fabs(e[qp1 - 1]);
        if ((ztest0 <= (fabs(s_0[qp1 - 1]) + fabs(s_0[qp1])) *
             2.2204460492503131E-16) || (ztest0 <= 1.0020841800044864E-292) ||
            ((rt > 20.0) && (ztest0 <= 2.2204460492503131E-16 * nrm))) {
          e[qp1 - 1] = 0.0;
          exitg1 = true;
        } else {
          qp1--;
        }
      }
    }

    if (colqp1 == m) {
      ztest0 = 4.0;
    } else {
      qp1 = m + 1;
      qq = m + 1;
      exitg1 = false;
      while ((!exitg1) && (qq >= colqp1)) {
        qp1 = qq;
        if (qq == colqp1) {
          exitg1 = true;
        } else {
          ztest0 = 0.0;
          if (qq < m + 1) {
            ztest0 = fabs(e[qq - 1]);
          }

          if (qq > colqp1 + 1) {
            ztest0 += fabs(e[qq - 2]);
          }

          ztest = fabs(s_0[qq - 1]);
          if ((ztest <= 2.2204460492503131E-16 * ztest0) || (ztest <=
               1.0020841800044864E-292)) {
            s_0[qq - 1] = 0.0;
            exitg1 = true;
          } else {
            qq--;
          }
        }
      }

      if (qp1 == colqp1) {
        ztest0 = 3.0;
      } else if (m + 1 == qp1) {
        ztest0 = 1.0;
      } else {
        ztest0 = 2.0;
        colqp1 = qp1;
      }
    }

    switch ((int32_T)ztest0) {
     case 1:
      ztest0 = e[m - 1];
      e[m - 1] = 0.0;
      for (qq = m; qq >= colqp1 + 1; qq--) {
        ArmReverseKinematics_xrotg(s_0[qq - 1], ztest0, &s_0[qq - 1], &ztest0,
          &ztest, &sqds);
        if (qq > colqp1 + 1) {
          emm1 = e[0];
          ztest0 = -sqds * emm1;
          e[0] = emm1 * ztest;
        }

        memcpy(&A_0[0], &V[0], 9U * sizeof(real_T));
        ArmReverseKinematics_xrot(A_0, (qq - 1) * 3 + 1, 3 * m + 1, ztest, sqds,
          V);
      }
      break;

     case 2:
      ztest0 = e[colqp1 - 1];
      e[colqp1 - 1] = 0.0;
      for (qp1 = colqp1 + 1; qp1 <= m + 1; qp1++) {
        ArmReverseKinematics_xrotg(s_0[qp1 - 1], ztest0, &s_0[qp1 - 1], &ztest,
          &sqds, &smm1);
        emm1 = e[qp1 - 1];
        ztest0 = -smm1 * emm1;
        e[qp1 - 1] = emm1 * sqds;
        memcpy(&A_0[0], &U[0], 9U * sizeof(real_T));
        ArmReverseKinematics_xrot(A_0, (qp1 - 1) * 3 + 1, (colqp1 - 1) * 3 + 1,
          sqds, smm1, U);
      }
      break;

     case 3:
      smm1 = s_0[m - 1];
      sqds = e[m - 1];
      ztest = fmax(fmax(fmax(fmax(fabs(s_0[m]), fabs(smm1)), fabs(sqds)), fabs
                        (s_0[colqp1])), fabs(e[colqp1]));
      ztest0 = s_0[m] / ztest;
      smm1 /= ztest;
      emm1 = sqds / ztest;
      sqds = s_0[colqp1] / ztest;
      smm1 = ((smm1 + ztest0) * (smm1 - ztest0) + emm1 * emm1) / 2.0;
      emm1 *= ztest0;
      emm1 *= emm1;
      if ((smm1 != 0.0) || (emm1 != 0.0)) {
        shift = sqrt(smm1 * smm1 + emm1);
        if (smm1 < 0.0) {
          shift = -shift;
        }

        shift = emm1 / (smm1 + shift);
      } else {
        shift = 0.0;
      }

      ztest0 = (sqds + ztest0) * (sqds - ztest0) + shift;
      ztest = e[colqp1] / ztest * sqds;
      for (qq = colqp1 + 1; qq <= m; qq++) {
        ArmReverseKinematics_xrotg(ztest0, ztest, &sqds, &smm1, &emm1, &shift);
        if (qq > colqp1 + 1) {
          e[0] = sqds;
        }

        ztest0_tmp = e[qq - 1];
        ztest0 = s_0[qq - 1];
        e[qq - 1] = ztest0_tmp * emm1 - ztest0 * shift;
        ztest = shift * s_0[qq];
        s_0[qq] *= emm1;
        qq_tmp = (qq - 1) * 3 + 1;
        b = 3 * qq + 1;
        memcpy(&A_0[0], &V[0], 9U * sizeof(real_T));
        ArmReverseKinematics_xrot(A_0, qq_tmp, b, emm1, shift, V);
        ArmReverseKinematics_xrotg(ztest0 * emm1 + ztest0_tmp * shift, ztest,
          &s_0[qq - 1], &sqds, &smm1, &emm1);
        ztest0_tmp = e[qq - 1];
        ztest0 = ztest0_tmp * smm1 + emm1 * s_0[qq];
        s_0[qq] = ztest0_tmp * -emm1 + smm1 * s_0[qq];
        ztest = emm1 * e[qq];
        e[qq] *= smm1;
        memcpy(&A_0[0], &U[0], 9U * sizeof(real_T));
        ArmReverseKinematics_xrot(A_0, qq_tmp, b, smm1, emm1, U);
      }

      e[m - 1] = ztest0;
      rt++;
      break;

     default:
      if (s_0[colqp1] < 0.0) {
        s_0[colqp1] = -s_0[colqp1];
        qq = 3 * colqp1;
        scalarLB = qq + 3;
        vectorUB = qq + 1;
        for (qjj = qq + 1; qjj <= vectorUB; qjj += 2) {
          tmp = _mm_loadu_pd(&V[qjj - 1]);
          _mm_storeu_pd(&V[qjj - 1], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
        }

        for (qjj = scalarLB; qjj <= qq + 3; qjj++) {
          V[qjj - 1] = -V[qjj - 1];
        }
      }

      qp1 = colqp1 + 1;
      while ((colqp1 + 1 < 3) && (s_0[colqp1] < s_0[qp1])) {
        rt = s_0[colqp1];
        s_0[colqp1] = s_0[qp1];
        s_0[qp1] = rt;
        qq_tmp = 3 * colqp1 + 1;
        b = (colqp1 + 1) * 3 + 1;
        memcpy(&A_0[0], &V[0], 9U * sizeof(real_T));
        ArmReverseKinematics_xswap(A_0, qq_tmp, b, V);
        memcpy(&A_0[0], &U[0], 9U * sizeof(real_T));
        ArmReverseKinematics_xswap(A_0, qq_tmp, b, U);
        colqp1 = qp1;
        qp1++;
      }

      rt = 0.0;
      m--;
      break;
    }
  }

  s[0] = s_0[0];
  s[1] = s_0[1];
  s[2] = s_0[2];
}

static void ArmReverseK_IKHelpers_poseError(const real_T Td[16], const real_T
  T_data[], const int32_T T_size[2], real_T errorvec[6])
{
  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_2;
  creal_T tmp;
  creal_T u;
  creal_T u_0;
  creal_T v_0;
  real_T V[9];
  real_T b_I[9];
  real_T b_U[9];
  real_T y[9];
  real_T v[3];
  real_T v_1[3];
  real_T vspecial_data[3];
  real_T T;
  real_T q;
  real_T t4;
  int32_T b_i;
  int32_T iy;
  int32_T trueCount;
  int8_T y_tmp[3];
  int8_T y_tmp_0;
  boolean_T exitg1;
  boolean_T xneg;
  y_tmp[0] = 1;
  y_tmp[1] = 2;
  y_tmp[2] = 3;
  for (iy = 0; iy < 3; iy++) {
    y_tmp_0 = y_tmp[iy];
    t4 = T_data[y_tmp_0 - 1];
    q = T_data[(y_tmp_0 + T_size[0]) - 1];
    T = T_data[((T_size[0] << 1) + y_tmp_0) - 1];
    for (trueCount = 0; trueCount <= 0; trueCount += 2) {
      tmp_0 = _mm_loadu_pd(&Td[trueCount + 4]);
      tmp_1 = _mm_loadu_pd(&Td[trueCount]);
      tmp_2 = _mm_loadu_pd(&Td[trueCount + 8]);
      _mm_storeu_pd(&y[trueCount + 3 * iy], _mm_add_pd(_mm_add_pd(_mm_mul_pd
        (_mm_set1_pd(q), tmp_0), _mm_mul_pd(_mm_set1_pd(t4), tmp_1)), _mm_mul_pd
        (_mm_set1_pd(T), tmp_2)));
    }

    for (trueCount = 2; trueCount < 3; trueCount++) {
      y[trueCount + 3 * iy] = (Td[trueCount + 4] * q + t4 * Td[trueCount]) +
        Td[trueCount + 8] * T;
    }
  }

  u.re = (((y[0] + y[4]) + y[8]) - 1.0) * 0.5;
  if (!(fabs(u.re) > 1.0)) {
    v_0.re = acos(u.re);
  } else {
    u_0.re = u.re + 1.0;
    u_0.im = 0.0;
    tmp.re = 1.0 - u.re;
    tmp.im = 0.0;
    v_0.re = 2.0 * rt_atan2d_snf((ArmReverseKinematics_sqrt(tmp)).re,
      (ArmReverseKinematics_sqrt(u_0)).re);
  }

  t4 = 2.0 * sin(v_0.re);
  v[0] = (y[5] - y[7]) / t4;
  v[1] = (y[6] - y[2]) / t4;
  v[2] = (y[1] - y[3]) / t4;
  if (rtIsNaN(v_0.re) || rtIsInf(v_0.re)) {
    t4 = (rtNaN);
  } else if (v_0.re == 0.0) {
    t4 = 0.0;
  } else {
    t4 = fmod(v_0.re, 3.1415926535897931);
    xneg = (t4 == 0.0);
    if (!xneg) {
      q = fabs(v_0.re / 3.1415926535897931);
      xneg = !(fabs(q - floor(q + 0.5)) > 2.2204460492503131E-16 * q);
    }

    if (xneg) {
      t4 = 0.0;
    } else if (v_0.re < 0.0) {
      t4 += 3.1415926535897931;
    }
  }

  xneg = true;
  iy = 0;
  exitg1 = false;
  while ((!exitg1) && (iy < 3)) {
    if (!(v[iy] == 0.0)) {
      xneg = false;
      exitg1 = true;
    } else {
      iy++;
    }
  }

  xneg = ((t4 == 0.0) || xneg);
  if (xneg) {
    for (iy = 0; iy < 3; iy++) {
      vspecial_data[iy] = 0.0;
    }

    trueCount = 0;
    for (b_i = 0; b_i < 1; b_i++) {
      memset(&b_I[0], 0, 9U * sizeof(real_T));
      b_I[0] = 1.0;
      b_I[4] = 1.0;
      b_I[8] = 1.0;
      for (iy = 0; iy <= 6; iy += 2) {
        tmp_0 = _mm_loadu_pd(&b_I[iy]);
        tmp_1 = _mm_loadu_pd(&y[iy]);
        _mm_storeu_pd(&b_I[iy], _mm_sub_pd(tmp_0, tmp_1));
      }

      for (iy = 8; iy < 9; iy++) {
        b_I[iy] -= y[iy];
      }

      xneg = true;
      for (iy = 0; iy < 9; iy++) {
        if (xneg) {
          t4 = b_I[iy];
          if ((!rtIsInf(t4)) && (!rtIsNaN(t4))) {
          } else {
            xneg = false;
          }
        } else {
          xneg = false;
        }
      }

      if (xneg) {
        ArmReverseKinematics_svd(b_I, b_U, v_1, V);
      } else {
        for (iy = 0; iy < 9; iy++) {
          V[iy] = (rtNaN);
        }
      }

      vspecial_data[0] = V[6];
      vspecial_data[1] = V[7];
      vspecial_data[2] = V[8];
      trueCount++;
    }

    if (trueCount - 1 >= 0) {
      v[0] = vspecial_data[0];
      v[1] = vspecial_data[1];
      v[2] = vspecial_data[2];
    }
  }

  v_1[0] = v[0];
  v_1[1] = v[1];
  v_1[2] = v[2];
  t4 = 1.0 / sqrt((v[0] * v[0] + v[1] * v[1]) + v[2] * v[2]);
  errorvec[0] = v_1[0] * t4 * v_0.re;
  errorvec[3] = Td[12] - T_data[T_size[0] * 3];
  errorvec[1] = v_1[1] * t4 * v_0.re;
  errorvec[4] = Td[13] - T_data[T_size[0] * 3 + 1];
  errorvec[2] = v_1[2] * t4 * v_0.re;
  errorvec[5] = Td[14] - T_data[T_size[0] * 3 + 2];
}

static void ArmReverseKinematics_mtimes_e(const real_T A[6], const
  emxArray_real_T_ArmReverseKin_T *B_1, emxArray_real_T_ArmReverseKin_T *C)
{
  real_T s;
  int32_T b_j;
  int32_T b_k;
  int32_T boffset;
  int32_T n;
  n = B_1->size[1] - 1;
  b_j = C->size[0] * C->size[1];
  C->size[0] = 1;
  C->size[1] = B_1->size[1];
  ArmRev_emxEnsureCapacity_real_T(C, b_j);
  for (b_j = 0; b_j <= n; b_j++) {
    boffset = b_j * 6 - 1;
    s = 0.0;
    for (b_k = 0; b_k < 6; b_k++) {
      s += B_1->data[(boffset + b_k) + 1] * A[b_k];
    }

    C->data[b_j] = s;
  }
}

static void ArmReverseKin_emxInit_boolean_T(emxArray_boolean_T_ArmReverse_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_boolean_T_ArmReverse_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_boolean_T_ArmReverse_T *)malloc(sizeof
    (emxArray_boolean_T_ArmReverse_T));
  emxArray = *pEmxArray;
  emxArray->data = (boolean_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * (uint32_T)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static real_T ArmReverseKinematics_norm_e(const real_T x[6])
{
  real_T scale;
  real_T y;
  int32_T b_k;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (b_k = 0; b_k < 6; b_k++) {
    real_T absxk;
    absxk = fabs(x[b_k]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

static void ArmReverseKinematics_minus_e(emxArray_real_T_ArmReverseKin_T *in1,
  const emxArray_real_T_ArmReverseKin_T *in2)
{
  emxArray_real_T_ArmReverseKin_T *in2_0;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  ArmReverseKinema_emxInit_real_T(&in2_0, 1);
  if (in1->size[0] == 1) {
    i = in2_0->size[0];
    in2_0->size[0] = in2->size[0];
    ArmRev_emxEnsureCapacity_real_T(in2_0, i);
  } else {
    i = in2_0->size[0];
    in2_0->size[0] = in1->size[0];
    ArmRev_emxEnsureCapacity_real_T(in2_0, i);
  }

  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in1->size[0] != 1);
  if (in1->size[0] == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = in1->size[0];
  }

  for (i = 0; i < loop_ub; i++) {
    in2_0->data[i] = in2->data[i * stride_0_0] - in1->data[i * stride_1_0];
  }

  i = in1->size[0];
  in1->size[0] = in2_0->size[0];
  ArmRev_emxEnsureCapacity_real_T(in1, i);
  loop_ub = in2_0->size[0];
  if (loop_ub - 1 >= 0) {
    memcpy(&in1->data[0], &in2_0->data[0], (uint32_T)loop_ub * sizeof(real_T));
  }

  ArmReverseKinema_emxFree_real_T(&in2_0);
}

static void Arm_emxEnsureCapacity_boolean_T(emxArray_boolean_T_ArmReverse_T
  *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = malloc((uint32_T)i * sizeof(boolean_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(boolean_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (boolean_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static real_T ArmReverseKinematics_toc(real_T tstart_tv_sec, real_T
  tstart_tv_nsec)
{
  coderTimespec b_timespec;
  if (!rtDW.method_not_empty) {
    rtDW.method_not_empty = true;
    coderInitTimeFunctions(&rtDW.freq);
  }

  coderTimeClockGettimeMonotonic(&b_timespec, rtDW.freq);
  return (b_timespec.tv_nsec - tstart_tv_nsec) / 1.0E+9 + (b_timespec.tv_sec -
    tstart_tv_sec);
}

static void ArmReverseKinematics_mldivide(const real_T A[16], const
  emxArray_real_T_ArmReverseKin_T *B_2, real_T Y_data[], int32_T *Y_size)
{
  real_T c_x[16];
  real_T smax;
  int32_T c;
  int32_T ijA;
  int32_T jA;
  int32_T jj;
  int32_T jp1j;
  int32_T kAcol;
  int8_T b_ipiv[4];
  memcpy(&c_x[0], &A[0], sizeof(real_T) << 4U);
  b_ipiv[0] = 1;
  b_ipiv[1] = 2;
  b_ipiv[2] = 3;
  for (kAcol = 0; kAcol < 3; kAcol++) {
    int32_T c_0;
    int32_T iy;
    c = kAcol * 5 + 2;
    jj = kAcol * 5;
    c_0 = 4 - kAcol;
    iy = 1;
    smax = fabs(c_x[jj]);
    for (jA = 2; jA <= c_0; jA++) {
      real_T s;
      s = fabs(c_x[(c + jA) - 3]);
      if (s > smax) {
        iy = jA;
        smax = s;
      }
    }

    if (c_x[(c + iy) - 3] != 0.0) {
      if (iy - 1 != 0) {
        iy += kAcol;
        b_ipiv[kAcol] = (int8_T)iy;
        smax = c_x[kAcol];
        c_x[kAcol] = c_x[iy - 1];
        c_x[iy - 1] = smax;
        smax = c_x[kAcol + 4];
        c_x[kAcol + 4] = c_x[iy + 3];
        c_x[iy + 3] = smax;
        smax = c_x[kAcol + 8];
        c_x[kAcol + 8] = c_x[iy + 7];
        c_x[iy + 7] = smax;
        smax = c_x[kAcol + 12];
        c_x[kAcol + 12] = c_x[iy + 11];
        c_x[iy + 11] = smax;
      }

      iy = c - kAcol;
      for (jA = c; jA <= iy + 2; jA++) {
        c_x[jA - 1] /= c_x[jj];
      }
    }

    jA = jj;
    jj += 4;
    iy = 3 - kAcol;
    for (jp1j = 0; jp1j < iy; jp1j++) {
      smax = c_x[(jp1j << 2) + jj];
      if (smax != 0.0) {
        int32_T d;
        c_0 = jA + 6;
        d = (jA - kAcol) + 8;
        for (ijA = c_0; ijA <= d; ijA++) {
          c_x[ijA - 1] += c_x[((c + ijA) - jA) - 7] * -smax;
        }
      }

      jA += 4;
    }
  }

  *Y_size = B_2->size[0];
  c = B_2->size[0];
  if (c - 1 >= 0) {
    memcpy(&Y_data[0], &B_2->data[0], (uint32_T)c * sizeof(real_T));
  }

  if (b_ipiv[0] != 1) {
    smax = Y_data[0];
    Y_data[0] = Y_data[b_ipiv[0] - 1];
    Y_data[b_ipiv[0] - 1] = smax;
  }

  if (b_ipiv[1] != 2) {
    smax = Y_data[1];
    Y_data[1] = Y_data[b_ipiv[1] - 1];
    Y_data[b_ipiv[1] - 1] = smax;
  }

  if (b_ipiv[2] != 3) {
    smax = Y_data[2];
    Y_data[2] = Y_data[b_ipiv[2] - 1];
    Y_data[b_ipiv[2] - 1] = smax;
  }

  for (c = 0; c < 4; c++) {
    kAcol = (c << 2) - 1;
    if (Y_data[c] != 0.0) {
      for (jA = c + 2; jA < 5; jA++) {
        Y_data[jA - 1] -= c_x[jA + kAcol] * Y_data[c];
      }
    }
  }

  for (jA = 3; jA >= 0; jA--) {
    kAcol = jA << 2;
    smax = Y_data[jA];
    if (smax != 0.0) {
      Y_data[jA] = smax / c_x[jA + kAcol];
      c = jA - 1;
      for (jj = 0; jj <= c; jj++) {
        Y_data[jj] -= c_x[jj + kAcol] * Y_data[jA];
      }
    }
  }
}

static void ArmReverseKi_binary_expand_op_1(real_T in1_data[], int32_T *in1_size,
  const emxArray_real_T_ArmReverseKin_T *in2, real_T in3, const real_T in4[16],
  const emxArray_real_T_ArmReverseKin_T *in5)
{
  real_T in2_0[16];
  int32_T aux_0_1;
  int32_T i;
  int32_T in2_tmp;
  int32_T stride_0_0;
  int32_T stride_0_1;
  stride_0_0 = (in2->size[0] != 1);
  stride_0_1 = (in2->size[1] != 1);
  aux_0_1 = 0;
  for (i = 0; i < 4; i++) {
    in2_tmp = i << 2;
    in2_0[in2_tmp] = -(in4[in2_tmp] * in3 + in2->data[in2->size[0] * aux_0_1]);
    in2_0[in2_tmp + 1] = -(in4[in2_tmp + 1] * in3 + in2->data[in2->size[0] *
      aux_0_1 + stride_0_0]);
    in2_0[in2_tmp + 2] = -(in4[in2_tmp + 2] * in3 + in2->data[(stride_0_0 << 1)
      + in2->size[0] * aux_0_1]);
    in2_0[in2_tmp + 3] = -(in4[in2_tmp + 3] * in3 + in2->data[3 * stride_0_0 +
      in2->size[0] * aux_0_1]);
    aux_0_1 += stride_0_1;
  }

  ArmReverseKinematics_mldivide(in2_0, in5, in1_data, in1_size);
}

static void ArmReverseKinematics_expand_max(const
  emxArray_real_T_ArmReverseKin_T *a, const real_T b[4], real_T c[4])
{
  c[0] = fmax(a->data[0], b[0]);
  c[1] = fmax(a->data[0], b[1]);
  c[2] = fmax(a->data[0], b[2]);
  c[3] = fmax(a->data[0], b[3]);
}

static void ArmReverseKinematics_expand_min(const
  emxArray_real_T_ArmReverseKin_T *a, const real_T b[4], real_T c[4])
{
  c[0] = fmin(a->data[0], b[0]);
  c[1] = fmin(a->data[0], b[1]);
  c[2] = fmin(a->data[0], b[2]);
  c[3] = fmin(a->data[0], b[3]);
}

static void ArmReverseKin_emxFree_boolean_T(emxArray_boolean_T_ArmReverse_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_boolean_T_ArmReverse_T *)NULL) {
    if (((*pEmxArray)->data != (boolean_T *)NULL) && (*pEmxArray)->canFreeData)
    {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_boolean_T_ArmReverse_T *)NULL;
  }
}

static void ErrorDampedLevenbergMarquardt_s(h_robotics_core_internal_Erro_T *obj,
  real_T xSol[4], c_robotics_core_internal_NLPS_T *exitFlag, real_T *en, real_T *
  iter)
{
  emxArray_boolean_T_ArmReverse_T *x_0;
  emxArray_real_T_ArmReverseKin_T *H0;
  emxArray_real_T_ArmReverseKin_T *J;
  emxArray_real_T_ArmReverseKin_T *J_0;
  emxArray_real_T_ArmReverseKin_T *b;
  emxArray_real_T_ArmReverseKin_T *ev;
  emxArray_real_T_ArmReverseKin_T *evprev;
  emxArray_real_T_ArmReverseKin_T *grad;
  emxArray_real_T_ArmReverseKin_T *tmp;
  emxArray_real_T_ArmReverseKin_T *y;
  f_robotics_manip_internal_IKE_T *args;
  x_robotics_manip_internal_Rig_T *treeInternal;
  real_T a[36];
  real_T weightMatrix[36];
  real_T T_data[16];
  real_T Td[16];
  real_T e[6];
  real_T step_data[4];
  real_T y_0[4];
  real_T bidx;
  real_T cc;
  real_T cost;
  real_T d;
  real_T scale;
  real_T t;
  int32_T aoffset;
  int32_T b_i;
  int32_T boffset;
  int32_T coffset;
  int32_T kend;
  int32_T m;
  boolean_T x[4];
  boolean_T flag;
  static const real_T tmp_0[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  __m128d tmp_1;
  __m128d tmp_2;
  emxArray_real_T_ArmReverseKin_T *J_1;
  emxArray_real_T_ArmReverseKin_T *J_2;
  emxArray_real_T_ArmReverseKin_T *J_3;
  real_T H0_0[16];
  real_T H0_1[16];
  real_T e_0[6];
  real_T xprev_idx_0;
  real_T xprev_idx_1;
  real_T xprev_idx_2;
  real_T xprev_idx_3;
  int32_T T_size[2];
  int32_T J_4;
  int32_T exitg1;
  int32_T loop_ub;
  boolean_T exitg2;
  boolean_T guard1;
  boolean_T guard2;
  ArmReverseKinema_emxInit_real_T(&evprev, 1);
  ArmReverseKinema_emxInit_real_T(&H0, 2);
  ArmReverseKinema_emxInit_real_T(&J, 2);
  ArmReverseKinema_emxInit_real_T(&ev, 1);
  ArmReverseKinema_emxInit_real_T(&J_0, 2);
  ArmReverseKinema_emxInit_real_T(&J_1, 2);
  ArmReverseKinema_emxInit_real_T(&J_2, 2);
  ArmReverseKinema_emxInit_real_T(&J_3, 2);
  xSol[0] = obj->SeedInternal[0];
  xSol[1] = obj->SeedInternal[1];
  xSol[2] = obj->SeedInternal[2];
  xSol[3] = obj->SeedInternal[3];
  ArmReverseKinematics_tic(&obj->TimeObjInternal.StartTime.tv_sec,
    &obj->TimeObjInternal.StartTime.tv_nsec);
  xprev_idx_0 = xSol[0];
  xprev_idx_1 = xSol[1];
  xprev_idx_2 = xSol[2];
  xprev_idx_3 = xSol[3];
  args = obj->ExtraArgs;
  treeInternal = args->Robot;
  for (kend = 0; kend < 16; kend++) {
    Td[kend] = args->Tform[kend];
  }

  for (kend = 0; kend < 36; kend++) {
    weightMatrix[kend] = args->WeightMatrix[kend];
  }

  bidx = args->BodyIndex;
  RigidBodyTree_efficientFKAndJac(treeInternal, xSol, bidx, T_data, T_size, J_0);
  ArmReverseK_IKHelpers_poseError(Td, T_data, T_size, e);
  J_4 = args->ErrTemp->size[0];
  args->ErrTemp->size[0] = 6;
  ArmRev_emxEnsureCapacity_real_T(args->ErrTemp, J_4);
  for (kend = 0; kend < 6; kend++) {
    args->ErrTemp->data[kend] = e[kend];
  }

  bidx = 0.0;
  for (kend = 0; kend < 6; kend++) {
    t = 0.0;
    for (J_4 = 0; J_4 < 6; J_4++) {
      t += weightMatrix[6 * kend + J_4] * (0.5 * e[J_4]);
    }

    bidx += t * e[kend];
  }

  args->CostTemp = bidx;
  for (kend = 0; kend < 6; kend++) {
    bidx = 0.0;
    for (J_4 = 0; J_4 < 6; J_4++) {
      bidx += weightMatrix[6 * kend + J_4] * e[J_4];
    }

    e_0[kend] = bidx;
  }

  J_4 = J_1->size[0] * J_1->size[1];
  J_1->size[0] = 6;
  J_1->size[1] = J_0->size[1];
  ArmRev_emxEnsureCapacity_real_T(J_1, J_4);
  loop_ub = 6 * J_0->size[1];
  m = (loop_ub / 2) << 1;
  coffset = m - 2;
  for (kend = 0; kend <= coffset; kend += 2) {
    tmp_2 = _mm_loadu_pd(&J_0->data[kend]);
    _mm_storeu_pd(&J_1->data[kend], _mm_mul_pd(tmp_2, _mm_set1_pd(-1.0)));
  }

  for (kend = m; kend < loop_ub; kend++) {
    J_1->data[kend] = -J_0->data[kend];
  }

  ArmReverseKinema_emxInit_real_T(&tmp, 2);
  ArmReverseKinematics_mtimes_e(e_0, J_1, tmp);
  J_4 = args->GradTemp->size[0];
  args->GradTemp->size[0] = tmp->size[1];
  ArmRev_emxEnsureCapacity_real_T(args->GradTemp, J_4);
  loop_ub = tmp->size[1];
  for (kend = 0; kend < loop_ub; kend++) {
    args->GradTemp->data[kend] = tmp->data[kend];
  }

  obj->ExtraArgs = args;
  args = obj->ExtraArgs;
  J_4 = evprev->size[0];
  evprev->size[0] = args->ErrTemp->size[0];
  ArmRev_emxEnsureCapacity_real_T(evprev, J_4);
  loop_ub = args->ErrTemp->size[0];
  for (kend = 0; kend < loop_ub; kend++) {
    evprev->data[kend] = args->ErrTemp->data[kend];
  }

  d = obj->MaxNumIterationInternal;
  b_i = 0;
  ArmReverseKinema_emxInit_real_T(&grad, 1);
  ArmReverseKinema_emxInit_real_T(&y, 2);
  ArmReverseKinema_emxInit_real_T(&b, 1);
  ArmReverseKin_emxInit_boolean_T(&x_0, 1);
  do {
    exitg1 = 0;
    if (b_i <= (int32_T)d - 1) {
      args = obj->ExtraArgs;
      treeInternal = args->Robot;
      for (kend = 0; kend < 16; kend++) {
        Td[kend] = args->Tform[kend];
      }

      for (kend = 0; kend < 36; kend++) {
        weightMatrix[kend] = args->WeightMatrix[kend];
      }

      bidx = args->BodyIndex;
      RigidBodyTree_efficientFKAndJac(treeInternal, xSol, bidx, T_data, T_size,
        J_0);
      J_4 = J->size[0] * J->size[1];
      J->size[0] = 6;
      J->size[1] = J_0->size[1];
      ArmRev_emxEnsureCapacity_real_T(J, J_4);
      loop_ub = 6 * J_0->size[1];
      m = (loop_ub / 2) << 1;
      coffset = m - 2;
      for (kend = 0; kend <= coffset; kend += 2) {
        tmp_2 = _mm_loadu_pd(&J_0->data[kend]);
        _mm_storeu_pd(&J->data[kend], _mm_mul_pd(tmp_2, _mm_set1_pd(-1.0)));
      }

      for (kend = m; kend < loop_ub; kend++) {
        J->data[kend] = -J_0->data[kend];
      }

      ArmReverseK_IKHelpers_poseError(Td, T_data, T_size, e);
      J_4 = args->ErrTemp->size[0];
      args->ErrTemp->size[0] = 6;
      ArmRev_emxEnsureCapacity_real_T(args->ErrTemp, J_4);
      for (kend = 0; kend < 6; kend++) {
        args->ErrTemp->data[kend] = e[kend];
      }

      bidx = 0.0;
      for (kend = 0; kend < 6; kend++) {
        t = 0.0;
        for (J_4 = 0; J_4 < 6; J_4++) {
          t += weightMatrix[6 * kend + J_4] * (0.5 * e[J_4]);
        }

        bidx += t * e[kend];
      }

      args->CostTemp = bidx;
      for (kend = 0; kend < 6; kend++) {
        bidx = 0.0;
        for (J_4 = 0; J_4 < 6; J_4++) {
          bidx += weightMatrix[6 * kend + J_4] * e[J_4];
        }

        e_0[kend] = bidx;
      }

      ArmReverseKinematics_mtimes_e(e_0, J, tmp);
      J_4 = args->GradTemp->size[0];
      args->GradTemp->size[0] = tmp->size[1];
      ArmRev_emxEnsureCapacity_real_T(args->GradTemp, J_4);
      loop_ub = tmp->size[1];
      for (kend = 0; kend < loop_ub; kend++) {
        args->GradTemp->data[kend] = tmp->data[kend];
      }

      cost = args->CostTemp;
      obj->ExtraArgs = args;
      args = obj->ExtraArgs;
      J_4 = grad->size[0];
      grad->size[0] = args->GradTemp->size[0];
      ArmRev_emxEnsureCapacity_real_T(grad, J_4);
      loop_ub = args->GradTemp->size[0];
      for (kend = 0; kend < loop_ub; kend++) {
        grad->data[kend] = args->GradTemp->data[kend];
      }

      args = obj->ExtraArgs;
      for (kend = 0; kend < 36; kend++) {
        a[kend] = args->WeightMatrix[kend];
      }

      J_4 = b->size[0];
      b->size[0] = args->ErrTemp->size[0];
      ArmRev_emxEnsureCapacity_real_T(b, J_4);
      loop_ub = args->ErrTemp->size[0];
      for (kend = 0; kend < loop_ub; kend++) {
        b->data[kend] = args->ErrTemp->data[kend];
      }

      J_4 = ev->size[0];
      ev->size[0] = args->ErrTemp->size[0];
      ArmRev_emxEnsureCapacity_real_T(ev, J_4);
      loop_ub = args->ErrTemp->size[0];
      for (kend = 0; kend < loop_ub; kend++) {
        ev->data[kend] = args->ErrTemp->data[kend];
      }

      for (kend = 0; kend < 6; kend++) {
        cc = 0.0;
        for (J_4 = 0; J_4 < 6; J_4++) {
          cc += a[6 * J_4 + kend] * b->data[J_4];
        }

        e[kend] = cc;
      }

      *en = ArmReverseKinematics_norm_e(e);
      *iter = (real_T)b_i + 1.0;
      if (grad->size[0] == 0) {
        cc = 0.0;
      } else {
        cc = 0.0;
        if (grad->size[0] == 1) {
          cc = fabs(grad->data[0]);
        } else {
          scale = 3.3121686421112381E-170;
          kend = grad->size[0] - 1;
          for (J_4 = 0; J_4 <= kend; J_4++) {
            bidx = fabs(grad->data[J_4]);
            if (bidx > scale) {
              t = scale / bidx;
              cc = cc * t * t + 1.0;
              scale = bidx;
            } else {
              t = bidx / scale;
              cc += t * t;
            }
          }

          cc = scale * sqrt(cc);
        }
      }

      flag = (cc < obj->GradientTolerance);
      if (flag) {
        *exitFlag = LocalMinimumFound;
        exitg1 = 1;
      } else {
        guard1 = false;
        guard2 = false;
        if ((real_T)b_i + 1.0 > 1.0) {
          x[0] = (fabs(xSol[0] - xprev_idx_0) < obj->StepTolerance);
          x[1] = (fabs(xSol[1] - xprev_idx_1) < obj->StepTolerance);
          x[2] = (fabs(xSol[2] - xprev_idx_2) < obj->StepTolerance);
          x[3] = (fabs(xSol[3] - xprev_idx_3) < obj->StepTolerance);
          flag = true;
          J_4 = 0;
          exitg2 = false;
          while ((!exitg2) && (J_4 < 4)) {
            if (!x[J_4]) {
              flag = false;
              exitg2 = true;
            } else {
              J_4++;
            }
          }

          if (flag) {
            *exitFlag = StepSizeBelowMinimum;
            exitg1 = 1;
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }

        if (guard2) {
          if ((real_T)b_i + 1.0 > 1.0) {
            if (ev->size[0] == evprev->size[0]) {
              J_4 = evprev->size[0];
              evprev->size[0] = ev->size[0];
              ArmRev_emxEnsureCapacity_real_T(evprev, J_4);
              loop_ub = ev->size[0];
              m = (loop_ub / 2) << 1;
              coffset = m - 2;
              for (kend = 0; kend <= coffset; kend += 2) {
                tmp_2 = _mm_loadu_pd(&ev->data[kend]);
                tmp_1 = _mm_loadu_pd(&evprev->data[kend]);
                _mm_storeu_pd(&evprev->data[kend], _mm_sub_pd(tmp_2, tmp_1));
              }

              for (kend = m; kend < loop_ub; kend++) {
                evprev->data[kend] = ev->data[kend] - evprev->data[kend];
              }
            } else {
              ArmReverseKinematics_minus_e(evprev, ev);
            }

            kend = evprev->size[0] - 1;
            J_4 = b->size[0];
            b->size[0] = evprev->size[0];
            ArmRev_emxEnsureCapacity_real_T(b, J_4);
            for (J_4 = 0; J_4 <= kend; J_4++) {
              b->data[J_4] = fabs(evprev->data[J_4]);
            }

            J_4 = x_0->size[0];
            x_0->size[0] = b->size[0];
            Arm_emxEnsureCapacity_boolean_T(x_0, J_4);
            loop_ub = b->size[0];
            for (kend = 0; kend < loop_ub; kend++) {
              x_0->data[kend] = (b->data[kend] < obj->ErrorChangeTolerance);
            }

            flag = true;
            J_4 = 0;
            exitg2 = false;
            while ((!exitg2) && (J_4 + 1 <= x_0->size[0])) {
              if (!x_0->data[J_4]) {
                flag = false;
                exitg2 = true;
              } else {
                J_4++;
              }
            }

            if (flag) {
              *exitFlag = ChangeInErrorBelowMinimum;
              exitg1 = 1;
            } else {
              guard1 = true;
            }
          } else {
            guard1 = true;
          }
        }

        if (guard1) {
          cc = ArmReverseKinematics_toc(obj->TimeObjInternal.StartTime.tv_sec,
            obj->TimeObjInternal.StartTime.tv_nsec);
          flag = (cc > obj->MaxTimeInternal);
          if (flag) {
            *exitFlag = TimeLimitExceeded;
            exitg1 = 1;
          } else {
            J_4 = evprev->size[0];
            evprev->size[0] = ev->size[0];
            ArmRev_emxEnsureCapacity_real_T(evprev, J_4);
            loop_ub = ev->size[0];
            if (loop_ub - 1 >= 0) {
              memcpy(&evprev->data[0], &ev->data[0], (uint32_T)loop_ub * sizeof
                     (real_T));
            }

            xprev_idx_0 = xSol[0];
            xprev_idx_1 = xSol[1];
            xprev_idx_2 = xSol[2];
            xprev_idx_3 = xSol[3];
            flag = obj->UseErrorDamping;
            cc = (real_T)flag * cost;
            bidx = cc + obj->DampingBias;
            m = J->size[1];
            J_4 = y->size[0] * y->size[1];
            y->size[0] = J->size[1];
            y->size[1] = 6;
            ArmRev_emxEnsureCapacity_real_T(y, J_4);
            for (kend = 0; kend < 6; kend++) {
              coffset = kend * m - 1;
              boffset = kend * 6 - 1;
              for (loop_ub = 0; loop_ub < m; loop_ub++) {
                aoffset = loop_ub * 6 - 1;
                scale = 0.0;
                for (J_4 = 0; J_4 < 6; J_4++) {
                  scale += J->data[(J_4 + aoffset) + 1] * weightMatrix[(J_4 +
                    boffset) + 1];
                }

                y->data[(coffset + loop_ub) + 1] = scale;
              }
            }

            m = y->size[0];
            aoffset = J->size[1] - 1;
            J_4 = H0->size[0] * H0->size[1];
            H0->size[0] = y->size[0];
            H0->size[1] = J->size[1];
            ArmRev_emxEnsureCapacity_real_T(H0, J_4);
            for (kend = 0; kend <= aoffset; kend++) {
              coffset = kend * m - 1;
              boffset = kend * 6 - 1;
              for (loop_ub = 0; loop_ub < m; loop_ub++) {
                scale = 0.0;
                for (J_4 = 0; J_4 < 6; J_4++) {
                  scale += y->data[J_4 * y->size[0] + loop_ub] * J->data
                    [(boffset + J_4) + 1];
                }

                H0->data[(coffset + loop_ub) + 1] = scale;
              }
            }

            if ((H0->size[0] == 4) && (H0->size[1] == 4)) {
              for (kend = 0; kend <= 14; kend += 2) {
                tmp_2 = _mm_loadu_pd(&tmp_0[kend]);
                tmp_1 = _mm_loadu_pd(&H0->data[kend]);
                _mm_storeu_pd(&H0_0[kend], _mm_mul_pd(_mm_add_pd(tmp_1,
                  _mm_mul_pd(_mm_set1_pd(bidx), tmp_2)), _mm_set1_pd(-1.0)));
              }

              ArmReverseKinematics_mldivide(H0_0, grad, step_data, &kend);
            } else {
              ArmReverseKi_binary_expand_op_1(step_data, &kend, H0, bidx, tmp_0,
                grad);
            }

            args = obj->ExtraArgs;
            treeInternal = args->Robot;
            for (kend = 0; kend < 16; kend++) {
              Td[kend] = args->Tform[kend];
            }

            for (kend = 0; kend < 36; kend++) {
              weightMatrix[kend] = args->WeightMatrix[kend];
            }

            bidx = args->BodyIndex;
            y_0[0] = xSol[0] + step_data[0];
            y_0[1] = xSol[1] + step_data[1];
            y_0[2] = xSol[2] + step_data[2];
            y_0[3] = xSol[3] + step_data[3];
            RigidBodyTree_efficientFKAndJac(treeInternal, y_0, bidx, T_data,
              T_size, J_0);
            ArmReverseK_IKHelpers_poseError(Td, T_data, T_size, e);
            J_4 = args->ErrTemp->size[0];
            args->ErrTemp->size[0] = 6;
            ArmRev_emxEnsureCapacity_real_T(args->ErrTemp, J_4);
            for (kend = 0; kend < 6; kend++) {
              args->ErrTemp->data[kend] = e[kend];
            }

            bidx = 0.0;
            for (kend = 0; kend < 6; kend++) {
              t = 0.0;
              for (J_4 = 0; J_4 < 6; J_4++) {
                t += weightMatrix[6 * kend + J_4] * (0.5 * e[J_4]);
              }

              bidx += t * e[kend];
            }

            args->CostTemp = bidx;
            for (kend = 0; kend < 6; kend++) {
              bidx = 0.0;
              for (J_4 = 0; J_4 < 6; J_4++) {
                bidx += weightMatrix[6 * kend + J_4] * e[J_4];
              }

              e_0[kend] = bidx;
            }

            J_4 = J_2->size[0] * J_2->size[1];
            J_2->size[0] = 6;
            J_2->size[1] = J_0->size[1];
            ArmRev_emxEnsureCapacity_real_T(J_2, J_4);
            loop_ub = 6 * J_0->size[1];
            m = (loop_ub / 2) << 1;
            coffset = m - 2;
            for (kend = 0; kend <= coffset; kend += 2) {
              tmp_2 = _mm_loadu_pd(&J_0->data[kend]);
              _mm_storeu_pd(&J_2->data[kend], _mm_mul_pd(tmp_2, _mm_set1_pd(-1.0)));
            }

            for (kend = m; kend < loop_ub; kend++) {
              J_2->data[kend] = -J_0->data[kend];
            }

            ArmReverseKinematics_mtimes_e(e_0, J_2, tmp);
            J_4 = args->GradTemp->size[0];
            args->GradTemp->size[0] = tmp->size[1];
            ArmRev_emxEnsureCapacity_real_T(args->GradTemp, J_4);
            loop_ub = tmp->size[1];
            for (kend = 0; kend < loop_ub; kend++) {
              args->GradTemp->data[kend] = tmp->data[kend];
            }

            bidx = args->CostTemp;
            scale = 1.0;
            while (bidx > cost) {
              scale *= 2.5;
              bidx = scale * obj->DampingBias + cc;
              if ((H0->size[0] == 4) && (H0->size[1] == 4)) {
                for (kend = 0; kend <= 14; kend += 2) {
                  tmp_2 = _mm_loadu_pd(&tmp_0[kend]);
                  tmp_1 = _mm_loadu_pd(&H0->data[kend]);
                  _mm_storeu_pd(&H0_1[kend], _mm_mul_pd(_mm_add_pd(tmp_1,
                    _mm_mul_pd(_mm_set1_pd(bidx), tmp_2)), _mm_set1_pd(-1.0)));
                }

                ArmReverseKinematics_mldivide(H0_1, grad, step_data, &kend);
              } else {
                ArmReverseKi_binary_expand_op_1(step_data, &kend, H0, bidx,
                  tmp_0, grad);
              }

              args = obj->ExtraArgs;
              treeInternal = args->Robot;
              for (kend = 0; kend < 16; kend++) {
                Td[kend] = args->Tform[kend];
              }

              for (kend = 0; kend < 36; kend++) {
                weightMatrix[kend] = args->WeightMatrix[kend];
              }

              bidx = args->BodyIndex;
              y_0[0] = xSol[0] + step_data[0];
              y_0[1] = xSol[1] + step_data[1];
              y_0[2] = xSol[2] + step_data[2];
              y_0[3] = xSol[3] + step_data[3];
              RigidBodyTree_efficientFKAndJac(treeInternal, y_0, bidx, T_data,
                T_size, J_0);
              ArmReverseK_IKHelpers_poseError(Td, T_data, T_size, e);
              J_4 = args->ErrTemp->size[0];
              args->ErrTemp->size[0] = 6;
              ArmRev_emxEnsureCapacity_real_T(args->ErrTemp, J_4);
              for (kend = 0; kend < 6; kend++) {
                args->ErrTemp->data[kend] = e[kend];
              }

              bidx = 0.0;
              for (kend = 0; kend < 6; kend++) {
                t = 0.0;
                for (J_4 = 0; J_4 < 6; J_4++) {
                  t += weightMatrix[6 * kend + J_4] * (0.5 * e[J_4]);
                }

                bidx += t * e[kend];
              }

              args->CostTemp = bidx;
              for (kend = 0; kend < 6; kend++) {
                bidx = 0.0;
                for (J_4 = 0; J_4 < 6; J_4++) {
                  bidx += weightMatrix[6 * kend + J_4] * e[J_4];
                }

                e_0[kend] = bidx;
              }

              J_4 = J_3->size[0] * J_3->size[1];
              J_3->size[0] = 6;
              J_3->size[1] = J_0->size[1];
              ArmRev_emxEnsureCapacity_real_T(J_3, J_4);
              loop_ub = 6 * J_0->size[1];
              m = (loop_ub / 2) << 1;
              coffset = m - 2;
              for (kend = 0; kend <= coffset; kend += 2) {
                tmp_2 = _mm_loadu_pd(&J_0->data[kend]);
                _mm_storeu_pd(&J_3->data[kend], _mm_mul_pd(tmp_2, _mm_set1_pd
                  (-1.0)));
              }

              for (kend = m; kend < loop_ub; kend++) {
                J_3->data[kend] = -J_0->data[kend];
              }

              ArmReverseKinematics_mtimes_e(e_0, J_3, tmp);
              J_4 = args->GradTemp->size[0];
              args->GradTemp->size[0] = tmp->size[1];
              ArmRev_emxEnsureCapacity_real_T(args->GradTemp, J_4);
              loop_ub = tmp->size[1];
              for (kend = 0; kend < loop_ub; kend++) {
                args->GradTemp->data[kend] = tmp->data[kend];
              }

              bidx = args->CostTemp;
            }

            bidx = xSol[0] + step_data[0];
            xSol[0] += step_data[0];
            t = xSol[1] + step_data[1];
            xSol[1] += step_data[1];
            cost = xSol[2] + step_data[2];
            xSol[2] += step_data[2];
            cc = xSol[3] + step_data[3];
            xSol[3] += step_data[3];
            if (obj->ConstraintsOn) {
              args = obj->ExtraArgs;
              J_4 = grad->size[0];
              grad->size[0] = args->Limits->size[0];
              ArmRev_emxEnsureCapacity_real_T(grad, J_4);
              loop_ub = args->Limits->size[0];
              for (kend = 0; kend < loop_ub; kend++) {
                grad->data[kend] = args->Limits->data[kend];
              }

              if (grad->size[0] == 4) {
                y_0[0] = fmax(grad->data[0], bidx);
                y_0[1] = fmax(grad->data[1], t);
                y_0[2] = fmax(grad->data[2], cost);
                y_0[3] = fmax(grad->data[3], cc);
              } else {
                ArmReverseKinematics_expand_max(grad, xSol, y_0);
              }

              J_4 = grad->size[0];
              grad->size[0] = args->Limits->size[0];
              ArmRev_emxEnsureCapacity_real_T(grad, J_4);
              loop_ub = args->Limits->size[0];
              for (kend = 0; kend < loop_ub; kend++) {
                grad->data[kend] = args->Limits->data[kend + args->Limits->size
                  [0]];
              }

              if (grad->size[0] == 4) {
                xSol[0] = fmin(grad->data[0], y_0[0]);
                xSol[1] = fmin(grad->data[1], y_0[1]);
                xSol[2] = fmin(grad->data[2], y_0[2]);
                xSol[3] = fmin(grad->data[3], y_0[3]);
              } else {
                ArmReverseKinematics_expand_min(grad, y_0, xSol);
              }
            }

            b_i++;
          }
        }
      }
    } else {
      args = obj->ExtraArgs;
      for (kend = 0; kend < 36; kend++) {
        a[kend] = args->WeightMatrix[kend];
      }

      J_4 = b->size[0];
      b->size[0] = args->ErrTemp->size[0];
      ArmRev_emxEnsureCapacity_real_T(b, J_4);
      loop_ub = args->ErrTemp->size[0];
      for (kend = 0; kend < loop_ub; kend++) {
        b->data[kend] = args->ErrTemp->data[kend];
      }

      for (kend = 0; kend < 6; kend++) {
        cc = 0.0;
        for (J_4 = 0; J_4 < 6; J_4++) {
          cc += a[6 * J_4 + kend] * b->data[J_4];
        }

        e[kend] = cc;
      }

      *en = ArmReverseKinematics_norm_e(e);
      *iter = obj->MaxNumIterationInternal;
      *exitFlag = IterationLimitExceeded;
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  ArmReverseKinema_emxFree_real_T(&tmp);
  ArmReverseKin_emxFree_boolean_T(&x_0);
  ArmReverseKinema_emxFree_real_T(&b);
  ArmReverseKinema_emxFree_real_T(&y);
  ArmReverseKinema_emxFree_real_T(&grad);
  ArmReverseKinema_emxFree_real_T(&J_3);
  ArmReverseKinema_emxFree_real_T(&J_2);
  ArmReverseKinema_emxFree_real_T(&J_1);
  ArmReverseKinema_emxFree_real_T(&J_0);
  ArmReverseKinema_emxFree_real_T(&ev);
  ArmReverseKinema_emxFree_real_T(&J);
  ArmReverseKinema_emxFree_real_T(&H0);
  ArmReverseKinema_emxFree_real_T(&evprev);
}

static boolean_T ArmReverseKinematics_any(const emxArray_boolean_T_ArmReverse_T *
  x)
{
  int32_T ix;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  ix = 0;
  exitg1 = false;
  while ((!exitg1) && (ix + 1 <= x->size[0])) {
    if (x->data[ix]) {
      y = true;
      exitg1 = true;
    } else {
      ix++;
    }
  }

  return y;
}

static void ArmReverseKinematics_randn(const real_T varargin_1[2],
  emxArray_real_T_ArmReverseKin_T *r)
{
  real_T xi[257];
  real_T b_r;
  real_T d_u;
  real_T x;
  int32_T b_k;
  int32_T i;
  uint32_T u32[2];
  static const real_T tmp[257] = { 1.0, 0.977101701267673, 0.959879091800108,
    0.9451989534423, 0.932060075959231, 0.919991505039348, 0.908726440052131,
    0.898095921898344, 0.887984660755834, 0.878309655808918, 0.869008688036857,
    0.860033621196332, 0.851346258458678, 0.842915653112205, 0.834716292986884,
    0.826726833946222, 0.818929191603703, 0.811307874312656, 0.803849483170964,
    0.796542330422959, 0.789376143566025, 0.782341832654803, 0.775431304981187,
    0.768637315798486, 0.761953346836795, 0.755373506507096, 0.748892447219157,
    0.742505296340151, 0.736207598126863, 0.729995264561476, 0.72386453346863,
    0.717811932630722, 0.711834248878248, 0.705928501332754, 0.700091918136512,
    0.694321916126117, 0.688616083004672, 0.682972161644995, 0.677388036218774,
    0.671861719897082, 0.66639134390875, 0.660975147776663, 0.655611470579697,
    0.650298743110817, 0.645035480820822, 0.639820277453057, 0.634651799287624,
    0.629528779924837, 0.624450015547027, 0.619414360605834, 0.614420723888914,
    0.609468064925773, 0.604555390697468, 0.599681752619125, 0.594846243767987,
    0.590047996332826, 0.585286179263371, 0.580559996100791, 0.575868682972354,
    0.571211506735253, 0.566587763256165, 0.561996775814525, 0.557437893618766,
    0.552910490425833, 0.548413963255266, 0.543947731190026, 0.539511234256952,
    0.535103932380458, 0.530725304403662, 0.526374847171684, 0.522052074672322,
    0.517756517229756, 0.513487720747327, 0.509245245995748, 0.505028667943468,
    0.500837575126149, 0.49667156905249, 0.492530263643869, 0.488413284705458,
    0.484320269426683, 0.480250865909047, 0.476204732719506, 0.47218153846773,
    0.468180961405694, 0.464202689048174, 0.460246417812843, 0.456311852678716,
    0.452398706861849, 0.448506701507203, 0.444635565395739, 0.440785034665804,
    0.436954852547985, 0.433144769112652, 0.429354541029442, 0.425583931338022,
    0.421832709229496, 0.418100649837848, 0.414387534040891, 0.410693148270188,
    0.407017284329473, 0.403359739221114, 0.399720314980197, 0.396098818515832,
    0.392495061459315, 0.388908860018789, 0.385340034840077, 0.381788410873393,
    0.378253817245619, 0.374736087137891, 0.371235057668239, 0.367750569779032,
    0.364282468129004, 0.360830600989648, 0.357394820145781, 0.353974980800077,
    0.350570941481406, 0.347182563956794, 0.343809713146851, 0.340452257044522,
    0.337110066637006, 0.333783015830718, 0.330470981379163, 0.327173842813601,
    0.323891482376391, 0.320623784956905, 0.317370638029914, 0.314131931596337,
    0.310907558126286, 0.307697412504292, 0.30450139197665, 0.301319396100803,
    0.298151326696685, 0.294997087799962, 0.291856585617095, 0.288729728482183,
    0.285616426815502, 0.282516593083708, 0.279430141761638, 0.276356989295668,
    0.273297054068577, 0.270250256365875, 0.267216518343561, 0.264195763997261,
    0.261187919132721, 0.258192911337619, 0.255210669954662, 0.252241126055942,
    0.249284212418529, 0.246339863501264, 0.24340801542275, 0.240488605940501,
    0.237581574431238, 0.23468686187233, 0.231804410824339, 0.228934165414681,
    0.226076071322381, 0.223230075763918, 0.220396127480152, 0.217574176724331,
    0.214764175251174, 0.211966076307031, 0.209179834621125, 0.206405406397881,
    0.203642749310335, 0.200891822494657, 0.198152586545776, 0.195425003514135,
    0.192709036903589, 0.190004651670465, 0.187311814223801, 0.1846304924268,
    0.181960655599523, 0.179302274522848, 0.176655321443735, 0.174019770081839,
    0.171395595637506, 0.168782774801212, 0.166181285764482, 0.163591108232366,
    0.161012223437511, 0.158444614155925, 0.15588826472448, 0.153343161060263,
    0.150809290681846, 0.148286642732575, 0.145775208005994, 0.143274978973514,
    0.140785949814445, 0.138308116448551, 0.135841476571254, 0.133386029691669,
    0.130941777173644, 0.12850872228, 0.126086870220186, 0.123676228201597,
    0.12127680548479, 0.11888861344291, 0.116511665625611, 0.114145977827839,
    0.111791568163838, 0.109448457146812, 0.107116667774684, 0.104796225622487,
    0.102487158941935, 0.10018949876881, 0.0979032790388625, 0.095628536713009,
    0.093365311912691, 0.0911136480663738, 0.0888735920682759,
    0.0866451944505581, 0.0844285095703535, 0.082223595813203,
    0.0800305158146631, 0.0778493367020961, 0.0756801303589272,
    0.0735229737139814, 0.0713779490588905, 0.0692451443970068,
    0.0671246538277886, 0.065016577971243, 0.0629210244377582, 0.06083810834954,
    0.0587679529209339, 0.0567106901062031, 0.0546664613248891,
    0.0526354182767924, 0.0506177238609479, 0.0486135532158687,
    0.0466230949019305, 0.0446465522512946, 0.0426841449164746,
    0.0407361106559411, 0.0388027074045262, 0.0368842156885674,
    0.0349809414617162, 0.0330932194585786, 0.0312214171919203,
    0.0293659397581334, 0.0275272356696031, 0.0257058040085489,
    0.0239022033057959, 0.0221170627073089, 0.0203510962300445,
    0.0186051212757247, 0.0168800831525432, 0.0151770883079353,
    0.0134974506017399, 0.0118427578579079, 0.0102149714397015,
    0.00861658276939875, 0.00705087547137324, 0.00552240329925101,
    0.00403797259336304, 0.00260907274610216, 0.0012602859304986,
    0.000477467764609386 };

  const real_T *fitab;
  int32_T d_tmp;
  int32_T exitg1;
  int32_T exitg2;
  b_k = r->size[0];
  r->size[0] = (int32_T)varargin_1[0];
  ArmRev_emxEnsureCapacity_real_T(r, b_k);
  d_tmp = (int32_T)varargin_1[0] - 1;
  if ((int32_T)varargin_1[0] - 1 >= 0) {
    xi[0] = 0.0;
    xi[1] = 0.215241895984875;
    xi[2] = 0.286174591792068;
    xi[3] = 0.335737519214422;
    xi[4] = 0.375121332878378;
    xi[5] = 0.408389134611989;
    xi[6] = 0.43751840220787;
    xi[7] = 0.46363433679088;
    xi[8] = 0.487443966139235;
    xi[9] = 0.50942332960209;
    xi[10] = 0.529909720661557;
    xi[11] = 0.549151702327164;
    xi[12] = 0.567338257053817;
    xi[13] = 0.584616766106378;
    xi[14] = 0.601104617755991;
    xi[15] = 0.61689699000775;
    xi[16] = 0.63207223638606;
    xi[17] = 0.646695714894993;
    xi[18] = 0.660822574244419;
    xi[19] = 0.674499822837293;
    xi[20] = 0.687767892795788;
    xi[21] = 0.700661841106814;
    xi[22] = 0.713212285190975;
    xi[23] = 0.725446140909999;
    xi[24] = 0.737387211434295;
    xi[25] = 0.749056662017815;
    xi[26] = 0.760473406430107;
    xi[27] = 0.771654424224568;
    xi[28] = 0.782615023307232;
    xi[29] = 0.793369058840623;
    xi[30] = 0.80392911698997;
    xi[31] = 0.814306670135215;
    xi[32] = 0.824512208752291;
    xi[33] = 0.834555354086381;
    xi[34] = 0.844444954909153;
    xi[35] = 0.854189171008163;
    xi[36] = 0.863795545553308;
    xi[37] = 0.87327106808886;
    xi[38] = 0.882622229585165;
    xi[39] = 0.891855070732941;
    xi[40] = 0.900975224461221;
    xi[41] = 0.909987953496718;
    xi[42] = 0.91889818364959;
    xi[43] = 0.927710533401999;
    xi[44] = 0.936429340286575;
    xi[45] = 0.945058684468165;
    xi[46] = 0.953602409881086;
    xi[47] = 0.96206414322304;
    xi[48] = 0.970447311064224;
    xi[49] = 0.978755155294224;
    xi[50] = 0.986990747099062;
    xi[51] = 0.99515699963509;
    xi[52] = 1.00325667954467;
    xi[53] = 1.01129241744;
    xi[54] = 1.01926671746548;
    xi[55] = 1.02718196603564;
    xi[56] = 1.03504043983344;
    xi[57] = 1.04284431314415;
    xi[58] = 1.05059566459093;
    xi[59] = 1.05829648333067;
    xi[60] = 1.06594867476212;
    xi[61] = 1.07355406579244;
    xi[62] = 1.0811144097034;
    xi[63] = 1.08863139065398;
    xi[64] = 1.09610662785202;
    xi[65] = 1.10354167942464;
    xi[66] = 1.11093804601357;
    xi[67] = 1.11829717411934;
    xi[68] = 1.12562045921553;
    xi[69] = 1.13290924865253;
    xi[70] = 1.14016484436815;
    xi[71] = 1.14738850542085;
    xi[72] = 1.15458145035993;
    xi[73] = 1.16174485944561;
    xi[74] = 1.16887987673083;
    xi[75] = 1.17598761201545;
    xi[76] = 1.18306914268269;
    xi[77] = 1.19012551542669;
    xi[78] = 1.19715774787944;
    xi[79] = 1.20416683014438;
    xi[80] = 1.2111537262437;
    xi[81] = 1.21811937548548;
    xi[82] = 1.22506469375653;
    xi[83] = 1.23199057474614;
    xi[84] = 1.23889789110569;
    xi[85] = 1.24578749554863;
    xi[86] = 1.2526602218949;
    xi[87] = 1.25951688606371;
    xi[88] = 1.26635828701823;
    xi[89] = 1.27318520766536;
    xi[90] = 1.27999841571382;
    xi[91] = 1.28679866449324;
    xi[92] = 1.29358669373695;
    xi[93] = 1.30036323033084;
    xi[94] = 1.30712898903073;
    xi[95] = 1.31388467315022;
    xi[96] = 1.32063097522106;
    xi[97] = 1.32736857762793;
    xi[98] = 1.33409815321936;
    xi[99] = 1.3408203658964;
    xi[100] = 1.34753587118059;
    xi[101] = 1.35424531676263;
    xi[102] = 1.36094934303328;
    xi[103] = 1.36764858359748;
    xi[104] = 1.37434366577317;
    xi[105] = 1.38103521107586;
    xi[106] = 1.38772383568998;
    xi[107] = 1.39441015092814;
    xi[108] = 1.40109476367925;
    xi[109] = 1.4077782768464;
    xi[110] = 1.41446128977547;
    xi[111] = 1.42114439867531;
    xi[112] = 1.42782819703026;
    xi[113] = 1.43451327600589;
    xi[114] = 1.44120022484872;
    xi[115] = 1.44788963128058;
    xi[116] = 1.45458208188841;
    xi[117] = 1.46127816251028;
    xi[118] = 1.46797845861808;
    xi[119] = 1.47468355569786;
    xi[120] = 1.48139403962819;
    xi[121] = 1.48811049705745;
    xi[122] = 1.49483351578049;
    xi[123] = 1.50156368511546;
    xi[124] = 1.50830159628131;
    xi[125] = 1.51504784277671;
    xi[126] = 1.521803020761;
    xi[127] = 1.52856772943771;
    xi[128] = 1.53534257144151;
    xi[129] = 1.542128153229;
    xi[130] = 1.54892508547417;
    xi[131] = 1.55573398346918;
    xi[132] = 1.56255546753104;
    xi[133] = 1.56939016341512;
    xi[134] = 1.57623870273591;
    xi[135] = 1.58310172339603;
    xi[136] = 1.58997987002419;
    xi[137] = 1.59687379442279;
    xi[138] = 1.60378415602609;
    xi[139] = 1.61071162236983;
    xi[140] = 1.61765686957301;
    xi[141] = 1.62462058283303;
    xi[142] = 1.63160345693487;
    xi[143] = 1.63860619677555;
    xi[144] = 1.64562951790478;
    xi[145] = 1.65267414708306;
    xi[146] = 1.65974082285818;
    xi[147] = 1.66683029616166;
    xi[148] = 1.67394333092612;
    xi[149] = 1.68108070472517;
    xi[150] = 1.68824320943719;
    xi[151] = 1.69543165193456;
    xi[152] = 1.70264685479992;
    xi[153] = 1.7098896570713;
    xi[154] = 1.71716091501782;
    xi[155] = 1.72446150294804;
    xi[156] = 1.73179231405296;
    xi[157] = 1.73915426128591;
    xi[158] = 1.74654827828172;
    xi[159] = 1.75397532031767;
    xi[160] = 1.76143636531891;
    xi[161] = 1.76893241491127;
    xi[162] = 1.77646449552452;
    xi[163] = 1.78403365954944;
    xi[164] = 1.79164098655216;
    xi[165] = 1.79928758454972;
    xi[166] = 1.80697459135082;
    xi[167] = 1.81470317596628;
    xi[168] = 1.82247454009388;
    xi[169] = 1.83028991968276;
    xi[170] = 1.83815058658281;
    xi[171] = 1.84605785028518;
    xi[172] = 1.8540130597602;
    xi[173] = 1.86201760539967;
    xi[174] = 1.87007292107127;
    xi[175] = 1.878180486293;
    xi[176] = 1.88634182853678;
    xi[177] = 1.8945585256707;
    xi[178] = 1.90283220855043;
    xi[179] = 1.91116456377125;
    xi[180] = 1.91955733659319;
    xi[181] = 1.92801233405266;
    xi[182] = 1.93653142827569;
    xi[183] = 1.94511656000868;
    xi[184] = 1.95376974238465;
    xi[185] = 1.96249306494436;
    xi[186] = 1.97128869793366;
    xi[187] = 1.98015889690048;
    xi[188] = 1.98910600761744;
    xi[189] = 1.99813247135842;
    xi[190] = 2.00724083056053;
    xi[191] = 2.0164337349062;
    xi[192] = 2.02571394786385;
    xi[193] = 2.03508435372962;
    xi[194] = 2.04454796521753;
    xi[195] = 2.05410793165065;
    xi[196] = 2.06376754781173;
    xi[197] = 2.07353026351874;
    xi[198] = 2.0833996939983;
    xi[199] = 2.09337963113879;
    xi[200] = 2.10347405571488;
    xi[201] = 2.11368715068665;
    xi[202] = 2.12402331568952;
    xi[203] = 2.13448718284602;
    xi[204] = 2.14508363404789;
    xi[205] = 2.15581781987674;
    xi[206] = 2.16669518035431;
    xi[207] = 2.17772146774029;
    xi[208] = 2.18890277162636;
    xi[209] = 2.20024554661128;
    xi[210] = 2.21175664288416;
    xi[211] = 2.22344334009251;
    xi[212] = 2.23531338492992;
    xi[213] = 2.24737503294739;
    xi[214] = 2.25963709517379;
    xi[215] = 2.27210899022838;
    xi[216] = 2.28480080272449;
    xi[217] = 2.29772334890286;
    xi[218] = 2.31088825060137;
    xi[219] = 2.32430801887113;
    xi[220] = 2.33799614879653;
    xi[221] = 2.35196722737914;
    xi[222] = 2.36623705671729;
    xi[223] = 2.38082279517208;
    xi[224] = 2.39574311978193;
    xi[225] = 2.41101841390112;
    xi[226] = 2.42667098493715;
    xi[227] = 2.44272531820036;
    xi[228] = 2.4592083743347;
    xi[229] = 2.47614993967052;
    xi[230] = 2.49358304127105;
    xi[231] = 2.51154444162669;
    xi[232] = 2.53007523215985;
    xi[233] = 2.54922155032478;
    xi[234] = 2.56903545268184;
    xi[235] = 2.58957598670829;
    xi[236] = 2.61091051848882;
    xi[237] = 2.63311639363158;
    xi[238] = 2.65628303757674;
    xi[239] = 2.68051464328574;
    xi[240] = 2.70593365612306;
    xi[241] = 2.73268535904401;
    xi[242] = 2.76094400527999;
    xi[243] = 2.79092117400193;
    xi[244] = 2.82287739682644;
    xi[245] = 2.85713873087322;
    xi[246] = 2.89412105361341;
    xi[247] = 2.93436686720889;
    xi[248] = 2.97860327988184;
    xi[249] = 3.02783779176959;
    xi[250] = 3.08352613200214;
    xi[251] = 3.147889289518;
    xi[252] = 3.2245750520478;
    xi[253] = 3.32024473383983;
    xi[254] = 3.44927829856143;
    xi[255] = 3.65415288536101;
    xi[256] = 3.91075795952492;
    fitab = &tmp[0];
  }

  for (b_k = 0; b_k <= d_tmp; b_k++) {
    do {
      exitg1 = 0;
      ArmReve_genrand_uint32_vector_e(rtDW.state_i, u32);
      i = (int32_T)((u32[1] >> 24U) + 1U);
      b_r = (((real_T)(u32[0] >> 3U) * 1.6777216E+7 + (real_T)((int32_T)u32[1] &
               16777215)) * 2.2204460492503131E-16 - 1.0) * xi[i];
      if (fabs(b_r) <= xi[i - 1]) {
        exitg1 = 1;
      } else if (i < 256) {
        /* ========================= COPYRIGHT NOTICE ============================ */
        /*  This is a uniform (0,1) pseudorandom number generator based on:        */
        /*                                                                         */
        /*  A C-program for MT19937, with initialization improved 2002/1/26.       */
        /*  Coded by Takuji Nishimura and Makoto Matsumoto.                        */
        /*                                                                         */
        /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      */
        /*  All rights reserved.                                                   */
        /*                                                                         */
        /*  Redistribution and use in source and binary forms, with or without     */
        /*  modification, are permitted provided that the following conditions     */
        /*  are met:                                                               */
        /*                                                                         */
        /*    1. Redistributions of source code must retain the above copyright    */
        /*       notice, this list of conditions and the following disclaimer.     */
        /*                                                                         */
        /*    2. Redistributions in binary form must reproduce the above copyright */
        /*       notice, this list of conditions and the following disclaimer      */
        /*       in the documentation and/or other materials provided with the     */
        /*       distribution.                                                     */
        /*                                                                         */
        /*    3. The names of its contributors may not be used to endorse or       */
        /*       promote products derived from this software without specific      */
        /*       prior written permission.                                         */
        /*                                                                         */
        /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
        /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
        /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
        /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  */
        /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
        /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
        /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
        /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
        /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
        /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
        /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */
        /*                                                                         */
        /* =============================   END   ================================= */
        do {
          exitg2 = 0;
          ArmReve_genrand_uint32_vector_e(rtDW.state_i, u32);
          x = ((real_T)(u32[0] >> 5U) * 6.7108864E+7 + (real_T)(u32[1] >> 6U)) *
            1.1102230246251565E-16;
          if (x == 0.0) {
            if (!ArmReverseKinema_is_valid_state(rtDW.state_i)) {
              rtDW.state_i[0] = 5489U;
              rtDW.state_i[624] = 624U;
            }
          } else {
            exitg2 = 1;
          }
        } while (exitg2 == 0);

        if ((fitab[i - 1] - fitab[i]) * x + fitab[i] < exp(-0.5 * b_r * b_r)) {
          exitg1 = 1;
        }
      } else {
        do {
          /* ========================= COPYRIGHT NOTICE ============================ */
          /*  This is a uniform (0,1) pseudorandom number generator based on:        */
          /*                                                                         */
          /*  A C-program for MT19937, with initialization improved 2002/1/26.       */
          /*  Coded by Takuji Nishimura and Makoto Matsumoto.                        */
          /*                                                                         */
          /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      */
          /*  All rights reserved.                                                   */
          /*                                                                         */
          /*  Redistribution and use in source and binary forms, with or without     */
          /*  modification, are permitted provided that the following conditions     */
          /*  are met:                                                               */
          /*                                                                         */
          /*    1. Redistributions of source code must retain the above copyright    */
          /*       notice, this list of conditions and the following disclaimer.     */
          /*                                                                         */
          /*    2. Redistributions in binary form must reproduce the above copyright */
          /*       notice, this list of conditions and the following disclaimer      */
          /*       in the documentation and/or other materials provided with the     */
          /*       distribution.                                                     */
          /*                                                                         */
          /*    3. The names of its contributors may not be used to endorse or       */
          /*       promote products derived from this software without specific      */
          /*       prior written permission.                                         */
          /*                                                                         */
          /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
          /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
          /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
          /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  */
          /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
          /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
          /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
          /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
          /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
          /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
          /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */
          /*                                                                         */
          /* =============================   END   ================================= */
          do {
            exitg2 = 0;
            ArmReve_genrand_uint32_vector_e(rtDW.state_i, u32);
            x = ((real_T)(u32[0] >> 5U) * 6.7108864E+7 + (real_T)(u32[1] >> 6U))
              * 1.1102230246251565E-16;
            if (x == 0.0) {
              if (!ArmReverseKinema_is_valid_state(rtDW.state_i)) {
                rtDW.state_i[0] = 5489U;
                rtDW.state_i[624] = 624U;
              }
            } else {
              exitg2 = 1;
            }
          } while (exitg2 == 0);

          x = log(x) * 0.273661237329758;

          /* ========================= COPYRIGHT NOTICE ============================ */
          /*  This is a uniform (0,1) pseudorandom number generator based on:        */
          /*                                                                         */
          /*  A C-program for MT19937, with initialization improved 2002/1/26.       */
          /*  Coded by Takuji Nishimura and Makoto Matsumoto.                        */
          /*                                                                         */
          /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      */
          /*  All rights reserved.                                                   */
          /*                                                                         */
          /*  Redistribution and use in source and binary forms, with or without     */
          /*  modification, are permitted provided that the following conditions     */
          /*  are met:                                                               */
          /*                                                                         */
          /*    1. Redistributions of source code must retain the above copyright    */
          /*       notice, this list of conditions and the following disclaimer.     */
          /*                                                                         */
          /*    2. Redistributions in binary form must reproduce the above copyright */
          /*       notice, this list of conditions and the following disclaimer      */
          /*       in the documentation and/or other materials provided with the     */
          /*       distribution.                                                     */
          /*                                                                         */
          /*    3. The names of its contributors may not be used to endorse or       */
          /*       promote products derived from this software without specific      */
          /*       prior written permission.                                         */
          /*                                                                         */
          /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
          /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
          /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
          /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  */
          /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
          /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
          /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
          /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
          /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
          /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
          /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */
          /*                                                                         */
          /* =============================   END   ================================= */
          do {
            exitg2 = 0;
            ArmReve_genrand_uint32_vector_e(rtDW.state_i, u32);
            d_u = ((real_T)(u32[0] >> 5U) * 6.7108864E+7 + (real_T)(u32[1] >> 6U))
              * 1.1102230246251565E-16;
            if (d_u == 0.0) {
              if (!ArmReverseKinema_is_valid_state(rtDW.state_i)) {
                rtDW.state_i[0] = 5489U;
                rtDW.state_i[624] = 624U;
              }
            } else {
              exitg2 = 1;
            }
          } while (exitg2 == 0);
        } while (!(-2.0 * log(d_u) > x * x));

        if (b_r < 0.0) {
          b_r = x - 3.65415288536101;
        } else {
          b_r = 3.65415288536101 - x;
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);

    r->data[b_k] = b_r;
  }
}

static void ArmReverseKinematics_minus(emxArray_real_T_ArmReverseKin_T *in1,
  const emxArray_real_T_ArmReverseKin_T *in2)
{
  emxArray_real_T_ArmReverseKin_T *in1_0;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  ArmReverseKinema_emxInit_real_T(&in1_0, 1);
  if (in2->size[0] == 1) {
    i = in1_0->size[0];
    in1_0->size[0] = in1->size[0];
    ArmRev_emxEnsureCapacity_real_T(in1_0, i);
  } else {
    i = in1_0->size[0];
    in1_0->size[0] = in2->size[0];
    ArmRev_emxEnsureCapacity_real_T(in1_0, i);
  }

  stride_0_0 = (in1->size[0] != 1);
  stride_1_0 = (in2->size[0] != 1);
  if (in2->size[0] == 1) {
    loop_ub = in1->size[0];
  } else {
    loop_ub = in2->size[0];
  }

  for (i = 0; i < loop_ub; i++) {
    in1_0->data[i] = in1->data[i * stride_0_0] - in2->data[i * stride_1_0];
  }

  i = in1->size[0];
  in1->size[0] = in1_0->size[0];
  ArmRev_emxEnsureCapacity_real_T(in1, i);
  loop_ub = in1_0->size[0];
  if (loop_ub - 1 >= 0) {
    memcpy(&in1->data[0], &in1_0->data[0], (uint32_T)loop_ub * sizeof(real_T));
  }

  ArmReverseKinema_emxFree_real_T(&in1_0);
}

static void ArmReverseKinematics_plus(emxArray_real_T_ArmReverseKin_T *in1,
  const emxArray_real_T_ArmReverseKin_T *in2)
{
  emxArray_real_T_ArmReverseKin_T *in2_0;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  ArmReverseKinema_emxInit_real_T(&in2_0, 1);
  if (in1->size[0] == 1) {
    i = in2_0->size[0];
    in2_0->size[0] = in2->size[0];
    ArmRev_emxEnsureCapacity_real_T(in2_0, i);
  } else {
    i = in2_0->size[0];
    in2_0->size[0] = in1->size[0];
    ArmRev_emxEnsureCapacity_real_T(in2_0, i);
  }

  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in1->size[0] != 1);
  if (in1->size[0] == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = in1->size[0];
  }

  for (i = 0; i < loop_ub; i++) {
    in2_0->data[i] = in2->data[i * stride_0_0] + in1->data[i * stride_1_0];
  }

  i = in1->size[0];
  in1->size[0] = in2_0->size[0];
  ArmRev_emxEnsureCapacity_real_T(in1, i);
  loop_ub = in2_0->size[0];
  if (loop_ub - 1 >= 0) {
    memcpy(&in1->data[0], &in2_0->data[0], (uint32_T)loop_ub * sizeof(real_T));
  }

  ArmReverseKinema_emxFree_real_T(&in2_0);
}

static void ArmReverseKinematics_rand_ev(real_T varargin_1,
  emxArray_real_T_ArmReverseKin_T *r)
{
  real_T b_r;
  int32_T b_k;
  int32_T d;
  int32_T exitg1;
  uint32_T b_u[2];
  b_k = r->size[0];
  r->size[0] = (int32_T)varargin_1;
  ArmRev_emxEnsureCapacity_real_T(r, b_k);
  d = (int32_T)varargin_1 - 1;
  for (b_k = 0; b_k <= d; b_k++) {
    /* ========================= COPYRIGHT NOTICE ============================ */
    /*  This is a uniform (0,1) pseudorandom number generator based on:        */
    /*                                                                         */
    /*  A C-program for MT19937, with initialization improved 2002/1/26.       */
    /*  Coded by Takuji Nishimura and Makoto Matsumoto.                        */
    /*                                                                         */
    /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      */
    /*  All rights reserved.                                                   */
    /*                                                                         */
    /*  Redistribution and use in source and binary forms, with or without     */
    /*  modification, are permitted provided that the following conditions     */
    /*  are met:                                                               */
    /*                                                                         */
    /*    1. Redistributions of source code must retain the above copyright    */
    /*       notice, this list of conditions and the following disclaimer.     */
    /*                                                                         */
    /*    2. Redistributions in binary form must reproduce the above copyright */
    /*       notice, this list of conditions and the following disclaimer      */
    /*       in the documentation and/or other materials provided with the     */
    /*       distribution.                                                     */
    /*                                                                         */
    /*    3. The names of its contributors may not be used to endorse or       */
    /*       promote products derived from this software without specific      */
    /*       prior written permission.                                         */
    /*                                                                         */
    /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
    /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
    /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
    /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  */
    /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
    /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
    /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
    /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
    /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
    /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
    /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */
    /*                                                                         */
    /* =============================   END   ================================= */
    do {
      exitg1 = 0;
      ArmReve_genrand_uint32_vector_e(rtDW.state_i, b_u);
      b_r = ((real_T)(b_u[0] >> 5U) * 6.7108864E+7 + (real_T)(b_u[1] >> 6U)) *
        1.1102230246251565E-16;
      if (b_r == 0.0) {
        if (!ArmReverseKinema_is_valid_state(rtDW.state_i)) {
          rtDW.state_i[0] = 5489U;
          rtDW.state_i[624] = 624U;
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);

    r->data[b_k] = b_r;
  }
}

static void ArmReverseKine_binary_expand_op(emxArray_real_T_ArmReverseKin_T *in1,
  const emxArray_real_T_ArmReverseKin_T *in2, const
  emxArray_real_T_ArmReverseKin_T *in3)
{
  emxArray_real_T_ArmReverseKin_T *in2_0;
  real_T in2_tmp;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0_tmp;
  int32_T stride_1_0;
  int32_T stride_2_0;
  ArmReverseKinema_emxInit_real_T(&in2_0, 1);
  if (in2->size[0] == 1) {
    i = in1->size[0];
  } else {
    i = in2->size[0];
  }

  if (i == 1) {
    i = in3->size[0];
  } else if (in2->size[0] == 1) {
    i = in1->size[0];
  } else {
    i = in2->size[0];
  }

  if (i == 1) {
    i = in2_0->size[0];
    in2_0->size[0] = in2->size[0];
    ArmRev_emxEnsureCapacity_real_T(in2_0, i);
  } else {
    if (in2->size[0] == 1) {
      i = in1->size[0];
    } else {
      i = in2->size[0];
    }

    if (i == 1) {
      i = in2_0->size[0];
      in2_0->size[0] = in3->size[0];
      ArmRev_emxEnsureCapacity_real_T(in2_0, i);
    } else if (in2->size[0] == 1) {
      i = in2_0->size[0];
      in2_0->size[0] = in1->size[0];
      ArmRev_emxEnsureCapacity_real_T(in2_0, i);
    } else {
      i = in2_0->size[0];
      in2_0->size[0] = in2->size[0];
      ArmRev_emxEnsureCapacity_real_T(in2_0, i);
    }
  }

  stride_0_0_tmp = (in2->size[0] != 1);
  stride_1_0 = (in3->size[0] != 1);
  stride_2_0 = (in1->size[0] != 1);
  if (in2->size[0] == 1) {
    i = in1->size[0];
  } else {
    i = in2->size[0];
  }

  if (i == 1) {
    i = in3->size[0];
  } else if (in2->size[0] == 1) {
    i = in1->size[0];
  } else {
    i = in2->size[0];
  }

  if (i == 1) {
    loop_ub = in2->size[0];
  } else {
    if (in2->size[0] == 1) {
      i = in1->size[0];
    } else {
      i = in2->size[0];
    }

    if (i == 1) {
      loop_ub = in3->size[0];
    } else if (in2->size[0] == 1) {
      loop_ub = in1->size[0];
    } else {
      loop_ub = in2->size[0];
    }
  }

  for (i = 0; i < loop_ub; i++) {
    in2_tmp = in2->data[i * stride_0_0_tmp];
    in2_0->data[i] = (in1->data[i * stride_2_0] - in2_tmp) * in3->data[i *
      stride_1_0] + in2_tmp;
  }

  i = in1->size[0];
  in1->size[0] = in2_0->size[0];
  ArmRev_emxEnsureCapacity_real_T(in1, i);
  loop_ub = in2_0->size[0];
  if (loop_ub - 1 >= 0) {
    memcpy(&in1->data[0], &in2_0->data[0], (uint32_T)loop_ub * sizeof(real_T));
  }

  ArmReverseKinema_emxFree_real_T(&in2_0);
}

static void ArmRev_NLPSolverInterface_solve(h_robotics_core_internal_Erro_T *obj,
  const real_T seed[4], real_T xSol[4], real_T *solutionInfo_Iterations, real_T *
  solutionInfo_RRAttempts, real_T *solutionInfo_Error, real_T
  *solutionInfo_ExitFlag, char_T solutionInfo_Status_data[], int32_T
  solutionInfo_Status_size[2])
{
  __m128d tmp;
  __m128d tmp_0;
  __m128d tmp_1;
  c_rigidBodyJoint_ArmReverse_e_T *obj_1;
  emxArray_boolean_T_ArmReverse_T *b;
  emxArray_real_T_ArmReverseKin_T *lb;
  emxArray_real_T_ArmReverseKin_T *newseed;
  emxArray_real_T_ArmReverseKin_T *rn;
  emxArray_real_T_ArmReverseKin_T *ub;
  f_robotics_manip_internal_IKE_T *args;
  x_robotics_manip_internal_Rig_T *obj_0;
  real_T c_xSol[4];
  real_T ub_0[2];
  real_T err;
  real_T iter;
  real_T lb_0;
  real_T tol;
  c_robotics_core_internal_NLPS_T exitFlag;
  c_robotics_core_internal_NLPS_T exitFlagPrev;
  int32_T b_i;
  int32_T c;
  int32_T loop_ub;
  int32_T nx;
  int32_T scalarLB;
  int32_T vectorUB;
  boolean_T y;
  static const char_T tmp_2[14] = { 'b', 'e', 's', 't', ' ', 'a', 'v', 'a', 'i',
    'l', 'a', 'b', 'l', 'e' };

  static const char_T tmp_3[7] = { 's', 'u', 'c', 'c', 'e', 's', 's' };

  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T guard1;
  boolean_T guard2;
  boolean_T guard3;
  ArmReverseKinema_emxInit_real_T(&ub, 1);
  ArmReverseKinema_emxInit_real_T(&lb, 1);
  ArmReverseKinema_emxInit_real_T(&rn, 1);
  obj->MaxNumIterationInternal = obj->MaxNumIteration;
  obj->MaxTimeInternal = obj->MaxTime;
  obj->SeedInternal[0] = seed[0];
  obj->SeedInternal[1] = seed[1];
  obj->SeedInternal[2] = seed[2];
  obj->SeedInternal[3] = seed[3];
  tol = obj->SolutionTolerance;
  ArmReverseKinematics_tic(&obj->TimeObj.StartTime.tv_sec,
    &obj->TimeObj.StartTime.tv_nsec);
  ErrorDampedLevenbergMarquardt_s(obj, xSol, &exitFlag, &err, &iter);
  *solutionInfo_RRAttempts = 0.0;
  *solutionInfo_Iterations = iter;
  *solutionInfo_Error = err;
  exitFlagPrev = exitFlag;
  ArmReverseKinema_emxInit_real_T(&newseed, 1);
  ArmReverseKin_emxInit_boolean_T(&b, 1);
  exitg1 = false;
  while ((!exitg1) && (obj->RandomRestart && (err > tol))) {
    obj->MaxNumIterationInternal -= iter;
    err = ArmReverseKinematics_toc(obj->TimeObj.StartTime.tv_sec,
      obj->TimeObj.StartTime.tv_nsec);
    obj->MaxTimeInternal = obj->MaxTime - err;
    if (obj->MaxNumIterationInternal <= 0.0) {
      exitFlag = IterationLimitExceeded;
    }

    if ((exitFlag == IterationLimitExceeded) || (exitFlag == TimeLimitExceeded))
    {
      exitFlagPrev = exitFlag;
      exitg1 = true;
    } else {
      args = obj->ExtraArgs;
      obj_0 = args->Robot;
      loop_ub = newseed->size[0];
      newseed->size[0] = (int32_T)obj_0->PositionNumber;
      ArmRev_emxEnsureCapacity_real_T(newseed, loop_ub);
      loop_ub = (int32_T)obj_0->PositionNumber;
      if (loop_ub - 1 >= 0) {
        memset(&newseed->data[0], 0, (uint32_T)loop_ub * sizeof(real_T));
      }

      err = obj_0->NumBodies;
      c = (int32_T)err - 1;
      for (b_i = 0; b_i <= c; b_i++) {
        err = obj_0->PositionDoFMap[b_i];
        iter = obj_0->PositionDoFMap[b_i + 5];
        if (err <= iter) {
          obj_1 = obj_0->Bodies[b_i]->JointInternal;
          if ((int32_T)obj_1->PositionNumber == 0) {
            loop_ub = ub->size[0];
            ub->size[0] = 1;
            ArmRev_emxEnsureCapacity_real_T(ub, loop_ub);
            ub->data[0] = (rtNaN);
          } else {
            loop_ub = ub->size[0];
            ub->size[0] = obj_1->PositionLimitsInternal->size[0];
            ArmRev_emxEnsureCapacity_real_T(ub, loop_ub);
            loop_ub = obj_1->PositionLimitsInternal->size[0];
            for (nx = 0; nx < loop_ub; nx++) {
              ub->data[nx] = obj_1->PositionLimitsInternal->data[nx +
                obj_1->PositionLimitsInternal->size[0]];
            }

            loop_ub = lb->size[0];
            lb->size[0] = obj_1->PositionLimitsInternal->size[0];
            ArmRev_emxEnsureCapacity_real_T(lb, loop_ub);
            loop_ub = obj_1->PositionLimitsInternal->size[0];
            for (nx = 0; nx < loop_ub; nx++) {
              lb->data[nx] = obj_1->PositionLimitsInternal->data[nx];
            }

            loop_ub = b->size[0];
            b->size[0] = lb->size[0];
            Arm_emxEnsureCapacity_boolean_T(b, loop_ub);
            loop_ub = lb->size[0];
            for (nx = 0; nx < loop_ub; nx++) {
              lb_0 = lb->data[nx];
              b->data[nx] = ((!rtIsInf(lb_0)) && (!rtIsNaN(lb_0)));
            }

            y = true;
            loop_ub = 0;
            exitg2 = false;
            while ((!exitg2) && (loop_ub + 1 <= b->size[0])) {
              if (!b->data[loop_ub]) {
                y = false;
                exitg2 = true;
              } else {
                loop_ub++;
              }
            }

            guard1 = false;
            guard2 = false;
            guard3 = false;
            if (y) {
              loop_ub = b->size[0];
              b->size[0] = ub->size[0];
              Arm_emxEnsureCapacity_boolean_T(b, loop_ub);
              loop_ub = ub->size[0];
              for (nx = 0; nx < loop_ub; nx++) {
                lb_0 = ub->data[nx];
                b->data[nx] = ((!rtIsInf(lb_0)) && (!rtIsNaN(lb_0)));
              }

              loop_ub = 0;
              exitg2 = false;
              while ((!exitg2) && (loop_ub + 1 <= b->size[0])) {
                if (!b->data[loop_ub]) {
                  y = false;
                  exitg2 = true;
                } else {
                  loop_ub++;
                }
              }

              if (y) {
                ArmReverseKinematics_rand_ev(obj_1->PositionNumber, rn);
                if (ub->size[0] == 1) {
                  nx = lb->size[0];
                } else {
                  nx = ub->size[0];
                }

                if (rn->size[0] == 1) {
                  if (ub->size[0] == 1) {
                    loop_ub = lb->size[0];
                  } else {
                    loop_ub = ub->size[0];
                  }
                } else {
                  loop_ub = rn->size[0];
                }

                if ((ub->size[0] == lb->size[0]) && (nx == rn->size[0]) &&
                    (loop_ub == lb->size[0])) {
                  loop_ub = ub->size[0];
                  ub->size[0] = lb->size[0];
                  ArmRev_emxEnsureCapacity_real_T(ub, loop_ub);
                  loop_ub = lb->size[0];
                  scalarLB = (loop_ub / 2) << 1;
                  vectorUB = scalarLB - 2;
                  for (nx = 0; nx <= vectorUB; nx += 2) {
                    tmp_0 = _mm_loadu_pd(&ub->data[nx]);
                    tmp_1 = _mm_loadu_pd(&lb->data[nx]);
                    tmp = _mm_loadu_pd(&rn->data[nx]);
                    _mm_storeu_pd(&ub->data[nx], _mm_add_pd(_mm_mul_pd
                      (_mm_sub_pd(tmp_0, tmp_1), tmp), tmp_1));
                  }

                  for (nx = scalarLB; nx < loop_ub; nx++) {
                    lb_0 = lb->data[nx];
                    ub->data[nx] = (ub->data[nx] - lb_0) * rn->data[nx] + lb_0;
                  }
                } else {
                  ArmReverseKine_binary_expand_op(ub, lb, rn);
                }
              } else {
                guard3 = true;
              }
            } else {
              guard3 = true;
            }

            if (guard3) {
              loop_ub = b->size[0];
              b->size[0] = lb->size[0];
              Arm_emxEnsureCapacity_boolean_T(b, loop_ub);
              loop_ub = lb->size[0];
              for (nx = 0; nx < loop_ub; nx++) {
                lb_0 = lb->data[nx];
                b->data[nx] = ((!rtIsInf(lb_0)) && (!rtIsNaN(lb_0)));
              }

              y = true;
              loop_ub = 0;
              exitg2 = false;
              while ((!exitg2) && (loop_ub + 1 <= b->size[0])) {
                if (!b->data[loop_ub]) {
                  y = false;
                  exitg2 = true;
                } else {
                  loop_ub++;
                }
              }

              if (y) {
                loop_ub = b->size[0];
                b->size[0] = ub->size[0];
                Arm_emxEnsureCapacity_boolean_T(b, loop_ub);
                loop_ub = ub->size[0];
                for (nx = 0; nx < loop_ub; nx++) {
                  lb_0 = ub->data[nx];
                  b->data[nx] = (rtIsInf(lb_0) || rtIsNaN(lb_0));
                }

                if (ArmReverseKinematics_any(b)) {
                  ub_0[0] = lb->size[0];
                  ub_0[1] = 1.0;
                  ArmReverseKinematics_randn(ub_0, rn);
                  nx = rn->size[0] - 1;
                  loop_ub = ub->size[0];
                  ub->size[0] = rn->size[0];
                  ArmRev_emxEnsureCapacity_real_T(ub, loop_ub);
                  for (loop_ub = 0; loop_ub <= nx; loop_ub++) {
                    ub->data[loop_ub] = fabs(rn->data[loop_ub]);
                  }

                  if (lb->size[0] == ub->size[0]) {
                    loop_ub = ub->size[0];
                    ub->size[0] = lb->size[0];
                    ArmRev_emxEnsureCapacity_real_T(ub, loop_ub);
                    loop_ub = lb->size[0];
                    scalarLB = (loop_ub / 2) << 1;
                    vectorUB = scalarLB - 2;
                    for (nx = 0; nx <= vectorUB; nx += 2) {
                      tmp_0 = _mm_loadu_pd(&lb->data[nx]);
                      tmp_1 = _mm_loadu_pd(&ub->data[nx]);
                      _mm_storeu_pd(&ub->data[nx], _mm_add_pd(tmp_0, tmp_1));
                    }

                    for (nx = scalarLB; nx < loop_ub; nx++) {
                      ub->data[nx] += lb->data[nx];
                    }
                  } else {
                    ArmReverseKinematics_plus(ub, lb);
                  }
                } else {
                  guard2 = true;
                }
              } else {
                guard2 = true;
              }
            }

            if (guard2) {
              loop_ub = b->size[0];
              b->size[0] = lb->size[0];
              Arm_emxEnsureCapacity_boolean_T(b, loop_ub);
              loop_ub = lb->size[0];
              for (nx = 0; nx < loop_ub; nx++) {
                lb_0 = lb->data[nx];
                b->data[nx] = (rtIsInf(lb_0) || rtIsNaN(lb_0));
              }

              if (ArmReverseKinematics_any(b)) {
                loop_ub = b->size[0];
                b->size[0] = ub->size[0];
                Arm_emxEnsureCapacity_boolean_T(b, loop_ub);
                loop_ub = ub->size[0];
                for (nx = 0; nx < loop_ub; nx++) {
                  lb_0 = ub->data[nx];
                  b->data[nx] = ((!rtIsInf(lb_0)) && (!rtIsNaN(lb_0)));
                }

                y = true;
                loop_ub = 0;
                exitg2 = false;
                while ((!exitg2) && (loop_ub + 1 <= b->size[0])) {
                  if (!b->data[loop_ub]) {
                    y = false;
                    exitg2 = true;
                  } else {
                    loop_ub++;
                  }
                }

                if (y) {
                  ub_0[0] = ub->size[0];
                  ub_0[1] = 1.0;
                  ArmReverseKinematics_randn(ub_0, rn);
                  nx = rn->size[0] - 1;
                  loop_ub = lb->size[0];
                  lb->size[0] = rn->size[0];
                  ArmRev_emxEnsureCapacity_real_T(lb, loop_ub);
                  for (loop_ub = 0; loop_ub <= nx; loop_ub++) {
                    lb->data[loop_ub] = fabs(rn->data[loop_ub]);
                  }

                  if (ub->size[0] == lb->size[0]) {
                    loop_ub = ub->size[0];
                    scalarLB = (loop_ub / 2) << 1;
                    vectorUB = scalarLB - 2;
                    for (nx = 0; nx <= vectorUB; nx += 2) {
                      tmp_0 = _mm_loadu_pd(&ub->data[nx]);
                      tmp_1 = _mm_loadu_pd(&lb->data[nx]);
                      _mm_storeu_pd(&ub->data[nx], _mm_sub_pd(tmp_0, tmp_1));
                    }

                    for (nx = scalarLB; nx < loop_ub; nx++) {
                      ub->data[nx] -= lb->data[nx];
                    }
                  } else {
                    ArmReverseKinematics_minus(ub, lb);
                  }
                } else {
                  guard1 = true;
                }
              } else {
                guard1 = true;
              }
            }

            if (guard1) {
              ub_0[0] = ub->size[0];
              ub_0[1] = 1.0;
              ArmReverseKinematics_randn(ub_0, ub);
            }
          }

          if (err > iter) {
            loop_ub = 0;
            nx = 0;
          } else {
            loop_ub = (int32_T)err - 1;
            nx = (int32_T)iter;
          }

          scalarLB = nx - loop_ub;
          for (nx = 0; nx < scalarLB; nx++) {
            newseed->data[loop_ub + nx] = ub->data[nx];
          }
        }
      }

      obj->SeedInternal[0] = newseed->data[0];
      obj->SeedInternal[1] = newseed->data[1];
      obj->SeedInternal[2] = newseed->data[2];
      obj->SeedInternal[3] = newseed->data[3];
      ErrorDampedLevenbergMarquardt_s(obj, c_xSol, &exitFlag, &err, &iter);
      if (err < *solutionInfo_Error) {
        xSol[0] = c_xSol[0];
        xSol[1] = c_xSol[1];
        xSol[2] = c_xSol[2];
        xSol[3] = c_xSol[3];
        *solutionInfo_Error = err;
        exitFlagPrev = exitFlag;
      }

      (*solutionInfo_RRAttempts)++;
      *solutionInfo_Iterations += iter;
    }
  }

  ArmReverseKin_emxFree_boolean_T(&b);
  ArmReverseKinema_emxFree_real_T(&newseed);
  *solutionInfo_ExitFlag = exitFlagPrev;
  if (*solutionInfo_Error < tol) {
    solutionInfo_Status_size[0] = 1;
    solutionInfo_Status_size[1] = 7;
    for (nx = 0; nx < 7; nx++) {
      solutionInfo_Status_data[nx] = tmp_3[nx];
    }
  } else {
    solutionInfo_Status_size[0] = 1;
    solutionInfo_Status_size[1] = 14;
    for (nx = 0; nx < 14; nx++) {
      solutionInfo_Status_data[nx] = tmp_2[nx];
    }
  }

  ArmReverseKinema_emxFree_real_T(&rn);
  ArmReverseKinema_emxFree_real_T(&lb);
  ArmReverseKinema_emxFree_real_T(&ub);
}

static void ArmReverseKinem_emxInit_int32_T(emxArray_int32_T_ArmReverseKi_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_int32_T_ArmReverseKi_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_int32_T_ArmReverseKi_T *)malloc(sizeof
    (emxArray_int32_T_ArmReverseKi_T));
  emxArray = *pEmxArray;
  emxArray->data = (int32_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * (uint32_T)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void ArmRe_emxEnsureCapacity_int32_T(emxArray_int32_T_ArmReverseKi_T
  *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = malloc((uint32_T)i * sizeof(int32_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(int32_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (int32_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void ArmReverseKinem_emxFree_int32_T(emxArray_int32_T_ArmReverseKi_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_int32_T_ArmReverseKi_T *)NULL) {
    if (((*pEmxArray)->data != (int32_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_int32_T_ArmReverseKi_T *)NULL;
  }
}

static void ArmReverseKine_emxInit_uint32_T(emxArray_uint32_T_ArmReverseK_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_uint32_T_ArmReverseK_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_uint32_T_ArmReverseK_T *)malloc(sizeof
    (emxArray_uint32_T_ArmReverseK_T));
  emxArray = *pEmxArray;
  emxArray->data = (uint32_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * (uint32_T)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void ArmR_emxEnsureCapacity_uint32_T(emxArray_uint32_T_ArmReverseK_T
  *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = malloc((uint32_T)i * sizeof(uint32_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(uint32_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (uint32_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void ArmReverseKine_emxFree_uint32_T(emxArray_uint32_T_ArmReverseK_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_uint32_T_ArmReverseK_T *)NULL) {
    if (((*pEmxArray)->data != (uint32_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_uint32_T_ArmReverseK_T *)NULL;
  }
}

static void ArmRe_inverseKinematics_solve_e(b_inverseKinematics_ArmRevers_T *obj,
  real_T initialGuess[4], real_T *solutionInfo_Iterations, real_T
  *solutionInfo_NumRandomRestarts, real_T *solutionInfo_PoseErrorNorm, real_T
  *solutionInfo_ExitFlag, char_T solutionInfo_Status_data[], int32_T
  solutionInfo_Status_size[2])
{
  emxArray_int32_T_ArmReverseKi_T *tmp;
  emxArray_real_T_ArmReverseKin_T *bodyIndices;
  emxArray_real_T_ArmReverseKin_T *e;
  emxArray_real_T_ArmReverseKin_T *limits;
  emxArray_real_T_ArmReverseKin_T *positionIndices;
  emxArray_uint32_T_ArmReverseK_T *y;
  v_robotics_manip_internal_Rig_T *body;
  x_robotics_manip_internal_Rig_T *obj_0;
  real_T qvSolRaw[4];
  real_T apnd;
  real_T cdiff;
  real_T i;
  real_T limits_0;
  real_T limits_1;
  real_T ndbl;
  real_T numPositions_tmp;
  int32_T indicesUpperBoundViolation_data[4];
  int32_T tmp_data[4];
  int32_T b_k;
  int32_T c;
  int32_T i_0;
  int32_T indicesUpperBoundViolation;
  int32_T indicesUpperBoundViolation_size;
  int32_T nm1d2;
  boolean_T lbOK[4];
  boolean_T ubOK[4];
  boolean_T ubOK_0[4];
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T y_0;
  obj_0 = obj->RigidBodyTreeInternal;
  ArmReverseKinema_emxInit_real_T(&limits, 2);
  RigidBodyTree_get_JointPosition(obj_0, limits);
  if (limits->size[0] == 4) {
    ubOK[0] = (initialGuess[0] <= limits->data[limits->size[0]] +
               4.4408920985006262E-16);
    ubOK[1] = (initialGuess[1] <= limits->data[1 + limits->size[0]] +
               4.4408920985006262E-16);
    ubOK[2] = (initialGuess[2] <= limits->data[2 + limits->size[0]] +
               4.4408920985006262E-16);
    ubOK[3] = (initialGuess[3] <= limits->data[3 + limits->size[0]] +
               4.4408920985006262E-16);
  } else {
    ArmReverseKi_binary_expand_op_4(ubOK, initialGuess, limits);
  }

  if (limits->size[0] == 4) {
    lbOK[0] = (initialGuess[0] >= limits->data[0] - 4.4408920985006262E-16);
    lbOK[1] = (initialGuess[1] >= limits->data[1] - 4.4408920985006262E-16);
    lbOK[2] = (initialGuess[2] >= limits->data[2] - 4.4408920985006262E-16);
    lbOK[3] = (initialGuess[3] >= limits->data[3] - 4.4408920985006262E-16);
  } else {
    ArmReverseKi_binary_expand_op_3(lbOK, initialGuess, limits);
  }

  y_0 = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 4)) {
    if (!ubOK[b_k]) {
      y_0 = false;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  guard1 = false;
  if (y_0) {
    b_k = 0;
    exitg1 = false;
    while ((!exitg1) && (b_k < 4)) {
      if (!lbOK[b_k]) {
        y_0 = false;
        exitg1 = true;
      } else {
        b_k++;
      }
    }

    if (y_0) {
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    ubOK_0[0] = !ubOK[0];
    ubOK_0[1] = !ubOK[1];
    ubOK_0[2] = !ubOK[2];
    ubOK_0[3] = !ubOK[3];
    ArmReverseKinematics_eml_find(ubOK_0, tmp_data, &b_k);
    if (b_k - 1 >= 0) {
      memcpy(&indicesUpperBoundViolation_data[0], &tmp_data[0], (uint32_T)b_k *
             sizeof(int32_T));
    }

    for (i_0 = 0; i_0 < b_k; i_0++) {
      indicesUpperBoundViolation = indicesUpperBoundViolation_data[i_0];
      initialGuess[indicesUpperBoundViolation - 1] = limits->data
        [(indicesUpperBoundViolation + limits->size[0]) - 1];
    }

    ubOK[0] = !lbOK[0];
    ubOK[1] = !lbOK[1];
    ubOK[2] = !lbOK[2];
    ubOK[3] = !lbOK[3];
    ArmReverseKinematics_eml_find(ubOK, tmp_data, &b_k);
    if (b_k - 1 >= 0) {
      memcpy(&indicesUpperBoundViolation_data[0], &tmp_data[0], (uint32_T)b_k *
             sizeof(int32_T));
    }

    for (i_0 = 0; i_0 < b_k; i_0++) {
      indicesUpperBoundViolation = indicesUpperBoundViolation_data[i_0];
      initialGuess[indicesUpperBoundViolation - 1] = limits->
        data[indicesUpperBoundViolation - 1];
    }
  }

  ArmRev_NLPSolverInterface_solve(obj->Solver, initialGuess, qvSolRaw,
    solutionInfo_Iterations, solutionInfo_NumRandomRestarts,
    solutionInfo_PoseErrorNorm, solutionInfo_ExitFlag, solutionInfo_Status_data,
    solutionInfo_Status_size);
  obj_0 = obj->RigidBodyTreeInternal;
  i = obj->Solver->ExtraArgs->BodyIndex;
  ArmReverseKinema_emxInit_real_T(&bodyIndices, 1);
  nm1d2 = bodyIndices->size[0];
  bodyIndices->size[0] = (int32_T)obj_0->NumBodies;
  ArmRev_emxEnsureCapacity_real_T(bodyIndices, nm1d2);
  b_k = (int32_T)obj_0->NumBodies;
  if (b_k - 1 >= 0) {
    memset(&bodyIndices->data[0], 0, (uint32_T)b_k * sizeof(real_T));
  }

  if (i == 0.0) {
    nm1d2 = bodyIndices->size[0];
    bodyIndices->size[0] = 1;
    ArmRev_emxEnsureCapacity_real_T(bodyIndices, nm1d2);
    bodyIndices->data[0] = 0.0;
  } else {
    body = obj_0->Bodies[(int32_T)i - 1];
    i = 1.0;
    while (body->ParentIndex != 0.0) {
      bodyIndices->data[(int32_T)i - 1] = body->Index;
      body = obj_0->Bodies[(int32_T)body->ParentIndex - 1];
      i++;
    }

    if (i - 1.0 < 1.0) {
      indicesUpperBoundViolation_size = 0;
    } else {
      indicesUpperBoundViolation_size = (int32_T)(i - 1.0);
    }

    nm1d2 = bodyIndices->size[0];
    bodyIndices->size[0] = indicesUpperBoundViolation_size + 2;
    ArmRev_emxEnsureCapacity_real_T(bodyIndices, nm1d2);
    bodyIndices->data[indicesUpperBoundViolation_size] = body->Index;
    bodyIndices->data[indicesUpperBoundViolation_size + 1] = 0.0;
  }

  obj_0 = obj->RigidBodyTreeInternal;
  indicesUpperBoundViolation = bodyIndices->size[0] - 1;
  b_k = 0;
  for (i_0 = 0; i_0 <= indicesUpperBoundViolation; i_0++) {
    if (bodyIndices->data[i_0] != 0.0) {
      b_k++;
    }
  }

  ArmReverseKinem_emxInit_int32_T(&tmp, 1);
  nm1d2 = tmp->size[0];
  tmp->size[0] = b_k;
  ArmRe_emxEnsureCapacity_int32_T(tmp, nm1d2);
  b_k = 0;
  for (i_0 = 0; i_0 <= indicesUpperBoundViolation; i_0++) {
    if (bodyIndices->data[i_0] != 0.0) {
      tmp->data[b_k] = i_0;
      b_k++;
    }
  }

  nm1d2 = limits->size[0] * limits->size[1];
  limits->size[0] = tmp->size[0];
  limits->size[1] = 2;
  ArmRev_emxEnsureCapacity_real_T(limits, nm1d2);
  b_k = tmp->size[0];
  for (i_0 = 0; i_0 < 2; i_0++) {
    for (indicesUpperBoundViolation = 0; indicesUpperBoundViolation < b_k;
         indicesUpperBoundViolation++) {
      limits->data[indicesUpperBoundViolation + limits->size[0] * i_0] =
        obj_0->PositionDoFMap[(5 * i_0 + (int32_T)bodyIndices->data[tmp->
        data[indicesUpperBoundViolation]]) - 1];
    }
  }

  ArmReverseKinem_emxFree_int32_T(&tmp);
  ArmReverseKinema_emxFree_real_T(&bodyIndices);
  ArmReverseKinema_emxInit_real_T(&positionIndices, 2);
  nm1d2 = positionIndices->size[0] * positionIndices->size[1];
  positionIndices->size[0] = 1;
  positionIndices->size[1] = (int32_T)obj_0->PositionNumber;
  ArmRev_emxEnsureCapacity_real_T(positionIndices, nm1d2);
  b_k = (int32_T)obj_0->PositionNumber;
  if (b_k - 1 >= 0) {
    memset(&positionIndices->data[0], 0, (uint32_T)b_k * sizeof(real_T));
  }

  i = 0.0;
  indicesUpperBoundViolation_size = limits->size[0] - 1;
  ArmReverseKinema_emxInit_real_T(&e, 2);
  ArmReverseKine_emxInit_uint32_T(&y, 2);
  for (indicesUpperBoundViolation = 0; indicesUpperBoundViolation <=
       indicesUpperBoundViolation_size; indicesUpperBoundViolation++) {
    numPositions_tmp = limits->data[indicesUpperBoundViolation + limits->size[0]]
      - limits->data[indicesUpperBoundViolation];
    if (numPositions_tmp + 1.0 > 0.0) {
      if (numPositions_tmp + 1.0 < 1.0) {
        y->size[0] = 1;
        y->size[1] = 0;
      } else {
        nm1d2 = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = (int32_T)((numPositions_tmp + 1.0) - 1.0) + 1;
        ArmR_emxEnsureCapacity_uint32_T(y, nm1d2);
        b_k = (int32_T)((numPositions_tmp + 1.0) - 1.0);
        for (i_0 = 0; i_0 <= b_k; i_0++) {
          y->data[i_0] = (uint32_T)i_0 + 1U;
        }
      }

      limits_1 = limits->data[indicesUpperBoundViolation];
      limits_0 = limits->data[indicesUpperBoundViolation + limits->size[0]];
      if (rtIsNaN(limits_1) || rtIsNaN(limits_0)) {
        nm1d2 = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = 1;
        ArmRev_emxEnsureCapacity_real_T(e, nm1d2);
        e->data[0] = (rtNaN);
      } else if (limits_0 < limits_1) {
        e->size[0] = 1;
        e->size[1] = 0;
      } else if ((rtIsInf(limits_1) || rtIsInf(limits_0)) && (limits_1 ==
                  limits_0)) {
        nm1d2 = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = 1;
        ArmRev_emxEnsureCapacity_real_T(e, nm1d2);
        e->data[0] = (rtNaN);
      } else if (floor(limits_1) == limits_1) {
        nm1d2 = e->size[0] * e->size[1];
        e->size[0] = 1;
        b_k = (int32_T)numPositions_tmp;
        e->size[1] = (int32_T)numPositions_tmp + 1;
        ArmRev_emxEnsureCapacity_real_T(e, nm1d2);
        for (i_0 = 0; i_0 <= b_k; i_0++) {
          e->data[i_0] = limits_1 + (real_T)i_0;
        }
      } else {
        ndbl = floor(numPositions_tmp + 0.5);
        apnd = limits_1 + ndbl;
        cdiff = apnd - limits_0;
        if (fabs(cdiff) < 4.4408920985006262E-16 * fmax(fabs(limits_1), fabs
             (limits_0))) {
          ndbl++;
          apnd = limits_0;
        } else if (cdiff > 0.0) {
          apnd = (ndbl - 1.0) + limits_1;
        } else {
          ndbl++;
        }

        if (ndbl >= 0.0) {
          i_0 = (int32_T)ndbl;
        } else {
          i_0 = 0;
        }

        nm1d2 = e->size[0] * e->size[1];
        e->size[0] = 1;
        e->size[1] = i_0;
        ArmRev_emxEnsureCapacity_real_T(e, nm1d2);
        if (i_0 > 0) {
          e->data[0] = limits_1;
          if (i_0 > 1) {
            e->data[i_0 - 1] = apnd;
            nm1d2 = (int32_T)((uint32_T)(i_0 - 1) >> 1);
            c = nm1d2 - 2;
            for (b_k = 0; b_k <= c; b_k++) {
              e->data[b_k + 1] = ((real_T)b_k + 1.0) + limits_1;
              e->data[(i_0 - b_k) - 2] = apnd - ((real_T)b_k + 1.0);
            }

            if (nm1d2 << 1 == i_0 - 1) {
              e->data[nm1d2] = (limits->data[indicesUpperBoundViolation] + apnd)
                / 2.0;
            } else {
              e->data[nm1d2] = limits->data[indicesUpperBoundViolation] +
                (real_T)nm1d2;
              e->data[nm1d2 + 1] = apnd - (real_T)nm1d2;
            }
          }
        }
      }

      b_k = e->size[1];
      for (i_0 = 0; i_0 < b_k; i_0++) {
        positionIndices->data[(int32_T)(i + (real_T)y->data[i_0]) - 1] = e->
          data[i_0];
      }

      i += numPositions_tmp + 1.0;
    }
  }

  ArmReverseKine_emxFree_uint32_T(&y);
  ArmReverseKinema_emxFree_real_T(&e);
  ArmReverseKinema_emxFree_real_T(&limits);
  if (i < 1.0) {
    b_k = -1;
  } else {
    b_k = (int32_T)i - 1;
  }

  for (i_0 = 0; i_0 <= b_k; i_0++) {
    i = positionIndices->data[i_0];
    initialGuess[(int32_T)i - 1] = qvSolRaw[(int32_T)i - 1];
  }

  ArmReverseKinema_emxFree_real_T(&positionIndices);
}

static void ArmR_inverseKinematics_stepImpl(b_inverseKinematics_ArmRevers_T *obj,
  const real_T tform[16], const real_T weights[6], const real_T initialGuess[4],
  real_T QSol[4])
{
  emxArray_char_T_ArmReverseKin_T *bname;
  f_robotics_manip_internal_IKE_T *args;
  v_robotics_manip_internal_Rig_T *obj_1;
  x_robotics_manip_internal_Rig_T *obj_0;
  real_T weightMatrix[36];
  real_T b;
  real_T bidx;
  real_T expl_temp;
  real_T expl_temp_0;
  int32_T b_i;
  int32_T b_kstr;
  int32_T loop_ub;
  char_T expl_temp_data[14];
  char_T b_0[5];
  boolean_T b_bool;
  static const char_T tmp[5] = { 'B', 'o', 'd', 'y', '5' };

  int32_T expl_temp_size[2];
  int32_T exitg1;
  boolean_T exitg2;
  obj_0 = obj->RigidBodyTreeInternal;
  bidx = -1.0;
  ArmReverseKinema_emxInit_char_T(&bname, 2);
  b_kstr = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj_0->Base.NameInternal->size[1];
  ArmRev_emxEnsureCapacity_char_T(bname, b_kstr);
  loop_ub = obj_0->Base.NameInternal->size[1];
  for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
    bname->data[b_kstr] = obj_0->Base.NameInternal->data[b_kstr];
  }

  for (b_kstr = 0; b_kstr < 5; b_kstr++) {
    b_0[b_kstr] = tmp[b_kstr];
  }

  b_bool = false;
  if (bname->size[1] != 5) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 5) {
        if (bname->data[b_kstr - 1] != b_0[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (b_bool) {
    bidx = 0.0;
  } else {
    b = obj_0->NumBodies;
    b_i = 0;
    exitg2 = false;
    while ((!exitg2) && (b_i <= (int32_T)b - 1)) {
      obj_1 = obj_0->Bodies[b_i];
      b_kstr = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = obj_1->NameInternal->size[1];
      ArmRev_emxEnsureCapacity_char_T(bname, b_kstr);
      loop_ub = obj_1->NameInternal->size[1];
      for (b_kstr = 0; b_kstr < loop_ub; b_kstr++) {
        bname->data[b_kstr] = obj_1->NameInternal->data[b_kstr];
      }

      for (b_kstr = 0; b_kstr < 5; b_kstr++) {
        b_0[b_kstr] = tmp[b_kstr];
      }

      if (bname->size[1] != 5) {
      } else {
        b_kstr = 1;
        do {
          exitg1 = 0;
          if (b_kstr - 1 < 5) {
            if (bname->data[b_kstr - 1] != b_0[b_kstr - 1]) {
              exitg1 = 1;
            } else {
              b_kstr++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (b_bool) {
        bidx = (real_T)b_i + 1.0;
        exitg2 = true;
      } else {
        b_i++;
      }
    }
  }

  ArmReverseKinema_emxFree_char_T(&bname);
  memset(&weightMatrix[0], 0, 36U * sizeof(real_T));
  for (b_kstr = 0; b_kstr < 6; b_kstr++) {
    weightMatrix[b_kstr + 6 * b_kstr] = weights[b_kstr];
  }

  args = obj->Solver->ExtraArgs;
  for (b_kstr = 0; b_kstr < 36; b_kstr++) {
    args->WeightMatrix[b_kstr] = weightMatrix[b_kstr];
  }

  args->BodyIndex = bidx;
  args->KinematicModel = obj->RigidBodyTreeKinematicModel;
  for (b_kstr = 0; b_kstr < 16; b_kstr++) {
    args->Tform[b_kstr] = tform[b_kstr];
  }

  QSol[0] = initialGuess[0];
  QSol[1] = initialGuess[1];
  QSol[2] = initialGuess[2];
  QSol[3] = initialGuess[3];
  ArmRe_inverseKinematics_solve_e(obj, QSol, &bidx, &b, &expl_temp, &expl_temp_0,
    expl_temp_data, expl_temp_size);
}

static void ArmReverseK_emxInit_f_cell_wrap(emxArray_f_cell_wrap_ArmRever_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_f_cell_wrap_ArmRever_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_f_cell_wrap_ArmRever_T *)malloc(sizeof
    (emxArray_f_cell_wrap_ArmRever_T));
  emxArray = *pEmxArray;
  emxArray->data = (f_cell_wrap_ArmReverseKinemat_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * (uint32_T)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void A_emxEnsureCapacity_f_cell_wrap(emxArray_f_cell_wrap_ArmRever_T
  *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = malloc((uint32_T)i * sizeof(f_cell_wrap_ArmReverseKinemat_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(f_cell_wrap_ArmReverseKinemat_T)
             * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (f_cell_wrap_ArmReverseKinemat_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void Ar_rigidBodyJoint_get_JointAxis(const
  c_rigidBodyJoint_ArmReverseKi_T *obj, real_T ax[3])
{
  int32_T b_kstr;
  char_T b_0[9];
  char_T b[8];
  boolean_T b_bool;
  static const char_T tmp[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char_T tmp_0[9] = { 'p', 'r', 'i', 's', 'm', 'a', 't', 'i', 'c' };

  int32_T exitg1;
  boolean_T guard1;
  for (b_kstr = 0; b_kstr < 8; b_kstr++) {
    b[b_kstr] = tmp[b_kstr];
  }

  b_bool = false;
  if (obj->Type->size[1] != 8) {
  } else {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 8) {
        if (obj->Type->data[b_kstr - 1] != b[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  guard1 = false;
  if (b_bool) {
    guard1 = true;
  } else {
    for (b_kstr = 0; b_kstr < 9; b_kstr++) {
      b_0[b_kstr] = tmp_0[b_kstr];
    }

    if (obj->Type->size[1] != 9) {
    } else {
      b_kstr = 1;
      do {
        exitg1 = 0;
        if (b_kstr - 1 < 9) {
          if (obj->Type->data[b_kstr - 1] != b_0[b_kstr - 1]) {
            exitg1 = 1;
          } else {
            b_kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      guard1 = true;
    } else {
      ax[0] = (rtNaN);
      ax[1] = (rtNaN);
      ax[2] = (rtNaN);
    }
  }

  if (guard1) {
    ax[0] = obj->JointAxisInternal[0];
    ax[1] = obj->JointAxisInternal[1];
    ax[2] = obj->JointAxisInternal[2];
  }
}

static void ArmReverseK_emxFree_f_cell_wrap(emxArray_f_cell_wrap_ArmRever_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_f_cell_wrap_ArmRever_T *)NULL) {
    if (((*pEmxArray)->data != (f_cell_wrap_ArmReverseKinemat_T *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_f_cell_wrap_ArmRever_T *)NULL;
  }
}

static void emxFreeStruct_v_robotics_manip_(v_robotics_manip_internal_Rig_T
  *pStruct)
{
  ArmReverseKinema_emxFree_char_T(&pStruct->NameInternal);
}

static void emxFreeStruct_n_robotics_manip_(n_robotics_manip_internal_Col_T
  *pStruct)
{
  emxFree_m_robotics_manip_intern(&pStruct->CollisionGeometries);
}

static void emxFreeMatrix_n_robotics_manip_(n_robotics_manip_internal_Col_T
  pMatrix[11])
{
  int32_T i;
  for (i = 0; i < 11; i++) {
    emxFreeStruct_n_robotics_manip_(&pMatrix[i]);
  }
}

static void emxFreeStruct_c_rigidBodyJoint(c_rigidBodyJoint_ArmReverse_e_T
  *pStruct)
{
  ArmReverseKinema_emxFree_char_T(&pStruct->Type);
  ArmReverseKinema_emxFree_real_T(&pStruct->MotionSubspace);
  ArmReverseKinema_emxFree_char_T(&pStruct->NameInternal);
  ArmReverseKinema_emxFree_real_T(&pStruct->PositionLimitsInternal);
  ArmReverseKinema_emxFree_real_T(&pStruct->HomePositionInternal);
}

static void emxFreeMatrix_c_rigidBodyJoint(c_rigidBodyJoint_ArmReverse_e_T
  pMatrix[11])
{
  int32_T i;
  for (i = 0; i < 11; i++) {
    emxFreeStruct_c_rigidBodyJoint(&pMatrix[i]);
  }
}

static void emxFreeMatrix_v_robotics_manip_(v_robotics_manip_internal_Rig_T
  pMatrix[10])
{
  int32_T i;
  for (i = 0; i < 10; i++) {
    emxFreeStruct_v_robotics_manip_(&pMatrix[i]);
  }
}

static void emxFreeStruct_w_robotics_manip_(w_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_v_robotics_manip_(&pStruct->Base);
  emxFreeMatrix_n_robotics_manip_(pStruct->_pobj0);
  emxFreeMatrix_c_rigidBodyJoint(pStruct->_pobj1);
  emxFreeMatrix_v_robotics_manip_(pStruct->_pobj2);
}

static void emxFreeStruct_f_robotics_manip_(f_robotics_manip_internal_IKE_T
  *pStruct)
{
  ArmReverseKinema_emxFree_real_T(&pStruct->Limits);
  ArmReverseKinema_emxFree_real_T(&pStruct->ErrTemp);
  ArmReverseKinema_emxFree_real_T(&pStruct->GradTemp);
}

static void emxFreeMatrix_c_rigidBodyJoint1(c_rigidBodyJoint_ArmReverse_e_T
  pMatrix[10])
{
  int32_T i;
  for (i = 0; i < 10; i++) {
    emxFreeStruct_c_rigidBodyJoint(&pMatrix[i]);
  }
}

static void emxFreeMatrix_v_robotics_mani_e(v_robotics_manip_internal_Rig_T
  pMatrix[5])
{
  int32_T i;
  for (i = 0; i < 5; i++) {
    emxFreeStruct_v_robotics_manip_(&pMatrix[i]);
  }
}

static void emxFreeMatrix_n_robotics_mani_e(n_robotics_manip_internal_Col_T
  pMatrix[6])
{
  int32_T i;
  for (i = 0; i < 6; i++) {
    emxFreeStruct_n_robotics_manip_(&pMatrix[i]);
  }
}

static void emxFreeMatrix_c_rigidBodyJoint2(c_rigidBodyJoint_ArmReverse_e_T
  pMatrix[6])
{
  int32_T i;
  for (i = 0; i < 6; i++) {
    emxFreeStruct_c_rigidBodyJoint(&pMatrix[i]);
  }
}

static void emxFreeStruct_x_robotics_manip_(x_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_v_robotics_manip_(&pStruct->Base);
  emxFreeMatrix_v_robotics_mani_e(pStruct->_pobj0);
  emxFreeMatrix_n_robotics_mani_e(pStruct->_pobj1);
  emxFreeMatrix_c_rigidBodyJoint2(pStruct->_pobj2);
}

static void emxFreeStruct_b_inverseKinemati(b_inverseKinematics_ArmRevers_T
  *pStruct)
{
  ArmReverseKinema_emxFree_real_T(&pStruct->Limits);
  emxFreeStruct_f_robotics_manip_(&pStruct->_pobj0);
  emxFreeMatrix_c_rigidBodyJoint1(pStruct->_pobj1);
  emxFreeMatrix_v_robotics_mani_e(pStruct->_pobj2);
  emxFreeMatrix_n_robotics_manip_(pStruct->_pobj3);
  emxFreeStruct_x_robotics_manip_(&pStruct->_pobj4);
}

static void emxFreeStruct_robotics_slmanip_(robotics_slmanip_internal_b_e_T
  *pStruct)
{
  emxFreeStruct_w_robotics_manip_(&pStruct->TreeInternal);
  emxFreeStruct_b_inverseKinemati(&pStruct->IKInternal);
}

static void emxFreeStruct_c_rigidBodyJoint1(c_rigidBodyJoint_ArmReverseKi_T
  *pStruct)
{
  ArmReverseKinema_emxFree_char_T(&pStruct->Type);
}

static void emxFreeStruct_i_robotics_manip_(i_robotics_manip_internal_Col_T
  *pStruct)
{
  emxFree_h_robotics_manip_intern(&pStruct->CollisionGeometries);
}

static void emxFreeStruct_k_robotics_manip_(k_robotics_manip_internal_Rig_T
  *pStruct)
{
  ArmReverseKinema_emxFree_char_T(&pStruct->NameInternal);
  emxFreeStruct_c_rigidBodyJoint1(&pStruct->JointInternal);
  emxFreeStruct_i_robotics_manip_(&pStruct->CollisionsInternal);
}

static void emxFreeMatrix_k_robotics_manip_(k_robotics_manip_internal_Rig_T
  pMatrix[10])
{
  int32_T i;
  for (i = 0; i < 10; i++) {
    emxFreeStruct_k_robotics_manip_(&pMatrix[i]);
  }
}

static void emxFreeStruct_l_robotics_manip_(l_robotics_manip_internal_Rig_T
  *pStruct)
{
  emxFreeStruct_k_robotics_manip_(&pStruct->Base);
  emxFreeMatrix_k_robotics_manip_(pStruct->_pobj0);
}

static void emxFreeStruct_robotics_slmani_e(robotics_slmanip_internal_blo_T
  *pStruct)
{
  emxFreeStruct_l_robotics_manip_(&pStruct->TreeInternal);
}

/* Start for root system: '<Root>' */
void MdlStart(void)
{
  NeModelParameters modelParameters;
  NeModelParameters modelParameters_0;
  NeslSimulationData *tmp_1;
  NeslSimulator *tmp;
  NeuDiagnosticManager *diagnosticManager;
  NeuDiagnosticTree *diagnosticTree;
  NeuDiagnosticTree *diagnosticTree_0;
  char *msg;
  char *msg_0;
  real_T tmp_2;
  int32_T i;
  boolean_T tmp_0;
  boolean_T val;
  static const uint32_T tmp_3[625] = { 5489U, 1301868182U, 2938499221U,
    2950281878U, 1875628136U, 751856242U, 944701696U, 2243192071U, 694061057U,
    219885934U, 2066767472U, 3182869408U, 485472502U, 2336857883U, 1071588843U,
    3418470598U, 951210697U, 3693558366U, 2923482051U, 1793174584U, 2982310801U,
    1586906132U, 1951078751U, 1808158765U, 1733897588U, 431328322U, 4202539044U,
    530658942U, 1714810322U, 3025256284U, 3342585396U, 1937033938U, 2640572511U,
    1654299090U, 3692403553U, 4233871309U, 3497650794U, 862629010U, 2943236032U,
    2426458545U, 1603307207U, 1133453895U, 3099196360U, 2208657629U, 2747653927U,
    931059398U, 761573964U, 3157853227U, 785880413U, 730313442U, 124945756U,
    2937117055U, 3295982469U, 1724353043U, 3021675344U, 3884886417U, 4010150098U,
    4056961966U, 699635835U, 2681338818U, 1339167484U, 720757518U, 2800161476U,
    2376097373U, 1532957371U, 3902664099U, 1238982754U, 3725394514U, 3449176889U,
    3570962471U, 4287636090U, 4087307012U, 3603343627U, 202242161U, 2995682783U,
    1620962684U, 3704723357U, 371613603U, 2814834333U, 2111005706U, 624778151U,
    2094172212U, 4284947003U, 1211977835U, 991917094U, 1570449747U, 2962370480U,
    1259410321U, 170182696U, 146300961U, 2836829791U, 619452428U, 2723670296U,
    1881399711U, 1161269684U, 1675188680U, 4132175277U, 780088327U, 3409462821U,
    1036518241U, 1834958505U, 3048448173U, 161811569U, 618488316U, 44795092U,
    3918322701U, 1924681712U, 3239478144U, 383254043U, 4042306580U, 2146983041U,
    3992780527U, 3518029708U, 3545545436U, 3901231469U, 1896136409U, 2028528556U,
    2339662006U, 501326714U, 2060962201U, 2502746480U, 561575027U, 581893337U,
    3393774360U, 1778912547U, 3626131687U, 2175155826U, 319853231U, 986875531U,
    819755096U, 2915734330U, 2688355739U, 3482074849U, 2736559U, 2296975761U,
    1029741190U, 2876812646U, 690154749U, 579200347U, 4027461746U, 1285330465U,
    2701024045U, 4117700889U, 759495121U, 3332270341U, 2313004527U, 2277067795U,
    4131855432U, 2722057515U, 1264804546U, 3848622725U, 2211267957U, 4100593547U,
    959123777U, 2130745407U, 3194437393U, 486673947U, 1377371204U, 17472727U,
    352317554U, 3955548058U, 159652094U, 1232063192U, 3835177280U, 49423123U,
    3083993636U, 733092U, 2120519771U, 2573409834U, 1112952433U, 3239502554U,
    761045320U, 1087580692U, 2540165110U, 641058802U, 1792435497U, 2261799288U,
    1579184083U, 627146892U, 2165744623U, 2200142389U, 2167590760U, 2381418376U,
    1793358889U, 3081659520U, 1663384067U, 2009658756U, 2689600308U, 739136266U,
    2304581039U, 3529067263U, 591360555U, 525209271U, 3131882996U, 294230224U,
    2076220115U, 3113580446U, 1245621585U, 1386885462U, 3203270426U, 123512128U,
    12350217U, 354956375U, 4282398238U, 3356876605U, 3888857667U, 157639694U,
    2616064085U, 1563068963U, 2762125883U, 4045394511U, 4180452559U, 3294769488U,
    1684529556U, 1002945951U, 3181438866U, 22506664U, 691783457U, 2685221343U,
    171579916U, 3878728600U, 2475806724U, 2030324028U, 3331164912U, 1708711359U,
    1970023127U, 2859691344U, 2588476477U, 2748146879U, 136111222U, 2967685492U,
    909517429U, 2835297809U, 3206906216U, 3186870716U, 341264097U, 2542035121U,
    3353277068U, 548223577U, 3170936588U, 1678403446U, 297435620U, 2337555430U,
    466603495U, 1132321815U, 1208589219U, 696392160U, 894244439U, 2562678859U,
    470224582U, 3306867480U, 201364898U, 2075966438U, 1767227936U, 2929737987U,
    3674877796U, 2654196643U, 3692734598U, 3528895099U, 2796780123U, 3048728353U,
    842329300U, 191554730U, 2922459673U, 3489020079U, 3979110629U, 1022523848U,
    2202932467U, 3583655201U, 3565113719U, 587085778U, 4176046313U, 3013713762U,
    950944241U, 396426791U, 3784844662U, 3477431613U, 3594592395U, 2782043838U,
    3392093507U, 3106564952U, 2829419931U, 1358665591U, 2206918825U, 3170783123U,
    31522386U, 2988194168U, 1782249537U, 1105080928U, 843500134U, 1225290080U,
    1521001832U, 3605886097U, 2802786495U, 2728923319U, 3996284304U, 903417639U,
    1171249804U, 1020374987U, 2824535874U, 423621996U, 1988534473U, 2493544470U,
    1008604435U, 1756003503U, 1488867287U, 1386808992U, 732088248U, 1780630732U,
    2482101014U, 976561178U, 1543448953U, 2602866064U, 2021139923U, 1952599828U,
    2360242564U, 2117959962U, 2753061860U, 2388623612U, 4138193781U, 2962920654U,
    2284970429U, 766920861U, 3457264692U, 2879611383U, 815055854U, 2332929068U,
    1254853997U, 3740375268U, 3799380844U, 4091048725U, 2006331129U, 1982546212U,
    686850534U, 1907447564U, 2682801776U, 2780821066U, 998290361U, 1342433871U,
    4195430425U, 607905174U, 3902331779U, 2454067926U, 1708133115U, 1170874362U,
    2008609376U, 3260320415U, 2211196135U, 433538229U, 2728786374U, 2189520818U,
    262554063U, 1182318347U, 3710237267U, 1221022450U, 715966018U, 2417068910U,
    2591870721U, 2870691989U, 3418190842U, 4238214053U, 1540704231U, 1575580968U,
    2095917976U, 4078310857U, 2313532447U, 2110690783U, 4056346629U, 4061784526U,
    1123218514U, 551538993U, 597148360U, 4120175196U, 3581618160U, 3181170517U,
    422862282U, 3227524138U, 1713114790U, 662317149U, 1230418732U, 928171837U,
    1324564878U, 1928816105U, 1786535431U, 2878099422U, 3290185549U, 539474248U,
    1657512683U, 552370646U, 1671741683U, 3655312128U, 1552739510U, 2605208763U,
    1441755014U, 181878989U, 3124053868U, 1447103986U, 3183906156U, 1728556020U,
    3502241336U, 3055466967U, 1013272474U, 818402132U, 1715099063U, 2900113506U,
    397254517U, 4194863039U, 1009068739U, 232864647U, 2540223708U, 2608288560U,
    2415367765U, 478404847U, 3455100648U, 3182600021U, 2115988978U, 434269567U,
    4117179324U, 3461774077U, 887256537U, 3545801025U, 286388911U, 3451742129U,
    1981164769U, 786667016U, 3310123729U, 3097811076U, 2224235657U, 2959658883U,
    3370969234U, 2514770915U, 3345656436U, 2677010851U, 2206236470U, 271648054U,
    2342188545U, 4292848611U, 3646533909U, 3754009956U, 3803931226U, 4160647125U,
    1477814055U, 4043852216U, 1876372354U, 3133294443U, 3871104810U, 3177020907U,
    2074304428U, 3479393793U, 759562891U, 164128153U, 1839069216U, 2114162633U,
    3989947309U, 3611054956U, 1333547922U, 835429831U, 494987340U, 171987910U,
    1252001001U, 370809172U, 3508925425U, 2535703112U, 1276855041U, 1922855120U,
    835673414U, 3030664304U, 613287117U, 171219893U, 3423096126U, 3376881639U,
    2287770315U, 1658692645U, 1262815245U, 3957234326U, 1168096164U, 2968737525U,
    2655813712U, 2132313144U, 3976047964U, 326516571U, 353088456U, 3679188938U,
    3205649712U, 2654036126U, 1249024881U, 880166166U, 691800469U, 2229503665U,
    1673458056U, 4032208375U, 1851778863U, 2563757330U, 376742205U, 1794655231U,
    340247333U, 1505873033U, 396524441U, 879666767U, 3335579166U, 3260764261U,
    3335999539U, 506221798U, 4214658741U, 975887814U, 2080536343U, 3360539560U,
    571586418U, 138896374U, 4234352651U, 2737620262U, 3928362291U, 1516365296U,
    38056726U, 3599462320U, 3585007266U, 3850961033U, 471667319U, 1536883193U,
    2310166751U, 1861637689U, 2530999841U, 4139843801U, 2710569485U, 827578615U,
    2012334720U, 2907369459U, 3029312804U, 2820112398U, 1965028045U, 35518606U,
    2478379033U, 643747771U, 1924139484U, 4123405127U, 3811735531U, 3429660832U,
    3285177704U, 1948416081U, 1311525291U, 1183517742U, 1739192232U, 3979815115U,
    2567840007U, 4116821529U, 213304419U, 4125718577U, 1473064925U, 2442436592U,
    1893310111U, 4195361916U, 3747569474U, 828465101U, 2991227658U, 750582866U,
    1205170309U, 1409813056U, 678418130U, 1171531016U, 3821236156U, 354504587U,
    4202874632U, 3882511497U, 1893248677U, 1903078632U, 26340130U, 2069166240U,
    3657122492U, 3725758099U, 831344905U, 811453383U, 3447711422U, 2434543565U,
    4166886888U, 3358210805U, 4142984013U, 2988152326U, 3527824853U, 982082992U,
    2809155763U, 190157081U, 3340214818U, 2365432395U, 2548636180U, 2894533366U,
    3474657421U, 2372634704U, 2845748389U, 43024175U, 2774226648U, 1987702864U,
    3186502468U, 453610222U, 4204736567U, 1392892630U, 2471323686U, 2470534280U,
    3541393095U, 4269885866U, 3909911300U, 759132955U, 1482612480U, 667715263U,
    1795580598U, 2337923983U, 3390586366U, 581426223U, 1515718634U, 476374295U,
    705213300U, 363062054U, 2084697697U, 2407503428U, 2292957699U, 2426213835U,
    2199989172U, 1987356470U, 4026755612U, 2147252133U, 270400031U, 1367820199U,
    2369854699U, 2844269403U, 79981964U, 624U };

  /* Start for FromWorkspace: '<S8>/fromWS_Signal 1' */
  {
    FWksInfo *fromwksInfo;
    if ((fromwksInfo = (FWksInfo *) calloc(1, sizeof(FWksInfo))) == (NULL)) {
      ssSetErrorStatus(rtS,
                       "from workspace STRING(Name) memory allocation error");
    } else {
      fromwksInfo->origWorkspaceVarName =
        "Simulink.signaleditorblock.SimulationData.getData('QXJtUmV2ZXJzZUtpbmVtYXRpY3MvU2lnbmFsIEVkaXRvcg==','1')";
      fromwksInfo->origDataTypeId = 0;
      fromwksInfo->origIsComplex = 0;
      fromwksInfo->origWidth = 1;
      fromwksInfo->origElSize = sizeof(real_T);
      fromwksInfo->data = (void *)rtP.fromWS_Signal1_Data0;
      fromwksInfo->nDataPoints = 5;
      fromwksInfo->time = (double *)rtP.fromWS_Signal1_Time0;
      rtDW.fromWS_Signal1_PWORK.TimePtr = fromwksInfo->time;
      rtDW.fromWS_Signal1_PWORK.DataPtr = fromwksInfo->data;
      rtDW.fromWS_Signal1_PWORK.RSimInfoPtr = fromwksInfo;
    }

    rtDW.fromWS_Signal1_IWORK.PrevIndex = 0;
  }

  /* Start for FromWorkspace: '<S8>/From Workspace' */
  {
    FWksInfo *fromwksInfo;
    if ((fromwksInfo = (FWksInfo *) calloc(1, sizeof(FWksInfo))) == (NULL)) {
      ssSetErrorStatus(rtS,
                       "from workspace STRING(Name) memory allocation error");
    } else {
      fromwksInfo->origWorkspaceVarName =
        "Simulink.signaleditorblock.SimulationData.getData('QXJtUmV2ZXJzZUtpbmVtYXRpY3MvU2lnbmFsIEVkaXRvcg==','2')";
      fromwksInfo->origDataTypeId = 0;
      fromwksInfo->origIsComplex = 0;
      fromwksInfo->origWidth = 1;
      fromwksInfo->origElSize = sizeof(real_T);
      fromwksInfo->data = (void *)rtP.FromWorkspace_Data0;
      fromwksInfo->nDataPoints = 5;
      fromwksInfo->time = (double *)rtP.FromWorkspace_Time0;
      rtDW.FromWorkspace_PWORK.TimePtr = fromwksInfo->time;
      rtDW.FromWorkspace_PWORK.DataPtr = fromwksInfo->data;
      rtDW.FromWorkspace_PWORK.RSimInfoPtr = fromwksInfo;
    }

    rtDW.FromWorkspace_IWORK.PrevIndex = 0;
  }

  /* Start for FromWorkspace: '<S8>/From Workspace1' */
  {
    FWksInfo *fromwksInfo;
    if ((fromwksInfo = (FWksInfo *) calloc(1, sizeof(FWksInfo))) == (NULL)) {
      ssSetErrorStatus(rtS,
                       "from workspace STRING(Name) memory allocation error");
    } else {
      fromwksInfo->origWorkspaceVarName =
        "Simulink.signaleditorblock.SimulationData.getData('QXJtUmV2ZXJzZUtpbmVtYXRpY3MvU2lnbmFsIEVkaXRvcg==','3')";
      fromwksInfo->origDataTypeId = 0;
      fromwksInfo->origIsComplex = 0;
      fromwksInfo->origWidth = 1;
      fromwksInfo->origElSize = sizeof(real_T);
      fromwksInfo->data = (void *)rtP.FromWorkspace1_Data0;
      fromwksInfo->nDataPoints = 5;
      fromwksInfo->time = (double *)rtP.FromWorkspace1_Time0;
      rtDW.FromWorkspace1_PWORK.TimePtr = fromwksInfo->time;
      rtDW.FromWorkspace1_PWORK.DataPtr = fromwksInfo->data;
      rtDW.FromWorkspace1_PWORK.RSimInfoPtr = fromwksInfo;
    }

    rtDW.FromWorkspace1_IWORK.PrevIndex = 0;
  }

  /* Start for MATLABSystem: '<S7>/Coordinate Transformation Conversion' */
  rtDW.objisempty_p = true;
  rtDW.obj_ln.isInitialized = 1;
  emxInitStruct_robotics_slmanip_(&rtDW.obj);

  /* Start for MATLABSystem: '<S25>/MATLAB System' */
  for (i = 0; i < 6; i++) {
    rtDW.obj.IKInternal._pobj4._pobj1[i].matlabCodegenIsDeleted = true;
  }

  for (i = 0; i < 11; i++) {
    rtDW.obj.IKInternal._pobj3[i].matlabCodegenIsDeleted = true;
  }

  for (i = 0; i < 11; i++) {
    rtDW.obj.TreeInternal._pobj0[i].matlabCodegenIsDeleted = true;
  }

  for (i = 0; i < 5; i++) {
    rtDW.obj.IKInternal._pobj4._pobj0[i].matlabCodegenIsDeleted = true;
  }

  rtDW.obj.IKInternal._pobj4.Base.matlabCodegenIsDeleted = true;
  for (i = 0; i < 5; i++) {
    rtDW.obj.IKInternal._pobj2[i].matlabCodegenIsDeleted = true;
  }

  for (i = 0; i < 10; i++) {
    rtDW.obj.TreeInternal._pobj2[i].matlabCodegenIsDeleted = true;
  }

  rtDW.obj.TreeInternal.Base.matlabCodegenIsDeleted = true;
  rtDW.obj.TreeInternal.matlabCodegenIsDeleted = true;
  rtDW.obj.IKInternal._pobj4.matlabCodegenIsDeleted = true;
  rtDW.obj.IKInternal._pobj0.matlabCodegenIsDeleted = true;
  rtDW.obj.IKInternal._pobj5.matlabCodegenIsDeleted = true;
  rtDW.obj.IKInternal.matlabCodegenIsDeleted = true;
  rtDW.method_d = 7U;
  rtDW.freq_not_empty = true;
  rtDW.state = 1144108930U;
  rtDW.state_not_empty = true;
  rtDW.state_l[0] = 362436069U;
  rtDW.state_l[1] = 521288629U;
  rtDW.state_not_empty_j = true;
  memcpy(&rtDW.state_i[0], &tmp_3[0], 625U * sizeof(uint32_T));
  rtDW.method_not_empty_m = true;
  rtDW.method = 0U;
  rtDW.state_not_empty_d = true;
  rtDW.state_k[0] = 362436069U;
  rtDW.state_k[1] = 521288629U;
  rtDW.state_not_empty_k = true;
  rtDW.obj.isInitialized = 0;
  rtDW.obj.matlabCodegenIsDeleted = false;
  rtDW.objisempty = true;
  ArmReverseKi_SystemCore_setup_e(&rtDW.obj);

  /* End of Start for MATLABSystem: '<S25>/MATLAB System' */

  /* Start for SimscapeExecutionBlock: '<S19>/OUTPUT_1_0' */
  tmp = nesl_lease_simulator("ArmReverseKinematics/Arm/Solver Configuration_1",
    1, 0);
  rtDW.OUTPUT_1_0_Simulator = (void *)tmp;
  tmp_0 = pointer_is_null(rtDW.OUTPUT_1_0_Simulator);
  if (tmp_0) {
    ArmReverseKinematics_aa99d991_1_gateway();
    tmp = nesl_lease_simulator("ArmReverseKinematics/Arm/Solver Configuration_1",
      1, 0);
    rtDW.OUTPUT_1_0_Simulator = (void *)tmp;
  }

  tmp_1 = nesl_create_simulation_data();
  rtDW.OUTPUT_1_0_SimData = (void *)tmp_1;
  diagnosticManager = rtw_create_diagnostics();
  rtDW.OUTPUT_1_0_DiagMgr = (void *)diagnosticManager;
  modelParameters.mSolverType = NE_SOLVER_TYPE_DAE;
  modelParameters.mSolverAbsTol = 0.001;
  modelParameters.mSolverRelTol = 0.001;
  modelParameters.mSolverModifyAbsTol = NE_MODIFY_ABS_TOL_MAYBE;
  modelParameters.mStartTime = 0.0;
  modelParameters.mLoadInitialState = false;
  modelParameters.mUseSimState = false;
  modelParameters.mLinTrimCompile = false;
  modelParameters.mLoggingMode = SSC_LOGGING_OFF;
  modelParameters.mRTWModifiedTimeStamp = 6.37666902E+8;
  modelParameters.mZcDisabled = false;
  modelParameters.mUseModelRefSolver = false;
  modelParameters.mTargetFPGAHIL = false;
  tmp_2 = 0.001;
  modelParameters.mSolverTolerance = tmp_2;
  tmp_2 = 0.0;
  modelParameters.mFixedStepSize = tmp_2;
  tmp_0 = true;
  modelParameters.mVariableStepSolver = tmp_0;
  tmp_0 = false;
  modelParameters.mIsUsingODEN = tmp_0;
  tmp_0 = slIsRapidAcceleratorSimulating();
  val = ssGetGlobalInitialStatesAvailable(rtS);
  if (tmp_0) {
    val = (val && ssIsFirstInitCond(rtS));
  }

  modelParameters.mLoadInitialState = val;
  modelParameters.mZcDisabled = false;
  diagnosticManager = (NeuDiagnosticManager *)rtDW.OUTPUT_1_0_DiagMgr;
  diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
  i = nesl_initialize_simulator((NeslSimulator *)rtDW.OUTPUT_1_0_Simulator,
    &modelParameters, diagnosticManager);
  if (i != 0) {
    tmp_0 = error_buffer_is_empty(ssGetErrorStatus(rtS));
    if (tmp_0) {
      msg = rtw_diagnostics_msg(diagnosticTree);
      ssSetErrorStatus(rtS, msg);
    }
  }

  /* End of Start for SimscapeExecutionBlock: '<S19>/OUTPUT_1_0' */
  emxInitStruct_robotics_slmani_e(&rtDW.obj_l);

  /* Start for MATLABSystem: '<S20>/MATLAB System' */
  for (i = 0; i < 10; i++) {
    rtDW.obj_l.TreeInternal._pobj0[i].CollisionsInternal.matlabCodegenIsDeleted =
      true;
  }

  rtDW.obj_l.TreeInternal.Base.CollisionsInternal.matlabCodegenIsDeleted = true;
  for (i = 0; i < 10; i++) {
    rtDW.obj_l.TreeInternal._pobj0[i].matlabCodegenIsDeleted = true;
  }

  rtDW.obj_l.TreeInternal.Base.matlabCodegenIsDeleted = true;
  rtDW.obj_l.TreeInternal.matlabCodegenIsDeleted = true;
  rtDW.method_e = 7U;
  rtDW.state_not_empty_kp = true;
  rtDW.state_h = 1144108930U;
  rtDW.state_not_empty_p = true;
  rtDW.state_n[0] = 362436069U;
  rtDW.state_n[1] = 521288629U;
  rtDW.method_not_empty_k = true;
  memcpy(&rtDW.state_j[0], &tmp_3[0], 625U * sizeof(uint32_T));
  rtDW.state_not_empty_h = true;
  rtDW.obj_l.isInitialized = 0;
  rtDW.obj_l.matlabCodegenIsDeleted = false;
  rtDW.objisempty_n = true;
  ArmReverseKine_SystemCore_setup(&rtDW.obj_l);

  /* End of Start for MATLABSystem: '<S20>/MATLAB System' */

  /* Start for MATLABSystem: '<S2>/Coordinate Transformation Conversion' */
  rtDW.objisempty_j = true;
  rtDW.obj_p.isInitialized = 1;

  /* Start for SimscapeExecutionBlock: '<S19>/STATE_1' */
  tmp = nesl_lease_simulator("ArmReverseKinematics/Arm/Solver Configuration_1",
    0, 0);
  rtDW.STATE_1_Simulator = (void *)tmp;
  tmp_0 = pointer_is_null(rtDW.STATE_1_Simulator);
  if (tmp_0) {
    ArmReverseKinematics_aa99d991_1_gateway();
    tmp = nesl_lease_simulator("ArmReverseKinematics/Arm/Solver Configuration_1",
      0, 0);
    rtDW.STATE_1_Simulator = (void *)tmp;
  }

  tmp_1 = nesl_create_simulation_data();
  rtDW.STATE_1_SimData = (void *)tmp_1;
  diagnosticManager = rtw_create_diagnostics();
  rtDW.STATE_1_DiagMgr = (void *)diagnosticManager;
  modelParameters_0.mSolverType = NE_SOLVER_TYPE_DAE;
  modelParameters_0.mSolverAbsTol = 0.001;
  modelParameters_0.mSolverRelTol = 0.001;
  modelParameters_0.mSolverModifyAbsTol = NE_MODIFY_ABS_TOL_MAYBE;
  modelParameters_0.mStartTime = 0.0;
  modelParameters_0.mLoadInitialState = false;
  modelParameters_0.mUseSimState = false;
  modelParameters_0.mLinTrimCompile = false;
  modelParameters_0.mLoggingMode = SSC_LOGGING_OFF;
  modelParameters_0.mRTWModifiedTimeStamp = 6.37666902E+8;
  modelParameters_0.mZcDisabled = false;
  modelParameters_0.mUseModelRefSolver = false;
  modelParameters_0.mTargetFPGAHIL = false;
  tmp_2 = 0.001;
  modelParameters_0.mSolverTolerance = tmp_2;
  tmp_2 = 0.0;
  modelParameters_0.mFixedStepSize = tmp_2;
  tmp_0 = true;
  modelParameters_0.mVariableStepSolver = tmp_0;
  tmp_0 = false;
  modelParameters_0.mIsUsingODEN = tmp_0;
  tmp_0 = slIsRapidAcceleratorSimulating();
  val = ssGetGlobalInitialStatesAvailable(rtS);
  if (tmp_0) {
    val = (val && ssIsFirstInitCond(rtS));
  }

  modelParameters_0.mLoadInitialState = val;
  modelParameters_0.mZcDisabled = false;
  diagnosticManager = (NeuDiagnosticManager *)rtDW.STATE_1_DiagMgr;
  diagnosticTree_0 = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
  i = nesl_initialize_simulator((NeslSimulator *)rtDW.STATE_1_Simulator,
    &modelParameters_0, diagnosticManager);
  if (i != 0) {
    tmp_0 = error_buffer_is_empty(ssGetErrorStatus(rtS));
    if (tmp_0) {
      msg_0 = rtw_diagnostics_msg(diagnosticTree_0);
      ssSetErrorStatus(rtS, msg_0);
    }
  }

  /* End of Start for SimscapeExecutionBlock: '<S19>/STATE_1' */
}

/* Outputs for root system: '<Root>' */
void MdlOutputs(int_T tid)
{
  /* local block i/o variables */
  real_T rtb_Xaxis;
  real_T rtb_Yaxis;
  real_T rtb_Zaxis;
  __m128d tmp_5;
  __m128d tmp_6;
  __m128d tmp_7;
  __m128d tmp_8;
  NeslSimulationData *simulationData;
  NeuDiagnosticManager *diagnosticManager;
  NeuDiagnosticTree *diagnosticTree;
  NeuDiagnosticTree *diagnosticTree_0;
  b_inverseKinematics_ArmRevers_T *obj;
  char *msg;
  char *msg_0;
  emxArray_char_T_ArmReverseKin_T *switch_expression;
  emxArray_f_cell_wrap_ArmRever_T *Ttree;
  emxArray_int8_T_ArmReverseKin_T *b_gradTmp;
  emxArray_real_T_ArmReverseKin_T *tmp_4;
  f_cell_wrap_ArmReverseKinemat_T expl_temp;
  k_robotics_manip_internal_Rig_T *body;
  l_robotics_manip_internal_Rig_T *obj_0;
  real_T R_1[16];
  real_T T1[16];
  real_T T1_0[16];
  real_T out[16];
  real_T rtb_MATLABSystem[16];
  real_T tmp_0[16];
  real_T tmp_2[16];
  real_T R[9];
  real_T R_0[9];
  real_T tempR[9];
  real_T u1[6];
  real_T result_data[4];
  real_T rtb_OUTPUT_1_0[4];
  real_T rtb_TmpSignalConversionAtMATLAB[4];
  real_T v[3];
  real_T cth;
  real_T k;
  real_T n;
  real_T tempR_tmp;
  real_T tempR_tmp_0;
  real_T time;
  real_T time_0;
  real_T time_1;
  real_T time_2;
  real_T time_tmp;
  real_T time_tmp_0;
  int32_T b_jcol;
  int32_T b_k;
  int32_T i;
  int32_T iacol;
  int32_T loop_ub;
  int_T tmp_1[5];
  int_T tmp_3[5];
  char_T a_0[8];
  char_T a[5];
  char_T b[4];
  boolean_T first_output;
  boolean_T result;
  boolean_T tmp;
  static const int8_T tmp_9[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    1 };

  static const char_T tmp_a[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char_T tmp_b[5] = { 'B', 'o', 'd', 'y', '5' };

  static const char_T tmp_c[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  real_T cth_tmp;
  real_T cth_tmp_0;
  int32_T exitg1;
  int32_T ibcol_tmp;
  boolean_T exitg2;

  /* FromWorkspace: '<S8>/fromWS_Signal 1' */
  {
    real_T *pDataValues = (real_T *) rtDW.fromWS_Signal1_PWORK.DataPtr;
    real_T *pTimeValues = (real_T *) rtDW.fromWS_Signal1_PWORK.TimePtr;
    int_T currTimeIndex = rtDW.fromWS_Signal1_IWORK.PrevIndex;
    real_T t = ssGetTaskTime(rtS,0);
    if (t > pTimeValues[4]) {
      rtb_Xaxis = 0.0;
    } else {
      int numPoints, lastPoint;
      FWksInfo *fromwksInfo = (FWksInfo *) rtDW.fromWS_Signal1_PWORK.RSimInfoPtr;
      numPoints = fromwksInfo->nDataPoints;
      lastPoint = numPoints - 1;

      /* Get index */
      if (t <= pTimeValues[0]) {
        currTimeIndex = 0;
      } else if (t >= pTimeValues[lastPoint]) {
        currTimeIndex = lastPoint-1;
      } else {
        if (t < pTimeValues[currTimeIndex]) {
          while (t < pTimeValues[currTimeIndex]) {
            currTimeIndex--;
          }
        } else {
          while (t >= pTimeValues[currTimeIndex + 1]) {
            currTimeIndex++;
          }
        }
      }

      rtDW.fromWS_Signal1_IWORK.PrevIndex = currTimeIndex;

      /* Post output */
      {
        real_T t1 = pTimeValues[currTimeIndex];
        real_T t2 = pTimeValues[currTimeIndex + 1];
        if (t1 == t2) {
          if (t < t1) {
            rtb_Xaxis = pDataValues[currTimeIndex];
          } else {
            rtb_Xaxis = pDataValues[currTimeIndex + 1];
          }
        } else {
          real_T f1 = (t2 - t) / (t2 - t1);
          real_T f2 = 1.0 - f1;
          real_T d1;
          real_T d2;
          int_T TimeIndex = currTimeIndex;
          d1 = pDataValues[TimeIndex];
          d2 = pDataValues[TimeIndex + 1];
          rtb_Xaxis = (real_T) rtInterpolate(d1, d2, f1, f2);
          pDataValues += numPoints;
        }
      }
    }
  }

  /* FromWorkspace: '<S8>/From Workspace' */
  {
    real_T *pDataValues = (real_T *) rtDW.FromWorkspace_PWORK.DataPtr;
    real_T *pTimeValues = (real_T *) rtDW.FromWorkspace_PWORK.TimePtr;
    int_T currTimeIndex = rtDW.FromWorkspace_IWORK.PrevIndex;
    real_T t = ssGetTaskTime(rtS,0);
    if (t > pTimeValues[4]) {
      rtb_Yaxis = 0.0;
    } else {
      int numPoints, lastPoint;
      FWksInfo *fromwksInfo = (FWksInfo *) rtDW.FromWorkspace_PWORK.RSimInfoPtr;
      numPoints = fromwksInfo->nDataPoints;
      lastPoint = numPoints - 1;

      /* Get index */
      if (t <= pTimeValues[0]) {
        currTimeIndex = 0;
      } else if (t >= pTimeValues[lastPoint]) {
        currTimeIndex = lastPoint-1;
      } else {
        if (t < pTimeValues[currTimeIndex]) {
          while (t < pTimeValues[currTimeIndex]) {
            currTimeIndex--;
          }
        } else {
          while (t >= pTimeValues[currTimeIndex + 1]) {
            currTimeIndex++;
          }
        }
      }

      rtDW.FromWorkspace_IWORK.PrevIndex = currTimeIndex;

      /* Post output */
      {
        real_T t1 = pTimeValues[currTimeIndex];
        real_T t2 = pTimeValues[currTimeIndex + 1];
        if (t1 == t2) {
          if (t < t1) {
            rtb_Yaxis = pDataValues[currTimeIndex];
          } else {
            rtb_Yaxis = pDataValues[currTimeIndex + 1];
          }
        } else {
          real_T f1 = (t2 - t) / (t2 - t1);
          real_T f2 = 1.0 - f1;
          real_T d1;
          real_T d2;
          int_T TimeIndex = currTimeIndex;
          d1 = pDataValues[TimeIndex];
          d2 = pDataValues[TimeIndex + 1];
          rtb_Yaxis = (real_T) rtInterpolate(d1, d2, f1, f2);
          pDataValues += numPoints;
        }
      }
    }
  }

  /* FromWorkspace: '<S8>/From Workspace1' */
  {
    real_T *pDataValues = (real_T *) rtDW.FromWorkspace1_PWORK.DataPtr;
    real_T *pTimeValues = (real_T *) rtDW.FromWorkspace1_PWORK.TimePtr;
    int_T currTimeIndex = rtDW.FromWorkspace1_IWORK.PrevIndex;
    real_T t = ssGetTaskTime(rtS,0);
    if (t > pTimeValues[4]) {
      rtb_Zaxis = 0.0;
    } else {
      int numPoints, lastPoint;
      FWksInfo *fromwksInfo = (FWksInfo *) rtDW.FromWorkspace1_PWORK.RSimInfoPtr;
      numPoints = fromwksInfo->nDataPoints;
      lastPoint = numPoints - 1;

      /* Get index */
      if (t <= pTimeValues[0]) {
        currTimeIndex = 0;
      } else if (t >= pTimeValues[lastPoint]) {
        currTimeIndex = lastPoint-1;
      } else {
        if (t < pTimeValues[currTimeIndex]) {
          while (t < pTimeValues[currTimeIndex]) {
            currTimeIndex--;
          }
        } else {
          while (t >= pTimeValues[currTimeIndex + 1]) {
            currTimeIndex++;
          }
        }
      }

      rtDW.FromWorkspace1_IWORK.PrevIndex = currTimeIndex;

      /* Post output */
      {
        real_T t1 = pTimeValues[currTimeIndex];
        real_T t2 = pTimeValues[currTimeIndex + 1];
        if (t1 == t2) {
          if (t < t1) {
            rtb_Zaxis = pDataValues[currTimeIndex];
          } else {
            rtb_Zaxis = pDataValues[currTimeIndex + 1];
          }
        } else {
          real_T f1 = (t2 - t) / (t2 - t1);
          real_T f2 = 1.0 - f1;
          real_T d1;
          real_T d2;
          int_T TimeIndex = currTimeIndex;
          d1 = pDataValues[TimeIndex];
          d2 = pDataValues[TimeIndex + 1];
          rtb_Zaxis = (real_T) rtInterpolate(d1, d2, f1, f2);
          pDataValues += numPoints;
        }
      }
    }
  }

  /* SignalConversion generated from: '<S7>/Coordinate Transformation Conversion' */
  rtB.TmpSignalConversionAtCoordinate[0] = rtb_Xaxis;
  rtB.TmpSignalConversionAtCoordinate[1] = rtb_Yaxis;
  rtB.TmpSignalConversionAtCoordinate[2] = rtb_Zaxis;

  /* MATLABSystem: '<S7>/Coordinate Transformation Conversion' */
  memset(&T1[0], 0, sizeof(real_T) << 4U);
  T1[0] = 1.0;
  T1[5] = 1.0;
  T1[10] = 1.0;
  T1[15] = 1.0;
  for (b_jcol = 0; b_jcol < 4; b_jcol++) {
    i = (b_jcol << 2) - 1;
    out[i + 1] = T1[i + 1];
    out[i + 2] = T1[i + 2];
    out[i + 3] = T1[i + 3];
    out[i + 4] = T1[i + 4];
  }

  out[12] = rtB.TmpSignalConversionAtCoordinate[0];
  out[13] = rtB.TmpSignalConversionAtCoordinate[1];
  out[14] = rtB.TmpSignalConversionAtCoordinate[2];
  if (ssIsSampleHit(rtS, 1, 0)) {
    for (i = 0; i < 6; i++) {
      /* Constant: '<S7>/weight' */
      rtB.weight[i] = rtP.weight_Value[i];
    }

    /* Constant: '<S7>/init' */
    rtB.init[0] = rtP.init_Value[0];
    rtB.init[1] = rtP.init_Value[1];
    rtB.init[2] = rtP.init_Value[2];
    rtB.init[3] = rtP.init_Value[3];
  }

  /* MATLABSystem: '<S25>/MATLAB System' incorporates:
   *  Constant: '<S7>/weight'
   *  MATLABSystem: '<S7>/Coordinate Transformation Conversion'
   */
  for (i = 0; i < 6; i++) {
    u1[i] = rtB.weight[i];
  }

  rtb_TmpSignalConversionAtMATLAB[0] = rtB.init[0];
  rtb_TmpSignalConversionAtMATLAB[1] = rtB.init[1];
  rtb_TmpSignalConversionAtMATLAB[2] = rtB.init[2];
  rtb_TmpSignalConversionAtMATLAB[3] = rtB.init[3];
  obj = &rtDW.obj.IKInternal;
  if (rtDW.obj.IKInternal.isInitialized != 1) {
    rtDW.obj.IKInternal.isSetupComplete = false;
    rtDW.obj.IKInternal.isInitialized = 1;
    RigidBodyTree_get_JointPosition(rtDW.obj.IKInternal.RigidBodyTreeInternal,
      rtDW.obj.IKInternal.Limits);
    obj->_pobj0.matlabCodegenIsDeleted = false;
    rtDW.obj.IKInternal.Solver->ExtraArgs = &obj->_pobj0;
    for (i = 0; i < 36; i++) {
      rtDW.obj.IKInternal.Solver->ExtraArgs->WeightMatrix[i] = 0.0;
    }

    rtDW.obj.IKInternal.Solver->ExtraArgs->Robot =
      rtDW.obj.IKInternal.RigidBodyTreeInternal;
    rtDW.obj.IKInternal.Solver->ExtraArgs->KinematicModel =
      rtDW.obj.IKInternal.RigidBodyTreeKinematicModel;
    b_k = rtDW.obj.IKInternal.Limits->size[0] << 1;
    i = rtDW.obj.IKInternal.Solver->ExtraArgs->Limits->size[0] *
      rtDW.obj.IKInternal.Solver->ExtraArgs->Limits->size[1];
    rtDW.obj.IKInternal.Solver->ExtraArgs->Limits->size[0] =
      rtDW.obj.IKInternal.Limits->size[0];
    rtDW.obj.IKInternal.Solver->ExtraArgs->Limits->size[1] = 2;
    ArmRev_emxEnsureCapacity_real_T(rtDW.obj.IKInternal.Solver->
      ExtraArgs->Limits, i);
    ArmReverseKinema_emxInit_real_T(&tmp_4, 1);
    i = tmp_4->size[0];
    tmp_4->size[0] = b_k;
    ArmRev_emxEnsureCapacity_real_T(tmp_4, i);
    for (iacol = 0; iacol < b_k; iacol++) {
      tmp_4->data[iacol] = rtDW.obj.IKInternal.Limits->data[iacol];
    }

    loop_ub = tmp_4->size[0];
    for (iacol = 0; iacol < loop_ub; iacol++) {
      rtDW.obj.IKInternal.Solver->ExtraArgs->Limits->data[iacol] = tmp_4->
        data[iacol];
    }

    ArmReverseKinema_emxFree_real_T(&tmp_4);
    memset(&T1[0], 0, sizeof(real_T) << 4U);
    T1[0] = 1.0;
    T1[5] = 1.0;
    T1[10] = 1.0;
    T1[15] = 1.0;
    for (i = 0; i < 16; i++) {
      rtDW.obj.IKInternal.Solver->ExtraArgs->Tform[i] = T1[i];
    }

    rtDW.obj.IKInternal.Solver->ExtraArgs->BodyIndex = -1.0;
    i = rtDW.obj.IKInternal.Solver->ExtraArgs->ErrTemp->size[0];
    rtDW.obj.IKInternal.Solver->ExtraArgs->ErrTemp->size[0] = 6;
    ArmRev_emxEnsureCapacity_real_T(rtDW.obj.IKInternal.Solver->
      ExtraArgs->ErrTemp, i);
    for (i = 0; i < 6; i++) {
      rtDW.obj.IKInternal.Solver->ExtraArgs->ErrTemp->data[i] = 0.0;
    }

    rtDW.obj.IKInternal.Solver->ExtraArgs->CostTemp = 0.0;
    ArmReverseKinema_emxInit_int8_T(&b_gradTmp, 1);
    i = b_gradTmp->size[0];
    b_gradTmp->size[0] = (int32_T)
      rtDW.obj.IKInternal.RigidBodyTreeInternal->PositionNumber;
    ArmRev_emxEnsureCapacity_int8_T(b_gradTmp, i);
    loop_ub = (int32_T)rtDW.obj.IKInternal.RigidBodyTreeInternal->PositionNumber;
    if (loop_ub - 1 >= 0) {
      memset(&b_gradTmp->data[0], 0, (uint32_T)loop_ub * sizeof(int8_T));
    }

    i = rtDW.obj.IKInternal.Solver->ExtraArgs->GradTemp->size[0];
    rtDW.obj.IKInternal.Solver->ExtraArgs->GradTemp->size[0] = b_gradTmp->size[0];
    ArmRev_emxEnsureCapacity_real_T(rtDW.obj.IKInternal.Solver->
      ExtraArgs->GradTemp, i);
    loop_ub = b_gradTmp->size[0];
    ArmReverseKinema_emxFree_int8_T(&b_gradTmp);
    for (i = 0; i < loop_ub; i++) {
      rtDW.obj.IKInternal.Solver->ExtraArgs->GradTemp->data[i] = 0.0;
    }

    rtDW.obj.IKInternal.isSetupComplete = true;
  }

  ArmR_inverseKinematics_stepImpl(&rtDW.obj.IKInternal, out, u1,
    rtb_TmpSignalConversionAtMATLAB, rtB.MATLABSystem);

  /* End of MATLABSystem: '<S25>/MATLAB System' */

  /* SimscapeInputBlock: '<S19>/INPUT_4_1_1' */
  if (rtDW.INPUT_4_1_1_FirstOutput == 0.0) {
    rtDW.INPUT_4_1_1_FirstOutput = 1.0;
    rtX.ArmReverseKinematicsSimulink_PS[0] = rtB.MATLABSystem[3];
    rtX.ArmReverseKinematicsSimulink_PS[1] = 0.0;
  }

  rtB.INPUT_4_1_1[0] = rtX.ArmReverseKinematicsSimulink_PS[0];
  rtB.INPUT_4_1_1[1] = rtX.ArmReverseKinematicsSimulink_PS[1];
  rtB.INPUT_4_1_1[2] = ((rtB.MATLABSystem[3] -
    rtX.ArmReverseKinematicsSimulink_PS[0]) * 1000.0 - 2.0 *
                        rtX.ArmReverseKinematicsSimulink_PS[1]) * 1000.0;
  rtB.INPUT_4_1_1[3] = 0.0;

  /* End of SimscapeInputBlock: '<S19>/INPUT_4_1_1' */

  /* SimscapeInputBlock: '<S19>/INPUT_1_1_1' */
  if (rtDW.INPUT_1_1_1_FirstOutput == 0.0) {
    rtDW.INPUT_1_1_1_FirstOutput = 1.0;
    rtX.ArmReverseKinematicsSimulink__b[0] = rtB.MATLABSystem[0];
    rtX.ArmReverseKinematicsSimulink__b[1] = 0.0;
  }

  rtB.INPUT_1_1_1[0] = rtX.ArmReverseKinematicsSimulink__b[0];
  rtB.INPUT_1_1_1[1] = rtX.ArmReverseKinematicsSimulink__b[1];
  rtB.INPUT_1_1_1[2] = ((rtB.MATLABSystem[0] -
    rtX.ArmReverseKinematicsSimulink__b[0]) * 1000.0 - 2.0 *
                        rtX.ArmReverseKinematicsSimulink__b[1]) * 1000.0;
  rtB.INPUT_1_1_1[3] = 0.0;

  /* End of SimscapeInputBlock: '<S19>/INPUT_1_1_1' */

  /* SimscapeInputBlock: '<S19>/INPUT_2_1_1' */
  if (rtDW.INPUT_2_1_1_FirstOutput == 0.0) {
    rtDW.INPUT_2_1_1_FirstOutput = 1.0;
    rtX.ArmReverseKinematicsSimulink__l[0] = rtB.MATLABSystem[1];
    rtX.ArmReverseKinematicsSimulink__l[1] = 0.0;
  }

  rtB.INPUT_2_1_1[0] = rtX.ArmReverseKinematicsSimulink__l[0];
  rtB.INPUT_2_1_1[1] = rtX.ArmReverseKinematicsSimulink__l[1];
  rtB.INPUT_2_1_1[2] = ((rtB.MATLABSystem[1] -
    rtX.ArmReverseKinematicsSimulink__l[0]) * 1000.0 - 2.0 *
                        rtX.ArmReverseKinematicsSimulink__l[1]) * 1000.0;
  rtB.INPUT_2_1_1[3] = 0.0;

  /* End of SimscapeInputBlock: '<S19>/INPUT_2_1_1' */

  /* SimscapeInputBlock: '<S19>/INPUT_3_1_1' */
  if (rtDW.INPUT_3_1_1_FirstOutput == 0.0) {
    rtDW.INPUT_3_1_1_FirstOutput = 1.0;
    rtX.ArmReverseKinematicsSimulink__p[0] = rtB.MATLABSystem[2];
    rtX.ArmReverseKinematicsSimulink__p[1] = 0.0;
  }

  rtB.INPUT_3_1_1[0] = rtX.ArmReverseKinematicsSimulink__p[0];
  rtB.INPUT_3_1_1[1] = rtX.ArmReverseKinematicsSimulink__p[1];
  rtB.INPUT_3_1_1[2] = ((rtB.MATLABSystem[2] -
    rtX.ArmReverseKinematicsSimulink__p[0]) * 1000.0 - 2.0 *
                        rtX.ArmReverseKinematicsSimulink__p[1]) * 1000.0;
  rtB.INPUT_3_1_1[3] = 0.0;

  /* End of SimscapeInputBlock: '<S19>/INPUT_3_1_1' */

  /* SimscapeExecutionBlock: '<S19>/OUTPUT_1_0' incorporates:
   *  SimscapeExecutionBlock: '<S19>/STATE_1'
   */
  simulationData = (NeslSimulationData *)rtDW.OUTPUT_1_0_SimData;
  time_tmp = ssGetT(rtS);
  time = time_tmp;
  simulationData->mData->mTime.mN = 1;
  simulationData->mData->mTime.mX = &time;
  simulationData->mData->mContStates.mN = 0;
  simulationData->mData->mContStates.mX = NULL;
  simulationData->mData->mDiscStates.mN = 0;
  simulationData->mData->mDiscStates.mX = &rtDW.OUTPUT_1_0_Discrete;
  simulationData->mData->mModeVector.mN = 0;
  simulationData->mData->mModeVector.mX = &rtDW.OUTPUT_1_0_Modes;
  first_output = (ssIsMajorTimeStep(rtS) && ssGetRTWSolverInfo(rtS)
                  ->foundContZcEvents);
  simulationData->mData->mFoundZcEvents = first_output;
  first_output = ssIsMajorTimeStep(rtS);
  simulationData->mData->mIsMajorTimeStep = first_output;
  tmp = (ssGetMdlInfoPtr(rtS)->mdlFlags.solverAssertCheck == 1U);
  simulationData->mData->mIsSolverAssertCheck = tmp;
  tmp = ssIsSolverCheckingCIC(rtS);
  simulationData->mData->mIsSolverCheckingCIC = tmp;
  simulationData->mData->mIsComputingJacobian = false;
  simulationData->mData->mIsEvaluatingF0 = false;
  tmp = ssIsSolverRequestingReset(rtS);
  simulationData->mData->mIsSolverRequestingReset = tmp;
  tmp = ssIsModeUpdateTimeStep(rtS);
  simulationData->mData->mIsModeUpdateTimeStep = tmp;
  tmp_1[0] = 0;
  tmp_0[0] = rtB.INPUT_4_1_1[0];
  tmp_0[1] = rtB.INPUT_4_1_1[1];
  tmp_0[2] = rtB.INPUT_4_1_1[2];
  tmp_0[3] = rtB.INPUT_4_1_1[3];
  tmp_1[1] = 4;
  tmp_0[4] = rtB.INPUT_1_1_1[0];
  tmp_0[5] = rtB.INPUT_1_1_1[1];
  tmp_0[6] = rtB.INPUT_1_1_1[2];
  tmp_0[7] = rtB.INPUT_1_1_1[3];
  tmp_1[2] = 8;
  tmp_0[8] = rtB.INPUT_2_1_1[0];
  tmp_0[9] = rtB.INPUT_2_1_1[1];
  tmp_0[10] = rtB.INPUT_2_1_1[2];
  tmp_0[11] = rtB.INPUT_2_1_1[3];
  tmp_1[3] = 12;
  tmp_0[12] = rtB.INPUT_3_1_1[0];
  tmp_0[13] = rtB.INPUT_3_1_1[1];
  tmp_0[14] = rtB.INPUT_3_1_1[2];
  tmp_0[15] = rtB.INPUT_3_1_1[3];
  tmp_1[4] = 16;
  simulationData->mData->mInputValues.mN = 16;
  simulationData->mData->mInputValues.mX = &tmp_0[0];
  simulationData->mData->mInputOffsets.mN = 5;
  simulationData->mData->mInputOffsets.mX = &tmp_1[0];
  simulationData->mData->mOutputs.mN = 4;
  simulationData->mData->mOutputs.mX = &rtb_OUTPUT_1_0[0];
  simulationData->mData->mTolerances.mN = 0;
  simulationData->mData->mTolerances.mX = NULL;
  simulationData->mData->mCstateHasChanged = false;
  time_tmp_0 = ssGetTaskTime(rtS,0);
  time_0 = time_tmp_0;
  simulationData->mData->mTime.mN = 1;
  simulationData->mData->mTime.mX = &time_0;
  simulationData->mData->mSampleHits.mN = 0;
  simulationData->mData->mSampleHits.mX = NULL;
  simulationData->mData->mIsFundamentalSampleHit = false;
  diagnosticManager = (NeuDiagnosticManager *)rtDW.OUTPUT_1_0_DiagMgr;
  diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
  b_k = ne_simulator_method((NeslSimulator *)rtDW.OUTPUT_1_0_Simulator,
    NESL_SIM_OUTPUTS, simulationData, diagnosticManager);
  if (b_k != 0) {
    result = error_buffer_is_empty(ssGetErrorStatus(rtS));
    if (result) {
      msg = rtw_diagnostics_msg(diagnosticTree);
      ssSetErrorStatus(rtS, msg);
    }
  }

  if (first_output && simulationData->mData->mCstateHasChanged) {
    ssSetBlockStateForSolverChangedAtMajorStep(rtS);
  }

  /* End of SimscapeExecutionBlock: '<S19>/OUTPUT_1_0' */

  /* SignalConversion generated from: '<S20>/MATLAB System' */
  rtb_TmpSignalConversionAtMATLAB[0] = rtb_OUTPUT_1_0[1];
  rtb_TmpSignalConversionAtMATLAB[1] = rtb_OUTPUT_1_0[2];
  rtb_TmpSignalConversionAtMATLAB[2] = rtb_OUTPUT_1_0[3];
  rtb_TmpSignalConversionAtMATLAB[3] = rtb_OUTPUT_1_0[0];

  /* MATLABSystem: '<S20>/MATLAB System' */
  obj_0 = &rtDW.obj_l.TreeInternal;
  n = rtDW.obj_l.TreeInternal.NumBodies;
  for (i = 0; i < 16; i++) {
    expl_temp.f1[i] = tmp_9[i];
  }

  ArmReverseK_emxInit_f_cell_wrap(&Ttree, 2);

  /* MATLABSystem: '<S20>/MATLAB System' */
  i = Ttree->size[0] * Ttree->size[1];
  Ttree->size[0] = 1;
  Ttree->size[1] = (int32_T)n;
  A_emxEnsureCapacity_f_cell_wrap(Ttree, i);
  if ((int32_T)n != 0) {
    i = (int32_T)n - 1;
    for (b_k = 0; b_k <= i; b_k++) {
      Ttree->data[b_k] = expl_temp;
    }
  }

  k = 1.0;
  ibcol_tmp = (int32_T)n - 1;
  if ((int32_T)n - 1 >= 0) {
    for (i = 0; i < 5; i++) {
      a[i] = tmp_a[i];
    }
  }

  ArmReverseKinema_emxInit_char_T(&switch_expression, 2);

  /* MATLABSystem: '<S20>/MATLAB System' */
  for (b_jcol = 0; b_jcol <= ibcol_tmp; b_jcol++) {
    body = obj_0->Bodies[b_jcol];
    n = body->JointInternal.PositionNumber;
    n += k;
    if (k > n - 1.0) {
      iacol = 0;
      b_k = 0;
    } else {
      iacol = (int32_T)k - 1;
      b_k = (int32_T)(n - 1.0);
    }

    for (i = 0; i < 16; i++) {
      T1[i] = body->JointInternal.JointToParentTransform[i];
    }

    i = switch_expression->size[0] * switch_expression->size[1];
    switch_expression->size[0] = 1;
    switch_expression->size[1] = body->JointInternal.Type->size[1];
    ArmRev_emxEnsureCapacity_char_T(switch_expression, i);
    loop_ub = body->JointInternal.Type->size[1];
    for (i = 0; i < loop_ub; i++) {
      switch_expression->data[i] = body->JointInternal.Type->data[i];
    }

    result = false;
    if (switch_expression->size[1] != 5) {
    } else {
      i = 1;
      do {
        exitg1 = 0;
        if (i - 1 < 5) {
          if (a[i - 1] != switch_expression->data[i - 1]) {
            exitg1 = 1;
          } else {
            i++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (result) {
      cth = 0.0;
    } else {
      for (i = 0; i < 8; i++) {
        a_0[i] = tmp_c[i];
      }

      if (switch_expression->size[1] != 8) {
      } else {
        i = 1;
        do {
          exitg1 = 0;
          if (i - 1 < 8) {
            if (a_0[i - 1] != switch_expression->data[i - 1]) {
              exitg1 = 1;
            } else {
              i++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        cth = 1.0;
      } else {
        cth = -1.0;
      }
    }

    switch ((int32_T)cth) {
     case 0:
      memset(&out[0], 0, sizeof(real_T) << 4U);
      out[0] = 1.0;
      out[5] = 1.0;
      out[10] = 1.0;
      out[15] = 1.0;
      break;

     case 1:
      Ar_rigidBodyJoint_get_JointAxis(&body->JointInternal, v);
      result_data[0] = v[0];
      result_data[1] = v[1];
      result_data[2] = v[2];
      if ((b_k - iacol != 0) - 1 >= 0) {
        result_data[3] = rtb_TmpSignalConversionAtMATLAB[iacol];
      }

      k = result_data[0];
      v[0] = k * k;
      cth_tmp = result_data[1];
      v[1] = cth_tmp * cth_tmp;
      cth_tmp_0 = result_data[2];
      v[2] = cth_tmp_0 * cth_tmp_0;
      cth = 1.0 / sqrt((v[0] + v[1]) + v[2]);
      v[0] = k * cth;
      v[1] = cth_tmp * cth;
      v[2] = cth_tmp_0 * cth;
      k = result_data[3];
      cth = cos(k);
      k = sin(k);
      tempR[0] = v[0] * v[0] * (1.0 - cth) + cth;
      cth_tmp = v[0] * v[1] * (1.0 - cth);
      cth_tmp_0 = v[2] * k;
      tempR[1] = cth_tmp - cth_tmp_0;
      tempR_tmp = v[0] * v[2] * (1.0 - cth);
      tempR_tmp_0 = v[1] * k;
      tempR[2] = tempR_tmp + tempR_tmp_0;
      tempR[3] = cth_tmp + cth_tmp_0;
      tempR[4] = v[1] * v[1] * (1.0 - cth) + cth;
      cth_tmp = v[1] * v[2] * (1.0 - cth);
      cth_tmp_0 = v[0] * k;
      tempR[5] = cth_tmp - cth_tmp_0;
      tempR[6] = tempR_tmp - tempR_tmp_0;
      tempR[7] = cth_tmp + cth_tmp_0;
      tempR[8] = v[2] * v[2] * (1.0 - cth) + cth;
      for (b_k = 0; b_k < 3; b_k++) {
        R[b_k] = tempR[b_k * 3];
        R[b_k + 3] = tempR[b_k * 3 + 1];
        R[b_k + 6] = tempR[b_k * 3 + 2];
      }

      memset(&out[0], 0, sizeof(real_T) << 4U);
      for (i = 0; i < 3; i++) {
        b_k = i << 2;
        out[b_k] = R[3 * i];
        out[b_k + 1] = R[3 * i + 1];
        out[b_k + 2] = R[3 * i + 2];
      }

      out[15] = 1.0;
      break;

     default:
      Ar_rigidBodyJoint_get_JointAxis(&body->JointInternal, v);
      memset(&tempR[0], 0, 9U * sizeof(real_T));
      tempR[0] = 1.0;
      tempR[4] = 1.0;
      tempR[8] = 1.0;
      k = rtb_TmpSignalConversionAtMATLAB[iacol];
      for (i = 0; i < 3; i++) {
        b_k = i << 2;
        out[b_k] = tempR[3 * i];
        out[b_k + 1] = tempR[3 * i + 1];
        out[b_k + 2] = tempR[3 * i + 2];
        out[i + 12] = v[i] * k;
      }

      out[3] = 0.0;
      out[7] = 0.0;
      out[11] = 0.0;
      out[15] = 1.0;
      break;
    }

    for (i = 0; i < 16; i++) {
      rtb_MATLABSystem[i] = body->JointInternal.ChildToJointTransform[i];
    }

    for (i = 0; i < 4; i++) {
      cth = T1[i + 4];
      cth_tmp = T1[i];
      cth_tmp_0 = T1[i + 8];
      tempR_tmp = T1[i + 12];
      for (iacol = 0; iacol < 4; iacol++) {
        b_k = iacol << 2;
        T1_0[i + b_k] = ((out[b_k + 1] * cth + out[b_k] * cth_tmp) + out[b_k + 2]
                         * cth_tmp_0) + out[b_k + 3] * tempR_tmp;
      }

      cth = T1_0[i + 4];
      cth_tmp = T1_0[i];
      cth_tmp_0 = T1_0[i + 8];
      tempR_tmp = T1_0[i + 12];
      for (iacol = 0; iacol < 4; iacol++) {
        b_k = iacol << 2;
        Ttree->data[b_jcol].f1[i + b_k] = ((rtb_MATLABSystem[b_k + 1] * cth +
          rtb_MATLABSystem[b_k] * cth_tmp) + rtb_MATLABSystem[b_k + 2] *
          cth_tmp_0) + rtb_MATLABSystem[b_k + 3] * tempR_tmp;
      }
    }

    k = n;
    if (body->ParentIndex > 0.0) {
      for (i = 0; i < 16; i++) {
        T1[i] = Ttree->data[(int32_T)body->ParentIndex - 1].f1[i];
      }

      for (i = 0; i < 4; i++) {
        cth = T1[i + 4];
        cth_tmp = T1[i];
        cth_tmp_0 = T1[i + 8];
        tempR_tmp = T1[i + 12];
        for (iacol = 0; iacol < 4; iacol++) {
          b_k = iacol << 2;
          T1_0[i + b_k] = ((Ttree->data[b_jcol].f1[b_k + 1] * cth + Ttree->
                            data[b_jcol].f1[b_k] * cth_tmp) + Ttree->data[b_jcol]
                           .f1[b_k + 2] * cth_tmp_0) + Ttree->data[b_jcol]
            .f1[b_k + 3] * tempR_tmp;
        }
      }

      memcpy(&Ttree->data[b_jcol].f1[0], &T1_0[0], sizeof(real_T) << 4U);
    }
  }

  k = -1.0;
  i = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj_0->Base.NameInternal->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, i);
  loop_ub = obj_0->Base.NameInternal->size[1];
  for (i = 0; i < loop_ub; i++) {
    switch_expression->data[i] = obj_0->Base.NameInternal->data[i];
  }

  for (i = 0; i < 5; i++) {
    a[i] = tmp_b[i];
  }

  result = false;
  if (switch_expression->size[1] != 5) {
  } else {
    i = 1;
    do {
      exitg1 = 0;
      if (i - 1 < 5) {
        if (switch_expression->data[i - 1] != a[i - 1]) {
          exitg1 = 1;
        } else {
          i++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    k = 0.0;
  } else {
    cth = rtDW.obj_l.TreeInternal.NumBodies;
    b_jcol = 0;
    exitg2 = false;
    while ((!exitg2) && (b_jcol <= (int32_T)cth - 1)) {
      body = obj_0->Bodies[b_jcol];
      i = switch_expression->size[0] * switch_expression->size[1];
      switch_expression->size[0] = 1;
      switch_expression->size[1] = body->NameInternal->size[1];
      ArmRev_emxEnsureCapacity_char_T(switch_expression, i);
      loop_ub = body->NameInternal->size[1];
      for (i = 0; i < loop_ub; i++) {
        switch_expression->data[i] = body->NameInternal->data[i];
      }

      for (i = 0; i < 5; i++) {
        a[i] = tmp_b[i];
      }

      if (switch_expression->size[1] != 5) {
      } else {
        i = 1;
        do {
          exitg1 = 0;
          if (i - 1 < 5) {
            if (switch_expression->data[i - 1] != a[i - 1]) {
              exitg1 = 1;
            } else {
              i++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        k = (real_T)b_jcol + 1.0;
        exitg2 = true;
      } else {
        b_jcol++;
      }
    }
  }

  if (k == 0.0) {
    memset(&T1[0], 0, sizeof(real_T) << 4U);
    T1[0] = 1.0;
    T1[5] = 1.0;
    T1[10] = 1.0;
    T1[15] = 1.0;
  } else {
    for (i = 0; i < 16; i++) {
      T1[i] = Ttree->data[(int32_T)k - 1].f1[i];
    }
  }

  k = -1.0;
  i = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj_0->Base.NameInternal->size[1];
  ArmRev_emxEnsureCapacity_char_T(switch_expression, i);
  loop_ub = obj_0->Base.NameInternal->size[1];
  for (i = 0; i < loop_ub; i++) {
    switch_expression->data[i] = obj_0->Base.NameInternal->data[i];
  }

  b[0] = 'B';
  b[1] = 'a';
  b[2] = 's';
  b[3] = 'e';
  result = false;
  if (switch_expression->size[1] != 4) {
  } else {
    i = 1;
    do {
      exitg1 = 0;
      if (i - 1 < 4) {
        if (switch_expression->data[i - 1] != b[i - 1]) {
          exitg1 = 1;
        } else {
          i++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (result) {
    k = 0.0;
  } else {
    cth = rtDW.obj_l.TreeInternal.NumBodies;
    b_jcol = 0;
    exitg2 = false;
    while ((!exitg2) && (b_jcol <= (int32_T)cth - 1)) {
      body = obj_0->Bodies[b_jcol];
      i = switch_expression->size[0] * switch_expression->size[1];
      switch_expression->size[0] = 1;
      switch_expression->size[1] = body->NameInternal->size[1];
      ArmRev_emxEnsureCapacity_char_T(switch_expression, i);
      loop_ub = body->NameInternal->size[1];
      for (i = 0; i < loop_ub; i++) {
        switch_expression->data[i] = body->NameInternal->data[i];
      }

      b[0] = 'B';
      b[1] = 'a';
      b[2] = 's';
      b[3] = 'e';
      if (switch_expression->size[1] != 4) {
      } else {
        i = 1;
        do {
          exitg1 = 0;
          if (i - 1 < 4) {
            if (switch_expression->data[i - 1] != b[i - 1]) {
              exitg1 = 1;
            } else {
              i++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (result) {
        k = (real_T)b_jcol + 1.0;
        exitg2 = true;
      } else {
        b_jcol++;
      }
    }
  }

  ArmReverseKinema_emxFree_char_T(&switch_expression);

  /* MATLABSystem: '<S20>/MATLAB System' */
  if (k == 0.0) {
    memset(&out[0], 0, sizeof(real_T) << 4U);
    out[0] = 1.0;
    out[5] = 1.0;
    out[10] = 1.0;
    out[15] = 1.0;
  } else {
    for (i = 0; i < 16; i++) {
      out[i] = Ttree->data[(int32_T)k - 1].f1[i];
    }
  }

  ArmReverseK_emxFree_f_cell_wrap(&Ttree);

  /* MATLABSystem: '<S20>/MATLAB System' */
  for (i = 0; i < 3; i++) {
    R[3 * i] = out[i];
    R[3 * i + 1] = out[i + 4];
    R[3 * i + 2] = out[i + 8];
  }

  for (i = 0; i <= 6; i += 2) {
    /* MATLABSystem: '<S20>/MATLAB System' */
    tmp_8 = _mm_loadu_pd(&R[i]);
    _mm_storeu_pd(&R_0[i], _mm_mul_pd(tmp_8, _mm_set1_pd(-1.0)));
  }

  /* MATLABSystem: '<S20>/MATLAB System' */
  for (i = 8; i < 9; i++) {
    R_0[i] = -R[i];
  }

  n = out[13];
  k = out[12];
  cth = out[14];
  for (i = 0; i < 3; i++) {
    b_jcol = i << 2;
    R_1[b_jcol] = R[3 * i];
    R_1[b_jcol + 1] = R[3 * i + 1];
    R_1[b_jcol + 2] = R[3 * i + 2];
    R_1[i + 12] = (R_0[i + 3] * n + R_0[i] * k) + R_0[i + 6] * cth;
  }

  R_1[3] = 0.0;
  R_1[7] = 0.0;
  R_1[11] = 0.0;
  R_1[15] = 1.0;
  for (i = 0; i < 4; i++) {
    b_k = i << 2;
    cth = T1[b_k + 1];
    cth_tmp = T1[b_k];
    cth_tmp_0 = T1[b_k + 2];
    tempR_tmp = T1[b_k + 3];
    for (iacol = 0; iacol <= 2; iacol += 2) {
      tmp_8 = _mm_loadu_pd(&R_1[iacol + 4]);
      tmp_5 = _mm_loadu_pd(&R_1[iacol]);
      tmp_6 = _mm_loadu_pd(&R_1[iacol + 8]);
      tmp_7 = _mm_loadu_pd(&R_1[iacol + 12]);
      _mm_storeu_pd(&rtb_MATLABSystem[iacol + b_k], _mm_add_pd(_mm_add_pd
        (_mm_add_pd(_mm_mul_pd(_mm_set1_pd(cth), tmp_8), _mm_mul_pd(_mm_set1_pd
        (cth_tmp), tmp_5)), _mm_mul_pd(_mm_set1_pd(cth_tmp_0), tmp_6)),
        _mm_mul_pd(_mm_set1_pd(tempR_tmp), tmp_7)));
    }
  }

  /* MATLABSystem: '<S2>/Coordinate Transformation Conversion' incorporates:
   *  MATLABSystem: '<S20>/MATLAB System'
   */
  v[0] = rtb_MATLABSystem[12] / rtb_MATLABSystem[15];
  v[1] = rtb_MATLABSystem[13] / rtb_MATLABSystem[15];
  v[2] = rtb_MATLABSystem[14] / rtb_MATLABSystem[15];

  /* SimscapeExecutionBlock: '<S19>/STATE_1' */
  simulationData = (NeslSimulationData *)rtDW.STATE_1_SimData;
  time_1 = time_tmp;
  simulationData->mData->mTime.mN = 1;
  simulationData->mData->mTime.mX = &time_1;
  simulationData->mData->mContStates.mN = 0;
  simulationData->mData->mContStates.mX = NULL;
  simulationData->mData->mDiscStates.mN = 0;
  simulationData->mData->mDiscStates.mX = &rtDW.STATE_1_Discrete;
  simulationData->mData->mModeVector.mN = 0;
  simulationData->mData->mModeVector.mX = &rtDW.STATE_1_Modes;
  result = (ssIsMajorTimeStep(rtS) && ssGetRTWSolverInfo(rtS)->foundContZcEvents);
  simulationData->mData->mFoundZcEvents = result;
  simulationData->mData->mIsMajorTimeStep = first_output;
  result = (ssGetMdlInfoPtr(rtS)->mdlFlags.solverAssertCheck == 1U);
  simulationData->mData->mIsSolverAssertCheck = result;
  result = ssIsSolverCheckingCIC(rtS);
  simulationData->mData->mIsSolverCheckingCIC = result;
  simulationData->mData->mIsComputingJacobian = false;
  simulationData->mData->mIsEvaluatingF0 = false;
  result = ssIsSolverRequestingReset(rtS);
  simulationData->mData->mIsSolverRequestingReset = result;
  simulationData->mData->mIsModeUpdateTimeStep = tmp;
  tmp_3[0] = 0;
  tmp_2[0] = rtB.INPUT_4_1_1[0];
  tmp_2[1] = rtB.INPUT_4_1_1[1];
  tmp_2[2] = rtB.INPUT_4_1_1[2];
  tmp_2[3] = rtB.INPUT_4_1_1[3];
  tmp_3[1] = 4;
  tmp_2[4] = rtB.INPUT_1_1_1[0];
  tmp_2[5] = rtB.INPUT_1_1_1[1];
  tmp_2[6] = rtB.INPUT_1_1_1[2];
  tmp_2[7] = rtB.INPUT_1_1_1[3];
  tmp_3[2] = 8;
  tmp_2[8] = rtB.INPUT_2_1_1[0];
  tmp_2[9] = rtB.INPUT_2_1_1[1];
  tmp_2[10] = rtB.INPUT_2_1_1[2];
  tmp_2[11] = rtB.INPUT_2_1_1[3];
  tmp_3[3] = 12;
  tmp_2[12] = rtB.INPUT_3_1_1[0];
  tmp_2[13] = rtB.INPUT_3_1_1[1];
  tmp_2[14] = rtB.INPUT_3_1_1[2];
  tmp_2[15] = rtB.INPUT_3_1_1[3];
  tmp_3[4] = 16;
  simulationData->mData->mInputValues.mN = 16;
  simulationData->mData->mInputValues.mX = &tmp_2[0];
  simulationData->mData->mInputOffsets.mN = 5;
  simulationData->mData->mInputOffsets.mX = &tmp_3[0];
  simulationData->mData->mOutputs.mN = 0;
  simulationData->mData->mOutputs.mX = NULL;
  simulationData->mData->mTolerances.mN = 0;
  simulationData->mData->mTolerances.mX = NULL;
  simulationData->mData->mCstateHasChanged = false;
  time_2 = time_tmp_0;
  simulationData->mData->mTime.mN = 1;
  simulationData->mData->mTime.mX = &time_2;
  simulationData->mData->mSampleHits.mN = 0;
  simulationData->mData->mSampleHits.mX = NULL;
  simulationData->mData->mIsFundamentalSampleHit = false;
  diagnosticManager = (NeuDiagnosticManager *)rtDW.STATE_1_DiagMgr;
  diagnosticTree_0 = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
  b_k = ne_simulator_method((NeslSimulator *)rtDW.STATE_1_Simulator,
    NESL_SIM_OUTPUTS, simulationData, diagnosticManager);
  if (b_k != 0) {
    tmp = error_buffer_is_empty(ssGetErrorStatus(rtS));
    if (tmp) {
      msg_0 = rtw_diagnostics_msg(diagnosticTree_0);
      ssSetErrorStatus(rtS, msg_0);
    }
  }

  if (first_output && simulationData->mData->mCstateHasChanged) {
    ssSetBlockStateForSolverChangedAtMajorStep(rtS);
  }

  UNUSED_PARAMETER(tid);
}

/* Update for root system: '<Root>' */
void MdlUpdate(int_T tid)
{
  NeslSimulationData *simulationData;
  NeuDiagnosticManager *diagnosticManager;
  NeuDiagnosticTree *diagnosticTree;
  char *msg;
  real_T tmp_0[16];
  real_T time;
  int32_T tmp_2;
  int_T tmp_1[5];
  boolean_T tmp;

  /* Update for SimscapeExecutionBlock: '<S19>/STATE_1' */
  simulationData = (NeslSimulationData *)rtDW.STATE_1_SimData;
  time = ssGetT(rtS);
  simulationData->mData->mTime.mN = 1;
  simulationData->mData->mTime.mX = &time;
  simulationData->mData->mContStates.mN = 0;
  simulationData->mData->mContStates.mX = NULL;
  simulationData->mData->mDiscStates.mN = 0;
  simulationData->mData->mDiscStates.mX = &rtDW.STATE_1_Discrete;
  simulationData->mData->mModeVector.mN = 0;
  simulationData->mData->mModeVector.mX = &rtDW.STATE_1_Modes;
  tmp = (ssIsMajorTimeStep(rtS) && ssGetRTWSolverInfo(rtS)->foundContZcEvents);
  simulationData->mData->mFoundZcEvents = tmp;
  simulationData->mData->mIsMajorTimeStep = ssIsMajorTimeStep(rtS);
  tmp = (ssGetMdlInfoPtr(rtS)->mdlFlags.solverAssertCheck == 1U);
  simulationData->mData->mIsSolverAssertCheck = tmp;
  tmp = ssIsSolverCheckingCIC(rtS);
  simulationData->mData->mIsSolverCheckingCIC = tmp;
  simulationData->mData->mIsComputingJacobian = false;
  simulationData->mData->mIsEvaluatingF0 = false;
  tmp = ssIsSolverRequestingReset(rtS);
  simulationData->mData->mIsSolverRequestingReset = tmp;
  simulationData->mData->mIsModeUpdateTimeStep = ssIsModeUpdateTimeStep(rtS);
  tmp_1[0] = 0;
  tmp_0[0] = rtB.INPUT_4_1_1[0];
  tmp_0[1] = rtB.INPUT_4_1_1[1];
  tmp_0[2] = rtB.INPUT_4_1_1[2];
  tmp_0[3] = rtB.INPUT_4_1_1[3];
  tmp_1[1] = 4;
  tmp_0[4] = rtB.INPUT_1_1_1[0];
  tmp_0[5] = rtB.INPUT_1_1_1[1];
  tmp_0[6] = rtB.INPUT_1_1_1[2];
  tmp_0[7] = rtB.INPUT_1_1_1[3];
  tmp_1[2] = 8;
  tmp_0[8] = rtB.INPUT_2_1_1[0];
  tmp_0[9] = rtB.INPUT_2_1_1[1];
  tmp_0[10] = rtB.INPUT_2_1_1[2];
  tmp_0[11] = rtB.INPUT_2_1_1[3];
  tmp_1[3] = 12;
  tmp_0[12] = rtB.INPUT_3_1_1[0];
  tmp_0[13] = rtB.INPUT_3_1_1[1];
  tmp_0[14] = rtB.INPUT_3_1_1[2];
  tmp_0[15] = rtB.INPUT_3_1_1[3];
  tmp_1[4] = 16;
  simulationData->mData->mInputValues.mN = 16;
  simulationData->mData->mInputValues.mX = &tmp_0[0];
  simulationData->mData->mInputOffsets.mN = 5;
  simulationData->mData->mInputOffsets.mX = &tmp_1[0];
  diagnosticManager = (NeuDiagnosticManager *)rtDW.STATE_1_DiagMgr;
  diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
  tmp_2 = ne_simulator_method((NeslSimulator *)rtDW.STATE_1_Simulator,
    NESL_SIM_UPDATE, simulationData, diagnosticManager);
  if (tmp_2 != 0) {
    tmp = error_buffer_is_empty(ssGetErrorStatus(rtS));
    if (tmp) {
      msg = rtw_diagnostics_msg(diagnosticTree);
      ssSetErrorStatus(rtS, msg);
    }
  }

  /* End of Update for SimscapeExecutionBlock: '<S19>/STATE_1' */
  UNUSED_PARAMETER(tid);
}

/* Derivatives for root system: '<Root>' */
void MdlDerivatives(void)
{
  XDot *_rtXdot;
  _rtXdot = ((XDot *) ssGetdX(rtS));

  /* Derivatives for SimscapeInputBlock: '<S19>/INPUT_4_1_1' */
  _rtXdot->ArmReverseKinematicsSimulink_PS[0] =
    rtX.ArmReverseKinematicsSimulink_PS[1];
  _rtXdot->ArmReverseKinematicsSimulink_PS[1] = ((rtB.MATLABSystem[3] -
    rtX.ArmReverseKinematicsSimulink_PS[0]) * 1000.0 - 2.0 *
    rtX.ArmReverseKinematicsSimulink_PS[1]) * 1000.0;

  /* Derivatives for SimscapeInputBlock: '<S19>/INPUT_1_1_1' */
  _rtXdot->ArmReverseKinematicsSimulink__b[0] =
    rtX.ArmReverseKinematicsSimulink__b[1];
  _rtXdot->ArmReverseKinematicsSimulink__b[1] = ((rtB.MATLABSystem[0] -
    rtX.ArmReverseKinematicsSimulink__b[0]) * 1000.0 - 2.0 *
    rtX.ArmReverseKinematicsSimulink__b[1]) * 1000.0;

  /* Derivatives for SimscapeInputBlock: '<S19>/INPUT_2_1_1' */
  _rtXdot->ArmReverseKinematicsSimulink__l[0] =
    rtX.ArmReverseKinematicsSimulink__l[1];
  _rtXdot->ArmReverseKinematicsSimulink__l[1] = ((rtB.MATLABSystem[1] -
    rtX.ArmReverseKinematicsSimulink__l[0]) * 1000.0 - 2.0 *
    rtX.ArmReverseKinematicsSimulink__l[1]) * 1000.0;

  /* Derivatives for SimscapeInputBlock: '<S19>/INPUT_3_1_1' */
  _rtXdot->ArmReverseKinematicsSimulink__p[0] =
    rtX.ArmReverseKinematicsSimulink__p[1];
  _rtXdot->ArmReverseKinematicsSimulink__p[1] = ((rtB.MATLABSystem[2] -
    rtX.ArmReverseKinematicsSimulink__p[0]) * 1000.0 - 2.0 *
    rtX.ArmReverseKinematicsSimulink__p[1]) * 1000.0;
}

/* Projection for root system: '<Root>' */
void MdlProjection(void)
{
}

/* Termination for root system: '<Root>' */
void MdlTerminate(void)
{
  b_inverseKinematics_ArmRevers_T *obj;
  f_robotics_manip_internal_IKE_T *obj_1;
  h_robotics_core_internal_Erro_T *obj_0;
  h_robotics_manip_internal_Col_T obj_a;
  i_robotics_manip_internal_Col_T *obj_9;
  k_robotics_manip_internal_Rig_T *obj_8;
  l_robotics_manip_internal_Rig_T *obj_7;
  m_robotics_manip_internal_Col_T obj_6;
  n_robotics_manip_internal_Col_T *obj_5;
  v_robotics_manip_internal_Rig_T *obj_4;
  w_robotics_manip_internal_Rig_T *obj_3;
  x_robotics_manip_internal_Rig_T *obj_2;
  real_T b_0;
  int32_T b;
  int32_T b_i;
  int32_T e;

  /* Terminate for FromWorkspace: '<S8>/fromWS_Signal 1' */
  rt_FREE(rtDW.fromWS_Signal1_PWORK.RSimInfoPtr);

  /* Terminate for FromWorkspace: '<S8>/From Workspace' */
  rt_FREE(rtDW.FromWorkspace_PWORK.RSimInfoPtr);

  /* Terminate for FromWorkspace: '<S8>/From Workspace1' */
  rt_FREE(rtDW.FromWorkspace1_PWORK.RSimInfoPtr);

  /* Terminate for MATLABSystem: '<S25>/MATLAB System' */
  if (!rtDW.obj.matlabCodegenIsDeleted) {
    rtDW.obj.matlabCodegenIsDeleted = true;
  }

  obj = &rtDW.obj.IKInternal;
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    if (obj->isInitialized == 1) {
      obj->isInitialized = 2;
    }
  }

  obj_0 = &rtDW.obj.IKInternal._pobj5;
  if (!obj_0->matlabCodegenIsDeleted) {
    obj_0->matlabCodegenIsDeleted = true;
  }

  obj_1 = &rtDW.obj.IKInternal._pobj0;
  if (!obj_1->matlabCodegenIsDeleted) {
    obj_1->matlabCodegenIsDeleted = true;
  }

  obj_2 = &rtDW.obj.IKInternal._pobj4;
  if (!obj_2->matlabCodegenIsDeleted) {
    obj_2->matlabCodegenIsDeleted = true;
  }

  obj_3 = &rtDW.obj.TreeInternal;
  if (!obj_3->matlabCodegenIsDeleted) {
    obj_3->matlabCodegenIsDeleted = true;
  }

  obj_4 = &rtDW.obj.TreeInternal.Base;
  if (!obj_4->matlabCodegenIsDeleted) {
    obj_4->matlabCodegenIsDeleted = true;
  }

  for (b = 0; b < 10; b++) {
    obj_4 = &rtDW.obj.TreeInternal._pobj2[b];
    if (!obj_4->matlabCodegenIsDeleted) {
      obj_4->matlabCodegenIsDeleted = true;
    }
  }

  for (b = 0; b < 5; b++) {
    obj_4 = &rtDW.obj.IKInternal._pobj2[b];
    if (!obj_4->matlabCodegenIsDeleted) {
      obj_4->matlabCodegenIsDeleted = true;
    }
  }

  obj_4 = &rtDW.obj.IKInternal._pobj4.Base;
  if (!obj_4->matlabCodegenIsDeleted) {
    obj_4->matlabCodegenIsDeleted = true;
  }

  for (b = 0; b < 5; b++) {
    obj_4 = &rtDW.obj.IKInternal._pobj4._pobj0[b];
    if (!obj_4->matlabCodegenIsDeleted) {
      obj_4->matlabCodegenIsDeleted = true;
    }
  }

  for (e = 0; e < 11; e++) {
    obj_5 = &rtDW.obj.TreeInternal._pobj0[e];
    if (!obj_5->matlabCodegenIsDeleted) {
      obj_5->matlabCodegenIsDeleted = true;
      b_0 = obj_5->Size;
      b = (int32_T)b_0 - 1;
      for (b_i = 0; b_i <= b; b_i++) {
        obj_6 = obj_5->CollisionGeometries->data[b_i];
        collisioncodegen_destructGeometry(&obj_6.CollisionPrimitive);
        obj_5->CollisionGeometries->data[b_i] = obj_6;
      }
    }
  }

  for (e = 0; e < 11; e++) {
    obj_5 = &rtDW.obj.IKInternal._pobj3[e];
    if (!obj_5->matlabCodegenIsDeleted) {
      obj_5->matlabCodegenIsDeleted = true;
      b_0 = obj_5->Size;
      b = (int32_T)b_0 - 1;
      for (b_i = 0; b_i <= b; b_i++) {
        obj_6 = obj_5->CollisionGeometries->data[b_i];
        collisioncodegen_destructGeometry(&obj_6.CollisionPrimitive);
        obj_5->CollisionGeometries->data[b_i] = obj_6;
      }
    }
  }

  for (e = 0; e < 6; e++) {
    obj_5 = &rtDW.obj.IKInternal._pobj4._pobj1[e];
    if (!obj_5->matlabCodegenIsDeleted) {
      obj_5->matlabCodegenIsDeleted = true;
      b_0 = obj_5->Size;
      b = (int32_T)b_0 - 1;
      for (b_i = 0; b_i <= b; b_i++) {
        obj_6 = obj_5->CollisionGeometries->data[b_i];
        collisioncodegen_destructGeometry(&obj_6.CollisionPrimitive);
        obj_5->CollisionGeometries->data[b_i] = obj_6;
      }
    }
  }

  /* End of Terminate for MATLABSystem: '<S25>/MATLAB System' */
  emxFreeStruct_robotics_slmanip_(&rtDW.obj);

  /* Terminate for SimscapeExecutionBlock: '<S19>/OUTPUT_1_0' */
  neu_destroy_diagnostic_manager((NeuDiagnosticManager *)rtDW.OUTPUT_1_0_DiagMgr);
  nesl_destroy_simulation_data((NeslSimulationData *)rtDW.OUTPUT_1_0_SimData);
  nesl_erase_simulator("ArmReverseKinematics/Arm/Solver Configuration_1");
  nesl_destroy_registry();

  /* Terminate for MATLABSystem: '<S20>/MATLAB System' */
  if (!rtDW.obj_l.matlabCodegenIsDeleted) {
    rtDW.obj_l.matlabCodegenIsDeleted = true;
  }

  obj_7 = &rtDW.obj_l.TreeInternal;
  if (!obj_7->matlabCodegenIsDeleted) {
    obj_7->matlabCodegenIsDeleted = true;
  }

  obj_8 = &rtDW.obj_l.TreeInternal.Base;
  if (!obj_8->matlabCodegenIsDeleted) {
    obj_8->matlabCodegenIsDeleted = true;
  }

  for (b = 0; b < 10; b++) {
    obj_8 = &rtDW.obj_l.TreeInternal._pobj0[b];
    if (!obj_8->matlabCodegenIsDeleted) {
      obj_8->matlabCodegenIsDeleted = true;
    }
  }

  obj_9 = &rtDW.obj_l.TreeInternal.Base.CollisionsInternal;
  if (!obj_9->matlabCodegenIsDeleted) {
    obj_9->matlabCodegenIsDeleted = true;
    b_0 = obj_9->Size;
    b = (int32_T)b_0 - 1;
    for (b_i = 0; b_i <= b; b_i++) {
      obj_a = obj_9->CollisionGeometries->data[b_i];
      collisioncodegen_destructGeometry(&obj_a.CollisionPrimitive);
      obj_9->CollisionGeometries->data[b_i] = obj_a;
    }
  }

  for (b = 0; b < 10; b++) {
    obj_9 = &rtDW.obj_l.TreeInternal._pobj0[b].CollisionsInternal;
    if (!obj_9->matlabCodegenIsDeleted) {
      obj_9->matlabCodegenIsDeleted = true;
      b_0 = obj_9->Size;
      e = (int32_T)b_0 - 1;
      for (b_i = 0; b_i <= e; b_i++) {
        obj_a = obj_9->CollisionGeometries->data[b_i];
        collisioncodegen_destructGeometry(&obj_a.CollisionPrimitive);
        obj_9->CollisionGeometries->data[b_i] = obj_a;
      }
    }
  }

  /* End of Terminate for MATLABSystem: '<S20>/MATLAB System' */
  emxFreeStruct_robotics_slmani_e(&rtDW.obj_l);

  /* Terminate for SimscapeExecutionBlock: '<S19>/STATE_1' */
  neu_destroy_diagnostic_manager((NeuDiagnosticManager *)rtDW.STATE_1_DiagMgr);
  nesl_destroy_simulation_data((NeslSimulationData *)rtDW.STATE_1_SimData);
  nesl_erase_simulator("ArmReverseKinematics/Arm/Solver Configuration_1");
  nesl_destroy_registry();
}

/* Function to initialize sizes */
void MdlInitializeSizes(void)
{
  ssSetNumContStates(rtS, 8);          /* Number of continuous states */
  ssSetNumPeriodicContStates(rtS, 0); /* Number of periodic continuous states */
  ssSetNumY(rtS, 0);                   /* Number of model outputs */
  ssSetNumU(rtS, 0);                   /* Number of model inputs */
  ssSetDirectFeedThrough(rtS, 0);      /* The model is not direct feedthrough */
  ssSetNumSampleTimes(rtS, 2);         /* Number of sample times */
  ssSetNumBlocks(rtS, 70);             /* Number of blocks */
  ssSetNumBlockIO(rtS, 8);             /* Number of block outputs */
  ssSetNumBlockParams(rtS, 40);        /* Sum of parameter "widths" */
}

/* Function to initialize sample times. */
void MdlInitializeSampleTimes(void)
{
  /* task periods */
  ssSetSampleTime(rtS, 0, 0.0);
  ssSetSampleTime(rtS, 1, 0.0);

  /* task offsets */
  ssSetOffsetTime(rtS, 0, 0.0);
  ssSetOffsetTime(rtS, 1, 1.0);
}

/* Function to register the model */
/* Turns off all optimizations on Windows because of issues with VC 2015 compiler.
   This function is not performance-critical, hence this is not a problem.
 */
#if defined(_MSC_VER)

#pragma optimize( "", off )

#endif

SimStruct * ArmReverseKinematics(void)
{
  static struct _ssMdlInfo mdlInfo;
  static struct _ssBlkInfo2 blkInfo2;
  static struct _ssBlkInfoSLSize blkInfoSLSize;
  (void) memset((char_T *)rtS, 0,
                sizeof(SimStruct));
  (void) memset((char_T *)&mdlInfo, 0,
                sizeof(struct _ssMdlInfo));
  (void) memset((char_T *)&blkInfo2, 0,
                sizeof(struct _ssBlkInfo2));
  (void) memset((char_T *)&blkInfoSLSize, 0,
                sizeof(struct _ssBlkInfoSLSize));
  ssSetBlkInfo2Ptr(rtS, &blkInfo2);
  ssSetBlkInfoSLSizePtr(rtS, &blkInfoSLSize);
  ssSetMdlInfoPtr(rtS, &mdlInfo);

  /* timing info */
  {
    static time_T mdlPeriod[NSAMPLE_TIMES];
    static time_T mdlOffset[NSAMPLE_TIMES];
    static time_T mdlTaskTimes[NSAMPLE_TIMES];
    static int_T mdlTsMap[NSAMPLE_TIMES];
    static int_T mdlSampleHits[NSAMPLE_TIMES];
    static boolean_T mdlTNextWasAdjustedPtr[NSAMPLE_TIMES];
    static int_T mdlPerTaskSampleHits[NSAMPLE_TIMES * NSAMPLE_TIMES];
    static time_T mdlTimeOfNextSampleHit[NSAMPLE_TIMES];

    {
      int_T i;
      for (i = 0; i < NSAMPLE_TIMES; i++) {
        mdlPeriod[i] = 0.0;
        mdlOffset[i] = 0.0;
        mdlTaskTimes[i] = 0.0;
        mdlTsMap[i] = i;
        mdlSampleHits[i] = 1;
      }
    }

    ssSetSampleTimePtr(rtS, &mdlPeriod[0]);
    ssSetOffsetTimePtr(rtS, &mdlOffset[0]);
    ssSetSampleTimeTaskIDPtr(rtS, &mdlTsMap[0]);
    ssSetTPtr(rtS, &mdlTaskTimes[0]);
    ssSetSampleHitPtr(rtS, &mdlSampleHits[0]);
    ssSetTNextWasAdjustedPtr(rtS, &mdlTNextWasAdjustedPtr[0]);
    ssSetPerTaskSampleHitsPtr(rtS, &mdlPerTaskSampleHits[0]);
    ssSetTimeOfNextSampleHitPtr(rtS, &mdlTimeOfNextSampleHit[0]);
  }

  ssSetSolverMode(rtS, SOLVER_MODE_SINGLETASKING);

  /*
   * initialize model vectors and cache them in SimStruct
   */

  /* block I/O */
  {
    ssSetBlockIO(rtS, ((void *) &rtB));
    (void) memset(((void *) &rtB), 0,
                  sizeof(B));
  }

  /* states (continuous)*/
  {
    real_T *x = (real_T *) &rtX;
    ssSetContStates(rtS, x);
    (void) memset((void *)x, 0,
                  sizeof(X));
  }

  /* states (dwork) */
  {
    void *dwork = (void *) &rtDW;
    ssSetRootDWork(rtS, dwork);
    (void) memset(dwork, 0,
                  sizeof(DW));
  }

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    ssSetModelMappingInfo(rtS, &dtInfo);
    dtInfo.numDataTypes = 27;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.BTransTable = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.PTransTable = &rtPTransTable;
  }

  /* Model specific registration */
  ssSetRootSS(rtS, rtS);
  ssSetVersion(rtS, SIMSTRUCT_VERSION_LEVEL2);
  ssSetModelName(rtS, "ArmReverseKinematics");
  ssSetPath(rtS, "ArmReverseKinematics");
  ssSetTStart(rtS, 0.0);
  ssSetTFinal(rtS, 4.0);

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    ssSetRTWLogInfo(rtS, &rt_DataLoggingInfo);
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(ssGetRTWLogInfo(rtS), (NULL));
    rtliSetLogXSignalPtrs(ssGetRTWLogInfo(rtS), (NULL));
    rtliSetLogT(ssGetRTWLogInfo(rtS), "tout");
    rtliSetLogX(ssGetRTWLogInfo(rtS), "");
    rtliSetLogXFinal(ssGetRTWLogInfo(rtS), "");
    rtliSetLogVarNameModifier(ssGetRTWLogInfo(rtS), "rt_");
    rtliSetLogFormat(ssGetRTWLogInfo(rtS), 4);
    rtliSetLogMaxRows(ssGetRTWLogInfo(rtS), 0);
    rtliSetLogDecimation(ssGetRTWLogInfo(rtS), 1);
    rtliSetLogY(ssGetRTWLogInfo(rtS), "");
    rtliSetLogYSignalInfo(ssGetRTWLogInfo(rtS), (NULL));
    rtliSetLogYSignalPtrs(ssGetRTWLogInfo(rtS), (NULL));
  }

  {
    static struct _ssStatesInfo2 statesInfo2;
    ssSetStatesInfo2(rtS, &statesInfo2);
  }

  {
    static ssPeriodicStatesInfo periodicStatesInfo;
    ssSetPeriodicStatesInfo(rtS, &periodicStatesInfo);
  }

  {
    static ssJacobianPerturbationBounds jacobianPerturbationBounds;
    ssSetJacobianPerturbationBounds(rtS, &jacobianPerturbationBounds);
  }

  {
    static ssSolverInfo slvrInfo;
    static boolean_T contStatesDisabled[8];
    static real_T absTol[8] = { 1.0E-6, 1.0E-6, 1.0E-6, 1.0E-6, 1.0E-6, 1.0E-6,
      1.0E-6, 1.0E-6 };

    static uint8_T absTolControl[8] = { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };

    static real_T contStateJacPerturbBoundMinVec[8];
    static real_T contStateJacPerturbBoundMaxVec[8];

    /* Initialize cont state perturbation bound */
    {
      int i;
      for (i = 0; i < 8; ++i) {
        contStateJacPerturbBoundMinVec[i] = 0;
        contStateJacPerturbBoundMaxVec[i] = rtGetInf();
      }
    }

    ssSetSolverRelTol(rtS, 0.001);
    ssSetStepSize(rtS, 0.0);
    ssSetMinStepSize(rtS, 0.0);
    ssSetMaxNumMinSteps(rtS, -1);
    ssSetMinStepViolatedError(rtS, 0);
    ssSetMaxStepSize(rtS, 0.08);
    ssSetSolverMaxOrder(rtS, -1);
    ssSetSolverRefineFactor(rtS, 1);
    ssSetOutputTimes(rtS, (NULL));
    ssSetNumOutputTimes(rtS, 0);
    ssSetOutputTimesOnly(rtS, 0);
    ssSetOutputTimesIndex(rtS, 0);
    ssSetZCCacheNeedsReset(rtS, 0);
    ssSetDerivCacheNeedsReset(rtS, 0);
    ssSetNumNonContDerivSigInfos(rtS, 0);
    ssSetNonContDerivSigInfos(rtS, (NULL));
    ssSetSolverInfo(rtS, &slvrInfo);
    ssSetSolverName(rtS, "VariableStepAuto");
    ssSetVariableStepSolver(rtS, 1);
    ssSetSolverConsistencyChecking(rtS, 0);
    ssSetSolverAdaptiveZcDetection(rtS, 0);
    ssSetSolverRobustResetMethod(rtS, 0);
    ssSetAbsTolVector(rtS, absTol);
    ssSetAbsTolControlVector(rtS, absTolControl);
    ssSetSolverAbsTol_Obsolete(rtS, absTol);
    ssSetSolverAbsTolControl_Obsolete(rtS, absTolControl);
    ssSetJacobianPerturbationBoundsMinVec(rtS, contStateJacPerturbBoundMinVec);
    ssSetJacobianPerturbationBoundsMaxVec(rtS, contStateJacPerturbBoundMaxVec);
    ssSetSolverStateProjection(rtS, 0);
    ssSetSolverMassMatrixType(rtS, (ssMatrixType)0);
    ssSetSolverMassMatrixNzMax(rtS, 0);
    ssSetModelOutputs(rtS, MdlOutputs);
    ssSetModelLogData(rtS, rt_UpdateTXYLogVars);
    ssSetModelLogDataIfInInterval(rtS, rt_UpdateTXXFYLogVars);
    ssSetModelUpdate(rtS, MdlUpdate);
    ssSetModelDerivatives(rtS, MdlDerivatives);
    ssSetSolverMaxConsecutiveMinStep(rtS, 1);
    ssSetSolverShapePreserveControl(rtS, 2);
    ssSetTNextTid(rtS, INT_MIN);
    ssSetTNext(rtS, rtMinusInf);
    ssSetSolverNeedsReset(rtS);
    ssSetNumNonsampledZCs(rtS, 0);
    ssSetContStateDisabled(rtS, contStatesDisabled);
    ssSetSolverMaxConsecutiveMinStep(rtS, 1);
  }

  ssSetChecksumVal(rtS, 0, 3096637441U);
  ssSetChecksumVal(rtS, 1, 3214740571U);
  ssSetChecksumVal(rtS, 2, 3317619168U);
  ssSetChecksumVal(rtS, 3, 1222207877U);
  return rtS;
}

/* When you use the on parameter, it resets the optimizations to those that you
   specified with the /O compiler option. */
#if defined(_MSC_VER)

#pragma optimize( "", on )

#endif
