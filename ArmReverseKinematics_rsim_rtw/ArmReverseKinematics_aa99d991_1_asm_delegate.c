/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'ArmReverseKinematics/Arm/Solver Configuration'.
 */

#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
#include "sm_CTarget.h"

void ArmReverseKinematics_aa99d991_1_setTargets(const RuntimeDerivedValuesBundle
  *rtdv, CTarget *targets)
{
  (void) rtdv;
  (void) targets;
}

void ArmReverseKinematics_aa99d991_1_resetAsmStateVector(const void *mech,
  double *state)
{
  double xx[1];
  (void) mech;
  xx[0] = 0.0;
  state[0] = xx[0];
  state[1] = xx[0];
  state[2] = xx[0];
  state[3] = xx[0];
  state[4] = xx[0];
  state[5] = xx[0];
  state[6] = xx[0];
  state[7] = xx[0];
}

void ArmReverseKinematics_aa99d991_1_initializeTrackedAngleState(const void
  *mech, const RuntimeDerivedValuesBundle *rtdv, const int *modeVector, const
  double *motionData, double *state)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) modeVector;
  (void) motionData;
}

void ArmReverseKinematics_aa99d991_1_computeDiscreteState(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, double *state)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
}

void ArmReverseKinematics_aa99d991_1_adjustPosition(const void *mech, const
  double *dofDeltas, double *state)
{
  (void) mech;
  state[0] = state[0] + dofDeltas[0];
  state[2] = state[2] + dofDeltas[1];
  state[4] = state[4] + dofDeltas[2];
  state[6] = state[6] + dofDeltas[3];
}

static void perturbAsmJointPrimitiveState_0_0(double mag, double *state)
{
  state[0] = state[0] + mag;
}

static void perturbAsmJointPrimitiveState_0_0v(double mag, double *state)
{
  state[0] = state[0] + mag;
  state[1] = state[1] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_1_0(double mag, double *state)
{
  state[2] = state[2] + mag;
}

static void perturbAsmJointPrimitiveState_1_0v(double mag, double *state)
{
  state[2] = state[2] + mag;
  state[3] = state[3] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_2_0(double mag, double *state)
{
  state[4] = state[4] + mag;
}

static void perturbAsmJointPrimitiveState_2_0v(double mag, double *state)
{
  state[4] = state[4] + mag;
  state[5] = state[5] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_3_0(double mag, double *state)
{
  state[6] = state[6] + mag;
}

static void perturbAsmJointPrimitiveState_3_0v(double mag, double *state)
{
  state[6] = state[6] + mag;
  state[7] = state[7] - 0.875 * mag;
}

void ArmReverseKinematics_aa99d991_1_perturbAsmJointPrimitiveState(const void
  *mech, size_t stageIdx, size_t primIdx, double mag, boolean_T
  doPerturbVelocity, double *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) mag;
  (void) doPerturbVelocity;
  (void) state;
  switch ((stageIdx * 6 + primIdx) * 2 + (doPerturbVelocity ? 1 : 0))
  {
   case 0:
    perturbAsmJointPrimitiveState_0_0(mag, state);
    break;

   case 1:
    perturbAsmJointPrimitiveState_0_0v(mag, state);
    break;

   case 12:
    perturbAsmJointPrimitiveState_1_0(mag, state);
    break;

   case 13:
    perturbAsmJointPrimitiveState_1_0v(mag, state);
    break;

   case 24:
    perturbAsmJointPrimitiveState_2_0(mag, state);
    break;

   case 25:
    perturbAsmJointPrimitiveState_2_0v(mag, state);
    break;

   case 36:
    perturbAsmJointPrimitiveState_3_0(mag, state);
    break;

   case 37:
    perturbAsmJointPrimitiveState_3_0v(mag, state);
    break;
  }
}

void ArmReverseKinematics_aa99d991_1_computePosDofBlendMatrix(const void *mech,
  size_t stageIdx, size_t primIdx, const double *state, int partialType, double *
  matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void ArmReverseKinematics_aa99d991_1_computeVelDofBlendMatrix(const void *mech,
  size_t stageIdx, size_t primIdx, const double *state, int partialType, double *
  matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void ArmReverseKinematics_aa99d991_1_projectPartiallyTargetedPos(const void
  *mech, size_t stageIdx, size_t primIdx, const double *origState, int
  partialType, double *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) origState;
  (void) partialType;
  (void) state;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void ArmReverseKinematics_aa99d991_1_propagateMotion(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const double *state, double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[40];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  xx[0] = 0.5;
  xx[1] = xx[0] * state[0];
  xx[2] = 0.0;
  xx[3] = 0.05;
  xx[4] = 0.7071067811865476;
  xx[5] = xx[0] * state[2];
  xx[6] = xx[4] * sin(xx[5]);
  xx[7] = - xx[6];
  xx[8] = xx[4] * cos(xx[5]);
  xx[4] = xx[0] * xx[6];
  xx[5] = xx[4] * xx[6];
  xx[9] = xx[0] * xx[8];
  xx[10] = xx[9] * xx[8];
  xx[11] = 2.0;
  xx[12] = (xx[5] + xx[10]) * xx[11] - xx[0];
  xx[13] = (xx[9] * xx[6] + xx[4] * xx[8]) * xx[11];
  xx[4] = xx[0] * state[4];
  xx[9] = cos(xx[4]);
  xx[14] = sin(xx[4]);
  xx[4] = xx[0] * xx[14];
  xx[15] = xx[0] - (xx[11] * xx[4] * xx[14] - xx[0]);
  xx[16] = xx[11] * xx[9] * xx[4];
  xx[4] = xx[0] * state[6];
  xx[17] = cos(xx[4]);
  xx[18] = sin(xx[4]);
  xx[4] = xx[0] * xx[18];
  xx[19] = xx[0] - (xx[11] * xx[4] * xx[18] - xx[0]);
  xx[20] = xx[11] * xx[17] * xx[4];
  xx[4] = 0.6;
  xx[21] = xx[8] * state[1];
  xx[22] = xx[8] * xx[21];
  xx[23] = xx[6] * state[1];
  xx[24] = xx[6] * xx[23];
  xx[25] = xx[11] * (xx[22] - xx[24]);
  xx[26] = (xx[6] * xx[21] + xx[8] * xx[23]) * xx[11];
  xx[21] = state[1] - (xx[22] + xx[24]) * xx[11] + state[3];
  xx[22] = xx[12] * state[1];
  xx[23] = xx[8] * xx[22];
  xx[27] = xx[8];
  xx[28] = xx[7];
  xx[29] = xx[8];
  xx[24] = xx[13] * state[1];
  xx[30] = xx[8] * xx[24];
  xx[31] = xx[23] - xx[6] * xx[24];
  xx[32] = xx[23];
  xx[33] = xx[30];
  xx[34] = - xx[31];
  pm_math_Vector3_cross_ra(xx + 27, xx + 32, xx + 35);
  xx[27] = (xx[6] * xx[23] + xx[35]) * xx[11] + xx[24];
  xx[23] = xx[11] * (xx[36] + xx[6] * xx[30]) - xx[22] + xx[0] * state[3];
  xx[22] = xx[11] * (xx[37] - xx[31] * xx[6]);
  xx[6] = xx[26] * xx[14];
  xx[24] = xx[25] * xx[14];
  xx[28] = xx[25] - (xx[9] * xx[6] + xx[24] * xx[14]) * xx[11];
  xx[29] = xx[11] * (xx[6] * xx[14] - xx[9] * xx[24]) - xx[26];
  xx[6] = xx[21] + state[5];
  xx[24] = xx[27] - xx[21] * xx[16];
  xx[30] = xx[21] * xx[15] + xx[23];
  xx[31] = xx[30] * xx[14];
  xx[32] = xx[24] * xx[14];
  xx[33] = xx[24] + xx[11] * (xx[9] * xx[31] - xx[32] * xx[14]);
  xx[24] = xx[30] - (xx[9] * xx[32] + xx[31] * xx[14]) * xx[11] + xx[0] * state
    [5];
  xx[30] = xx[26] * xx[15] + xx[16] * xx[25] + xx[22];
  xx[31] = xx[18] * xx[29];
  xx[32] = xx[18] * xx[28];
  xx[34] = xx[28] + xx[11] * (xx[17] * xx[31] - xx[32] * xx[18]);
  xx[35] = xx[29] - (xx[17] * xx[32] + xx[31] * xx[18]) * xx[11];
  xx[31] = xx[6] + state[7];
  xx[32] = xx[33] - xx[6] * xx[20];
  xx[36] = xx[6] * xx[19] + xx[24];
  xx[37] = xx[36] * xx[18];
  xx[38] = xx[32] * xx[18];
  xx[39] = xx[32] + xx[11] * (xx[17] * xx[37] - xx[38] * xx[18]);
  xx[32] = xx[36] - (xx[17] * xx[38] + xx[37] * xx[18]) * xx[11] + xx[0] *
    state[7];
  xx[0] = xx[30] - (xx[19] * xx[29] - xx[20] * xx[28]);
  motionData[0] = cos(xx[1]);
  motionData[1] = xx[2];
  motionData[2] = xx[2];
  motionData[3] = sin(xx[1]);
  motionData[4] = xx[2];
  motionData[5] = xx[2];
  motionData[6] = xx[3];
  motionData[7] = xx[7];
  motionData[8] = xx[8];
  motionData[9] = xx[7];
  motionData[10] = xx[8];
  motionData[11] = - xx[12];
  motionData[12] = - xx[13];
  motionData[13] = xx[3] - xx[11] * (xx[5] - xx[10]);
  motionData[14] = xx[9];
  motionData[15] = xx[2];
  motionData[16] = xx[2];
  motionData[17] = xx[14];
  motionData[18] = xx[15];
  motionData[19] = xx[16];
  motionData[20] = xx[2];
  motionData[21] = xx[17];
  motionData[22] = xx[2];
  motionData[23] = xx[2];
  motionData[24] = xx[18];
  motionData[25] = xx[19];
  motionData[26] = xx[20];
  motionData[27] = xx[2];
  motionData[28] = 1.0;
  motionData[29] = xx[2];
  motionData[30] = xx[2];
  motionData[31] = xx[2];
  motionData[32] = xx[4];
  motionData[33] = xx[2];
  motionData[34] = xx[2];
  motionData[35] = xx[2];
  motionData[36] = xx[2];
  motionData[37] = state[1];
  motionData[38] = xx[2];
  motionData[39] = xx[2];
  motionData[40] = xx[2];
  motionData[41] = xx[25];
  motionData[42] = - xx[26];
  motionData[43] = xx[21];
  motionData[44] = xx[27];
  motionData[45] = xx[23];
  motionData[46] = xx[22];
  motionData[47] = xx[28];
  motionData[48] = xx[29];
  motionData[49] = xx[6];
  motionData[50] = xx[33];
  motionData[51] = xx[24];
  motionData[52] = xx[30];
  motionData[53] = xx[34];
  motionData[54] = xx[35];
  motionData[55] = xx[31];
  motionData[56] = xx[39];
  motionData[57] = xx[32];
  motionData[58] = xx[0];
  motionData[59] = xx[34];
  motionData[60] = xx[35];
  motionData[61] = xx[31];
  motionData[62] = xx[39];
  motionData[63] = xx[31] * xx[4] + xx[32];
  motionData[64] = xx[0] - xx[4] * xx[35];
}

size_t ArmReverseKinematics_aa99d991_1_computeAssemblyError(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, const int
  *modeVector, const double *motionData, double *error)
{
  (void) mech;
  (void)rtdv;
  (void) modeVector;
  (void) motionData;
  (void) error;
  switch (constraintIdx)
  {
  }

  return 0;
}

size_t ArmReverseKinematics_aa99d991_1_computeAssemblyJacobian(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, boolean_T
  forVelocitySatisfaction, const double *state, const int *modeVector, const
  double *motionData, double *J)
{
  (void) mech;
  (void) rtdv;
  (void) state;
  (void) modeVector;
  (void) forVelocitySatisfaction;
  (void) motionData;
  (void) J;
  switch (constraintIdx)
  {
  }

  return 0;
}

size_t ArmReverseKinematics_aa99d991_1_computeFullAssemblyJacobian(const void
  *mech, const RuntimeDerivedValuesBundle *rtdv, const double *state, const int *
  modeVector, const double *motionData, double *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) modeVector;
  (void) motionData;
  (void) J;
  return 0;
}

boolean_T ArmReverseKinematics_aa99d991_1_isInKinematicSingularity(const void
  *mech, const RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, const int
  *modeVector, const double *motionData)
{
  (void) mech;
  (void) rtdv;
  (void) modeVector;
  (void) motionData;
  switch (constraintIdx)
  {
  }

  return 0;
}

void ArmReverseKinematics_aa99d991_1_convertStateVector(const void *asmMech,
  const RuntimeDerivedValuesBundle *rtdv, const void *simMech, const double
  *asmState, const int *asmModeVector, const int *simModeVector, double
  *simState)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) asmMech;
  (void) rtdvd;
  (void) rtdvi;
  (void) simMech;
  (void) asmModeVector;
  (void) simModeVector;
  simState[0] = asmState[0];
  simState[1] = asmState[1];
  simState[2] = asmState[2];
  simState[3] = asmState[3];
  simState[4] = asmState[4];
  simState[5] = asmState[5];
  simState[6] = asmState[6];
  simState[7] = asmState[7];
}
