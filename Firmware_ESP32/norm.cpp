/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: norm.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 27-Jan-2020 15:26:11
 */

/* Include Files */
#include <math.h>
#include "LM_RTL_V0_4c2.h"
#include "norm.h"

/* Function Definitions */

/*
 * Arguments    : const float x[3]
 * Return Type  : float
 */
float b_norm(const float x[3])
{
  float y;
  float scale;
  float absxk;
  float t;
  scale = 1.29246971E-26F;
  absxk = fabsf(x[0]);
  if (absxk > 1.29246971E-26F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    y = t * t;
  }

  absxk = fabsf(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = 1.0F + y * t * t;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabsf(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = 1.0F + y * t * t;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrtf(y);
}

/*
 * File trailer for norm.c
 *
 * [EOF]
 */
