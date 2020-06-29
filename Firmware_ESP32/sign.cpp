/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sign.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 27-Jan-2020 15:26:11
 */

/* Include Files */
#include "LM_RTL_V0_4c2.h"
#include "sign.h"

/* Function Definitions */

/*
 * Arguments    : float *x
 * Return Type  : void
 */
void b_sign(float *x)
{
  if (*x < 0.0F) {
    *x = -1.0F;
  } else if (*x > 0.0F) {
    *x = 1.0F;
  } else {
    if (*x == 0.0F) {
      *x = 0.0F;
    }
  }
}

/*
 * File trailer for sign.c
 *
 * [EOF]
 */
