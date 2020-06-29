/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: LM_RTL_V0_4c2.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 27-Jan-2020 15:26:11
 */

/* Include Files */
#include <math.h>
#include <string.h>
#include "LM_RTL_V0_4c2.h"
#include "norm.h"
#include "J_B_zyx.h"

/* Function Definitions */

/*
 * LM_RTL
 * Arguments    : const float x0[6]
 *                const float r_s[81]
 *                const float B[27]
 *                float p[6]
 * Return Type  : void
 */
void LM_RTL_V0_4c2(const float x0[6], const float r_s[81], const float B[27],
                   float p[6])
{
  float lambda;
  int i;
  int n;
  bool exitg1;
  float y_hat[27];
  int c;
  float J[162];
  int jy;
  int i0;
  int iy;
  float smax;
  float b_c[36];
  float s;
  int i1;
  float b_p[3];
  float p_tmp;
  int j;
  float h_LM_tmp_tmp[162];
  float b_p_tmp;
  int jj;
  float a;
  int k;
  signed char c_p[6];
  int jp1j;
  signed char ipiv[6];
  float b_h_LM_tmp_tmp[162];
  static const signed char b[729] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  int ix;
  float x[36];
  static const signed char b_b[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  float y;
  float z1[27];
  float h_LM[6];
  float c_tmp[6];
  float b_B[27];

  /* 1e-11; */
  /* lambda = sigma^2; */
  lambda = 0.001F;

  /* 1e-24; */
  /* gamma = 1e-18; */
  /* W = diag(1e-11*ones([27 1])); */
  for (i = 0; i < 6; i++) {
    p[i] = x0[i];
  }

  /* p = meas+[10,0,0,0,0,0]'*1e-3 */
  n = 0;
  exitg1 = false;
  while ((!exitg1) && (n < 10)) {
    /*  Gleichung f�r magnetischen Dipol */
    /*  r - Abstandsvektor zu Ursprung vom B [m] */
    /*  m - magnetisches Moment [Am�] [Am^2] */
    /*  B - magnetische Flussdichte [T] */
    /*  magnetische Feldkonstante [T m/A] */
    memset(&y_hat[0], 0, 27U * sizeof(float));
    for (c = 0; c < 9; c++) {
      jy = c * 3;
      iy = 3 * (jy + 2);
      smax = p[0] - r_s[iy];
      s = 3.0F * smax;
      b_p[0] = smax;
      p_tmp = p[1] - r_s[1 + iy];
      b_p[1] = p_tmp;
      b_p_tmp = p[2] - r_s[2 + iy];
      b_p[2] = b_p_tmp;
      a = b_norm(b_p);
      b_p[0] = smax;
      b_p[1] = p_tmp;
      b_p[2] = b_p_tmp;
      y_hat[jy + 2] = 1.25663712E-6F / (12.566371F * powf(b_norm(b_p), 5.0F)) *
        (((s * p[3] * smax + s * p[4] * p_tmp) + s * p[5] * b_p_tmp) - p[3] * (a
          * a));

      /*  x */
      iy = 3 * (jy + 1);
      smax = p[1] - r_s[1 + iy];
      s = 3.0F * smax;
      p_tmp = p[0] - r_s[iy];
      b_p[0] = p_tmp;
      b_p[1] = smax;
      b_p_tmp = p[2] - r_s[2 + iy];
      b_p[2] = b_p_tmp;
      a = b_norm(b_p);
      b_p[0] = p_tmp;
      b_p[1] = smax;
      b_p[2] = b_p_tmp;
      y_hat[jy + 1] = 1.25663712E-6F / (12.566371F * powf(b_norm(b_p), 5.0F)) *
        (((s * p[3] * p_tmp + s * p[4] * smax) + s * p[5] * b_p_tmp) - p[4] * (a
          * a));

      /*  y */
      smax = p[2] - r_s[2 + 3 * jy];
      s = 3.0F * smax;
      p_tmp = p[0] - r_s[3 * jy];
      b_p[0] = p_tmp;
      b_p_tmp = p[1] - r_s[1 + 3 * jy];
      b_p[1] = b_p_tmp;
      b_p[2] = smax;
      a = b_norm(b_p);
      b_p[0] = p_tmp;
      b_p[1] = b_p_tmp;
      b_p[2] = smax;
      y_hat[jy] = 1.25663712E-6F / (12.566371F * powf(b_norm(b_p), 5.0F)) * (((s
        * p[3] * p_tmp + s * p[4] * b_p_tmp) + s * p[5] * smax) - p[5] * (a * a));

      /*  z */
    }

    J_B_zyx(p, J);
    for (i0 = 0; i0 < 27; i0++) {
      y_hat[i0] = B[i0] - y_hat[i0];
      for (i1 = 0; i1 < 6; i1++) {
        h_LM_tmp_tmp[i1 + 6 * i0] = J[i0 + 27 * i1];
      }
    }

    memset(&b_c[0], 0, 36U * sizeof(float));
    for (i0 = 0; i0 < 6; i0++) {
      for (i1 = 0; i1 < 27; i1++) {
        smax = 0.0F;
        for (iy = 0; iy < 27; iy++) {
          smax += h_LM_tmp_tmp[i0 + 6 * iy] * (float)b[iy + 27 * i1];
        }

        b_h_LM_tmp_tmp[i0 + 6 * i1] = smax;
      }

      for (i1 = 0; i1 < 6; i1++) {
        smax = 0.0F;
        for (iy = 0; iy < 27; iy++) {
          smax += b_h_LM_tmp_tmp[i0 + 6 * iy] * J[iy + 27 * i1];
        }

        iy = i0 + 6 * i1;
        x[iy] = smax + lambda * (float)b_b[iy];
      }

      ipiv[i0] = (signed char)(1 + i0);
    }

    for (j = 0; j < 5; j++) {
      c = j * 7;
      jj = j * 7;
      jp1j = c + 2;
      i = 6 - j;
      jy = 0;
      ix = c;
      smax = fabsf(x[c]);
      for (k = 2; k <= i; k++) {
        ix++;
        s = fabsf(x[ix]);
        if (s > smax) {
          jy = k - 1;
          smax = s;
        }
      }

      if (x[jj + jy] != 0.0F) {
        if (jy != 0) {
          iy = j + jy;
          ipiv[j] = (signed char)(iy + 1);
          ix = j;
          for (k = 0; k < 6; k++) {
            smax = x[ix];
            x[ix] = x[iy];
            x[iy] = smax;
            ix += 6;
            iy += 6;
          }
        }

        i0 = (jj - j) + 6;
        for (i = jp1j; i <= i0; i++) {
          x[i - 1] /= x[jj];
        }
      }

      i = 4 - j;
      jy = c + 6;
      iy = jj;
      for (jp1j = 0; jp1j <= i; jp1j++) {
        smax = x[jy];
        if (x[jy] != 0.0F) {
          ix = jj + 1;
          i0 = iy + 8;
          i1 = (iy - j) + 12;
          for (c = i0; c <= i1; c++) {
            x[c - 1] += x[ix] * -smax;
            ix++;
          }
        }

        jy += 6;
        iy += 6;
      }
    }

    for (i0 = 0; i0 < 6; i0++) {
      c_p[i0] = (signed char)(1 + i0);
    }

    for (k = 0; k < 5; k++) {
      if (ipiv[k] > 1 + k) {
        iy = ipiv[k] - 1;
        jy = c_p[iy];
        c_p[iy] = c_p[k];
        c_p[k] = (signed char)jy;
      }
    }

    for (k = 0; k < 6; k++) {
      c = c_p[k] - 1;
      b_c[k + 6 * (c_p[k] - 1)] = 1.0F;
      for (j = k + 1; j < 7; j++) {
        i0 = (j + 6 * c) - 1;
        if (b_c[i0] != 0.0F) {
          i1 = j + 1;
          for (i = i1; i < 7; i++) {
            jp1j = (i + 6 * c) - 1;
            b_c[jp1j] -= b_c[i0] * x[(i + 6 * (j - 1)) - 1];
          }
        }
      }
    }

    for (j = 0; j < 6; j++) {
      jy = 6 * j;
      for (k = 5; k >= 0; k--) {
        iy = 6 * k;
        i0 = k + jy;
        if (b_c[i0] != 0.0F) {
          b_c[i0] /= x[k + iy];
          for (i = 0; i < k; i++) {
            jp1j = i + jy;
            b_c[jp1j] -= b_c[i0] * x[i + iy];
          }
        }
      }
    }

    for (i0 = 0; i0 < 6; i0++) {
      for (i1 = 0; i1 < 27; i1++) {
        smax = 0.0F;
        for (iy = 0; iy < 6; iy++) {
          smax += b_c[i0 + 6 * iy] * h_LM_tmp_tmp[iy + 6 * i1];
        }

        b_h_LM_tmp_tmp[i0 + 6 * i1] = smax;
      }

      h_LM[i0] = 0.0F;
      for (i1 = 0; i1 < 27; i1++) {
        smax = 0.0F;
        for (iy = 0; iy < 27; iy++) {
          smax += b_h_LM_tmp_tmp[i0 + 6 * iy] * (float)b[iy + 27 * i1];
        }

        h_LM[i0] += smax * y_hat[i1];
      }
    }

    for (k = 0; k < 27; k++) {
      z1[k] = y_hat[k] * y_hat[k];
    }

    y = z1[0];
    for (k = 0; k < 26; k++) {
      y += z1[k + 1];
    }

    for (i = 0; i < 6; i++) {
      c_tmp[i] = p[i] + h_LM[i];
    }

    /*  Gleichung f�r magnetischen Dipol */
    /*  r - Abstandsvektor zu Ursprung vom B [m] */
    /*  m - magnetisches Moment [Am�] [Am^2] */
    /*  B - magnetische Flussdichte [T] */
    /*  magnetische Feldkonstante [T m/A] */
    memset(&b_B[0], 0, 27U * sizeof(float));
    for (c = 0; c < 9; c++) {
      jy = c * 3;
      iy = 3 * (jy + 2);
      smax = c_tmp[0] - r_s[iy];
      s = 3.0F * smax;
      b_p[0] = smax;
      p_tmp = c_tmp[1] - r_s[1 + iy];
      b_p[1] = p_tmp;
      b_p_tmp = c_tmp[2] - r_s[2 + iy];
      b_p[2] = b_p_tmp;
      a = b_norm(b_p);
      b_p[0] = smax;
      b_p[1] = p_tmp;
      b_p[2] = b_p_tmp;
      b_B[jy + 2] = 1.25663712E-6F / (12.566371F * powf(b_norm(b_p), 5.0F)) *
        (((s * c_tmp[3] * smax + s * c_tmp[4] * p_tmp) + s * c_tmp[5] * b_p_tmp)
         - c_tmp[3] * (a * a));

      /*  x */
      iy = 3 * (jy + 1);
      smax = c_tmp[1] - r_s[1 + iy];
      s = 3.0F * smax;
      p_tmp = c_tmp[0] - r_s[iy];
      b_p[0] = p_tmp;
      b_p[1] = smax;
      b_p_tmp = c_tmp[2] - r_s[2 + iy];
      b_p[2] = b_p_tmp;
      a = b_norm(b_p);
      b_p[0] = p_tmp;
      b_p[1] = smax;
      b_p[2] = b_p_tmp;
      b_B[jy + 1] = 1.25663712E-6F / (12.566371F * powf(b_norm(b_p), 5.0F)) *
        (((s * c_tmp[3] * p_tmp + s * c_tmp[4] * smax) + s * c_tmp[5] * b_p_tmp)
         - c_tmp[4] * (a * a));

      /*  y */
      smax = c_tmp[2] - r_s[2 + 3 * jy];
      s = 3.0F * smax;
      p_tmp = c_tmp[0] - r_s[3 * jy];
      b_p[0] = p_tmp;
      b_p_tmp = c_tmp[1] - r_s[1 + 3 * jy];
      b_p[1] = b_p_tmp;
      b_p[2] = smax;
      a = b_norm(b_p);
      b_p[0] = p_tmp;
      b_p[1] = b_p_tmp;
      b_p[2] = smax;
      b_B[jy] = 1.25663712E-6F / (12.566371F * powf(b_norm(b_p), 5.0F)) * (((s *
        c_tmp[3] * p_tmp + s * c_tmp[4] * b_p_tmp) + s * c_tmp[5] * smax) -
        c_tmp[5] * (a * a));

      /*  z */
    }

    for (k = 0; k < 27; k++) {
      smax = B[k] - b_B[k];
      b_B[k] = smax;
      z1[k] = smax * smax;
    }

    s = z1[0];
    for (k = 0; k < 26; k++) {
      s += z1[k + 1];
    }

    if (y < 1.0E-8F) {
      exitg1 = true;
    } else {
      /* p_i = (chi2(n)-chi2_lm)/(h_LM'*(lambda*h_LM+J'*W*(y-y_hat)')); */
      p_tmp = 0.0F;
      for (i0 = 0; i0 < 6; i0++) {
        smax = 0.0F;
        for (i1 = 0; i1 < 27; i1++) {
          b_p_tmp = 0.0F;
          for (iy = 0; iy < 27; iy++) {
            b_p_tmp += h_LM_tmp_tmp[i0 + 6 * iy] * (float)b[iy + 27 * i1];
          }

          smax += b_p_tmp * y_hat[i1];
        }

        p_tmp += h_LM[i0] * (lambda * h_LM[i0] + smax);
      }

      if (1.0E-12F > (y - s) / p_tmp) {
        lambda *= 11.0F;
      } else {
        lambda /= 9.0F;
        for (i = 0; i < 6; i++) {
          p[i] = c_tmp[i];
        }

        if (lambda < 1.0E-32F) {
          lambda = 1.0E-32F;
        }
      }

      n++;
    }
  }
}

/*
 * File trailer for LM_RTL_V0_4c2.c
 *
 * [EOF]
 */
