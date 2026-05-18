/* -------------------------------------------------------------------
 *
 * The banner will be completed upon code publication.
 *
 * Title:   getPayloadCoriolisRegressionMatrix.h
 * Author:  Vincenzo Petrone, Antonio Annunziata
 * Org.:    UNISA
 * Date:    July 24, 2023
 *
 * This is a function returning the torque-level regressor Y such
 * that the torques due to the presence of a payload attached at the
 * robot end-effector are computed as i = Y * pi, where pi are the
 * payload inertial parameters.
 *
 * -------------------------------------------------------------------
 */

//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getPayloadRegressionMatrix.h
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 21-Jul-2023 19:12:49
//

#pragma once

// Include Files
// #include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace ur10 {
extern void getPayloadRegressionMatrix(const double in1[6], const double in2[6], const double in3[6], double payload_regression_matrix[114]);

}

//
// File trailer for getPayloadRegressionMatrix.h
//
// [EOF]
//
