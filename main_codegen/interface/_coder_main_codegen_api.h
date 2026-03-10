//
// File: _coder_main_codegen_api.h
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 10-Mar-2026 14:00:27
//

#ifndef _CODER_MAIN_CODEGEN_API_H
#define _CODER_MAIN_CODEGEN_API_H

// Include Files
#include "coder_array_mex.h"
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void main_codegen(coder::array<real_T, 2U> *drone1_state_out);

void main_codegen_api(const mxArray **plhs);

void main_codegen_atexit();

void main_codegen_initialize();

void main_codegen_terminate();

void main_codegen_xil_shutdown();

void main_codegen_xil_terminate();

#endif
//
// File trailer for _coder_main_codegen_api.h
//
// [EOF]
//
