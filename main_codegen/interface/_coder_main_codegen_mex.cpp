//
// File: _coder_main_codegen_mex.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 10-Mar-2026 14:00:27
//

// Include Files
#include "_coder_main_codegen_mex.h"
#include "_coder_main_codegen_api.h"

// Function Definitions
//
// Arguments    : int32_T nlhs
//                mxArray *plhs[]
//                int32_T nrhs
//                const mxArray *prhs[]
// Return Type  : void
//
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray *[])
{
  mexAtExit(&main_codegen_atexit);
  // Module initialization.
  main_codegen_initialize();
  // Dispatch the entry-point.
  unsafe_main_codegen_mexFunction(nlhs, plhs, nrhs);
  // Module termination.
  main_codegen_terminate();
}

//
// Arguments    : void
// Return Type  : emlrtCTX
//
emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, nullptr, 1,
                           nullptr, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

//
// Arguments    : int32_T nlhs
//                mxArray *plhs[1]
//                int32_T nrhs
// Return Type  : void
//
void unsafe_main_codegen_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                     int32_T nrhs)
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 0) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 0, 4,
                        12, "main_codegen");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 12,
                        "main_codegen");
  }
  // Call the function.
  main_codegen_api(&outputs);
  // Copy over outputs to the caller.
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

//
// File trailer for _coder_main_codegen_mex.cpp
//
// [EOF]
//
