//
// File: _coder_main_codegen_api.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 10-Mar-2026 14:00:27
//

// Include Files
#include "_coder_main_codegen_api.h"
#include "_coder_main_codegen_mex.h"
#include "coder_array_mex.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131643U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "main_codegen",                                       // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

// Function Declarations
static const mxArray *emlrt_marshallOut(const coder::array<real_T, 2U> &u);

// Function Definitions
//
// Arguments    : const coder::array<real_T, 2U> &u
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const coder::array<real_T, 2U> &u)
{
  static const int32_T iv[2]{0, 0};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, &(((coder::array<real_T, 2U> *)&u)->data())[0]);
  emlrtSetDimensions((mxArray *)m, ((coder::array<real_T, 2U> *)&u)->size(), 2);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const mxArray **plhs
// Return Type  : void
//
void main_codegen_api(const mxArray **plhs)
{
  coder::array<real_T, 2U> drone1_state_out;
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  st.tls = emlrtRootTLSGlobal;
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  // Invoke the target function
  main_codegen(drone1_state_out);
  // Marshall function outputs
  drone1_state_out.no_free();
  *plhs = emlrt_marshallOut(drone1_state_out);
  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

//
// Arguments    : void
// Return Type  : void
//
void main_codegen_atexit()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  main_codegen_xil_terminate();
  main_codegen_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void main_codegen_initialize()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void main_codegen_terminate()
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

//
// File trailer for _coder_main_codegen_api.cpp
//
// [EOF]
//
