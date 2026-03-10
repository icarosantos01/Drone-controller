//
// File: sec.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 10-Mar-2026 14:00:27
//

// Include Files
#include "sec.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// Arguments    : double &x
// Return Type  : void
//
namespace coder {
void sec(double &x)
{
  x = 1.0 / std::cos(x);
}

} // namespace coder

//
// File trailer for sec.cpp
//
// [EOF]
//
