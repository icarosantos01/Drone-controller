//
// File: mod.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 10-Mar-2026 14:00:27
//

// Include Files
#include "mod.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// Arguments    : double x
// Return Type  : double
//
namespace coder {
double b_mod(double x)
{
  double r;
  r = std::fmod(x, 100.0);
  if (r == 0.0) {
    r = 0.0;
  }
  return r;
}

} // namespace coder

//
// File trailer for mod.cpp
//
// [EOF]
//
