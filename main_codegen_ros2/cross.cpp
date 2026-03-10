//
// File: cross.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 10-Mar-2026 14:00:27
//

// Include Files
#include "cross.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : const double a[3]
//                const double b[3]
//                double c[3]
// Return Type  : void
//
namespace coder {
void cross(const double a[3], const double b[3], double c[3])
{
  c[0] = a[1] * b[2] - b[1] * a[2];
  c[1] = b[0] * a[2] - a[0] * b[2];
  c[2] = a[0] * b[1] - b[0] * a[1];
}

} // namespace coder

//
// File trailer for cross.cpp
//
// [EOF]
//
