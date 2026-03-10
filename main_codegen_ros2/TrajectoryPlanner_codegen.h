//
// File: TrajectoryPlanner_codegen.h
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 10-Mar-2026 14:00:27
//

#ifndef TRAJECTORYPLANNER_CODEGEN_H
#define TRAJECTORYPLANNER_CODEGEN_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
class TrajectoryPlanner_codegen {
public:
  TrajectoryPlanner_codegen *init();
  double waypoints[12];
  double segmentTimes[3];
  coder::array<double, 3U> coefficients;
  double numSegments;
  double X_final[3];
  double hoverTime[4];
  boolean_T inHover;
  double hoverStartReal;
  double currentSegment;
  double tAccum;
};

#endif
//
// File trailer for TrajectoryPlanner_codegen.h
//
// [EOF]
//
