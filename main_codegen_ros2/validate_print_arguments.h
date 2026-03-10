//
// File: validate_print_arguments.h
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 10-Mar-2026 14:00:27
//

#ifndef VALIDATE_PRINT_ARGUMENTS_H
#define VALIDATE_PRINT_ARGUMENTS_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
int validate_print_arguments(int varargin_1);

void validate_print_arguments(double varargin_1, double varargin_2,
                              double varargin_3, double varargin_4,
                              double varargin_5, double varargin_6,
                              double varargin_7, double validatedArguments[7]);

void validate_print_arguments(double varargin_1, double varargin_2,
                              double varargin_3, double varargin_4,
                              double validatedArguments[4]);

void validate_print_arguments(double varargin_1, double varargin_2,
                              double varargin_3, double validatedArguments[3]);

double validate_print_arguments(double varargin_1);

} // namespace internal
} // namespace coder

#endif
//
// File trailer for validate_print_arguments.h
//
// [EOF]
//
