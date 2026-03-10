//
// File: Drone_codegen.h
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 10-Mar-2026 14:00:27
//

#ifndef DRONE_CODEGEN_H
#define DRONE_CODEGEN_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
class Drone_codegen {
public:
  Drone_codegen *PositionCtrl(const double Xd[3], const double Vd[3],
                              const double Ad[3]);
  Drone_codegen *init();

private:
  Drone_codegen *handle_init();

public:
  double g;
  double t;
  double dt;
  double m;
  double l;
  double b_I[9];
  double x[12];
  double r[3];
  double dr[3];
  double euler[3];
  double w[3];
  double dx[12];
  double u[4];
  double T;
  double M[3];
  double phi_des;
  double phi_err;
  double phi_err_sum;
  double theta_des;
  double theta_err;
  double theta_err_sum;
  double psi_des;
  double psi_err;
  double psi_err_sum;
  double zdot_des;
  double zdot_err;
  double zdot_err_prev;
  double zdot_err_sum;
  double kP_phi;
  double kI_phi;
  double kD_phi;
  double kP_theta;
  double kI_theta;
  double kD_theta;
  double kP_psi;
  double kI_psi;
  double kD_psi;
  double kP_zdot;
  double kI_zdot;
  double kD_zdot;
  double r_des[3];
  double dr_des[3];
  double r_err[3];
  double r_err_sum[3];
  double kP_pos[3];
  double kI_pos[3];
  double kD_pos[3];
  double kT;
  double kQ;
  double maxOmega;
  double Omega[4];
};

#endif
//
// File trailer for Drone_codegen.h
//
// [EOF]
//
