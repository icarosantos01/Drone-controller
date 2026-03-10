//
// File: Drone_codegen.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 10-Mar-2026 14:00:27
//

// Include Files
#include "Drone_codegen.h"
#include "cos.h"
#include "minOrMax.h"
#include "mldivide.h"
#include "rt_nonfinite.h"
#include "sqrt.h"

// Function Definitions
//
// Arguments    : void
// Return Type  : Drone_codegen *
//
Drone_codegen *Drone_codegen::handle_init()
{
  return this;
}

//
// State Reference Assignment
//
// Arguments    : const double Xd[3]
//                const double Vd[3]
//                const double Ad[3]
// Return Type  : Drone_codegen *
//
Drone_codegen *Drone_codegen::PositionCtrl(const double Xd[3],
                                           const double Vd[3],
                                           const double Ad[3])
{
  Drone_codegen *obj;
  double A[16];
  double refSig[4];
  double dr_err[3];
  double dv[3];
  double d;
  double d1;
  double p_cmd;
  obj = this;
  //  POSITION CONTROLLER (Outer Loop)
  //  Computes desired attitude and vertical velocity
  //  based on position, velocity, and acceleration references.
  //  Inputs:
  //    Xd - Desired position [x; y; z] [m]
  //    Vd - Desired velocity [dx; dy; dz] [m/s]
  //    Ad - Desired acceleration [ax; ay; az] [m/s^2] (feedforward)
  //  The controller uses PID on position with velocity damping and acceleration
  //  feedforward to compute a commanded acceleration vector. This is then
  //  mapped to desired roll/pitch using the small‑angle approximation (theta ~
  //  -ax/g, phi ~ ay/g). Vertical channel is handled by a P controller on z
  //  error to produce zdot_des. Tracking Error Calculation Error Integration
  //  (Anti-steady-state error)
  p_cmd = obj->dt;
  //  Acceleration Command Generation
  //  Combines PID feedback with acceleration feedforward (Ad)
  obj->r_des[0] = Xd[0];
  obj->dr_des[0] = Vd[0];
  obj->r_err[0] = obj->r_des[0] - obj->r[0];
  d = obj->dr_des[0] - obj->dr[0];
  d1 = obj->r_err[0];
  obj->r_err_sum[0] += d1 * p_cmd;
  d = ((obj->kP_pos[0] * obj->r_err[0] + obj->kI_pos[0] * obj->r_err_sum[0]) +
       obj->kD_pos[0] * d) +
      Ad[0];
  dr_err[0] = d;
  obj->r_des[1] = Xd[1];
  obj->dr_des[1] = Vd[1];
  obj->r_err[1] = obj->r_des[1] - obj->r[1];
  d = obj->dr_des[1] - obj->dr[1];
  d1 = obj->r_err[1];
  obj->r_err_sum[1] += d1 * p_cmd;
  d = ((obj->kP_pos[1] * obj->r_err[1] + obj->kI_pos[1] * obj->r_err_sum[1]) +
       obj->kD_pos[1] * d) +
      Ad[1];
  dr_err[1] = d;
  obj->r_des[2] = Xd[2];
  obj->dr_des[2] = Vd[2];
  obj->r_err[2] = obj->r_des[2] - obj->r[2];
  d = obj->dr_des[2] - obj->dr[2];
  d1 = obj->r_err[2];
  obj->r_err_sum[2] += d1 * p_cmd;
  d = ((obj->kP_pos[2] * obj->r_err[2] + obj->kI_pos[2] * obj->r_err_sum[2]) +
       obj->kD_pos[2] * d) +
      Ad[2];
  dr_err[2] = d;
  //  Dynamic Clamping: Prevents extreme tilt angles during large setpoint
  //  changes These limits must be high enough to support the accelerations
  //  requested by the TrajectoryPlanner (Tseg).
  //  Max commanded acceleration [m/s^2]
  coder::internal::maximum2(dr_err, dv);
  coder::internal::minimum2(dv, dr_err);
  //  Map horizontal acceleration to desired roll/pitch (small angle
  //  approximation) theta_des approx -ax/g | phi_des approx ay/g
  obj->theta_des = -dr_err[0] / obj->g;
  //  Negative because positive pitch gives negative x acceleration. Check sign
  //  convention.
  obj->phi_des = dr_err[1] / obj->g;
  //  Vertical Channel Mapping
  //  Directly commands vertical velocity (z-dot) based on velocity
  //  feedforward and position error correction.
  obj->zdot_des = Vd[2] + obj->kP_pos[2] * obj->r_err[2];
  //  Saturation of attitude commands
  //  Limits maximum tilt to maintain lift and sensor validity
  //  Max allowed tilt [rad]
  p_cmd = obj->phi_des;
  obj->phi_des = coder::internal::minimum2(
      coder::internal::maximum2(p_cmd, -0.78539816339744828),
      0.78539816339744828);
  p_cmd = obj->theta_des;
  obj->theta_des = coder::internal::minimum2(
      coder::internal::maximum2(p_cmd, -0.78539816339744828),
      0.78539816339744828);
  //  Send references to inner attitude controller
  refSig[0] = obj->phi_des;
  refSig[1] = obj->theta_des;
  refSig[2] = obj->psi_des;
  refSig[3] = obj->zdot_des;
  //  Execute Inner Loop (Attitude Control)
  //  Setpoint Assignment
  //  CONTROLLER
  //  Attitude and vertical speed controller.
  //  Input:
  //    refSig - 4x1 reference signal [phi_des; theta_des; psi_des; zdot_des]
  //  Computes control moments (Mx, My, Mz) and total thrust (T) using PID,
  //  then calls MotorMixing to compute individual motor speeds.
  obj->phi_des = refSig[0];
  obj->theta_des = refSig[1];
  obj->psi_des = refSig[2];
  obj->zdot_des = refSig[3];
  //  Error calculation (pg15)
  obj->phi_err = obj->phi_des - obj->euler[0];
  obj->theta_err = obj->theta_des - obj->euler[1];
  obj->psi_err = obj->psi_des - obj->euler[2];
  obj->zdot_err = obj->zdot_des - obj->dr[2];
  //  Error in vertical velocity
  //  CASCADED PID FOR ATTITUDE (Roll, Pitch, Yaw) (pg15)
  //  The P-gain generates a commanded angular rate,
  //  and the D-gain acts on the rate error (Rate Feedback Control).
  //  PID for roll (moment Mx)
  p_cmd = obj->kP_phi * obj->phi_err;
  obj->u[1] =
      obj->kI_phi * obj->phi_err_sum + obj->kD_phi * (p_cmd - obj->w[0]);
  obj->phi_err_sum += obj->phi_err * obj->dt;
  //  PID for pitch (moment My)
  p_cmd = obj->kP_theta * obj->theta_err;
  obj->u[2] =
      obj->kI_theta * obj->theta_err_sum + obj->kD_theta * (p_cmd - obj->w[1]);
  obj->theta_err_sum += obj->theta_err * obj->dt;
  //  PID for yaw (moment Mz)
  p_cmd = obj->kP_psi * obj->psi_err;
  obj->u[3] =
      obj->kI_psi * obj->psi_err_sum + obj->kD_psi * (p_cmd - obj->w[2]);
  obj->psi_err_sum += obj->psi_err * obj->dt;
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  //  Total thrust calculation with tilt compensation
  //  As the drone tilts (Roll/Pitch) to move horizontally, the vertical
  //  component of the thrust vector decreases. To prevent the drone from
  //  losing altitude during aggressive maneuvers (like your 2s segments),
  //  we divide the hover thrust (m*g) by the cosine of the tilt angles.
  //  The 'max(..., 0.5)' is a safety guard to prevent division by zero
  //  or infinite thrust if the drone flips over.
  d = obj->euler[0];
  coder::b_cos(d);
  d1 = obj->euler[1];
  coder::b_cos(d1);
  p_cmd = obj->m * obj->g / coder::internal::maximum2(d * d1, 0.5);
  //  We add the PID correction based on vertical velocity error (zdot).
  //  Note: In this model's NED-like convention, a negative Z result
  //  usually implies upward movement, so the signs must match the
  //  specific coordinate frame.
  //  PID for Vertical Rate (Z-dot)
  //  Integrates velocity error to eliminate steady-state altitude offset.
  obj->zdot_err_sum += obj->zdot_err * obj->dt;
  obj->u[0] =
      p_cmd -
      ((obj->kP_zdot * obj->zdot_err + obj->kI_zdot * obj->zdot_err_sum) +
       obj->kD_zdot * (obj->zdot_err - obj->zdot_err_prev) / obj->dt);
  //  State update for next derivative calculation
  obj->zdot_err_prev = obj->zdot_err;
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  //  Alternative thrust commands (commented out) for testing.
  //  obj.u(1) = 0.0;
  //  obj.u(1) = obj.m * obj.g;  % Initial pose for hover
  //  obj.u(2) = 0.0;
  //  obj.u(3) = 0.0;
  //  obj.u(4) = 0.0;
  //  OUTPUT ASSIGNMENT & MIXING
  obj->T = obj->u[0];
  obj->M[0] = obj->u[1];
  obj->M[1] = obj->u[2];
  obj->M[2] = obj->u[3];
  //  Map control efforts (T, M) to physical Motor RPMs
  //  Projected distance for X configuration (d = l/sqrt(2))
  //  MOTOR MIXING (Control Allocation)
  //  Computes individual motor angular velocities (Omega)
  //  from total thrust (T) and moments (Mx, My, Mz).
  //
  //  Configuration: "X" geometry.
  //  Motor Order: 1:Front-Right (FR), 2:Front-Left (FL), 3:Rear-Left (RL),
  //  4:Rear-Right (RR). Rotation: M1, M3: CW (-) | M2, M4: CCW (+). Indicates
  //  direction of rotation; affects yaw sign. The matrix A relates [T; Mx; My;
  //  Mz] to [omega1^2; omega2^2; omega3^2; omega4^2]. After solving, each
  //  omega^2 is saturated and square‑rooted to get omega.
  p_cmd = obj->l / 1.4142135623730951;
  //  Mixing Matrix for Configuration X (see reference pg10)
  A[0] = obj->kT;
  A[4] = obj->kT;
  A[8] = obj->kT;
  A[12] = obj->kT;
  A[1] = -obj->kT * p_cmd;
  A[5] = obj->kT * p_cmd;
  A[9] = obj->kT * p_cmd;
  A[13] = -obj->kT * p_cmd;
  A[2] = obj->kT * p_cmd;
  A[6] = obj->kT * p_cmd;
  A[10] = -obj->kT * p_cmd;
  A[14] = -obj->kT * p_cmd;
  A[3] = obj->kQ;
  A[7] = -obj->kQ;
  A[11] = obj->kQ;
  A[15] = -obj->kQ;
  //  Total thrust (all positive)
  //  Roll moment Mx
  //  Pitch moment My
  //  Yaw moment Mz (alternating signs)
  //  Solve for squared motor speeds: Omega^2 = A \ u
  refSig[0] = obj->u[0];
  refSig[1] = obj->u[1];
  refSig[2] = obj->u[2];
  refSig[3] = obj->u[3];
  coder::b_mldivide(A, refSig);
  //  Apply saturation (0 to maxOmega^2) and square root
  p_cmd = obj->maxOmega;
  d = coder::internal::minimum2(
      coder::internal::maximum2(refSig[0], static_cast<double>(0.0)),
      p_cmd * p_cmd);
  coder::b_sqrt(d);
  obj->Omega[0] = d;
  p_cmd = obj->maxOmega;
  d = coder::internal::minimum2(
      coder::internal::maximum2(refSig[1], static_cast<double>(0.0)),
      p_cmd * p_cmd);
  coder::b_sqrt(d);
  obj->Omega[1] = d;
  p_cmd = obj->maxOmega;
  d = coder::internal::minimum2(
      coder::internal::maximum2(refSig[2], static_cast<double>(0.0)),
      p_cmd * p_cmd);
  coder::b_sqrt(d);
  obj->Omega[2] = d;
  p_cmd = obj->maxOmega;
  d = coder::internal::minimum2(
      coder::internal::maximum2(refSig[3], static_cast<double>(0.0)),
      p_cmd * p_cmd);
  coder::b_sqrt(d);
  obj->Omega[3] = d;
  //  Optional debug print
  //  fprintf('t=%.3f | My=%.4f | RPM: M1=%.0f  M2=%.0f  M3=%.0f  M4=%.0f\n',
  //  ...
  //      obj.t, obj.M(2), ...
  //      obj.Omega(1) * obj.RPS2RPM, ...
  //      obj.Omega(2) * obj.RPS2RPM, ...
  //      obj.Omega(3) * obj.RPS2RPM, ...
  //      obj.Omega(4) * obj.RPS2RPM);
  //  Convert control commands to motor speeds
  return obj;
}

//
// Arguments    : void
// Return Type  : Drone_codegen *
//
Drone_codegen *Drone_codegen::init()
{
  static const double dv[9]{0.00605922, 0.0, 0.0, 0.0,      0.00605922,
                            0.0,        0.0, 0.0, 0.0121184};
  Drone_codegen *obj;
  obj = this;
  //  INITIALIZER
  //  Constructor for Drone class.
  //  Inputs:
  //    params     - containers.Map with keys:
  //    'mass','armLength','Ixx','Iyy','Izz','kT','kQ','maxOmega' initStates -
  //    Initial state vector (12x1) initInputs - Initial control inputs [T; Mx;
  //    My; Mz] (4x1) gains      - containers.Map with PID gains (see main.m for
  //    keys) simTime    - Total simulation time [s]
  obj = obj->handle_init();
  obj->g = 9.81;
  obj->t = 0.0;
  obj->dt = 0.01;
  obj->m = 1.25;
  obj->l = 0.225;
  for (int i{0}; i < 9; i++) {
    obj->b_I[i] = dv[i];
  }
  for (int i{0}; i < 12; i++) {
    obj->x[i] = 0.0;
    obj->dx[i] = 0.0;
  }
  obj->r[0] = obj->x[0];
  obj->dr[0] = obj->x[3];
  obj->euler[0] = obj->x[6];
  obj->w[0] = obj->x[9];
  obj->r[1] = obj->x[1];
  obj->dr[1] = obj->x[4];
  obj->euler[1] = obj->x[7];
  obj->w[1] = obj->x[10];
  obj->r[2] = obj->x[2];
  obj->dr[2] = obj->x[5];
  obj->euler[2] = obj->x[8];
  obj->w[2] = obj->x[11];
  obj->u[0] = 0.0;
  obj->u[1] = 0.0;
  obj->u[2] = 0.0;
  obj->u[3] = 0.0;
  obj->T = obj->u[0];
  //  Initialize attitude control variables
  obj->phi_des = 0.0;
  obj->phi_err = 0.0;
  obj->phi_err_sum = 0.0;
  obj->theta_des = 0.0;
  obj->theta_err = 0.0;
  obj->theta_err_sum = 0.0;
  obj->psi_des = 0.0;
  obj->psi_err = 0.0;
  obj->psi_err_sum = 0.0;
  obj->zdot_des = 0.0;
  obj->zdot_err = 0.0;
  obj->zdot_err_prev = 0.0;
  obj->zdot_err_sum = 0.0;
  //  Position control variables
  //  Attitude gains
  obj->kP_phi = 4.5;
  obj->kI_phi = 0.0;
  obj->kD_phi = 0.4;
  obj->kP_theta = 4.5;
  obj->kI_theta = 0.0;
  obj->kD_theta = 0.4;
  obj->kP_psi = 2.0;
  obj->kI_psi = 0.0;
  obj->kD_psi = 0.2;
  obj->kP_zdot = 6.0;
  obj->kI_zdot = 0.1;
  obj->kD_zdot = 0.05;
  //  Position gains
  obj->M[0] = obj->u[1];
  obj->r_des[0] = 0.0;
  obj->dr_des[0] = 0.0;
  obj->r_err[0] = 0.0;
  obj->r_err_sum[0] = 0.0;
  obj->kP_pos[0] = 4.2;
  obj->kI_pos[0] = 0.93;
  obj->kD_pos[0] = 3.0;
  obj->M[1] = obj->u[2];
  obj->r_des[1] = 0.0;
  obj->dr_des[1] = 0.0;
  obj->r_err[1] = 0.0;
  obj->r_err_sum[1] = 0.0;
  obj->kP_pos[1] = 4.2;
  obj->kI_pos[1] = 0.93;
  obj->kD_pos[1] = 3.0;
  obj->M[2] = obj->u[3];
  obj->r_des[2] = 0.0;
  obj->dr_des[2] = 0.0;
  obj->r_err[2] = 0.0;
  obj->r_err_sum[2] = 0.0;
  obj->kP_pos[2] = 5.0;
  obj->kI_pos[2] = 1.0;
  obj->kD_pos[2] = 3.0;
  //  Motor properties
  obj->kT = 1.22E-5;
  obj->kQ = 2.19E-7;
  obj->maxOmega = 1047.1975512;
  //  10000 * (pi / 30) -> Equivalent to 10000 RPM
  obj->Omega[0] = 0.0;
  obj->Omega[1] = 0.0;
  obj->Omega[2] = 0.0;
  obj->Omega[3] = 0.0;
  return obj;
}

//
// File trailer for Drone_codegen.cpp
//
// [EOF]
//
