//
// File: main_codegen.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 10-Mar-2026 14:00:27
//

// Include Files
#include "main_codegen.h"
#include "Drone_codegen.h"
#include "TrajectoryPlanner_codegen.h"
#include "cos.h"
#include "cross.h"
#include "minOrMax.h"
#include "mldivide.h"
#include "mod.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "sec.h"
#include "sin.h"
#include "tan.h"
#include "validate_print_arguments.h"
#include "validator_check_type.h"
#include "coder_array.h"
#include <cmath>
#include <cstdio>

// Function Declarations
static double rt_powd_snf(double u0, double u1);

static double rt_roundd_snf(double u);

// Function Definitions
//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = std::pow(u0, u1);
    }
  }
  return y;
}

//
// Arguments    : double u
// Return Type  : double
//
static double rt_roundd_snf(double u)
{
  double y;
  if (std::abs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = std::floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = std::ceil(u - 0.5);
    }
  } else {
    y = u;
  }
  return y;
}

//
// Arguments    : coder::array<double, 2U> &drone1_state_out
// Return Type  : void
//
void main_codegen(coder::array<double, 2U> &drone1_state_out)
{
  static const signed char iv[12]{0, 3, 5, 6, 0, 3, 2, 3, 0, -3, -6, -3};
  static const signed char iv1[6]{1, 0, 0, 0, 0, 0};
  static const signed char iv2[6]{0, 1, 0, 0, 0, 0};
  static const signed char iv3[6]{0, 0, 2, 0, 0, 0};
  static const signed char iv4[3]{0, 0, 1};
  Drone_codegen drone1;
  TrajectoryPlanner_codegen planner;
  TrajectoryPlanner_codegen *obj;
  double dv[6];
  double Xd[3];
  double Xend[3];
  double Xstart[3];
  double R_1_tmp;
  double R_3_tmp;
  double Tseg;
  double b_R_1_tmp;
  double b_R_3_tmp;
  double d;
  double t_plan;
  double theta;
  int actual_steps;
  int i;
  int i1;
  int k;
  boolean_T b;
  boolean_T exitg1;
  //  Total desired simulation time
  //  Fixed maximum size (for codegen) - 100 s with dt=0.01
  //  Limit to avoid exceeding
  obj = planner.init();
  for (i = 0; i < 12; i++) {
    obj->waypoints[i] = iv[i];
  }
  obj->hoverTime[0] = 0.0;
  obj->hoverTime[1] = 0.0;
  obj->hoverTime[2] = 0.0;
  obj->hoverTime[3] = 10.0;
  obj->numSegments = 3.0;
  obj->X_final[0] = obj->waypoints[3];
  obj->segmentTimes[0] = 6.0;
  obj->X_final[1] = obj->waypoints[7];
  obj->segmentTimes[1] = 6.0;
  obj->X_final[2] = obj->waypoints[11];
  obj->segmentTimes[2] = 6.0;
  //  durations of segments 1→2, 2→3, 3→4
  actual_steps = static_cast<int>(obj->numSegments);
  obj->coefficients.set_size(6, 3, actual_steps);
  actual_steps *= 18;
  for (i = 0; i < actual_steps; i++) {
    obj->coefficients[i] = 0.0;
  }
  d = obj->numSegments;
  i = static_cast<int>(d);
  for (k = 0; k < i; k++) {
    Xstart[0] = obj->waypoints[k];
    Xend[0] = obj->waypoints[k + 1];
    Xstart[1] = obj->waypoints[k + 4];
    Xend[1] = obj->waypoints[k + 5];
    Xstart[2] = obj->waypoints[k + 8];
    Xend[2] = obj->waypoints[k + 9];
    for (actual_steps = 0; actual_steps < 3; actual_steps++) {
      double dv1[36];
      Tseg = obj->segmentTimes[k];
      //  Constraint matrix A (6x6) for the boundary conditions:
      //  Rows 1-3: conditions at t = 0 (position, velocity, acceleration)
      //  Rows 4-6: conditions at t = T (position, velocity, acceleration)
      //  POLY5_SEGMENT  Generates coefficients for a quintic polynomial
      //  trajectory segment.
      //    coeff = poly5_segment(x0, xf, T) computes the coefficients of a
      //    5th-order polynomial that satisfies boundary conditions on position,
      //    velocity, and acceleration at the start and end of a time interval
      //    [0, T].
      //
      //    The polynomial is of the form:
      //        x(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
      //    for t in [0, T]. It is commonly used for smooth point-to-point
      //    motion planning in robotics and automation.
      //
      //    Boundary conditions:
      //        At t = 0:   x(0) = x0,   dx/dt(0) = 0,   d²x/dt²(0) = 0
      //        At t = T:   x(T) = xf,   dx/dt(T) = 0,   d²x/dt²(T) = 0
      //
      //    Inputs:
      //        x0 - Initial position (scalar)
      //        xf - Final position (scalar)
      //        T  - Duration of the segment (positive scalar)
      //
      //    Output:
      //        coeff - 6x1 column vector of polynomial coefficients
      //                [c0; c1; c2; c3; c4; c5] corresponding to the powers
      //                t^0, t^1, t^2, t^3, t^4, t^5.
      //
      //    The function sets up a linear system A * coeff = b, where the rows
      //    of A enforce the six boundary conditions. The matrix is inverted to
      //    obtain the coefficients.
      //  x(0) = c0
      //  dx/dt(0) = c1
      //  d²x/dt²(0) = 2*c2
      //  x(T) = c0 + c1*T + c2*T^2 + c3*T^3 + c4*T^4 + c5*T^5
      //  dx/dt(T) = c1 + 2*c2*T + 3*c3*T^2 + 4*c4*T^3 + 5*c5*T^4
      //  d²x/dt²(T) = 2*c2 + 6*c3*T + 12*c4*T^2 + 20*c5*T^3
      //  Right-hand side vector b (6x1) containing the desired boundary values
      //  Solve for the coefficients
      dv[0] = Xstart[actual_steps];
      dv[1] = 0.0;
      dv[2] = 0.0;
      dv[3] = Xend[actual_steps];
      dv[4] = 0.0;
      dv[5] = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        dv1[6 * i1] = iv1[i1];
        dv1[6 * i1 + 1] = iv2[i1];
        dv1[6 * i1 + 2] = iv3[i1];
      }
      dv1[3] = 1.0;
      dv1[9] = Tseg;
      d = Tseg * Tseg;
      dv1[15] = d;
      R_1_tmp = rt_powd_snf(Tseg, 3.0);
      dv1[21] = R_1_tmp;
      b_R_1_tmp = rt_powd_snf(Tseg, 4.0);
      dv1[27] = b_R_1_tmp;
      dv1[33] = rt_powd_snf(Tseg, 5.0);
      dv1[4] = 0.0;
      dv1[10] = 1.0;
      dv1[16] = 2.0 * Tseg;
      dv1[22] = 3.0 * d;
      dv1[28] = 4.0 * R_1_tmp;
      dv1[34] = 5.0 * b_R_1_tmp;
      dv1[5] = 0.0;
      dv1[11] = 0.0;
      dv1[17] = 2.0;
      dv1[23] = 6.0 * Tseg;
      dv1[29] = 12.0 * d;
      dv1[35] = 20.0 * R_1_tmp;
      coder::mldivide(dv1, dv);
      for (i1 = 0; i1 < 6; i1++) {
        obj->coefficients[(i1 + 6 * actual_steps) + 18 * k] = dv[i1];
      }
    }
  }
  //  Initialize state
  b = coder::internal::validator_check_type(false);
  obj->inHover = b;
  Tseg = coder::internal::b_validator_check_type(0.0);
  obj->hoverStartReal = Tseg;
  obj->currentSegment = coder::internal::b_validator_check_type(1.0);
  obj->tAccum = Tseg;
  drone1.init();
  drone1_state_out.set_size(10000, 12);
  for (i = 0; i < 120000; i++) {
    drone1_state_out[i] = 0.0;
  }
  //  Fixed size for codegen
  //  Allow first dimension to vary
  actual_steps = 0;
  t_plan = 0.0;
  //  Constantes para prints
  //  seconds between prints
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 10000)) {
    double R_2_tmp[9];
    double dv2[9];
    double dv3[9];
    double old_segment;
    double t_sim;
    double y;
    boolean_T old_hover;
    t_sim = ((static_cast<double>(k) + 1.0) - 1.0) * 0.01;
    //  real simulation time
    //  --- Save state BEFORE calling getReference ---
    old_segment = planner.currentSegment;
    old_hover = planner.inHover;
    int exitg2;
    do {
      exitg2 = 0;
      if (planner.currentSegment > planner.numSegments) {
        Xd[0] = planner.X_final[0];
        Xstart[0] = 0.0;
        Xend[0] = 0.0;
        Xd[1] = planner.X_final[1];
        Xstart[1] = 0.0;
        Xend[1] = 0.0;
        Xd[2] = planner.X_final[2];
        Xstart[2] = 0.0;
        Xend[2] = 0.0;
        exitg2 = 1;
      } else if (planner.inHover) {
        //  Check hover end using REAL time
        if (t_sim - planner.hoverStartReal >=
            planner.hoverTime[static_cast<int>(planner.currentSegment + 1.0) -
                              1]) {
          planner.inHover = b;
          Tseg =
              planner.tAccum +
              planner
                  .segmentTimes[static_cast<int>(planner.currentSegment) - 1];
          planner.tAccum = coder::internal::b_validator_check_type(Tseg);
          Tseg = planner.currentSegment + 1.0;
          planner.currentSegment =
              coder::internal::b_validator_check_type(Tseg);
        } else {
          actual_steps = static_cast<int>(planner.currentSegment + 1.0);
          Xd[0] = planner.waypoints[actual_steps - 1];
          Xstart[0] = 0.0;
          Xend[0] = 0.0;
          Xd[1] = planner.waypoints[actual_steps + 3];
          Xstart[1] = 0.0;
          Xend[1] = 0.0;
          Xd[2] = planner.waypoints[actual_steps + 7];
          Xstart[2] = 0.0;
          Xend[2] = 0.0;
          exitg2 = 1;
        }
      } else {
        //  Not in hover → moving along current segment
        Tseg =
            planner.segmentTimes[static_cast<int>(planner.currentSegment) - 1];
        Tseg += planner.tAccum;
        if (t_plan < Tseg) {
          double C[18];
          double Xd_tmp[18];
          Tseg = t_plan - planner.tAccum;
          actual_steps = static_cast<int>(planner.currentSegment);
          for (i = 0; i < 3; i++) {
            for (i1 = 0; i1 < 6; i1++) {
              C[i1 + 6 * i] =
                  planner.coefficients[(i1 + 6 * i) + 18 * (actual_steps - 1)];
            }
          }
          for (i = 0; i < 6; i++) {
            Xd_tmp[3 * i] = C[i];
            Xd_tmp[3 * i + 1] = C[i + 6];
            Xd_tmp[3 * i + 2] = C[i + 12];
          }
          dv[0] = 1.0;
          dv[1] = Tseg;
          d = Tseg * Tseg;
          dv[2] = d;
          R_1_tmp = rt_powd_snf(Tseg, 3.0);
          dv[3] = R_1_tmp;
          b_R_1_tmp = rt_powd_snf(Tseg, 4.0);
          dv[4] = b_R_1_tmp;
          dv[5] = rt_powd_snf(Tseg, 5.0);
          for (i = 0; i < 3; i++) {
            theta = 0.0;
            for (i1 = 0; i1 < 6; i1++) {
              theta += Xd_tmp[i + 3 * i1] * dv[i1];
            }
            Xd[i] = theta;
          }
          dv[0] = 0.0;
          dv[1] = 1.0;
          dv[2] = 2.0 * Tseg;
          dv[3] = 3.0 * d;
          dv[4] = 4.0 * R_1_tmp;
          dv[5] = 5.0 * b_R_1_tmp;
          for (i = 0; i < 3; i++) {
            b_R_1_tmp = 0.0;
            for (i1 = 0; i1 < 6; i1++) {
              b_R_1_tmp += Xd_tmp[i + 3 * i1] * dv[i1];
            }
            Xstart[i] = b_R_1_tmp;
          }
          dv[0] = 0.0;
          dv[1] = 0.0;
          dv[2] = 2.0;
          dv[3] = 6.0 * Tseg;
          dv[4] = 12.0 * d;
          dv[5] = 20.0 * R_1_tmp;
          for (i = 0; i < 3; i++) {
            d = 0.0;
            for (i1 = 0; i1 < 6; i1++) {
              d += Xd_tmp[i + 3 * i1] * dv[i1];
            }
            Xend[i] = d;
          }
          exitg2 = 1;

          //  Segment finished
        } else if (planner.hoverTime[static_cast<int>(planner.currentSegment +
                                                      1.0) -
                                     1] > 0.0) {
          planner.inHover = coder::internal::validator_check_type(true);
          //  (optional, not used)
          planner.hoverStartReal =
              coder::internal::b_validator_check_type(t_sim);
          //  store real time
          actual_steps = static_cast<int>(planner.currentSegment + 1.0);
          Xd[0] = planner.waypoints[actual_steps - 1];
          Xstart[0] = 0.0;
          Xend[0] = 0.0;
          Xd[1] = planner.waypoints[actual_steps + 3];
          Xstart[1] = 0.0;
          Xend[1] = 0.0;
          Xd[2] = planner.waypoints[actual_steps + 7];
          Xstart[2] = 0.0;
          Xend[2] = 0.0;
          exitg2 = 1;
        } else {
          //  No hover → advance to next segment and loop
          planner.tAccum = coder::internal::b_validator_check_type(Tseg);
          Tseg = planner.currentSegment + 1.0;
          planner.currentSegment =
              coder::internal::b_validator_check_type(Tseg);
        }
      }
    } while (exitg2 == 0);
    drone1.PositionCtrl(Xd, Xstart, Xend);
    //  PREDICT NEXT DRONE STATE
    //  Advances the simulation by one time step using Euler integration.
    //  Calls EvalEOM to compute derivatives, then updates state.
    drone1.t += drone1.dt;
    //  Find (update) the next state of obj.X
    //  Euler method
    //  STATE SPACE (DIFFERENTIAL) EQUATIONS
    //  Evaluates the time derivatives of the state vector based on current
    //  state and inputs. This method implements the 6-DOF rigid body dynamics
    //  of a quadcopter. It updates obj.dx, which is then used by UpdateState
    //  for Euler integration.
    //
    //  The dynamics include:
    //    - Translational kinematics: dr/dt = velocity.
    //    - Translational dynamics: dv/dt = (1/m)*(gravity force + thrust
    //    force).
    //    - Rotational kinematics: Euler angle rates from body angular rates
    //    (using ZYX convention).
    //    - Rotational dynamics: Euler's equation for rigid body (body frame).
    //
    //  Note: The rotation matrix used is R = (RPY2Rot(obj.euler))', which
    //  transforms vectors
    //        from the body frame to the inertial (world) frame. This matches
    //        the convention that thrust (in body -z direction) is rotated to
    //        inertial.
    Xstart[0] = drone1.euler[0];
    Xstart[1] = drone1.euler[1];
    Xstart[2] = drone1.euler[2];
    //  RPY2Rot  Converts roll-pitch-yaw Euler angles to a rotation matrix.
    //    bRi = RPY2Rot(angles) returns the rotation matrix that transforms a
    //    vector from the inertial (world) frame to the body-fixed frame,
    //    using the ZYX Euler angle convention (yaw, then pitch, then roll).
    //
    //    The rotation sequence is:
    //        1. Yaw (psi)   about the inertial z-axis.
    //        2. Pitch (theta) about the new y-axis (after yaw).
    //        3. Roll (phi)   about the new x-axis (after pitch).
    //
    //    The resulting matrix bRi satisfies: v_body = bRi * v_inertial.
    //    Its transpose (bRi') performs the inverse transformation (body to
    //    inertial).
    //
    //    Input:
    //        angles - 3-element vector [phi; theta; psi] (roll, pitch, yaw) in
    //        radians.
    //
    //    Output:
    //        bRi    - 3x3 rotation matrix (inertial to body).
    //
    //    Reference: See equation on page 17 of the course notes.
    //  Roll angle  [rad]
    //  Pitch angle [rad]
    //  Yaw angle   [rad]
    //  Elementary rotation matrices
    //  R_3: Yaw rotation about z-axis
    R_3_tmp = Xstart[2];
    coder::b_sin(R_3_tmp);
    b_R_3_tmp = Xstart[2];
    coder::b_cos(b_R_3_tmp);
    //  R_2: Pitch rotation about y-axis
    Tseg = Xstart[1];
    coder::b_sin(Tseg);
    theta = Xstart[1];
    coder::b_cos(theta);
    //  R_1: Roll rotation about x-axis
    b_R_1_tmp = Xstart[0];
    coder::b_sin(b_R_1_tmp);
    R_1_tmp = Xstart[0];
    coder::b_cos(R_1_tmp);
    //  Combined rotation: R = R_roll * R_pitch * R_yaw
    //  Rotation matrix from inertial to body
    //  Transpose -> rotation from body to inertial
    //  Translational motions
    //  Time derivative of position = velocity
    //  Acceleration in inertial frame: (1/m)*(weight + thrust)
    //  Weight is [0; 0; m*g] in inertial (gravity points down, but we defined g
    //  positive) Thrust force in body is [0; 0; -T] (acts upward in body -z),
    //  transformed to inertial by R
    y = drone1.T;
    dv2[1] = 0.0;
    dv2[4] = R_1_tmp;
    dv2[7] = b_R_1_tmp;
    dv2[2] = 0.0;
    dv2[5] = -b_R_1_tmp;
    dv2[8] = R_1_tmp;
    R_2_tmp[0] = theta;
    R_2_tmp[3] = 0.0;
    R_2_tmp[6] = -Tseg;
    drone1.dx[0] = drone1.dr[0];
    dv2[0] = 1.0;
    R_2_tmp[1] = 0.0;
    drone1.dx[1] = drone1.dr[1];
    dv2[3] = 0.0;
    R_2_tmp[4] = 1.0;
    drone1.dx[2] = drone1.dr[2];
    dv2[6] = 0.0;
    R_2_tmp[7] = 0.0;
    R_2_tmp[2] = Tseg;
    R_2_tmp[5] = 0.0;
    R_2_tmp[8] = theta;
    for (i = 0; i < 3; i++) {
      i1 = static_cast<int>(dv2[i]);
      d = dv2[i + 3];
      R_1_tmp = dv2[i + 6];
      for (actual_steps = 0; actual_steps < 3; actual_steps++) {
        dv3[i + 3 * actual_steps] =
            (static_cast<double>(i1) * R_2_tmp[3 * actual_steps] +
             d * R_2_tmp[3 * actual_steps + 1]) +
            R_1_tmp * R_2_tmp[3 * actual_steps + 2];
      }
    }
    R_2_tmp[0] = b_R_3_tmp;
    R_2_tmp[3] = R_3_tmp;
    R_2_tmp[6] = 0.0;
    R_2_tmp[1] = -R_3_tmp;
    R_2_tmp[4] = b_R_3_tmp;
    R_2_tmp[7] = 0.0;
    for (i = 0; i < 3; i++) {
      i1 = iv4[i];
      R_2_tmp[3 * i + 2] = i1;
      d = R_2_tmp[3 * i];
      R_1_tmp = R_2_tmp[3 * i + 1];
      for (actual_steps = 0; actual_steps < 3; actual_steps++) {
        dv2[i + 3 * actual_steps] =
            (dv3[actual_steps] * d + dv3[actual_steps + 3] * R_1_tmp) +
            dv3[actual_steps + 6] * static_cast<double>(i1);
      }
    }
    for (i = 0; i < 9; i++) {
      dv2[i] *= y;
    }
    for (i = 0; i < 3; i++) {
      Xstart[i] = (dv2[i] * 0.0 + dv2[i + 3] * 0.0) - dv2[i + 6];
    }
    Tseg = 1.0 / drone1.m;
    Xend[2] = drone1.m * drone1.g + Xstart[2];
    drone1.dx[3] = Tseg * Xstart[0];
    drone1.dx[4] = Tseg * Xstart[1];
    drone1.dx[5] = Tseg * Xend[2];
    //  Rotational kinematics: relate body angular rates (p,q,r) to Euler angle
    //  rates (phi_dot, theta_dot, psi_dot) Using the standard ZYX Euler angle
    //  convention (see page 11 of notes)
    Tseg = drone1.euler[0];
    theta = drone1.euler[1];
    d = theta;
    coder::b_tan(d);
    R_1_tmp = Tseg;
    coder::b_sin(R_1_tmp);
    coder::b_cos(Tseg);
    coder::sec(theta);
    Xend[0] = drone1.w[0];
    Xend[1] = drone1.w[1];
    Xend[2] = drone1.w[2];
    dv2[0] = 1.0;
    dv2[3] = R_1_tmp * d;
    dv2[6] = Tseg * d;
    dv2[1] = 0.0;
    dv2[4] = Tseg;
    dv2[7] = -R_1_tmp;
    dv2[2] = 0.0;
    dv2[5] = R_1_tmp * theta;
    dv2[8] = Tseg * theta;
    d = Xend[0];
    R_1_tmp = Xend[1];
    b_R_1_tmp = Xend[2];
    for (i = 0; i < 3; i++) {
      drone1.dx[i + 6] =
          (dv2[i] * d + dv2[i + 3] * R_1_tmp) + dv2[i + 6] * b_R_1_tmp;
    }
    //  Rotational dynamics: Euler's equation for rigid body in body frame
    //  I * dw/dt = M - w × (I * w)
    Xend[0] = drone1.w[0];
    Xend[1] = drone1.w[1];
    Xend[2] = drone1.w[2];
    d = Xend[0];
    R_1_tmp = Xend[1];
    b_R_1_tmp = Xend[2];
    for (i = 0; i < 3; i++) {
      theta = drone1.b_I[i] * d;
      theta += drone1.b_I[i + 3] * R_1_tmp;
      theta += drone1.b_I[i + 6] * b_R_1_tmp;
      Xend[i] = theta;
    }
    coder::cross(drone1.w, Xend, Xstart);
    Xend[0] = drone1.M[0] - Xstart[0];
    Xend[1] = drone1.M[1] - Xstart[1];
    Xend[2] = drone1.M[2] - Xstart[2];
    coder::mldivide(drone1.b_I, Xend, Xstart);
    drone1.dx[9] = Xstart[0];
    drone1.dx[10] = Xstart[1];
    drone1.dx[11] = Xstart[2];
    //  Update derived state variables for convenience
    actual_steps = k + 1;
    //  RETURNS DRONE STATE
    //  Returns the current full state vector.
    //  Output:
    //    state - 12x1 vector [x y z dx dy dz phi theta psi p q r]'
    for (i = 0; i < 12; i++) {
      drone1.x[i] += drone1.dx[i] * drone1.dt;
      drone1_state_out[k + drone1_state_out.size(0) * i] = drone1.x[i];
    }
    drone1.r[0] = drone1.x[0];
    drone1.dr[0] = drone1.x[3];
    drone1.euler[0] = drone1.x[6];
    drone1.w[0] = drone1.x[9];
    drone1.r[1] = drone1.x[1];
    drone1.dr[1] = drone1.x[4];
    drone1.euler[1] = drone1.x[7];
    drone1.w[1] = drone1.x[10];
    drone1.r[2] = drone1.x[2];
    drone1.dr[2] = drone1.x[5];
    drone1.euler[2] = drone1.x[8];
    drone1.w[2] = drone1.x[11];
    //  RETURNS DRONE STATE
    //  Returns the current full state vector.
    //  Output:
    //    state - 12x1 vector [x y z dx dy dz phi theta psi p q r]'
    //  --- Check waypoint events (comparing with old) ---
    if ((!old_hover) && planner.inHover) {
      //  Entrou em hover: waypoint atingido
      Tseg = planner.currentSegment + 1.0;
      std::printf("\n=================================================\n");
      std::fflush(stdout);
      d = rt_roundd_snf(Tseg);
      if (d < 2.147483648E+9) {
        if (d >= -2.147483648E+9) {
          i = static_cast<int>(d);
        } else {
          i = MIN_int32_T;
        }
      } else if (d >= 2.147483648E+9) {
        i = MAX_int32_T;
      } else {
        i = 0;
      }
      std::printf("WAYPOINT %d REACHED (hover)\n",
                  coder::internal::validate_print_arguments(i));
      std::fflush(stdout);
    } else if (old_hover && (!planner.inHover)) {
      //  Saiu do hover: término do hover no waypoint anterior
      d = rt_roundd_snf(old_segment + 1.0);
      if (d < 2.147483648E+9) {
        if (d >= -2.147483648E+9) {
          i = static_cast<int>(d);
        } else {
          i = MIN_int32_T;
        }
      } else if (d >= 2.147483648E+9) {
        i = MAX_int32_T;
      } else {
        i = 0;
      }
      std::printf("WP Hover %d completed.\n\n",
                  coder::internal::validate_print_arguments(i));
      std::fflush(stdout);
    } else if ((planner.currentSegment > old_segment) && (!planner.inHover)) {
      //  Avançou de segmento sem hover (fly-through): waypoint atingido
      Tseg = planner.currentSegment;
      std::printf("\n=================================================\n");
      std::fflush(stdout);
      d = rt_roundd_snf(Tseg);
      if (d < 2.147483648E+9) {
        if (d >= -2.147483648E+9) {
          i = static_cast<int>(d);
        } else {
          i = MIN_int32_T;
        }
      } else if (d >= 2.147483648E+9) {
        i = MAX_int32_T;
      } else {
        i = 0;
      }
      std::printf("WAYPOINT %d REACHED (fly-through)\n",
                  coder::internal::validate_print_arguments(i));
      std::fflush(stdout);
    }
    //  Telemetry (every ~1 s or at start/end)
    if ((coder::b_mod(static_cast<double>(k) + 1.0) == 0.0) || (k + 1 == 1) ||
        (k + 1 == 6000)) {
      double validatedHoleFilling[7];
      double b_validatedHoleFilling[4];
      double OmegaRPM_idx_0;
      double OmegaRPM_idx_1;
      double OmegaRPM_idx_2;
      Xstart[0] = drone1.x[6] * 57.295779513082323;
      Xstart[1] = drone1.x[7] * 57.295779513082323;
      Xstart[2] = drone1.x[8] * 57.295779513082323;
      d = drone1.Omega[0];
      d *= 9.5492965855137211;
      OmegaRPM_idx_0 = d;
      d = drone1.Omega[1];
      d *= 9.5492965855137211;
      OmegaRPM_idx_1 = d;
      d = drone1.Omega[2];
      d *= 9.5492965855137211;
      OmegaRPM_idx_2 = d;
      d = drone1.Omega[3];
      d *= 9.5492965855137211;
      //  ---- Determine waypoints for display ----
      if (t_sim == 0.0) {
        //  Initial instant: current = 1 (origin), target = 2 (first
        //  destination)
        Tseg = 1.0;
        theta = 2.0;
      } else if (planner.currentSegment > planner.numSegments) {
        //  After last segment and hover completed: all at last waypoint
        Tseg = 4.0;
        theta = 4.0;
      } else if (planner.inHover) {
        //  During hover: at waypoint currentSegment+1, target is the same
        Tseg = planner.currentSegment + 1.0;
        theta = Tseg;
      } else {
        //  Normal movement: current = current segment, target = next waypoint
        Tseg = planner.currentSegment;
        theta = planner.currentSegment + 1.0;
      }
      //  Ensure indices do not exceed bounds
      Tseg =
          coder::internal::maximum2(1.0, coder::internal::minimum2(Tseg, 4.0));
      theta =
          coder::internal::maximum2(1.0, coder::internal::minimum2(theta, 4.0));
      //  --- Hover time to display: only shows if in hover, otherwise 0 ---
      if (planner.inHover) {
        b_R_3_tmp = planner.hoverTime[static_cast<int>(Tseg) - 1];
      } else {
        b_R_3_tmp = 0.0;
      }
      std::printf("-------------------------------------------------\n");
      std::fflush(stdout);
      std::printf("Time: %.1f s\n",
                  coder::internal::validate_print_arguments(t_sim));
      std::fflush(stdout);
      //  Display Current WP and fixed coordinates of Target WP
      y = planner.waypoints[static_cast<int>(Tseg) - 1];
      old_segment = planner.waypoints[static_cast<int>(Tseg) + 3];
      b_R_1_tmp = planner.waypoints[static_cast<int>(Tseg) + 7];
      R_1_tmp = planner.waypoints[static_cast<int>(theta) - 1];
      R_3_tmp = planner.waypoints[static_cast<int>(theta) + 3];
      Tseg = planner.waypoints[static_cast<int>(theta) + 7];
      coder::internal::validate_print_arguments(
          y, old_segment, b_R_1_tmp, R_1_tmp, R_3_tmp, Tseg, b_R_3_tmp,
          validatedHoleFilling);
      std::printf("Current WP: [%.1f %.1f %.1f]   Target WP: [%.1f %.1f %.1f]  "
                  " Hover: %.1fs\n",
                  validatedHoleFilling[0], validatedHoleFilling[1],
                  validatedHoleFilling[2], validatedHoleFilling[3],
                  validatedHoleFilling[4], validatedHoleFilling[5],
                  validatedHoleFilling[6]);
      std::fflush(stdout);
      coder::internal::validate_print_arguments(drone1.x[0], drone1.x[1],
                                                drone1.x[2], Xend);
      std::printf("Position : [%.3f  %.3f  %.3f] m\n", Xend[0], Xend[1],
                  Xend[2]);
      std::fflush(stdout);
      coder::internal::validate_print_arguments(drone1.x[3], drone1.x[4],
                                                drone1.x[5], Xend);
      std::printf("Velocity : [%.3f  %.3f  %.3f] m/s\n", Xend[0], Xend[1],
                  Xend[2]);
      std::fflush(stdout);
      coder::internal::validate_print_arguments(Xstart[0], Xstart[1], Xstart[2],
                                                Xend);
      std::printf("Attitude : [%.2f  %.2f  %.2f] deg\n", Xend[0], Xend[1],
                  Xend[2]);
      std::fflush(stdout);
      Xend[0] = drone1.x[0] - Xd[0];
      Xend[1] = drone1.x[1] - Xd[1];
      Xend[2] = drone1.x[2] - Xd[2];
      std::printf(
          "Tracking Error Norm: %.4f m\n",
          coder::internal::validate_print_arguments(coder::b_norm(Xend)));
      std::fflush(stdout);
      coder::internal::validate_print_arguments(OmegaRPM_idx_0, OmegaRPM_idx_1,
                                                OmegaRPM_idx_2, d,
                                                b_validatedHoleFilling);
      std::printf("Motors RPM: [%5.0f  %5.0f  %5.0f  %5.0f]\n",
                  b_validatedHoleFilling[0], b_validatedHoleFilling[1],
                  b_validatedHoleFilling[2], b_validatedHoleFilling[3]);
      std::fflush(stdout);
    }
    //  Stopping condition: mission complete (all segments + final hover)
    if ((planner.currentSegment > planner.numSegments) && (!planner.inHover)) {
      std::printf("\n=== MISSION COMPLETED SUCCESSFULLY ===\n");
      std::fflush(stdout);
      coder::internal::validate_print_arguments(drone1.x[0], drone1.x[1],
                                                drone1.x[2], Xend);
      std::printf("Final Position: [%.3f %.3f %.3f] m\n", Xend[0], Xend[1],
                  Xend[2]);
      std::fflush(stdout);
      std::printf("Total Simulation Time: %.1f s\n",
                  coder::internal::validate_print_arguments(t_sim));
      std::fflush(stdout);
      exitg1 = true;
    } else {
      //  --- Increment planner time ONLY if not in hover ---
      if (!planner.inHover) {
        t_plan += 0.01;
      }
      //  Safety: do not exceed N_max
      if (k + 1 >= 10000) {
        std::printf("\n=== Maximum step limit reached ===\n");
        std::fflush(stdout);
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  //  Trim output array to actual size
  for (i = 0; i < 12; i++) {
    for (i1 = 0; i1 < actual_steps; i1++) {
      drone1_state_out[i1 + actual_steps * i] =
          drone1_state_out[i1 + drone1_state_out.size(0) * i];
    }
  }
  drone1_state_out.set_size(actual_steps, 12);
}

//
// File trailer for main_codegen.cpp
//
// [EOF]
//
