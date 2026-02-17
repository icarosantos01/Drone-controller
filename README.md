# Quadcopter Simulation with Attitude and Position Control and Trajectory Planning (X‑Configuration)

This repository contains a complete MATLAB simulation of a quadcopter in **X‑configuration**. The simulation includes a nonlinear 6‑DOF dynamic model, cascaded PID controllers for attitude and position, a quintic polynomial trajectory planner, motor mixing for X‑config, and real‑time 3D visualization.

The simulation is designed as an educational tool to understand quadcopter dynamics, control design, and trajectory planning.

## Features

- **6‑DOF Dynamics**: Full rigid‑body equations of motion (Newton–Euler), including Coriolis terms and Euler angle kinematics.
- **Cascaded Control**: Inner‑loop attitude control (PID) and outer‑loop position control (PID + feedforward).
- **Trajectory Planning**: Smooth multi‑segment trajectories using quintic polynomials (continuous position, velocity, acceleration).
- **Waypoint Navigation**: Follow a list of waypoints; each waypoint can have a hover time.
- **X‑Configuration Motor Mixing**: Correct mapping from control commands (thrust and moments) to individual motor speeds.
- **Real‑time Visualization**: 3D animation of the drone and live plots of attitude, position, and motor RPM.
- **Modular Code**: Object‑oriented design (`Drone`, `TrajectoryPlanner`) for easy modification and extension.

## Control Architecture

The control system follows a standard cascaded structure:

- **Trajectory Planner** (`TrajectoryPlanner.m`): Generates reference position, velocity, and acceleration at each time step from a set of waypoints.
- **Position Controller** (outer loop): Computes desired roll, pitch, and vertical velocity from the position error using a PID + feedforward law.
- **Attitude Controller** (inner loop): Tracks desired angles and vertical velocity using independent PID controllers, producing the required total thrust and moments.
- **Motor Mixing**: Solves for the squared motor speeds from the thrust and moments, saturates them, and calculates the actual RPM.
- **Plant** (`Drone.m`): Integrates the 6‑DOF equations of motion to update the drone’s state.

## ⚙️ Motor Mixing (X-Configuration)

For an **X-configured** quadcopter, motors are placed at the corners of a square rotated by 45° relative to the body axes. 

* **Arm length ($l$):** Distance from center to motor.
* **Projected distance ($d$):** $d = l / \sqrt{2}$.
* **Coefficients:** $k_T$ (thrust) and $k_Q$ (torque).

### Thrust and Moments Matrix
The relationship between total thrust $T_\Sigma$, moments $[M_1, M_2, M_3]$ (roll, pitch, yaw), and squared motor speeds $\Omega_i^2$.

## File Structure

- `main.m` – Main simulation script; sets up parameters, runs the loop, and visualizes results.
- `Drone.m` – Class defining the quadcopter model, controllers, and motor mixing.
- `TrajectoryPlanner.m` – Class for generating quintic polynomial trajectories through waypoints.
- `RPY2Rot.m` – Converts roll‑pitch‑yaw angles to a rotation matrix (inertial to body).
- `poly5_segment.m` – Computes coefficients for a quintic polynomial segment with zero velocity/acceleration at ends.
- `Clip.m` – Utility function to clamp values within bounds.
- `WaypointConstraints.m` – (Optional) Checks if a position is inside a rectangular waypoint region.
- `lib/` – Folder containing the above files (added to path in `main.m`).

## Getting Started

### Prerequisites

- MATLAB R2016b or later (for object‑oriented features and `animatedline`).
- No additional toolboxes are required.

### Running the Simulation

1. Clone or download this repository.
2. Open MATLAB and navigate to the folder.
3. Run `main.m`.
4. Three figures will appear:
   - **Figure 1**: 3D animation of the drone (with axes oriented for a top‑down view).
   - **Figure 2**: Real‑time plots of attitude, vertical speed, and position (reference vs. actual).
   - **Figure 3**: Motor speeds in RPM.

The simulation will stop when all waypoints are reached (with a final hover) or after 60 seconds.

### Modifying the Trajectory

Edit the `waypoints` and `hoverTime` properties in the constructor of `TrajectoryPlanner.m`.

Each row is a waypoint [x, y, z] in meters. The trajectory will pass through them in order.

### Tuning Controllers

PID gains are set in main.m using a `containers.Map`. Adjust values to change the response.

### Extending the Project

- Add wind disturbances – modify the EvalEOM method to include external forces.
- Implement obstacle avoidance – integrate with a path planner.
- Replace PID with LQR or adaptive control.
- Add sensor noise to simulate real‑world measurements.
- Export data for post‑processing.

### This project is provided for educational purposes.
