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

```mermaid
graph LR
    A[Trajectory Planner] -->|"(X,Y,Z)_cmd"| B[Position Controller]
    B -->|"u1(Tz)"| C[Attitude Controller]
    B -->|"(phi, psi, theta)_cmd"| C
    C -->|"u2, u3, u4"| D[Plant (Quadcopter)]
    D -->|"y1, Sensor measurements"| B
    D -->|"y1, Sensor measurements"| C

    style A fill:#f9f,stroke:#333,stroke-width:2px
    style B fill:#bbf,stroke:#333,stroke-width:2px
    style C fill:#bbf,stroke:#333,stroke-width:2px
    style D fill:#bfb,stroke:#333,stroke-width:2px
```

## Control Architecture

The control system follows a standard cascaded structure:

- **Trajectory Planner** (`TrajectoryPlanner.m`): Generates reference position, velocity, and acceleration at each time step from a set of waypoints.
- **Position Controller** (outer loop): Computes desired roll, pitch, and vertical velocity from the position error using a PID + feedforward law.
- **Attitude Controller** (inner loop): Tracks desired angles and vertical velocity using independent PID controllers, producing the required total thrust and moments.
- **Motor Mixing**: Solves for the squared motor speeds from the thrust and moments, saturates them, and calculates the actual RPM.
- **Plant** (`Drone.m`): Integrates the 6‑DOF equations of motion to update the drone’s state.

## Drone Model

The quadcopter is modeled as a rigid body with mass \(m\) and inertia matrix \(\text{diag}(I_{xx}, I_{yy}, I_{zz})\). The state vector is

\[
\mathbf{x} = [x,\ y,\ z,\ \dot{x},\ \dot{y},\ \dot{z},\ \phi,\ \theta,\ \psi,\ p,\ q,\ r]^T
\]

### Translational Dynamics

\[
m \ddot{\mathbf{r}} = m g \mathbf{e}_3 + \mathbf{R} \cdot T_\Sigma \cdot (-\mathbf{e}_3)
\]

where \(\mathbf{R}\) is the rotation matrix from body to inertial frame, and \(T_\Sigma\) is the total thrust.

### Rotational Dynamics

\[
\begin{aligned}
\dot{\phi} &= p + q \sin\phi \tan\theta + r \cos\phi \tan\theta \\
\dot{\theta} &= q \cos\phi - r \sin\phi \\
\dot{\psi} &= q \sin\phi \sec\theta + r \cos\phi \sec\theta
\end{aligned}
\]

\[
\begin{aligned}
\dot{p} &= \frac{I_{yy}-I_{zz}}{I_{xx}} q r + \frac{1}{I_{xx}} M_1 \\
\dot{q} &= \frac{I_{zz}-I_{xx}}{I_{yy}} r p + \frac{1}{I_{yy}} M_2 \\
\dot{r} &= \frac{I_{xx}-I_{yy}}{I_{zz}} p q + \frac{1}{I_{zz}} M_3
\end{aligned}
\]

## Motor Mixing (X‑Configuration)

For an **X‑configured** quadcopter, the four motors are placed at the corners of a square rotated by 45° relative to the body axes. The arm length is \(l\), and the projection onto the x‑ and y‑axes is \(d = l / \sqrt{2}\). The thrust coefficient is \(k_T\), and the torque coefficient is \(k_Q\). The relationship between the total thrust \(T_\Sigma\), the moments \([M_1, M_2, M_3]\) (roll, pitch, yaw), and the squared motor speeds \(\Omega_i^2\) is:

\[
\begin{bmatrix}
T_\Sigma \\ M_1 \\ M_2 \\ M_3
\end{bmatrix}
=
\begin{bmatrix}
k_T & k_T & k_T & k_T \\
-k_T d & k_T d & k_T d & -k_T d \\
k_T d & k_T d & -k_T d & -k_T d \\
k_Q & -k_Q & k_Q & -k_Q
\end{bmatrix}
\begin{bmatrix}
\Omega_1^2 \\ \Omega_2^2 \\ \Omega_3^2 \\ \Omega_4^2
\end{bmatrix}
\]

The signs correspond to the motor positions (1: front‑right, 2: front‑left, 3: rear‑left, 4: rear‑right) and their rotation directions (assumed alternating to cancel yaw torques). In the simulation, the matrix is inverted to solve for the required squared speeds:

\[
\begin{bmatrix}
\Omega_1^2 \\ \Omega_2^2 \\ \Omega_3^2 \\ \Omega_4^2
\end{bmatrix}
=
\begin{bmatrix}
k_T & k_T & k_T & k_T \\
-k_T d & k_T d & k_T d & -k_T d \\
k_T d & k_T d & -k_T d & -k_T d \\
k_Q & -k_Q & k_Q & -k_Q
\end{bmatrix}^{-1}
\begin{bmatrix}
T_\Sigma \\ M_1 \\ M_2 \\ M_3
\end{bmatrix}
\]

The resulting \(\Omega_i^2\) are then saturated between \(0\) and \(\Omega_{\max}^2\), and the motor speeds \(\Omega_i\) are obtained by taking the square root.

## File Structure

- `main.m` – Main simulation script; sets up parameters, runs the loop, and visualizes results.
- `Drone.m` – Class defining the quadcopter model, controllers, and motor mixing.
- `TrajectoryPlanner.m` – Class for generating quintic polynomial trajectories through waypoints.
- `RPY2Rot.m` – Converts roll‑pitch‑yaw angles to a rotation matrix (inertial → body).
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
