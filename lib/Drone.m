% DRONE  Quadcopter drone model class.
%   This class implements a 6-DOF quadcopter simulation model with attitude
%   and position control. It inherits from handle to allow pass-by-reference
%   behavior, enabling the drone object to be updated in place.
%
%   The drone state includes position, velocity, Euler angles, and angular rates.
%   Control is achieved through PID controllers for attitude (roll, pitch, yaw),
%   vertical speed, and position. Motor mixing converts control commands
%   (total thrust and moments) into individual motor speeds with saturation.
%
%   See also: RPY2Rot, Clip

classdef Drone < handle
    %% MEMBERS
    properties
        % Simulation parameters
        g           % Gravity acceleration [m/s^2] (positive, assumed 9.81)
        t           % Current simulation time [s]
        dt          % Simulation time step [s] (fixed, 0.01)
        tf          % Final simulation time [s]
        
        % Physical properties
        m           % Mass of the drone [kg]
        l           % Arm length (distance from center to each motor) [m]
        I           % Inertia matrix (3x3) [kg*m^2]
        
        % State variables
        x           % Full state vector (12x1): [x y z dx dy dz phi theta psi p q r]'
        r           % Position vector [x; y; z] (extracted from x)
        dr          % Velocity vector [dx; dy; dz] (extracted from x)
        euler       % Euler angles [phi; theta; psi] (roll, pitch, yaw) [rad]
        w           % Angular rates [p; q; r] (body-fixed frame) [rad/s]
        
        dx          % Time derivative of state vector (used in Euler integration)
        
        % Control inputs
        u           % Control vector [T_sum; Mx; My; Mz]' (thrust and moments)
        T           % Total thrust [T_sum] (from u(1))
        M           % Moment vector [Mx; My; Mz] (from u(2:4))
    end
    
    properties
        % Attitude and vertical speed control variables
        phi_des       % Desired roll angle [rad]
        phi_err       % Current roll error [rad]
        phi_err_prev  % Previous roll error
        phi_err_sum   % Integrated roll error

        theta_des       % Desired pitch angle [rad]
        theta_err       % Current pitch error [rad]
        theta_err_prev  % Previous pitch error 
        theta_err_sum   % Integrated pitch error

        psi_des       % Desired yaw angle [rad]
        psi_err       % Current yaw error [rad]
        psi_err_prev  % Previous yaw error
        psi_err_sum   % Integrated yaw error
        
        zdot_des       % Desired vertical velocity [m/s]
        zdot_err       % Current vertical velocity error [m/s]
        zdot_err_prev  % Previous vertical velocity error
        zdot_err_sum   % Integrated vertical velocity error
    end
    
    properties
        % PID gains for attitude and vertical speed control
        kP_phi    % Proportional gain for roll
        kI_phi    % Integral gain for roll
        kD_phi    % Derivative gain for roll (used with rate feedback)
        
        kP_theta  % Proportional gain for pitch
        kI_theta  % Integral gain for pitch
        kD_theta  % Derivative gain for pitch
        
        kP_psi    % Proportional gain for yaw
        kI_psi    % Integral gain for yaw
        kD_psi    % Derivative gain for yaw
        
        kP_zdot   % Proportional gain for vertical velocity
        kI_zdot   % Integral gain for vertical velocity
        kD_zdot   % Derivative gain for vertical velocity
    end
    
    properties
        % Position control variables
        r_des       % Desired position [x; y; z] [m]
        dr_des      % Desired velocity [dx; dy; dz] [m/s]
        r_err       % Position error [m]
        r_err_prev  % Previous position error (unused)
        r_err_sum   % Integrated position error [m*s]
    end
    
    properties
        % PID gains for position control
        kP_pos  % Proportional gain vector [Px; Py; Pz]
        kI_pos  % Integral gain vector [Ix; Iy; Iz]
        kD_pos  % Derivative gain vector [Dx; Dy; Dz]
    end
    
    properties
        % Motor and aerodynamic properties
        kT        % Thrust coefficient [N/(rad/s)^2] (maps omega^2 to thrust)
        kQ        % Torque coefficient [Nm/(rad/s)^2] (maps omega^2 to reaction torque)
        maxOmega  % Maximum motor angular speed [rad/s]
        RPS2RPM   % Conversion factor from rad/s to RPM (30/pi)
        Omega     % Current motor speeds [omega1; omega2; omega3; omega4] [rad/s]
    end
    
    %% METHODS
    methods
        %% INITIALIZER
        % Constructor for Drone class.
        % Inputs:
        %   params     - containers.Map with keys: 'mass','armLength','Ixx','Iyy','Izz','kT','kQ','maxOmega'
        %   initStates - Initial state vector (12x1)
        %   initInputs - Initial control inputs [T; Mx; My; Mz] (4x1)
        %   gains      - containers.Map with PID gains (see main.m for keys)
        %   simTime    - Total simulation time [s]
        function obj = Drone(params, initStates, initInputs, gains, simTime)
            obj.g = 9.81;
            obj.t = 0.0;
            obj.dt = 0.01;
            obj.tf = simTime;
            
            obj.m = params('mass');
            obj.l = params('armLength');
            obj.I = [params('Ixx'), 0, 0; ...
                     0, params('Iyy'), 0; ...
                     0, 0, params('Izz')];
            
            obj.x = initStates;
            obj.r = obj.x(1:3);
            obj.dr = obj.x(4:6);
            obj.euler = obj.x(7:9);
            obj.w = obj.x(10:12);
            
            obj.dx = zeros(12,1);
            
            obj.u = initInputs;
            obj.T = obj.u(1);
            obj.M = obj.u(2:4);
            
            % Initialize attitude control variables
            obj.phi_des = 0.0;   
            obj.phi_err = 0.0;   
            obj.phi_err_prev = 0.0;   
            obj.phi_err_sum = 0.0;

            obj.theta_des = 0.0;   
            obj.theta_err = 0.0;   
            obj.theta_err_prev = 0.0; 
            obj.theta_err_sum = 0.0;

            obj.psi_des = 0.0;   
            obj.psi_err = 0.0;   
            obj.psi_err_prev = 0.0;   
            obj.psi_err_sum = 0.0;

            obj.zdot_des = 0.0;   
            obj.zdot_err = 0.0;   
            obj.zdot_err_prev = 0.0;  
            obj.zdot_err_sum = 0.0;
            
            % Position control variables
            obj.r_des = zeros(3,1);
            obj.dr_des = zeros(3,1);
            obj.r_err = zeros(3,1);
            obj.r_err_prev = zeros(3,1);
            obj.r_err_sum = zeros(3,1);
            
            % Attitude gains
            obj.kP_phi   = gains('P_phi');
            obj.kI_phi   = gains('I_phi');
            obj.kD_phi   = gains('D_phi');
            
            obj.kP_theta = gains('P_theta');
            obj.kI_theta = gains('I_theta');
            obj.kD_theta = gains('D_theta');
            
            obj.kP_psi   = gains('P_psi');
            obj.kI_psi   = gains('I_psi');
            obj.kD_psi   = gains('D_psi');
            
            obj.kP_zdot  = gains('P_zdot');
            obj.kI_zdot  = gains('I_zdot');
            obj.kD_zdot  = gains('D_zdot');
            
            % Position gains
            obj.kP_pos = gains('P_pos');
            obj.kI_pos = gains('I_pos');
            obj.kD_pos = gains('D_pos');
            
            % Motor properties
            obj.kT       = 1.22e-5;
            obj.kQ       = 2.19e-7;
            obj.maxOmega = (10000 * pi) / 30;  % Equivalent to 10000 RPM
            obj.RPS2RPM  = 30 / pi;
            obj.Omega    = zeros(4,1);
        end
        
        %% RETURNS DRONE STATE
        % Returns the current full state vector.
        % Output:
        %   state - 12x1 vector [x y z dx dy dz phi theta psi p q r]'
        function state = GetState(obj)
            state = obj.x;
        end
        
        %% STATE SPACE (DIFFERENTIAL) EQUATIONS
        % Evaluates the time derivatives of the state vector based on current state and inputs.
        % This method implements the 6-DOF rigid body dynamics of a quadcopter.
        % It updates obj.dx, which is then used by UpdateState for Euler integration.
        %
        % The dynamics include:
        %   - Translational kinematics: dr/dt = velocity.
        %   - Translational dynamics: dv/dt = (1/m)*(gravity force + thrust force).
        %   - Rotational kinematics: Euler angle rates from body angular rates (using ZYX convention).
        %   - Rotational dynamics: Euler's equation for rigid body (body frame).
        %
        % Note: The rotation matrix used is R = (RPY2Rot(obj.euler))', which transforms vectors
        %       from the body frame to the inertial (world) frame. This matches the convention
        %       that thrust (in body -z direction) is rotated to inertial.
        function obj = EvalEOM(obj)
            bRi = RPY2Rot(obj.euler);  % Rotation matrix from inertial to body
            R   = bRi';                % Transpose -> rotation from body to inertial
            
            % Translational motions
            obj.dx(1:3) = obj.dr;      % Time derivative of position = velocity
            % Acceleration in inertial frame: (1/m)*(weight + thrust)
            % Weight is [0; 0; m*g] in inertial (gravity points down, but we defined g positive)
            % Thrust force in body is [0; 0; -T] (acts upward in body -z), transformed to inertial by R
            obj.dx(4:6) = 1 / obj.m * ([0; 0; obj.m * obj.g] + R * obj.T * [0; 0; -1]);
            
            % Rotational kinematics: relate body angular rates (p,q,r) to Euler angle rates (phi_dot, theta_dot, psi_dot)
            % Using the standard ZYX Euler angle convention (see page 11 of notes)
            phi   = obj.euler(1);
            theta = obj.euler(2);
            obj.dx(7:9) = [1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
                           0, cos(phi),            -sin(phi);
                           0, sin(phi)*sec(theta),  cos(phi)*sec(theta)] * obj.w;
            
            % Rotational dynamics: Euler's equation for rigid body in body frame
            % I * dw/dt = M - w × (I * w)
            obj.dx(10:12) = (obj.I) \ (obj.M - cross(obj.w, obj.I * obj.w));
        end
        
        %% PREDICT NEXT DRONE STATE
        % Advances the simulation by one time step using Euler integration.
        % Calls EvalEOM to compute derivatives, then updates state.
        function obj = UpdateState(obj)
            obj.t = obj.t + obj.dt;
            
            % Find (update) the next state of obj.X
            % Euler method
            obj.EvalEOM();
            obj.x = obj.x + obj.dx .* obj.dt;
            
            % Update derived state variables for convenience
            obj.r     = obj.x(1:3);
            obj.dr    = obj.x(4:6);
            obj.euler = obj.x(7:9);
            obj.w     = obj.x(10:12);
        end
        
        %% CONTROLLER
        % Attitude and vertical speed controller.
        % Input:
        %   refSig - 4x1 reference signal [phi_des; theta_des; psi_des; zdot_des]
        % Computes control moments (Mx, My, Mz) and total thrust (T) using PID,
        % then calls MotorMixing to compute individual motor speeds.
        function obj = AttitudeCtrl(obj, refSig)
            obj.phi_des   = refSig(1);
            obj.theta_des = refSig(2);
            obj.psi_des   = refSig(3);
            obj.zdot_des  = refSig(4);
            
            % Error calculation (pg15)
            obj.phi_err   = obj.phi_des   - obj.euler(1);
            obj.theta_err = obj.theta_des - obj.euler(2);
            obj.psi_err   = obj.psi_des   - obj.euler(3);
            obj.zdot_err  = obj.zdot_des  - obj.dr(3);  % Error in vertical velocity
            
            % PID for roll (moment Mx)
            % Derivative term uses negative of body roll rate (p) as a rate feedback (common practice)
            obj.u(2) = (obj.kP_phi * obj.phi_err + ...
                        obj.kI_phi * (obj.phi_err_sum) + ...
                        obj.kD_phi * (0 - obj.w(1)));  % With small angle Approx. Recommended. Rate damping (p)
                        % obj.kD_phi * (obj.phi_err - obj.phi_err_prev)/obj.dt);
                        % (Diff. is not stable in micro_processors such as DSP, ARM)(Gain might differ)

            obj.phi_err_prev = obj.phi_err;
            obj.phi_err_sum  = obj.phi_err_sum + obj.phi_err;
            
            % PID for pitch (moment My)
            obj.u(3) = (obj.kP_theta * obj.theta_err + ...
                        obj.kI_theta * (obj.theta_err_sum) + ...
                        obj.kD_theta * (0 - obj.w(2)));  % Rate damping (q)
                        % obj.kD_theta * (obj.theta_err - obj.theta_err_prev)/obj.dt);

            obj.theta_err_prev = obj.theta_err;
            obj.theta_err_sum  = obj.theta_err_sum + obj.theta_err;
            
            % PID for yaw (moment Mz)
            obj.u(4) = (obj.kP_psi * obj.psi_err + ...
                        obj.kI_psi * (obj.psi_err_sum) + ...
                        obj.kD_psi * (0 - obj.w(3)));  % Rate damping (r)
                        % obj.kD_psi * (obj.psi_err - obj.psi_err_prev)/obj.dt);

            obj.psi_err_prev = obj.psi_err;
            obj.psi_err_sum  = obj.psi_err_sum + obj.psi_err;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Total thrust calculation (simplified: uses vertical velocity PID)
            % This assumes that the vertical acceleration is directly controlled by thrust.
            % The basic thrust is set to hover (m*g) plus correction from vertical velocity error.
            obj.u(1) = obj.m * obj.g - ...
                       (obj.kP_zdot * obj.zdot_err + ...
                        obj.kI_zdot * (obj.zdot_err_sum) + ...
                        obj.kD_zdot * (obj.zdot_err - obj.zdot_err_prev)/obj.dt);

            obj.zdot_err_prev = obj.zdot_err;
            obj.zdot_err_sum  = obj.zdot_err_sum + obj.zdot_err;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Alternative thrust commands (commented out) for testing.
            % obj.u(1) = 0.0;
            % obj.u(1) = obj.m * obj.g;  % Initial pose for hover
            % obj.u(2) = 0.0;
            % obj.u(3) = 0.0;
            % obj.u(4) = 0.0;
            
            obj.T = obj.u(1);
            obj.M = obj.u(2:4);

            obj.MotorMixing();  % Convert control commands to motor speeds
        end
        
        %% MOTOR MIXING
        % Computes individual motor speeds from total thrust and moments.
        % Uses a mixing matrix for an X‑configuration quadcopter.
        % Motors 1..4 are arranged as: FR, FL, RL, RR (Front-Right, Front-Left, Rear-Left, Rear-Right)
        % Rotation directions: assumed CW- for M1 and M3? The mixing matrix determines signs.
        % The matrix A relates [T; Mx; My; Mz] to [omega1^2; omega2^2; omega3^2; omega4^2].
        % After solving, each omega^2 is saturated and square‑rooted to get omega.
        function obj = MotorMixing(obj)
            % Projected distance for X configuration (d = l/sqrt(2))
            d = obj.l / sqrt(2);
            
            % Mixing Matrix for Configuration X (see reference pg10)
            % CW-, CCW+  (indicates direction of rotation; affects yaw sign)
            A = [ obj.kT,   obj.kT,    obj.kT,    obj.kT;    % Total thrust (all positive)
                 -obj.kT*d, obj.kT*d,  obj.kT*d, -obj.kT*d;  % Roll moment Mx
                  obj.kT*d, obj.kT*d, -obj.kT*d, -obj.kT*d;  % Pitch moment My
                  obj.kQ,  -obj.kQ,    obj.kQ,   -obj.kQ ];  % Yaw moment Mz (alternating signs)
            
            % Solve for squared motor speeds: Omega^2 = A \ u
            Omega2 = A \ [obj.u(1); obj.u(2); obj.u(3); obj.u(4)];
            
            % Apply saturation (0 to maxOmega^2) and square root
            for i = 1:4
                val_saturado = Clip(Omega2(i), 0, obj.maxOmega^2);
                obj.Omega(i) = sqrt(val_saturado);
            end
            
            % Optional debug print
            % fprintf('t=%.3f | My=%.4f | RPM: M1=%.0f  M2=%.0f  M3=%.0f  M4=%.0f\n', ...
            %     obj.t, obj.M(2), ...
            %     obj.Omega(1) * obj.RPS2RPM, ...
            %     obj.Omega(2) * obj.RPS2RPM, ...
            %     obj.Omega(3) * obj.RPS2RPM, ...
            %     obj.Omega(4) * obj.RPS2RPM);
        end
        
        %% POSITION CONTROLLER
        % Outer loop position controller that computes desired attitude and vertical velocity
        % based on position, velocity, and acceleration references.
        % Inputs:
        %   Xd - Desired position [x; y; z] [m]
        %   Vd - Desired velocity [dx; dy; dz] [m/s]
        %   Ad - Desired acceleration [ax; ay; az] [m/s^2] (feedforward)
        % The controller uses PID on position with velocity damping and acceleration feedforward
        % to compute a commanded acceleration vector. This is then mapped to desired roll/pitch
        % using the small‑angle approximation (theta ~ -ax/g, phi ~ ay/g). Vertical channel is
        % handled by a P controller on z error to produce zdot_des.
        function obj = PositionCtrl(obj, Xd, Vd, Ad)
        
            % Store desired states
            obj.r_des  = Xd;
            obj.dr_des = Vd;
        
            % Compute errors
            obj.r_err = obj.r_des - obj.r;
            dr_err    = obj.dr_des - obj.dr;
        
            % Integrate position error
            obj.r_err_sum = obj.r_err_sum + obj.r_err * obj.dt;
        
            % PID + feedforward acceleration command
            a_cmd = obj.kP_pos .* obj.r_err + ...
                    obj.kI_pos .* obj.r_err_sum + ...
                    obj.kD_pos .* dr_err + ...
                    Ad;
            
            % Saturate acceleration command to avoid extreme angles
            a_max = 5; % m/s^2
            a_cmd = Clip(a_cmd, -a_max, a_max);
        
            % Map horizontal acceleration to desired roll/pitch (small angle approximation)
            obj.theta_des = -a_cmd(1) / obj.g;  % Negative because positive pitch gives negative x acceleration. Check sign convention.
            obj.phi_des   =  a_cmd(2) / obj.g;
        
            % Vertical channel: desired vertical velocity (P control on z error)
            obj.zdot_des = Vd(3) + obj.kP_pos(3) * obj.r_err(3);
        
            % Saturation of attitude commands
            maxAngle = 20 * pi/180;
            obj.phi_des   = Clip(obj.phi_des,   -maxAngle, maxAngle);
            obj.theta_des = Clip(obj.theta_des, -maxAngle, maxAngle);
        
            % Send references to inner attitude controller
            refSig = [obj.phi_des; 
                      obj.theta_des; 
                      obj.psi_des; 
                      obj.zdot_des];
        
            obj.AttitudeCtrl(refSig);
        end

    end
end