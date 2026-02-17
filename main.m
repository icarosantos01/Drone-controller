% ============================================================
%  QUADCOPTER SIMULATION
%  Attitude control: roll, pitch, yaw, vertical speed
%  Position control: x, y, z
%  Trajectory planning
% ============================================================

close all;
clc;
clear;

addpath('./lib')   % Add library folder to MATLAB path

% ============================================================
% CONSTANTS
% ============================================================

R2D = 180/pi;      % Radians to degrees conversion factor
D2R = pi/180;      % Degrees to radians conversion factor
dt  = 0.01;        % Simulation time step [s] (fixed-step integration)
RPS2RPM = 30/pi;   % Conversion from rad/s to RPM (revolutions per minute)

% ============================================================
% DRONE PARAMETERS
% ============================================================

% Physical parameters stored in a containers.Map for easy access
drone1_params = containers.Map( ...
    {'mass','armLength','Ixx','Iyy','Izz','kT','kQ','maxOmega'}, ...
    {1.25, 0.225, 0.00605922, 0.00605922, 0.0121184, ...
     1.22e-5, 2.19e-7, 1047.1975512});

% ============================================================
% INITIAL STATE
% ============================================================

% State vector definition:
% [x y z dx dy dz phi theta psi p q r]'
% Units: positions in meters, velocities in m/s, angles in radians, angular rates in rad/s
drone1_initStates = [ 0, 0, 0, ...       % Position [x, y, z]
                      0, 0, 0, ...       % Linear velocity [dx, dy, dz]
                      0, 0, 0, ...       % Euler angles [phi (roll), theta (pitch), psi (yaw)]
                      0, 0, 0]';         % Angular rates [p, q, r] (body-fixed frame)

% Initial control inputs [T, Mx, My, Mz]' (thrust and moments)
drone1_initInputs = [0, 0, 0, 0]';

% ============================================================
% DRONE GEOMETRY — X CONFIGURATION
% ============================================================

l = drone1_params('armLength');   % Distance from center to each motor [m]
d = l / sqrt(2);                  % Projection of arm length onto X and Y axes (for X‑configuration)

% Body model in homogeneous coordinates (4x6 matrix)
% Each column: [x; y; z; 1] for a point on the drone
drone1_body = [ ...
     d,  d,     0, 1;    % Motor 1 (Front-Right)
    +d, -d,     0, 1;    % Motor 2 (Front-Left)
    -d, -d,     0, 1;    % Motor 3 (Rear-Left)
    -d,  d,     0, 1;    % Motor 4 (Rear-Right)
     0,  0,     0, 1;    % Center of mass
     0,  0, -0.05, 1]';  % Payload

% ============================================================
% CONTROLLER GAINS
% ============================================================

% PID gains for attitude, vertical speed, and position controllers
% Stored in a containers.Map; values are tuned for this specific drone
drone1_gains = containers.Map( ...
    {'P_phi','I_phi','D_phi', ...
     'P_theta','I_theta','D_theta', ...
     'P_psi','I_psi','D_psi', ...
     'P_zdot','I_zdot','D_zdot', ...
     'P_pos','I_pos','D_pos'}, ...
    { ...
     0.65, 0.0, 0.2, ...  % Roll PID
     0.65, 0.0, 0.2, ...  % Pitch PID
     0.40, 0.0, 0.15, ... % Yaw PID
     10.0, 0.11, 0.0, ... % Vertical speed PID
     [1.0; 1.0; 2.0], [0.005; 0.005; 0.05], [2.6; 2.6; 1.0] });  % P_pos [x, y, z], I_pos [x, y, z], D_pos [x, y, z]

% ============================================================
% Trajectory Planner
% ============================================================

% Create an instance of the TrajectoryPlanner class
planner = TrajectoryPlanner();
WP = planner.waypoints;         % Extract the list of waypoints (N×3 matrix)  

numWP = size(WP,1);             % Number of waypoints
currentWP = 2;                  % Index of the current target waypoint (start from second waypoint. First is initial)
wpReached = false(numWP,1);     % Logical array marking which waypoints have been reached
wpHoverTimer = 0;               % Timer for hovering at waypoints that require a stop

% ============================================================
% SIMULATION SETUP
% ============================================================

simulationTime = 60;          % Total simulation time [s] (safety limit)
N = ceil(simulationTime/dt);  % Number of simulation steps (pre‑allocate if needed, but not used for pre‑allocation here)

t_now = 0;                    % Current simulation time [s]
t_plan = 0;                   % Current time in the trajectory plan [s] (may pause during hover)
i = 1;                        % Loop counter

% Create drone object (custom Drone class) with parameters, initial state, inputs, gains, and simulation time
drone1 = Drone(drone1_params, drone1_initStates, ...
               drone1_initInputs, drone1_gains, simulationTime);

% ============================================================
% 3D VISUALIZATION SETUP
% ============================================================

fig1 = figure('Position',[3 60 700 700]);  % Figure window for 3D animation
view(3);                                   % Default 3D view
axis equal;
grid on;
xlim([-5 5]); ylim([-5 5]); zlim([-8 0]);    % World limits (note z axis is inverted for display)
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Height [m]');  
set(gca,'ZDir','reverse','YDir','reverse');  % Reverse Z and Y directions to have height increase downward (visual convenience)

% --- Alternative views ---
% view([-60 30]);
% view([90 0]);    % To check phi (roll attitude) only
% view([0 0]);     % To check theta (pitch attitude) only
% view([270 90]);  % Top view
hold on;

% --- Drone Pose Transformation ---
% Get current state (pos, vel, angles)
drone1_state = drone1.GetState();
% Construct Homogeneous Transformation Matrix (Body to World)
wHb = [RPY2Rot(drone1_state(7:9))' drone1_state(1:3); 0 0 0 1];
% Transform body coordinates to world coordinates
drone1_world = wHb * drone1_body;
% Extract 3D position/geometry for rendering
drone1_atti  = drone1_world(1:3,:);

% --- Drone drawing ---
% Motors 1 and 3 (front‑right to rear‑left)
h13 = plot3(drone1_atti(1,[1 3]), drone1_atti(2,[1 3]), drone1_atti(3,[1 3]), '-ro','MarkerSize',5);
% Motors 2 and 4 (front‑left to rear‑right)
h24 = plot3(drone1_atti(1,[2 4]), drone1_atti(2,[2 4]), drone1_atti(3,[2 4]), '-bo','MarkerSize',5);
% Payload line
hpl = plot3(drone1_atti(1,[5 6]), drone1_atti(2,[5 6]), drone1_atti(3,[5 6]), '-k','MarkerSize',3);
% Shadow/marker at ground projection (will be updated)
hsh = plot3(0,0,0,'xk','LineWidth',2);
hold off;

% ============================================================
% PLOTS (REFERENCE VS REAL)
% ============================================================

fig2 = figure('Position',[705 380 810 400]);  % Figure for reference tracking plots

% --- Attitude subplots (phi, theta, psi) in degrees ---
subplot(2,4,1); hold on; grid on;
ylabel('\phi [deg]');
h_phi     = animatedline('Color',[0,0.5,0],'LineWidth',1.5);                % Measured roll
h_phi_ref = animatedline('Color','k','LineStyle','--','LineWidth',1.2);     % Reference roll

subplot(2,4,2); hold on; grid on;
ylabel('\theta [deg]');
h_theta     = animatedline('Color',[0,0.5,0],'LineWidth',1.5);              % Measured pitch
h_theta_ref = animatedline('Color','k','LineStyle','--','LineWidth',1.2);   % Reference pitch

subplot(2,4,3); hold on; grid on;
ylabel('\psi [deg]');
h_psi     = animatedline('Color',[0,0.5,0],'LineWidth',1.5);                % Measured yaw
h_psi_ref = animatedline('Color','k','LineStyle','--','LineWidth',1.2);     % Reference yaw

% --- Vertical speed subplot ---
subplot(2,4,4); hold on; grid on;
ylabel('Zdot [m/s]');
h_zdot     = animatedline('Color',[0,0.5,0],'LineWidth',1.5);
h_zdot_ref = animatedline('Color','k','LineStyle','--','LineWidth',1.2);

% --- Position subplots (X, Y, Height) ---
subplot(2,4,5); hold on; grid on;
ylabel('X [m]');
h_x     = animatedline('Color','b','LineWidth',1.5);
h_x_ref = animatedline('Color','k','LineStyle','--','LineWidth',1.2);

subplot(2,4,6); hold on; grid on;
ylabel('Y [m]');
h_y     = animatedline('Color','b','LineWidth',1.5);
h_y_ref = animatedline('Color','k','LineStyle','--','LineWidth',1.2);

subplot(2,4,7); hold on; grid on;
ylabel('Height [m]');
h_z     = animatedline('Color','b','LineWidth',1.5);
h_z_ref = animatedline('Color','k','LineStyle','--','LineWidth',1.2);

% ============================================================
% MOTOR SPEED PLOTS (RPM)
% ============================================================
fig3 = figure('Position',[705 50 810 245], 'Name', 'Motor Speeds');
hold on; grid on;
ylabel('Rotation [RPM]');
xlabel('Time [s]');
h_m1 = animatedline('Color','r','LineWidth',1.5,'DisplayName','M1 (FR)');        % Motor 1 (Front‑Right)
h_m2 = animatedline('Color','b','LineWidth',1.5,'DisplayName','M2 (FL)');        % Motor 2 (Front‑Left)
h_m3 = animatedline('Color',[0,0.5,0],'LineWidth',1.5,'DisplayName','M3 (RL)');  % Motor 3 (Rear‑Left)
h_m4 = animatedline('Color','m','LineWidth',1.5,'DisplayName','M4 (RR)');        % Motor 4 (Rear‑Right)
legend('Location','eastoutside');

% ============================================================
% POSITION COMMAND (REFERENCE)
% ============================================================

commandSig = [0; 0; 0; 0];  % Initialize the variable to avoid plot errors (will be overwritten each loop)

% ============================================================
% MAIN LOOP
% ============================================================

hoverTimer = 0;                        % Timer for counting time at final position before mission completion
positionTolerance = 0.20;              % 20 cm tolerance for final position
velocityTolerance = 0.10;              % 0.1 m/s tolerance for final velocity
X_final = planner.getFinalPosition();  % Retrieve final target position from planner
drone1_state_out = zeros(N,12);        % Pre‑allocate array to store state history (trimmed later)

while true

    % --- Time ---
    t_now = (i-1) * dt;  % Current simulation time

    % ---Get reference from planner at current plan time ---
    [Xd, Vd, Ad] = planner.getReference(t_plan);  % Desired position, velocity, acceleration vectors

    % % ===== DEBUG PRINTS =====
    % if mod(i,100) == 0
    %     fprintf('t=%.2f | Vd norm=%.4f | Ad norm=%.4f\n', ...
    %             t_now, norm(Vd), norm(Ad));
    % end
    % % ======================

    % --- Position controller using trajectory ---
    drone1.PositionCtrl(Xd, Vd, Ad);  % Compute desired attitude and vertical speed based on position error
    
    commandSig = [drone1.phi_des;     % Collect desired commands for plotting
                  drone1.theta_des; 
                  drone1.psi_des; 
                  drone1.zdot_des];

    % --- Dynamics update ---
    drone1.UpdateState();                   % Integrate drone dynamics one time step
    drone1_state = drone1.GetState();       % Retrieve updated state

    drone1_state_out(i,:) = drone1_state';  % Store state history

    % --- Telemetry ---
    printInterval = 1.0;   % seconds between console prints
    if mod(i, round(printInterval/dt)) == 0
        
        fprintf('-------------------------------------------------\n');
        fprintf('Time: %.1f s\n', t_now);
        fprintf('Position  : [%.3f  %.3f  %.3f] m\n', ...
                drone1_state(1), ...
                drone1_state(2), ...
                drone1_state(3));
        fprintf('Velocity  : [%.3f  %.3f  %.3f] m/s\n', ...
                drone1_state(4), ...
                drone1_state(5), ...
                drone1_state(6));
        fprintf('Attitude  : [%.2f  %.2f  %.2f] deg\n', ...
                drone1_state(7)*R2D, ...
                drone1_state(8)*R2D, ...
                drone1_state(9)*R2D);

        pos_error = norm(drone1_state(1:3) - Xd);
        fprintf('Tracking Error Norm: %.4f m\n', pos_error);

        % --- Motor Speeds ---
        OmegaRPM = drone1.Omega * RPS2RPM;  % Convert motor speeds to RPM
        fprintf('Motors RPM: [%4.0f %4.0f %4.0f %4.0f] | Δmax = %.1f RPM\n', ...
                OmegaRPM(1), OmegaRPM(2), ...
                OmegaRPM(3), OmegaRPM(4), ...
                max(OmegaRPM)-min(OmegaRPM));
    end

    % --- 3D visualization update ---
    wHb = [RPY2Rot(drone1_state(7:9))' drone1_state(1:3); 0 0 0 1];  % Update transform
    drone1_world = wHb * drone1_body;                                % Transform body points
    drone1_atti  = drone1_world(1:3,:);                              % Extract Cartesian coordinates
    
    % --- Update plot data ---
    set(h13,'XData',drone1_atti(1,[1 3]),'YData',drone1_atti(2,[1 3]),'ZData',drone1_atti(3,[1 3]));
    set(h24,'XData',drone1_atti(1,[2 4]),'YData',drone1_atti(2,[2 4]),'ZData',drone1_atti(3,[2 4]));
    set(hpl,'XData',drone1_atti(1,[5 6]),'YData',drone1_atti(2,[5 6]),'ZData',drone1_atti(3,[5 6]));
    set(hsh,'XData',drone1_state(1),'YData',drone1_state(2));  % Ground marker only uses X,Y

    % --- Real-time plots ---
    addpoints(h_phi,     t_now, drone1_state(7)*R2D);
    addpoints(h_phi_ref, t_now, commandSig(1)*R2D);

    addpoints(h_theta,     t_now, drone1_state(8)*R2D);
    addpoints(h_theta_ref, t_now, commandSig(2)*R2D);

    addpoints(h_psi,     t_now, drone1_state(9)*R2D);
    addpoints(h_psi_ref, t_now, commandSig(3)*R2D);

    addpoints(h_zdot,     t_now, drone1_state(6));
    addpoints(h_zdot_ref, t_now, commandSig(4));

    addpoints(h_x, t_now, drone1_state(1));
    addpoints(h_x_ref, t_now, Xd(1));

    addpoints(h_y, t_now, drone1_state(2));
    addpoints(h_y_ref, t_now, Xd(2));

    % Note: Height is plotted as negative of Z because of reversed Z axis in 3D view
    addpoints(h_z, t_now, -drone1_state(3));
    addpoints(h_z_ref, t_now, -Xd(3));

    % --- Update Motor Speed Plots (WITH OFFSET TO SEE ALL 4 LINES) ---
    addpoints(h_m1, t_now, (drone1.Omega(1) * RPS2RPM));
    addpoints(h_m2, t_now, (drone1.Omega(2) * RPS2RPM));
    addpoints(h_m3, t_now, (drone1.Omega(3) * RPS2RPM));
    addpoints(h_m4, t_now, (drone1.Omega(4) * RPS2RPM));

    drawnow  % Force update of figures

    % --- Crash detection ---
    % If drone goes above ground (z > 0.1), assume crash. (Note: ground at z=0, positive up)
    if drone1_state(3) > 0.1
        msgbox('Drone Crashed!','Error','error');
        break;
    end

    % ============================================================
    % WAYPOINT PASS CHECK
    % ============================================================
    inHoverZone = false; 
    if currentWP <= numWP
        targetWP = WP(currentWP,:)';                    % Current target waypoint coordinates
        distWP = norm(drone1_state(1:3) - targetWP);    % Distance to waypoint
        velNorm = norm(drone1_state(4:6));              % Current speed
        
        hoverTimeWP = planner.getHoverTime(currentWP);  % Get hover duration for this waypoint (0 if fly‑through)
        
        % Dynamic criterion: if hover time > 0, require tighter tolerance (0.5 m and 0.5 m/s)
        % If fly‑through (hoverTime = 0), use larger radius (1.0 m) to avoid skipping.
        isReached = false;
        if hoverTimeWP > 0
            if distWP < 0.5 && velNorm < 0.5  % Tolerance of 0.5 meters and 0.5 m/s
                isReached = true;
            end
        else
            if distWP < 1.00
                isReached = true;
            end
        end
        
        if isReached
            if ~wpReached(currentWP)
                fprintf('\n=================================================\n');
                fprintf('WAYPOINT %d ATINGIDO\n', currentWP);
                fprintf('Tempo Sim: %.2f s | Tempo Plan: %.2f s\n', t_now, t_plan);
                wpReached(currentWP) = true;
            end
            
            if hoverTimeWP > 0
                inHoverZone = true; 
                wpHoverTimer = wpHoverTimer + dt;  % Increment hover timer
                
                if wpHoverTimer >= hoverTimeWP
                    fprintf('Hover do WP %d concluído.\n\n', currentWP);
                    currentWP = currentWP + 1;     % Move to next waypoint
                    wpHoverTimer = 0;              % Reset hover timer
                    inHoverZone = false;
                end
            else
                currentWP = currentWP + 1;  % Fly‑through: immediately go to next waypoint
            end
        end
    end

    % --- Trajectory clock increment ---
    if ~inHoverZone
        t_plan = t_plan + dt;               % Advance plan time only when not hovering
    end
 
    % ============================================================
    % MISSION COMPLETION LOGIC
    % ============================================================
    distFinal = norm(drone1_state(1:3) - X_final);  % Distance to final target
    velNorm   = norm(drone1_state(4:6));
    
    % Only allow mission completion if all waypoints have been passed (currentWP > numWP)
    % and the drone is within tolerance of final position and stationary.
    if currentWP > numWP && distFinal < positionTolerance && velNorm < velocityTolerance
        hoverTimer = hoverTimer + dt;
        fprintf('\n====================================================\n');
        fprintf('MISSION COMPLETED SUCCESSFULLY\n');
        fprintf('Final Position      : [%.3f  %.3f  %.3f] m\n', drone1_state(1:3));
        fprintf('Total Simulation Time: %.2f s\n', t_now);
        fprintf('====================================================\n');

        msgbox('Mission Completed Successfully!', 'Simulation Status', 'help');
        break;
    end

    if t_now > simulationTime
        warning('Simulation timeout.');
        break;
    end

    i = i + 1;  % Increment loop counter

end

% Trim the state output to the actual number of steps simulated

drone1_state_out = drone1_state_out(1:i-1,:);
