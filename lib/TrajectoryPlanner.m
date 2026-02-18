% TRAJECTORYPLANNER  Generates a trajectory through waypoints using quintic polynomials.
%   This class creates a multi‑segment trajectory from a list of waypoints.
%   Each segment is a 5th‑order polynomial (quintic) in time for each coordinate
%   (x, y, z), ensuring continuous position, velocity, and acceleration at the
%   segment boundaries (zero velocity and acceleration at start and end of each segment).
%   The class also stores hover times per waypoint, which can be used to pause
%   the trajectory at specific points.
%
%   Usage:
%       planner = TrajectoryPlanner();
%       [pos, vel, acc] = planner.getReference(t);
%
%   See also: poly5_segment

classdef TrajectoryPlanner < handle
    
    properties
        waypoints     % Nx3 matrix of waypoints [x y z] (N points)
        segmentTimes  % (N-1)x1 vector of durations for each segment [s]
        coefficients  % Cell array of size (N-1)x1, each cell containing a 6x3 matrix
                      % of polynomial coefficients for x, y, z (columns correspond to coordinates,
                      % rows to coefficients [c0; c1; c2; c3; c4; c5] for that coordinate)
        numSegments   % Number of segments (N-1)
        X_final       % Final position [x; y; z] (last waypoint)
        hoverTime     % (N)x1 vector of hover durations for each waypoint [s]
                      % (0 means no hover, used by the main simulation to pause at waypoints)
    end
    
    methods

        % Constructor
        % Initializes the trajectory planner with predefined waypoints, hover times,
        % and segment durations. Computes quintic polynomial coefficients for each segment.
        function obj = TrajectoryPlanner()

            % ==============================
            % DEFINE WAYPOINTS HERE (N x 3)
            % ==============================
            obj.waypoints = [
                0 0  0;
                3 3 -4;
                6 1 -2;
            ];
            
            % ==============================
            % DEFINE HOVER TIME FOR EACH WAYPOINT
            % (0 = no hover, duration in seconds)
            % ==============================
            obj.hoverTime = [0; 0; 10];
            
            % Store final position for quick access
            obj.X_final = obj.waypoints(end,:)';
            
            % Number of segments = number of waypoints minus one
            obj.numSegments = size(obj.waypoints,1) - 1;
            
            % ============================================================
            % TRAJECTORY TIMING CONFIGURATION
            % ============================================================
            % Tseg defines the duration (in seconds) for each flight segment.
            %
            % IMPORTANT: Lowering Tseg (e.g., to 2s) increases required 
            % acceleration and tilt angles. If you change this to a low value,
            % you MUST check/update the following in Drone.m:
            %   1. increase 'a_max' (e.g., 12.0)
            %   2. increase 'maxAngle' (e.g., 45 degrees)
            %   3. ensure 'thrust_base' tilt compensation is active.
            %
            % A value of 5s-6s provides a smoother, more stable trajectory.
            Tseg = 6;  % seconds per segment

            % Assign the same duration to all segments in the mission
            obj.segmentTimes = Tseg * ones(obj.numSegments,1);
            
            % Pre‑allocate cell array for coefficients
            obj.coefficients = cell(obj.numSegments,1);
            
            % Compute coefficients for each segment
            for k = 1:obj.numSegments
                
                % Start and end waypoints for this segment
                Xstart = obj.waypoints(k,:);
                Xend   = obj.waypoints(k+1,:);
                
                % Coefficient matrix: 6 rows (polynomial order 0..5) × 3 columns (x, y, z)
                C = zeros(6,3);
                
                % For each coordinate, compute quintic coefficients
                for i=1:3
                    C(:,i) = poly5_segment(Xstart(i), Xend(i), Tseg);
                end
                
                obj.coefficients{k} = C;
            end
        end
        
        % Get reference at time t
        % Returns the desired position, velocity, and acceleration vectors
        % at the given simulation time t.
        %
        % Input:
        %   t - Current time [s] (scalar, measured along the trajectory)
        %
        % Outputs:
        %   Xd - Desired position [x; y; z] [m]
        %   Vd - Desired velocity [dx; dy; dz] [m/s]
        %   Ad - Desired acceleration [ax; ay; az] [m/s²]
        %
        % If t exceeds the total duration of all segments, the outputs are
        % held constant at the final position (zero velocity and acceleration).
        function [Xd, Vd, Ad] = getReference(obj, t)
            
            t_accum = 0;  % accumulated time at the start of each segment
            
            for k = 1:obj.numSegments
                
                Tseg = obj.segmentTimes(k);
                
                if t <= t_accum + Tseg
                    % Current segment found
                    tau = t - t_accum;        % local time within segment [0, Tseg]
                    C = obj.coefficients{k};  % coefficients for this segment (6x3)
                    
                    % Basis vectors for position, velocity, acceleration
                    T  = [1; tau; tau^2; tau^3; tau^4; tau^5];
                    dT = [0; 1; 2*tau; 3*tau^2; 4*tau^3; 5*tau^4];
                    ddT = [0; 0; 2; 6*tau; 12*tau^2; 20*tau^3];
                    
                    % Compute desired quantities by multiplying coefficient matrix
                    % with basis vectors (note: C' is 3x6, so C'*basis gives 3x1)
                    Xd = C' * T;
                    Vd = C' * dT;
                    Ad = C' * ddT;
                    
                    return
                end
                
                t_accum = t_accum + Tseg;
            end
            
            % If t is beyond all segments, hold at final position
            Xd = obj.X_final;
            Vd = [0;0;0];
            Ad = [0;0;0];
        end
        
        % Get final position
        % Returns the last waypoint position as a column vector.
        function Xf = getFinalPosition(obj)
            Xf = obj.X_final;
        end
        
        % Get hover time for a given waypoint index
        % Returns the hover duration (seconds) for the waypoint at the specified index.
        % Used by the main simulation to determine how long to pause at that waypoint.
        function t_hover = getHoverTime(obj, index)
            t_hover = obj.hoverTime(index);
        end
        
    end
end

