classdef TrajectoryPlanner_codegen < handle
    %#codegen
    properties
        waypoints
        segmentTimes
        coefficients
        numSegments
        X_final
        hoverTime
        % Persistent hover state
        inHover logical = false;
        hoverStart double = 0;       % planner time at hover start (no longer used, kept for compatibility)
        hoverStartReal double = 0;   % real time at hover start
        currentSegment double = 1;
        tAccum double = 0;
    end
    
    methods
        function obj = TrajectoryPlanner_codegen()
            
            obj.waypoints = [
                0 0 0;
                3 3 -3;
                5 2 -6;
                6 3 -3;
            ];

            obj.hoverTime = [0; 0; 0; 10];
            
            obj.X_final = obj.waypoints(end,:)';
            
            obj.numSegments = size(obj.waypoints,1) - 1;
            
            obj.segmentTimes = [6; 6; 6];    % durations of segments 1→2, 2→3, 3→4
            
            obj.coefficients = zeros(6,3,obj.numSegments);
            
            for k = 1:obj.numSegments
                Xstart = obj.waypoints(k,:);
                Xend = obj.waypoints(k+1,:);
                for i = 1:3
                    obj.coefficients(:,i,k) = poly5_segment(Xstart(i), Xend(i), obj.segmentTimes(k));
                end
            end
            % Initialize state
            obj.inHover = false;
            obj.hoverStart = 0;
            obj.hoverStartReal = 0;
            obj.currentSegment = 1;
            obj.tAccum = 0;
        end
        
        function [Xd, Vd, Ad] = getReference(obj, t_plan, t_real)
            %#codegen
            while true
                if obj.currentSegment > obj.numSegments
                    Xd = obj.X_final;
                    Vd = zeros(3,1);
                    Ad = zeros(3,1);
                    return;
                end
                
                if obj.inHover
                    % Check hover end using REAL time
                    if t_real - obj.hoverStartReal >= obj.hoverTime(obj.currentSegment + 1)
                        obj.inHover = false;
                        obj.tAccum = obj.tAccum + obj.segmentTimes(obj.currentSegment);
                        obj.currentSegment = obj.currentSegment + 1;
                        continue;   % Re‑evaluate for new segment
                    else
                        Xd = obj.waypoints(obj.currentSegment + 1, :)';
                        Vd = zeros(3,1);
                        Ad = zeros(3,1);
                        return;
                    end
                end
                
                % Not in hover → moving along current segment
                Tseg = obj.segmentTimes(obj.currentSegment);
                if t_plan < obj.tAccum + Tseg
                    tau = t_plan - obj.tAccum;
                    C = obj.coefficients(:,:,obj.currentSegment);
                    T_vec = [1; tau; tau^2; tau^3; tau^4; tau^5];
                    dT_vec = [0; 1; 2*tau; 3*tau^2; 4*tau^3; 5*tau^4];
                    ddT_vec = [0; 0; 2; 6*tau; 12*tau^2; 20*tau^3];
                    Xd = C' * T_vec;
                    Vd = C' * dT_vec;
                    Ad = C' * ddT_vec;
                    return;
                else
                    % Segment finished
                    if obj.hoverTime(obj.currentSegment + 1) > 0
                        obj.inHover = true;
                        obj.hoverStart = t_plan;        % (optional, not used)
                        obj.hoverStartReal = t_real;    % store real time
                        Xd = obj.waypoints(obj.currentSegment + 1, :)';
                        Vd = zeros(3,1);
                        Ad = zeros(3,1);
                        return;
                    else
                        % No hover → advance to next segment and loop
                        obj.tAccum = obj.tAccum + Tseg;
                        obj.currentSegment = obj.currentSegment + 1;
                        continue;
                    end
                end
            end
        end
    end
end