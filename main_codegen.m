function drone1_state_out = main_codegen()
    %#codegen
    
    dt = 0.01;
    simulationTime = 60;                   % Total desired simulation time
    N_max = 10000;                         % Fixed maximum size (for codegen) - 100 s with dt=0.01
    N = min(ceil(simulationTime / dt), N_max);  % Limit to avoid exceeding
    
    params = struct(...
        'mass', 1.25, ...
        'armLength', 0.225, ...
        'Ixx', 0.00605922, ...
        'Iyy', 0.00605922, ...
        'Izz', 0.0121184, ...
        'kT', 1.22e-5, ...
        'kQ', 2.19e-7, ...
        'maxOmega', 1047.1975512);
    
    gains = struct(...
        'P_phi', 4.5, 'I_phi', 0.0, 'D_phi', 0.4, ...
        'P_theta', 4.5, 'I_theta', 0.0, 'D_theta', 0.4, ...
        'P_psi', 2.0, 'I_psi', 0.0, 'D_psi', 0.2, ...
        'P_zdot', 6.0, 'I_zdot', 0.1, 'D_zdot', 0.05, ...
        'P_pos', [4.2; 4.2; 5.0], ...
        'I_pos', [0.93; 0.93; 1.0], ...
        'D_pos', [3.0; 3.0; 3.0]);
    
    initStates = zeros(12,1);
    initInputs = zeros(4,1);
    
    planner = TrajectoryPlanner_codegen();
    drone1 = Drone_codegen(params, initStates, initInputs, gains, simulationTime);
    
    drone1_state_out = zeros(N_max, 12);   % Fixed size for codegen
    coder.varsize('drone1_state_out', [N_max, 12], [1, 0]);   % Allow first dimension to vary
    actual_steps = 0;
    t_plan = 0;
    
    % Constantes para prints
    R2D = 180 / pi;           %#codegen
    RPS2RPM = 30 / pi;        %#codegen
    printInterval = 1.0;      % seconds between prints
    
    for k = 1:N_max
        t_sim = (k-1)*dt;     % real simulation time
        
        % --- Save state BEFORE calling getReference ---
        old_segment = planner.currentSegment;
        old_hover   = planner.inHover;
        
        [Xd, Vd, Ad] = planner.getReference(t_plan, t_sim);
        drone1 = drone1.PositionCtrl(Xd, Vd, Ad);
        drone1 = drone1.UpdateState();
        
        actual_steps = k;
        drone1_state_out(k,:) = drone1.GetState()';

        drone_state = drone1.GetState();
        pos = drone_state(1:3);
        
        % --- Check waypoint events (comparing with old) ---
        if ~old_hover && planner.inHover
            % Entrou em hover: waypoint atingido
            reached_wp = planner.currentSegment + 1;
            fprintf('\n=================================================\n');
            fprintf('WAYPOINT %d REACHED (hover)\n', int32(reached_wp));
        elseif old_hover && ~planner.inHover
            % Saiu do hover: término do hover no waypoint anterior
            finished_wp = old_segment + 1;
            fprintf('WP Hover %d completed.\n\n', int32(finished_wp));
        elseif planner.currentSegment > old_segment && ~planner.inHover
            % Avançou de segmento sem hover (fly-through): waypoint atingido
            reached_wp = planner.currentSegment;
            fprintf('\n=================================================\n');
            fprintf('WAYPOINT %d REACHED (fly-through)\n', int32(reached_wp));
        end
        
        % Telemetry (every ~1 s or at start/end)
        if mod(k, round(printInterval / dt)) == 0 || k == 1 || k == N
            vel = drone_state(4:6);
            att_deg = drone_state(7:9) * R2D;
            OmegaRPM = drone1.Omega * RPS2RPM;
            pos_error = norm(pos - Xd);
            
            % ---- Determine waypoints for display ----
            numWP = size(planner.waypoints, 1);
            
            if t_sim == 0
                % Initial instant: current = 1 (origin), target = 2 (first destination)
                current_wp_display = 1;
                target_wp_display = 2;
            elseif planner.currentSegment > planner.numSegments
                % After last segment and hover completed: all at last waypoint
                current_wp_display = numWP;
                target_wp_display = numWP;
            elseif planner.inHover
                % During hover: at waypoint currentSegment+1, target is the same
                current_wp_display = planner.currentSegment + 1;
                target_wp_display = planner.currentSegment + 1;
            else
                % Normal movement: current = current segment, target = next waypoint
                current_wp_display = planner.currentSegment;
                target_wp_display = planner.currentSegment + 1;
            end
            
            % Ensure indices do not exceed bounds
            current_wp_display = max(1, min(current_wp_display, numWP));
            target_wp_display   = max(1, min(target_wp_display, numWP));
            
            % --- Hover time to display: only shows if in hover, otherwise 0 ---
            if planner.inHover
                hover_this_wp = planner.hoverTime(current_wp_display);
            else
                hover_this_wp = 0;
            end
            
            fprintf('-------------------------------------------------\n');
            fprintf('Time: %.1f s\n', t_sim);
            
            % Display Current WP and fixed coordinates of Target WP
            fprintf('Current WP: [%.1f %.1f %.1f]   Target WP: [%.1f %.1f %.1f]   Hover: %.1fs\n', ...
                    planner.waypoints(current_wp_display, 1), ...
                    planner.waypoints(current_wp_display, 2), ...
                    planner.waypoints(current_wp_display, 3), ...
                    planner.waypoints(target_wp_display, 1), ...
                    planner.waypoints(target_wp_display, 2), ...
                    planner.waypoints(target_wp_display, 3), ...
                    hover_this_wp);
            
            fprintf('Position : [%.3f  %.3f  %.3f] m\n', pos(1), pos(2), pos(3));
            fprintf('Velocity : [%.3f  %.3f  %.3f] m/s\n', vel(1), vel(2), vel(3));
            fprintf('Attitude : [%.2f  %.2f  %.2f] deg\n', att_deg(1), att_deg(2), att_deg(3));
            fprintf('Tracking Error Norm: %.4f m\n', pos_error);
            fprintf('Motors RPM: [%5.0f  %5.0f  %5.0f  %5.0f]\n', ...
                    OmegaRPM(1), OmegaRPM(2), OmegaRPM(3), OmegaRPM(4));
        end
        
        % Stopping condition: mission complete (all segments + final hover)
        if planner.currentSegment > planner.numSegments && ~planner.inHover
            fprintf('\n=== MISSION COMPLETED SUCCESSFULLY ===\n');
            fprintf('Final Position: [%.3f %.3f %.3f] m\n', pos(1), pos(2), pos(3));
            fprintf('Total Simulation Time: %.1f s\n', t_sim);
            break;
        end
        
        % --- Increment planner time ONLY if not in hover ---
        if ~planner.inHover
            t_plan = t_plan + dt;
        end
        
        % Safety: do not exceed N_max
        if k >= N_max
            fprintf('\n=== Maximum step limit reached ===\n');
            break;
        end
    end
    
    % Trim output array to actual size
    drone1_state_out = drone1_state_out(1:actual_steps, :);
end