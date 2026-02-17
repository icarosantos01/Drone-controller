function euler = Quat2Euler(q)
    % q = [qw, qx, qy, qz]
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    
    % Roll (phi)
    sinr_cosp = 2 * (qw * qx + qy * qz);
    cosr_cosp = 1 - 2 * (qx^2 + qy^2);
    phi = atan2(sinr_cosp, cosr_cosp);
    
    % Pitch (theta)
    sinp = 2 * (qw * qy - qz * qx);
    if abs(sinp) >= 1
        theta = copysign(pi / 2, sinp); % gimbal lock
    else
        theta = asin(sinp);
    end
    
    % Yaw (psi)
    siny_cosp = 2 * (qw * qz + qx * qy);
    cosy_cosp = 1 - 2 * (qy^2 + qz^2);
    psi = atan2(siny_cosp, cosy_cosp);
    
    euler = [phi; theta; psi];
end