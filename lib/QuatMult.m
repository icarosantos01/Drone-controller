function qout = QuatMult(q, p)
    % Multiplication of Quaternions (Hamilton product)
    % qout = q * p
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    pw = p(1); px = p(2); py = p(3); pz = p(4);
    
    qout = [qw*pw - qx*px - qy*py - qz*pz;
            qw*px + qx*pw + qy*pz - qz*py;
            qw*py - qx*pz + qy*pw + qz*px;
            qw*pz + qx*py - qy*px + qz*pw];
end