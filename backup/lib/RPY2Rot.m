% RPY2Rot  Converts roll-pitch-yaw Euler angles to a rotation matrix.
%   bRi = RPY2Rot(angles) returns the rotation matrix that transforms a
%   vector from the inertial (world) frame to the body-fixed frame,
%   using the ZYX Euler angle convention (yaw, then pitch, then roll).
%
%   The rotation sequence is:
%       1. Yaw (psi)   about the inertial z-axis.
%       2. Pitch (theta) about the new y-axis (after yaw).
%       3. Roll (phi)   about the new x-axis (after pitch).
%
%   The resulting matrix bRi satisfies: v_body = bRi * v_inertial.
%   Its transpose (bRi') performs the inverse transformation (body to inertial).
%
%   Input:
%       angles - 3-element vector [phi; theta; psi] (roll, pitch, yaw) in radians.
%
%   Output:
%       bRi    - 3x3 rotation matrix (inertial to body).
%
%   Reference: See equation on page 17 of the course notes.

function bRi = RPY2Rot(angles)
    phi = angles(1);    % Roll angle  [rad]
    theta = angles(2);  % Pitch angle [rad]
    psi = angles(3);    % Yaw angle   [rad]
    
    % Elementary rotation matrices
    % R_3: Yaw rotation about z-axis
    R_3 = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
    % R_2: Pitch rotation about y-axis
    R_2 = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
    % R_1: Roll rotation about x-axis
    R_1 = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
    
    % Combined rotation: R = R_roll * R_pitch * R_yaw
    bRi = R_1 * R_2 * R_3;
end