% POLY5_SEGMENT  Generates coefficients for a quintic polynomial trajectory segment.
%   coeff = poly5_segment(x0, xf, T) computes the coefficients of a 5th-order
%   polynomial that satisfies boundary conditions on position, velocity, and
%   acceleration at the start and end of a time interval [0, T].
%
%   The polynomial is of the form:
%       x(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
%   for t in [0, T]. It is commonly used for smooth point-to-point motion
%   planning in robotics and automation.
%
%   Boundary conditions:
%       At t = 0:   x(0) = x0,   dx/dt(0) = 0,   d²x/dt²(0) = 0
%       At t = T:   x(T) = xf,   dx/dt(T) = 0,   d²x/dt²(T) = 0
%
%   Inputs:
%       x0 - Initial position (scalar)
%       xf - Final position (scalar)
%       T  - Duration of the segment (positive scalar)
%
%   Output:
%       coeff - 6x1 column vector of polynomial coefficients
%               [c0; c1; c2; c3; c4; c5] corresponding to the powers
%               t^0, t^1, t^2, t^3, t^4, t^5.
%
%   The function sets up a linear system A * coeff = b, where the rows of A
%   enforce the six boundary conditions. The matrix is inverted to obtain
%   the coefficients.

function coeff = poly5_segment(x0, xf, T)
    
    % Constraint matrix A (6x6) for the boundary conditions:
    % Rows 1-3: conditions at t = 0 (position, velocity, acceleration)
    % Rows 4-6: conditions at t = T (position, velocity, acceleration)
    A = [1 0 0 0 0 0;                % x(0) = c0
         0 1 0 0 0 0;                % dx/dt(0) = c1
         0 0 2 0 0 0;                % d²x/dt²(0) = 2*c2
         1 T T^2 T^3 T^4 T^5;        % x(T) = c0 + c1*T + c2*T^2 + c3*T^3 + c4*T^4 + c5*T^5
         0 1 2*T 3*T^2 4*T^3 5*T^4;  % dx/dt(T) = c1 + 2*c2*T + 3*c3*T^2 + 4*c4*T^3 + 5*c5*T^4
         0 0 2 6*T 12*T^2 20*T^3];   % d²x/dt²(T) = 2*c2 + 6*c3*T + 12*c4*T^2 + 20*c5*T^3
    
    % Right-hand side vector b (6x1) containing the desired boundary values
    b = [x0; 0; 0; xf; 0; 0];
    
    % Solve for the coefficients
    coeff = A \ b;

end
