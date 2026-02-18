% WAYPOINTCONSTRAINTS  Checks if a position meets waypoint region constraints.
%   [insideRegion, precise] = WaypointConstraints(pos, center, orientation, R)
%   evaluates two conditions:
%       1. Whether the point lies inside a rectangular region centered at
%          'center' with dimensions 2 m (long side) by 1 m (short side),
%          oriented either north-south (NS) or east-west (EW).
%       2. Whether the point is within a sphere of radius 0.3*R around the
%          center (a precision condition).
%
%   Inputs:
%       pos         - 3-element vector [x; y; z] representing drone position [m].
%       center      - 3-element vector [xw; yw; zw] center of the waypoint region [m].
%       orientation - String, either 'NS' or 'EW', specifying the orientation
%                     of the rectangle. If 'NS', the long side (2 m) is aligned
%                     with the x‑axis (north‑south) and the short side (1 m) with
%                     the y‑axis (east‑west). If 'EW', the long side is aligned
%                     with the y‑axis.
%       R           - Scalar, radius of the drone [m] (used to scale the precision sphere).
%
%   Outputs:
%       insideRegion - Logical true if the point is inside the oriented rectangle.
%       precise      - Logical true if the Euclidean distance from pos to center
%                      is ≤ 0.3 * R.
%
%   Note: The function currently ignores the z‑coordinate for the rectangle
%         check (only x and y are considered). The precision check uses all
%         three dimensions.

function [insideRegion, precise] = WaypointConstraints(pos, center, orientation, R)

    x = pos(1);
    y = pos(2);
    
    xc = center(1);
    yc = center(2);
    
    % Rectangle dimensions (fixed)
    long_side = 2;   % 2 meters (long side)
    short_side = 1;  % 1 meter (short side)
    
    if strcmp(orientation,'NS')
        % Long side aligned with x‑axis (north‑south)
        insideRegion = abs(x - xc) <= long_side/2 && ...
                       abs(y - yc) <= short_side/2;
    
    elseif strcmp(orientation,'EW')
        % Long side aligned with y‑axis (east‑west)
        insideRegion = abs(x - xc) <= short_side/2 && ...
                       abs(y - yc) <= long_side/2;
    else
        error('Orientation must be NS or EW');
    end
    
    % Spherical precision condition (3D distance)
    dist = norm(pos - center);
    precise = dist <= 0.3*R;

end