%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

function [R3Position] = S2xR_R3(S2xRPosition)
%S2XR_R3 converts from spherical coordinates to 3D cartesian coordinates
%   Detailed explanation goes here

[x,y,z]    = sph2cart(S2xRPosition(1),S2xRPosition(2),S2xRPosition(3));
R3Position = [x; y; z];

end

