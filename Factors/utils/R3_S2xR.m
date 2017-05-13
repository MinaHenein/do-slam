function [S2xRPosition] = R3_S2xR(R3Position)
%R3_S2XR Converts from 3D cartesian coordinates to spherical coordinates
%   Detailed explanation goes here

[az,el,r]    = cart2sph(R3Position(1),R3Position(2),R3Position(3));
S2xRPosition = [az; el; r];

end

