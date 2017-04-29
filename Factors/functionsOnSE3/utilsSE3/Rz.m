function H=Rz(roll)
% Rotation around Z (roll).
%
% Homogeneous matrix for a rotation about Z axis.

H = [ypr2R(0,0,roll) zeros(3,1); 0 0 0 1];
