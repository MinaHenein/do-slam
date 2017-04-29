function H=Ry(pitch)
% Rotation around Y (pitch).
%
% Homogeneous matrix for a rotation about Y axis.

H = [ypr2R(0,pitch,0) zeros(3,1); 0 0 0 1];
