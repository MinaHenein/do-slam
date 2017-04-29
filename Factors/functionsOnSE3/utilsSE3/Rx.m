function H=Rx(yaw)
% Rotation around X (yaw).
%
% Homogeneous matrix for a rotation about X axis.

H = [ypr2R(yaw,0,0) zeros(3,1); 0 0 0 1];
