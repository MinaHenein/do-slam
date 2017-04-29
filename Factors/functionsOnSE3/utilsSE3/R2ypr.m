function ypr = R2ypr(R)
% Extracts angles from a rotation matrix.
%
% yaw pitch roll angles from rotation matrix

roll = atan2(R(2,1),R(1,1));
pitch = atan2(-R(3,1),R(1,1)*cos(roll)+R(2,1)*sin(roll));
yaw = atan2(R(3,2),R(3,3));

ypr = [yaw;pitch;roll];

