% rotation around x -90 deg
theta = -pi/2;
Rx = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
% rotation around y 90 deg
theta = pi/2;
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
R =  Rx*Ry;

x = [1; 0; 0];
y = [0; 1; 0];
z = [0; 0; 1];

R*x %should give -y
R*y %should give -z
R*z %should give  x

