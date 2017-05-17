function [points] = generateEllipsoidPoints(param)
% Generates a rigid body, represented by an object class instance and 
% associated points instances in the already existant objects and points array. 
% Input the objects array,points array and parameters in order for an ellipsoid.

N = 8;
[x, y, z] = ellipsoid(0, 0, 0, param(1), param(2), param(3), N);
x = reshape(x, [1 (N+1)*(N+1)]);
y = reshape(y, [1 (N+1)*(N+1)]);
z = reshape(z, [1 (N+1)*(N+1)]);

points = [x; y; z];
points = unique(points','rows')';

end

