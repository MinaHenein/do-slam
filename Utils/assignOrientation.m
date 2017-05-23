%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

function [axisAngle] = assignOrientation(v,varargin)
%ASSIGNORIENTATION assigns orientation to vector v
%   new orientation is (x,y,z)
%   absolute coords are (X,Y,Z)
%   this implementation forces x axis of orientation to align with v, and z
%   axis of orientation to lie in plane vProjXY-Z, where X,Y,Z are absolute

% x points in direction of motion
xAxis  = unit(v);
vProjXY = [xAxis(1:2); 0];

% y axis normal to plane formed by xAxis and projection of xAxis onto XY
% plane
if dot(xAxis,vProjXY)==1
	yAxis = unit(cross(xAxis,[0 0 1]')); 
else
    yAxis = unit(cross(xAxis,vProjXY));
end

% z axis 
zAxis  = unit(cross(xAxis,yAxis));
if zAxis(3) < 0
    yAxis = -yAxis;
    zAxis = -zAxis;
end

% compute scaled axis orientation
R      = [xAxis,yAxis,zAxis];
axisAngle = arot(R);

end
