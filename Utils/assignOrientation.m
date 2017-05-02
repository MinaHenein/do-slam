function [axisAngle] = assignOrientation(v,varargin)
%ASSIGNORIENTATION assigns orientation to vector v
%   new orientation is (x,y,z)
%   absolute coords are (X,Y,Z)
%   this implementation forces x axis of orientation to align with v, and z
%   axis of orientation to lie in plane vProjXY-Z, where X,Y,Z are absolute
%   *NOTE: different orientation,change how xAxis,yAxis,zAxis placed into R
%          based on varargin

xAxis  = unit(v);
vProjXY = [v(1:2); 0];
yAxis  = cross(v,vProjXY);
zAxis  = cross(xAxis,yAxis);
R      = [xAxis,yAxis,zAxis];
axisAngle = arot(R);


end

