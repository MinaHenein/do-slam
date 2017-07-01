function [c] = computeAngle(a,b)
%ANGLE Summary of this function goes here
%   Detailed explanation goes here

c = atan2(norm(cross(a,b)),dot(a,b));

end

