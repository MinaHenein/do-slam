function [fit,parameters] = fitPlane(points)
%FITPLANE Summary of this function goes here
%   Detailed explanation goes here

fit = 0;
parameters = [0 0 0 0]';

nPoints = size(points,2);
if nPoints > 2
    %   fit plane
    [n,V,p] = affine_fit(points');
    d = p*n;
    
    fit = 1;
    parameters = [n; d];
%     if d < 0
%         n = -n;
%         d = -d;
%     end
%     %   points colinear
%     if d~=0
%         fit = 1;
%         parameters = [n; d];
%     end   
end

end

