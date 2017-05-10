function [covariance] = stdToCovariance(variance)
%VARIANCETOCOVARIANCE Summary of this function goes here
%   Detailed explanation goes here

covariance = diag(variance.^2);

end

