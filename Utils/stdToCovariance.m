%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

function [covariance] = stdToCovariance(variance)
%STDTOCOVARIANCE converts stdDev vector to covariance matrix

covariance = diag(variance.^2);

end

