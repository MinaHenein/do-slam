function [covariance] = stdToCovariance(variance)
%STDTOCOVARIANCE converts stdDev vector to covariance matrix

covariance = diag(variance.^2);

end

