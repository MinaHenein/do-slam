%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

function [vec] = covToUpperTriVec(cov)
%COVTOUPPERTRIVEC converts square covariance matrix to vector
%representation of upper triangle

%*TODO - get rid of find
vec = cov(find(triu(ones(size(cov)))'))';

end

