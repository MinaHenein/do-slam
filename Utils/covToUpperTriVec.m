function [vec] = covToUpperTriVec(cov)
%COVTOUPPERTRIVEC Summary of this function goes here
%   Detailed explanation goes here

%*TODO - get rid of find
vec = cov(find(triu(ones(size(cov)))'))';

end

