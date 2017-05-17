function [cov] = upperTriVecToCov(vec)
%UPPERTRIVECTOCOV converts vector representation of upper triangle to
%square covariance matrix

rows = (sqrt(8*numel(vec)+1)-1)/2;
cov = zeros(rows);
ind = find(tril(ones(rows)));
cov(ind) = vec;
cov = cov + tril(cov,-1)';

end

