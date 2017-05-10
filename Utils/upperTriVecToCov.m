function [cov] = upperTriVecToCov(vec)
%UPPERTRIVECTOCOV Summary of this function goes here
%   Detailed explanation goes here



rows = (sqrt(8*numel(vec)+1)-1)/2;
cov = zeros(rows);
ind = find(tril(ones(rows)));
cov(ind) = vec;
cov = cov + tril(cov,-1)';


% % for i = 1:rows
% %     for j = 1:rows
% %         cov(i,j) = 
% %     end
% % end
% 
% cov(find(triu(ones(size(cov))))) = vec;

end

