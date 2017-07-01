function [k, kPerp] = buildKernel(sA, etas, positions)

% sA number of rows of A
% etas matrix containing by row all etas
% positions matrix containing by row the starting row of eta in A

k = zeros(sA,size(etas,1));
% k = sparse(sA,size(etas,1));
for i=1:size(etas,1)
    k(positions(i):positions(i)+2,i) = etas(i,:)';
end

% kPerp = null(k','r'); %rational basis
kPerp = null(k'); %orthonormal basis
% kPerp = sparseNull(k'); %orthonormal basis - doesn't work yet
% kPerp = spspaces(k',3); % doesn't work yet

end