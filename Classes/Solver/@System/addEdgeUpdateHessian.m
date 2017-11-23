function [obj] = addEdgeUpdateHessian(obj,config,graph,edgeCell)

% addEdgeUpdate
iBlock = blockMap(obj,obj.iActiveEdges(edgeCell{2}),'edge');
residual = obj.b(iBlock,1);
[Omega, omega] = computeOmegas(config,graph,graph.edges(edgeCell{2}),residual);
iBlock = size(obj.H,1)+1:size(obj.H,1)+size(Omega,1);
% Hessian matrix
obj.H(iBlock,iBlock) = Omega;
% right hand term
obj.c(iBlock,1) = omega;

end