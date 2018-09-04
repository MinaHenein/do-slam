function [obj] = constructLinearSystem(obj,config,graph,measurementsCell,weight)
%CONSTRUCTLINEARSYSTEM constructs linear system from graph
%   1. Determine sizes of A,b,covariance matrices from dimensions of edges
%      and vertices. Remove inactive edges.
%   2. Loop over active edges. 
%      Take jacobians from edge and use blockMap to place in A. 
%      Take covariance from edge and use blockMap to place in covariance
%      matrix.
%      Compute residual from edge and use blockMap to place in b.

%% 1. sizes
%   dimensions of each edge/vertex
edgeSizes   = {graph.edges.value};
vertexSizes = {graph.vertices.value};
edgeHeights      = cell2mat([cellfun(@size,edgeSizes,'UniformOutput',0)]');
obj.edgeHeights  = edgeHeights(:,1);
vertexWidths     = cell2mat([cellfun(@size,vertexSizes,'UniformOutput',0)]');
obj.vertexWidths = vertexWidths(:,1);

%filter out inactive edges
activeEdges = logical(cell2mat({graph.edges.active}'));
iEdges = 1:graph.nEdges;
nActiveEdges = sum(activeEdges);
obj.iActiveEdges = iEdges(activeEdges);

%   preallocate linear system variables
dimEdges    = sum(obj.edgeHeights(obj.iActiveEdges));
dimVertices = sum(obj.vertexWidths);
obj.A = sparse(dimEdges,dimVertices); %jacobians
obj.b = sparse(dimEdges,1); %residuals
obj.covariance = sparse(sum(dimEdges,dimEdges));
obj.covSqrtInv = sparse(sum(dimEdges,dimEdges));

%% 2. loop over edges
for i = 1:nActiveEdges
    iEdge = obj.iActiveEdges(i);
    edgeRow = measurementsCell(iEdge,:);
    iVertices = graph.edges(iEdge).iVertices;
    nVertices = size(iVertices,2);
    iBlock = blockMap(obj,iEdge,'edge');
    for j = 1:nVertices
        jBlock = blockMap(obj,iVertices(j),'vertex');
        obj.A(iBlock,jBlock) = graph.edges(iEdge).jacobians{j};
    end
        
    %covariance in covSqrtInv
    obj.covariance(iBlock,iBlock) = graph.edges(iEdge).covariance;
    obj.covSqrtInv(iBlock,iBlock) = graph.edges(iEdge).covariance^-0.5;
    %compute residuals
    obj.b(iBlock,1) = graph.computeResidual(config,iEdge,edgeRow{5});
    
end

%kPerp - need this for GN with planar constraints
iPlaneVertices = identifyVertices(graph,'plane');
if ~isempty(iPlaneVertices)
    iPlaneParameters = [graph.vertices(iPlaneVertices).value];
    etas = iPlaneParameters(1:3,:)';
    positions = zeros(size(etas,1),1);
    for j = 1:numel(iPlaneVertices)
        jBlock = blockMap(obj,iPlaneVertices(j),'vertex');
        positions(j) = jBlock(1);
    end
    [~, kPerp] = buildKernel(size(obj.H,1), etas, positions);
    obj.kPerp = kPerp;
    obj.A = obj.A*obj.kPerp;
%     system.Hk = kPerp'*system.H*kPerp;
%     system.ck = kPerp'*system.c;
else
    obj.kPerp = 1;
end

if ~isempty(config.robustCostFunction)
    obj.A = weight*obj.A;
    obj.b = weight*obj.b;
end

end

