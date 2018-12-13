function [b] = constructResiduals(obj,config,measurementsCell)
%CONSTRUCTRESIDUALS constructs residuals vector b from graph and
%measurements

%% 1. sizes
%   dimensions of each edge/vertex
edgeSizes   = {obj.edges.value};
edgeHeights  = cell2mat([cellfun(@size,edgeSizes,'UniformOutput',0)]');
edgeHeights  = edgeHeights(:,1);

%filter out inactive edges
activeEdges = logical(cell2mat({obj.edges.active}'));
iEdges = 1:obj.nEdges;
nActiveEdges = sum(activeEdges);
iActiveEdges = iEdges(activeEdges);

%   preallocate linear system variables
dimEdges    = sum(edgeHeights(iActiveEdges));
b = zeros(dimEdges,1); %residuals

%% 2. loop over edges
for i = 1:nActiveEdges
    iEdge = iActiveEdges(i);
    %if vertices reordered, DONT use vertex indices from edgeRow
    edgeRow = measurementsCell(iEdge,:);
    %compute residuals
    iStart = sum(edgeHeights(1:iEdge-1)) + 1;
    iEnd   = iStart + edgeHeights(iEdge) - 1;
    iBlock = iStart:iEnd;
    if ~isempty(obj.edges(iEdge).type)
        b(iBlock,1) = obj.computeResidual(config,iEdge,edgeRow{5});
    end
end

end

