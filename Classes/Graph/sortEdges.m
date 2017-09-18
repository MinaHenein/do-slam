function [graph,newToOldMap,measurementsCellSorted] = sortEdges(graph,measurementsCell)
%REORDERVERTICES Summary of this function goes here
%   Detailed explanation goes here

%identify indexes
iPosePriorEdges     = graph.identifyEdges('posePrior');
iPosePoseEdges      = graph.identifyEdges('pose-pose');
iPosePointEdges     = graph.identifyEdges('pose-point');
% iPlanePriorEdges    = graph.identifyEdges('planePrior');
% iPointPlaneEdges    = graph.identifyEdges('point-plane');
% iAngleEdges         = graph.identifyEdges('plane-plane-angle');
% iDistanceEdges      = graph.identifyEdges('plane-plane-distance');
% iFixedAngleEdges    = graph.identifyEdges('plane-plane-fixedAngle');
% iFixedDistanceEdges = graph.identifyEdges('plane-plane-fixedDistance');
iPointMotionEdges = graph.identifyEdges('2points-SE3Motion');


% newToOldMap = [iPosePriorEdges; iPosePoseEdges; iPosePointEdges;...
%                iPlanePriorEdges; iPointPlaneEdges; iAngleEdges;...
%                iDistanceEdges; iFixedAngleEdges; iFixedDistanceEdges];
newToOldMap = [iPosePriorEdges; iPosePoseEdges; iPosePointEdges;iPointMotionEdges];

oldToNewMap = zeros(size(newToOldMap));
for i = 1:graph.nEdges
    oldToNewMap(newToOldMap(i)) = i;
end

%preallocate
edgesReordered = Edge();
edgesReordered(1:graph.nEdges) = Edge();

%reorder edges
for i = 1:graph.nEdges
    edgesReordered(i) = graph.edges(newToOldMap(i));
    edgesReordered(i).index = i;
end
graph.edges = edgesReordered;

%map edge indexes in vertices
for i = 1:graph.nVertices
    graph.vertices(i).iEdges = oldToNewMap(graph.vertices(i).iEdges)';
end

%fix measurementsCell
measurementsCellSorted = measurementsCell;
for i = 1:graph.nEdges
    measurementsCellSorted(i,:) = measurementsCell(newToOldMap(i),:);
    measurementsCellSorted{i,2} = i;
end

end

