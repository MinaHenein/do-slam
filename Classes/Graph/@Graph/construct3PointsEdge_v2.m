function [obj] = construct3PointsEdge_v2(obj,config,edgeRow)
%CONSTRUCT3POINTSEDGE constructs edge representing measurement of
%trinary factor between three points.

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
pointVertices = edgeRow{3};
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};

%% 2. properties
value       = edgeValue;
covariance  = upperTriVecToCov(edgeCovariance);
jacobians   = [];
type        = '3-points';
iVertices   = pointVertices;
index       = edgeIndex;

%% 3. construct edge
obj.edges(edgeIndex) = Edge(value,covariance,jacobians,type,iVertices,index);

%% 4. update edge
%computes value & jacobians
obj = obj.update3PointsEdge_v2(config,edgeIndex);

%% 5. add index to point vertices
obj.vertices(pointVertices(1)).iEdges = [obj.vertices(pointVertices(1)).iEdges edgeIndex];
obj.vertices(pointVertices(2)).iEdges = [obj.vertices(pointVertices(2)).iEdges edgeIndex];
obj.vertices(pointVertices(3)).iEdges = [obj.vertices(pointVertices(3)).iEdges edgeIndex];
end

