function [obj] = constructPointPointEdge(obj,config,edgeRow)
%CONSTRUCTPOINTPOINTEDGE constructs edge representing measurement of
%relative position between two points.

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
point1Vertex = edgeRow{3};
point2Vertex = edgeRow{4};
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};

%% 2. properties
value       = edgeValue;
covariance  = upperTriVecToCov(edgeCovariance);
jacobians   = [];
type        = 'point-point';
iVertices   = [point1Vertex point2Vertex];
index       = edgeIndex;

%% 3. construct edge
obj.edges(edgeIndex) = Edge(value,covariance,jacobians,type,iVertices,index);

%% 4. update edge
%computes value & jacobians
obj = obj.updatePointPointEdge(config,edgeIndex);

%% 5. add index to point vertices
obj.vertices(point1Vertex).iEdges = [obj.vertices(point1Vertex).iEdges edgeIndex];
obj.vertices(point2Vertex).iEdges = [obj.vertices(point2Vertex).iEdges edgeIndex];
end

