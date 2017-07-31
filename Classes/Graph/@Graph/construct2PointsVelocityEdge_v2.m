function [obj] = construct2PointsVelocityEdge(obj,config,edgeRow)
%CONSTRUCTPOINTVELOCITYEDGE constructs edge representing measurement of
%difference between two points and their respective velocity vertex

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
pointVertices = edgeRow{3};
velocityVertex = edgeRow{4};
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};

%% 2. properties
value       = edgeValue;
covariance  = upperTriVecToCov(edgeCovariance);
jacobians   = [];
type        = '2points-velocity';
iVertices   = [pointVertices velocityVertex];
index       = edgeIndex;

%% 3. construct edge
obj.edges(edgeIndex) = Edge(value,covariance,jacobians,type,iVertices,index);

%% 4. update edge
%computes value & jacobians
obj = obj.update2PointsVelocityEdge_v2(config,edgeIndex);

%% 5. add index to point and velocity vertices
obj.vertices(pointVertices(1)).iEdges = [obj.vertices(pointVertices(1)).iEdges edgeIndex];
obj.vertices(pointVertices(2)).iEdges = [obj.vertices(pointVertices(2)).iEdges edgeIndex];
obj.vertices(velocityVertex).iEdges = [obj.vertices(velocityVertex).iEdges edgeIndex];
end

