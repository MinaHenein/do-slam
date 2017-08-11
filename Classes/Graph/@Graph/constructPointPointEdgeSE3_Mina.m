function [obj] = constructPointPointEdgeSE3_Mina(obj,config,edgeRow)
%CONSTRUCTPOINTPOINTEDGESE3 constructs edge representing measurement of
%relative SE3 transformation between two points on an object moving with constant SE3 motion.

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
type        = 'point-pointSE3_Mina';
iVertices   = [point1Vertex point2Vertex];
index       = edgeIndex;

%% 3. construct edge
obj.edges(edgeIndex) = Edge(value,covariance,jacobians,type,iVertices,index);

%% 4. update edge
%computes value & jacobians
obj = obj.updatePointPointEdgeSE3_Mina(config,edgeIndex);

%% 5. add index to point vertices
obj.vertices(point1Vertex).iEdges = [obj.vertices(point1Vertex).iEdges edgeIndex];
obj.vertices(point2Vertex).iEdges = [obj.vertices(point2Vertex).iEdges edgeIndex];
end

