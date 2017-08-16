function [obj] = construct2PointsSE3MotionEdge(obj,config,edgeRow)
%CONSTRUCT2POINTSSE3MOTIONEDGE constructs edge representing measurement
%between two points and their respective SE3 motion vertex

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
pointVertices = edgeRow{3};
SE3MotionVertex = edgeRow{4};
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};

%% 2. properties
value       = edgeValue;
covariance  = upperTriVecToCov(edgeCovariance);
jacobians   = [];
type        = '2points-SE3Motion';
iVertices   = [pointVertices SE3MotionVertex];
index       = edgeIndex;

%% 3. construct edge
obj.edges(edgeIndex) = Edge(value,covariance,jacobians,type,iVertices,index);

%% 4. update edge
%computes value & jacobians
obj = obj.update2PointsSE3MotionEdge(config,edgeIndex);

%% 5. add index to point and velocity vertices
obj.vertices(pointVertices(1)).iEdges = [obj.vertices(pointVertices(1)).iEdges edgeIndex];
obj.vertices(pointVertices(2)).iEdges = [obj.vertices(pointVertices(2)).iEdges edgeIndex];
obj.vertices(SE3MotionVertex).iEdges = [obj.vertices(SE3MotionVertex).iEdges edgeIndex];
end

