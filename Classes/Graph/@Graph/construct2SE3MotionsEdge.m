function [obj] = construct2SE3MotionsEdge(obj,config,edgeRow)
%CONSTRUCT2SE3MOTIONSEDGE constructs edge representing measurement
%between two consecutive SE3 motion verteices

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
SE3MotionVertex1 = edgeRow{3};
SE3MotionVertex2 = edgeRow{4};
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};

%% 2. properties
value       = edgeValue;
covariance  = upperTriVecToCov(edgeCovariance);
jacobians   = [];
type        = 'SE3Motion-SE3Motion';
iVertices   = [SE3MotionVertex1 SE3MotionVertex2];
index       = edgeIndex;

%% 3. construct edge
obj.edges(edgeIndex) = Edge(value,covariance,jacobians,type,iVertices,index);

%% 4. update edge
%computes value & jacobians
obj = obj.update2SE3MotionsEdge(config,edgeIndex);

%% 5. add index to point and motion vertices
obj.vertices(SE3MotionVertex1).iEdges = unique([obj.vertices(SE3MotionVertex1).iEdges edgeIndex]);
obj.vertices(SE3MotionVertex2).iEdges = unique([obj.vertices(SE3MotionVertex1).iEdges edgeIndex]);

end

