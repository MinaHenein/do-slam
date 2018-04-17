function [obj] = constructPosePointIntrinsicEdge(obj,config,edgeRow)
%CONSTRUCTPOSEPOINTINTRINSICEDGE constructs edge representing measurement of
%relative position of a point in image frame

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
posePointVertexes = edgeRow{3};
poseVertex = posePointVertexes(1);
pointVertex = posePointVertexes(2);
intrinsicVertex = edgeRow{4}; 
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};

%% 2. properties
value       = edgeValue;
covariance  = upperTriVecToCov(edgeCovariance);
jacobians   = [];
type        = 'pose-point-intrinsic';
iVertices   = [poseVertex pointVertex intrinsicVertex];
index       = edgeIndex;

%% 3. construct edge
obj.edges(edgeIndex) = Edge(value,covariance,jacobians,type,iVertices,index);

%% 4. update edge
%computes value & jacobians
obj = obj.updatePosePointIntrinsicEdge(config,edgeIndex);

%% 5. add index to pose vertex
obj.vertices(poseVertex).iEdges = [obj.vertices(poseVertex).iEdges edgeIndex];

end

