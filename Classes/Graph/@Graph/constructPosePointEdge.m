function [obj] = constructPosePointEdge(obj,config,edgeRow)
%CONSTRUCTPOSEPOINTEDGE constructs edge representing measurement of
%relative position of a point from a pose

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
poseVertex = edgeRow{3};
pointVertex = edgeRow{4};
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};

%% 2. properties
value       = edgeValue;
covariance  = upperTriVecToCov(edgeCovariance);
jacobians   = [];
type        = 'pose-point';
iVertices   = [poseVertex pointVertex];
index       = edgeIndex;

%% 3. construct edge
obj.edges(edgeIndex) = Edge(value,covariance,jacobians,type,iVertices,index);

%% 4. update edge
%computes value & jacobians
obj = obj.updatePosePointEdge(config,edgeIndex);

%% 5. add index to pose vertex
obj.vertices(poseVertex).iEdges = [obj.vertices(poseVertex).iEdges edgeIndex];

end

