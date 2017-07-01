function [obj] = constructPosePoseEdge(obj,config,edgeRow)
%CONSTRUCTPOSEPOSEEDGE constructs edge representing odometry measurement.
%Odometry measurement arriving at start of time N indicates relative
%pose of pose at time N-1 and pose at time N

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
pose1Vertex = edgeRow{3};
pose2Vertex = edgeRow{4};
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};

%% 2. properties
value       = edgeValue;
covariance  = upperTriVecToCov(edgeCovariance);
jacobians   = [];
type        = 'pose-pose';
iVertices   = [pose1Vertex pose2Vertex];
index       = edgeIndex;

%% 3. construct edge
obj.edges(edgeIndex) = Edge(value,covariance,jacobians,type,iVertices,index);

%% 4. update edge
%computes value & jacobians
obj = obj.updatePosePoseEdge(config,edgeIndex);

%% 5. add index to input pose vertex
obj.vertices(pose1Vertex).iEdges = [obj.vertices(pose1Vertex).iEdges edgeIndex];

end

