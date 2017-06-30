function [obj] = constructFixedAngleEdge(obj,config,edgeRow)
%CONSTRUCTFIXEDANGLEEDGE constructs edge representing dot product between
%two planes - there is no output vertex for this edge

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
planeVertices = edgeRow{3};
outputVertices = edgeRow{4};
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};

%% 2. properties
value       = edgeValue;
covariance  = edgeCovariance;
jacobians   = [];
type        = 'plane-plane-fixedAngle';
iVertices   = [planeVertices];
index       = edgeIndex;

%% 3. construct edge
obj.edges(edgeIndex) = Edge(value,covariance,jacobians,type,iVertices,index);

%% 4. update edge
%computes value & jacobians
obj = obj.updateFixedAngleEdge(config,edgeIndex);

%% 5. add index to pose vertex & plane vertex
obj.vertices(planeVertices(1)).iEdges = [obj.vertices(planeVertices(1)).iEdges edgeIndex];
obj.vertices(planeVertices(2)).iEdges = [obj.vertices(planeVertices(2)).iEdges edgeIndex];

end

