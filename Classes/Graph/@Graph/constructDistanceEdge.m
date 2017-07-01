function [obj] = constructDistanceEdge(obj,config,edgeRow)
%CONSTRUCTANGLEEDGE constructs hyperedge representing distance between two
%planes
%   inputVertices are planes, outputVertex is distance

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
planeVertices = edgeRow{3};
distanceVertex = edgeRow{4};
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};

%% 2. properties
value       = edgeValue;
covariance  = edgeCovariance;
jacobians   = [];
type        = 'plane-plane-distance';
iVertices   = [planeVertices distanceVertex];
index       = edgeIndex;

%% 3. construct edge
obj.edges(edgeIndex) = Edge(value,covariance,jacobians,type,iVertices,index);

%% 4. update edge
%computes value & jacobians
obj = obj.updateDistanceEdge(edgeIndex);

%% 5. add index to pose vertex & plane vertex
obj.vertices(planeVertices(1)).iEdges = [obj.vertices(planeVertices(1)).iEdges edgeIndex];
obj.vertices(planeVertices(2)).iEdges = [obj.vertices(planeVertices(2)).iEdges edgeIndex];
obj.vertices(distanceVertex).iEdges = unique([obj.vertices(distanceVertex).iEdges edgeIndex]);

end

