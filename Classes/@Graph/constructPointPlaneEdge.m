function [obj] = constructPointPlaneEdge(obj,config,edgeRow)
%CONSTRUCTPOINTPLANEEDGE constructs edge reprsenting distance between a
%point and a plane

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
pointVertex = edgeRow{3};
planeVertex = edgeRow{4};
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};

%% 2. properties
value       = edgeValue;
covariance  = edgeCovariance;
jacobians   = [];
type        = 'point-plane';
iVertices   = [pointVertex planeVertex];
index       = edgeIndex;

%% 3. construct edge
obj.edges(edgeIndex) = Edge(value,covariance,jacobians,type,iVertices,index);

%% 4. update edge
%computes value & jacobians
obj = obj.updatePointPlaneEdge(config,edgeIndex);

%% 5. add index to pose vertex & plane vertex
obj.vertices(pointVertex).iEdges = [obj.vertices(pointVertex).iEdges edgeIndex];
obj.vertices(planeVertex).iEdges = unique([obj.vertices(planeVertex).iEdges edgeIndex]);

end

