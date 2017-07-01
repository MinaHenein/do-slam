function [obj] = constructPlanePriorEdge(obj,config,edgeRow)
%CONSTRUCTPLANEPRIOREDGE constructs edge constraining plane normal vector
%to have magnitude = 1.
%Used when config.planeNormalParameterisation = 'R3'

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
planeVertex = edgeRow{3};
outputVertex = edgeRow{4};
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};

%% 2. properties
value       = edgeValue;
covariance  = edgeCovariance;
jacobians   = [];
type        = 'planePrior';
iVertices   = [planeVertex];
index       = edgeIndex;

%% 3. construct edge
obj.edges(edgeIndex) = Edge(value,covariance,jacobians,type,iVertices,index);

%% 4. update edge
%computes value & jacobians
obj = obj.updatePlanePriorEdge(edgeIndex);

%% 5. add index to pose vertex & plane vertex
obj.vertices(planeVertex).iEdges = unique([obj.vertices(planeVertex).iEdges edgeIndex]);

end

