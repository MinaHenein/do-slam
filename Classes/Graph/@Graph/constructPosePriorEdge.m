function [obj] = constructPosePriorEdge(obj,config,edgeRow)
%CONSTRUCTPOSEPRIOREDGE Constructs prior on pose vertex. prior value comes
%from ground truth graph file.

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
inputVertices = edgeRow{3};
outputVertices = edgeRow{4};
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};

%% 2. properties
value       = edgeValue;
covariance  = edgeCovariance;
jacobians   = {-eye(6)};
type        = 'posePrior';
switchable  = 0;
switchValue = [];
active      = 1;
iVertices   = outputVertices;
observation = [];
index       = edgeIndex;

%% 3. construct edge
obj.edges(edgeIndex) = Edge(value,covariance,jacobians,type,iVertices,index);
                      
%% 4. update edge
obj = obj.updatePosePriorEdge(config,edgeIndex);   

end

