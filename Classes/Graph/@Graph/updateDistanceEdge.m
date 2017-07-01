function [obj] = updateDistanceEdge(obj,config,iEdge)
%UPDATEANGLEEDGE Summary of this function goes here
%   Detailed explanation goes here

%% 1. identify vertices
iVertices = obj.edges(iEdge).iVertices;
logicalPlaneVertices  = logical(strcmp({obj.vertices(iVertices).type},'plane'));
logicalDistanceVertex = logical(strcmp({obj.vertices(iVertices).type},'distance'));
iPlaneVertices  = iVertices(logicalPlaneVertices);
iDistanceVertex = iVertices(logicalDistanceVertex);

%% 2. compute edge value
planeParameters = cell2mat({obj.vertices(iPlaneVertices).value});
planePlaneDistance = abs(planeParameters(4,1)-planeParameters(4,2)); %WRONG???
value = planePlaneDistance;

%% 3. jacobians
jacobians = cell(1,3);
jacobians(logicalPlaneVertices)  = {[0 0 0 1],[0 0 0 1]}; 
jacobians{logicalDistanceVertex} = eye(1);

%% 4. update properties
obj.edges(iEdge).value      = value;
obj.edges(iEdge).jacobians  = jacobians;

end

