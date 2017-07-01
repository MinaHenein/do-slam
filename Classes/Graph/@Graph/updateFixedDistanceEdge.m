function [obj] = updateFixedDistanceEdge(obj,config,iEdge)
%UPDATEFIXEDDISTANCEEDGE recomputes edge value, jacobian, covariance and switch
%properties from connected vertices.

%% 1. identify vertices
iVertices = obj.edges(iEdge).iVertices;
logicalPlaneVertices = logical(strcmp({obj.vertices(iVertices).type},'plane'));
iPlaneVertices = iVertices(logicalPlaneVertices);

%% 2. compute edge value
planeParameters = cell2mat({obj.vertices(iPlaneVertices).value});
% p1 = planeParameters(1:3,1)*planeParameters(4,1);
% p2 = planeParameters(1:3,2)*planeParameters(4,2);
% planePlaneDistance = norm(p1-p2);
planePlaneDistance = abs(planeParameters(4,1)-planeParameters(4,2)); %WRONG???
value = planePlaneDistance;

%% 3. jacobians
jacobians = cell(1,2);
jacobians(logicalPlaneVertices) = {[0 0 0 1],[0 0 0 1]}; 
%% 4. update properties
obj.edges(iEdge).value      = value;
obj.edges(iEdge).jacobians  = jacobians;

end

