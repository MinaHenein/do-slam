function [obj] = updateFixedAngleEdge(obj,config,iEdge)
%UPDATEFIXEDANGLEEDGE recomputes edge value, jacobian, covariance and switch
%properties from connected vertices.

%% 1. identify vertices
iVertices = obj.edges(iEdge).iVertices;
logicalPlaneVertices = logical(strcmp({obj.vertices(iVertices).type},'plane'));
iPlaneVertices = iVertices(logicalPlaneVertices);

%% 2. compute edge value
planeParameters = cell2mat({obj.vertices(iPlaneVertices).value});
dotProduct = dot(planeParameters(1:3,1),planeParameters(1:3,2));
value = dotProduct;

%% 3. jacobians
jacobians = cell(1,2);
jacobians(logicalPlaneVertices) = {[planeParameters(1:3,2)' 0],[planeParameters(1:3,1)' 0]}; 


%% 4. update properties
obj.edges(iEdge).value      = value;
obj.edges(iEdge).jacobians  = jacobians;

end

