function [obj] = updateAngleEdge(obj,config,iEdge)
%UPDATEANGLEEDGE recomputes edge value, jacobian, covariance and switch
%properties from connected vertices.

%% 1. identify vertices
iVertices = obj.edges(iEdge).iVertices;
logicalPlaneVertices = logical(strcmp({obj.vertices(iVertices).type},'plane'));
logicalAngleVertex   = logical(strcmp({obj.vertices(iVertices).type},'angle'));
iPlaneVertices = iVertices(logicalPlaneVertices);
iAngleVertex   = iVertices(logicalAngleVertex);

%% 2. compute edge value
planeParameters = cell2mat({obj.vertices(iPlaneVertices).value});
normal1 = planeParameters(1:3,1);
normal2 = planeParameters(1:3,2);
dotProduct = dot(normal1,normal2);
value = dotProduct;

%% 3. jacobians
jacobians = cell(1,3);
jacobians(logicalPlaneVertices) = {[((eye(3)-normal1*normal1')*normal2)' 0],[((eye(3)-normal2*normal2')*normal1)' 0]}; 
jacobians{logicalAngleVertex}   = eye(1);

%% 4. update properties
obj.edges(iEdge).value      = value;
obj.edges(iEdge).jacobians  = jacobians;

end

