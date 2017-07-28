function [obj] = update2PointsVelocityEdge(obj,config,iEdge)
%UPDATEPOINTVELOCITYEDGE recomputes edge value, jacobian, covariance and switch
%properties from connected vertices.

%% 1. check order (assumes pose before point)
iVertices = obj.edges(iEdge).iVertices;
logicalPointVertices = logical(strcmp({obj.vertices(iVertices).type},'point'));
logicalVelocityVertex = logical(strcmp({obj.vertices(iVertices).type},'velocity'));
iPointVertices = iVertices(logicalPointVertices);
iVelocityVertex = iVertices(logicalVelocityVertex);

%% 2. compute edge value & jacobians
pointPositions = cell2mat({obj.vertices(iPointVertices).value});
velocity = obj.vertices(iVelocityVertex).value;
value = velocity - norm(pointPositions(1:3,2)-pointPositions(1:3,1));
jacobian1 = (-sign(pointPositions(1:3,1)-pointPositions(1:3,2)))';
jacobian2 = (sign(pointPositions(1:3,1)-pointPositions(1:3,2)))';
jacobian3 = 1;
jacobians   = {jacobian1,jacobian2,jacobian3};

%% 3. update properties
obj.edges(iEdge).value       = value;
obj.edges(iEdge).jacobians   = jacobians;

end

