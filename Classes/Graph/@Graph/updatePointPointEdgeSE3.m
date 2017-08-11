function [obj] = updatePointPointEdgeSE3(obj,config,iEdge)
%UPDATEPOINTPOINTEDGESE3 recomputes edge value, jacobian, covariance and switch
%properties from connected vertices.

%% 1. check order (assumes pose before point)
point1Vertex  = obj.edges(iEdge).iVertices(1);
point2Vertex = obj.edges(iEdge).iVertices(2);
assert(strcmp(obj.vertices(point1Vertex).type,'point'))
assert(strcmp(obj.vertices(point2Vertex).type,'point'))

%% 2. compute edge value & jacobians
value = obj.vertices(point1Vertex).value -...
    config.constantSE3Motion\obj.vertices(point2Vertex).value;
jacobian1 = eye(4);
jacobian2 = -config.constantSE3Motion;
jacobians   = {jacobian1,jacobian2};

%% 3. update properties
obj.edges(iEdge).value       = value;
obj.edges(iEdge).jacobians   = jacobians;

end

