function [obj] = update2PointsSE3MotionEdge(obj,config,iEdge)
%UPDATEPOINTVELOCITYEDGE recomputes edge value, jacobian, covariance and switch
%properties from connected vertices.

%% 1. check order (assumes pose before point)
iVertices = obj.edges(iEdge).iVertices;
logicalPointVertices = logical(strcmp({obj.vertices(iVertices).type},'point'));
logicalSE3MotionVertex = logical(strcmp({obj.vertices(iVertices).type},'SE3Motion'));
iPointVertices = iVertices(logicalPointVertices);
iSE3MotionVertex = iVertices(logicalSE3MotionVertex);

%% 2. compute edge value & jacobians
pointPositions = cell2mat({obj.vertices(iPointVertices).value});
SE3Motion = obj.vertices(iSE3MotionVertex).value;
SE3MotionTransformationMatrix = [rot(SE3Motion(4:6)) SE3Motion(1:3); 0 0 0 1];
value = pointPositions(:,1) - SE3MotionTransformationMatrix\pointPositions(:,2);
jacobian1 = eye(4);
jacobian2 = -inv(SE3MotionTransformationMatrix);

point1 = pointPositions(:,1);
point2 = pointPositions(:,2);
J = update2PointsSE3MotionEdgeJacobian(config,point1,point2,SE3Motion);
jacobian3 = J; %computed numerically
jacobians   = {jacobian1,jacobian2,jacobian3};

%% 3. update properties
obj.edges(iEdge).value       = value;
obj.edges(iEdge).jacobians   = jacobians;

end
