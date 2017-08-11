function [obj] = updatePointPointEdgeSE3_Mina(obj,config,iEdge)
%UPDATEPOINTPOINTEDGESE3 recomputes edge value, jacobian, covariance and switch
%properties from connected vertices.

%% 1. check order (assumes pose before point)
point1Vertex  = obj.edges(iEdge).iVertices(1);
point2Vertex = obj.edges(iEdge).iVertices(2);
assert(strcmp(obj.vertices(point1Vertex).type,'point'))
assert(strcmp(obj.vertices(point2Vertex).type,'point'))

%% 2. compute edge value & jacobians
objPtsRelative = getGlobalObjPtsRelative;

if (point1Vertex==2 && point2Vertex==6) || (point1Vertex==6 && point2Vertex==10)
    a = objPtsRelative{1,1};
    disp('HARD CODED!! SHOULD CHANGE')
elseif (point1Vertex==3 && point2Vertex==7) || (point1Vertex==7 && point2Vertex==11)
    a = objPtsRelative{1,2};
    disp('HARD CODED!! SHOULD CHANGE')
elseif (point1Vertex==4 && point2Vertex==8) || (point1Vertex==8 && point2Vertex==12)
    a = objPtsRelative{1,3};
    disp('HARD CODED!! SHOULD CHANGE')
end

T = vectors2TransformationMatrix(obj.vertices(point2Vertex).value, a);
       
value = obj.vertices(point1Vertex).value -...
    T*inv(config.constantSE3Motion)*inv(T)*obj.vertices(point2Vertex).value;
jacobian1 = eye(4);
jacobian2 = -T*inv(config.constantSE3Motion)*inv(T);
jacobians   = {jacobian1,jacobian2};

%% 3. update properties
obj.edges(iEdge).value       = value;
obj.edges(iEdge).jacobians   = jacobians;

end

