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

nSteps = 0;
objPtsAtTime = [];
timeStep = 0;
for i=1:obj.nVertices
    if strcmp(obj.vertices(i).type,'pose')
       nSteps = nSteps+1;
       nPointsPerStep = 0;
       if obj.vertices(i).index < point2Vertex
        timeStep = timeStep +1;
       end
    end
    if strcmp(obj.vertices(i).type,'point')
        nPointsPerStep = nPointsPerStep +1;
        objPtsAtTime = [objPtsAtTime, obj.vertices(i).value];
    end
end

for i=1:size(objPtsAtTime,2)
    objPtsAtTime(:,i) = objPtsAtTime(:,i)/objPtsAtTime(end,i); 
end

[rotM,t] = Kabsch(cell2mat(objPtsRelative),objPtsAtTime(1:3,mapping(timeStep,nPointsPerStep)));
T = [rotM t; 0 0 0 1];

value = obj.vertices(point1Vertex).value -...
    T*inv(config.constantSE3Motion)*inv(T)*obj.vertices(point2Vertex).value;
jacobian1 = eye(4);
jacobian2 = -T*inv(config.constantSE3Motion)*inv(T);
jacobians   = {jacobian1,jacobian2};

%% 3. update properties
obj.edges(iEdge).value       = value;
obj.edges(iEdge).jacobians   = jacobians;

end

