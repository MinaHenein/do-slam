function [obj] = updatePointPlaneEdge(obj,config,iEdge)
%UPDATEPOINTPLANEEDGE recomputes edge value, jacobian, covariance and switch
%properties from connected vertices.

%% 1. identify vertices
iVertices = obj.edges(iEdge).iVertices;
logicalPointVertex  = logical(strcmp({obj.vertices(iVertices).type},'point'));
logicalPlaneVertex  = logical(strcmp({obj.vertices(iVertices).type},'plane'));
iPointVertex  = iVertices(logicalPointVertex);
iPlaneVertex  = iVertices(logicalPlaneVertex);

%% 2. compute edge value
pointPosition = obj.vertices(iPointVertex).value;
planeParameters = obj.vertices(iPlaneVertex).value;
pointPlaneDistance = pointPosition'*planeParameters(1:3) - planeParameters(4);
value = pointPlaneDistance;

%% 3. jacobians
%jacobians
jacobians = cell(1,2);
jacobians{logicalPointVertex} = planeParameters(1:3)';
switch config.planeNormalParameterisation
    case 'R3'
        jacobians{logicalPlaneVertex} = [pointPosition' -1]; 
    case 'S2'
        jacobians{logicalPlaneVertex} = [((eye(3)-planeParameters(1:3)*planeParameters(1:3)')*pointPosition)' -1]; 
    otherwise; error('parameterisation not implemented')
end
%% 4. update properties
obj.edges(iEdge).value      = value;
obj.edges(iEdge).jacobians  = jacobians;

end

