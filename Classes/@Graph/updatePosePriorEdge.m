function [obj] = updatePosePriorEdge(obj,config,iEdge)
%CONSTRUCTPOSEPRIOREDGE recomputes edge value, jacobian, covariance and switch
%properties from connected vertices.

%update value
obj.edges(iEdge).value = obj.vertices(obj.edges(iEdge).iVertices).value;

end

