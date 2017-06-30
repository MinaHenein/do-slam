function [obj] = updatePlanePriorEdge(obj,config,iEdge)
%CONSTRUCTPOSEPRIOREDGE recomputes edge value, jacobian, covariance and switch
%properties from connected vertices.

%update value & jacobians
normal = obj.vertices(obj.edges(iEdge).iVertices).value(1:3);
obj.edges(iEdge).value = normal'*normal-1;
obj.edges(iEdge).jacobians = {[normal' 0]};

end

