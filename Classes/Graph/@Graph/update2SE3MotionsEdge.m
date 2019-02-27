function [obj] = update2SE3MotionsEdge(obj,config,iEdge)
%UPDATE2SE3MOTIONSEDGE recomputes edge value, jacobian, covariance and switch
%properties from connected vertices.

if obj.edges(iEdge).active
    
    %% 1. check order (assumes pose before point)
    iVertices = obj.edges(iEdge).iVertices;
    logicalSE3MotionVertices = logical(strcmp({obj.vertices(iVertices).type},'SE3Motion'));
    iSE3MotionVertices = iVertices(logicalSE3MotionVertices);
    
    %% 2. compute edge value & jacobians
    SE3MotionVertex1 = obj.vertices(iSE3MotionVertices(1)).value;
    SE3MotionVertex2 = obj.vertices(iSE3MotionVertices(2)).value;
    
    relativeMotion = config.absoluteToRelativePoseHandle(SE3MotionVertex1,SE3MotionVertex2);
    
    value = relativeMotion;
    
    [jacobian1,jacobian2] = AbsoluteToRelativePoseJacobian(config,SE3MotionVertex1,SE3MotionVertex2,relativeMotion);
    jacobians   = {jacobian1,jacobian2};
    
    %% 3. update properties
    obj.edges(iEdge).value       = value;
    obj.edges(iEdge).jacobians   = jacobians;
    
end
end