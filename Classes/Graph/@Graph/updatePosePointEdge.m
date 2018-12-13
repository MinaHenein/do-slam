function [obj] = updatePosePointEdge(obj,config,iEdge)
%UPDATEPOSEPOINTEDGE recomputes edge value, jacobian, covariance and switch
%properties from connected vertices.
if obj.edges(iEdge).active
    %% 1. check order (assumes pose before point)
    poseVertex  = obj.edges(iEdge).iVertices(1);
    pointVertex = obj.edges(iEdge).iVertices(2);
    assert(strcmp(obj.vertices(poseVertex).type,'pose'))
    assert(strcmp(obj.vertices(pointVertex).type,'point'))
    
    %% 2. compute edge value & jacobians
    switch config.cameraPointParameterisation
        case 'euclidean'
            pose          = obj.vertices(poseVertex).value;
            pointPosition = obj.vertices(pointVertex).value;
            %edge value - expected measurement from vertices
            if strcmp(config.landmarkErrorToMinimize,'reprojectionKnownIntrinsics')
                pointPositionRelative = AbsoluteToRelativePositionR3xso3Image(pose,...
                    pointPosition, config.intrinsics, config.R);
                pointPositionRelative = pointPositionRelative(1:2,:);
            else
                pointPositionRelative = config.absoluteToRelativePointHandle(pose,pointPosition);
            end
            %         pointPositionRelative = AbsoluteToRelativePosition(pose,pointPosition);
            %         pointPositionRelative = AbsolutePoint2RelativePoint3D(pose,pointPosition);
            value = pointPositionRelative;
            [jacobian1,jacobian2] = AbsoluteToRelativePositionJacobian(config,pose,pointPosition,pointPositionRelative);
        otherwise
            error('error: %s cameraPointParameterisation not implemented',config.cameraPointParameterisation)
    end
    jacobians   = {jacobian1,jacobian2};
    
    %% 3. update properties
    obj.edges(iEdge).value       = value;
    obj.edges(iEdge).jacobians   = jacobians;
    
end

end

