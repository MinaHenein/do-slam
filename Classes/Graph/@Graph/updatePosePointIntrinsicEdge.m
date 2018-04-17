function [obj] = updatePosePointIntrinsicEdge(obj,config,iEdge)
%UPDATEPOSEPOINTINTRINSICEDGE recomputes edge value, jacobian, covariance and switch
%properties from connected vertices.

%% 1. check order (assumes pose before point)
poseVertex  = obj.edges(iEdge).iVertices(1);
pointVertex = obj.edges(iEdge).iVertices(2);
intrinsicVertex = obj.edges(iEdge).iVertices(3);
assert(strcmp(obj.vertices(poseVertex).type,'pose'))
assert(strcmp(obj.vertices(pointVertex).type,'point'))
assert(strcmp(obj.vertices(intrinsicVertex).type,'intrinsics'))

%% 2. compute edge value & jacobians
switch config.cameraPointParameterisation
    case 'euclidean'
        pose          = obj.vertices(poseVertex).value;
        pointPosition = obj.vertices(pointVertex).value;
        if strcmp(config.landmarkErrorToMinimize,'reprojection')
            intrinsics = obj.vertices(intrinsicVertex).value;
            %edge value - expected measurement from vertices
            pointPositionRelative = config.absoluteToRelativePointHandle(pose,pointPosition,intrinsics);
            value = pointPositionRelative;
            [jacobian1,jacobian2,jacobian3] = AbsoluteToRelativePositionJacobianImage(config,pose,pointPosition,...
                pointPositionRelative,intrinsics);
        else
            %edge value - expected measurement from vertices
            pointPositionRelative = config.absoluteToRelativePointHandle(pose,pointPosition);
            %         pointPositionRelative = AbsoluteToRelativePosition(pose,pointPosition);
            %         pointPositionRelative = AbsolutePoint2RelativePoint3D(pose,pointPosition);
            value = pointPositionRelative;
            [jacobian1,jacobian2] = AbsoluteToRelativePositionJacobian(config,pose,pointPosition,pointPositionRelative);
        end
    otherwise
        error('error: %s cameraPointParameterisation not implemented',config.cameraPointParameterisation)
end
jacobians   = {jacobian1,jacobian2, jacobian3};

%% 3. update properties
obj.edges(iEdge).value       = value;
obj.edges(iEdge).jacobians   = jacobians;

end

