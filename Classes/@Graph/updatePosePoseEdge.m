function [obj] = updatePosePoseEdge(obj,config,iEdge)
%CONSTRUCTPOSEPOSEEDGE recomputes edge value, jacobian, covariance and switch
%properties from connected vertices.

%% 1. check order (assumes pose1 before pose2)
vertex1 = obj.edges(iEdge).iVertices(1);
vertex2 = obj.edges(iEdge).iVertices(2);
assert(vertex1 < vertex2)

%% 2. compute edge value and jacobian
switch config.cameraControlInput
    case 'relativePose'
        pose1 = obj.vertices(vertex1).value;   
        pose2 = obj.vertices(vertex2).value;
        %edge value - expected odometry from vertices
        relativePose = config.absoluteToRelativePoseHandle(pose1,pose2);
%         relativePose(4:6) = wrapAxisAngle(relativePose(4:6),'pi');
%         relativePose = AbsoluteToRelativePose(pose1,pose2);
%         relativePose = Absolute2RelativeSE3(pose1,pose2);
        value = relativePose;
%         jacobian1 = RelativeToAbsolutePoseJacobian(pose1,pose2,relativePose);
%         jacobian1 = AbsoluteToRelativePoseJacobian(pose1,relativePose,pose2);
%         jacobian2 = AbsoluteToRelativePoseJacobian(pose2,relativePoseInv,pose1);
        [jacobian1,jacobian2] = AbsoluteToRelativePoseJacobian(config,pose1,pose2,relativePose);
%         jacobian2 = -eye(6);
    otherwise
        error('error: %s cameraControlInput not implemented',config.cameraControlInput)
end
jacobians   = {jacobian1,jacobian2};

%% 3. update properties
obj.edges(iEdge).value       = value;
obj.edges(iEdge).jacobians   = jacobians;

end

