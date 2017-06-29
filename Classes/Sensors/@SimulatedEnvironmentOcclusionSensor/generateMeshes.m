%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 30/05/2017
% Contributors:
%--------------------------------------------------------------------------
function meshes = generateMeshes(self,environment,t)
%GENERATEMESHES takes the original sensor environment and generates the 
%meshes for the relevant objects within the camera FoV, in relative coordinate

currentSensorPose = self.get('GP_Pose',t).get('R3xso3Pose');
meshes = [];
for i=1:self.nObjects
    % calculations are done in SE3
    meshRelative = self.get('objects').get('mesh');
    meshAbsolute = zeros(size(meshRelative));
    for p=1:size(meshRelative,1)
        % original relative which is the relative to the object frame
        meshAbsolute(p,1:3) = RelativeToAbsoluteR3xso3(currentSensorPose,meshAbsolute(p,1:3)')';
        meshAbsolute(p,4:6) = RelativeToAbsoluteR3xso3(currentSensorPose,meshAbsolute(p,4:6)')';
        meshAbsolute(p,7:9) = RelativeToAbsoluteR3xso3(currentSensorPose,meshAbsolute(p,7:9)')';
    end
    for p=1:size(meshAbsolute,1)
        meshRelative(p,1:3) = AbsoluteToRelativeR3xso3(currentSensorPose,meshAbsolute(p,1:3)')';
        meshRelative(p,4:6) = AbsoluteToRelativeR3xso3(currentSensorPose,meshAbsolute(p,4:6)')';
        meshRelative(p,7:9) = AbsoluteToRelativeR3xso3(currentSensorPose,meshAbsolute(p,7:9)')';
    end
    meshes = [meshes; meshRelative];
end

visibility = zeros(size(meshes));
for p=1:size(meshes,1)
    for i=[1 4 7] % all 3 points in the row representation
        S2xRelativePosition = R3_S2xR(meshRelative(p,i:i+2));
        % check for FoV - saves in long run
        if (S2xRRelativePosition(1) >= self.fieldOfView(1)) && (S2xRRelativePosition(1) <= self.fieldOfView(2)) &&...
        (S2xRRelativePosition(2) >= self.fieldOfView(3)) && (S2xRRelativePosition(2) <= self.fieldOfView(4)) &&...   
        (S2xRRelativePosition(3) >= self.fieldOfView(5)) && (S2xRRelativePosition(3) <= self.fieldOfView(6))
            visibility(p,i:i+2) = 1;
        else
            visibility(p,i:i+2) = 0;
        end
        if ~any(visibility(p,:))
            continue
        else
            meshes = [meshes; meshRelative(p,:)];
        end
    end
end
    % only take mesh triangles where at least one point is in FoV
    
    
end

