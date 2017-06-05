%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 30/05/2017
% Contributors:
%--------------------------------------------------------------------------
function meshes = generateMeshes(self,t)
%GENERATEMESHES takes the sensor environment and generates the meshes for
%the relevant objects within the camera FoV, in relative coordinate

currentSensorPose = self.get('GP_Pose',t).get('logSE3Pose');
meshes = [];
for i=1:self.nObjects
    % calculations are done in SE3
    objectPose = self.get('objects',i).get('GP_Pose').get('logSE3Pose');
    meshesRelative = self.get('objects').get('objects',i).get('meshes');
    meshesAbsolute = zeros(size(meshesRelative));
    for p=1:size(meshesRelative,1)
        % put pose on the points
        meshesAbsolute(p,1:3) = Relative2AbsoluteSE3(objectPose,meshesRelative(p,1:3)')';
        meshesAbsolute(p,4:6) = Relative2AbsoluteSE3(objectPose,meshesRelative(p,4:6)')';
        meshesAbsolute(p,7:9) = Relative2AbsoluteSE3(objectPose,meshesRelative(p,7:9)')';
        
        % convert to camera relative points - Note: this replaces the
        % original relative which is the relative to the object frame
        meshesRelative(p,1:3) = Absolute2RelativeSE3(currentSensorPose,meshesAbsolute(p,1:3)')';
        meshesRelative(p,4:6) = Absolute2RelativeSE3(currentSensorPose,meshesAbsolute(p,4:6)')';
        meshesRelative(p,7:9) = Absolute2RelativeSE3(currentSensorPose,meshesAbsolute(p,7:9)')';
    end
    meshes = [meshes; meshesRelative];
    
end

visibility = zeros(size(meshes));
for p=1:size(meshes,1)
    for i=[1 4 7] % all 3 points in the row representation
        S2xRelativePosition = R3_S2xR(meshesRelative(p,i:i+2));
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
            meshes = [meshes; meshesRelative(p,:)];
        end
    end
end
    % only take mesh triangles where at least one point is in FoV
    
    
end

