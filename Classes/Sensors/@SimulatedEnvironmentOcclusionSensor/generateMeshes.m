%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 30/05/2017
% Contributors:
%--------------------------------------------------------------------------
function meshes = generateMeshes(self,environment,t)
%GENERATEMESHES takes the original sensor environment and generates the 
%meshes for the relevant objects within the camera FoV, in relative coordinate

currentSensorPose = self.get('GP_Pose',t);
meshes = [];
for i=1:self.nObjects
    % calculations are done in SE3
    environmentPrimitive = environment.get('environmentPrimitives',i);
    meshRelative = [];
    if isa(environmentPrimitive,'EP_Default')
        meshPointsAbsolute = environmentPrimitive.get('meshPointsAbsolute',t);
        meshRelative = meshPointsAbsolute.RelativeToAbsolutePoint(currentSensorPose);
        pointsRelative = meshRelative.get('R3Position');
        links = environmentPrimitive.get('meshLinks');
        meshRelative = [pointsRelative(:,links(:,1))' pointsRelative(:,links(:,2))' pointsRelative(:,links(:,3))'];
    end
    meshes = [meshes; meshRelative];
end

visibility = zeros(size(meshes));
validRows = ones(size(meshes,1),1);
for p=1:validRows
    for j=[1 4 7] % all 3 points in the row representation
        S2xRRelativePosition = R3_S2xR(meshRelative(p,j:j+2));
        % check this meshTriangle points for FoV - saves in long run
        if (S2xRRelativePosition(1) >= self.fieldOfView(1)) && (S2xRRelativePosition(1) <= self.fieldOfView(2)) &&...
        (S2xRRelativePosition(2) >= self.fieldOfView(3)) && (S2xRRelativePosition(2) <= self.fieldOfView(4)) &&...   
        (S2xRRelativePosition(3) >= self.fieldOfView(5)) && (S2xRRelativePosition(3) <= self.fieldOfView(6))
            visibility(p,j) = 1;
        else
            visibility(p,j) = 0;
        end
    end
    if any(visibility(p,:))
        validRows(p,1) = 1;
    else
        validRows(p,1) = 0;
    end
end

meshes = meshes(find(validRows==1),:);
end

