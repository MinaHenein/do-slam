%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 30/06/2017
% Contributors:
%--------------------------------------------------------------------------

function plot(self,t,varargin)
%PLOT Plots sensor data, similar to environment but also includes
%visibility. Optionally can also plot the sensor information on the
%environment.
fig = gcf; % creates new figure or grabs current handle
nSteps = numel(t);
hold on

pointVisibility = self.get('pointVisibility');
for i=1:nSteps
    clf(fig)
    visiblePointIndexes = find(pointVisibility(:,i));
    invisiblePointIndexes = find(~pointVisibility(:,i));
    visiblePointPositions = self.get('points',visiblePointIndexes).get('R3Position',t(i));
    invisiblePointPositions = self.get('points',invisiblePointIndexes).get('R3Position',t(i));
    if ~isempty(visiblePointIndexes)
        plot3(visiblePointPositions(1,:),visiblePointPositions(2,:),visiblePointPositions(3,:),'g.');
    end
    if ~isempty(invisiblePointIndexes)
        plot3(invisiblePointPositions(1,:),invisiblePointPositions(2,:),invisiblePointPositions(3,:),'k.');
    end
    cameraPose = self.get('trajectory').get('R3xso3Pose',t(i));
    cameraHandle = plotCamera('Location',cameraPose(1:3),'Orientation',rot(-cameraPose(4:6))); %LHS invert pose
    cameraHandle.Opacity = 0.1;
    cameraHandle.Size = 0.1;
end



end

