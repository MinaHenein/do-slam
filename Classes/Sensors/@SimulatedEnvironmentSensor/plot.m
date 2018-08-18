%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 30/06/2017
% Contributors:
%--------------------------------------------------------------------------

function varargout = plot(self,t,varargin)
%PLOT Plots sensor data, similar to environment but also includes
%visibility. Optionally can also plot the sensor information on the
%environment.
nSteps = numel(t);
hold on
frames(nSteps) = struct('cdata',[],'colormap',[]);
currentFig = gcf;
pointVisibility = self.get('pointVisibility');
poseConversion = GP_Pose([0 0 0 0 pi/2 0]','R3xso3');

for i=1:nSteps
    figHandles = []; % holds figure handles
    visiblePointIndexes = find(pointVisibility(:,i));
    invisiblePointIndexes = find(~pointVisibility(:,i));
    visiblePointPositions = self.get('points',visiblePointIndexes).get('R3Position',t(i));
    invisiblePointPositions = self.get('points',invisiblePointIndexes).get('R3Position',t(i));
    if ~isempty(visiblePointIndexes)
        figHandles(1) = plot3(visiblePointPositions(1,:),visiblePointPositions(2,:),visiblePointPositions(3,:),'r.');
    end
    if ~isempty(invisiblePointIndexes)
        figHandles(2) = plot3(invisiblePointPositions(1,:),invisiblePointPositions(2,:),invisiblePointPositions(3,:),'k.');
    end
    cameraGP_Pose = self.get('trajectory').get('GP_Pose',t(i));
    cameraPose = poseConversion.RelativeToAbsolutePose(cameraGP_Pose).get('R3xso3Pose');
    cameraHandle = plotCamera('Location',cameraPose(1:3),'Orientation',rot(-cameraPose(4:6)));%,'Color',varargin{2}); %LHS invert pose
    cameraHandle.Opacity = 0.1;
    cameraHandle.Size = 0.5;
    
    if numel(varargin)>0
        environment = varargin{1};
            for k=1:environment.nEnvironmentPrimitives
                primitive = environment.get('environmentPrimitives',k);
                if isa(primitive,'EP_Default')
                    figHandles(k+2) = primitive.plot(t(i),[0 1 0]);
                end
            end
    end
    
    drawnow
    [~] = set(currentFig);
    frames(i) = getframe(gcf); % put IF statement for nargout, add movie mode integration here
    pause(0)
    if i < nSteps
        delete(figHandles);
        delete(cameraHandle);
    end
end

if nargout>0
    varargout{1} = frames;
end

end

