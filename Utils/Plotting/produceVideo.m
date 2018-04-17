
function frames = produceVideo(sensors,cameraPoses,cameraGraphIndexes,...
    pointPositions,t,environment,nIterations)

nSteps = numel(t);
frames = struct('cdata',[],'colormap',[]);
currentFig = gcf;
poseConversion = GP_Pose([0 0 0 0 pi/2 0]','R3xso3');

sensorTimeStart = zeros(length(sensors),1);
sensorTimeEnd = zeros(length(sensors),1);
for i=1:length(sensors)
    sensorsPointVisibility{i} = sensors(i).get('pointVisibility');
    [~,col] = find(sensorsPointVisibility{i});
    sensorTimeStart(i,1) = col(1);
    sensorTimeEnd(i,1) = col(end);
end

for i= 1:nIterations
    hold on
    if i>1 && i<nIterations
        delete(findobj(gca,'Type','patch'))
    end
    pointGraphIndexes = [];
    for j= 1:nSteps
        for k= 1:length(sensors)
            %% GT data
            sensorPointVisibility = sensorsPointVisibility{k};
            sensorVisiblePointIndexes = find(sensorPointVisibility(:,j));
            sensorVisiblePointPositions = sensors(k).get('points',...
                sensorVisiblePointIndexes).get('R3Position',t(j));
            if ~isempty(sensorVisiblePointIndexes)
                pointGTHandle = plot3(sensorVisiblePointPositions(1,:),...
                    sensorVisiblePointPositions(2,:),sensorVisiblePointPositions(3,:),'r.');
                pointIndexes = sensorVisiblePointIndexes;
            end
            
            if j == nSteps && i== nIterations
                cameraGP_Pose = sensors(k).get('trajectory').get('GP_Pose',t(j));
                cameraPose = poseConversion.RelativeToAbsolutePose(cameraGP_Pose).get('R3xso3Pose');
                cameraGTHandle = plotCamera('Location',cameraPose(1:3),...
                    'Orientation',rot(-cameraPose(4:6)),'Color',([0 1 0])); %LHS invert pose
                cameraGTHandle.Opacity = 0.1;
                cameraGTHandle.Size = 0.5;
            end
            
            %% iteration results
            if j== sensorTimeStart(k,1)
                pointCount = cameraGraphIndexes(k)+1:cameraGraphIndexes(k)+size(pointIndexes,1);
                pointGraphIndexes{k,1} = pointCount;
                point_Positions = cell2mat(pointPositions(i,k));
                figHandle = plot3(point_Positions(1,pointCount-cameraGraphIndexes(k)),...
                    point_Positions(2,pointCount-cameraGraphIndexes(k)),...
                    point_Positions(3,pointCount-cameraGraphIndexes(k)),'b.');
                
            elseif j>sensorTimeStart(k,1) && j<=sensorTimeEnd(k,1)
                points = [pointGraphIndexes{k,:}];
                pointCount = points(end)+1:points(end)+size(pointIndexes,1);
                pointGraphIndexes{k,end+1} = pointCount;
                point_Positions = cell2mat(pointPositions(i,k));
                figHandle = plot3(point_Positions(1,pointCount-cameraGraphIndexes(k)),...
                    point_Positions(2,pointCount-cameraGraphIndexes(k)),...
                    point_Positions(3,pointCount-cameraGraphIndexes(k)),'b.');
                
            end
            
            if j >= sensorTimeStart(k) && j <= sensorTimeEnd(k)
                camera_Poses = cell2mat(cameraPoses(i));
                cameraPose = GP_Pose(camera_Poses(:,k));
                cameraPose = poseConversion.RelativeToAbsolutePose(cameraPose).get('R3xso3Pose');
                if k==1 && i==1
                    camera1Handle = plotCamera('Location',cameraPose(1:3),...
                        'Orientation',rot(-cameraPose(4:6)),'Color',([0 0 1])); %LHS invert pose
                    camera1Handle.Opacity = 0.1;
                    camera1Handle.Size = 0.5;
                elseif k==1 && i>1 && i<nIterations
                    camera1Handle.Location = cameraPose(1:3);
                    camera1Handle.Orientation = rot(-cameraPose(4:6));
                elseif k==2 && i==1
                    camera2Handle = plotCamera('Location',cameraPose(1:3),...
                        'Orientation',rot(-cameraPose(4:6)),'Color',([0 0 1])); %LHS invert pose
                    camera2Handle.Opacity = 0.1;
                    camera2Handle.Size = 0.5;
                elseif k==2 && i>1 && i<nIterations
                    camera2Handle.Location = cameraPose(1:3);
                    camera2Handle.Orientation = rot(-cameraPose(4:6));
                elseif k==3
                    camera3Handle = plotCamera('Location',cameraPose(1:3),...
                        'Orientation',rot(-cameraPose(4:6)),'Color',([0 0 1])); %LHS invert pose
                    camera3Handle.Opacity = 0.1;
                    camera3Handle.Size = 0.5;
                elseif k==3 && i>1 && i<nIterations
                    camera3Handle.Location = cameraPose(1:3);
                    camera3Handle.Orientation = rot(-cameraPose(4:6));
                end
            end
        
        for m=1:environment.nEnvironmentPrimitives
            primitive = environment.get('environmentPrimitives',m);
            if isa(primitive,'EP_Default')
                primitiveHandle(m) = primitive.plot(t(j),[0 1 0]);
            end
        end
        
        %drawnow
        view([-1 -2 3]);
        axis([-12 12 -12 12 -6 6]);
        [~] = set(currentFig);
        if j==1 && k==1
            frames(1) = getframe(gcf);
        else
            frames(end+1) = getframe(gcf);
        end
        if j < nSteps
            delete(primitiveHandle);
            if ~isempty(sensorVisiblePointIndexes)
                delete(pointGTHandle);
            end
            
        end
        end
    end
% hold off
% plot(0,0)
% currentFig.Children.Box = 'off';
end
end
