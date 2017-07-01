function plotMap(config,map,camera,mapColour)
%PLOTMAP Summary of this function goes here
%   Detailed explanation goes here

%***assumes static features
hold on

%% 1. plot cameras
for i = 1:camera.nPoses
    iPose = camera.pose(:,i);
    if strcmp(config.poseParameterisation,'SE3')
        iPose = LogSE3_Rxt(iPose);
    end
    plotiCamera = plotCamera('Location',iPose(1:3),'Orientation',rot(-iPose(4:6))); %LHS invert pose
    plotiCamera.Opacity = 0.25;
    plotiCamera.Size = 0.1;
%     plotiCamera.Color = mapColour;
    plotiCamera.Color = [0 0 0];
end

%% 2. points
points = cell2mat({map.points.position}');
points = points(:,1); %assuming static points
points = reshape(points,3,map.nPoints);
plotPoints = plot3(points(1,:),points(2,:),points(3,:),'.');
set(plotPoints,'MarkerEdgeColor',mapColour)
set(plotPoints,'MarkerSize',10)

%% 3. plot entities 
for i = 1:map.nEntities
    switch map.entities(i).type
        case 'plane'
%             entityColour = rand(1,3);
            entityColour = mapColour;
            
            %plot plane points
            iPlanePoints = map.entities(i).iPoints;
            pointPositions = cell2mat({map.points(iPlanePoints).position}');
            pointPositions = pointPositions(:,1);
            pointPositions = reshape(pointPositions,3,map.entities(i).nPoints);
%             plotPlanePoints(i) = plot3(pointPositions(1,:),pointPositions(2,:),pointPositions(3,:),'*');
%             set(plotPlanePoints(i),'MarkerEdgeColor',entityColour)
%             set(plotPlanePoints(i),'MarkerSize',5)
            
            if config.plotPlanes
                %plot plane
                normal = map.entities(i).parameters(1:3,1); %assuming static entities
                distance = map.entities(i).parameters(4,1);
                plotPlane(normal,distance,pointPositions,entityColour);
            end

    end;
end

%% 4. plot objects
for i = 1:map.nObjects
end

end