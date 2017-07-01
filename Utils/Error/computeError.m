function [error] = computeError(map,camera,measurements,graph)
%COMPUTEERROR Summary of this function goes here
%   Detailed explanation goes here

%identify types
if (isempty(graph.iPoseVertices) || isempty(graph.iPointVertices))
    graph = graph.identifyTypes;
end

error.poseTranslation = 0;
error.poseRotation    = 0;
error.allPoseTranslation = 0;
error.allPoseRotation    = 0;
error.points = 0;
error.allPoints = 0;
error.planeNormalError = 0;
error.planeDistanceError = 0;
error.planeFitError = 0;
%% 1. Pose Error
% 1.1. corresponding poses
for i = 1:camera.nPoses
    poseEstimate = graph.vertices(graph.iPoseVertices(i)).value;
    poseGroundTruth = camera.pose(:,i);
    relativePose = SmartMinus(poseEstimate,poseGroundTruth);
    translationError = relativePose(1:3);
    rotationError = relativePose(4:6);
    %this is equivalent 
%     translationError = SmartMinus([poseEstimate(1:3);0;0;0],[poseGroundTruth(1:3);0;0;0]);
%     rotationError    = SmartMinus([0;0;0;poseEstimate(4:6)],[0;0;0;poseGroundTruth(4:6)]);
    %add
    error.poseTranslation = error.poseTranslation + norm(translationError);
    error.poseRotation    = error.poseRotation + norm(rotationError);
end
% 1.2. all to all pose error
translationError = 0;
rotationError    = 0;
for i = 1:camera.nPoses
    iPoseEstimate = graph.vertices(graph.iPoseVertices(i)).value;
    iPoseGroundTruth = camera.pose(:,i);
    for j = i+1:camera.nPoses
        jPoseEstimate = graph.vertices(graph.iPoseVertices(j)).value;
        jPoseGroundTruth = camera.pose(:,j);
        relativePoseEstimate = SmartMinus(iPoseEstimate,jPoseEstimate);
        relativePoseGroundTruth = SmartMinus(iPoseGroundTruth,jPoseGroundTruth);
        %add
        poseError = SmartMinus(relativePoseEstimate,relativePoseGroundTruth);
        translationError = translationError + norm(poseError(1:3));
        rotationError = rotationError + norm(poseError(4:6));
    end
end
error.allPoseTranslation = translationError;
error.allPoseRotation    = rotationError;

%% 2. Point Error
% 2.1. Corresponding points
for i = 1:map.nPoints
    positionEstimate = graph.vertices(graph.iPointVertices(i)).value;
    %corresponding point in map
    mapIndex = measurements.map.points(i).index;
    positionGroundTruth = map.points(mapIndex).position(:,1);
    error.points = error.points + norm(positionGroundTruth - positionEstimate);
end
% 2.2. all to all point error
pointError = 0;
for i = 1:map.nPoints
    iPositionEstimate    = graph.vertices(graph.iPointVertices(i)).value;
    iMapIndex = measurements.map.points(i).index;
    iPositionGroundTruth = map.points(iMapIndex).position(:,1); %corresponding
    for j = i+1:map.nPoints
        jPositionEstimate    = graph.vertices(graph.iPointVertices(j)).value;
        jMapIndex = measurements.map.points(j).index;
        jPositionGroundTruth = map.points(jMapIndex).position(:,1); %corresponding
        %add
        pointErrorEstimate    = norm(jPositionEstimate - iPositionEstimate);
        pointErrorGroundTruth = norm(jPositionGroundTruth - iPositionGroundTruth);
        pointError = pointError + abs(pointErrorGroundTruth - pointErrorEstimate);
    end
end
error.allPoints = pointError;

%% 3. Plane Error
%mapping from graph to map point indexes
graphToMap = zeros(map.nPoints,1);
mapToGraph = zeros(map.nPoints,1);
for i = 1:map.nPoints
    graphToMap(i) = measurements.map.points(i).index;
    mapToGraph(measurements.map.points(i).index) = i;
end

planeVertices = graph.identifyVertices('plane');
nPlaneVertices = numel(planeVertices);
nPlanes = sum(strcmp({map.entities.type},'plane'));
planeEntities = find(strcmp({map.entities.type},'plane'));
if (nPlaneVertices == 0) && (nPlanes > 0) %no constraints in graph
    fitPlanes = 1;
else
    fitPlanes = 0;
end

planeNormalError = 0;
planeDistanceError = 0;
% planeFitError = 0;
planeFitErrorEstimate = 0;
planeFitErrorGroundTruth = 0;
for i = 1:nPlanes    
    %ground truth plane parameters    
    planeParametersGroundTruth = map.entities(planeEntities(i)).parameters(:,1);
    %ground truth plane points
    planePointsGroundTruth = map.entities(planeEntities(i)).iPoints;
    planePoints = cell2mat({map.points(planePointsGroundTruth).position}');
    pointPositionsGroundTruth = reshape(planePoints(:,1),3,numel(planePointsGroundTruth));
    
    %get points in plane i
    planePointsGraph = mapToGraph(planePointsGroundTruth);
    pointPositionsEstimate = [graph.vertices(graph.iPointVertices(planePointsGraph)).value];
    if fitPlanes        
        %fit plane to points
        [~,planeParametersEstimate] = fitPlane(pointPositionsEstimate);
    else
        planeParametersEstimate = graph.vertices(planeVertices(i)).value;
    end
    
    %plane parameters error
    planeNormalError = planeNormalError + abs(dot(planeParametersGroundTruth(1:3),planeParametersEstimate(1:3)));
    planeDistanceError = planeDistanceError + abs(planeParametersGroundTruth(4) - planeParametersEstimate(4));
    
    %plane fit error
    planeFitErrorEstimate = planeFitErrorEstimate + norm(pointPositionsEstimate'*planeParametersEstimate(1:3) - planeParametersEstimate(4));
    planeFitErrorGroundTruth = planeFitErrorGroundTruth + norm(pointPositionsGroundTruth'*planeParametersGroundTruth(1:3) - planeParametersGroundTruth(4));
%     planeFitError = planeFitError + abs(planeFitErrorGroundTruth - planeFitErrorEstimate);    
end
error.planeNormalError = planeNormalError;
error.planeDistanceError = planeDistanceError;
% error.planeFitError = planeFitError;
error.planeFitErrorEstimate = planeFitErrorEstimate;
error.planeFitErrorGroundTruth = planeFitErrorGroundTruth;

end

