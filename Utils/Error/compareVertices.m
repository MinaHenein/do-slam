function [pointError,poseTError,poseRError,planeFitError,planeNError,planeDError,allPoseError,allPointError] = compareVertices(map,camera,measurements,graph)
%COMPAREVERTICES Summary of this function goes here
%   Detailed explanation goes here

%identify types
if (isempty(graph.iPoseVertices) || isempty(graph.iPointVertices))
    graph = graph.identifyTypes;
end

poseTError = zeros(6,camera.nPoses);
poseRError = zeros(6,camera.nPoses);
pointError = zeros(3,map.nPoints);
planeFitError = zeros(1,map.nEntities);
planeNError = zeros(1,map.nEntities);
planeDError = zeros(1,map.nEntities);

%% pose error
for i = 1:camera.nPoses
    poseEstimate = graph.vertices(graph.iPoseVertices(i)).value;
    poseGroundTruth = camera.pose(:,i);
    poseTError(:,i) = SmartMinus([poseEstimate(1:3);0;0;0],[poseGroundTruth(1:3);0;0;0]);
    poseRError(:,i) = SmartMinus([0;0;0;poseEstimate(4:6)],[0;0;0;poseGroundTruth(4:6)]);
end

%% point error
for i = 1:map.nPoints
    positionEstimate = graph.vertices(graph.iPointVertices(i)).value;
    mapIndex = measurements.map.points(i).index;
    positionGroundTruth = map.points(mapIndex).position(:,1);
    pointError(:,i) = positionGroundTruth - positionEstimate;
end

%% plane fit error & plane error
%get plane vertices
planeVertices = graph.identifyVertices('plane');
nPlaneVertices = numel(planeVertices);
for i = 1:nPlaneVertices
    %distance between points-plane for plane estimate
    planeEdges = graph.vertices(planeVertices(i)).iEdges;
    edgeTypes = {graph.edges(planeEdges).type}';
    pointPlaneEdges = strcmp(edgeTypes,'point-plane');
    iVertices = [graph.edges(planeEdges(pointPlaneEdges)).iVertices]';
    vertexTypes = {graph.vertices(iVertices).type}';
    pointVertices = strcmp(vertexTypes,'point');
    iPointVertices = iVertices(pointVertices);
    iPointVertices = unique(iPointVertices);
    pointPositions = [graph.vertices(iPointVertices).value];
    planeParametersEstimate = graph.vertices(planeVertices(i)).value;
    planeEstimateError = pointPositions'*planeParametersEstimate(1:3) - planeParametersEstimate(4);

    %distance between points-plane for ground truth
    mapIndex = measurements.map.entities(i).index; %***is this the right plane?
    planeParametersGroundTruth = map.entities(mapIndex).parameters(:,1);
    pointPositionsCell = {map.points(map.entities(mapIndex).iPoints).position}';
    pointPositions = cell2mat(pointPositionsCell);
    pointPositions = pointPositions(:,1); %assuming static points
    pointPositions = reshape(pointPositions,3,numel(map.entities(mapIndex).iPoints));
    planeGroundTruthError = pointPositions'*planeParametersGroundTruth(1:3) - planeParametersGroundTruth(4);
    
    planeFitError(:,i) = norm(planeGroundTruthError - planeEstimateError);
    planeNError(1,i) = dot(planeParametersGroundTruth(1:3),planeParametersEstimate(1:3));
    planeDError(1,i) = planeParametersGroundTruth(4) - planeParametersEstimate(4);
end

%% all-to-all pose error
%estimate
relativePosesEstimate = [];
for i = 1:camera.nPoses
    iPose = graph.vertices(graph.iPoseVertices(i)).value;
    for j = i+1:camera.nPoses
        jPose = graph.vertices(graph.iPoseVertices(j)).value;
        relativePosesEstimate = [relativePosesEstimate SmartMinus(iPose,jPose)];
    end
end

%ground truth
relativePosesGroundTruth = [];
for i = 1:camera.nPoses
    iPose = camera.pose(:,i);
    for j = i+1:camera.nPoses
        jPose = camera.pose(:,j);
        relativePosesGroundTruth = [relativePosesGroundTruth SmartMinus(iPose,jPose)];
    end
end

%% all-to-all point error
%estimate
relativePointsEstimate = [];
for i = 1:map.nPoints
    iPoint = graph.vertices(graph.iPointVertices(i)).value;
    for j = i+1:map.nPoints
        jPoint = graph.vertices(graph.iPointVertices(j)).value;
        relativePointsEstimate = [relativePointsEstimate (jPoint - iPoint)];
    end
end

%ground truth
relativePointsGroundTruth = [];
for i = 1:map.nPoints
    iPoint = map.points(i).position(:,1);
    for j = i+1:map.nPoints
        jPoint = map.points(j).position(:,1);
        relativePointsGroundTruth = [relativePointsGroundTruth (jPoint - iPoint)];
    end
end

%% norm
poseTError = norm(poseTError);
poseRError = norm(poseRError);
pointError = norm(pointError);
planeFitError = norm(planeFitError);
planeNError = norm(planeNError);
planeDError = norm(planeDError);
allPoseError = norm(relativePosesGroundTruth) - norm(relativePosesEstimate);
allPointError = norm(relativePointsGroundTruth) - norm(relativePointsEstimate);

end

