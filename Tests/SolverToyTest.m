%% general setup
% run startup first
clear all
close all

nSteps = 3;

%% set up sensor
sensorPose = zeros(6,nSteps);
%constant linear velocity in x-axis direction, constant angular velocity about x-axis
sensorPose(1,:) = -5+linspace(0,2,nSteps);
sensorPose(2,:) = linspace(0,1,nSteps);
sensorPose(3,:) = linspace(0,0,nSteps);
sensorPose(4,:) = linspace(0,0,nSteps);
sensorPose(5,:) = linspace(0,0,nSteps);
sensorPose(6,:) = linspace(0,pi/6,nSteps);

%adjust based on parameterisation
for i = 1:nSteps
    sensorPose(:,i) = R3xso3_LogSE3(sensorPose(:,i));
end

%% set up object
objPts = {[1 0 -1/sqrt(2)]',[-1 0 -1/sqrt(2)]',[0 1 1/sqrt(2)]',[0 -1 1/sqrt(2)]'};
objPts = cell2mat(objPts);

% pose is set in R3xSO3
% objPose = zeros(3,nSteps); % initialises 3 poses
% objPose(1,:) = linspace(0,5,nSteps);
% objPose(2,:) = linspace(0,10,nSteps); % moves in y direction
% objPose(3,:) = linspace(0,1,nSteps); % above ground
% objPose(6,:) = linspace(0,pi/2,nSteps); % slight rotation about z axis to expose different points
% 
% for i = 1:nSteps % convert to SE(3)
%     objPose(:,i) = R3xso3_LogSE3(objPose(:,i));
% end
% 
% for j=1:size(objPts,2)
%     for i=1:nSteps
%         % apply object pose on the points, each column is a time step
%         objPts{j}(:,i) = RelativePoint2AbsolutePoint3D(objPose(:,i),objPts{j}(:,1));
%     end
% end

%% create ground truth and measurements
groundTruthVertices = {};
groundTruthEdges = {};
vertexCount = 1;

for i=1:nSteps
    % create vertex for odometry reading
    currentVertex = struct();
    currentVertex.label = 'VERTEX_POSE_LOG_SE3';
    grountTruthVertex.time = i;
    currentVertex.value = sensorPose(:,i);
    currentVertex.index = vertexCount;
    groundTruthVertices{end+1} = currentVertex;
    vertexCount = vertexCount+1;
end

for j=1:size(objPts,2)
    % create vertex for point location
    currentVertex = struct();
    currentVertex.label = 'VERTEX_POINT_3D';
    currentVertex.index = vertexCount;
    currentVertex.value = objPts(:,j);
    groundTruthVertices{end+1} = currentVertex;
    vertexCount = vertexCount+1;
end

for i=2:size(sensorPose,2)
    currentEdge = struct();
    currentEdge.index1 = i-1;
    currentEdge.index2 = i;
    currentEdge.label = 'EDGE_LOG_SE3';
    currentEdge.value = AbsoluteToRelativePoseSE3(sensorPose(:,i),sensorPose(:,i-1));
    currentEdge.std = [0.01,0.01,0.01,pi/90,pi/90,pi/90]';
    groundTruthEdges{end+1} = currentEdge;
end

for i=1:size(sensorPose,2)
    for j=1:size(objPts,2)
        currentEdge = struct();
        currentEdge.index1 = i;
        currentEdge.index2 = j+3;
        currentEdge.label = 'EDGE_3D';
        currentEdge.value = RelativePoint2AbsolutePoint3D(sensorPose(:,i),objPts(:,j));
        currentEdge.std = [0.02,0.02,0.02]';
        groundTruthEdges{end+1} = currentEdge;
    end
end

measurementEdges = groundTruthEdges; % copies grouthTruth to add noise
rng(2); % fixes random seed
for i=1:size(measurementEdges,2) % add noise on measurements
    dim = size(measurementEdges{i}.std,1);
    noise = normrnd(zeros(dim,1),measurementEdges{i}.std);
    measurementEdges{i}.value = measurementEdges{i}.value+noise;
end
    
groundTruthGraph = fopen('groundTruth.graph','w');

for i=1:size(groundTruthVertices,2)
    vertex = groundTruthVertices{i};
    formatSpec = strcat('%s %d ',repmat(' %6.6f',1,numel(vertex.value)),'\n');
    fprintf(groundTruthGraph, formatSpec, vertex.label, vertex.index, vertex.value);
end

for i=1:size(groundTruthEdges,2)
    edge = groundTruthEdges{i};
    formatSpec = strcat('%s %d %d',repmat(' %.6f',1,numel(edge.value)),repmat(' %.6f',1,numel(edge.std)),'\n');
    fprintf(groundTruthGraph, formatSpec, edge.label, edge.index1, edge.index2, edge.value, edge.std);
end

fclose(groundTruthGraph);
measurementGraph = fopen('measurementGraph.graph','w');

for i=1:size(measurementEdges,2)
    edge = measurementEdges{i};
    formatSpec = strcat('%s %d %d',repmat(' %.6f',1,numel(edge.value)),repmat(' %.6f',1,numel(edge.std)),'\n');
    fprintf(measurementGraph, formatSpec, edge.label, edge.index1, edge.index2, edge.value, edge.std);
end

fclose(measurementGraph);

    
