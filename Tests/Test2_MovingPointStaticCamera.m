%% general setup
% run startup first
clear all
close all

nSteps = 3;

%% config setup 
config = CameraConfig();
config = setUnitTestConfig(config);

rng(config.rngSeed);
%% set up sensor - MANUAL
sensorPose = zeros(6,nSteps);
%constant linear velocity in x-axis direction, constant angular velocity about x-axis
sensorPose(1,:) = -5+linspace(0,2,nSteps);
sensorPose(2,:) = linspace(0,0,nSteps);
sensorPose(3,:) = linspace(0,0,nSteps);
sensorPose(4,:) = linspace(0,0,nSteps);
sensorPose(5,:) = linspace(pi/2,pi/2,nSteps);
sensorPose(6,:) = linspace(pi/6,-pi/6,nSteps);

%% set up object
objPts = {[0 0 0]',[1 -1 1]',[1 1 1]'};
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
    currentVertex.label = config.poseVertexLabel;
    currentVertex.value = sensorPose(:,i);
    currentVertex.index = vertexCount;
    groundTruthVertices{end+1} = currentVertex;
    vertexCount = vertexCount+1;
end

for j=1:size(objPts,2)
    % create vertex for point location
    currentVertex = struct();
    currentVertex.label = config.pointVertexLabel;
    currentVertex.index = vertexCount;
    currentVertex.value = objPts(:,j);
    groundTruthVertices{end+1} = currentVertex;
    vertexCount = vertexCount+1;
end

for i=2:size(sensorPose,2)
    % ground Truth edges for odometry
    currentEdge = struct();
    currentEdge.index1 = i-1;
    currentEdge.index2 = i;
    currentEdge.label = config.posePoseEdgeLabel;
    currentEdge.value = AbsoluteToRelativePoseR3xso3(sensorPose(:,i-1),sensorPose(:,i));
    currentEdge.std = config.stdPosePose;
    currentEdge.cov = config.covPosePose;
    currentEdge.covUT = covToUpperTriVec(currentEdge.cov);
    groundTruthEdges{end+1} = currentEdge;
end

for i=1:size(sensorPose,2)
    for j=1:size(objPts,2)
        currentEdge = struct();
        currentEdge.index1 = i;
        currentEdge.index2 = j+3;
        currentEdge.label = config.posePointEdgeLabel;
        currentEdge.value = AbsoluteToRelativePositionR3xso3(sensorPose(:,i),objPts(:,j));
        currentEdge.std = config.stdPosePoint;
        currentEdge.cov = config.covPosePoint;
        currentEdge.covUT = covToUpperTriVec(currentEdge.cov);
        groundTruthEdges{end+1} = currentEdge;
    end
end

measurementEdges = groundTruthEdges; % copies grouthTruth to add noise
for i=1:size(measurementEdges,2) % add noise on measurements
    noise = normrnd(measurementEdges{i}.value,measurementEdges{i}.std);
    measurementEdges{i}.value = noise;
end
    
groundTruthGraph = fopen(strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,config.groundTruthFileName),'w');

for i=1:size(groundTruthVertices,2)
    vertex = groundTruthVertices{i};
    formatSpec = strcat('%s %d ',repmat(' %6.6f',1,numel(vertex.value)),'\n');
    fprintf(groundTruthGraph, formatSpec, vertex.label, vertex.index, vertex.value);
end

for i=1:size(groundTruthEdges,2)
    edge = groundTruthEdges{i};
    formatSpec = strcat('%s %d %d',repmat(' %.6f',1,numel(edge.value)),repmat(' %.6f',1,numel(edge.covUT)),'\n');
    fprintf(groundTruthGraph, formatSpec, edge.label, edge.index1, edge.index2, edge.value, edge.covUT);
end

fclose(groundTruthGraph);
measurementGraph = fopen(strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,config.measurementsFileName),'w');

for i=1:size(measurementEdges,2)
    edge = measurementEdges{i};
    formatSpec = strcat('%s %d %d',repmat(' %.6f',1,numel(edge.value)),repmat(' %.6f',1,numel(edge.covUT)),'\n');
    fprintf(measurementGraph, formatSpec, edge.label, edge.index1, edge.index2, edge.value, edge.covUT);
end

fclose(measurementGraph);

%% solver
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);
measurementsCell = graphFileToCell(config,config.measurementsFileName);
graph0 = Graph();
solver = graph0.process(config,measurementsCell,groundTruthCell);
solverEnd = solver(end);

graphN  = solverEnd.graphs(end);
graphN.saveGraphFile(config,'resultsTest1.graph');

graphGT = Graph(config,groundTruthCell);
results = errorAnalysis(config,graphGT,graphN);

%% plot graph files
h = figure; 
axis equal;
axis([-6 1 -5 2 -1 5])
xlabel('x')
ylabel('y')
zlabel('z')
hold on
plotGraphFile(config,groundTruthCell,[1 0 0]);
resultsCell = graphFileToCell(config,'resultsTest1.graph');
plotGraphFile(config,resultsCell,[0 0 1])
