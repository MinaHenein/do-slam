%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 06/07/2017
% Contributors:
%--------------------------------------------------------------------------

%% general setup
% run startup first
clear all
close all

nSteps = 3;

%% config setup 
config = CameraConfig();
config = setUnitTestConfig(config);
config.set('groundTruthFileName' ,'groundTruthTest1.graph');
config.set('measurementsFileName','measurementsTest1.graph');

rng(config.rngSeed);
%% set up sensor - MANUAL
sensorPose = zeros(6,nSteps);

% applies relative motion - constant velocity in forward (x) axis and rotation about z axis
for i=2:nSteps
    rotationMatrix = eul2rot([pi/12 0 0]);
    relativeSensorPose = [1; 0; 0; arot(rotationMatrix)];
    sensorPose(:,i) = RelativeToAbsolutePoseR3xso3(sensorPose(:,i-1),relativeSensorPose);
end

%% set up object
objectPts = {[0 0 0]',[1 -1 1]',[1 1 1]'};
objPose = repmat([5, 0, 0, 0, 0, 0]',1,3);
objectPts = cell2mat(objectPts);
objectPts = RelativeToAbsolutePositionR3xso3(objPose,objectPts);

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

for j=1:size(objectPts,2)
    % create vertex for point location
    currentVertex = struct();
    currentVertex.label = config.pointVertexLabel;
    currentVertex.index = vertexCount;
    currentVertex.value = objectPts(:,j);
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
    for j=1:size(objectPts,2)
        currentEdge = struct();
        currentEdge.index1 = i;
        currentEdge.index2 = j+3;
        currentEdge.label = config.posePointEdgeLabel;
        currentEdge.value = AbsoluteToRelativePositionR3xso3(sensorPose(:,i),objectPts(:,j));
        currentEdge.std = config.stdPosePoint;
        currentEdge.cov = config.covPosePoint;
        currentEdge.covUT = covToUpperTriVec(currentEdge.cov);
        groundTruthEdges{end+1} = currentEdge;
    end
end

measurementEdges = groundTruthEdges; % copies grouthTruth to add noise
for i=1:size(measurementEdges,2) % add noise on measurements
    if strcmp(config.noiseModel,'Gaussian')
        noise = normrnd(measurementEdges{i}.value,measurementEdges{i}.std);
    elseif strcmp(config.noiseModel,'Off')
        noise = measurementEdges{i}.value;
    end
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

%% check vertices
checkVertices = {};
checkVertices{1} = groundTruthVertices{1};
for i=2:nSteps
    checkVertex = struct();
    checkVertex.value = RelativeToAbsolutePoseR3xso3(checkVertices{i-1}.value,groundTruthEdges{i-1}.value);
    checkVertices{i} = checkVertex;
    if norm(checkVertices{i}.value - groundTruthVertices{i}.value) > 1e-15
        error('Reconstructed vertices and original vertices do not match.')
    end
end

for i=1:nSteps
    for j=1:size(objectPts,2)
        checkVertex = struct();
        checkVertex.value = RelativeToAbsolutePositionR3xso3(checkVertices{i}.value,groundTruthEdges{3*i+j-1}.value);
        if norm(groundTruthVertices{nSteps+j}.value - checkVertex.value) > 1e-15
            error('Reconstructed vertices and original vertices do not match.')
        end
    end
end

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
fprintf('Chi Squared Error: %.4d \n',solverEnd.systems.chiSquaredError)
fprintf('Absolute Trajectory Translation Error: %.4d \n',results.ATE_translation_error)
fprintf('Absolute Trajectory Rotation Error: %.4d \n',results.ATE_rotation_error)
fprintf('Absolute Structure Points Error: %d \n',results.ASE_translation_error);
fprintf('All to All Relative Pose Squared Translation Error: %.4d \n',results.AARPE_squared_translation_error)
fprintf('All to All Relative Pose Squared Rotation Error: %.4d \n',results.AARPE_squared_rotation_error)
fprintf('All to All Relative Point Squared Translation Error: %.4d \n',results.AARPTE_squared_translation_error)

%% plot graph files
h = figure; 
axis equal;
xlabel('x')
ylabel('y')
zlabel('z')
hold on
% resultsCell = graphFileToCell(config,'resultsTest1.graph');
% plotGraph(config,resultsCell,[0 0 1])
plotGraph(config,graphN,'red');
plotGraphFile(config,groundTruthCell,'blue');

figure
subplot(1,2,1)
spy(solverEnd.systems(end).A)
subplot(1,2,2)
spy(solverEnd.systems(end).H)