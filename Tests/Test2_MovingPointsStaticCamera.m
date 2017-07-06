%% general setup
% run startup first
clear all
close all

nSteps = 2;

%% config setup 
config = CameraConfig();
config = setUnitTestConfig(config);
config.set('groundTruthFileName' ,'groundTruthTest2.graph');
config.set('measurementsFileName','measurementsTest2.graph');

rng(config.rngSeed);
%% set up sensor - MANUAL
sensorPose = zeros(6,nSteps);

%% set up object
objPtsRelative = {[0 0 0]',[1 -1 1]',[1 1 1]'};
objPts = objPtsRelative;

% pose is set in R3xSO3
objPose = zeros(6,nSteps); % initialises 3 poses
objPose(1,:) = linspace(5,7,nSteps);
objPose(2,:) = linspace(0,2,nSteps); % moves in y direction
objPose(6,:) = linspace(0,pi/4,nSteps); % slight rotation about z axis to expose different points

figure()
hold on;
% axis equal;
for j=1:size(objPts,2)
    for i=1:nSteps
        % apply object pose on the points, each column is a time step
        objPts{j}(:,i) = RelativeToAbsolutePositionR3xso3(objPose(:,i),objPtsRelative{j}(:,1));
    end
    plot3(objPts{j}(1,:),objPts{j}(2,:),objPts{j}(3,:),'k');
end


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
    groundTruthVertices{i,1} = currentVertex;
    vertexCount = vertexCount+1;
end

for i=1:nSteps
    for j=1:size(objPts,2)
        % create vertex for point location
        currentVertex = struct();
        currentVertex.label = config.pointVertexLabel;
        currentVertex.index = vertexCount;
        currentVertex.value = objPts{j}(:,i);
        groundTruthVertices{i,j+1} = currentVertex;
        vertexCount = vertexCount+1;
    end
end

for i=1:size(groundTruthVertices,1)
    % ground Truth edges for odometry
    if i > 1
        currentEdge = struct();
        currentEdge.index1 = groundTruthVertices{i-1,1}.index;
        currentEdge.index2 = groundTruthVertices{i,1}.index;
        currentEdge.label = config.posePoseEdgeLabel;
        currentEdge.value = AbsoluteToRelativePoseR3xso3(sensorPose(:,i-1),sensorPose(:,i));
        currentEdge.std = config.stdPosePose;
        currentEdge.cov = config.covPosePose;
        currentEdge.covUT = covToUpperTriVec(currentEdge.cov);
        groundTruthEdges{i,1} = currentEdge;
    end
    for j=1:size(objPts,2)
        currentEdge = struct();
        currentEdge.index1 = groundTruthVertices{i,1}.index;
        currentEdge.index2 = groundTruthVertices{i,j+1}.index;
        currentEdge.label = config.posePointEdgeLabel;
        currentEdge.value = AbsoluteToRelativePositionR3xso3(sensorPose(:,i),objPts{j}(:,i));
        currentEdge.std = config.stdPosePoint;
        currentEdge.cov = config.covPosePoint;
        currentEdge.covUT = covToUpperTriVec(currentEdge.cov);
        groundTruthEdges{i,j+1} = currentEdge;
    end
end

for i=2:nSteps
    for j=1:size(objPts,2)
        currentEdge = struct();
        currentEdge.index1 = groundTruthVertices{i-1,j+1}.index;
        currentEdge.index2 = groundTruthVertices{i,j+1}.index;
        currentEdge.label = config.pointPointEdgeLabel;
        currentEdge.value = groundTruthVertices{i-1,j+1}.value-groundTruthVertices{i,j+1}.value;
        currentEdge.std = config.stdPointPoint;
        currentEdge.cov = config.covPointPoint;
        currentEdge.covUT = covToUpperTriVec(currentEdge.cov);
        groundTruthEdges{i,j+4} = currentEdge; % add to end
    end
end

measurementEdges = groundTruthEdges; % copies grouthTruth to add noise
for i=1:numel(measurementEdges) % add noise on measurements
    if ~isempty(measurementEdges{i})
        noise = normrnd(measurementEdges{i}.value,measurementEdges{i}.std);
        measurementEdges{i}.value = noise;
    end
end
    
groundTruthGraph = fopen(strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,config.groundTruthFileName),'w');
measurementGraph = fopen(strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,config.measurementsFileName),'w');

groundTruthVertices{size(groundTruthEdges,1),size(groundTruthEdges,2)} = []; % only done to avoid index error
[nRows, nColumns] = size(groundTruthEdges);
for i=1:nRows
    for j=1:nColumns
    if ~isempty(groundTruthVertices{i,j})
        vertex = groundTruthVertices{i,j};
        formatSpec = strcat('%s %d ',repmat(' %6.6f',1,numel(vertex.value)),'\n');
        fprintf(groundTruthGraph, formatSpec, vertex.label, vertex.index, vertex.value);
    end
    if ~isempty(groundTruthEdges{i,j})
        % print groundTruth Edge
        edge = groundTruthEdges{i,j};
        formatSpec = strcat('%s %d %d',repmat(' %.6f',1,numel(edge.value)),repmat(' %.6f',1,numel(edge.covUT)),'\n');
        fprintf(groundTruthGraph, formatSpec, edge.label, edge.index1, edge.index2, edge.value, edge.covUT);
        
        % print Measurement edge
        edge = measurementEdges{i,j};
        fprintf(measurementGraph, formatSpec, edge.label, edge.index1, edge.index2, edge.value, edge.covUT);
    end
    end
end

fclose(groundTruthGraph);
fclose(measurementGraph);

%% solver
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);
measurementsCell = graphFileToCell(config,config.measurementsFileName);
graph0 = Graph();
solver = graph0.process(config,measurementsCell,groundTruthCell);
solverEnd = solver(end);
% 
graphN  = solverEnd.graphs(end);
graphN.saveGraphFile(config,'resultsTest2.graph');
% 
graphGT = Graph(config,groundTruthCell);
results = errorAnalysis(config,graphGT,graphN)
% 
%% plot graph files
% h = figure; 
axis equal;
xlabel('x')
ylabel('y')
zlabel('z')
hold on
plotGraphFile(config,groundTruthCell,'b');
resultsCell = graphFileToCell(config,'resultsTest2.graph');
plotGraphFile(config,resultsCell,'r')
