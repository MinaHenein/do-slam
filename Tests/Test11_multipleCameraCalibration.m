%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 09/01/18
% Contributors:
%--------------------------------------------------------------------------
% Test11_multipleCameraCalibration

%% general setup
% run startup first
% clear all
close all

nSteps = 2;
nCameras = 3;

%% config setup
config = CameraConfig();
config.set('groundTruthFileName' ,'groundTruthTest11.graph');
config.set('measurementsFileName','measurementsTest11.graph');
config.set('motionModel','constantSE3MotionDA');
config.set('std2PointsSE3Motion', [0.1,0.1,0.1]');
config = setUnitTestConfig(config);
rng(config.rngSeed);

%% set up cameras
camera1Pose1 = zeros(6,1);
camera2Pose1 = [3.53;1.46;1;0.39;0.19;0.26];
camera3Pose1 = [5;5;2;0.78;0.39;0.52];
camera1Pose2 = camera1Pose1;
camera2Pose2 = camera2Pose1;
camera3Pose2 = camera3Pose1;

camerasPoses = [camera1Pose1,camera2Pose1,camera3Pose1;
    camera1Pose2,camera2Pose2,camera3Pose2];

%% set up planar pattern
normal = [0.8,0.2,0.82]';
distance = 8;
planeParameters1 = [normal; distance];
planeParameters1 = planeParameters1/norm(planeParameters1(1:3));
% Points
nPlane1Points = 5;
dependentVar = (normal==max(normal));
vars = {'x','y','z'};
dependentVar = vars{dependentVar};
dependentVar = dependentVar(1);
pointPositions1 = zeros(3,nPlane1Points);
a = planeParameters1(1); b = planeParameters1(2); c = planeParameters1(3); d = planeParameters1(4);
switch dependentVar
    case 'x'
        pointPositions1(2:3,:) = [10*rand(2,nPlane1Points)];
        pointPositions1(1,:)   = (d - b*pointPositions1(2,:) - c*pointPositions1(3,:))/a;
    case 'y'
        pointPositions1([1 3],:) = [10*rand(2,nPlane1Points)];
        pointPositions1(2,:)   = (d - a*pointPositions1(1,:) - c*pointPositions1(3,:))/b;
    case 'z'
        pointPositions1(1:2,:) = [10*rand(2,nPlane1Points)];
        pointPositions1(3,:)   = (d - a*pointPositions1(1,:) - b*pointPositions1(2,:))/c;
end

planeSE3motion = [3,0,0,0.6,0,0]';
transformationMatrix = [rot(planeSE3motion(4:6)) planeSE3motion(1:3); 0 0 0 1];
% plane points after an SE3 transform
pointPositions2 = transformationMatrix*[pointPositions1;ones(1,nPlane1Points)];
pointPositions  = [pointPositions1;pointPositions2(1:3,:)];

% entity parameters
[n,~,p] = affine_fit(pointPositions2(1:3,:)');
planeParameters2 = [n',p*n]';
entityParameters = {planeParameters1,planeParameters2}';


%% create ground truth and measurements
groundTruthVertices = {};
groundTruthEdges = {};
vertexCount = 1;

for i=1:nSteps
    rowCount = 0;
    for j = 1:nCameras
        % create vertex for cameras poses
        currentVertex = struct();
        currentVertex.label = config.poseVertexLabel;
        currentVertex.value = camerasPoses(mapping(i,6),j);
        currentVertex.index = vertexCount;
        groundTruthVertices{i,j} = currentVertex;
        vertexCount = vertexCount+1;
        rowCount = rowCount+1;
    end
    for k=1:size(pointPositions,2)
        % create vertex for point positions
        currentVertex = struct();
        currentVertex.label = config.pointVertexLabel;
        currentVertex.index = vertexCount;
        currentVertex.value = pointPositions(mapping(i,3),k);
        groundTruthVertices{i,rowCount+1} = currentVertex;
        vertexCount = vertexCount+1;
        rowCount = rowCount+1;
    end
    % create vertex for planes parameters
    currentVertex = struct();
    currentVertex.label = config.planeVertexLabel;
    currentVertex.index = vertexCount;
    currentVertex.value = entityParameters{i,1};
    groundTruthVertices{i,rowCount+1} = currentVertex;
    vertexCount = vertexCount+1;
    rowCount = rowCount+1;
end

for i=1:size(groundTruthVertices,1)
    % ground Truth edges for odometry
    if i > 1
        for j= 1:nCameras
            currentEdge = struct();
            currentEdge.index1 = groundTruthVertices{i-1,j}.index;
            currentEdge.index2 = groundTruthVertices{i,j}.index;
            currentEdge.label = config.posePoseEdgeLabel;
            currentEdge.value = AbsoluteToRelativePoseR3xso3(camerasPoses(mapping(i-1,6),j),...
                camerasPoses(mapping(i,6),j));
            currentEdge.std = config.stdPosePose;
            currentEdge.cov = config.covPosePose;
            currentEdge.covUT = covToUpperTriVec(currentEdge.cov);
            groundTruthEdges{i,end+1} = currentEdge;
        end
    end
    % ground Truth edges for intercamera poses
    for j= 2:nCameras
        currentEdge = struct();
        currentEdge.index1 = groundTruthVertices{i,j-1}.index;
        currentEdge.index2 = groundTruthVertices{i,j}.index;
        currentEdge.label = config.posePoseEdgeLabel;
        currentEdge.value = AbsoluteToRelativePoseR3xso3(camerasPoses(mapping(i,6),j-1),...
            camerasPoses(mapping(i,6),j));
        currentEdge.std = config.stdPosePose;
        currentEdge.cov = config.covPosePose;
        currentEdge.covUT = covToUpperTriVec(currentEdge.cov);
        groundTruthEdges{i,end+1} = currentEdge;
    end
    % ground Truth edges for point observations
    % camera 1 & 2 see pattern at time 1
    % camera 2 & 3 see pattern at time 2
    if i > 1
        k = 2:nCameras;
    else
        k = 1:nCameras-1;
    end
    for o = k
        for j=1:size(pointPositions,2)
            currentEdge = struct();
            currentEdge.index1 = groundTruthVertices{i,o}.index;
            currentEdge.index2 = groundTruthVertices{i,j+nCameras}.index;
            currentEdge.label = config.posePointEdgeLabel;
            currentEdge.value = AbsoluteToRelativePositionR3xso3(camerasPoses(mapping(i,6),o),...
                pointPositions(mapping(i,3),j));
            currentEdge.std = config.stdPosePoint;
            currentEdge.cov = config.covPosePoint;
            currentEdge.covUT = covToUpperTriVec(currentEdge.cov);
            groundTruthEdges{i,end+1} = currentEdge;
        end
    end
    for j=1:size(pointPositions,2)
        % ground Truth edges for point-plane measurements
        currentEdge = struct();
        currentEdge.index1 = groundTruthVertices{i,j+nCameras}.index;
        currentEdge.index2 = groundTruthVertices{i,end}.index;
        currentEdge.label = config.pointPlaneEdgeLabel;
        planeParameters = entityParameters{i,1};
        currentEdge.value = pointPositions(mapping(i,3),j)'*planeParameters(1:3)-planeParameters(4);
        currentEdge.std = config.stdPointPlane;
        currentEdge.cov = config.covPointPlane;
        currentEdge.covUT = covToUpperTriVec(currentEdge.cov);
        groundTruthEdges{i,end+1} = currentEdge;
    end
end
% ground truth edges for data associations
if i == nSteps
    for j=1:size(pointPositions,2)
        for l = 1:i-1
            currentEdge = struct();
            currentEdge.index1 = groundTruthVertices{l,j+nCameras}.index;
            currentEdge.index2 = groundTruthVertices{l+1,j+nCameras}.index;
            currentEdge.label = config.pointDataAssociationLabel;
            groundTruthEdges{i,end+1} = currentEdge;
        end
    end
end


measurementEdges = groundTruthEdges; % copies grouthTruth to add noise
for i=1:numel(measurementEdges) % add noise on measurements
    if ~isempty(measurementEdges{i})
        if isfield(measurementEdges{i},'value')
            valueEdge = measurementEdges{i}.value;
            muEdge =  zeros(size(valueEdge,1),1);
            sigmaEdge = measurementEdges{i}.std;
            if strcmp(measurementEdges{i}.label,'EDGE_R3_SO3') || ...
                    strcmp(measurementEdges{i}.label,'EDGE_LOG_SE3')
                measurementEdges{i}.value = ...
                    addGaussianNoise(config,muEdge,sigmaEdge,valueEdge,'pose');
            else
                measurementEdges{i}.value = ...
                    addGaussianNoise(config,muEdge,sigmaEdge,valueEdge);
            end
        else
        end
    end
end

groundTruthGraph = fopen(strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,config.groundTruthFileName),'w');
measurementGraph = fopen(strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,config.measurementsFileName),'w');

[nRows, nColumns] = size(groundTruthVertices);
for i=1:nRows
    for j=1:nColumns
        if ~isempty(groundTruthVertices{i,j})
            vertex = groundTruthVertices{i,j};
            formatSpec = strcat('%s %d ',repmat(' %6.6f',1,numel(vertex.value)),'\n');
            fprintf(groundTruthGraph, formatSpec, vertex.label, vertex.index,...
                vertex.value);
        end
    end
end

[nRows, nColumns] = size(groundTruthEdges);
for i=1:nRows
    for j=1:nColumns
        if ~isempty(groundTruthEdges{i,j})
            % print groundTruth Edge
            edge = groundTruthEdges{i,j};
            if isfield( edge,'value')
                formatSpec = strcat('%s %d %d',repmat(' %.6f',1,numel(edge.value)),...
                    repmat(' %.6f',1,numel(edge.covUT)),'\n');
                if isfield(edge, 'index3')
                    formatSpec = strcat('%s %d %d %d',repmat(' %.6f',1,numel(edge.value)),...
                        repmat(' %.6f',1,numel(edge.covUT)),'\n');
                    fprintf(groundTruthGraph,formatSpec,edge.label,edge.index1,...
                        edge.index2,edge.index3,edge.value,edge.covUT);
                else
                    fprintf(groundTruthGraph,formatSpec,edge.label,edge.index1,...
                        edge.index2,edge.value,edge.covUT);
                end
            else
                formatSpec = '%s %d %d \n';
                fprintf(groundTruthGraph,formatSpec,edge.label,edge.index1,...
                    edge.index2);
            end
            % print Measurement edge
            edge = measurementEdges{i,j};
            if isfield(edge,'value')
                if isfield(edge, 'index3')
                    fprintf(measurementGraph,formatSpec,edge.label,edge.index1,...
                        edge.index2,edge.index3,edge.value,edge.covUT);
                else
                    fprintf(measurementGraph,formatSpec,edge.label,edge.index1,...
                        edge.index2,edge.value,edge.covUT);
                end
            else
                formatSpec = '%s %d %d \n';
                fprintf(measurementGraph,formatSpec,edge.label,edge.index1,...
                    edge.index2);
            end
        end
    end
end
fclose(groundTruthGraph);
fclose(measurementGraph);

%% solver
writeDataAssociationVerticesEdges(config,planeSE3motion)

groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);
measurementsCell = graphFileToCell(config,config.measurementsFileName);
timeStart = tic;
graph0 = Graph();
solver = graph0.process(config,measurementsCell,groundTruthCell);
solverEnd = solver(end);
totalTime = toc(timeStart);
fprintf('\nTotal time solving: %f\n',totalTime)
%
graphN  = solverEnd.graphs(end);
graphN.saveGraphFile(config,'resultsTest11.graph');
%
graphGT = Graph(config,groundTruthCell);
results = errorAnalysis(config,graphGT,graphN);
results2HIGH = results;
fprintf('Chi Squared Error: %.4d \n',solverEnd.systems.chiSquaredError)
fprintf('Absolute Trajectory Translation Error: %.4d \n',results.ATE_translation_error)
fprintf('Absolute Trajectory Rotation Error: %.4d \n',results.ATE_rotation_error)
fprintf('Absolute Structure Points Error: %d \n',results.ASE_translation_error);
fprintf('All to All Relative Pose Squared Translation Error: %.4d \n',...
    results.AARPE_squared_translation_error)
fprintf('All to All Relative Pose Squared Rotation Error: %.4d \n',...
    results.AARPE_squared_rotation_error)
fprintf('All to All Relative Point Squared Translation Error: %.4d \n',...
    results.AARPTE_squared_translation_error)

%% plot graph files
% h = figure;
axis equal;
xlabel('x')
ylabel('y')
zlabel('z')
hold on
plotGraphFile(config,groundTruthCell,[0 0 1]);
plotGraph(config,graphN,[1 0 0]);

figure
subplot(2,2,1)
spy(solverEnd.systems(end).A)
subplot(2,2,2)
spy(solverEnd.systems(end).H)
subplot(2,2,3)
spy(solverEnd.systems(end).covariance)
subplot(2,2,4)
spy(solverEnd.systems(end).covSqrtInv)