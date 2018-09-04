%% config
config = CameraConfig();
config = setAppConfig(config);
measurementsCell = graphFileToCell(config,'sphere400DO-SLAM.graph');
groundTruthCell  = graphFileToCell(config,'sphere400_groundtruthDO-SLAM.graph');

%% solve
graph0 = Graph();
solver = graph0.process(config,measurementsCell,groundTruthCell);
solverEnd = solver(end);
graph0  = solverEnd.graphs(1);
graphN  = solverEnd.graphs(end);

% to put first poses at {0}
poses = graphN.vertices(graphN.identifyVertices('pose'));
firstPose = poseToTransformationMatrix(poses(1).value);
for i=1:length(poses)
    poseValue = firstPose \ poseToTransformationMatrix(graphN.vertices(i).value);
    graphN.vertices(i).value = transformationMatrixToPose(poseValue);
end
graphN.saveGraphFile(config,'sphere400_results.graph');
graphGT = Graph(config,groundTruthCell);

% % to put GT first poses at {0}
% posesGT = graphGT.vertices(graphGT.identifyVertices('pose'));
% firstPoseGT = poseToTransformationMatrix(posesGT(1).value);
% for i=1:length(posesGT)
%     poseValueGT = firstPoseGT \ poseToTransformationMatrix(graphGT.vertices(i).value);
%     graphGT.vertices(i).value = transformationMatrixToPose(poseValueGT);
% end
% graphGT.saveGraphFile(config,'app7_groundTruthStaticOnlyPosesOnly.graph');


%% plot
figure('units','normalized','color','w');
axis equal
view(3)
resultsCell = graphFileToCell(config,'sphere400_results.graph');
results = errorAnalysis(config,graphGT,graphN);
plotGraphFileICRA(config,resultsCell,'solverResults',zeros(6,1),[])
title('do-slam solution')

% figure('units','normalized','color','w');
% axis equal
% view(3)
% groundTruthCell = graphFileToCell(config,'sphere2500_groundtruthDO-SLAM.graph');
% plotGraphFileICRA(config,groundTruthCell,'groundTruth');
% title('ground-truth')