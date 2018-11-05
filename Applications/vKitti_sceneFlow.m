%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 24/10/2018
% Contributors:
%--------------------------------------------------------------------------
% vKitti scene flow
%--------------------------------------------------------------------------
%% 1. Config
config = CameraConfig();
config = setAppConfig(config);
% config.set('noiseModel','Off');
config.set('motionModel','constantVelocity');
config.set('std2PointsVelocity',[1,1,1]');
config.set('pointMotionMeasurement','velocity');
config.set('pointsDataAssociationLabel','2PointsDataAssociation');

%% 2. Load Graph Files
config.set('measurementsFileName','finalNoiseSequence0001_334to426_final_sceneFlow_Meas.graph');
config.set('groundTruthFileName','finalNoiseSequence0001_334to426_final_sceneFlow_GT.graph'); 
measurementsCell = graphFileToCell(config,config.measurementsFileName);
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);

%% 3. Solve
timeStart = tic;
graph0 = Graph();
solver = graph0.process(config,measurementsCell,groundTruthCell);
solverEnd = solver(end);
totalTime = toc(timeStart);
fprintf('\nTotal time solving: %f\n',totalTime)
%get desired graphs & systems
graph0  = solverEnd.graphs(1);
graphN  = solverEnd.graphs(end);
%save results to graph file
graphN.saveGraphFile(config,'finalNoiseSequence0001_final_sceneFlow_results.graph');

%% 4. Error analysis
graphGT = Graph(config,groundTruthCell);
fprintf('\nFinal results for SE(3) Transform:\n')
resultsSE3 = errorAnalysis(config,graphGT,graphN);

%% 5. Plot
figure('units','normalized','color','w');
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
hold on
grid on
axis equal
view([-50,25])
%plot groundtruth
plotGraphFileICRA(config,groundTruthCell,'groundTruth');
%plot results
resultsCell = graphFileToCell(config,'finalNoiseSequence0001_final_sceneFlow_results.graph');
plotGraphFileICRA(config,resultsCell,'solverResults',resultsSE3.relPose.get('R3xso3Pose'),...
    resultsSE3.posePointsN.get('R3xso3Pose'),graphN)