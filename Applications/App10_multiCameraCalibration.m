%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 08/02/18
% Contributors:
%--------------------------------------------------------------------------
% multipleCameraCalibration_main

%% config setup
config = CameraConfig();
config.set('motionModel','constantSE3MotionDA');
config.set('std2PointsSE3Motion', [0.05,0.05,0.05]');
config.set('SE3MotionVertexInitialization','eye');
config.set('newMotionVertexPerNLandmarks',inf)
config = setUnitTestConfig(config);
rng(config.rngSeed);

%% solver
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('measurementsFileName','app10_measurementsRotTr.graph');
config.set('groundTruthFileName','app10_groundTruth.graph');
writeDataAssociationVerticesEdges_constantSE3MotionNoGT(config);
measurementsCell = graphFileToCell(config,config.measurementsFileName);
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);

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
graphN.saveGraphFile(config,'app10_results.graph');