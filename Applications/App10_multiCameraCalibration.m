%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 08/02/18
% Contributors:
%--------------------------------------------------------------------------
% multipleCameraCalibration_main

%% config setup
config = CameraConfig();
config.set('motionModel','constantSE3MotionDA');
config.set('std2PointsSE3Motion', [0.01,0.01,0.01]');
config.set('SE3MotionVertexInitialization','translation');
config.set('newMotionVertexPerNLandmarks',inf)
config = setUnitTestConfig(config);

rng(config.rngSeed);

%% solver
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('measurementsFileName','noOverlapLongerGraph_v10.graph');
config.set('groundTruthFileName','app10_groundTruth.graph');
nObjects = 1;
if nObjects >1
    dataAssociationTest(config,config.measurementsFileName,nObjects)
end
status = graphFileTest(config.measurementsFileName,nObjects,config);
if status == 0
    error('Graph file contains errors; one or more vertices have more than 1 type!')
end
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
graphGT = Graph(config,groundTruthCell);
%% error analysis
posesN = [graphN.vertices(graphN.identifyVertices('pose')).value];
posesGT = [graphGT.vertices(graphGT.identifyVertices('pose')).value];
%convert to R3xSO3
v_rel_pose = AbsoluteToRelativePoseR3xso3(posesGT(:,1),posesN(:,1));
%% 1. Pose Error
%ATE
[ATE_translation_error,ATE_rotation_error,...
    ATE_squared_translation_error,ATE_squared_rotation_error] = ...
    Compute_AbsoluteTrajectoryError(posesN,posesGT,v_rel_pose);
%RPE
n_delta = 1;
[RPE_translation_error,RPE_rotation_error,...
    RPE_squared_translation_error,RPE_squared_rotation_error] = ...
    Compute_RelativePoseError(posesN,posesGT,v_rel_pose,n_delta);
%AARPE
[AARPE_translation_error,AARPE_rotation_error,AARPE_squared_translation_error,...
    AARPE_squared_rotation_error] = ...
    Compute_RelativePoseError_AllToAll(posesN,posesGT,v_rel_pose);
results.ATE_translation_error               = ATE_translation_error;
results.ATE_rotation_error                  = ATE_rotation_error;
results.ATE_squared_translation_error       = ATE_squared_translation_error;
results.ATE_squared_rotation_error          = ATE_squared_rotation_error;
results.RPE_translation_error               = RPE_translation_error;
results.RPE_rotation_error                  = RPE_rotation_error;
results.RPE_squared_translation_error       = RPE_squared_translation_error;
results.RPE_squared_rotation_error          = RPE_squared_rotation_error;
results.AARPE_translation_error             = AARPE_translation_error;
results.AARPE_rotation_error                = AARPE_rotation_error;
results.AARPE_squared_translation_error     = AARPE_squared_translation_error;
results.AARPE_squared_rotation_error        = AARPE_squared_rotation_error;
fprintf('Absolute Trajectory Translation Error: %.3f \n',results.ATE_translation_error)
fprintf('Absolute Trajectory Rotation Error: %.3f \n',results.ATE_rotation_error)
fprintf('All to All Relative Pose Translation Error: %.3f \n',results.AARPE_translation_error)
fprintf('All to All Relative Pose Rotation Error: %.3f \n',results.AARPE_rotation_error)

plotRealCameraCalibration(config,'app10_results.graph');
