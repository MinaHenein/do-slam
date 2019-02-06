%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 19/12/2018
% Contributors:
%--------------------------------------------------------------------------
% Test vKITTI static and dynamic reprojectioin error minimization
%--------------------------------------------------------------------------
% vKitti main
config = CameraConfig();
K = [725,   0,     620.5;
    0,    725,     187.0;
    0,      0,        1];
config.set('focalLength',725);
config.set('opticalCentreX',620.5);
config.set('opticalCentreY',187.0);
config.set('R',[0,0,1,0; -1,0,0,0; 0,-1,0,0;0,0,0,1]);
config = setAppConfig(config);
config.set('motionModel','constantSE3MotionDA');
config.set('std2PointsSE3Motion', [1,1,1]');
config.set('SE3MotionVertexInitialization','eye');
config.set('newMotionVertexPerNLandmarks',inf);
config.set('landmarksSlidingWindowSize',inf);
config.set('objectPosesSlidingWindow',false);
config.set('objectPosesSlidingWindowSize',inf);
config.set('newMotionVertexPerNObjectPoses',inf);
config.absoluteToRelativePointHandle = @AbsoluteToRelativePositionR3xso3Image;
config.set('landmarkErrorToMinimize' ,'reprojectionKnownIntrinsics');
% config.set('noiseModel','Off');
config.set('stdPosePixel',[0.02;0.02]);


%% 5. Generate Measurements & Save to Graph File, load graph file as well
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('pointsDataAssociationLabel','2PointsDataAssociation');
config.set('measurementsFileName','finalNoiseSequence0001_short_reprojection_Meas.graph')
config.set('groundTruthFileName','finalNoiseSequence0001_short_reprojection_GT.graph')

writeGraphFileImagePlane(config)
config.set('posePointEdgeLabel','EDGE_2D_PIXEL');
config.set('pointInitialisation','3DMeasurement');
config.set('measurementsFileName',strcat(config.measurementsFileName(1:end-6),'_Test.graph'))
config.set('groundTruthFileName',strcat(config.groundTruthFileName(1:end-6),'_Test.graph'))
groundTruthCellReprojection = graphFileToCell(config,config.groundTruthFileName);
measurementsCellReprojection = graphFileToCell(config,config.measurementsFileName);
%% 6. Solve
%% 6.1 Reprojection error minimization
timeStartReprojection = tic;
reprojectionGraph0 = Graph();
reprojectionSolver = reprojectionGraph0.process(config,measurementsCellReprojection,groundTruthCellReprojection);
reprojectionSolverEnd = reprojectionSolver(end);
totalTimeReprojection = toc(timeStartReprojection);
%get desired graphs & systems
reprojectionGraph0  = reprojectionSolverEnd.graphs(1);
reprojectionGraphN  = reprojectionSolverEnd.graphs(end);
%save results to graph file
reprojectionGraphN.saveGraphFile(config,'finalNoiseSequence0001_short_reprojection_results.graph');

%% 6.2 3D error minimization
config.set('landmarkErrorToMinimize' ,'3D');
config.set('posePointEdgeLabel','EDGE_3D');
config.absoluteToRelativePointHandle = @AbsoluteToRelativePositionR3xso3;
config.set('measurementsFileName','finalNoiseSequence0001_short_reprojection_Meas.graph')
config.set('groundTruthFileName','finalNoiseSequence0001_short_reprojection_GT.graph')
groundTruthCell = graphFileToCell(config,config.groundTruthFileName);
measurementsCell = graphFileToCell(config,config.measurementsFileName);
timeStart = tic;
Graph0 = Graph();
Solver = Graph0.process(config,measurementsCell,groundTruthCell);
SolverEnd = Solver(end);
totalTime = toc(timeStart);
%get desired graphs & systems
Graph0  = SolverEnd.graphs(1);
GraphN  = SolverEnd.graphs(end);
%save results to graph file
GraphN.saveGraphFile(config,'finalNoiseSequence0001_short_3D_results.graph');

%% 7. Error analysis
fprintf('\nTotal time solving (reprojection error minimization): %f\n',totalTimeReprojection)
fprintf('\nTotal time solving (3D error minimization): %f\n',totalTime)

% load ground truth into graph, sort if required
graphGTReprojection = Graph(config,groundTruthCellReprojection);
fprintf('\nReprojection error minimization results:\n')
resultsReprojection = errorAnalysis(config,graphGTReprojection,reprojectionGraphN);
fprintf('\nReprojection error:\n')
reprojectionError = calculate_reprojection_error(config,graphGTReprojection,reprojectionGraphN);

graphGT = Graph(config,groundTruthCell);
fprintf('\n3D error minimization results:\n')
results3D = errorAnalysis(config,graphGT,GraphN);

%% 8. GT Plot 
figure('units','normalized','color','w');
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
grid on
axis equal
view([-50,25])
plotGraphFileICRA(config,groundTruthCell,'groundTruth');