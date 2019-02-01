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


%% 5. Generate Measurements & Save to Graph File, load graph file as well
config.set('pointMotionMeasurement','Off')
config.set('measurementsFileName','Sequence0001_350to370_pixelValues_dynamic_weight_Meas.graph')
config.set('groundTruthFileName','Sequence0001_350to370_pixelValues_dynamic_weight_GT.graph')

writeGraphFileImagePlane(config)
config.set('posePointEdgeLabel','EDGE_2D_PIXEL');
config.set('pointInitialisation','3DMeasurement');
config.set('measurementsFileName',strcat(config.measurementsFileName(1:end-6),'_Test.graph'))
groundTruthCell = graphFileToCell(config,config.groundTruthFileName);
measurementsCell = graphFileToCell(config,config.measurementsFileName);
 
%% 6. Solve
%% 6.1 Without SE3
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
graphN.saveGraphFile(config,'Sequence0001_350to370_pixelValues_dynamic_weight_results.graph');

%% 7. Error analysis
% load ground truth into graph, sort if required
graphGT = Graph(config,groundTruthCell);
fprintf('\nInitial results for without SE(3) Transform:\n')
results = errorAnalysis(config,graphGT,graphN);

%% 8. Plot
figure('units','normalized','color','w');
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
hold on
grid on
axis equal
% axisLimits = [-30,50,-10,60,-25,25];
% axis(axisLimits)
view([-50,25])
%plot groundtruth
plotGraphFileICRA(config,groundTruthCell,'groundTruth');
%plot results
resultsCell = graphFileToCell(config,'Sequence0001_350to370_pixelValues_dynamic_weight_results.graph');
plotGraphFileICRA(config,resultsNoSE3Cell,'initial',...
    results.relPose.get('R3xso3Pose'),results.posePointsN.get('R3xso3Pose'))