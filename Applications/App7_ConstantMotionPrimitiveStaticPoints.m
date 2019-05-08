%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 03/07/2017
% Contributors:
%--------------------------------------------------------------------------
% frontEndSolverExample
clear all 
% close all 

%% 1. Config
% time
nSteps = 121;
t0 = 0;
tN = 120;
dt = (tN-t0)/(nSteps-1);
t  = linspace(t0,tN,nSteps);

config = CameraConfig();
config = setAppConfig(config); % copy same settings for error Analysis
config.set('t',t);
% config.set('staticDataAssociation','Off');
config.set('staticDataAssociation','On');
% set motion model in setAppConfig function
% config.set('groundTruthFileName','app7_groundTruth.graph');
% config.set('measurementsFileName','app7_measurements.graph');

% SE3 Motion
% config.set('pointMotionMeasurement','point2DataAssociation');
config.set('motionModel','constantSE3MotionDA');
config.set('std2PointsSE3Motion', [0.05,0.05,0.05]');
config.set('SE3MotionVertexInitialization','eye');

%% 2. Generate Environment
if config.rngSeed
    rng(config.rngSeed); 
end

primitiveInitialPose_R3xso3 = [10 3 3 0 0 0.2]';
primitiveMotion_R3xso3 = [0.8*dt; 0; 0; arot(eul2rot([0.06*dt,0,0.001*dt]))];

% construct trajectories
primitiveTrajectory = ConstantMotionDiscretePoseTrajectory(t,primitiveInitialPose_R3xso3,primitiveMotion_R3xso3,'R3xso3');
constantSE3ObjectMotion = primitiveTrajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));

sampleTimes = t(1:floor(numel(t)/5):numel(t));
sampleWaypoints = primitiveTrajectory.get('R3xso3Pose',sampleTimes);
robotWaypoints = [linspace(0,tN+5,7); 0 sampleWaypoints(1,:); 0 (sampleWaypoints(2,:)+0.1); 0 (sampleWaypoints(3,:)-0.1)];
robotTrajectory = PositionModelPoseTrajectory(robotWaypoints,'R3','smoothingspline');

environment = Environment();
environment.addEllipsoid([1.25 1.25 3],8,'R3',primitiveTrajectory);
% nPoints = 5;
% environment.addStaticPoints([30*ones(1,nPoints); 40*rand(1,nPoints); 20*rand(1,nPoints)]);
% environment.addStaticPoints([30*rand(1,nPoints); 40*ones(1,nPoints); 20*rand(1,nPoints)]);

%% 3. Initialise Sensor
cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);

% standard sensor
% sensor = SimulatedEnvironmentSensor();
% sensor.addEnvironment(environment);
% sensor.addCamera(config.fieldOfView,cameraTrajectory);
% sensor.setVisibility(config);

% occlusion sensor
sensor = SimulatedEnvironmentOcclusionSensor();
sensor.addEnvironment(environment);
sensor.addCamera(config.fieldOfView,cameraTrajectory);
sensor.setVisibility(config,environment);

figure
spy(sensor.get('pointVisibility'));

%% 4. Plot Environment
% figure
% viewPoint = [-50,25];
% axisLimits = [-8 30 -5 40 -10 20];
% % title('Sensed Environment')
% axis equal
% xlabel('x (m)')
% ylabel('y (m)')
% zlabel('z (m)')
% view(viewPoint)
% axis(axisLimits)
% hold on
% grid on
% primitiveTrajectory.plot(t,[0 0 0],'axesOFF')
% cameraTrajectory.plot(t,[0 0 1],'axesOFF')
% % set(gcf,'Position',[0 0 1024 768]);
% frames = sensor.plot(t,environment);
% % implay(frames);

    %% 4.a output video
% v = VideoWriter('Data/Videos/App7_sensor_environment.mp4','MPEG-4');
% open(v)
% writeVideo(v,frames);
% close(v)

%% 5. Generate Measurements & Save to Graph File, load graph file as well
config.set('constantSE3Motion',constantSE3ObjectMotion);
    %% 5.1 For initial (without SE3)
    config.set('pointMotionMeasurement','Off');
    config.set('groundTruthFileName','app7_groundTruthNoSE3.graph');
    config.set('measurementsFileName','app7_measurementsNoSE3.graph');
    sensor.generateMeasurements(config);
    measurementsNoSE3Cell = graphFileToCell(config,config.measurementsFileName);
    groundTruthNoSE3Cell  = graphFileToCell(config,config.groundTruthFileName);
    
    %% 5.2 For test (with SE3)
    config.set('pointMotionMeasurement','point2DataAssociation');
    config.set('groundTruthFileName','app7_groundTruth.graph');
    config.set('measurementsFileName','app7_measurements.graph');
    sensor.generateMeasurements(config);
    writeDataAssociationVerticesEdges_constantSE3Motion(config,constantSE3ObjectMotion);
    measurementsCell = graphFileToCell(config,config.measurementsFileName);
    groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);

%% 6. Solve
    %% 6.1 Without SE3
    %no constraints
    timeStart = tic;
    initialGraph0 = Graph();
    initialSolver = initialGraph0.process(config,measurementsNoSE3Cell,groundTruthNoSE3Cell);
    initialSolverEnd = initialSolver(end);
    totalTime = toc(timeStart);
    fprintf('\nTotal time solving: %f\n',totalTime)

    %get desired graphs & systems
    initialGraph0  = initialSolverEnd.graphs(1);
    initialGraphN  = initialSolverEnd.graphs(end);
    %save results to graph file
    initialGraphN.saveGraphFile(config,'app7_resultsNoSE3.graph');
    
    %% 6.2 With SE3
    %no constraints
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
    graphN.saveGraphFile(config,'app7_results.graph');

%% 7. Error analysis
%load ground truth into graph, sort if required
graphGTNoSE3 = Graph(config,groundTruthNoSE3Cell);
graphGT = Graph(config,groundTruthCell);

fprintf('\nInitial results for without SE(3) Transform:\n')
resultsNoSE3 = errorAnalysis(config,graphGTNoSE3,initialGraphN);
fprintf('\nFinal results for SE(3) Transform:\n')
resultsSE3 = errorAnalysis(config,graphGT,graphN);

%% 8. Plot
    %% 8.1 Plot initial, final and ground-truth solutions
%no constraints
figure
subplot(1,2,1)
spy(solverEnd.systems(end).A)
subplot(1,2,2)
spy(solverEnd.systems(end).H)

h = figure; 
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
hold on
grid on
axis equal
axisLimits = [-8 30 0 40 -5 20];
axis(axisLimits)
view([-50,25])
%plot groundtruth
plotGraphFileICRA(config,groundTruthCell,'groundTruth');
%plot results
resultsNoSE3Cell = graphFileToCell(config,'app7_resultsNoSE3.graph');
resultsCell = graphFileToCell(config,'app7_results.graph');
plotGraphFileICRA(config,resultsNoSE3Cell,'initial',resultsNoSE3.relPose.get('R3xso3Pose'),resultsNoSE3.posePointsN.get('R3xso3Pose'))
plotGraphFileICRA(config,resultsCell,'solverResults',resultsSE3.relPose.get('R3xso3Pose'),resultsSE3.posePointsN.get('R3xso3Pose'))