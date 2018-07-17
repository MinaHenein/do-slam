%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 28/01/18
% Contributors:
%--------------------------------------------------------------------------
% frontEndSolverExample
clear all 
% close all 

%% 1. Config
% time
nSteps = 64;
t0 = 1;
tN = 64;
dt = (tN-t0)/(nSteps-1);
t  = linspace(t0,tN,nSteps);

config = CameraConfig();
config = setAppConfig(config); 
config.set('t',t);
config.set('groundTruthFileName','RSS18ExpC_groundTruth.graph');
config.set('measurementsFileName','RSS18ExpC_measurements.graph');

% SE3 Motion
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('motionModel','constantSE3MotionDA');
config.set('std2PointsSE3Motion', [0.05,0.05,0.05]');

%% 2. Generate Environment
if config.rngSeed
    rng(config.rngSeed); 
end

robotWaypoints = [( -6 - .5 * (.5 + 10 * sin(t / 10))); sin(t * .5); cos(t * .5)]';
robotWaypoints = reshape(robotWaypoints',[size(robotWaypoints,2),size(robotWaypoints,1)]);
robotTrajectoryWaypoints = [linspace(0,tN,nSteps);robotWaypoints];

primitive1InitialPose_R3xso3 = [-15 5 5 pi/2 0 0]';
primitive1Motion_R3xso3 = [0.1*dt; 0; 0; arot(eul2rot([0,0,0]))];

primitive2InitialPose_R3xso3 = [3 -5 2 pi/2 0 0]';
primitive2Motion_R3xso3 = [-0.15*dt; 0; 0; arot(eul2rot([0,0,0]))];

% construct trajectories
robotTrajectory = PositionModelPoseTrajectory(robotTrajectoryWaypoints,'R3','smoothingspline');
primitive1Trajectory = ConstantMotionDiscretePoseTrajectory(t,primitive1InitialPose_R3xso3,primitive1Motion_R3xso3,'R3xso3');
primitive2Trajectory = ConstantMotionDiscretePoseTrajectory(t,primitive2InitialPose_R3xso3,primitive2Motion_R3xso3,'R3xso3');
constantSE3ObjectMotion = [];
constantSE3ObjectMotion(:,1) = primitive1Trajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));
constantSE3ObjectMotion(:,2) = primitive2Trajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));

environment = Environment();
environment.addEllipsoid([2.5 1 1],8,'R3',primitive1Trajectory);
environment.addEllipsoid([2.5 1 1],8,'R3',primitive2Trajectory);
%% 3. Initialise Sensor
cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);

% occlusion sensor
sensor = SimulatedEnvironmentOcclusionSensor();
sensor.addEnvironment(environment);
sensor.addCamera(config.fieldOfView,cameraTrajectory);
sensor.setVisibility(config,environment);

figure
spy(sensor.get('pointVisibility'));
print('RSS18ExpC_PointVisibility','-dpdf')
%% 4. Plot Environment
% figure
% viewPoint = [-35,35];
% % axisLimits = [-30,30,-5,30,-10,10];
% % title('Environment')
% axis equal
% xlabel('x (m)')
% ylabel('y (m)')
% zlabel('z (m)')
% view(viewPoint)
% % axis(axisLimits)
% hold on
% grid on
% primitive1Trajectory.plot(t,[0 0 0],'axesOFF')
% primitive2Trajectory.plot(t,[0 0 0],'axesOFF')
% cameraTrajectory.plot(t,[0 0 1],'axesOFF')
% % set(gcf,'Position',[0 0 1024 768]);
% frames = sensor.plot(t,environment);
% print('RSS18ExpC_Environment','-dpdf')
% % implay(frames);

    %% 4.a output video
% v = VideoWriter('Data/Videos/RSS18ExpC_sensor_environment.mp4','MPEG-4');
% open(v)
% writeVideo(v,frames);
% close(v)

%% 5. Generate Measurements & Save to Graph File, load graph file as well
config.set('constantSE3Motion',constantSE3ObjectMotion);
     %% 5.1 For initial (without SE3)
    config.set('pointMotionMeasurement','Off')
    config.set('measurementsFileName','RSS18ExpC_measurementsNoSE3.graph')
    config.set('groundTruthFileName','RSS18ExpC_groundTruthNoSE3.graph')
    sensor.generateMeasurements(config);
    groundTruthNoSE3Cell = graphFileToCell(config,config.groundTruthFileName);
    measurementsNoSE3Cell = graphFileToCell(config,config.measurementsFileName);
    
    %% 5.2 For test (with SE3)
    config.set('pointMotionMeasurement','point2DataAssociation');
    config.set('SE3MotionVertexInitialization','translation')
    config.set('newMotionVertexPerNLandmarks',inf)
    config.set('measurementsFileName','RSS18ExpC_measurements.graph');
    config.set('groundTruthFileName','RSS18ExpC_groundTruth.graph');
    sensor.generateMeasurements(config);
    writeDataAssociationVerticesEdges_constantSE3Motion(config,constantSE3ObjectMotion);
    measurementsCell = graphFileToCell(config,config.measurementsFileName);
    groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);

%% 6. Solve
    %% 6.1 Without SE3
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
    initialGraphN.saveGraphFile(config,'RSS18ExpC_resultsNoSE3.graph');
    
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
    graphN.saveGraphFile(config,'RSS18ExpC_results.graph');

%% 7. Error analysis
%load ground truth into graph, sort if required
graphGT = Graph(config,groundTruthCell);
graphGTNoSE3 = Graph(config,groundTruthNoSE3Cell);
fprintf('\nInitial results for without SE(3) Transform:\n')
resultsNoSE3 = errorAnalysis(config,graphGTNoSE3,initialGraphN);
fprintf('\nFinal results for SE(3) Transform:\n')
resultsSE3 = errorAnalysis(config,graphGT,graphN);

%% 8. Plot
    %% 8.1 Plot initial, final and ground-truth solutions
%no constraints
figure
spy(solverEnd.systems(end).H)
print('RSS18ExpC_H','-dpdf')

figure
spy(chol(solverEnd.systems(end).H))
print('RSS18ExpC_chol(H)','-dpdf')

h = figure; 
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
hold on
grid on
axis equal
axisLimits = [-25,25,0,30,-5,15];
axis(axisLimits)
view([-50,25])
%plot groundtruth
plotGraphFileICRA(config,groundTruthCell,'groundTruth');
%plot results
resultsNoSE3Cell = graphFileToCell(config,'RSS18ExpC_resultsNoSE3.graph');
resultsCell = graphFileToCell(config,'RSS18ExpC_results.graph');
plotGraphFileICRA(config,resultsNoSE3Cell,'initial',resultsNoSE3.relPose.get('R3xso3Pose'),resultsNoSE3.posePointsN.get('R3xso3Pose'))
plotGraphFileICRA(config,resultsCell,'solverResults',resultsSE3.relPose.get('R3xso3Pose'),resultsSE3.posePointsN.get('R3xso3Pose'))
print('RSS18ExpC_Solution','-dpdf')