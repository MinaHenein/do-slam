%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 28/02/18
% Contributors:
%--------------------------------------------------------------------------
% test16_multipleCarCameraCalibration

%% general setup
clear
close all

t0 = 1;
tN = 320;
nSteps = tN;
t  = linspace(t0,tN,nSteps);
plotEnvironment = 0;

%% config setup
config = CameraConfig();
config = setUnitTestConfig(config);
config.set('t',t);
config.set('motionModel','constantSE3MotionDA');
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('std2PointsSE3Motion', [0.001,0.001,0.001]');
config.set('SE3MotionVertexInitialization','eye');
config.set('newMotionVertexPerNLandmarks',inf)
% config.set('plotPlanes',1);
config.set('noiseModel','Off');
rng(config.rngSeed);

%% set up cameras
camera1Pose = [0;0;0;0;0;-1.93];
camera2Pose = [1.8;0;0;0;0;-1.2];
camera3Pose = [1.8;4.7;0;0;0;1.2];
camera4Pose = [0;4.7;0;0;0;1.93];

camera1Trajectory = StaticPoseTrajectory(camera1Pose);
camera2Trajectory = StaticPoseTrajectory(camera2Pose);
camera3Trajectory = StaticPoseTrajectory(camera3Pose);
camera4Trajectory = StaticPoseTrajectory(camera4Pose);
%% set up primitive
primitiveInitialPose_R3xso3 = [-9 3 0 0 0 0.002]';
primitiveMotion_R3xso3 = [0; -0.2; 0; arot(eul2rot([0.02,0,0]))];

% construct trajectories
primitiveTrajectory = ConstantMotionDiscretePoseTrajectory(t,primitiveInitialPose_R3xso3,primitiveMotion_R3xso3,'R3xso3');
objectMotion = primitiveTrajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));

environment = Environment();
environment.addEllipsoid([1.25 1.25 3],12,'R3',primitiveTrajectory);

%% initialise sensor
% occlusion sensor
sensor1 = SimulatedEnvironmentOcclusionSensor();
sensor1.addEnvironment(environment);
sensor1.addCamera(config.fieldOfView,camera1Trajectory);
sensor1.setVisibility(config,environment);

sensor2 = SimulatedEnvironmentOcclusionSensor();
sensor2.addEnvironment(environment);
sensor2.addCamera(config.fieldOfView,camera2Trajectory);
sensor2.setVisibility(config,environment);

sensor3 = SimulatedEnvironmentOcclusionSensor();
sensor3.addEnvironment(environment);
sensor3.addCamera(config.fieldOfView,camera3Trajectory);
sensor3.setVisibility(config,environment);

sensor4 = SimulatedEnvironmentOcclusionSensor();
sensor4.addEnvironment(environment);
sensor4.addCamera(config.fieldOfView,camera4Trajectory);
sensor4.setVisibility(config,environment);
 
figure
spy(sensor1.get('pointVisibility'),'r');
figure
spy(sensor2.get('pointVisibility'),'g');
figure
spy(sensor3.get('pointVisibility'),'b');
figure
spy(sensor4.get('pointVisibility'),'k');
%% 4. Plot Environment
if plotEnvironment
    figure
    axis equal
    xlabel('x (m)')
    ylabel('y (m)')
    zlabel('z (m)')
    % view(viewPoint)
    % axis(axisLimits)
    hold on
    grid on
    primitiveTrajectory.plot(t,[0 0 0],'axesOFF')
    camera1Trajectory.plot(t,[0 0 1],'axesOFF')
    camera2Trajectory.plot(t,[0 0 1],'axesOFF')
    camera3Trajectory.plot(t,[0 0 1],'axesOFF')
    camera4Trajectory.plot(t,[0 0 1],'axesOFF')
    % set(gcf,'Position',[0 0 1024 768]);
    frames1 = sensor1.plot(t,environment,'r');
    frames2 = sensor2.plot(t,environment,'g');
    frames3 = sensor3.plot(t,environment,'b');
    frames4 = sensor4.plot(t,environment,'k');
    implay(frames1);
    implay(frames2);
    implay(frames3);
    implay(frames4);   
end
%% 5.Generate Measurements & Save to Graph Files
% config.set('groundTruthFileName' ,'groundTruthTest16a.graph');
% config.set('measurementsFileName','measurementsTest16a.graph');
% sensor1.generateMeasurements(config);
% config.set('groundTruthFileName' ,'groundTruthTest16b.graph');
% config.set('measurementsFileName','measurementsTest16b.graph');
% sensor2.generateMeasurements(config);
% config.set('groundTruthFileName' ,'groundTruthTest16c.graph');
% config.set('measurementsFileName','measurementsTest16c.graph');
% sensor3.generateMeasurements(config);
% config.set('groundTruthFileName' ,'groundTruthTest16d.graph');
% config.set('measurementsFileName','measurementsTest16d.graph');
% sensor4.generateMeasurements(config);

config.set('groundTruthFileName' ,'groundTruthTest16.graph');
config.set('measurementsFileName','measurementsTest16.graph');
%writeGraphFileMultipleCameras(config,[sensor1,sensor2,sensor3,sensor4]);
generateMeasurementsMultipleCameras(config,[sensor1,sensor2,sensor3,sensor4]);
writeDataAssociationVerticesEdges_constantSE3Motion(config,objectMotion);
% writeDataAssociationVerticesEdges_constantSE3Motion_NoOrdering(config,objectMotion);
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);
measurementsCell = graphFileToCell(config,config.measurementsFileName);

%% 6. solve
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
graphN.saveGraphFile(config,'resultsTest16.graph');

graphGT = Graph(config,groundTruthCell);
results = errorAnalysis(config,graphGT,graphN);

%% plot graph files
figure
xlabel('x')
ylabel('y')
zlabel('z')
hold on
%plot groundtruth
plotGraphFileICRA(config,groundTruthCell,'groundTruth');
resultsCell = graphFileToCell(config,'resultsTest16.graph');
hold  on
plotGraphFileICRA(config,resultsCell,'solverResults',results.relPose.get('R3xso3Pose'),results.posePointsN.get('R3xso3Pose'))

% figure
% spy(solverEnd.systems(end).A)
% 
% figure
% spy(chol(solverEnd.systems(end).H))
% 
% [~,~,S] = chol(solverEnd.systems(end).H); 
% figure
% spy(S)

% figure
% subplot(2,2,1)
% spy(solverEnd.systems(end).A)
% subplot(2,2,2)
% spy(solverEnd.systems(end).H)
% subplot(2,2,3)
% spy(solverEnd.systems(end).covariance)
% subplot(2,2,4)
% spy(solverEnd.systems(end).covSqrtInv)