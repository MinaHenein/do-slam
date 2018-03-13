%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 20/02/18
% Contributors:
%--------------------------------------------------------------------------
% test15_multipleCameraCalibration

%% general setup
clear
close all

nCameras = 3;
plotEnvironment = 0;

t0 = 1;
tN = 320;
nSteps = tN;
t  = linspace(t0,tN,nSteps);
%% config setup
config = CameraConfig();
config = setUnitTestConfig(config);
config.set('t',t);
config.set('motionModel','constantSE3MotionDA');
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('std2PointsSE3Motion', [0.01,0.01,0.01]');
config.set('SE3MotionVertexInitialization','eye');
config.set('newMotionVertexPerNLandmarks',inf)
% config.set('noiseModel','Off');
rng(config.rngSeed);

%% set up cameras
camera1Pose = [3;0;0;0;0;0];
camera2Pose = [0;3;0;0;0;1.57];
camera3Pose = [-3;0;0;0;0;3.14];
camera4Pose = [0;-3;0;0;0;-1.57];

camera1Trajectory = StaticPoseTrajectory(camera1Pose);
camera2Trajectory = StaticPoseTrajectory(camera2Pose);
camera3Trajectory = StaticPoseTrajectory(camera3Pose);
camera4Trajectory = StaticPoseTrajectory(camera4Pose);
%% set up primitive
primitiveInitialPose_R3xso3 = [0 -9 0 0 0 0.002]';
primitiveMotion_R3xso3 = [0.2; 0; 0; arot(eul2rot([0.02,0,0]))];

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

if nCameras == 4
    sensor4 = SimulatedEnvironmentOcclusionSensor();
    sensor4.addEnvironment(environment);
    sensor4.addCamera(config.fieldOfView,camera4Trajectory);
    sensor4.setVisibility(config,environment);
end
figure
spy(sensor1.get('pointVisibility'));
figure
spy(sensor2.get('pointVisibility'));
figure
spy(sensor3.get('pointVisibility'));
if nCameras == 4 
    figure
    spy(sensor4.get('pointVisibility'));
end
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
    if nCameras == 4
        camera4Trajectory.plot(t,[0 0 1],'axesOFF')
    end
    % set(gcf,'Position',[0 0 1024 768]);
    frames1 = sensor1.plot(t,environment);
    frames2 = sensor2.plot(t,environment);
    frames3 = sensor3.plot(t,environment);
    if nCameras == 4
        frames4 = sensor4.plot(t,environment);
    end    
    implay(frames1);
    implay(frames2);
    implay(frames3);
    if nCameras == 4
        implay(frames4);    
    end    
end
%% 5.Generate Measurements & Save to Graph Files
config.set('groundTruthFileName' ,'groundTruthTest15a.graph');
config.set('measurementsFileName','measurementsTest15a.graph');
sensor1.generateMeasurements(config);
config.set('groundTruthFileName' ,'groundTruthTest15b.graph');
config.set('measurementsFileName','measurementsTest15b.graph');
sensor2.generateMeasurements(config);
config.set('groundTruthFileName' ,'groundTruthTest15c.graph');
config.set('measurementsFileName','measurementsTest15c.graph');
sensor3.generateMeasurements(config);
if nCameras == 4
    config.set('groundTruthFileName' ,'groundTruthTest15d.graph');
    config.set('measurementsFileName','measurementsTest15d.graph');
    sensor4.generateMeasurements(config);
end

config.set('groundTruthFileName' ,'groundTruthTest15.graph');
config.set('measurementsFileName','measurementsTest15.graph');

if nCameras == 3
    writeGraphFileMultipleCameras(config,[sensor1,sensor2,sensor3]);
elseif nCameras == 4
    writeGraphFileMultipleCameras(config,[sensor1,sensor2,sensor3,sensor4]);
end
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
graphN.saveGraphFile(config,'resultsTest15.graph');

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
resultsCell = graphFileToCell(config,'resultsTest15.graph');
hold  on
plotGraphFileICRA(config,resultsCell,'solverResults',results.relPose.get('R3xso3Pose'),results.posePointsN.get('R3xso3Pose'))

figure
xlabel('x')
ylabel('y')
zlabel('z')
hold on
plotGraphFile(config,groundTruthCell,[0 1 0])
hold on
plotGraphFile(config,resultsCell,[0 0 1])

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