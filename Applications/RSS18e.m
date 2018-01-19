%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 17/01/18
% Contributors:
%--------------------------------------------------------------------------

%% 1. Config
% time
trajectoryMode = 1;
% trajectoryMode = 2;
% trajectoryMode = 3;
% trajectoryMode = 4;

factor =  trajectoryMode*0.25;

t0 = 1;
tN = 500;
nSteps = 10*500;
t = linspace(t0,tN,nSteps);
dt = (tN-t0)/(nSteps-1);

config = CameraConfig();
config = setAppConfig(config);
% config = setLowErrorAppConfig(config);
% config = setHighErrorAppConfig(config);
config.set('noiseModel','Off');
config.set('groundTruthFileName','RSS18e_groundTruth.graph');
config.set('measurementsFileName','RSS18e_measurements.graph');
config.set('t',t(1:factor*length(t)));

% SE3 Motion
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('motionModel','constantSE3MotionDA');
config.set('std2PointsSE3Motion', [0.05,0.05,0.05]');

%% 2. Generate Environment
if config.rngSeed
    rng(config.rngSeed);
end

%% 2.1 robot waypoints and trajectory
t1 = linspace(t0,tN/2,nSteps/2);
t2 = linspace(tN/2,t0,nSteps/2);
% shiftting spiral down
robotWaypoints1 = [sin(t1 * .5); cos(t1 * .5) + t1 * .1; (25 - 10 * (.5 + .5 * sin(t1 /10)))]';
% shiftting spiral up
robotWaypoints2 = [-sin(t2 * .5); cos(t2 * .5) + t2 * .1; (25 - 10 * (.5 + .5 * sin(t2/ 10)))]';
robotWaypoints = [robotWaypoints1; robotWaypoints2];
robotWaypoints = reshape(robotWaypoints',[size(robotWaypoints,2),size(robotWaypoints,1)]);
robotTrajectoryWaypoints = [t(1:factor*length(t));robotWaypoints(:,1:factor*length(t))];
robotTrajectory = PositionModelPoseTrajectory(robotTrajectoryWaypoints,'R3','smoothingspline');
%% 2.2 objects waypoints and trajectories
% primitive 1
primitive1InitialPose_R3xso3 = [-2 5 -2 pi/3 0 pi/2]';
primitive1Motion_R3xso3 = [1.05*dt; 0; 0; arot(eul2rot([0.105*dt,0,0]))];
primitive1Trajectory = ConstantMotionDiscretePoseTrajectory(t(1:0.25*length(t)),...
    primitive1InitialPose_R3xso3,primitive1Motion_R3xso3,'R3xso3');
% primitive 2
if trajectoryMode ==4
    primitive2InitialPose_R3xso3 = primitive1Trajectory.get('R3xso3Pose',t(0.25*length(t)));
    primitive2Motion_R3xso3 = -primitive1Motion_R3xso3;
    primitive2Trajectory = ConstantMotionDiscretePoseTrajectory(t(3/2*length(t)/2:end),...
        primitive2InitialPose_R3xso3,primitive2Motion_R3xso3,'R3xso3');
end
% primitive 3
if trajectoryMode >=2
    primitive3InitialPose_R3xso3 = [2 5 -2 pi/3 0 -pi/2]';
    primitive3Motion_R3xso3 = [-1.6*dt; 0; 0; arot(eul2rot([-0.105*dt,0,0]))];
    primitive3Trajectory = ConstantMotionDiscretePoseTrajectory(t(0.25*length(t)+1:length(t)/2),...
        primitive3InitialPose_R3xso3,primitive3Motion_R3xso3,'R3xso3');
end
% primitive 4
if trajectoryMode >=3
    primitive4InitialPose_R3xso3 = primitive3Trajectory.get('R3xso3Pose',t(length(t)/2));
    primitive4Motion_R3xso3 = -primitive3Motion_R3xso3;
    primitive4Trajectory = ConstantMotionDiscretePoseTrajectory(t(length(t)/2:3/2*length(t)/2),...
        primitive4InitialPose_R3xso3,primitive4Motion_R3xso3,'R3xso3');
end
% primitive 5
line1 = [linspace(0,5,0.25*nSteps);linspace(8,12,0.25*nSteps);zeros(1,0.25*nSteps)];
primitive5Waypoints = [t(1:0.25*length(t)); line1];
primitive5Trajectory = PositionModelPoseTrajectory(primitive5Waypoints,'R3','linearinterp');
% primitive 6
if trajectoryMode ==4
    reversedLine1 = [linspace(5,0,0.25*nSteps);linspace(12,8,0.25*nSteps);zeros(1,0.25*nSteps)];
    primitive6Waypoints = [t(3/2*length(t)/2+1:end); reversedLine1];
    primitive6Trajectory = PositionModelPoseTrajectory(primitive6Waypoints,'R3','linearinterp');
end
% primitive 7
if trajectoryMode >=2
    line2 = [linspace(2,8,0.25*nSteps);linspace(10,20,0.25*nSteps);linspace(10,0,0.25*nSteps)];
    primitive7Waypoints = [t(0.25*length(t)+1:length(t)/2); line2];
    primitive7Trajectory = PositionModelPoseTrajectory(primitive7Waypoints,'R3','linearinterp');
end
% primitive 8
if trajectoryMode >=3
    reversedLine2 = [linspace(8,2,0.25*nSteps);linspace(20,10,0.25*nSteps);linspace(0,10,0.25*nSteps)];
    primitive8Waypoints = [t(length(t)/2+1:3/2*length(t)/2); reversedLine2];
    primitive8Trajectory = PositionModelPoseTrajectory(primitive8Waypoints,'R3','linearinterp');
end
%% constant SE3 motion
constantSE3ObjectMotion = [];
constantSE3ObjectMotion(:,1) = primitive1Trajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));
if trajectoryMode ==4
    constantSE3ObjectMotion(:,7) = primitive2Trajectory.RelativePoseGlobalFrameR3xso3(...
        t(3/2*length(t)/2),t((3/2*length(t)/2)+1));
end
if trajectoryMode >=2
    constantSE3ObjectMotion(:,3) = primitive3Trajectory.RelativePoseGlobalFrameR3xso3(...
      t(0.25*length(t)+1),t(0.25*length(t)+2));
end
if trajectoryMode >=3
    constantSE3ObjectMotion(:,5) = primitive4Trajectory.RelativePoseGlobalFrameR3xso3(...
        t(length(t)/2),t((length(t)/2)+1));
end
constantSE3ObjectMotion(:,2) = primitive5Trajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));
if trajectoryMode ==4
    constantSE3ObjectMotion(:,8) = primitive6Trajectory.RelativePoseGlobalFrameR3xso3(...
        t(3/2*length(t)/2),t((3/2*length(t)/2)+1));
end
if trajectoryMode >=2
    constantSE3ObjectMotion(:,4) = primitive7Trajectory.RelativePoseGlobalFrameR3xso3(...
        t(length(t)/2),t((length(t)/2)+1));
end
if trajectoryMode >=3
    constantSE3ObjectMotion(:,6) = primitive8Trajectory.RelativePoseGlobalFrameR3xso3(...
        t(length(t)/2),t((length(t)/2)+1));
end
%% construct environment
environment = Environment();
environment.addEllipsoid([1 1 2.5],8,'R3',primitive1Trajectory);
if trajectoryMode ==4
    environment.addEllipsoid([1 1 2.5],8,'R3',primitive2Trajectory);
end
if trajectoryMode >=2
    environment.addEllipsoid([1 1 2.5],8,'R3',primitive3Trajectory);
end
if trajectoryMode >=3
    environment.addEllipsoid([1 1 2.5],8,'R3',primitive4Trajectory);
end
environment.addEllipsoid([1 1 2.5],8,'R3',primitive5Trajectory);
if trajectoryMode ==4
    environment.addEllipsoid([1 1 2.5],8,'R3',primitive6Trajectory);
end
if trajectoryMode >=2
    environment.addEllipsoid([1 1 2.5],8,'R3',primitive7Trajectory);
end
if trajectoryMode >=3
    environment.addEllipsoid([1 1 2.5],8,'R3',primitive8Trajectory);
end
%% 3. Initialise Sensor
cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);

% occlusion sensor
sensor = SimulatedEnvironmentOcclusionSensor();
sensor.addEnvironment(environment);
sensor.addCamera(config.fieldOfView,cameraTrajectory);
sensor.setVisibility(config,environment);

figure
spy(sensor.get('pointVisibility'));
%% 4. Plot Environment
figure
viewPoint = [-35,35];
% axisLimits = [-30,30,-5,30,-10,10];
% title('Environment')
axis equal
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
view(viewPoint)
% axis(axisLimits)
hold on
grid on
primitive1Trajectory.plot(t(1:0.25*length(t)),[0 0 0],'axesOFF')
if trajectoryMode ==4
    primitive2Trajectory.plot(t(3/2*length(t)/2:end),[0 0 0],'axesOFF')
end
if trajectoryMode >=2
    primitive3Trajectory.plot(t(0.25*length(t)+1:length(t)/2),[0 0 0],'axesOFF')
end
if trajectoryMode >=3
    primitive4Trajectory.plot(t(length(t)/2:3/2*length(t)/2),[0 0 0],'axesOFF')
end
primitive5Trajectory.plot(t(1:0.25*length(t)),[0 0 0],'axesOFF')
if trajectoryMode ==4
    primitive6Trajectory.plot(t(3/2*length(t)/2:end),[0 0 0],'axesOFF')
end
if trajectoryMode >=2
    primitive7Trajectory.plot(t(0.25*length(t)+1:length(t)/2),[0 0 0],'axesOFF')
end
if trajectoryMode >=3
    primitive8Trajectory.plot(t(length(t)/2:3/2*length(t)/2),[0 0 0],'axesOFF')
end
cameraTrajectory.plot(t(1:factor*length(t)),[0 0 1],'axesOFF')
% set(gcf,'Position',[0 0 1024 768]);
frames = sensor.plot(t(1:factor*length(t)),environment);
implay(frames);

%% 4.a output video
% v = VideoWriter('Data/Videos/App6_sensor_environment.mp4','MPEG-4');
% open(v)
% writeVideo(v,frames);
% close(v)

%% 5. Generate Measurements & Save to Graph File, load graph file as well
config.set('constantSE3Motion',constantSE3ObjectMotion);
%% 5.1 For initial (without SE3)
config.set('pointMotionMeasurement','Off')
config.set('measurementsFileName','RSS18e_measurementsNoSE3.graph')
config.set('groundTruthFileName','RSS18e_groundTruthNoSE3.graph')
sensor.generateMeasurements(config);
groundTruthNoSE3Cell = graphFileToCell(config,config.groundTruthFileName);
measurementsNoSE3Cell = graphFileToCell(config,config.measurementsFileName);

%% 5.2 For test (with SE3)
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('measurementsFileName','RSS18e_measurements.graph');
config.set('groundTruthFileName','RSS18e_groundTruth.graph');
sensor.generateMeasurements(config);
writeDataAssociationVerticesEdges_constantSE3Motion(config,constantSE3ObjectMotion);
measurementsCell = graphFileToCell(config,config.measurementsFileName);
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);

%% 6. Solve
% config.set('sortVertices',1);
% config.set('sortEdges',1);
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
initialGraphN.saveGraphFile(config,'RSS18e_resultsNoSE3.graph');

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
graphN.saveGraphFile(config,'RSS18e_results.graph');

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

% figure
% spy(chol(solverEnd.systems(end).H))

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
resultsNoSE3Cell = graphFileToCell(config,'RSS18e_resultsNoSE3.graph');
resultsCell = graphFileToCell(config,'RSS18e_results.graph');
plotGraphFileICRA(config,resultsNoSE3Cell,'initial',resultsNoSE3.relPose.get('R3xso3Pose'),...
    resultsNoSE3.posePointsN.get('R3xso3Pose'))
plotGraphFileICRA(config,resultsCell,'solverResults',resultsSE3.relPose.get('R3xso3Pose'),...
    resultsSE3.posePointsN.get('R3xso3Pose'))