%icra18 main
% 0- $ roscore

% I- copy to a new terminal
    % $ rostopic echo /odom >> odom.txt
    % $ rostopic echo /mobile_base/sensors/imu_data >> imu.txt
    % $ rosbag play <rosbag_name.bag>
    
% II- run the matlab script writeOdomMeas.m
writeOdomMeas()
filePath = '/home/mina/workspace/src/Git/do-slam/Utils/icra18/odometryMeasGraphFile.txt';
modifyOdometryFile(filePath,51)

% III- in a new terminal
    % $ cd catkin_ws/
    % $ catkin_make
    % $ source devel/setup.bash
    % $ rosrun depth_extraction extract_depth_images.py
    
% III- in a different terminal
    % $ roscore 
    % $ rosbag play <rosbag_name.bag>

% IV-
%% GT & Measurements Graph Files
n = 51;
VICONFilePath = '/home/mina/Downloads/icra18/VICON/';
writeVICONGroundtruth(VICONFilePath,'robot.txt')
filePath = '/home/mina/workspace/src/Git/do-slam/Utils/icra18/';
modifyGTFile(strcat(filePath,'cameraGroundtruth.txt'),n)
writeVICONGroundtruth(VICONFilePath,'obj1.txt')
modifyGTFile(strcat(filePath,'obj1Groundtruth.txt'),n)
writeVICONGroundtruth(VICONFilePath,'obj2.txt')
modifyGTFile(strcat(filePath,'obj2Groundtruth.txt'),n)

rgbImagesPath =  '/home/mina/Downloads/icra18/images/rgb/';
depthImagesPath =  '/home/mina/Downloads//icra18/images/depth/';
K_Cam = [526.37013657, 0.00000000  , 313.68782938;
         0.00000000  , 526.37013657, 259.01834898;
         0.00000000  , 0.00000000  , 1.00000000 ];
     
[pointsMeasurements,pointsLabels,pointsTurtlebotID,pointsCameras] = ...
    manualLandmarkExtraction(rgbImagesPath,depthImagesPath, K_Cam);
pointsMeasurements = reshape(pointsMeasurements,[3,size(pointsMeasurements,1)/3])';
save('pointsMeasurements','pointsMeasurements');
save('pointsLabels','pointsLabels');
save('pointsTurtlebotID','pointsTurtlebotID');
save('pointsCameras','pointsCameras');
writeLandmarkMeas(pointsMeasurements,pointsLabels,pointsCameras)

unique3DPoints = extractUnique3DPoints(filePath);
writeGroundtruthGraphFile(filePath,unique3DPoints)
writeMeasurementsGraphFile(filePath)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1. Config
config = CameraConfig();
config = setAppConfig(config);
%config.set('noiseModel','Off');
config.set('groundTruthFileName','GT_GraphFile.graph');
config.set('measurementsFileName','Meas_GraphFile.graph');

% SE3 Motion
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('motionModel','constantSE3MotionDA');
config.set('std2PointsSE3Motion', [0.1,0.1,0.1]');

config.set('constantSE3Motion',constantSE3ObjectMotion);

writeDataAssociationVerticesEdges(config,constantSE3ObjectMotion);

%% 6. load graph files
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);
measurementsCell = graphFileToCell(config,config.measurementsFileName);

%% 7. Solve
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
fprintf('\nChi-squared error: %f\n',solverEnd.systems(end).chiSquaredError)
%save results to graph file
graphN.saveGraphFile(config,'icra18_results.graph');

%% 9. Error analysis
%load ground truth into graph, sort if required
graphGT = Graph(config,groundTruthCell);
results = errorAnalysis(config,graphGT,graphN);
fprintf('Chi Squared Error: %.4d \n',solverEnd.systems.chiSquaredError)
fprintf('Absolute Trajectory Translation Error: %.4d \n',results.ATE_translation_error)
fprintf('Absolute Trajectory Rotation Error: %.4d \n',results.ATE_rotation_error)
fprintf('Absolute Structure Points Error: %d \n',results.ASE_translation_error);
fprintf('All to All Relative Pose Squared Translation Error: %.4d \n',results.AARPE_squared_translation_error)
fprintf('All to All Relative Pose Squared Rotation Error: %.4d \n',results.AARPE_squared_rotation_error)
fprintf('All to All Relative Point Squared Translation Error: %.4d \n',results.AARPTE_squared_translation_error)

%% 10. Plot
    %% 10.1 Plot intial, final and ground-truth solutions
figure
subplot(1,2,1)
spy(solverEnd.systems(end).A)
subplot(1,2,2)
spy(solverEnd.systems(end).H)
h = figure; 
xlabel('x')
ylabel('y')
zlabel('z')
hold on
view([-50,25])
plotGraphFile(config,groundTruthCell,[0 0 1]);
resultsCell = graphFileToCell(config,'icra18_results.graph');
plotGraphFile(config,resultsCell,[1 0 0])