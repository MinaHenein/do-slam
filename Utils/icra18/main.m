% icra18 main
%% 0- $ roscore

%% I- copy to a new terminal
    % $ rostopic echo /odom >> odom.txt
    % $ rostopic echo /mobile_base/sensors/imu_data >> imu.txt
    % $ rosbag play <rosbag_name.bag>
    
%% II- run the matlab script getOdomIMUTimeStamp & getOdomIMUMeas.m
getOdomIMUTimeStamp()
[odomMeas,imuMeas] = getOdomIMUMeas();
save('odomMeas','odomMeas');
save('imuMeas','imuMeas');

%% III- in a new terminal
    % $ cd catkin_ws/
    % $ catkin_make
    % $ source devel/setup.bash
    % $ rosrun depth_extraction extract_depth_images.py
    % save rgbTimeStamp & depthTimeStamp as .mat 
    % - in a different terminal
        % $ roscore 
        % $ rosbag play <rosbag_name.bag>

%% IV- manually extract landmarks        
rgbImagesPath =  '/home/mina/Downloads/icra18/images/rgb/';
depthImagesPath =  '/home/mina/Downloads//icra18/images/depth/';
K_Cam = [526.37013657, 0.00000000  , 313.68782938;
         0.00000000  , 526.37013657, 259.01834898;
         0.00000000  , 0.00000000  , 1.00000000 ];
[pointsMeasurements,pointsLabels,pointsTurtlebotID,pointsCameras] = ...
    manualLandmarkExtraction(rgbImagesPath,depthImagesPath, K_Cam);
pointsMeasurements = reshape(pointsMeasurements,[3,138])';
save('pointsMeasurements','pointsMeasurements');
save('pointsLabels','pointsLabels');
save('pointsTurtlebotID','pointsTurtlebotID');
save('pointsCameras','pointsCameras');

%% V- synchronise Data & write GT poses files
load('rgbTimeStamp');
rgbTimeStamp = rgbTimeStamp(3:30,1);
load('odomTimeStamp');
load('imuTimeStamp');
load('GTCameraPoseTimeStamp');
load('GTObj1PoseTimeStamp');
load('GTObj2PoseTimeStamp');
synchronisedData = synchroniseROSVICONData(rgbTimeStamp,odomTimeStamp,imuTimeStamp,...
    GTCameraPoseTimeStamp,GTObj1PoseTimeStamp,GTObj2PoseTimeStamp);
save('synchronisedData','synchronisedData');

VICONFilePath = '/home/mina/Downloads/icra18/VICON/';
robotGTPoses = getVICONGroundtruth(VICONFilePath,'robot.txt');
obj1GTPoses = getVICONGroundtruth(VICONFilePath,'obj1.txt');
obj2GTPoses = getVICONGroundtruth(VICONFilePath,'obj2.txt');
writeVICONGroundtruth('robot',robotGTPoses,synchronisedData)
writeVICONGroundtruth('obj1',obj1GTPoses,synchronisedData)
writeVICONGroundtruth('obj2',obj2GTPoses,synchronisedData)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% VI- write odometry & landmark measurements
odomSigma = [0.04,0.04,0.04,pi/90,pi/90,pi/90]';
pointMeasSigma = [0.2,0.2,0.2];

odomCov = [odomSigma(1)^2,0.0,0.0,0.0,0.0,0.0,odomSigma(2)^2,0.0,0.0,0.0,0.0,...
    odomSigma(3)^2,0.0,0.0,0.0,odomSigma(4)^2,0.0,0.0,odomSigma(5)^2,0.0,odomSigma(6)^2];
% writeOdomMeas(odomMeas,imuMeas,synchronisedData,odomCov)
% writeOdomMeas2(odomSigma,odomCov) % obtained from GT data + noise
writeOdomMeas3(odomMeas,imuMeas,synchronisedData,odomCov) % integrating odometry measurement between intervals
pointMeasCov = [pointMeasSigma(1)^2 0.0 0.0 pointMeasSigma(2)^2 0.0 pointMeasSigma(3)^2];
writeLandmarkMeas2(pointsMeasurements,pointsLabels,pointsCameras,pointMeasCov)

%% VII- GT & Measurements Graph Files
filePath = '/home/mina/workspace/src/Git/do-slam/Utils/icra18/';
% unique3DPoints = extractUnique3DPoints2(filePath);
load('unique3DPoints');
writeGroundtruthGraphFile2(filePath,unique3DPoints)
writeMeasurementsGraphFile2(filePath)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% VIII- config w/ SE3
config = CameraConfig();
setAppConfig(config); 
config.set('std2PointsSE3Motion', [0.05,0.05,0.05]');
config.set('groundTruthFileName','icra18GT_GraphFile.graph');
config.set('measurementsFileName','icra18Measurement_GraphFile.graph');

obj1FilePath = '/home/mina/workspace/src/Git/do-slam/Utils/icra18/obj1Groundtruth.txt';
constantSE3Object1Motion = getSE3MotionVertexValue(obj1FilePath);
obj2FilePath = '/home/mina/workspace/src/Git/do-slam/Utils/icra18/obj2Groundtruth.txt';
constantSE3Object2Motion = getSE3MotionVertexValue(obj2FilePath);
constantSE3ObjectMotion = [constantSE3Object1Motion,...
    constantSE3Object2Motion];
writeDataAssociationVerticesEdges(config,constantSE3ObjectMotion);

%% IX- solve w/ SE3
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);
measurementsCell = graphFileToCell(config,config.measurementsFileName);
timeStart = tic;
graph0 = Graph();
solver = graph0.process(config,measurementsCell,groundTruthCell);
solverEnd = solver(end);
totalTime = toc(timeStart);
fprintf('\nTotal time solving: %f\n',totalTime)
graph0  = solverEnd.graphs(1);
graphN  = solverEnd.graphs(end);
fprintf('\nChi-squared error: %f\n',solverEnd.systems(end).chiSquaredError)
graphN.saveGraphFile(config,'icra18_results.graph');

%% X- Error analysis w/ SE3
graphGT = Graph(config,groundTruthCell);
resultsSE3 = errorAnalysis(config,graphGT,graphN);

%% XI- Plot w/ SE3
% figure
% subplot(1,2,1)
% spy(solverEnd.systems(end).A)
% subplot(1,2,2)
% spy(solverEnd.systems(end).H)
% h = figure; 
% xlabel('x')
% ylabel('y')
% zlabel('z')
% hold on
% view([-50,25])
% plotGraphFile(config,groundTruthCell,[0 0 1]);
resultsCell = graphFileToCell(config,'icra18_results.graph');
% plotICRARealDataGraphFile(config,resultsCell,[1 0 0])

%% VIII- config w/o SE3
GTFilePath = '/home/mina/workspace/src/Git/do-slam/Data/GraphFiles/icra18GT_GraphFile';
generateGraphFilesWithNoSE3VerticesEdges(GTFilePath);
MeasFilePath = '/home/mina/workspace/src/Git/do-slam/Data/GraphFiles/icra18Measurement_GraphFile';
generateGraphFilesWithNoSE3VerticesEdges(MeasFilePath);
config = CameraConfig();
setAppConfig(config);
config.set('groundTruthFileName','icra18GT_GraphFileNOSE3.graph');
config.set('measurementsFileName','icra18Measurement_GraphFileNOSE3.graph');
%% IX- solve w/o SE3
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);
measurementsCell = graphFileToCell(config,config.measurementsFileName);
timeStart = tic;
graph0 = Graph();
solver = graph0.process(config,measurementsCell,groundTruthCell);
solverEnd = solver(end);
totalTime = toc(timeStart);
fprintf('\nTotal time solving: %f\n',totalTime)
graph0  = solverEnd.graphs(1);
graphN  = solverEnd.graphs(end);
fprintf('\nChi-squared error: %f\n',solverEnd.systems(end).chiSquaredError)
graphN.saveGraphFile(config,'icra18_resultsNoSE3.graph');
%% X- Error analysis w/o SE3
graphGT = Graph(config,groundTruthCell);
resultsNoSE3 = errorAnalysis(config,graphGT,graphN);
%% XI- Plot w/o SE3
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
resultsCell = graphFileToCell(config,'icra18_resultsNoSE3.graph');
plotGraphFile(config,resultsCell,[1 0 0])

% %% different plot

j = figure; 
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
hold on
grid on
axis equal
view([-50,25])
%plot groundtruth
plotGraphFileICRA(config,groundTruthCell,'groundTruth');
%plot results
resultsNoSE3Cell = graphFileToCell(config,'icra18_resultsNoSE3.graph');
resultsCell = graphFileToCell(config,'icra18_results.graph');
plotGraphFileICRA(config,resultsNoSE3Cell,'initial',...
resultsNoSE3.relPose.get('R3xso3Pose'),resultsNoSE3.posePointsN.get('R3xso3Pose'))
plotGraphFileICRA(config,resultsCell,'solverResults',...
resultsSE3.relPose.get('R3xso3Pose'),resultsSE3.posePointsN.get('R3xso3Pose'))

%% a third plot
cameraFilePath = '/home/mina/workspace/src/Git/do-slam/Utils/icra18/cameraGroundtruth.txt';
figure
plotObjectGTPoses(cameraFilePath,[0 1 0])
xlabel('x')
ylabel('y')
zlabel('z')
grid on
%axis equal
hold on
plotObjectGTPoses(obj1FilePath,'m')
xlabel('x')
ylabel('y')
zlabel('z')
grid on
%axis equal
hold on
plotObjectGTPoses(obj2FilePath,'c')
xlabel('x')
ylabel('y')
zlabel('z')
grid on
%axis equal
view([0 0 1])
hold on
%% 1. get vertices and labels
rowLengths = cellfun('length',resultsCell);
vertexRows = (rowLengths==3);
verticesCellTemp = resultsCell(vertexRows,:);
verticesCell = cell(sum(vertexRows),3);
for i = 1:sum(vertexRows)
    verticesCell(i,:) = verticesCellTemp{i,:};
end
vertexLabels = verticesCell(:,1);
poseVertices  = strcmp(vertexLabels,config.poseVertexLabel);
poses  = [verticesCell{poseVertices,3}];
plot3(poses(1,:),poses(2,:),poses(3,:),'Color','k','Marker','o','LineStyle','none');
% for i = 1:sum(poseVertices)
%     iPose = poses(:,i);
%     scale = 0.3;
%     plotCoordinates(iPose(1:3,:),scale*rot(iPose(4:6,1)))
% end
H1 = []';
H2 = []';
hold on
plotObjectPoses(obj1FilePath,H1,'r')
hold on
plotObjectPoses(obj2FilePath,H2,'b')


