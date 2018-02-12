%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 25/01/2018
% Contributors:
%--------------------------------------------------------------------------
% Testing SE3Motion vertex initialization vs nb of observations needed to
% converge -- when n-landmarks are connected to a new H
%--------------------------------------------------------------------------
%% 1. Config
% time
function Test14_newSE3MotionVertexPerNLandmarks(n)

edges  = zeros(100,1);
sequence  = 5:16;

for i= sequence

nSteps = fibonacci(i);
t0 = 0;
tN = nSteps-1;
dt = (tN-t0)/(nSteps-1);
t  = linspace(t0,tN,nSteps);

config = CameraConfig();
config = setAppConfig(config); 
config.set('t',t);

config = CameraConfig();
config = setAppConfig(config); 
config.set('t',t);

SE3VertexInitialization = 1;
SE3Vertex = 'SE3 Motion Vertex initialized as identity';

% SE3 Motion
config.set('motionModel','constantSE3MotionDA');

noiseLevel = 2;
config.set('std2PointsSE3Motion', [0.05,0.05,0.05]');
noise = '[0.05;0.05;0.05]';
config.set('newMotionVertexPerNLandmarks',n)

%% 2. Generate Environment
if config.rngSeed
    rng(config.rngSeed); 
end

primitiveInitialPose_R3xso3 = [10 3 3 0 0 0.2]';
primitiveMotion_R3xso3 = [0.8*dt; 0; -0.2*dt; arot(eul2rot([0.06*dt,0,0]))];

firstPose = primitiveInitialPose_R3xso3;
    
for j=1:10
    if j==1
        lastPose = poseToTransformationMatrix(firstPose)/(poseToTransformationMatrix(primitiveMotion_R3xso3));
    else
        lastPose = lastPose/(poseToTransformationMatrix(primitiveMotion_R3xso3));
    end
end


% construct trajectories
primitiveTrajectory = ConstantMotionDiscretePoseTrajectory(t,primitiveInitialPose_R3xso3,primitiveMotion_R3xso3,'R3xso3');
constantSE3ObjectMotion = primitiveTrajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));
pose = transformationMatrixToPose(lastPose);

robotInitialPose_R3xso3 = pose;
robotMotion_R3xso3 = primitiveMotion_R3xso3;robotTrajectory = ConstantMotionDiscretePoseTrajectory(t,robotInitialPose_R3xso3,robotMotion_R3xso3,'R3xso3');

environment = Environment();
environment.addEllipsoid([1.25 1.25 3],12,'R3',primitiveTrajectory);

cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);


% occlusion sensor
sensor = SimulatedEnvironmentOcclusionSensor();
sensor.addEnvironment(environment);
sensor.addCamera(config.fieldOfView,cameraTrajectory);
sensor.setVisibility(config,environment);

% figure
% spy(sensor.get('pointVisibility'));

%% 4. Plot Environment
% figure
% viewPoint = [-35,35];
% axis equal
% xlabel('x (m)')
% ylabel('y (m)')
% zlabel('z (m)')
% view(viewPoint)
% hold on
% grid on
% primitiveTrajectory.plot(t,[0 0 0],'axesOFF')
% cameraTrajectory.plot(t,[0 0 1],'axesOFF')
% frames = sensor.plot(t,environment);
% implay(frames);

%% 5. Generate Measurements & Save to Graph File, load graph file as well
config.set('constantSE3Motion',constantSE3ObjectMotion);
%% 5.1 For initial (without SE3)
config.set('pointMotionMeasurement','Off');
config.set('groundTruthFileName',strcat('test14_',num2str(nSteps),'_',...
    num2str(n),'groundTruthNoSE3.graph'));
config.set('measurementsFileName',strcat('test14_',num2str(nSteps),'_',...
    num2str(n),'measurementsNoSE3.graph'));
sensor.generateMeasurements(config);
measurementsNoSE3Cell = graphFileToCell(config,config.measurementsFileName);
groundTruthNoSE3Cell  = graphFileToCell(config,config.groundTruthFileName);

%% 5.2 For test (with SE3)
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('groundTruthFileName',strcat('test14_',num2str(nSteps),'_',...
    num2str(n),'groundTruth.graph'));
config.set('measurementsFileName',strcat('test14_',num2str(nSteps),'_',...
    num2str(n),'measurements.graph'));
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
initialGraphN.saveGraphFile(config,strcat('test14_',num2str(nSteps),'_',...
    num2str(n),'resultsNoSE3.graph'));

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
graphN.saveGraphFile(config,strcat('test14_',num2str(nSteps),'_',...
    num2str(n),'results.graph'));

%% 7. Error analysis
%load ground truth into graph, sort if required
graphGTNoSE3 = Graph(config,groundTruthNoSE3Cell);
graphGT = Graph(config,groundTruthCell);

fprintf('\nInitial results for without SE(3) Transform:\n')
resultsNoSE3 = errorAnalysis(config,graphGTNoSE3,initialGraphN);
fprintf('\nFinal results for SE(3) Transform:\n')
resultsSE3 = errorAnalysis(config,graphGT,graphN);

iSE3MotionVertex = graphN.identifyVertices('SE3Motion');
SE3MotionVertexValue = graphN.vertices(iSE3MotionVertex).value;
fprintf('\nNorm of SE(3)^-1 SE(3)_GT:\n');
disp(norm(poseToTransformationMatrix(SE3MotionVertexValue)\...
    poseToTransformationMatrix(constantSE3ObjectMotion)))

v_error = AbsoluteToRelativePoseR3xso3(SE3MotionVertexValue, constantSE3ObjectMotion);
f_trans_error2 = norm(v_error(1:3))^2;
r_f_translation_error = sqrt(f_trans_error2);
fprintf('\nTranslational error of SE(3)_N vs SE(3)_GT:\n');
disp(r_f_translation_error)
f_rot_error_degrees = wrapToPi(norm(v_error(4:6))) * 180/pi;
fprintf('\nRotational error of SE(3)_N vs SE(3)_GT:\n');
disp(f_rot_error_degrees)


fprintf('\nPoint visibility:\n');
disp(nnz(sensor.get('pointVisibility')))


% save results
H = solverEnd.systems(end).H;
save(strcat('Data/test14_results/resultsNoSE3_',num2str(nSteps),'_',...
    num2str(n)),'resultsNoSE3')
save(strcat('Data/test14_results/resultsSE3_',num2str(nSteps),'_',...
    num2str(n)),'resultsSE3')
save(strcat('Data/test14_results/H_',num2str(nSteps),'_',...
    num2str(n)),'H')


iSE3MotionVertex = graphN.identifyEdges('2points-SE3Motion');
nSE3Edges =  length(iSE3MotionVertex);
edges(i-4) = nSE3Edges;

save(strcat('Data/test14_results/edges_',num2str(nSteps),'_',...
    num2str(n)),'edges')
end


% figure
% for i = sequence
%     resultsSE3 = load(strcat('Data/test14_results/resultsSE3_',num2str(fibonacci(i)),'_',...
%     num2str(SE3VertexInitialization),'_',num2str(noiseLevel)));
%     resultsSE3 = resultsSE3.resultsSE3;
%     resultsNoSE3 = load(strcat('Data/test14_results/resultsNoSE3_',num2str(fibonacci(i)),'_',...
%     num2str(SE3VertexInitialization),'_',num2str(noiseLevel)));
%     resultsNoSE3 = resultsNoSE3.resultsNoSE3;
%     
%     plot(edges(i-(sequence(1)-1)),resultsSE3.ATE_translation_error,'*b')
%     hold on;
%     plot(edges(i-(sequence(1)-1)),resultsSE3.ATE_rotation_error,'*g')
%     hold on;
%     plot(edges(i-(sequence(1)-1)),resultsSE3.ASE_translation_error,'*m')
%     hold on;
%     plot(edges(i-(sequence(1)-1)),resultsNoSE3.ATE_translation_error,'ob')
%     hold on;
%     plot(edges(i-(sequence(1)-1)),resultsNoSE3.ATE_rotation_error,'og')
%     hold on;
%     plot(edges(i-(sequence(1)-1)),resultsNoSE3.ASE_translation_error,'om')
%     xlabel('nSE3MotionEdges')
%     ylabel('absolute error values')
%     legend('SE3 ATE','SE3 ARE','SE3 ASE','NoSE3 ATE','NoSE3 ARE',...
%         'NoSE3 ASE','Location','northwest')
%     title({SE3Vertex,strcat('Ternary factory noise level: ',noise)});
% end
% print(strcat('Absolute2_',num2str(trial)),'-dpdf')
% 
% figure
% for i= sequence
%     resultsSE3 = load(strcat('Data/test14_results/resultsSE3_',num2str(fibonacci(i)),'_',...
%     num2str(SE3VertexInitialization),'_',num2str(noiseLevel)));
%     resultsSE3 = resultsSE3.resultsSE3;
%     resultsNoSE3 = load(strcat('Data/test14_results/resultsNoSE3_',num2str(fibonacci(i)),'_',...
%     num2str(SE3VertexInitialization),'_',num2str(noiseLevel)));
%     resultsNoSE3 = resultsNoSE3.resultsNoSE3;
%     
%     plot(edges(i-(sequence(1)-1)),resultsSE3.AARPE_translation_error,'*c')
%     hold on;
%     plot(edges(i-(sequence(1)-1)),resultsSE3.AARPE_rotation_error,'*k')
%     hold on;
%     plot(edges(i-(sequence(1)-1)),resultsSE3.AARPTE_translation_error,'*r');
%     hold on;
%     plot(edges(i-(sequence(1)-1)),resultsNoSE3.AARPE_translation_error,'oc')
%     hold on;
%     plot(edges(i-(sequence(1)-1)),resultsNoSE3.AARPE_rotation_error,'ok')
%     hold on;
%     plot(edges(i-(sequence(1)-1)),resultsNoSE3.AARPTE_translation_error,'or');
%     xlabel('nSE3MotionEdges')
%     ylabel('relative all-all error values')
%     legend('SE3 allRTE','SE3 allRRE','SE3 allRSE','NoSE3 allRTE',...
%         'NoSE3 allRRE','NoSE3 allRSE','Location','northwest');
%     title({SE3Vertex,strcat('Ternary factory noise level: ',noise)});
% end
% print(strcat('Relative2_',num2str(trial)),'-dpdf')
end