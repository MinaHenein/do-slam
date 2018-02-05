%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 25/01/2018
% Contributors:
%--------------------------------------------------------------------------
% Testing SE3Motion vertex initialization vs nb of observations needed to
% converge
%--------------------------------------------------------------------------
%% 1. Config
% time
function Test12_SE3MotionNObservationsNeeded(SE3VertexInitialization, noiseLevel, trial)


edges  = zeros(100,1);
sequence  = 5:18;

for i= sequence

nSteps = fibonacci(i);
t0 = 0;
tN = nSteps-1;
dt = (tN-t0)/(nSteps-1);
t  = linspace(t0,tN,nSteps);

config = CameraConfig();
config = setAppConfig(config); 
config.set('t',t);

if SE3VertexInitialization == 1
    SE3Vertex = 'SE3 Motion Vertex initialized as identity';
    config.set('SE3MotionVertexInitialization','eye');
elseif SE3VertexInitialization == 2
    SE3Vertex = 'SE3 Motion Vertex initialized as translation only';
    config.set('SE3MotionVertexInitialization','translation');
elseif SE3VertexInitialization == 3
    SE3Vertex = 'SE3 Motion Vertex initialized as GT';
    config.set('SE3MotionVertexInitialization','GT')
end

% SE3 Motion
config.set('motionModel','constantSE3MotionDA');

if noiseLevel == 1
    config.set('std2PointsSE3Motion', [0.01,0.01,0.01]');
    noise = '[0.01;0.01;0.01]';
elseif noiseLevel == 2
    config.set('std2PointsSE3Motion', [0.05,0.05,0.05]');
    noise = '[0.05;0.05;0.05]';
elseif noiseLevel == 3
    config.set('std2PointsSE3Motion', [0.1,0.1,0.1]');
    noise = '[0.1;0.1;0.1]';
end


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
robotWaypoints = [linspace(0,tN+5,numel(sampleTimes)+1); 0 sampleWaypoints(1,:); 0 (sampleWaypoints(2,:)+0.1); 0 (sampleWaypoints(3,:)-0.1)];
robotTrajectory = PositionModelPoseTrajectory(robotWaypoints,'R3','smoothingspline');

environment = Environment();
environment.addEllipsoid([1.25 1.25 3],12,'R3',primitiveTrajectory);
%% 3. Initialise Sensor
cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);

% occlusion sensor
sensor = SimulatedEnvironmentOcclusionSensor();
sensor.addEnvironment(environment);
sensor.addCamera(config.fieldOfView,cameraTrajectory);
sensor.setVisibility(config,environment);

%% 5. Generate Measurements & Save to Graph File, load graph file as well
config.set('constantSE3Motion',constantSE3ObjectMotion);
%% 5.1 For initial (without SE3)
config.set('pointMotionMeasurement','Off');
config.set('groundTruthFileName',strcat('test12_',num2str(nSteps),'_',...
    num2str(SE3VertexInitialization),'_',num2str(noiseLevel),'groundTruthNoSE3.graph'));
config.set('measurementsFileName',strcat('test12_',num2str(nSteps),'_',...
    num2str(SE3VertexInitialization),'_',num2str(noiseLevel),'measurementsNoSE3.graph'));
sensor.generateMeasurements(config);
measurementsNoSE3Cell = graphFileToCell(config,config.measurementsFileName);
groundTruthNoSE3Cell  = graphFileToCell(config,config.groundTruthFileName);

%% 5.2 For test (with SE3)
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('groundTruthFileName',strcat('test12_',num2str(nSteps),'_',...
    num2str(SE3VertexInitialization),'_',num2str(noiseLevel),'groundTruth.graph'));
config.set('measurementsFileName',strcat('test12_',num2str(nSteps),'_',...
    num2str(SE3VertexInitialization),'_',num2str(noiseLevel),'measurements.graph'));
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
initialGraphN.saveGraphFile(config,strcat('test12_',num2str(nSteps),'_',...
    num2str(SE3VertexInitialization),'_',num2str(noiseLevel),'resultsNoSE3.graph'));

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
graphN.saveGraphFile(config,strcat('test12_',num2str(nSteps),'_',...
    num2str(SE3VertexInitialization),'_',num2str(noiseLevel),'results.graph'));

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
save(strcat('Data/test12_results/resultsNoSE3_',num2str(nSteps),'_',...
    num2str(SE3VertexInitialization),'_',num2str(noiseLevel)),'resultsNoSE3')
save(strcat('Data/test12_results/resultsSE3_',num2str(nSteps),'_',...
    num2str(SE3VertexInitialization),'_',num2str(noiseLevel)),'resultsSE3')
save(strcat('Data/test12_results/H_',num2str(nSteps),'_',...
    num2str(SE3VertexInitialization),'_',num2str(noiseLevel)),'H')


iSE3MotionVertex = graphN.identifyEdges('2points-SE3Motion');
nSE3Edges =  length(iSE3MotionVertex);
edges(i-4) = nSE3Edges;

save(strcat('Data/test12_results/edges_',num2str(nSteps),'_',...
    num2str(SE3VertexInitialization),'_',num2str(noiseLevel)),'edges')
end


figure
for i = sequence
    resultsSE3 = load(strcat('Data/test12_results/resultsSE3_',num2str(fibonacci(i)),'_',...
    num2str(SE3VertexInitialization),'_',num2str(noiseLevel)));
    resultsSE3 = resultsSE3.resultsSE3;
    resultsNoSE3 = load(strcat('Data/test12_results/resultsNoSE3_',num2str(fibonacci(i)),'_',...
    num2str(SE3VertexInitialization),'_',num2str(noiseLevel)));
    resultsNoSE3 = resultsNoSE3.resultsNoSE3;
    
    plot(edges(i-(sequence(1)-1)),resultsSE3.ATE_translation_error,'*b')
    hold on;
    plot(edges(i-(sequence(1)-1)),resultsSE3.ATE_rotation_error,'*g')
    hold on;
    plot(edges(i-(sequence(1)-1)),resultsSE3.ASE_translation_error,'*m')
    hold on;
    plot(edges(i-(sequence(1)-1)),resultsNoSE3.ATE_translation_error,'ob')
    hold on;
    plot(edges(i-(sequence(1)-1)),resultsNoSE3.ATE_rotation_error,'og')
    hold on;
    plot(edges(i-(sequence(1)-1)),resultsNoSE3.ASE_translation_error,'om')
    xlabel('nSE3MotionEdges')
    ylabel('absolute error values')
    legend('SE3 ATE','SE3 ARE','SE3 ASE','NoSE3 ATE','NoSE3 ARE',...
        'NoSE3 ASE','Location','northwest')
    title({SE3Vertex,strcat('Ternary factory noise level: ',noise)});
end
print(strcat('Absolute_',num2str(trial)),'-dpdf')

figure
for i= sequence
    resultsSE3 = load(strcat('Data/test12_results/resultsSE3_',num2str(fibonacci(i)),'_',...
    num2str(SE3VertexInitialization),'_',num2str(noiseLevel)));
    resultsSE3 = resultsSE3.resultsSE3;
    resultsNoSE3 = load(strcat('Data/test12_results/resultsNoSE3_',num2str(fibonacci(i)),'_',...
    num2str(SE3VertexInitialization),'_',num2str(noiseLevel)));
    resultsNoSE3 = resultsNoSE3.resultsNoSE3;
    
    plot(edges(i-(sequence(1)-1)),resultsSE3.AARPE_translation_error,'*c')
    hold on;
    plot(edges(i-(sequence(1)-1)),resultsSE3.AARPE_rotation_error,'*k')
    hold on;
    plot(edges(i-(sequence(1)-1)),resultsSE3.AARPTE_translation_error,'*r');
    hold on;
    plot(edges(i-(sequence(1)-1)),resultsNoSE3.AARPE_translation_error,'oc')
    hold on;
    plot(edges(i-(sequence(1)-1)),resultsNoSE3.AARPE_rotation_error,'ok')
    hold on;
    plot(edges(i-(sequence(1)-1)),resultsNoSE3.AARPTE_translation_error,'or');
    xlabel('nSE3MotionEdges')
    ylabel('relative all-all error values')
    legend('SE3 allRTE','SE3 allRRE','SE3 allRSE','NoSE3 allRTE',...
        'NoSE3 allRRE','NoSE3 allRSE','Location','northwest');
    title({SE3Vertex,strcat('Ternary factory noise level: ',noise)});
end
print(strcat('Relative_',num2str(trial)),'-dpdf')
end