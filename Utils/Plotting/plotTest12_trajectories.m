function plotTest12_trajectories()


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


pose = transformationMatrixToPose(lastPose);
% construct trajectories
primitiveTrajectory = ConstantMotionDiscretePoseTrajectory(t,primitiveInitialPose_R3xso3,primitiveMotion_R3xso3,'R3xso3');


robotInitialPose_R3xso3 = pose;
robotMotion_R3xso3 = primitiveMotion_R3xso3;robotTrajectory = ConstantMotionDiscretePoseTrajectory(t,robotInitialPose_R3xso3,robotMotion_R3xso3,'R3xso3');

environment = Environment();
environment.addEllipsoid([1.25 1.25 3],12,'R3',primitiveTrajectory);

cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);

sensor = SimulatedEnvironmentOcclusionSensor();
sensor.addEnvironment(environment);
sensor.addCamera(config.fieldOfView,cameraTrajectory);
sensor.setVisibility(config,environment);


figure
spy(sensor.get('pointVisibility'));

%% 4. Plot Environment
figure
viewPoint = [-35,35];
axis equal
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
view(viewPoint)
hold on
grid on
primitiveTrajectory.plot(t,[0 0 0],'axesOFF')
cameraTrajectory.plot(t,[0 0 1],'axesOFF')
frames = sensor.plot(t,environment);
% implay(frames);

end

end