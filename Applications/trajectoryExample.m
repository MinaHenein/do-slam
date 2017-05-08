clear all
close all

%% Example - trajectories
nSteps = 51;
t0 = 0;
tN = 10;
t  = linspace(t0,tN,nSteps);
% waypoints
waypoints = [0:2:tN; 4 5 7 10 7 3; 12 11 10 5 8 14; 8 5 2 3 6 4];
%construct trajectories
trajectory1 = PositionModelPoseTrajectory(waypoints,'R3','smoothingspline');
trajectory2 = StaticPoseTrajectory([1 2 3 0 0 pi/2]','R3xso3');
trajectory3 = PositionModelPointTrajectory(waypoints,'R3','smoothingspline');
trajectory4 = StaticPointTrajectory([5 5 5]','R3');

% getting and setting
trajectory1.get('logSE3Pose',3)
trajectory2.set('logSE3Pose',[0 0 1 0 pi/3 0]');
trajectory3.get('R3Position',2)
trajectories = [trajectory1 trajectory2];
trajectories.get('R3xso3Pose',3)

% transformation example inputs = (reference,time,varargin{1} = property)
trajectory1.AbsoluteToRelativePose(trajectory2,2) %returns GP_Pose
trajectory1.AbsoluteToRelativePose(trajectory2,2,'R3xso3Position')
trajectory1.RelativeToAbsolutePose(trajectory2,2,'R3xso3Pose')
trajectory3.AbsoluteToRelativePoint(trajectory2,1,'R3Position')

figure
hold on
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
trajectory1.plot(t)
trajectory2.plot()
trajectory3.plot(t)
trajectory4.plot()