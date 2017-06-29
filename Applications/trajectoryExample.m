%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

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
trajectory5 = DiscretePoseTrajectory(trajectory1.get('R3xso3Pose',t),t);
trajectory6 = DiscretePointTrajectory(5*rand(3,numel(t)),t);

% getting and setting
trajectory1.get('GP_Pose',1:0.1:1.5)
trajectory1.get('GP_Pose',1:0.1:1.5).get('R3xso3Pose')
trajectory1.get('R3xso3Pose',1:0.1:1.5)
trajectory1.get('logSE3Pose',3)
trajectory2.set('logSE3Pose',[0 0 1 0 pi/3 0]')
trajectory3.get('R3Position',1:2)
trajectory5.get('R3xso3Pose',[1 2 3])
% trajectory5.get('R3xso3Pose',1:0.1:1.5) % should create an error
trajectory5.get('R3xso3Pose',1.2:0.2:3)
trajectory5.set('R',eye(3),1);
trajectory6.set('R3Position',[1 2 3],1)
trajectories = [trajectory1 trajectory2];
trajectories.get('R3xso3Pose',3)

% transformation example inputs = (reference,time,varargin{1} = property)
trajectory2.AbsoluteToRelativePose(trajectory1,2) %returns GP_Pose
trajectory2.AbsoluteToRelativePose(trajectory1,2,'R3xso3Position')
trajectory1.AbsoluteToRelativePose(trajectory2,2)
trajectory1.AbsoluteToRelativePose(trajectory2,2,'R3xso3Position') % trajectory2 wrt trajectory1
AbsoluteToRelativePoseR3xso3(trajectory2.get('R3xso3Pose',2),trajectory1.get('R3xso3Pose',2))
trajectory1.RelativeToAbsolutePose(a,2,'R3xso3Pose')
trajectory3.AbsoluteToRelativePoint(trajectory2,1,'R3Position')
trajectory5.AbsoluteToRelativePose(trajectory2,[1 2])
trajectory5.RelativeToAbsolutePose(trajectory2,[1 2],'R3xso3Pose')
trajectory4.AbsoluteToRelativePoint(trajectory1,[1 2 3],'R3Position')

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
trajectory5.plot(t)
trajectory6.plot(t)