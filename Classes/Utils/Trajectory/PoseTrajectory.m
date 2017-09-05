%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

classdef PoseTrajectory < Trajectory
    %PoseTrajectory represents 6D poses
    %   subclasses of PoseTrajectory are:
    %       -PositionModelPoseTrajectory
    %       -PoseModelPoseTrajectory
    %       -StaticPoseTrajectory
    %   these subclasses are instantiated differently
    %   these subclasses have the same external interface (except setting)
    %   PoseTrajectory methods for frame transformations/plotting are
    %   implemented    
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
    end
    
    %% 2. Methods    
    % Constructor
    methods(Access = public)
        function self = PoseTrajectory()
        end
    end
    
    % Transformations
    methods(Access = public)
        % @ time t, get relative pose between two trajectories/trajectory 
        % GP_Pose 
        function out = AbsoluteToRelativePose(self,poseReference,t,varargin)
            poseAbsolute = self.get('GP_Pose',t);
            % if reference is trajectory, get pose at time t
            if any(strcmp(superclasses(poseReference),'PoseTrajectory'))
                poseReference = poseReference.get('GP_Pose',t);
            end
            % poseReference MUST now be GP_Pose
            assert(strcmp(class(poseReference),'GP_Pose'),'Error: poseReference must be PoseTrajectory or GP_Pose')
            poseRelative = poseAbsolute.AbsoluteToRelativePose(poseReference); 
            % output GP_Pose or one of its properties
            if numel(varargin) > 0
                property = varargin{1};
                out = poseRelative.get(property);
            else
                out = poseRelative;
            end
        end
        
        % @ time t, get absolute pose of trajectory w.r.t. another
        % trajectory or GP_Pose
        function out = RelativeToAbsolutePose(self,poseReference,t,varargin)
            poseRelative = self.get('GP_Pose',t);
            % if reference is trajectory, get pose at time t
            if any(strcmp(superclasses(poseReference),'PoseTrajectory'))
                poseReference = poseReference.get('GP_Pose',t);
            end
            % poseReference MUST now be GP_Pose
            assert(strcmp(class(poseReference),'GP_Pose'),'Error: poseReference must be PoseTrajectory or GP_Pose')
            poseAbsolute = poseRelative.RelativeToAbsolutePose(poseReference); 
            % output GP_Pose or one of its properties
            if numel(varargin) > 0
                property = varargin{1};
                out = poseAbsolute.get(property);
            else
                out = poseAbsolute;
            end
        end
        
        function value = RelativePoseGlobalFrameSE3(self,varargin)
            t1 = varargin{1};
            t2 = varargin{2};
            pose1 = self.get('GP_Pose',t1);
            pose2 = self.get('GP_Pose',t2);
            relativePose = pose2.AbsoluteToRelativePose(pose1);
            value = pose1.RelativePoseGlobalFrameSE3(relativePose);
        end
    end
    
    % Plotting
    methods(Access = public)
        function plot(self,t,varargin)
            % could put for loop to accommodate object arrays - not sure if
            % useful or wise though...
            assert(numel(self)==1,'Error: This function not designed for object arrays')
            
            %store poses for plotting
            poses = zeros(6,numel(t));
            for i = 1:numel(t)
                poses(:,i) = self.get('R3xso3Pose',t(i));
            end
            
            %plot positions
            if size(varargin,2) > 0
                color = varargin{1};
            else
                color = [0 0 0];
            end
            plot3(poses(1,:),poses(2,:),poses(3,:),'color',color,'LineStyle','-')
            
            %plot axes
            for i = 1:numel(t)
                scale = 0.5;
                plotCoordinates(poses(1:3,i),scale*rot(poses(4:6,i)))
            end
        end
    end
    
end

