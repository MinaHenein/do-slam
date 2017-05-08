classdef PointTrajectory < Trajectory
    %PointTrajectory represents 3D positions
    %   subclasses of PoseTrajectory are:
    %       -PositionModelPointTrajectory
    %       -StaticPointTrajectory
    %   these subclasses are instantiated differently
    %   these subclasses have the same external interface (except setting)
    %   PointTrajectory methods for frame transformations/plotting are
    %   implemented 
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
    end
    
    %% 2. Methods    
    % Constructor
    methods(Access = public)
        function self = PointTrajectory()
        end
    end
    
    % Transformations
    methods(Access = public)
        % @ time t, get relative position between two trajectories/trajectory 
        % GP_Point 
        function out = AbsoluteToRelativePoint(self,poseReference,t,varargin)
            pointAbsolute = self.get('GP_Point',t);
            % if reference is trajectory, get pose at time t
            if any(strcmp(superclasses(poseReference),'PoseTrajectory'))
                poseReference = poseReference.get('GP_Pose',t);
            end
            % poseReference MUST now be GP_Pose
            assert(strcmp(class(poseReference),'GP_Pose'),'Error: poseReference must be PoseTrajectory or GP_Pose')
            pointRelative = pointAbsolute.AbsoluteToRelativePoint(poseReference); 
            % output GP_Point or one of its properties
            if numel(varargin) > 0
                property = varargin{1};
                out = pointRelative.get(property);
            else
                out = pointRelative;
            end
        end
        
        % @ time t, get absolute position of trajectory w.r.t. another
        % trajectory or GP_Point
        function out = RelativeToAbsolutePoint(self,poseReference,t,varargin)
            pointRelative = self.get('GP_Point',t);
            % if reference is trajectory, get pose at time t
            if any(strcmp(superclasses(poseReference),'PoseTrajectory'))
                poseReference = poseReference.get('GP_Pose',t);
            end
            % poseReference MUST now be GP_Pose
            assert(strcmp(class(poseReference),'GP_Pose'),'Error: poseReference must be PoseTrajectory or GP_Pose')
            pointAbsolute = pointRelative.RelativeToAbsolutePoint(poseReference); 
            % output GP_Point or one of its properties
            if numel(varargin) > 0
                property = varargin{1};
                out = pointAbsolute.get(property);
            else
                out = pointAbsolute;
            end
        end
        
    end
    
    % Plotting
    methods(Access = public)
        function plot(self,t,varargin)
            % could put for loop to accommodate object arrays - not sure if
            % useful or wise though...
            assert(numel(self)==1,'Error: This function not designed for object arrays')
            
            %store positions for plotting
            positions = zeros(3,numel(t));
            for i = 1:numel(t)
                positions(:,i) = self.get('R3Position',t(i));
            end
            
            %plot positions
            plot3(positions(1,:),positions(2,:),positions(3,:),'k.')

        end
    end
    
end

