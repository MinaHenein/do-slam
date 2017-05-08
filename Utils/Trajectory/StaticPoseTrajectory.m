classdef StaticPoseTrajectory < PoseTrajectory
    %StaticPoseTrajectory represents a static pose trajectory
    %   This class essentially wraps a GP_Pose object with the methods of
    %   the PoseTrajectory class, allowing it to be treated to same as
    %   dynamic trajectory objects
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
    end
    
    properties(Hidden)
        GP_Pose
    end
    
    %% 2. Methods    
    % Constructor
    methods(Access = public) %set to private later??
        function self = StaticPoseTrajectory(pose,varargin)
            if ~strcmp(class(pose),'GP_Pose')
                pose = GP_Pose(pose,varargin{:});
            end
            self.GP_Pose = pose;
        end
        
    end
    
    % Get & Set
    methods(Access = public)
        function value = getSwitch(self,property,varargin)
            switch property
                case 'GP_Pose'
                    value = self.GP_Pose;
                case {'R3xso3Pose','logSE3Pose','R3xso3Position','logSE3Position','axisAngle','R'}
                    value = self.GP_Pose.get(property);
                otherwise 
                    error('Error: invalid property')
            end
        end
        
        function self = setSwitch(self,property,value,varargin)
            switch property
                case 'GP_Pose'
                    self.GP_Pose = value;
                case {'R3xso3Pose','logSE3Pose','R3xso3Position','logSE3Position','axisAngle','R'}
                    self.GP_Pose.set(property,value);
                otherwise 
                    error('Error: invalid property')
            end
        end
    end
    
    % Plotting
    methods(Access = public)
        % overload PoseTrajectory plot method
        % static trajectory - no unnecessary plotting
        function plot(self,varargin)
            % could put for loop to accommodate object arrays - not sure if
            % useful or wise though...
            assert(numel(self)==1,'Error: This function not designed for object arrays')
            
            %store poses for plotting
            pose = self.GP_Pose.get('R3xso3Pose');
            
            %plot positions
            plot3(pose(1,:),pose(2,:),pose(3,:),'k.')
            
            %plot axes
            scale = 0.5;
            plotCoordinates(pose(1:3),scale*rot(pose(4:6)))
        end
    end
    
end

