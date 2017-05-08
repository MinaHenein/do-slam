classdef PoseModelPoseTrajectory < PoseTrajectory
    %PoseModelPoseTrajectory class is not yet fully implemented. It is
    %intended as a PoseTrajectory subclass that is instantiated from 6D
    %pose waypoints. It could be used to estimate the trajectory of dynamic
    %objects from sparse observations
    %   *TODO: to be able to use this class, an orientationModel property
    %   must be set. This should be a 'fit' object which returns a 3D
    %   scaled axis angle vector from a time input. This must be set in the
    %   constructor
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
    end
    
    properties(Hidden)
        xModel 
        yModel
        zModel
        orientationModel
    end
    
    %% 2. Methods    
    % Constructor
    methods(Access = public) %set to private later??
        function self = PoseModelPoseTrajectory(waypoints,parameterisation,fitType)
            assert(strcmp(parameterisation,'R3'),'Error: Only R3 waypoints implemented.')
            self.xModel = fit(waypoints(1,:)',waypoints(2,:)',fitType);
            self.yModel = fit(waypoints(1,:)',waypoints(3,:)',fitType);
            self.zModel = fit(waypoints(1,:)',waypoints(4,:)',fitType);
%             self.orientationModel = 
            error('Error: orientation model not implemented')
        end
        
    end
    
    % Get & Set
    methods(Access = public)
        function value = getSwitch(self,property,varargin)
            switch property
                case 'GP_Pose'
                    t = varargin{1};
                    pose = computePose(self,t);
                    value = pose;
                case {'R3xso3Pose','logSE3Pose','R3xso3Position','logSE3Position','axisAngle','R'}
                    t = varargin{1};
                    pose = computePose(self,t);
                    value = pose.get(property);
                otherwise 
                    error('Error: invalid property')
            end
        end
        
        function self = setSwitch(self,property,value,varargin)
            self.(property) = value;
        end
    end
    
    % Computes pose @ time t
    methods(Access = private)
        function pose = computePose(self,t)
            pose = [self.xModel(t);
                    self.yModel(t);
                    self.zModel(t);
                    self.orientationModel(t)];
            pose = GP_Pose(pose);            
        end
    end
    
end

